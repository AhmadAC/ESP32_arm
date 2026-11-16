#include "joystick.h"
#include "servo_controller.h"
#include "command_processor.h"
#include "console_serial.h"
#include "ble_server.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include <math.h>
#include <stdio.h>

static const char *TAG = "JOYSTICK";

static adc_oneshot_unit_handle_t adc1_handle = NULL;

#define ADC_VRX_CHANNEL     ADC_CHANNEL_1  // GPIO 2
#define ADC_VRY_CHANNEL     ADC_CHANNEL_2  // GPIO 3

#define ADC_EFF_MIN         80.0f
#define ADC_EFF_MAX         4000.0f
#define ADC_MIN_SPAN        250.0f
#define DEADZONE_NORM       0.15f
#define EMA_ALPHA           0.35f
#define SLEW_SPEED_DEG      1.75f          // Max degrees change per 20ms tick (~87 deg/s)
#define CLAW_INDEX          0              // Servo 1 on GPIO 1
#define ARM_INDEX           1              // Servo 2 on GPIO 38
#define SHOULDER_INDEX      2              // Servo 3 on GPIO 39
#define CLAW_CLOSED_ANGLE   0              // Angle when claw is fully closed
#define CLAW_OPEN_ANGLE     130            // Default angle when claw is opened

static bool s_auto_retract_enabled = true;
static bool s_claw_was_deflected   = false;

static float read_adc_avg(adc_channel_t channel, int samples) {
    long sum = 0;
    int valid = 0;
    for (int i = 0; i < samples; i++) {
        int val = 0;
        if (adc_oneshot_read(adc1_handle, channel, &val) == ESP_OK) {
            sum += val;
            valid++;
        }
    }
    if (valid == 0) return 2048.0f;
    return (float)sum / (float)valid;
}

static float normalize_axis(float val, float center) {
    float delta = val - center;
    if (delta >= 0.0f) {
        float span = ADC_EFF_MAX - center;
        if (span < ADC_MIN_SPAN) span = ADC_MIN_SPAN;
        float n = delta / span;
        return (n > 1.0f) ? 1.0f : n;
    } else {
        float span = center - ADC_EFF_MIN;
        if (span < ADC_MIN_SPAN) span = ADC_MIN_SPAN;
        float n = delta / span;
        return (n < -1.0f) ? -1.0f : n;
    }
}

void joystick_set_auto_retract(bool enabled) {
    s_auto_retract_enabled = enabled;
    command_set_auto_retract(enabled);
    ESP_LOGI(TAG, "Claw Auto-Retract set to %s", enabled ? "ENABLED" : "DISABLED");
}

bool joystick_get_auto_retract(void) {
    return s_auto_retract_enabled;
}

void joystick_close_claw(void) {
    servo_set_angle(CLAW_INDEX, CLAW_CLOSED_ANGLE);
    s_claw_was_deflected = false;
    char msg[64];
    snprintf(msg, sizeof(msg), ">> [Joystick SW] Claw Auto-Closed to %d deg\n", CLAW_CLOSED_ANGLE);
    console_print(msg);
    ble_server_send_notify("CLAW:0\n");
    command_notify_status();
}

void joystick_open_claw(int target_angle) {
    if (target_angle < SERVO_MIN_ANGLE) target_angle = SERVO_MIN_ANGLE;
    if (target_angle > SERVO_MAX_ANGLE) target_angle = SERVO_MAX_ANGLE;
    servo_set_angle(CLAW_INDEX, target_angle);
    char msg[64];
    snprintf(msg, sizeof(msg), ">> Claw Opened to %d deg\n", target_angle);
    console_print(msg);
    ble_server_send_notify("CLAW:OPEN\n");
    command_notify_status();
}

static void joystick_poll_task(void *pvParameters) {
    (void)pvParameters;

    // 1. Initial ADC Calibration / Baseline determination
    float accum_x = 0.0f;
    float accum_y = 0.0f;
    const int CAL_SAMPLES = 32;

    for (int i = 0; i < CAL_SAMPLES; i++) {
        accum_x += read_adc_avg(ADC_VRX_CHANNEL, 4);
        accum_y += read_adc_avg(ADC_VRY_CHANNEL, 4);
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    float center_x = accum_x / (float)CAL_SAMPLES;
    float center_y = accum_y / (float)CAL_SAMPLES;
    float filt_x = center_x;
    float filt_y = center_y;

    ESP_LOGI(TAG, "PS2 Joystick Calibrated Center -> X: %.1f, Y: %.1f (VRX->GP38 Arm, VRY->GP39 Shoulder, SW->GP1 Claw)",
             center_x, center_y);

    // Track active target angles as floats for sub-degree smooth slewing
    float arm_f = (float)servo_get_angle(ARM_INDEX);
    float shoulder_f = (float)servo_get_angle(SHOULDER_INDEX);

    // Debounce tracking for SW push button
    uint8_t sw_press_count = 0;
    uint8_t sw_release_count = 0;
    bool sw_down = false;
    const uint8_t DEBOUNCE_TICKS = 2; // ~40ms debounce

    TickType_t last_notify_tick = 0;
    bool pending_status_notify = false;

    while (1) {
        // --- 1. Read & Filter Analog Joystick Axes ---
        float raw_x = read_adc_avg(ADC_VRX_CHANNEL, 6);
        float raw_y = read_adc_avg(ADC_VRY_CHANNEL, 6);

        filt_x += EMA_ALPHA * (raw_x - filt_x);
        filt_y += EMA_ALPHA * (raw_y - filt_y);

        float nx = normalize_axis(filt_x, center_x);
        float ny = normalize_axis(filt_y, center_y);

        float mag = sqrtf((nx * nx) + (ny * ny));
        if (mag > 1.0f) mag = 1.0f;

        bool in_motion = (mag > DEADZONE_NORM);

        // --- 2. Read SW Push Button (Active LOW) ---
        int sw_level = gpio_get_level(JOYSTICK_SW_PIN);
        bool sw_is_pressed = (sw_level == 0);

        if (sw_is_pressed) {
            sw_release_count = 0;
            if (sw_press_count < DEBOUNCE_TICKS) {
                sw_press_count++;
                if (sw_press_count >= DEBOUNCE_TICKS && !sw_down) {
                    sw_down = true;
                    // Click Event: Auto-close the claw (GPIO 1)
                    int cur_claw = servo_get_angle(CLAW_INDEX);
                    if (cur_claw <= 10) {
                        // If already closed, a click opens the claw
                        joystick_open_claw(CLAW_OPEN_ANGLE);
                    } else {
                        // Otherwise, immediately auto-close
                        joystick_close_claw();
                    }
                }
            }
        } else {
            sw_press_count = 0;
            if (sw_down) {
                if (sw_release_count < DEBOUNCE_TICKS) {
                    sw_release_count++;
                    if (sw_release_count >= DEBOUNCE_TICKS) {
                        sw_down = false;
                        sw_release_count = 0;
                    }
                }
            } else {
                sw_release_count = 0;
            }
        }

        // --- 3. Process Motion Control ---
        if (in_motion) {
            float eff_mag = (mag - DEADZONE_NORM) / (1.0f - DEADZONE_NORM);
            if (eff_mag > 1.0f) eff_mag = 1.0f;

            if (sw_down) {
                // If button is held down while moving joystick: open claw proportionally to deflection
                int claw_target = CLAW_CLOSED_ANGLE + (int)lroundf(eff_mag * (float)(CLAW_OPEN_ANGLE - CLAW_CLOSED_ANGLE));
                servo_set_angle(CLAW_INDEX, claw_target);
                s_claw_was_deflected = true;
                pending_status_notify = true;
            } else {
                // Normal mode: VRX controls Arm (GPIO 38) and VRY controls Shoulder (GPIO 39)
                float eff_x = 0.0f;
                float eff_y = 0.0f;

                if (fabsf(nx) > DEADZONE_NORM) {
                    eff_x = (fabsf(nx) - DEADZONE_NORM) / (1.0f - DEADZONE_NORM);
                    if (nx < 0.0f) eff_x = -eff_x;
                }
                if (fabsf(ny) > DEADZONE_NORM) {
                    eff_y = (fabsf(ny) - DEADZONE_NORM) / (1.0f - DEADZONE_NORM);
                    if (ny < 0.0f) eff_y = -eff_y;
                }

                // Slew Arm (Servo 2 / GPIO 38)
                if (fabsf(eff_x) > 0.01f) {
                    arm_f += eff_x * SLEW_SPEED_DEG;
                    if (arm_f < (float)SERVO_MIN_ANGLE) arm_f = (float)SERVO_MIN_ANGLE;
                    if (arm_f > (float)SERVO_MAX_ANGLE) arm_f = (float)SERVO_MAX_ANGLE;
                    servo_set_angle(ARM_INDEX, (int)lroundf(arm_f));
                    pending_status_notify = true;
                }

                // Slew Shoulder (Servo 3 / GPIO 39)
                if (fabsf(eff_y) > 0.01f) {
                    shoulder_f += eff_y * SLEW_SPEED_DEG;
                    if (shoulder_f < (float)SERVO_MIN_ANGLE) shoulder_f = (float)SERVO_MIN_ANGLE;
                    if (shoulder_f > (float)SERVO_MAX_ANGLE) shoulder_f = (float)SERVO_MAX_ANGLE;
                    servo_set_angle(SHOULDER_INDEX, (int)lroundf(shoulder_f));
                    pending_status_notify = true;
                }
            }
        } else {
            // --- 4. Joystick Stopped Moving (Inside Deadzone) -> Claw Auto-Retract ---
            // If Auto-Retract is enabled and the claw was previously opened or deflected
            if (s_auto_retract_enabled && s_claw_was_deflected) {
                s_claw_was_deflected = false;
                servo_set_angle(CLAW_INDEX, CLAW_CLOSED_ANGLE);
                console_print(">> Joystick Centered -> Claw Auto-Retracted to 0 deg\n");
                ble_server_send_notify("AUTORETRACT:RETRACTED\n");
                pending_status_notify = true;
            }

            // Sync internal float tracking with any external BLE/Console angle overrides
            arm_f = (float)servo_get_angle(ARM_INDEX);
            shoulder_f = (float)servo_get_angle(SHOULDER_INDEX);
        }

        // Throttle periodic status notification over BLE / Console during joystick deflection
        if (pending_status_notify && (xTaskGetTickCount() - last_notify_tick) >= pdMS_TO_TICKS(120)) {
            command_notify_status();
            last_notify_tick = xTaskGetTickCount();
            pending_status_notify = false;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void joystick_init(void) {
    // 1. Configure ADC1 for VRX (GPIO 2) and VRY (GPIO 3)
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
#if defined(ADC_ATTEN_DB_12)
        .atten = ADC_ATTEN_DB_12,
#else
        .atten = ADC_ATTEN_DB_11,
#endif
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_VRX_CHANNEL, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_VRY_CHANNEL, &config));

    // 2. Configure SW Click Pin (GPIO 4) as Input with Internal Pull-Up
    gpio_config_t btn_cfg = {};
    btn_cfg.mode = GPIO_MODE_INPUT;
    btn_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    btn_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    btn_cfg.intr_type = GPIO_INTR_DISABLE;
    btn_cfg.pin_bit_mask = (1ULL << JOYSTICK_SW_PIN);
    ESP_ERROR_CHECK(gpio_config(&btn_cfg));

    // 3. Launch background polling task
    xTaskCreatePinnedToCore(joystick_poll_task, "joystick_task", 4096, NULL, 4, NULL, 1);

    ESP_LOGI(TAG, "PS2 Joystick Initialized: VRX=GPIO2 (Arm), VRY=GPIO3 (Shoulder), SW=GPIO4 (Claw Auto-Close)");
}