#include "servo_controller.h"
#include "ble_server.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

static const char *TAG = "SERVO_ARM";

#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES       LEDC_TIMER_14_BIT
#define LEDC_FREQUENCY      50
#define MOTION_TICK_MS      20

static const gpio_num_t SERVO_PINS[SERVO_COUNT] = {
    SERVO_1_PIN,
    SERVO_2_PIN,
    SERVO_3_PIN
};

static const ledc_channel_t SERVO_CHANNELS[SERVO_COUNT] = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
    LEDC_CHANNEL_2
};

// Default programmable action slots: "Name|s1,s2,s3,move_ms,wait_ms;..."
static const char *DEFAULT_ACTIONS[ACTION_NUM_SLOTS] = {
    "Home|90,90,90,500,0",
    "Open|90,60,170,400,0",
    "Grab|90,120,20,400,0",
    "Pick & Place|45,130,160,500,200;45,130,25,400,300;45,60,25,500,200;135,60,25,600,200;135,130,25,500,200;135,130,160,400,200;90,90,90,500,0",
    "Wave|90,45,90,400,100;45,45,150,300,150;135,45,30,300,150;45,45,150,300,150;135,45,30,300,150;90,90,90,400,0",
    "Sweep|10,90,90,600,200;170,90,90,800,200;90,90,90,600,0",
    "Rest|90,160,10,600,0",
    "Nod|90,40,90,350,150;90,110,90,350,150;90,40,90,350,150;90,90,90,400,0"
};

typedef struct {
    int s1;
    int s2;
    int s3;
    int move_ms;
    int wait_ms;
} seq_step_t;

static SemaphoreHandle_t s_mutex = NULL;

static float s_current_f[SERVO_COUNT] = {
    (float)SERVO_DEFAULT_ANGLE,
    (float)SERVO_DEFAULT_ANGLE,
    (float)SERVO_DEFAULT_ANGLE
};
static int s_offsets[SERVO_COUNT] = { 0, 0, 0 };
static char s_actions[ACTION_NUM_SLOTS][ACTION_MAX_LEN];

// Motion / sequence state
static bool s_seq_running = false;
static bool s_seq_loop = false;
static bool s_seq_stop_req = false;
static int  s_active_slot = 0;
static int  s_seq_step_count = 0;
static int  s_seq_step_idx = 0;
static seq_step_t s_seq_steps[ACTION_MAX_STEPS];

typedef enum {
    PHASE_MOVE = 0,
    PHASE_WAIT = 1
} seq_phase_t;

static seq_phase_t s_seq_phase = PHASE_MOVE;
static float s_step_start_f[SERVO_COUNT] = { 90.0f, 90.0f, 90.0f };
static int   s_phase_elapsed_ms = 0;
static int   s_notify_counter = 0;

static int clamp_int(int val, int min_v, int max_v) {
    if (val < min_v) return min_v;
    if (val > max_v) return max_v;
    return val;
}

static float clamp_float(float val, float min_v, float max_v) {
    if (val < min_v) return min_v;
    if (val > max_v) return max_v;
    return val;
}

static uint32_t angle_to_duty(float logical_angle, int offset) {
    float eff = clamp_float(logical_angle + (float)offset, (float)SERVO_MIN_ANGLE, (float)SERVO_MAX_ANGLE);
    float pulse_us = (float)SERVO_MIN_US + (((float)(SERVO_MAX_US - SERVO_MIN_US) * eff) / 180.0f);
    uint32_t duty = (uint32_t)((pulse_us * (float)((1 << 14) - 1)) / 20000.0f);
    return duty;
}

static void apply_hardware_pwm(int idx) {
    if (idx < 0 || idx >= SERVO_COUNT) return;
    uint32_t duty = angle_to_duty(s_current_f[idx], s_offsets[idx]);
    ledc_set_duty(LEDC_MODE, SERVO_CHANNELS[idx], duty);
    ledc_update_duty(LEDC_MODE, SERVO_CHANNELS[idx]);
}

static void apply_all_hardware_pwm(void) {
    for (int i = 0; i < SERVO_COUNT; i++) {
        apply_hardware_pwm(i);
    }
}

static void load_nvs_config(void) {
    nvs_handle_t handle;
    if (nvs_open("arm_cfg", NVS_READONLY, &handle) == ESP_OK) {
        int32_t val = 0;
        if (nvs_get_i32(handle, "cal_o1", &val) == ESP_OK) s_offsets[0] = clamp_int((int)val, SERVO_OFFSET_MIN, SERVO_OFFSET_MAX);
        if (nvs_get_i32(handle, "cal_o2", &val) == ESP_OK) s_offsets[1] = clamp_int((int)val, SERVO_OFFSET_MIN, SERVO_OFFSET_MAX);
        if (nvs_get_i32(handle, "cal_o3", &val) == ESP_OK) s_offsets[2] = clamp_int((int)val, SERVO_OFFSET_MIN, SERVO_OFFSET_MAX);
        nvs_close(handle);
    }

    if (nvs_open("arm_acts", NVS_READWRITE, &handle) == ESP_OK) {
        bool need_commit = false;
        for (int i = 0; i < ACTION_NUM_SLOTS; i++) {
            char key[8];
            snprintf(key, sizeof(key), "act%d", i + 1);
            size_t len = sizeof(s_actions[i]);
            esp_err_t err = nvs_get_str(handle, key, s_actions[i], &len);
            if (err != ESP_OK || len <= 1 || strchr(s_actions[i], '|') == NULL) {
                strncpy(s_actions[i], DEFAULT_ACTIONS[i], ACTION_MAX_LEN - 1);
                s_actions[i][ACTION_MAX_LEN - 1] = '\0';
                nvs_set_str(handle, key, s_actions[i]);
                need_commit = true;
            }
        }
        if (need_commit) nvs_commit(handle);
        nvs_close(handle);
    } else {
        for (int i = 0; i < ACTION_NUM_SLOTS; i++) {
            strncpy(s_actions[i], DEFAULT_ACTIONS[i], ACTION_MAX_LEN - 1);
            s_actions[i][ACTION_MAX_LEN - 1] = '\0';
        }
    }
}

static int parse_steps_internal(const char *steps_str, seq_step_t *out_steps, int max_steps) {
    if (!steps_str || !*steps_str || max_steps <= 0) return 0;

    char buf[ACTION_MAX_LEN];
    strncpy(buf, steps_str, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    int count = 0;
    char *saveptr = NULL;
    char *tok = strtok_r(buf, ";", &saveptr);
    while (tok != NULL && count < max_steps) {
        while (*tok == ' ' || *tok == '\t') tok++;
        if (*tok != '\0') {
            int s1 = 90, s2 = 90, s3 = 90, move_ms = 400, wait_ms = 0;
            int parsed = sscanf(tok, "%d,%d,%d,%d,%d", &s1, &s2, &s3, &move_ms, &wait_ms);
            if (parsed >= 3) {
                out_steps[count].s1 = clamp_int(s1, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
                out_steps[count].s2 = clamp_int(s2, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
                out_steps[count].s3 = clamp_int(s3, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
                out_steps[count].move_ms = clamp_int(move_ms, 0, 10000);
                out_steps[count].wait_ms = clamp_int(wait_ms, 0, 30000);
                count++;
            }
        }
        tok = strtok_r(NULL, ";", &saveptr);
    }
    return count;
}

static void notify_angles_now(int a1, int a2, int a3) {
    if (!ble_server_is_connected()) return;
    char msg[48];
    snprintf(msg, sizeof(msg), "ANG:%d,%d,%d\n", a1, a2, a3);
    ble_server_send_notify(msg);
}

static void notify_action_state(bool running, int slot, bool loop) {
    if (!ble_server_is_connected()) return;
    char msg[64];
    snprintf(msg, sizeof(msg), "ACT_STATE:running=%d,slot=%d,loop=%d\n",
             running ? 1 : 0, slot, loop ? 1 : 0);
    ble_server_send_notify(msg);
}

static void servo_motion_task(void *pvParameters) {
    (void)pvParameters;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MOTION_TICK_MS));

        bool emit_ang = false;
        bool emit_done = false;
        int out_a[SERVO_COUNT] = { 90, 90, 90 };

        xSemaphoreTake(s_mutex, portMAX_DELAY);

        if (s_seq_running) {
            if (s_seq_stop_req || s_seq_step_count <= 0) {
                s_seq_running = false;
                s_seq_loop = false;
                s_seq_stop_req = false;
                s_active_slot = 0;
                emit_done = true;
            } else {
                seq_step_t *st = &s_seq_steps[s_seq_step_idx];
                int targets[SERVO_COUNT] = { st->s1, st->s2, st->s3 };

                if (s_seq_phase == PHASE_MOVE) {
                    s_phase_elapsed_ms += MOTION_TICK_MS;
                    if (st->move_ms <= 0 || s_phase_elapsed_ms >= st->move_ms) {
                        for (int i = 0; i < SERVO_COUNT; i++) {
                            s_current_f[i] = (float)targets[i];
                        }
                        apply_all_hardware_pwm();
                        s_seq_phase = PHASE_WAIT;
                        s_phase_elapsed_ms = 0;
                    } else {
                        float t = (float)s_phase_elapsed_ms / (float)st->move_ms;
                        float ease = (st->move_ms >= 100) ? (0.5f * (1.0f - cosf((float)M_PI * t))) : t;
                        for (int i = 0; i < SERVO_COUNT; i++) {
                            s_current_f[i] = s_step_start_f[i] + (((float)targets[i] - s_step_start_f[i]) * ease);
                        }
                        apply_all_hardware_pwm();
                    }
                } else {
                    s_phase_elapsed_ms += MOTION_TICK_MS;
                    if (s_phase_elapsed_ms >= st->wait_ms) {
                        s_seq_step_idx++;
                        if (s_seq_step_idx >= s_seq_step_count) {
                            if (s_seq_loop) {
                                s_seq_step_idx = 0;
                                for (int i = 0; i < SERVO_COUNT; i++) {
                                    s_step_start_f[i] = s_current_f[i];
                                }
                                s_seq_phase = PHASE_MOVE;
                                s_phase_elapsed_ms = 0;
                            } else {
                                s_seq_running = false;
                                s_active_slot = 0;
                                emit_done = true;
                                emit_ang = true;
                            }
                        } else {
                            for (int i = 0; i < SERVO_COUNT; i++) {
                                s_step_start_f[i] = s_current_f[i];
                            }
                            s_seq_phase = PHASE_MOVE;
                            s_phase_elapsed_ms = 0;
                        }
                    }
                }

                s_notify_counter++;
                if (s_notify_counter >= 5) {
                    s_notify_counter = 0;
                    emit_ang = true;
                }
            }

            for (int i = 0; i < SERVO_COUNT; i++) {
                out_a[i] = (int)lroundf(s_current_f[i]);
            }
        }

        xSemaphoreGive(s_mutex);

        if (emit_ang) {
            notify_angles_now(out_a[0], out_a[1], out_a[2]);
        }
        if (emit_done) {
            notify_action_state(false, 0, false);
        }
    }
}

void servo_controller_init(void) {
    s_mutex = xSemaphoreCreateMutex();
    load_nvs_config();

    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode       = LEDC_MODE;
    ledc_timer.duty_resolution  = LEDC_DUTY_RES;
    ledc_timer.timer_num        = LEDC_TIMER;
    ledc_timer.freq_hz          = LEDC_FREQUENCY;
    ledc_timer.clk_cfg          = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    for (int i = 0; i < SERVO_COUNT; i++) {
        ledc_channel_config_t ledc_channel = {};
        ledc_channel.gpio_num       = SERVO_PINS[i];
        ledc_channel.speed_mode     = LEDC_MODE;
        ledc_channel.channel        = SERVO_CHANNELS[i];
        ledc_channel.intr_type      = LEDC_INTR_DISABLE;
        ledc_channel.timer_sel      = LEDC_TIMER;
        ledc_channel.duty           = angle_to_duty(s_current_f[i], s_offsets[i]);
        ledc_channel.hpoint         = 0;
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }

    xTaskCreatePinnedToCore(servo_motion_task, "servo_motion_task", 4096, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "3-Servo MP90S Controller ready -> S1:GPIO%d, S2:GPIO%d, S3:GPIO%d (Offsets: %d,%d,%d)",
             SERVO_1_PIN, SERVO_2_PIN, SERVO_3_PIN, s_offsets[0], s_offsets[1], s_offsets[2]);
}

void servo_set_angle(int idx, int angle) {
    if (idx < 0 || idx >= SERVO_COUNT) return;
    angle = clamp_int(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

    bool was_running = false;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_seq_running) {
        s_seq_running = false;
        s_seq_loop = false;
        s_seq_stop_req = false;
        s_active_slot = 0;
        was_running = true;
    }
    s_current_f[idx] = (float)angle;
    apply_hardware_pwm(idx);
    xSemaphoreGive(s_mutex);

    if (was_running) {
        notify_action_state(false, 0, false);
    }
}

void servo_set_all(int s1, int s2, int s3) {
    s1 = clamp_int(s1, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    s2 = clamp_int(s2, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    s3 = clamp_int(s3, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

    bool was_running = false;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s_seq_running) {
        s_seq_running = false;
        s_seq_loop = false;
        s_seq_stop_req = false;
        s_active_slot = 0;
        was_running = true;
    }
    s_current_f[0] = (float)s1;
    s_current_f[1] = (float)s2;
    s_current_f[2] = (float)s3;
    apply_all_hardware_pwm();
    xSemaphoreGive(s_mutex);

    if (was_running) {
        notify_action_state(false, 0, false);
    }
}

void servo_move_smooth(int s1, int s2, int s3, int duration_ms) {
    s1 = clamp_int(s1, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    s2 = clamp_int(s2, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    s3 = clamp_int(s3, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    duration_ms = clamp_int(duration_ms, 0, 10000);

    if (duration_ms == 0) {
        servo_set_all(s1, s2, s3);
        notify_angles_now(s1, s2, s3);
        return;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_seq_steps[0].s1 = s1;
    s_seq_steps[0].s2 = s2;
    s_seq_steps[0].s3 = s3;
    s_seq_steps[0].move_ms = duration_ms;
    s_seq_steps[0].wait_ms = 0;
    s_seq_step_count = 1;
    s_seq_step_idx = 0;
    s_seq_loop = false;
    s_seq_stop_req = false;
    s_active_slot = 0;
    s_seq_phase = PHASE_MOVE;
    s_phase_elapsed_ms = 0;
    s_notify_counter = 0;
    for (int i = 0; i < SERVO_COUNT; i++) {
        s_step_start_f[i] = s_current_f[i];
    }
    s_seq_running = true;
    xSemaphoreGive(s_mutex);

    notify_action_state(true, 0, false);
}

int servo_get_angle(int idx) {
    if (idx < 0 || idx >= SERVO_COUNT) return 0;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int a = (int)lroundf(s_current_f[idx]);
    xSemaphoreGive(s_mutex);
    return a;
}

void servo_get_all_angles(int *s1, int *s2, int *s3) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (s1) *s1 = (int)lroundf(s_current_f[0]);
    if (s2) *s2 = (int)lroundf(s_current_f[1]);
    if (s3) *s3 = (int)lroundf(s_current_f[2]);
    xSemaphoreGive(s_mutex);
}

void servo_set_offset(int idx, int offset) {
    if (idx < 0 || idx >= SERVO_COUNT) return;
    offset = clamp_int(offset, SERVO_OFFSET_MIN, SERVO_OFFSET_MAX);
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_offsets[idx] = offset;
    apply_hardware_pwm(idx);
    xSemaphoreGive(s_mutex);
}

int servo_get_offset(int idx) {
    if (idx < 0 || idx >= SERVO_COUNT) return 0;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int o = s_offsets[idx];
    xSemaphoreGive(s_mutex);
    return o;
}

void servo_save_offsets(int o1, int o2, int o3) {
    o1 = clamp_int(o1, SERVO_OFFSET_MIN, SERVO_OFFSET_MAX);
    o2 = clamp_int(o2, SERVO_OFFSET_MIN, SERVO_OFFSET_MAX);
    o3 = clamp_int(o3, SERVO_OFFSET_MIN, SERVO_OFFSET_MAX);

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_offsets[0] = o1;
    s_offsets[1] = o2;
    s_offsets[2] = o3;
    apply_all_hardware_pwm();
    xSemaphoreGive(s_mutex);

    nvs_handle_t handle;
    if (nvs_open("arm_cfg", NVS_READWRITE, &handle) == ESP_OK) {
        nvs_set_i32(handle, "cal_o1", (int32_t)o1);
        nvs_set_i32(handle, "cal_o2", (int32_t)o2);
        nvs_set_i32(handle, "cal_o3", (int32_t)o3);
        nvs_commit(handle);
        nvs_close(handle);
    }
    ESP_LOGI(TAG, "Saved servo trim offsets to NVS: %d, %d, %d", o1, o2, o3);
}

void servo_get_all_offsets(int *o1, int *o2, int *o3) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    if (o1) *o1 = s_offsets[0];
    if (o2) *o2 = s_offsets[1];
    if (o3) *o3 = s_offsets[2];
    xSemaphoreGive(s_mutex);
}

bool servo_action_set(int slot, const char *def_str) {
    if (slot < 1 || slot > ACTION_NUM_SLOTS || def_str == NULL) return false;
    const char *pipe = strchr(def_str, '|');
    if (pipe == NULL || pipe == def_str) return false;

    seq_step_t tmp[ACTION_MAX_STEPS];
    int count = parse_steps_internal(pipe + 1, tmp, ACTION_MAX_STEPS);
    if (count <= 0) return false;

    int idx = slot - 1;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    strncpy(s_actions[idx], def_str, ACTION_MAX_LEN - 1);
    s_actions[idx][ACTION_MAX_LEN - 1] = '\0';
    xSemaphoreGive(s_mutex);

    nvs_handle_t handle;
    if (nvs_open("arm_acts", NVS_READWRITE, &handle) == ESP_OK) {
        char key[8];
        snprintf(key, sizeof(key), "act%d", slot);
        nvs_set_str(handle, key, s_actions[idx]);
        nvs_commit(handle);
        nvs_close(handle);
    }
    ESP_LOGI(TAG, "Action %d saved (%d steps): %s", slot, count, s_actions[idx]);
    return true;
}

const char* servo_action_get(int slot) {
    if (slot < 1 || slot > ACTION_NUM_SLOTS) return "";
    return s_actions[slot - 1];
}

void servo_action_reset_defaults(void) {
    nvs_handle_t handle;
    bool nvs_ok = (nvs_open("arm_acts", NVS_READWRITE, &handle) == ESP_OK);

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    for (int i = 0; i < ACTION_NUM_SLOTS; i++) {
        strncpy(s_actions[i], DEFAULT_ACTIONS[i], ACTION_MAX_LEN - 1);
        s_actions[i][ACTION_MAX_LEN - 1] = '\0';
        if (nvs_ok) {
            char key[8];
            snprintf(key, sizeof(key), "act%d", i + 1);
            nvs_set_str(handle, key, s_actions[i]);
        }
    }
    xSemaphoreGive(s_mutex);

    if (nvs_ok) {
        nvs_commit(handle);
        nvs_close(handle);
    }
    ESP_LOGI(TAG, "Programmable actions reset to factory defaults.");
}

bool servo_action_run(int slot, bool loop) {
    if (slot < 1 || slot > ACTION_NUM_SLOTS) return false;
    const char *def = s_actions[slot - 1];
    const char *pipe = strchr(def, '|');
    if (!pipe) return false;

    seq_step_t parsed_steps[ACTION_MAX_STEPS];
    int count = parse_steps_internal(pipe + 1, parsed_steps, ACTION_MAX_STEPS);
    if (count <= 0) return false;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memcpy(s_seq_steps, parsed_steps, sizeof(seq_step_t) * count);
    s_seq_step_count = count;
    s_seq_step_idx = 0;
    s_seq_loop = loop;
    s_seq_stop_req = false;
    s_active_slot = slot;
    s_seq_phase = PHASE_MOVE;
    s_phase_elapsed_ms = 0;
    s_notify_counter = 0;
    for (int i = 0; i < SERVO_COUNT; i++) {
        s_step_start_f[i] = s_current_f[i];
    }
    s_seq_running = true;
    xSemaphoreGive(s_mutex);

    notify_action_state(true, slot, loop);
    return true;
}

bool servo_run_sequence_str(const char *steps_str, bool loop) {
    seq_step_t parsed_steps[ACTION_MAX_STEPS];
    int count = parse_steps_internal(steps_str, parsed_steps, ACTION_MAX_STEPS);
    if (count <= 0) return false;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memcpy(s_seq_steps, parsed_steps, sizeof(seq_step_t) * count);
    s_seq_step_count = count;
    s_seq_step_idx = 0;
    s_seq_loop = loop;
    s_seq_stop_req = false;
    s_active_slot = 0;
    s_seq_phase = PHASE_MOVE;
    s_phase_elapsed_ms = 0;
    s_notify_counter = 0;
    for (int i = 0; i < SERVO_COUNT; i++) {
        s_step_start_f[i] = s_current_f[i];
    }
    s_seq_running = true;
    xSemaphoreGive(s_mutex);

    notify_action_state(true, 0, loop);
    return true;
}

void servo_stop_motion(void) {
    int a1, a2, a3;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_seq_running = false;
    s_seq_loop = false;
    s_seq_stop_req = false;
    s_active_slot = 0;
    a1 = (int)lroundf(s_current_f[0]);
    a2 = (int)lroundf(s_current_f[1]);
    a3 = (int)lroundf(s_current_f[2]);
    xSemaphoreGive(s_mutex);

    notify_angles_now(a1, a2, a3);
    notify_action_state(false, 0, false);
}

bool servo_is_sequence_running(void) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool r = s_seq_running;
    xSemaphoreGive(s_mutex);
    return r;
}

int servo_get_active_slot(void) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    int s = s_active_slot;
    xSemaphoreGive(s_mutex);
    return s;
}

bool servo_is_sequence_looping(void) {
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool l = s_seq_loop;
    xSemaphoreGive(s_mutex);
    return l;
}