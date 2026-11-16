#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// Hardware pin mapping for PS2 Joystick on YD-ESP32-S3:
// Servo assignments:
//   Claw     -> GPIO 1  (Servo 1)
//   Arm      -> GPIO 38 (Servo 2)
//   Shoulder -> GPIO 39 (Servo 3)
//
// Joystick module connections:
//   GND -> GND
//   +5V -> 3.3V (or 5V)
//   VRX -> GPIO 2 (ADC1 Channel 1) -> Controls Arm
//   VRY -> GPIO 3 (ADC1 Channel 2) -> Controls Shoulder
//   SW  -> GPIO 4 (Digital Input with internal pull-up) -> Auto-Close Claw

#define JOYSTICK_ADC_VRX_PIN    GPIO_NUM_2   // ADC1 Channel 1
#define JOYSTICK_ADC_VRY_PIN    GPIO_NUM_3   // ADC1 Channel 2
#define JOYSTICK_SW_PIN         GPIO_NUM_4   // Push button (Active LOW)

#define JOYSTICK_MIN_SPEED_DPS      30
#define JOYSTICK_MAX_SPEED_DPS      500
#define JOYSTICK_DEFAULT_SPEED_DPS  260

void joystick_init(void);
void joystick_set_auto_retract(bool enabled);
bool joystick_get_auto_retract(void);
void joystick_set_speed(int deg_per_sec);
int  joystick_get_speed(void);
void joystick_set_invert(bool x_inv, bool y_inv);
void joystick_get_invert(bool *x_inv, bool *y_inv);
void joystick_close_claw(void);
void joystick_open_claw(int target_angle);

#ifdef __cplusplus
}
#endif