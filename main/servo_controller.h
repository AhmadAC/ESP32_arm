

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SERVO_COUNT         3

#define SERVO_1_PIN         GPIO_NUM_1
#define SERVO_2_PIN         GPIO_NUM_38
#define SERVO_3_PIN         GPIO_NUM_39

// MP90S 50Hz PWM pulse bounds (0 - 180 degrees)
#define SERVO_MIN_US        500
#define SERVO_MAX_US        2500
#define SERVO_MIN_ANGLE     0
#define SERVO_MAX_ANGLE     180
#define SERVO_DEFAULT_ANGLE 90

#define SERVO_OFFSET_MIN    -45
#define SERVO_OFFSET_MAX    45

#define ACTION_NUM_SLOTS    8
#define ACTION_MAX_LEN      384
#define ACTION_MAX_STEPS    24

void servo_controller_init(void);

// Direct & smooth control (idx: 0, 1, 2 for S1, S2, S3)
void servo_set_angle(int idx, int angle);
void servo_set_all(int s1, int s2, int s3);
void servo_move_smooth(int s1, int s2, int s3, int duration_ms);
int  servo_get_angle(int idx);
void servo_get_all_angles(int *s1, int *s2, int *s3);

// Servo power / PWM attach & detach (idle = 0 duty, no self-movement or holding current)
void servo_detach(int idx);
void servo_detach_all(void);
void servo_attach_all(void);
bool servo_is_enabled(int idx);
void servo_get_all_enabled(bool *e1, bool *e2, bool *e3);
void servo_set_auto_idle(bool enabled);
bool servo_get_auto_idle(void);

// Calibration trim offsets (saved to NVS Flash)
void servo_set_offset(int idx, int offset);
int  servo_get_offset(int idx);
void servo_save_offsets(int o1, int o2, int o3);
void servo_get_all_offsets(int *o1, int *o2, int *o3);

// Programmable action slots & sequence execution
bool servo_action_set(int slot, const char *def_str);
const char* servo_action_get(int slot);
void servo_action_reset_defaults(void);
bool servo_action_run(int slot, bool loop);
bool servo_run_sequence_str(const char *steps_str, bool loop);
void servo_stop_motion(void);
bool servo_is_sequence_running(void);
int  servo_get_active_slot(void);
bool servo_is_sequence_looping(void);

#ifdef __cplusplus
}
#endif

