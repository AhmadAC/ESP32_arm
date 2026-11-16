#include "command_processor.h"
#include "servo_controller.h"
#include "joystick.h"
#include "ble_server.h"
#include "esp_system.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

static const char *TAG = "CMD_PROC";
static bool s_auto_retract = true;

static int parse_servo_index(const char *token) {
    if (!token) return -1;
    if (strcmp(token, "1") == 0 || strcasecmp(token, "s1") == 0 || strcasecmp(token, "claw") == 0) return 0;
    if (strcmp(token, "2") == 0 || strcasecmp(token, "s2") == 0 || strcasecmp(token, "arm") == 0) return 1;
    if (strcmp(token, "3") == 0 || strcasecmp(token, "s3") == 0 || strcasecmp(token, "shoulder") == 0) return 2;
    return -1;
}

void command_set_auto_retract(bool en) {
    s_auto_retract = en;
}

bool command_get_auto_retract(void) {
    return s_auto_retract;
}

void command_processor_init(void) {
    ESP_LOGI(TAG, "Command Processor Initialized (Claw: GPIO 1, Arm: GPIO 38, Shoulder: GPIO 39)");
}

void command_notify_status(void) {
    if (!ble_server_is_connected()) return;

    int s1 = 90, s2 = 90, s3 = 90;
    int o1 = 0, o2 = 0, o3 = 0;
    bool e1 = false, e2 = false, e3 = false;
    servo_get_all_angles(&s1, &s2, &s3);
    servo_get_all_offsets(&o1, &o2, &o3);
    servo_get_all_enabled(&e1, &e2, &e3);
    int brownout = (esp_reset_reason() == ESP_RST_BROWNOUT) ? 1 : 0;

    char status_str[220];
    snprintf(status_str, sizeof(status_str),
             "STATUS:s1=%d,s2=%d,s3=%d,o1=%d,o2=%d,o3=%d,en1=%d,en2=%d,en3=%d,idle=%d,run=%d,act=%d,loop=%d,bo=%d,ar=%d\n",
             s1, s2, s3, o1, o2, o3,
             e1 ? 1 : 0, e2 ? 1 : 0, e3 ? 1 : 0,
             servo_get_auto_idle() ? 1 : 0,
             servo_is_sequence_running() ? 1 : 0,
             servo_get_active_slot(),
             servo_is_sequence_looping() ? 1 : 0,
             brownout,
             s_auto_retract ? 1 : 0);
    ble_server_send_notify(status_str);
}

void command_process(const char *cmd_line, command_print_fn_t print_fn) {
    if (!cmd_line || !print_fn) return;

    const char *p = cmd_line;
    while (*p == ' ' || *p == '\t') p++;
    if (*p == '\0') return;

    // Direct Named Joint Controls: "claw <0-180|open|close>"
    if (strncmp(p, "claw ", 5) == 0) {
        const char *arg = p + 5;
        if (strcasecmp(arg, "close") == 0) {
            joystick_close_claw();
            print_fn(">> Claw auto-closed (0 deg)\n");
            return;
        } else if (strcasecmp(arg, "open") == 0) {
            joystick_open_claw(130);
            print_fn(">> Claw opened (130 deg)\n");
            return;
        } else {
            int angle = atoi(arg);
            if (angle >= SERVO_MIN_ANGLE && angle <= SERVO_MAX_ANGLE) {
                servo_set_angle(0, angle);
                char msg[48];
                snprintf(msg, sizeof(msg), "ANG:%d,%d,%d\n", angle, servo_get_angle(1), servo_get_angle(2));
                print_fn(msg);
                return;
            }
        }
    }

    if (strncmp(p, "arm ", 4) == 0) {
        int angle = atoi(p + 4);
        if (angle >= SERVO_MIN_ANGLE && angle <= SERVO_MAX_ANGLE) {
            servo_set_angle(1, angle);
            char msg[48];
            snprintf(msg, sizeof(msg), "ANG:%d,%d,%d\n", servo_get_angle(0), angle, servo_get_angle(2));
            print_fn(msg);
            return;
        }
    }

    if (strncmp(p, "shoulder ", 9) == 0) {
        int angle = atoi(p + 9);
        if (angle >= SERVO_MIN_ANGLE && angle <= SERVO_MAX_ANGLE) {
            servo_set_angle(2, angle);
            char msg[48];
            snprintf(msg, sizeof(msg), "ANG:%d,%d,%d\n", servo_get_angle(0), servo_get_angle(1), angle);
            print_fn(msg);
            return;
        }
    }

    // Auto-Retract toggle: "autoretract <on|off>"
    if (strncmp(p, "autoretract ", 12) == 0) {
        bool en = (strcasecmp(p + 12, "on") == 0 || strcmp(p + 12, "1") == 0 || strcasecmp(p + 12, "true") == 0);
        joystick_set_auto_retract(en);
        char msg[48];
        snprintf(msg, sizeof(msg), "AUTORETRACT:%d\n", en ? 1 : 0);
        print_fn(msg);
        return;
    }

    // Joystick slew speed: "jspeed [30-500]"
    if (strcmp(p, "jspeed") == 0) {
        char msg[48];
        snprintf(msg, sizeof(msg), "JSPEED:%d\n", joystick_get_speed());
        print_fn(msg);
        return;
    }
    if (strncmp(p, "jspeed ", 7) == 0) {
        int spd = atoi(p + 7);
        joystick_set_speed(spd);
        char msg[64];
        snprintf(msg, sizeof(msg), ">> Joystick speed set to %d deg/s\n", joystick_get_speed());
        print_fn(msg);
        return;
    }

    // Single servo angle commands: "s1 90", "s2 45", "s3 150"
    if ((p[0] == 's' || p[0] == 'S') && (p[1] >= '1' && p[1] <= '3') && (p[2] == ' ' || p[2] == ':')) {
        int idx = p[1] - '1';
        int angle = atoi(p + 3);
        if (angle >= SERVO_MIN_ANGLE && angle <= SERVO_MAX_ANGLE) {
            servo_set_angle(idx, angle);
            return;
        } else {
            char msg[80];
            snprintf(msg, sizeof(msg), ">> Error: Angle must be %d-%d deg.\n", SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
            print_fn(msg);
            return;
        }
    }

    // Set all 3 servos smoothly: "set <s1> <s2> <s3>"
    if (strncmp(p, "set ", 4) == 0) {
        int s1 = 90, s2 = 90, s3 = 90;
        if (sscanf(p + 4, "%d %d %d", &s1, &s2, &s3) == 3) {
            servo_set_all(s1, s2, s3);
            char msg[80];
            snprintf(msg, sizeof(msg), "ANG:%d,%d,%d\n", s1, s2, s3);
            print_fn(msg);
        } else {
            print_fn(">> Usage: set <s1> <s2> <s3>\n");
        }
        return;
    }

    // Smooth move all 3 servos: "move <s1> <s2> <s3> [duration_ms]"
    if (strncmp(p, "move ", 5) == 0) {
        int s1 = 90, s2 = 90, s3 = 90, dur = 400;
        int n = sscanf(p + 5, "%d %d %d %d", &s1, &s2, &s3, &dur);
        if (n >= 3) {
            servo_move_smooth(s1, s2, s3, dur);
            char msg[80];
            snprintf(msg, sizeof(msg), ">> Moving smoothly to (%d, %d, %d) over %dms\n", s1, s2, s3, dur);
            print_fn(msg);
        } else {
            print_fn(">> Usage: move <s1> <s2> <s3> [duration_ms]\n");
        }
        return;
    }

    // Nudge individual servo: "nudge <s1|s2|s3> <+/-deg>"
    if (strncmp(p, "nudge ", 6) == 0) {
        char tok[8] = {0};
        int delta = 0;
        if (sscanf(p + 6, "%7s %d", tok, &delta) == 2) {
            int idx = parse_servo_index(tok);
            if (idx >= 0) {
                int target = servo_get_angle(idx) + delta;
                if (target < SERVO_MIN_ANGLE) target = SERVO_MIN_ANGLE;
                if (target > SERVO_MAX_ANGLE) target = SERVO_MAX_ANGLE;
                servo_set_angle(idx, target);
                int s1, s2, s3;
                servo_get_all_angles(&s1, &s2, &s3);
                char msg[64];
                snprintf(msg, sizeof(msg), "ANG:%d,%d,%d\n", s1, s2, s3);
                print_fn(msg);
                return;
            }
        }
        print_fn(">> Usage: nudge <s1|s2|s3> <+/-deg>\n");
        return;
    }

    // Home / Center all 3 servos to 90 degrees
    if (strcmp(p, "home") == 0 || strcmp(p, "center") == 0) {
        servo_move_smooth(90, 90, 90, 550);
        print_fn(">> Homing all servos to 90 deg...\n");
        return;
    }

    // Stop active sequence/motion
    if (strcmp(p, "stop") == 0) {
        servo_stop_motion();
        print_fn(">> Motion stopped.\n");
        return;
    }

    // Detach / disable servo PWM (stops holding current & buzzing)
    if (strcmp(p, "detach") == 0 || strcmp(p, "idle") == 0 || strcmp(p, "disable") == 0) {
        servo_detach_all();
        print_fn(">> Servos detached (idle, PWM=0).\n");
        return;
    }

    if (strncmp(p, "detach ", 7) == 0) {
        int idx = parse_servo_index(p + 7);
        if (idx >= 0) {
            servo_detach(idx);
            char msg[48];
            snprintf(msg, sizeof(msg), ">> Servo %d detached.\n", idx + 1);
            print_fn(msg);
        } else {
            print_fn(">> Usage: detach [s1|s2|s3]\n");
        }
        return;
    }

    // Re-attach / enable all servos at current angles
    if (strcmp(p, "attach") == 0 || strcmp(p, "enable") == 0) {
        servo_attach_all();
        print_fn(">> Servos attached (PWM enabled).\n");
        return;
    }

    // Auto-Idle toggle: "autoidle on" / "autoidle off"
    if (strncmp(p, "autoidle ", 9) == 0) {
        bool en = (strcasecmp(p + 9, "on") == 0 || strcmp(p + 9, "1") == 0 || strcasecmp(p + 9, "true") == 0);
        servo_set_auto_idle(en);
        char msg[48];
        snprintf(msg, sizeof(msg), "AUTO_IDLE:%d\n", en ? 1 : 0);
        print_fn(msg);
        return;
    }

    // Live calibration trim offset: "offset <s1|s2|s3> <-45..45>"
    if (strncmp(p, "offset ", 7) == 0) {
        char tok[8] = {0};
        int off = 0;
        if (sscanf(p + 7, "%7s %d", tok, &off) == 2) {
            int idx = parse_servo_index(tok);
            if (idx >= 0) {
                servo_set_offset(idx, off);
                return;
            }
        }
        print_fn(">> Usage: offset <s1|s2|s3> <-45..45>\n");
        return;
    }

    // Save all 3 calibration offsets to NVS: "cal_set <o1> <o2> <o3>"
    if (strncmp(p, "cal_set ", 8) == 0) {
        int o1 = 0, o2 = 0, o3 = 0;
        if (sscanf(p + 8, "%d %d %d", &o1, &o2, &o3) == 3) {
            servo_save_offsets(o1, o2, o3);
            servo_get_all_offsets(&o1, &o2, &o3);
            char msg[64];
            snprintf(msg, sizeof(msg), "CAL:%d,%d,%d\n", o1, o2, o3);
            print_fn(msg);
            command_notify_status();
        } else {
            print_fn(">> Usage: cal_set <o1> <o2> <o3>\n");
        }
        return;
    }

    // Query calibration offsets
    if (strcmp(p, "cal_get") == 0) {
        int o1 = 0, o2 = 0, o3 = 0;
        servo_get_all_offsets(&o1, &o2, &o3);
        char msg[64];
        snprintf(msg, sizeof(msg), "CAL:%d,%d,%d\n", o1, o2, o3);
        print_fn(msg);
        return;
    }

    // Run programmable action slot: "action_run <1-8>" or "run <1-8>"
    if (strncmp(p, "action_run ", 11) == 0 || strncmp(p, "run ", 4) == 0) {
        int slot = atoi(strncmp(p, "run ", 4) == 0 ? (p + 4) : (p + 11));
        if (servo_action_run(slot, false)) {
            char msg[64];
            snprintf(msg, sizeof(msg), ">> Running programmable action %d\n", slot);
            print_fn(msg);
        } else {
            print_fn(">> Error: Invalid action slot (1-8).\n");
        }
        return;
    }

    // Loop programmable action slot: "action_loop <1-8>" or "loop <1-8>"
    if (strncmp(p, "action_loop ", 12) == 0 || strncmp(p, "loop ", 5) == 0) {
        int slot = atoi(strncmp(p, "loop ", 5) == 0 ? (p + 5) : (p + 12));
        if (servo_action_run(slot, true)) {
            char msg[64];
            snprintf(msg, sizeof(msg), ">> Looping programmable action %d\n", slot);
            print_fn(msg);
        } else {
            print_fn(">> Error: Invalid action slot (1-8).\n");
        }
        return;
    }

    // Save programmable action slot to NVS: "action_set <1-8> <name>|<s1,s2,s3,move,wait;...>"
    if (strncmp(p, "action_set ", 11) == 0) {
        const char *arg = p + 11;
        while (*arg == ' ') arg++;
        char *endptr = NULL;
        long slot = strtol(arg, &endptr, 10);
        if (slot >= 1 && slot <= ACTION_NUM_SLOTS && endptr != NULL) {
            while (*endptr == ' ' || *endptr == '\t') endptr++;
            if (servo_action_set((int)slot, endptr)) {
                char resp[ACTION_MAX_LEN + 16];
                snprintf(resp, sizeof(resp), "ACT:%ld=%s\n", slot, servo_action_get((int)slot));
                print_fn(resp);
                return;
            }
        }
        print_fn(">> Usage: action_set <1-8> <name>|<s1,s2,s3,move_ms,wait_ms;...>\n");
        return;
    }

    // Query all 8 programmable action slots
    if (strcmp(p, "action_get") == 0) {
        for (int i = 1; i <= ACTION_NUM_SLOTS; i++) {
            char line[ACTION_MAX_LEN + 16];
            snprintf(line, sizeof(line), "ACT:%d=%s\n", i, servo_action_get(i));
            print_fn(line);
            vTaskDelay(pdMS_TO_TICKS(25));
        }
        return;
    }

    // Reset all programmable actions to factory defaults
    if (strcmp(p, "action_reset") == 0) {
        servo_action_reset_defaults();
        for (int i = 1; i <= ACTION_NUM_SLOTS; i++) {
            char line[ACTION_MAX_LEN + 16];
            snprintf(line, sizeof(line), "ACT:%d=%s\n", i, servo_action_get(i));
            print_fn(line);
            vTaskDelay(pdMS_TO_TICKS(25));
        }
        print_fn(">> Programmable actions restored to defaults.\n");
        return;
    }

    // Run ad-hoc sequence: "seq <s1,s2,s3,move,wait;...>" or "seq_loop <...>"
    if (strncmp(p, "seq_loop ", 9) == 0) {
        if (servo_run_sequence_str(p + 9, true)) {
            print_fn(">> Looping custom sequence...\n");
        } else {
            print_fn(">> Error: Invalid sequence steps.\n");
        }
        return;
    }
    if (strncmp(p, "seq ", 4) == 0) {
        if (servo_run_sequence_str(p + 4, false)) {
            print_fn(">> Running custom sequence...\n");
        } else {
            print_fn(">> Error: Invalid sequence steps.\n");
        }
        return;
    }

    if (strcmp(p, "status") == 0) {
        int s1 = 90, s2 = 90, s3 = 90;
        int o1 = 0, o2 = 0, o3 = 0;
        bool e1 = false, e2 = false, e3 = false;
        servo_get_all_angles(&s1, &s2, &s3);
        servo_get_all_offsets(&o1, &o2, &o3);
        servo_get_all_enabled(&e1, &e2, &e3);
        int brownout = (esp_reset_reason() == ESP_RST_BROWNOUT) ? 1 : 0;
        char msg[220];
        snprintf(msg, sizeof(msg),
                 "STATUS:s1=%d,s2=%d,s3=%d,o1=%d,o2=%d,o3=%d,en1=%d,en2=%d,en3=%d,idle=%d,run=%d,act=%d,loop=%d,bo=%d,ar=%d\n",
                 s1, s2, s3, o1, o2, o3,
                 e1 ? 1 : 0, e2 ? 1 : 0, e3 ? 1 : 0,
                 servo_get_auto_idle() ? 1 : 0,
                 servo_is_sequence_running() ? 1 : 0,
                 servo_get_active_slot(),
                 servo_is_sequence_looping() ? 1 : 0,
                 brownout,
                 s_auto_retract ? 1 : 0);
        print_fn(msg);
        return;
    }

    if (strcmp(p, "help") == 0 || strcmp(p, "?") == 0) {
        print_fn("3-Servo Arm (Claw=GP1, Arm=GP38, Shoulder=GP39) + Joystick:\n"
                 "  claw <0-180|open|close>              : Direct Claw command\n"
                 "  arm <0-180> / shoulder <0-180>       : Move Arm or Shoulder\n"
                 "  autoretract <on|off>                 : Toggle Claw Auto-Retract\n"
                 "  jspeed [30-500]                      : Get/Set Joystick speed (deg/s)\n"
                 "  s1 <deg> / s2 <deg> / s3 <deg>       : Move specific servo\n"
                 "  set <s1> <s2> <s3>                   : Move all 3 servos\n"
                 "  move <s1> <s2> <s3> [ms]             : Smooth ease all 3\n"
                 "  home / stop / detach / attach        : Quick control\n"
                 "  action_run <1-8> / action_loop <1-8> : Saved routine\n"
                 "  status                               : Print full system state\n");
        return;
    }

    if (strcmp(p, "reboot") == 0 || strcmp(p, "restart") == 0) {
        print_fn("Rebooting ESP32-S3...\n");
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_restart();
        return;
    }

    char err_msg[96];
    snprintf(err_msg, sizeof(err_msg), ">> Unknown command: '%s'. Enter 'help'.\n", p);
    print_fn(err_msg);
}

void command_on_ble_received(const char *cmd, void (*reply_fn)(const char *reply)) {
    ESP_LOGI(TAG, "BLE Command: '%s'", cmd);
    command_process(cmd, reply_fn);
}