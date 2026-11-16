
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "servo_controller.h"
#include "ble_server.h"
#include "console_serial.h"
#include "command_processor.h"

static const char *TAG = "APP_MAIN";

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Booting YD-ESP32-S3 3-Servo MP90S Arm Controller...");

    // 1. Flash Storage (NVS) for calibration offsets & programmable actions
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Command Processor Subsystem
    command_processor_init();

    // 3. 3-Servo LEDC PWM Controller (GPIO 1, GPIO 38, GPIO 39)
    servo_controller_init();

    // 4. Bluetooth Low Energy NUS Server ('ESP32S3-Arm')
    ble_server_init(command_on_ble_received);

    // 5. Serial Console (UART0 & Native USB-Serial-JTAG REPL)
    console_serial_init();

    ESP_LOGI(TAG, "All subsystems running. Ready for BLE & Serial commands.");
}