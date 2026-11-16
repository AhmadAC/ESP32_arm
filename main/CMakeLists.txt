idf_component_register(
    SRCS
        "main.cpp"
        "servo_controller.cpp"
        "command_processor.cpp"
        "ble_server.cpp"
        "console_serial.cpp"
    INCLUDE_DIRS "."
    REQUIRES nvs_flash driver esp_timer bt
)