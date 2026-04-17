idf_component_register(SRCS "bluetooth_telemetry.c" "uart_echo.c"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos system_core hardware controllers nvs_flash bt esp_driver_uart)
