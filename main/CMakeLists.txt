file(GLOB app_sources "*.c" "*.cpp")
idf_component_register(SRCS ${app_sources}
                       INCLUDE_DIRS "../include"
                       REQUIRES driver esp_driver_pcnt esp_driver_gpio esp_driver_ledc esp_driver_i2c esp_driver_uart nvs_flash esp_timer bt)
