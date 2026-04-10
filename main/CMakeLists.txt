file(GLOB app_sources "*.c" "*.cpp")
idf_component_register(SRCS ${app_sources}
                       INCLUDE_DIRS "../include"
                       REQUIRES driver esp_driver_pcnt nvs_flash esp_timer bt)
