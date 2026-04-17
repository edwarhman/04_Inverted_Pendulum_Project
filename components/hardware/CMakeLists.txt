idf_component_register(SRCS "pulse_counter.c" "pwm_generator.c" "button_handler.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_timer driver freertos display system_core controllers esp_driver_ledc esp_driver_pcnt esp_driver_gpio)
