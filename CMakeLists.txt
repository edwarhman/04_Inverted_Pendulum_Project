cmake_minimum_required(VERSION 3.16.0)

# Workaround for ESP-IDF 5.5.3 bug: configTICK_RATE_HZ not defined from CONFIG_FREERTOS_HZ
add_compile_definitions(configTICK_RATE_HZ=CONFIG_FREERTOS_HZ)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(04_Inverted_Pendulum_Project)
