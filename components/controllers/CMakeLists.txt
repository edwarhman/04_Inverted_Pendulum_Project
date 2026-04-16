idf_component_register(SRCS "state_space_funcional.c" "state_space_reducido.c" "pid_controller.c" "state_space_controller.c" "swing_up_controller.c"
                       INCLUDE_DIRS "include"
                       REQUIRES freertos hardware system_core)
