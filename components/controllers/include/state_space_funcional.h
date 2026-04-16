// components/controllers/include/state_space_funcional.h
#ifndef STATE_SPACE_FUNCIONAL_H
#define STATE_SPACE_FUNCIONAL_H

#include <stdbool.h>

void SS_FUNC_Reset(void);
void SS_FUNC_UpdateReference(float x_ref, float theta_ref);

void ss_func_toggle_enable(void);
void ss_func_enable(void);
void ss_func_disable(void);
void ss_func_force_disable(void);
bool ss_func_is_enabled(void);

float ss_func_get_x_pos(void);
float ss_func_get_x_dot(void);
float ss_func_get_theta(void);
float ss_func_get_theta_dot_hat(void); // En un funcional, suele ser el efecto estimado
float ss_func_get_u_control(void);
float ss_func_get_estado_integrador(void);

void state_space_funcional_task(void *arg);

#endif // STATE_SPACE_FUNCIONAL_H
