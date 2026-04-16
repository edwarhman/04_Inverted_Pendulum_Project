// components/controllers/include/state_space_reducido.h
#ifndef STATE_SPACE_REDUCIDO_H
#define STATE_SPACE_REDUCIDO_H

#include <stdbool.h>

void SS_RED_Reset(void);
void SS_RED_UpdateReference(float x_ref, float theta_ref);

void ss_red_toggle_enable(void);
void ss_red_enable(void);
void ss_red_disable(void);
void ss_red_force_disable(void);
bool ss_red_is_enabled(void);

float ss_red_get_x_pos(void);
float ss_red_get_x_dot(void);
float ss_red_get_theta(void);
float ss_red_get_theta_dot_hat(void);
float ss_red_get_u_control(void);
float ss_red_get_estado_integrador(void);

void state_space_reducido_task(void *arg);

#endif // STATE_SPACE_REDUCIDO_H
