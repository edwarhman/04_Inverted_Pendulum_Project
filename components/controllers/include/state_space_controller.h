#ifndef STATE_SPACE_CONTROLLER_H
#define STATE_SPACE_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

void SS_Reset(void);
void SS_UpdateReference(float x_ref, float theta_ref);

void state_space_controller_task(void *arg);
void ss_toggle_enable(void);
void ss_enable(void);
void ss_disable(void);
void ss_force_disable(void);
bool ss_is_enabled(void);

float ss_get_x_pos(void);
float ss_get_x_dot(void);
float ss_get_theta(void);
float ss_get_theta_dot_hat(void);
float ss_get_u_control(void);
float ss_get_estado_integrador(void);

#endif
