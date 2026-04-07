#ifndef STATE_SPACE_CONTROLLER_H
#define STATE_SPACE_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

#define NUM_STATES 4
#define NUM_INPUTS 1
#define NUM_OUTPUTS 2

#define ENCODER_PULSES_PER_REV 4096.0f
#define CARRIAGE_MOVEMENT_M_PER_PULSE (2.0f * M_PI * 0.01f / ENCODER_PULSES_PER_REV)

void SS_UpdateObserver(float u, float x_meas, float theta_meas);
void SS_Compute(void);
float* SS_GetEstimatedStates(void);
float SS_GetControlOutput(void);
void SS_Reset(void);
void SS_UpdateReference(float x_ref, float theta_ref);

void state_space_controller_task(void *arg);
void ss_toggle_enable(void);
void ss_force_disable(void);
bool ss_is_enabled(void);

#endif
