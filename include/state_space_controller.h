#ifndef STATE_SPACE_CONTROLLER_H
#define STATE_SPACE_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define NUM_STATES 4
#define NUM_INPUTS 1
#define NUM_OUTPUTS 2

typedef struct {
    float A[NUM_STATES][NUM_STATES];
    float B[NUM_STATES][NUM_INPUTS];
    float C[NUM_OUTPUTS][NUM_STATES];
    float K[NUM_INPUTS][NUM_STATES];
    float L[NUM_STATES][NUM_OUTPUTS];
    float Q[NUM_STATES][NUM_STATES];
    float R[NUM_INPUTS][NUM_INPUTS];
    float x_hat[NUM_STATES];
    float x_ref[NUM_STATES];
    float u_max;
    float u_min;
    float dt;
} StateSpaceController;

void SS_Init(StateSpaceController *ss, float dt);
void SS_UpdateObserver(StateSpaceController *ss, float u, float x_meas, float theta_meas);
void SS_Compute(StateSpaceController *ss);
float* SS_GetEstimatedStates(StateSpaceController *ss);
float SS_GetControlOutput(StateSpaceController *ss);
void SS_Reset(StateSpaceController *ss);
void SS_SetMatrices(StateSpaceController *ss, float Q0, float Q1, float Q2, float Q3, float R0);
void SS_UpdateReference(StateSpaceController *ss, float x_ref, float theta_ref);

void state_space_controller_task(void *arg);
void ss_toggle_enable(void);
void ss_force_disable(void);
bool ss_is_enabled(void);

#endif
