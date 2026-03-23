#include "state_space_controller.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pulse_counter.h"
#include "pwm_generator.h"
#include "pid_controller.h"
#include <math.h>

static const char *TAG = "STATE_SPACE_CTRL";

#define SS_LOOP_PERIOD_MS 10
#define MAX_CONTROL_EFFORT 1700.0f

#define CAR_MASS_KG 1.0f
#define PENDULUM_MASS_KG 0.2f
#define PENDULUM_LENGTH_M 0.25f
#define GRAVITY 9.81f
#define PENDULUM_INERTIA (PENDULUM_MASS_KG * PENDULUM_LENGTH_M * PENDULUM_LENGTH_M / 3.0f)

#define ENCODER_PULSES_PER_REV 4096.0f
#define CARRIAGE_MOVEMENT_M_PER_PULSE (2.0f * M_PI * 0.01f / ENCODER_PULSES_PER_REV)
#define OBSERVER_POLE_MULTIPLIER 5.0f

static volatile bool g_ss_enabled = false;
static float g_control_output = 0.0f;
static StateSpaceController g_ss_controller;

static void compute_continuous_system_matrices(void) {
    float M = CAR_MASS_KG;
    float m = PENDULUM_MASS_KG;
    float l = PENDULUM_LENGTH_M;
    float g = GRAVITY;
    float I = PENDULUM_INERTIA;
    
    float denom = I * (M + m) + M * m * l * l;
    
    for (int i = 0; i < NUM_STATES; i++) {
        for (int j = 0; j < NUM_STATES; j++) {
            g_ss_controller.A[i][j] = 0.0f;
        }
    }
    for (int i = 0; i < NUM_STATES; i++) {
        for (int j = 0; j < NUM_INPUTS; j++) {
            g_ss_controller.B[i][j] = 0.0f;
        }
    }
    
    g_ss_controller.A[0][1] = 1.0f;
    g_ss_controller.A[1][2] = -(m * m * l * l * g) / denom;
    g_ss_controller.A[2][3] = 1.0f;
    g_ss_controller.A[3][2] = (m * l * g * (M + m)) / denom;
    
    g_ss_controller.B[1][0] = (I + m * l * l) / denom;
    g_ss_controller.B[3][0] = (-m * l) / denom;
    
    for (int i = 0; i < NUM_OUTPUTS; i++) {
        for (int j = 0; j < NUM_STATES; j++) {
            g_ss_controller.C[i][j] = 0.0f;
        }
    }
    g_ss_controller.C[0][0] = 1.0f;
    g_ss_controller.C[1][2] = 1.0f;
}

static void discretize_matrices(float dt) {
    float A_orig[NUM_STATES][NUM_STATES];
    float B_orig[NUM_STATES][NUM_INPUTS];
    
    for (int i = 0; i < NUM_STATES; i++) {
        for (int j = 0; j < NUM_STATES; j++) {
            A_orig[i][j] = g_ss_controller.A[i][j];
        }
    }
    for (int i = 0; i < NUM_STATES; i++) {
        for (int j = 0; j < NUM_INPUTS; j++) {
            B_orig[i][j] = g_ss_controller.B[i][j];
        }
    }
    
    for (int i = 0; i < NUM_STATES; i++) {
        for (int j = 0; j < NUM_STATES; j++) {
            g_ss_controller.A[i][j] = A_orig[i][j] * dt + (i == j ? 1.0f : 0.0f);
        }
    }
    for (int i = 0; i < NUM_STATES; i++) {
        for (int j = 0; j < NUM_INPUTS; j++) {
            g_ss_controller.B[i][j] = B_orig[i][j] * dt;
        }
    }
}

static void compute_lqr_gain(void) {
    float Q_default[NUM_STATES] = {100.0f, 10.0f, 1000.0f, 100.0f};
    
    for (int i = 0; i < NUM_STATES; i++) {
        for (int j = 0; j < NUM_STATES; j++) {
            g_ss_controller.Q[i][j] = (i == j) ? Q_default[i] : 0.0f;
        }
    }
    g_ss_controller.R[0][0] = 1.0f;
    
    float K_init[NUM_STATES] = {50.0f, 20.0f, 150.0f, 50.0f};
    for (int j = 0; j < NUM_STATES; j++) {
        g_ss_controller.K[0][j] = K_init[j];
    }
    
    ESP_LOGI(TAG, "LQR Gains initialized");
}

static void compute_observer_gain(void) {
    float poles[NUM_STATES];
    poles[0] = -5.0f * OBSERVER_POLE_MULTIPLIER;
    poles[1] = -8.0f * OBSERVER_POLE_MULTIPLIER;
    poles[2] = -12.0f * OBSERVER_POLE_MULTIPLIER;
    poles[3] = -15.0f * OBSERVER_POLE_MULTIPLIER;
    
    g_ss_controller.L[0][0] = 2.0f;
    g_ss_controller.L[0][1] = 0.1f;
    g_ss_controller.L[1][0] = 0.5f;
    g_ss_controller.L[1][1] = 0.05f;
    g_ss_controller.L[2][0] = 0.1f;
    g_ss_controller.L[2][1] = 2.0f;
    g_ss_controller.L[3][0] = 0.05f;
    g_ss_controller.L[3][1] = 0.5f;
    
    ESP_LOGI(TAG, "Observer Gains L configured (poles: %.1f, %.1f, %.1f, %.1f)",
             poles[0], poles[1], poles[2], poles[3]);
}

void SS_Init(StateSpaceController *ss, float dt) {
    ss->dt = dt;
    
    for (int i = 0; i < NUM_STATES; i++) {
        ss->x_hat[i] = 0.0f;
        ss->x_ref[i] = 0.0f;
    }
    ss->x_ref[2] = 0.0f;
    
    ss->u_max = MAX_CONTROL_EFFORT;
    ss->u_min = -MAX_CONTROL_EFFORT;
    
    compute_continuous_system_matrices();
    discretize_matrices(dt);
    compute_lqr_gain();
    compute_observer_gain();
    
    ESP_LOGI(TAG, "State Space Controller with Luenberger Observer initialized");
}

void SS_UpdateObserver(StateSpaceController *ss, float u, float x_meas, float theta_meas) {
    float x_hat_dot[NUM_STATES];
    
    for (int i = 0; i < NUM_STATES; i++) {
        x_hat_dot[i] = 0.0f;
        for (int j = 0; j < NUM_STATES; j++) {
            x_hat_dot[i] += ss->A[i][j] * ss->x_hat[j];
        }
        x_hat_dot[i] += ss->B[i][0] * u;
    }
    
    for (int i = 0; i < NUM_STATES; i++) {
        ss->x_hat[i] += ss->dt * x_hat_dot[i];
    }
    
    float y[NUM_OUTPUTS];
    y[0] = x_meas;
    y[1] = theta_meas;
    
    float y_hat[NUM_OUTPUTS];
    y_hat[0] = ss->x_hat[0];
    y_hat[1] = ss->x_hat[2];
    
    float error[NUM_OUTPUTS];
    error[0] = y[0] - y_hat[0];
    error[1] = y[1] - y_hat[1];
    
    for (int i = 0; i < NUM_STATES; i++) {
        float correction = ss->L[i][0] * error[0] + ss->L[i][1] * error[1];
        ss->x_hat[i] += correction;
    }
}

void SS_Compute(StateSpaceController *ss) {
    float u = 0.0f;
    
    for (int j = 0; j < NUM_STATES; j++) {
        u += ss->K[0][j] * (ss->x_hat[j] - ss->x_ref[j]);
    }
    
    if (u > ss->u_max) u = ss->u_max;
    if (u < ss->u_min) u = ss->u_min;
    
    g_control_output = u;
}

float* SS_GetEstimatedStates(StateSpaceController *ss) {
    return ss->x_hat;
}

float SS_GetControlOutput(StateSpaceController *ss) {
    (void)ss;
    return g_control_output;
}

void SS_Reset(StateSpaceController *ss) {
    for (int i = 0; i < NUM_STATES; i++) {
        ss->x_hat[i] = 0.0f;
    }
    g_control_output = 0.0f;
}

void SS_SetMatrices(StateSpaceController *ss, float Q0, float Q1, float Q2, float Q3, float R0) {
    ss->Q[0][0] = Q0;
    ss->Q[1][1] = Q1;
    ss->Q[2][2] = Q2;
    ss->Q[3][3] = Q3;
    ss->R[0][0] = R0;
    
    compute_lqr_gain();
    ESP_LOGI(TAG, "LQR matrices updated");
}

void SS_UpdateReference(StateSpaceController *ss, float x_ref, float theta_ref) {
    ss->x_ref[0] = x_ref;
    ss->x_ref[2] = theta_ref;
}

void ss_toggle_enable(void) {
    g_ss_enabled = !g_ss_enabled;
    if (g_ss_enabled) {
        SS_Reset(&g_ss_controller);
        ESP_LOGW(TAG, "State Space Control with Observer ENABLED");
    } else {
        motor_command_t stop_cmd = {0, 0, 0};
        xQueueOverwrite(motor_command_queue, &stop_cmd);
        ESP_LOGW(TAG, "State Space Control DISABLED");
    }
}

void ss_force_disable(void) {
    if (g_ss_enabled) {
        g_ss_enabled = false;
        SS_Reset(&g_ss_controller);
        motor_command_t stop_cmd = {0, 0, 0};
        xQueueOverwrite(motor_command_queue, &stop_cmd);
        ESP_LOGE(TAG, "EMERGENCY STOP! State Space Control disabled");
    }
}

bool ss_is_enabled(void) {
    return g_ss_enabled;
}

void state_space_controller_task(void *arg) {
    TickType_t last_wake_time = xTaskGetTickCount();
    float dt = SS_LOOP_PERIOD_MS / 1000.0f;
    
    SS_Init(&g_ss_controller, dt);
    
    ss_toggle_enable();
    ss_toggle_enable();
    
    while (1) {
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(SS_LOOP_PERIOD_MS));
        
        if (!g_ss_enabled) {
            SS_Reset(&g_ss_controller);
            continue;
        }
        
        int16_t current_theta = pulse_counter_get_value();
        int32_t current_x = g_car_position_pulses;
        
        float theta_rad = (float)current_theta * 2.0f * M_PI / ENCODER_PULSES_PER_REV;
        float x_meters = (float)current_x * CARRIAGE_MOVEMENT_M_PER_PULSE;
        
        SS_Compute(&g_ss_controller);
        float u = SS_GetControlOutput(&g_ss_controller);
        
        SS_UpdateObserver(&g_ss_controller, u, x_meters, theta_rad);
        
        float velocity = SS_GetControlOutput(&g_ss_controller);
        
        if (fabsf(velocity) > 0.01f) {
            int direction = (velocity > 0) ? 1 : 0;
            int frequency = 1000 + (int)(fabsf(velocity) * 80);
            if (frequency > 150000) frequency = 150000;
            
            motor_command_t cmd = {0, frequency, direction};
            xQueueOverwrite(motor_command_queue, &cmd);
        } else {
            motor_command_t stop_cmd = {0, 0, 0};
            xQueueOverwrite(motor_command_queue, &stop_cmd);
        }
    }
}
