// components/controllers/state_space_funcional.c
//
// Controlador LQI con Observador Funcional Lineal (1er orden).
// Estima directamente la combinación de estados para la ley de control.
//

#include "state_space_funcional.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pid_controller.h"
#include "pulse_counter.h"
#include "pwm_generator.h"
#include "system_status.h"

static const char *TAG = "SS_FUNC_CTRL";

#define SS_LOOP_PERIOD_MS 10
#define DT (SS_LOOP_PERIOD_MS / 1000.0f)
#define VEL_CMD_LIMIT 7.5f

// =============================================================================
// 1. PARÁMETROS DEL OBSERVADOR FUNCIONAL LINEAL (LFO)
// =============================================================================

typedef struct {
    // Coeficientes del Observador Funcional Lineal
    // Reconstruction: u_red = z + b1*x + b2*x_dot + b3*theta + b4*xi
    // Update: z[k+1] = F*z[k] + G1*x + G2*x_dot + G3*theta + G4*xi + H*u
    float F_func;
    float b[4]; // [x, x_dot, theta, xi]
    float G[4]; // [x, x_dot, theta, xi]
    float H_func;
    
    // Parámetros de referencia y límites
    float K_i; // No se usa directamente si b4/G4 ya integran xi, pero lo mantenemos por consistencia
} FUNC_Params;

/**
 * Diseño proporcionado por el usuario:
 */
static const FUNC_Params params_long = {
    .F_func = 0.5000f,
    .b = {-6.09f, -4.4684f, -412.62f, 3.8184f},
    .G = {-12.0077f, -8.9368f, 179.6182f, 7.3966f},
    .H_func = 0.2213f,
    .K_i = 1.0f // El peso de la integral ya está en b[3] y G[3]
};

// Placeholder para vara corta (duplicado temporal)
static const FUNC_Params params_short = {
    .F_func = 0.5000f,
    .b = {-5.0932f, -2.8f, -414.9038f, 3.9184f},
    .G = {-9.8997f, -5.6044f, 178.6182f, 7.6266f},
    .H_func = 0.1970f,
    .K_i = 1.0f // El peso de la integral ya está en b[3] y G[3]
};

// =============================================================================
// 2. VARIABLES GLOBALES
// =============================================================================

static volatile bool g_ss_func_enabled = false;
static float g_z_state = 0.0f; // Memoria del observador funcional (Z_estado)

static float g_x_pos = 0.0f;
static float g_x_dot = 0.0f;
static float g_theta = 0.0f;
static float g_u_control = 0.0f;
static float g_estado_integrador = 0.0f;
static float g_vel_cmd = 0.0f;

#define LIMITE_INTEGRAL 50.0f
#define ACEL_MAX 20.0f
#define VEL_MAX 1.10f

// =============================================================================
// 3. GESTIÓN DE ESTADO
// =============================================================================

void SS_FUNC_Reset(void) {
    g_z_state = 0.0f;
    g_u_control = 0.0f;
    g_vel_cmd = 0.0f;
    g_estado_integrador = 0.0f;
}

void SS_FUNC_UpdateReference(float x_ref, float theta_ref) {
    (void)theta_ref;
    status_set_ref_position(x_ref);
}

void ss_func_enable(void) {
    if (!g_ss_func_enabled) {
        g_ss_func_enabled = true;
        SS_FUNC_Reset();
        ESP_LOGW(TAG, "Linear Functional Observer ENABLED");
    }
}

void ss_func_disable(void) {
    if (g_ss_func_enabled) {
        g_ss_func_enabled = false;
        set_motor_velocity(0.0f);
        ESP_LOGW(TAG, "Linear Functional Observer DISABLED");
    }
}

void ss_func_toggle_enable(void) {
    if (g_ss_func_enabled) {
        ss_func_disable();
    } else {
        ss_func_enable();
    }
}

void ss_func_force_disable(void) {
    if (g_ss_func_enabled) {
        g_ss_func_enabled = false;
        SS_FUNC_Reset();
        set_motor_velocity(0.0f);
        ESP_LOGE(TAG, "EMERGENCY STOP — SS Functional disabled");
    }
}

bool ss_func_is_enabled(void) { return g_ss_func_enabled; }
float ss_func_get_x_pos(void) { return g_x_pos; }
float ss_func_get_x_dot(void) { return g_x_dot; }
float ss_func_get_theta(void) { return g_theta; }
float ss_func_get_theta_dot_hat(void) { return 0.0f; }
float ss_func_get_u_control(void) { return g_u_control; }
float ss_func_get_estado_integrador(void) { return g_estado_integrador; }

// =============================================================================
// 4. TAREA DE CONTROL
// =============================================================================

void state_space_funcional_task(void *arg) {
    (void)arg;
    TickType_t last_wake_time = xTaskGetTickCount();

    ESP_LOGI(TAG, "LFO User-defined task ready (Ref=0.18m, Ts=10ms)");

    while (1) {
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(SS_LOOP_PERIOD_MS));

        if (!g_ss_func_enabled) {
            SS_FUNC_Reset();
            continue;
        }

        const FUNC_Params *p = (status_get_pendulum_rod() == ROD_LONG) ? &params_long : &params_short;

        // --- PASO 1: LEER SENSORES FÍSICOS ---
        // θ = 0 en vertical arriba. pulse_counter_get_angle_rad() entrega 0 abajo (PI rad en vertical arriba)
        g_theta = pulse_counter_get_angle_rad() - (float)M_PI; 
        g_x_pos = pid_get_car_position_m();
        g_x_dot = g_vel_cmd; // Velocidad integrada (cinemática de motor de pasos)
        
        // --- PASO 2: SENSOR VIRTUAL (EL INTEGRADOR) ---
        float error_pos = status_get_ref_position() - g_x_pos;
        g_estado_integrador += error_pos * DT;
        
        if(g_estado_integrador > LIMITE_INTEGRAL) g_estado_integrador = LIMITE_INTEGRAL;
        if(g_estado_integrador < -LIMITE_INTEGRAL) g_estado_integrador = -LIMITE_INTEGRAL;

        // --- PASO 3: LEY DE CONTROL DIRECTA (RECONSTRUCCIÓN) ---
        // u = - (Z + b1*x + b2*ẋ + b3*θ + b4*xi)
        float reconstruccion_KX = g_z_state 
                                + (p->b[0] * g_x_pos) 
                                + (p->b[1] * g_x_dot) 
                                + (p->b[2] * g_theta) 
                                + (p->b[3] * g_estado_integrador);
        g_u_control = -reconstruccion_KX;

        // --- PASO 4: SATURACIÓN Y CINEMÁTICA DEL MOTOR ---
        if(g_u_control > ACEL_MAX) g_u_control = ACEL_MAX;
        if(g_u_control < -ACEL_MAX) g_u_control = -ACEL_MAX;

        // Integración de aceleración (u) para obtener velocidad (v_stepper)
        g_vel_cmd += g_u_control * DT;

        if(g_vel_cmd > VEL_MAX) g_vel_cmd = VEL_MAX;
        if(g_vel_cmd < -VEL_MAX) g_vel_cmd = -VEL_MAX;

        set_motor_velocity(g_vel_cmd);

        // --- PASO 5: ACTUALIZAR EL FILTRO (Z_k+1) ---
        // Z_next = F*Z + G1*x + G2*ẋ + G3*θ + G4*xi + H*u
        g_z_state = (p->F_func * g_z_state) 
                  + (p->G[0] * g_x_pos) 
                  + (p->G[1] * g_x_dot) 
                  + (p->G[2] * g_theta) 
                  + (p->G[3] * g_estado_integrador) 
                  + (p->H_func * g_u_control);
    }
}
