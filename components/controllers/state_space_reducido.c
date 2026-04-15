#include "state_space_reducido.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pid_controller.h"
#include "pulse_counter.h"
#include "pwm_generator.h"
#include "system_status.h"
#include <math.h>
#include <stdbool.h>

static const char *TAG = "STATE_SPACE_RED";

#define SS_LOOP_PERIOD_MS 10
#define DT (SS_LOOP_PERIOD_MS / 1000.0f)

// ==============================================================================
// 1. CONSTANTES DE CONTROL Y MATRICES (Valores de MATLAB)
// ==============================================================================

typedef struct {
  float L_obs;
  float F_obs;
  float G_obs;
  float H_obs;
  float K_x;
  float K_xdot;
  float K_theta;
  float K_w;
  float K_i;
} LQR_Red_Params;

static const LQR_Red_Params red_params_long = {.L_obs = 50,
                                               15,
                                               .F_obs = 0.500000f,
                                               .G_obs = -24.8f,
                                               .H_obs = -0.056f,
                                               .K_x = -29.90f,
                                               .K_xdot = -22.0110f,
                                               .K_theta = -48.71f,
                                               .K_w = -7.25f,
                                               .K_i = -18.6f};

// PLACEHOLDER: Usando mismos valores para la varilla corta por ahora.
static const LQR_Red_Params red_params_short = {.L_obs = 50.47,
                                                .F_obs = 0.500000f,
                                                .G_obs = -24.3800797f,
                                                .H_obs = -0.17233f,
                                                .K_x = -8.32,
                                                .K_xdot = -6.7f,
                                                .K_theta = -19.9f,
                                                .K_w = -2.76f,
                                                .K_i = -6.0f}; // -4.4

// ==============================================================================
// 2. VARIABLES GLOBALES DE ESTADO
// ==============================================================================

static volatile bool g_ss_red_enabled = false;

static float g_x_pos = 0.0f;
static float g_x_dot = 0.0f; // Velocidad del carro
static float g_theta = 0.0f;

static float g_theta_dot_hat = 0.0f;     // Velocidad angular estimada
static float g_Z_estado = 0.0f;          // Dinámica interna del filtro
static float g_estado_integrador = 0.0f; // Acumulador del error (X_i)
static PIDController g_ss_integrator;    // Utilizado para la integración LQI
static PIDController
    g_ss_accel_integrator; // Integrador de Aceleración a Velocidad
static PIDController g_ss_vel_integrator; // Integrador de Velocidad a Posición

static float g_u_control = 0.0f;
static float g_ref_posicion =
    0.10f; // Inicialmente como en arduino (18cm del origen)

// ==============================================================================
// 3. FUNCIONES DE MANEJO DE ESTADOS Y TAREAS
// ==============================================================================

void SS_RED_Reset(void) {
  g_Z_estado = 0.0f;
  g_estado_integrador = 0.0f;
  // float g_prev_x_pos = pid_get_car_position_m(); // Removed
  PID_Reset(&g_ss_integrator);
  PID_Reset(&g_ss_accel_integrator);
  PID_Reset(&g_ss_vel_integrator);
}

void SS_RED_UpdateReference(float x_ref, float theta_ref) {
  g_ref_posicion = x_ref;
}

void ss_red_toggle_enable(void) {
  g_ss_red_enabled = !g_ss_red_enabled;
  if (g_ss_red_enabled) {
    SS_RED_Reset();
    ESP_LOGW(TAG, "State Space Control REDUCIDO ENABLED");
  } else {
    set_motor_velocity(0.0f);
    ESP_LOGW(TAG, "State Space Control REDUCIDO DISABLED");
  }
}

void ss_red_force_disable(void) {
  if (g_ss_red_enabled) {
    g_ss_red_enabled = false;
    SS_RED_Reset();
    set_motor_velocity(0.0f);
    ESP_LOGE(TAG, "EMERGENCY STOP! (REDUCIDO)");
  }
}

bool ss_red_is_enabled(void) { return g_ss_red_enabled; }

float ss_red_get_x_pos(void) { return g_x_pos; }
float ss_red_get_x_dot(void) { return g_x_dot; }
float ss_red_get_theta(void) { return g_theta; }
float ss_red_get_theta_dot_hat(void) { return g_theta_dot_hat; }
float ss_red_get_u_control(void) { return g_u_control; }
float ss_red_get_estado_integrador(void) { return g_estado_integrador; }

// ==============================================================================
// 4. RUTINA DE INTERRUPCIÓN DE CONTROL (Implementada como FreeRTOS Task)
// ==============================================================================

void state_space_reducido_task(void *arg) {
  (void)arg;
  TickType_t last_wake_time = xTaskGetTickCount();

  ESP_LOGI(TAG, "State Space Controller REDUCIDO initialized");

  // Inicializar integrador del LQI (Kp=0, Ki=1, Kd=0, límites [-2.0, 2.0])
  PID_Init(&g_ss_integrator, 0.0f, 1.0f, 0.0f, DT, -25.0f/181.8, 25.0f/181.8, 0.03f);

  // Inicializar integradores cinemáticos
  PID_Init(&g_ss_accel_integrator, 0.0f, 1.0f, 0.0f, DT, -0.66f, 0.66f, 0.0f);
  PID_Init(&g_ss_vel_integrator, 0.0f, 1.0f, 0.0f, DT, -2.0f, 2.0f, 0.0f);

  ss_red_toggle_enable();
  ss_red_toggle_enable(); // Ciclo para forzar estado inicial limpio
                          // static float pos_control = 0.0f;
  static float vel_control = 0.0f;
  while (1) {
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(SS_LOOP_PERIOD_MS));

    if (!g_ss_red_enabled) {
      SS_RED_Reset();
      continue;
    }

    // --- PASO 1: LEER SENSORES ---
    g_theta = pulse_counter_get_angle_rad() - 3.14159265f;
    g_x_pos = -pid_get_car_position_m();
    // Estimación numérica de la velocidad del carro (x_dot)
    g_x_dot = vel_control;

    // --- PASO 2: DINÁMICA DEL ERROR (Integrador) ---
    g_estado_integrador =
        PID_Compute(&g_ss_integrator, g_ref_posicion, g_x_pos/2.7);

    float dynamic_angle_setpoint = g_estado_integrador;

    // --- PASO 3: ESTIMACIÓN (Observador Reducido Actual) ---

    const LQR_Red_Params *pLQR = (status_get_pendulum_rod() == ROD_LONG)
                                     ? &red_params_long
                                     : &red_params_short;

    g_theta_dot_hat = g_Z_estado + (pLQR->L_obs * g_theta);

    // --- PASO 4: LEY DE CONTROL (LQI) ---
    g_u_control = -2.7*((pLQR->K_x * g_x_pos) + (pLQR->K_xdot * g_x_dot) +
                    (pLQR->K_theta * g_theta) + (pLQR->K_w * g_theta_dot_hat) -
                    (pLQR->K_i * dynamic_angle_setpoint));

    if (g_u_control > 10000.0f)
      g_u_control = 10000.0f;
    if (g_u_control < -10000.0f)
      g_u_control = -10000.0f;

    // --- PASO 5: INTEGRACIONES CINEMÁTICAS ---
    // U es aceleración. Integramos 1 vez para sacar la Velocidad
    vel_control = PID_Compute(&g_ss_accel_integrator, g_u_control, 0.0f);
    // Integramos la velocidad resultante para sacar Posición
    // pos_control = PID_Compute(&g_ss_vel_integrator, vel_control, 0.0f);
    // S(void)pos_control; // silence unused variable warning

    // // --- PASO 6: ¡ACCIONAR MOTOR! ---
    // Puesto que el motor se comanda en Velocidad, enviamos la 1ra integral
    set_motor_velocity(-vel_control);

    // // --- PASO 7: PREDECIR SIGUIENTE ESTADO INTERNO ---
    g_Z_estado = (pLQR->F_obs * g_Z_estado) + (pLQR->G_obs * g_theta) +
                 (pLQR->H_obs * g_u_control);
  }
}
