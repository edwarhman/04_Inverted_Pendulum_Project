// components/controllers/state_space_reducido.c
//
// Controlador LQI con Observador de Orden Reducido (1er orden).
// Estima únicamente la velocidad angular (theta_dot) a partir del ángulo
// (theta).
//

#include "state_space_reducido.h"
#include "state_space_controller.h" // Para algunas utilidades si fuera necesario

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

static const char *TAG = "SS_RED_CTRL";

#define SS_LOOP_PERIOD_MS 10
#define DT (SS_LOOP_PERIOD_MS / 1000.0f)

// Límite de velocidad del carro (m/s)
#define VEL_CMD_LIMIT 1.1f
#define ACEL_MAX 20.0f

// =============================================================================
// 1. PARÁMETROS DEL SISTEMA Y OBSERVADOR REDUCIDO
// =============================================================================

typedef struct {
  // Coeficientes del Observador Reducido: z[k] = F*z[k-1] + G*u[k-1] +
  // H*theta[k-1] theta_dot_hat = z + L_obs*theta
  float F_obs;
  float G_obs;
  float H_obs;
  float L_obs;

  // Ganancias LQI: u = -(K_x*x + K_xdot*x_dot + K_theta*theta + K_w*theta_dot -
  // K_i*x_i)
  float K_x;
  float K_xdot;
  float K_theta;
  float K_w;
  float K_i;
} RED_Params;

/**
 * Cálculos para L = 40 (Polo en z = 0.6018):
 * F = Phi22 - L*Phi12 = 1.0018 - 40*0.01 = 0.6018
 * G = Gamma2 - L*Gamma1 = -0.075 - 40*(-0.000375) = -0.06
 * H = F*L + Phi21 - L*Phi11 = -15.6319
 */
static const RED_Params params_long = {.F_obs = 0.55f,
                                       .H_obs = -0.0560f,
                                       .G_obs = -15.6319f,
                                       .L_obs = 50.0f,
                                       .K_x = -8.0f,
                                       .K_xdot = -11.6f,
                                       .K_theta = -48.7f,
                                       .K_w = -7.25f,
                                       .K_i = -6.6f};

// Placeholder: Duplicado para la vara corta (ajustar tras cálculo en MATLAB)
static const RED_Params params_short = {.F_obs = 0.6018f,
                                        .H_obs = -0.0600f,
                                        .G_obs = -15.6319f,
                                        .L_obs = 40.0f,
                                        .K_x = -8.3f,
                                        .K_xdot = -11.27f,
                                        .K_theta = -40.28f,
                                        .K_w = -5.6f,
                                        .K_i = -6.0f};
;

// =============================================================================
// 2. VARIABLES GLOBALES DE ESTADO
// =============================================================================

static volatile bool g_ss_red_enabled = false;

// Estado del Observador Reducido
static float g_z_obs = 0.0f;

// Variables expuestas
static float g_x_pos = 0.0f;
static float g_x_dot = 0.0f;
static float g_theta = 0.0f;
static float g_theta_dot_hat = 0.0f;
static float g_u_control = 0.0f;
static float g_estado_integrador = 0.0f;

// Variables internas
static float g_u_prev = 0.0f;
static float g_theta_prev = 0.0f;
static float g_vel_cmd = 0.0f;
static float g_ref_posicion = 0.1f;

static PIDController g_ss_red_integrator;

// =============================================================================
// 3. FUNCIONES DE GESTIÓN
// =============================================================================

void SS_RED_Reset(void) {
  g_z_obs = 0.0f;
  g_u_prev = 0.0f;
  g_theta_prev = pulse_counter_get_angle_rad() - (float)M_PI;
  g_vel_cmd = 0.0f;
  g_estado_integrador = 0.0f;
  PID_Reset(&g_ss_red_integrator);
}

void SS_RED_UpdateReference(float x_ref, float theta_ref) {
  (void)theta_ref;
  g_ref_posicion = x_ref;
}

void ss_red_toggle_enable(void) {
  g_ss_red_enabled = !g_ss_red_enabled;
  if (g_ss_red_enabled) {
    SS_RED_Reset();
    ESP_LOGW(TAG, "State Space REDUCED ENABLED");
  } else {
    set_motor_velocity(0.0f);
    ESP_LOGW(TAG, "State Space REDUCED DISABLED");
  }
}

void ss_red_force_disable(void) {
  if (g_ss_red_enabled) {
    g_ss_red_enabled = false;
    SS_RED_Reset();
    set_motor_velocity(0.0f);
    ESP_LOGE(TAG, "EMERGENCY STOP — SS Reduced disabled");
  }
}

bool ss_red_is_enabled(void) { return g_ss_red_enabled; }
float ss_red_get_x_pos(void) { return g_x_pos; }
float ss_red_get_x_dot(void) { return g_x_dot; }
float ss_red_get_theta(void) { return g_theta; }
float ss_red_get_theta_dot_hat(void) { return g_theta_dot_hat; }
float ss_red_get_u_control(void) { return g_u_control; }
float ss_red_get_estado_integrador(void) { return g_estado_integrador; }

// =============================================================================
// 4. LÓGICA DEL OBSERVADOR REDUCIDO
// =============================================================================

static void reduced_observer_update(const RED_Params *p, float theta_meas,
                                    float u_prev, float theta_prev) {
  // Predicción del estado interno z
  // z[k] = F*z[k-1] + G*u[k-1] + H*theta[k-1]
  g_z_obs =
      (p->F_obs * g_z_obs) + (p->H_obs * u_prev) + (p->G_obs * theta_prev);

  // Estimación de la velocidad angular
  // theta_dot_hat[k] = z[k] + L*theta_meas[k]
  g_theta_dot_hat = g_z_obs + (p->L_obs * theta_meas);
}

// =============================================================================
// 5. TAREA PRINCIPAL
// =============================================================================

void state_space_reducido_task(void *arg) {
  (void)arg;
  TickType_t last_wake_time = xTaskGetTickCount();

  PID_Init(&g_ss_red_integrator, 0.0f, 1.0f, 0.0f, DT, -1000.0f, 1000.0f,
           0.03f);

  ESP_LOGI(TAG,
           "Reduced Order Observer SS task ready (1 state estimated, Ts=10ms)");

  while (1) {
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(SS_LOOP_PERIOD_MS));

    if (!g_ss_red_enabled) {
      SS_RED_Reset();
      continue;
    }

    const RED_Params *p =
        (status_get_pendulum_rod() == ROD_LONG) ? &params_long : &params_short;

    // ── PASO 1: LECTURA DE SENSORES ────────────────────────────────────
    g_theta = pulse_counter_get_angle_rad() - (float)M_PI;
    g_x_pos = -pid_get_car_position_m();

    // ── PASO 2: ACTUALIZACIÓN DEL OBSERVADOR REDUCIDO ──────────────────
    reduced_observer_update(p, g_theta, g_u_prev, g_theta_prev);

    // Para x_dot seguimos usando la velocidad integrada (o medida) como
    // referencia
    g_x_dot = g_vel_cmd;

    g_estado_integrador =
        PID_Compute(&g_ss_red_integrator, g_ref_posicion, g_x_pos);

    // ── PASO 3: LEY DE CONTROL LQI ─────────────────────────────────────
    // u = -(K_x·x + K_xdot·ẋ + K_theta·θ + K_w·θ̂̇ - K_i·x_i)
    g_u_control =
        -((p->K_x * g_x_pos) + (p->K_xdot * g_x_dot) + (p->K_theta * g_theta) +
          (p->K_w * g_theta_dot_hat) - (p->K_i * g_estado_integrador));

    // Saturación de seguridad
    if (g_u_control > ACEL_MAX)
      g_u_control = ACEL_MAX;
    if (g_u_control < -ACEL_MAX)
      g_u_control = -ACEL_MAX;

    // ── PASO 4: INTERGRACIÓN Y ACTUACIÓN ───────────────────────────────
    g_vel_cmd += g_u_control * DT;
    if (g_vel_cmd > VEL_CMD_LIMIT)
      g_vel_cmd = VEL_CMD_LIMIT;
    if (g_vel_cmd < -VEL_CMD_LIMIT)
      g_vel_cmd = -VEL_CMD_LIMIT;

    set_motor_velocity(-g_vel_cmd);

    // ── PASO 5: GUARDAR ESTADOS PARA EL PRÓXIMO CICLO ──────────────────
    g_u_prev = g_u_control;
    g_theta_prev = g_theta;
  }
}
