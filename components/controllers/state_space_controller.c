// components/controllers/state_space_controller.c
//
// Controlador LQI con Observador de Luenberger de Orden Completo.
// Estimador de estados para el péndulo invertido sobre carro.
//
// Vector de estado (orden MATLAB): x = [x_pos, x_dot, theta, theta_dot]
// Mediciones:   y = [x_pos, x_dot_meas, theta]   (3 salidas — C es 3×4)
// Estimaciones: theta_dot y x_dot provienen del observador (x_hat[3], x_hat[1])
// Control (LQI):
//   u = -(K_x·x + K_xdot·ẋ̂ + K_theta·θ + K_w·θ̂̇ + K_i·x_i)
//
// Parámetros físicos usados en MATLAB:
//   m=0.04 kg, g=9.81 m/s², l=0.13 m, I=(1/3)·m·l², Ts=0.01 s (ZOH)
//
// Polos del controlador LQI: calculados con dlqr sobre sistema aumentado 5×5
// Polos del observador: {0.40, 0.45, 0.50, 0.55} (más rápidos que el control)

#include "state_space_controller.h"

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

static const char *TAG = "SS_CTRL";

#define SS_LOOP_PERIOD_MS 10
#define DT (SS_LOOP_PERIOD_MS / 1000.0f)

// Límite de velocidad del carro (m/s) — igual que el integrador previo
#define VEL_CMD_LIMIT 1.1f
#define ACEL_MAX 20.0f

// =============================================================================
// 1. MATRICES DEL SISTEMA DISCRETO  (calculadas en MATLAB, c2d ZOH, Ts=10 ms)
//    Orden de estados: [x_pos, x_dot, theta, theta_dot]
// =============================================================================

typedef struct {
  float Ad[4][4];
  float Bd[4];
  float Cd[3][4];
  float L_obs[4][3];
  float K_x;
  float K_xdot;
  float K_theta;
  float K_w;
  float K_i;
} LQR_Params;

static const LQR_Params params_long = {
    .Ad = {{1.0000f, 0.0100f, 0.0000f, 0.0000f},
           {0.0000f, 1.0000f, 0.0000f, 0.0000f},
           {0.0000f, 0.0000f, 1.0018f, 0.0100f},
           {0.0000f, 0.0000f, 0.3681f, 1.0018f}},
    .Bd = {0.000051f, 0.0100f, -0.000375f, -0.0750},
    .Cd = {{1.0f, 0.0f, 0.0f, 0.0f},
           {0.0f, 1.0f, 0.0f, 0.0f},
           {0.0f, 0.0f, 1.0f, 0.0f}},
    .L_obs = {{0.3000f, 0.0100f, 0.0000f},
              {0.0000f, 0.2500f, 0.0000f},
              {0.0000f, 0.0000f, 0.75f},
              {0.0000f, 0.0000f, 14.5f}},
    .K_x = -29.81f,
    .K_xdot = -21.86,
    .K_theta = -48.64,
    .K_w = -7.23,
    .K_i = -0.0f};

// PLACEHOLDER: Duplicado de la vara larga para la vara corta.
// Sustituir con valores calculados posteriormente.
static const LQR_Params params_short = {
    .Ad = {{1.0000f, 0.0100f, 0.0000f, 0.0000f},
           {0.0000f, 1.0000f, 0.0000f, 0.0000f},
           {0.0000f, 0.0000f, 1.0018f, 0.0100f},
           {0.0000f, 0.0000f, 0.3681f, 1.0018f}},
    .Bd = {0.000051f, 0.0100f, -0.000375f, -0.0750},
    .Cd = {{1.0f, 0.0f, 0.0f, 0.0f},
           {0.0f, 1.0f, 0.0f, 0.0f},
           {0.0f, 0.0f, 1.0f, 0.0f}},
    .L_obs = {{0.3000f, 0.0100f, 0.0000f},
              {0.0000f, 0.2500f, 0.0000f},
              {0.0000f, 0.0000f, 0.7614f},
              {0.0000f, 0.0000f, 15.56f}},
    .K_x = -6.56f,
    .K_xdot = -8.55,
    .K_theta = -30.8,
    .K_w = -2.9708f,
    .K_i = -01.0f};

// =============================================================================
// 3. VARIABLES GLOBALES DE ESTADO
// =============================================================================

static volatile bool g_ss_enabled = false;

// Vector de estado estimado por el observador: [x_hat, x_dot_hat, theta_hat,
// theta_dot_hat]
static float x_hat[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// Variables expuestas vía getters (extraídas de x_hat o de sensores)
static float g_x_pos = 0.0f;
static float g_x_dot = 0.0f; // x_hat[1]
static float g_theta = 0.0f;
static float g_theta_dot_hat = 0.0f; // x_hat[3]
static float g_u_control = 0.0f;
static float g_estado_integrador = 0.0f;

// Variables internas del observador y del actuador
static float g_x_pos_prev = 0.0f; // para calcular x_dot por diferencia finita
static float g_u_prev = 0.0f;  // u[k-1] usado en la predicción del observador
static float g_vel_cmd = 0.0f; // velocidad integrada enviada al motor (m/s)

// Referencia de posición (m)
static float g_ref_posicion = 0.1f;

// Integrador LQI — 5to estado del sistema aumentado (Ki=1, límites ±2 m·s)
static PIDController g_ss_integrator;

// =============================================================================
// 4. OBSERVADOR DE LUENBERGER — función privada
//
//  Ecuación:  x̂[k+1] = Ad·x̂[k] + Bd·u[k-1] + L·(y[k] − Cd·x̂[k])
//
//  El término L·(y − Cd·x̂) corrige la predicción con el error de innovación,
//  manteniendo el estimador sincronizado con las mediciones reales.
// =============================================================================
static void luenberger_update(const LQR_Params *pLQR, const float y[3],
                              float u_prev) {
  // 1. Innovación: diferencia entre medición real y predicción
  //    innov[i] = y[i] - (Cd · x_hat)[i]
  float innov[3];
  for (int r = 0; r < 3; r++) {
    float cd_xhat = 0.0f;
    for (int c = 0; c < 4; c++)
      cd_xhat += pLQR->Cd[r][c] * x_hat[c];
    innov[r] = y[r] - cd_xhat;
  }

  // 2. Predicción + corrección en un solo paso
  //    x̂_next[i] = (Ad·x̂)[i] + Bd[i]·u_prev + (L·innov)[i]
  float x_hat_next[4];
  for (int i = 0; i < 4; i++) {
    float pred = 0.0f;
    for (int j = 0; j < 4; j++)
      pred += pLQR->Ad[i][j] * x_hat[j];
    pred += pLQR->Bd[i] * u_prev;

    float corr = 0.0f;
    for (int k = 0; k < 3; k++)
      corr += pLQR->L_obs[i][k] * innov[k];

    x_hat_next[i] = pred + corr;
  }

  memcpy(x_hat, x_hat_next, sizeof(x_hat));
}

// =============================================================================
// 5. FUNCIONES DE GESTIÓN DE ESTADO
// =============================================================================

void SS_Reset(void) {
  memset(x_hat, 0, sizeof(x_hat));
  g_x_pos_prev = pid_get_car_position_m();
  g_u_prev = 0.0f;
  g_vel_cmd = 0.0f;
  g_estado_integrador = 0.0f;
  PID_Reset(&g_ss_integrator);
}

void SS_UpdateReference(float x_ref, float theta_ref) {
  (void)theta_ref;
  g_ref_posicion = x_ref;
}

void ss_toggle_enable(void) {
  g_ss_enabled = !g_ss_enabled;
  if (g_ss_enabled) {
    SS_Reset();
    ESP_LOGW(TAG, "State Space LQI + Luenberger ENABLED");
  } else {
    set_motor_velocity(0.0f);
    ESP_LOGW(TAG, "State Space Control DISABLED");
  }
}

void ss_force_disable(void) {
  if (g_ss_enabled) {
    g_ss_enabled = false;
    SS_Reset();
    set_motor_velocity(0.0f);
    ESP_LOGE(TAG, "EMERGENCY STOP — State Space disabled");
  }
}

bool ss_is_enabled(void) { return g_ss_enabled; }
float ss_get_x_pos(void) { return g_x_pos; }
float ss_get_x_dot(void) { return g_x_dot; }
float ss_get_theta(void) { return g_theta; }
float ss_get_theta_dot_hat(void) { return g_theta_dot_hat; }
float ss_get_u_control(void) { return g_u_control; }
float ss_get_estado_integrador(void) { return g_estado_integrador; }

// =============================================================================
// 6. TAREA PRINCIPAL DEL CONTROLADOR
// =============================================================================

void state_space_controller_task(void *arg) {
  (void)arg;
  TickType_t last_wake_time = xTaskGetTickCount();

  // Integrador LQI: integra el error de posición  x_i[k+1] = x_i[k] + (r-x)·Ts
  // Kp=0, Ki=1, Kd=0 → salida = Σ(error)·DT, saturada en ±0.25 m·s
  PID_Init(&g_ss_integrator, 0.0f, 1.0f, 0.0f, DT, -1000.0f, 1000.0f, 0.03f);

  ESP_LOGI(
      TAG,
      "LQI + Luenberger Observer ready (4 states, 3 measurements, Ts=10ms)");

  while (1) {
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(SS_LOOP_PERIOD_MS));

    if (!g_ss_enabled) {
      SS_Reset();
      continue;
    }

    const LQR_Params *pLQR =
        (status_get_pendulum_rod() == ROD_LONG) ? &params_long : &params_short;

    // ── PASO 1: LECTURA DE SENSORES ────────────────────────────────────
    // θ = 0 en la posición vertical superior (punto de equilibrio)
    g_theta = pulse_counter_get_angle_rad() - (float)M_PI;

    // x = posición del carro (m), signo negado según convención física del
    // sistema
    g_x_pos = -pid_get_car_position_m();

    // ẋ medida = diferencia finita de posición  (entrada al observador como
    // y[1])
    float x_dot_meas = g_vel_cmd;
    g_x_pos_prev = g_x_pos;

    // ── PASO 2: ACTUALIZACIÓN DEL OBSERVADOR ───────────────────────────
    // y = [x_pos, x_dot_meas, theta]  → misma estructura que Cd en MATLAB
    float y_meas[3] = {g_x_pos, x_dot_meas, g_theta};
    luenberger_update(pLQR, y_meas, g_u_prev);

    // Extraer estados estimados del vector x_hat
    // Para estados medibles (x, θ) preferimos la medición directa;
    // el observador corrige la deriva del modelo pero no reemplaza el sensor.
    g_x_dot = x_hat[1]; // ẋ̂  — estimada, reemplaza integral de u
    g_theta_dot_hat =
        x_hat[3]; // θ̂̇  — estimada, reemplaza diferencia finita ruidosa
    g_estado_integrador =
        PID_Compute(&g_ss_integrator, g_ref_posicion, g_x_pos);

    // ── PASO 4: LEY DE CONTROL LQI ─────────────────────────────────────
    // u = -(K_x·x + K_xdot·ẋ̂ + K_theta·θ + K_w·θ̂̇ + K_i·x_i)
    // Los estados medibles (x_pos, theta) usan medición directa del sensor.
    // Los estados no medibles (x_dot, theta_dot) usan el estimador.
    g_u_control = -((pLQR->K_x * g_x_pos) + (pLQR->K_xdot * g_x_dot) +
                    (pLQR->K_theta * g_theta) + (pLQR->K_w * g_theta_dot_hat) -
                    (pLQR->K_i * g_estado_integrador));

    // Saturación de seguridad
    if (g_u_control > ACEL_MAX)
      g_u_control = ACEL_MAX;
    if (g_u_control < -ACEL_MAX)
      g_u_control = -ACEL_MAX;

    // ── PASO 5: INTEGRAR u → VELOCIDAD PARA EL MOTOR ──────────────────
    // u es una aceleración (m/s²). Integramos para obtener velocidad (m/s).
    // vel_cmd[k] = vel_cmd[k-1] + u[k] * DT
    g_vel_cmd += g_u_control * DT;
    if (g_vel_cmd > VEL_CMD_LIMIT)
      g_vel_cmd = VEL_CMD_LIMIT;
    if (g_vel_cmd < -VEL_CMD_LIMIT)
      g_vel_cmd = -VEL_CMD_LIMIT;

    set_motor_velocity(-g_vel_cmd);

    // ── PASO 6: GUARDAR u PARA EL PRÓXIMO CICLO DEL OBSERVADOR ─────────
    g_u_prev = g_u_control;
  }
}
