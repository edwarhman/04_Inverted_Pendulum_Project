#include "swing_up_controller.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pid_controller.h"
#include "pulse_counter.h"
#include "system_status.h"
#include "state_space_funcional.h" // Para la transición automática

static const char *TAG = "SWING_UP";

// Parámetros de la rutina de Swing-Up
#define SWING_UP_PERIOD_MS 10
#define SWING_UP_GAIN 0.8f  // Ganancia de energía para el balanceo
#define MAX_SWING_VELOCITY 1.1f // m/s

// Umbrales para captura y transición a control funcional
#define CATCH_ANGLE_DEG 20.0f
#define CATCH_VELOCITY_RADS 1.5f

// Filtro para velocidad angular
#define VELOCITY_FILTER_ALPHA 0.3f

static volatile bool g_swing_up_active = false;
static PIDController g_swing_velocity_integrator;

void swing_up_enable(void) {
  if (!g_swing_up_active) {
    g_swing_up_active = true;
    // Inicializar el integrador de velocidad (Kp=0, Ki=1, Kd=0)
    PID_Init(&g_swing_velocity_integrator, 0.0f, 1.0f, 0.0f,
             (SWING_UP_PERIOD_MS / 1000.0f), -MAX_SWING_VELOCITY,
             MAX_SWING_VELOCITY, 0.0f);

    status_set_control_mode(MODE_SWING_UP);
    ESP_LOGW(TAG, "Rutina de Swing-Up HABILITADA");
  }
}

void swing_up_disable(void) {
  if (g_swing_up_active) {
    g_swing_up_active = false;
    PID_Reset(&g_swing_velocity_integrator);
    set_motor_velocity(0.0f);
    ESP_LOGW(TAG, "Rutina de Swing-Up DESHABILITADA");
  }
}

void swing_up_force_disable(void) {
  if (g_swing_up_active) {
    g_swing_up_active = false;
    PID_Reset(&g_swing_velocity_integrator);
    set_motor_velocity(0.0f);
    ESP_LOGE(TAG, "EMERGENCY STOP - Swing-Up disabled");
  }
}

bool swing_up_is_active(void) { return g_swing_up_active; }

void swing_up_task(void *arg) {
  (void)arg;
  float dt = SWING_UP_PERIOD_MS / 1000.0f;
  float last_angle = pulse_counter_get_angle_rad();
  float filtered_theta_dot = 0.0f;

  // Punto de equilibrio inestable (vertical superior)
  const float vertical_setpoint = (float)M_PI;

  while (1) {
    vTaskDelay(pdMS_TO_TICKS(SWING_UP_PERIOD_MS));

    if (!g_swing_up_active) {
      last_angle = pulse_counter_get_angle_rad();
      filtered_theta_dot = 0.0f;
      continue;
    }

    // 1. Obtener estado del péndulo
    float current_angle = pulse_counter_get_angle_rad();

    // 2. Estimar velocidad angular (diferencia finita + filtro)
    float raw_theta_dot = (current_angle - last_angle) / dt;
    
    // Manejar wrap-around del encoder (0 a 2*PI)
    if (raw_theta_dot > M_PI / dt) raw_theta_dot -= (2 * M_PI) / dt;
    if (raw_theta_dot < -M_PI / dt) raw_theta_dot += (2 * M_PI) / dt;

    filtered_theta_dot = (VELOCITY_FILTER_ALPHA * raw_theta_dot) +
                         ((1.0f - VELOCITY_FILTER_ALPHA) * filtered_theta_dot);
    last_angle = current_angle;

    // 3. Verificar condición de "Captura" (Handoff to Functional Control)
    float angle_error = current_angle - vertical_setpoint;
    
    // Normalizar error a [-PI, PI]
    while (angle_error > M_PI) angle_error -= 2 * M_PI;
    while (angle_error < -M_PI) angle_error += 2 * M_PI;

    if (fabsf(angle_error) < (CATCH_ANGLE_DEG * M_PI / 180.0f)) {
      
      ESP_LOGW(TAG, "¡Péndulo capturado! Transicionando a modo FUNCIONAL...");
      swing_up_disable();
      
      // Transición automática al controlador funcional lineal
      status_set_control_mode(MODE_STATE_SPACE_FUNC);
      ss_func_enable(); 
      continue;
    }

    // 4. Calcular Aceleración de Swing-Up (Control por energía/ganancia)
    // a = G * theta_dot * cos(theta_err)
    float a_swing = SWING_UP_GAIN * filtered_theta_dot * cosf(angle_error);

    // 5. Protección de límites: Detener si estamos cerca de los extremos
    // Nota: status_get_travel_limits no está en system_status.h del main actual, 
    // pero g_car_position_pulses es visible o usamos getters del pid_controller.
    
    // int32_t current_pulses = pid_get_car_position_pulses();
    // Usamos un margen de seguridad genérico de 15cm (asumiendo ~20000 pulsos total)
    // Lo ideal es que el usuario calibre primero.
    
    // 6. Integrar Aceleración para obtener Velocidad
    float v_swing = PID_Compute(&g_swing_velocity_integrator, a_swing, 0.0f);

    // 7. Actuar
    set_motor_velocity(v_swing);
  }
}
