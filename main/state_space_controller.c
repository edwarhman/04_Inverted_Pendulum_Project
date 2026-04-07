#include "state_space_controller.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pid_controller.h"
#include "pulse_counter.h"
#include "pwm_generator.h"
#include <math.h>

static const char *TAG = "STATE_SPACE_CTRL";

#define SS_LOOP_PERIOD_MS 10
#define MAX_CONTROL_EFFORT 1700.0f

#define DT (SS_LOOP_PERIOD_MS / 1000.0f)

#define K_X 0.0f
#define K_X_DOT 0.0f
#define K_THETA 0.0f
#define K_TH_DOT 0.0f

#define L_00 0.0f
#define L_01 0.0f
#define L_10 0.0f
#define L_11 0.0f
#define L_20 0.0f
#define L_21 0.0f
#define L_30 0.0f
#define L_31 0.0f

static volatile bool g_ss_enabled = false;
static float g_control_output = 0.0f;
static float g_x_hat[NUM_STATES] = {0};
static float g_x_ref[NUM_STATES] = {0};

void SS_UpdateObserver(float u, float x_meas, float theta_meas) {
  float x_hat_dot[NUM_STATES];

  x_hat_dot[0] = g_x_hat[1];
  x_hat_dot[1] = u;
  x_hat_dot[2] = g_x_hat[3];
  x_hat_dot[3] = 0.0f;

  for (int i = 0; i < NUM_STATES; i++) {
    g_x_hat[i] += DT * x_hat_dot[i];
  }

  float error_x = x_meas - g_x_hat[0];
  float error_theta = theta_meas - g_x_hat[2];

  g_x_hat[0] += L_00 * error_x + L_01 * error_theta;
  g_x_hat[1] += L_10 * error_x + L_11 * error_theta;
  g_x_hat[2] += L_20 * error_x + L_21 * error_theta;
  g_x_hat[3] += L_30 * error_x + L_31 * error_theta;
}

void SS_Compute(void) {
  float u = K_X * (g_x_hat[0] - g_x_ref[0]) +
            K_X_DOT * (g_x_hat[1] - g_x_ref[1]) +
            K_THETA * (g_x_hat[2] - g_x_ref[2]) +
            K_TH_DOT * (g_x_hat[3] - g_x_ref[3]);

  if (u > MAX_CONTROL_EFFORT)
    u = MAX_CONTROL_EFFORT;
  if (u < -MAX_CONTROL_EFFORT)
    u = -MAX_CONTROL_EFFORT;

  g_control_output = u;
}

float *SS_GetEstimatedStates(void) { return g_x_hat; }

float SS_GetControlOutput(void) { return g_control_output; }

void SS_Reset(void) {
  for (int i = 0; i < NUM_STATES; i++) {
    g_x_hat[i] = 0.0f;
  }
  g_control_output = 0.0f;
}

void SS_UpdateReference(float x_ref, float theta_ref) {
  g_x_ref[0] = x_ref;
  g_x_ref[2] = theta_ref;
}

void ss_toggle_enable(void) {
  g_ss_enabled = !g_ss_enabled;
  if (g_ss_enabled) {
    SS_Reset();
    ESP_LOGW(TAG, "State Space Control ENABLED");
  } else {
    motor_command_t stop_cmd = {0, 0, 0};
    xQueueOverwrite(motor_command_queue, &stop_cmd);
    ESP_LOGW(TAG, "State Space Control DISABLED");
  }
}

void ss_force_disable(void) {
  if (g_ss_enabled) {
    g_ss_enabled = false;
    SS_Reset();
    motor_command_t stop_cmd = {0, 0, 0};
    xQueueOverwrite(motor_command_queue, &stop_cmd);
    ESP_LOGE(TAG, "EMERGENCY STOP!");
  }
}

bool ss_is_enabled(void) { return g_ss_enabled; }

void state_space_controller_task(void *arg) {
  (void)arg;
  TickType_t last_wake_time = xTaskGetTickCount();

  ESP_LOGI(TAG, "State Space Controller initialized (K and L need tuning)");

  ss_toggle_enable();
  ss_toggle_enable();

  while (1) {
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(SS_LOOP_PERIOD_MS));

    if (!g_ss_enabled) {
      SS_Reset();
      continue;
    }

    int16_t current_theta = pulse_counter_get_value();
    int32_t current_x = g_car_position_pulses;

    float theta_rad =
        (float)current_theta * 2.0f * M_PI / ENCODER_PULSES_PER_REV;
    float x_meters = (float)current_x * CARRIAGE_MOVEMENT_M_PER_PULSE;

    SS_Compute();
    float u = SS_GetControlOutput();

    SS_UpdateObserver(u, x_meters, theta_rad);

    if (fabsf(u) > 0.01f) {
      int direction = (u > 0) ? 1 : 0;
      int frequency = 1000 + (int)(fabsf(u) * 80);
      if (frequency > 150000)
        frequency = 150000;

      motor_command_t cmd = {0, frequency, direction};
      xQueueOverwrite(motor_command_queue, &cmd);
    } else {
      motor_command_t stop_cmd = {0, 0, 0};
      xQueueOverwrite(motor_command_queue, &stop_cmd);
    }
  }
}
