// src/button_handler.c
#include "button_handler.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h" // Añadido para medir tiempo de movimiento
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lcd_controller.h"
#include "pid_controller.h"
#include "pulse_counter.h"
#include "pwm_generator.h"
#include "system_status.h" // Para manejar el estado del movimiento manual
#include "state_space_controller.h" // AÑADIDO: LQR state space
#include "state_space_reducido.h" // AÑADIDO: LQR Reducido
#include "state_space_funcional.h" // AÑADIDO: LQR Funcional
#include "swing_up_controller.h" // AÑADIDO: Swing-Up
#include <stdio.h>
#include <math.h>

// --- PINES DE LOS BOTONES ---
#define VIEW_CYCLE_BUTTON_GPIO GPIO_NUM_19 // ¡NUEVO BOTÓN!
#define CALIBRATION_BUTTON_GPIO GPIO_NUM_15
#define ENABLE_PID_BUTTON_GPIO                                                 \
  GPIO_NUM_18 // Botón para Habilitar/Deshabilitar PID
#define MANUAL_LEFT_BUTTON_GPIO                                                \
  GPIO_NUM_16 // Nuevo botón para mover a la izquierda
#define MANUAL_RIGHT_BUTTON_GPIO                                               \
  GPIO_NUM_17 // Nuevo botón para mover a la derecha
#define EMERGENCY_STOP_GPIO_LEFT                                               \
  GPIO_NUM_34 // Botón de parada de emergencia izquierdo
#define EMERGENCY_STOP_GPIO_RIGHT                                              \
  GPIO_NUM_35 // Botón de parada de emergencia derecho

// --- PARÁMETROS DE MOVIMIENTO MANUAL ---
/*#define MANUAL_MOVE_SPEED_HZ 20000 // Velocidad constante para el movimiento
manual (alta) #define MANUAL_MOVE_PULSES   400   // Cantidad de pulsos por ciclo
(movimiento suave)

// --- PARÁMETROS DE MOVIMIENTO secuencia ---
#define SEQUENCE_BASE_PULSES 16000
#define SEQUENCE_BASE_SPEED_HZ 5000*/

// --- PARÁMETROS DE MOVIMIENTO ---
#define JOG_SPEED_HZ 20000         // Velocidad para el movimiento manual (jog)
#define JOG_PULSES 400             // Pulsos por ciclo de jog
#define CALIBRATION_SPEED_HZ 20000 // Velocidad constante para la calibración

// --- PARÁMETROS DE SINTONÍA PID ---
#define PID_SHORT_PRESS_STEP 0.1f // Paso para pulsación corta (1 paso)
#define PID_LONG_PRESS_STEP 1.0f  // Paso para pulsación larga (3 pasos)
#define PID_LONG_PRESS_THRESHOLD                                               \
  20 // Umbral para detectar pulsación larga (20 * 10ms = 200ms)
#define PID_PRESS_DURATION_MAX                                                 \
  30 // Duración máxima de verificación (30 * 10ms = 300ms)

typedef enum {
    CALIB_IDLE,
    CALIB_MOVING_RIGHT,
    CALIB_MOVING_LEFT,
    CALIB_MOVING_CENTER,
    CALIB_STABILIZING,
    CALIB_FINALIZING
} calib_state_t;

static volatile calib_state_t g_calib_state = CALIB_IDLE;
static int32_t g_limit_left_pos = 0;
static int32_t g_limit_right_pos = 0;
static int64_t g_calib_timer = 0;
static int g_actual_view_int = 0;

static const char *TAG = "BUTTON_HANDLER";
// Para saber si el PID está activo, llamaremos a una función.
// extern bool g_pid_enabled;

// --- Contador de posición del carro en micropasos ---
// static int32_t g_car_position_pulses = 0;
// static volatile bool g_is_calibrating = false; // Ya no lo usamos así
static int32_t g_calibrated_travel_range_pulses = 0;

// Función auxiliar para botones de comando (pulsar y soltar)
static bool is_command_button_pressed(int gpio_num) {
  if (gpio_get_level(gpio_num) == 0) {
    vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
    if (gpio_get_level(gpio_num) == 0) {
      while (gpio_get_level(gpio_num) == 0) {
        vTaskDelay(pdMS_TO_TICKS(50));
      }
      return true;
    }
  }
  return false;
}

void button_handler_task(void *arg) {
  ESP_LOGI(TAG, "Iniciando tarea de lectura de botones...");

  // Configuración de los 3 pines GPIO como entrada con pull-up
  gpio_config_t io_conf = {
      .pin_bit_mask = ((1ULL << ENABLE_PID_BUTTON_GPIO) |
                       (1ULL << MANUAL_LEFT_BUTTON_GPIO) |
                       (1ULL << MANUAL_RIGHT_BUTTON_GPIO) |
                       (1ULL << EMERGENCY_STOP_GPIO_RIGHT) |
                       (1ULL << EMERGENCY_STOP_GPIO_LEFT) |
                       (1ULL << VIEW_CYCLE_BUTTON_GPIO)),
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&io_conf);

  ESP_LOGI(
      TAG,
      "Tarea iniciada. BOOT:Habilitar PID. GPIO21:Izquierda. GPIO22:Derecha.");

  int last_button_state = 1; // 1 = no presionado
  // int last_sequence_button_state = 1;
  int last_stop_button_state_right = 1;
  int last_stop_button_state_left = 1;
  int last_view_button_state = 1;

  while (1) {
    // --- AÑADIDO: Lógica para el botón de cambio de vista ---
    int current_view_button_state = gpio_get_level(VIEW_CYCLE_BUTTON_GPIO);
    if (last_view_button_state == 1 && current_view_button_state == 0) {
      vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
      if (gpio_get_level(VIEW_CYCLE_BUTTON_GPIO) == 0) {
        ESP_LOGI(TAG, "Botón de cambio de vista presionado.");
        status_cycle_lcd_view(); // ¡Llamamos a nuestra nueva función!
      }
    }
    last_view_button_state = current_view_button_state;
    // --- ACTUALIZACIÓN DE CALIBRACIÓN (Máquina de Estados) ---
    void button_handler_update_calibration(void);
    button_handler_update_calibration();

    // --- AÑADIDO: Lógica para la Parada de Emergencia (máxima prioridad) ---
    int current_stop_button_state_right = gpio_get_level(EMERGENCY_STOP_GPIO_RIGHT);
    if (last_stop_button_state_right == 1 && current_stop_button_state_right == 0) {

      if (gpio_get_level(EMERGENCY_STOP_GPIO_RIGHT) == 0) {
        // Llama a la función que solo deshabilita
        pid_force_disable();
        ss_force_disable(); // Detener ambos siempre
        ss_red_force_disable();
        ss_func_force_disable();
        swing_up_force_disable();

        // Re-homing dinámico: Si conocemos el centro, recalibramos en el límite negativo (-travel_range/2)
        if (g_calibrated_travel_range_pulses > 0) {
            g_car_position_pulses = -(g_calibrated_travel_range_pulses / 2);
            ESP_LOGW(TAG, "Re-homing RIGHT LIMIT: position reset to %ld", (long)g_car_position_pulses);
        }
      }
      // vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
    }
    last_stop_button_state_right = current_stop_button_state_right;

    int current_stop_button_state_left =
        gpio_get_level(EMERGENCY_STOP_GPIO_LEFT);
    if (last_stop_button_state_left == 1 && current_stop_button_state_left == 0) {

      if (gpio_get_level(EMERGENCY_STOP_GPIO_LEFT) == 0) {
        // Llama a la función que solo deshabilita
        pid_force_disable();
        ss_force_disable(); // Detener ambos
        ss_red_force_disable();
        ss_func_force_disable();
        swing_up_force_disable();

        // Re-homing dinámico: Si conocemos el centro, recalibramos en el límite positivo (+travel_range/2)
        if (g_calibrated_travel_range_pulses > 0) {
            g_car_position_pulses = (g_calibrated_travel_range_pulses / 2);
            ESP_LOGW(TAG, "Re-homing LEFT LIMIT: position reset to %ld", (long)g_car_position_pulses);
        }
      }
    }
    last_stop_button_state_left = current_stop_button_state_left;

    int current_button_state = gpio_get_level(ENABLE_PID_BUTTON_GPIO);

    // Detectar el flanco de bajada (cuando se presiona el botón)
    if (last_button_state == 1 && current_button_state == 0) {
      // Anti-rebote: esperar un poco y confirmar
      vTaskDelay(pdMS_TO_TICKS(50));
      if (gpio_get_level(ENABLE_PID_BUTTON_GPIO) == 0 &&
          gpio_get_level(EMERGENCY_STOP_GPIO_LEFT) == 1 &&
          gpio_get_level(EMERGENCY_STOP_GPIO_RIGHT) == 1) {
        control_toggle_current();
      }
    }
    last_button_state = current_button_state;

    // Solo permitimos el movimiento manual si NO estamos en vista de sintonización o selección de modo, y el control está apagado
    if (!pid_is_enabled() && !ss_is_enabled() && !ss_red_is_enabled() && !ss_func_is_enabled() && !swing_up_is_active() && status_get_lcd_view() != VIEW_PID_GAINS && status_get_lcd_view() != VIEW_CONTROL_MODE && status_get_lcd_view() != VIEW_ROD_SELECTION) {

      // --- LÓGICA DE CALIBRACIÓN (HOMING) ---
      if (is_command_button_pressed(CALIBRATION_BUTTON_GPIO)) {
        if (status_get_lcd_view() == VIEW_PID_GAINS) {
          // Si estamos en la vista de ganancias, ciclar entre Kp, Ki y Kd
          status_cycle_pid_param();
          ESP_LOGI(TAG, "Parámetro PID seleccionado: %d",
                   status_get_pid_param());
        } else {
          button_handler_start_calibration();
        } // fin de else
      }

      // Leemos el estado de los botones de movimiento manual
      static bool jogging_active = false;
      static int64_t last_jog_time = 0;
      static int last_jog_direction = -1; // -1 inv, 0 izq, 1 der
      int left_button_state = gpio_get_level(MANUAL_LEFT_BUTTON_GPIO);
      int right_button_state = gpio_get_level(MANUAL_RIGHT_BUTTON_GPIO);

      // Si se presiona el botón izquierdo Y no el derecho
      if (left_button_state == 1 && right_button_state == 0 &&
          gpio_get_level(EMERGENCY_STOP_GPIO_RIGHT) == 1) {
        status_set_manual_move_state(MANUAL_MOVE_RIGHT); // Reportar estado
        ESP_LOGD(TAG, "Moviendo a la izquierda...");
        motor_command_t cmd = {.num_pulses = JOG_PULSES,
                               .frequency = JOG_SPEED_HZ,
                               .direction = 0};
        xQueueOverwrite(motor_command_queue, &cmd);

        if (!jogging_active || last_jog_direction != 0) {
          last_jog_time = esp_timer_get_time();
          jogging_active = true;
          last_jog_direction = 0;
        } else {
          int64_t current_time = esp_timer_get_time();
          int64_t dt_us = current_time - last_jog_time;
          int32_t pulses_moved = (int32_t)((dt_us * JOG_SPEED_HZ) / 1000000);
          g_car_position_pulses -= pulses_moved;
          last_jog_time = current_time;
        }
      }
      // Si se presiona el botón derecho Y no el izquierdo
      else if (right_button_state == 1 && left_button_state == 0 &&
               gpio_get_level(EMERGENCY_STOP_GPIO_LEFT) == 1) {
        status_set_manual_move_state(MANUAL_MOVE_LEFT); // Reportar estado
        ESP_LOGD(TAG, "Moviendo a la derecha...");
        motor_command_t cmd = {.num_pulses = JOG_PULSES,
                               .frequency = JOG_SPEED_HZ,
                               .direction = 1};
        xQueueOverwrite(motor_command_queue, &cmd);

        if (!jogging_active || last_jog_direction != 1) {
          last_jog_time = esp_timer_get_time();
          jogging_active = true;
          last_jog_direction = 1;
        } else {
          int64_t current_time = esp_timer_get_time();
          int64_t dt_us = current_time - last_jog_time;
          int32_t pulses_moved = (int32_t)((dt_us * JOG_SPEED_HZ) / 1000000);
          g_car_position_pulses += pulses_moved;
          last_jog_time = current_time;
        }
      } else {
        // Si no se presiona ningún botón o ambos (o hay choque de límites),
        // detenemos el movimiento
        status_set_manual_move_state(MANUAL_MOVE_NONE); // Reportar estado

        if (jogging_active) {
          int64_t current_time = esp_timer_get_time();
          int64_t dt_us = current_time - last_jog_time;
          int32_t pulses_moved = (int32_t)((dt_us * JOG_SPEED_HZ) / 1000000);
          if (last_jog_direction == 0) {
            g_car_position_pulses -= pulses_moved;
          } else if (last_jog_direction == 1) {
            g_car_position_pulses += pulses_moved;
          }

          motor_command_t stop_cmd = {
              .num_pulses = 0, .frequency = 0, .direction = 0};
          xQueueOverwrite(motor_command_queue, &stop_cmd);
          jogging_active = false;
          last_jog_direction = -1;
        }
      }
    } else {
      // Si el PID está activado o estamos sintonizando parámetros,
      // nos aseguramos de que el estado manual esté limpio y evaluamos
      // sintonización
      status_set_manual_move_state(MANUAL_MOVE_NONE);

      if (status_get_lcd_view() == VIEW_PID_GAINS) {
        // Lógica de sintonización
        if (is_command_button_pressed(CALIBRATION_BUTTON_GPIO)) {
          status_cycle_pid_param();
          ESP_LOGI(TAG, "Parámetro PID seleccionado: %d",
                   status_get_pid_param());
        }

        pid_param_select_t selected_param = status_get_pid_param();

        if (selected_param != SELECT_NONE) {
          int left_button_state = gpio_get_level(MANUAL_LEFT_BUTTON_GPIO);
          int right_button_state = gpio_get_level(MANUAL_RIGHT_BUTTON_GPIO);

          // Handle right button (increment)
          if (right_button_state == 0 && left_button_state == 1) {
            // Detect long press for faster increment
            int press_duration = 0;
            while (gpio_get_level(MANUAL_RIGHT_BUTTON_GPIO) == 0 &&
                   press_duration < PID_PRESS_DURATION_MAX) {
              vTaskDelay(pdMS_TO_TICKS(10));
              press_duration++;
            }

            float step = (press_duration >= PID_LONG_PRESS_THRESHOLD)
                             ? PID_LONG_PRESS_STEP
                             : PID_SHORT_PRESS_STEP;

            if (selected_param == SELECT_KP)
              pid_set_kp(pid_get_kp() + step);
            else if (selected_param == SELECT_KI)
              pid_set_ki(pid_get_ki() + step);
            else if (selected_param == SELECT_KD)
              pid_set_kd(pid_get_kd() + step);

            vTaskDelay(pdMS_TO_TICKS(150)); // Delay for continuous tuning
          }
          // Handle left button (decrement)
          else if (left_button_state == 0 && right_button_state == 1) {
            // Detect long press for faster decrement
            int press_duration = 0;
            while (gpio_get_level(MANUAL_LEFT_BUTTON_GPIO) == 0 &&
                   press_duration < PID_PRESS_DURATION_MAX) {
              vTaskDelay(pdMS_TO_TICKS(10));
              press_duration++;
            }

            float step = (press_duration >= PID_LONG_PRESS_THRESHOLD)
                             ? PID_LONG_PRESS_STEP
                             : PID_SHORT_PRESS_STEP;

            if (selected_param == SELECT_KP)
              pid_set_kp(pid_get_kp() - step);
            else if (selected_param == SELECT_KI)
              pid_set_ki(pid_get_ki() - step);
            else if (selected_param == SELECT_KD)
              pid_set_kd(pid_get_kd() - step);

            vTaskDelay(pdMS_TO_TICKS(150)); // Delay for continuous tuning
          }
        }
      } else if (status_get_lcd_view() == VIEW_CONTROL_MODE) {
        int left_button_state = gpio_get_level(MANUAL_LEFT_BUTTON_GPIO);
        int right_button_state = gpio_get_level(MANUAL_RIGHT_BUTTON_GPIO);

        if (right_button_state == 0 && left_button_state == 1) {
          int next_mode = (int)status_get_control_mode() + 1;
          if (next_mode > 3) next_mode = 0; // Skip MODE_SWING_UP (4)
          control_switch_mode((control_mode_t)next_mode);
          vTaskDelay(pdMS_TO_TICKS(150));
        } else if (left_button_state == 0 && right_button_state == 1) {
          int prev_mode = (int)status_get_control_mode() - 1;
          if (prev_mode < 0) prev_mode = 3; // Skip MODE_SWING_UP
          control_switch_mode((control_mode_t)prev_mode);
          vTaskDelay(pdMS_TO_TICKS(150));
        }
      } else if (status_get_lcd_view() == VIEW_ROD_SELECTION) {
        int left_button_state = gpio_get_level(MANUAL_LEFT_BUTTON_GPIO);
        int right_button_state = gpio_get_level(MANUAL_RIGHT_BUTTON_GPIO);

        if ((right_button_state == 0 && left_button_state == 1) || (left_button_state == 0 && right_button_state == 1)) {
          pendulum_rod_t act = status_get_pendulum_rod();
          if (act == ROD_LONG) status_set_pendulum_rod(ROD_SHORT);
          else status_set_pendulum_rod(ROD_LONG);
          vTaskDelay(pdMS_TO_TICKS(150));
        }
      }
    }

    button_handler_update_calibration();

    vTaskDelay(pdMS_TO_TICKS(20)); // Sondeo mayor o igual a 10ms 10ms
  }
}

void button_handler_update_calibration(void) {
  if (g_calib_state == CALIB_IDLE) return;

  // Si estamos en cualquier estado de movimiento, verificamos paradas de emergencia extras
  // pero ya se manejan en el loop principal. Solo nos enfocamos en la lógica de calib.

  switch (g_calib_state) {
    case CALIB_MOVING_RIGHT:
      if (gpio_get_level(EMERGENCY_STOP_GPIO_RIGHT) == 1) {
        set_motor_velocity(-0.25f); // Move Izquierda (Dir 0) en la convención de pwm_gen? 
        // Espera, en execute_movement(..., 0) es Izquierda.
        // set_motor_velocity(v) -> target_freq = meters_to_pulses(v).
        // Si v = -0.25, target_freq es negativo.
        // set_motor_velocity(-v) hace que si v es pos el motor va a la izq?
        // Revisemos pwm_generator.c: 473: int direction = (velocity_ms > 0) ? 1 : 0;
        // Si velocity_ms > 0 -> Dir 1 (Derecha). Si < 0 -> Dir 0 (Izquierda).
        // En execute_movement(..., 0) era Izquierda. OK.
      } else {
        set_motor_velocity(0.0f);
        g_limit_left_pos = g_car_position_pulses;
        ESP_LOGW(TAG, "Límite derecho detectado en: %ld pulsos", g_limit_left_pos);
        vTaskDelay(pdMS_TO_TICKS(200));
        g_calib_state = CALIB_MOVING_LEFT;
      }
      break;

    case CALIB_MOVING_LEFT:
      if (gpio_get_level(EMERGENCY_STOP_GPIO_LEFT) == 1) {
        set_motor_velocity(0.25f); // Move Derecha (Dir 1)
      } else {
        set_motor_velocity(0.0f);
        g_limit_right_pos = g_car_position_pulses;
        ESP_LOGW(TAG, "Límite izquierdo detectado en: %ld pulsos", g_limit_right_pos);
        vTaskDelay(pdMS_TO_TICKS(200));
        g_calib_state = CALIB_MOVING_CENTER;
      }
      break;

    case CALIB_MOVING_CENTER: {
      int32_t travel_range = abs(g_limit_right_pos - g_limit_left_pos);
      g_calibrated_travel_range_pulses = travel_range;
      int32_t center_pos = g_limit_left_pos + (travel_range / 2);
      
      float diff = center_pos - g_car_position_pulses;
      if (fabsf(diff) > 200.0f) {
        float speed = (diff > 0) ? 0.25f : -0.25f;
        set_motor_velocity(speed);
      } else {
        set_motor_velocity(0.0f);
        g_car_position_pulses = 0;
        ESP_LOGW(TAG, "Centro alcanzado. Estabilizando...");
        g_calib_timer = esp_timer_get_time();
        g_calib_state = CALIB_STABILIZING;
      }
      break;
    }

    case CALIB_STABILIZING:
      if (esp_timer_get_time() - g_calib_timer > 500000) { // 500ms
        g_calib_state = CALIB_FINALIZING;
      }
      break;

    case CALIB_FINALIZING: {
      ESP_LOGI(TAG, "Calculando setpoint vertical...");
      int16_t fallen_pos = pulse_counter_get_value();
      int32_t vertical_setpoint_32 = fallen_pos + (ENCODER_RESOLUTION / 2);
      float vertical_setpoint_rad = ((float)vertical_setpoint_32 / ENCODER_RESOLUTION) * 2.0f * 3.14159265f;
      pid_set_absolute_setpoint_rad(vertical_setpoint_rad);
      ESP_LOGW(TAG, "Setpoint vertical pre-calculado: %.3f rad.", vertical_setpoint_rad);
      
      g_lcd_view_state = (lcd_view_state_t)g_actual_view_int;
      g_calib_state = CALIB_IDLE;
      break;
    }

    default: g_calib_state = CALIB_IDLE; break;
  }
}

void button_handler_start_calibration(void) {
  if (is_any_controller_enabled() || g_calib_state != CALIB_IDLE) {
    ESP_LOGE(TAG, "No se puede calibrar ahora.");
    return;
  }

  g_actual_view_int = (int)status_get_lcd_view();
  g_lcd_view_state = VIEW_CALIBRATION;
  g_car_position_pulses = 0;
  g_calib_state = CALIB_MOVING_RIGHT;
  ESP_LOGW(TAG, "--- INICIANDO CALIBRACIÓN (ASYNC) ---");
}

bool button_handler_is_calibrating(void) {
    return g_calib_state != CALIB_IDLE;
}

bool is_any_controller_enabled(void) {
    return pid_is_enabled() || ss_is_enabled() || ss_red_is_enabled() || ss_func_is_enabled() || swing_up_is_active();
}

void control_disable_all(void) {
    pid_disable();
    ss_disable();
    ss_red_disable();
    ss_func_disable();
    swing_up_disable();
}

void control_switch_mode(control_mode_t new_mode) {
    bool was_enabled = is_any_controller_enabled();
    
    if (was_enabled) {
        control_disable_all();
    }
    
    status_set_control_mode(new_mode);
    
    if (was_enabled) {
        switch (new_mode) {
            case MODE_PID: pid_enable(); break;
            case MODE_STATE_SPACE: ss_enable(); break;
            case MODE_STATE_SPACE_RED: ss_red_enable(); break;
            case MODE_STATE_SPACE_FUNC: ss_func_enable(); break;
            case MODE_SWING_UP: swing_up_enable(); break;
        }
    }
}

void control_toggle_current(void) {
    if (is_any_controller_enabled()) {
        control_disable_all();
        ESP_LOGW("CONTROL", "Sistema DESHABILITADO");
    } else {
        // Lógica inteligente: ¿Péndulo arriba o abajo?
        float angle = pulse_counter_get_angle_rad();
        float error = angle - (float)M_PI;
        // Normalización a [-PI, PI]
        while (error > (float)M_PI) error -= 2.0f * (float)M_PI;
        while (error < -(float)M_PI) error += 2.0f * (float)M_PI;

        // Umbral de 30 grados para considerar que "está arriba"
        const float threshold_rad = 30.0f * (float)M_PI / 180.0f;

        if (fabsf(error) < threshold_rad) {
            control_mode_t mode = status_get_control_mode();
            ESP_LOGW("CONTROL", "Péndulo detectado ARRIBA. Habilitando modo: %s", status_get_control_mode_str());
            switch (mode) {
                case MODE_PID: pid_enable(); break;
                case MODE_STATE_SPACE: ss_enable(); break;
                case MODE_STATE_SPACE_RED: ss_red_enable(); break;
                case MODE_STATE_SPACE_FUNC: ss_func_enable(); break;
                case MODE_SWING_UP: swing_up_enable(); break;
            }
        } else {
            ESP_LOGW("CONTROL", "Péndulo detectado ABAJO. Iniciando SWING-UP...");
            swing_up_enable();
        }
    }
}