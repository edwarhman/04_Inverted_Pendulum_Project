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
#include "uart_echo.h" // Incluimos para acceder a la cola y la estructura de comando
#include <stdio.h>

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
#define PID_LONG_PRESS_STEP 10.0f // Paso para pulsación larga (3 pasos)
#define PID_LONG_PRESS_THRESHOLD                                               \
  20 // Umbral para detectar pulsación larga (20 * 10ms = 200ms)
#define PID_PRESS_DURATION_MAX                                                 \
  30 // Duración máxima de verificación (30 * 10ms = 300ms)

static const char *TAG = "BUTTON_HANDLER";
// Para saber si el PID está activo, llamaremos a una función.
// extern bool g_pid_enabled;

// --- Contador de posición del carro en micropasos ---
// static int32_t g_car_position_pulses = 0;

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
  int last_stop_button_state = 1;
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
    // --- AÑADIDO: Lógica para la Parada de Emergencia (máxima prioridad) ---
    int current_stop_button_state = gpio_get_level(EMERGENCY_STOP_GPIO_RIGHT);
    if (last_stop_button_state == 1 && current_stop_button_state == 0) {

      if (gpio_get_level(EMERGENCY_STOP_GPIO_RIGHT) == 0) {
        // Llama a la función que solo deshabilita
        pid_force_disable();
      }
      // vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
    }
    last_stop_button_state = current_stop_button_state;

    int current_stop_button_state_new =
        gpio_get_level(EMERGENCY_STOP_GPIO_LEFT);
    if (last_stop_button_state == 1 && current_stop_button_state_new == 0) {

      if (gpio_get_level(EMERGENCY_STOP_GPIO_LEFT) == 0) {
        // Llama a la función que solo deshabilita
        pid_force_disable();
      }
    }
    last_stop_button_state = current_stop_button_state_new;

    int current_button_state = gpio_get_level(ENABLE_PID_BUTTON_GPIO);

    // Detectar el flanco de bajada (cuando se presiona el botón)
    if (last_button_state == 1 && current_button_state == 0) {
      // Anti-rebote: esperar un poco y confirmar
      vTaskDelay(pdMS_TO_TICKS(50));
      if (gpio_get_level(ENABLE_PID_BUTTON_GPIO) == 0 &&
          gpio_get_level(EMERGENCY_STOP_GPIO_LEFT) == 1 &&
          gpio_get_level(EMERGENCY_STOP_GPIO_RIGHT) == 1) {
        pid_toggle_enable();
      }
    }
    last_button_state = current_button_state;

    // Solo permitimos el movimiento manual si el PID está explícitamente y NO
    // estamos en la vista de sintonización deshabilitado
    if (!pid_is_enabled() && status_get_lcd_view() != VIEW_PID_GAINS) {

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
      }
    }

    vTaskDelay(pdMS_TO_TICKS(20)); // Sondeo mayor o igual a 10ms 10ms
  }
}

void button_handler_start_calibration(void) {
  if (pid_is_enabled()) {
      ESP_LOGE(TAG, "No se puede calibrar con el PID habilitado.");
      return;
  }

  ESP_LOGW(TAG, "--- INICIANDO RUTINA DE CALIBRACIÓN DE LÍMITES ---");

  // guardamos vista actual
  int actual_view_int = (int)status_get_lcd_view();
  g_lcd_view_state = VIEW_CALIBRATION;

  int32_t limit_left_pos, limit_right_pos;
  g_car_position_pulses = 0;

  // 1. Mover a la izquierda hasta que el final de carrera se active
  ESP_LOGI(TAG, "Buscando límite derecho (GPIO %d)...",
           EMERGENCY_STOP_GPIO_RIGHT);
  // Leemos el estado del pin directamente. El bucle continúa MIENTRAS
  // el botón NO esté presionado.
  while (gpio_get_level(EMERGENCY_STOP_GPIO_RIGHT) == 1) {
    int pulses_moved =
        execute_movement(JOG_PULSES, CALIBRATION_SPEED_HZ,
                         0); // Dir 0 = Izquierda
    g_car_position_pulses -= pulses_moved;
  }
  // --- El bucle se rompe en cuanto el pin se va a BAJO ---
  limit_left_pos = g_car_position_pulses;
  ESP_LOGW(TAG, "Límite derecho detectado en: %ld pulsos", limit_left_pos);
  vTaskDelay(pdMS_TO_TICKS(200)); // Pausa para estabilizar

  // 2. Mover a la derecha hasta que el final de carrera se active
  ESP_LOGI(TAG, "Buscando límite izquierdo...");
  while (gpio_get_level(EMERGENCY_STOP_GPIO_LEFT) == 1) {
    int pulses_moved =
        execute_movement(JOG_PULSES, CALIBRATION_SPEED_HZ,
                         1); // Dir 1 = Derecha
    g_car_position_pulses += pulses_moved;
  }
  limit_right_pos = g_car_position_pulses;
  ESP_LOGW(TAG, "Límite izquierdo detectado en: %ld pulsos", limit_right_pos);
  vTaskDelay(pdMS_TO_TICKS(200));

  // 3. Calcular el centro y mover el carro
  int32_t travel_range = abs(limit_right_pos - limit_left_pos);
  int32_t center_pos = limit_left_pos + (travel_range / 2);
  ESP_LOGW(TAG, "Recorrido: %ld pulsos. Centro: %ld", travel_range,
           center_pos);

  ESP_LOGI(TAG, "Moviendo al centro...");

  int32_t pulses_to_center = abs(center_pos - g_car_position_pulses);
  int direction_to_center =
      (center_pos > g_car_position_pulses) ? 1 : 0;
  execute_movement(pulses_to_center, JOG_SPEED_HZ, direction_to_center);
  g_car_position_pulses = 0;

  ESP_LOGW(TAG, "--- CALIBRACIÓN FINALIZADA. Posición: %ld ---",
           g_car_position_pulses);
  ESP_LOGI(TAG,
           "Esperando 5 segundos para estabilizar..."); 

  vTaskDelay(pdMS_TO_TICKS(5000));

  // --- AÑADIDO: Cálculo y establecimiento del setpoint vertical ---
  ESP_LOGI(TAG, "Calculando setpoint vertical...");

  // 1. Leer la posición del encoder con el péndulo caído y centrado.
  int16_t fallen_pos = pulse_counter_get_value();
  ESP_LOGI(TAG, "Posición 'caída' detectada: %d", fallen_pos);

  // 2. Calcular la posición vertical (180 grados de diferencia).
  int32_t vertical_setpoint_32 = fallen_pos + (ENCODER_RESOLUTION / 2);
  float vertical_setpoint_rad = ((float)vertical_setpoint_32 / ENCODER_RESOLUTION) * 2.0f * 3.14159265f;

  // 3. Establecer el setpoint calculado en radianes.
  pid_set_absolute_setpoint_rad(vertical_setpoint_rad);

  ESP_LOGW(
      TAG,
      "Setpoint vertical pre-calculado: %.3f rad. El sistema está listo.",
      vertical_setpoint_rad);
  ESP_LOGW(TAG,
           "Levante el péndulo y presione el botón de habilitar PID.");

  // devolvemos a la vista actual antes de la calibracion
  g_lcd_view_state = (lcd_view_state_t)actual_view_int;
}