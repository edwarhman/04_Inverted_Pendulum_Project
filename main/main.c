// src/main.c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

// 1. Inclusión de todas las cabeceras de los módulos
#include "bluetooth_telemetry.h" // Módulo Bluetooth
#include "button_handler.h"      // Para la tarea de lectura del botón
#include "freertos/queue.h"
#include "lcd_controller.h" // ¡Solo incluimos nuestro módulo!
#include "nvs_flash.h"      // Para la memoria no volátil
#include "pid_controller.h"
#include "pulse_counter.h"  // Para la tarea de lectura del encoder
#include "pwm_generator.h"  // Para la tarea de control del motor
#include "simulink_comms.h" // Telemetría binaria hacia Simulink por UART0
#include "state_space_controller.h"
#include "state_space_reducido.h"
#include "state_space_funcional.h"
#include "swing_up_controller.h" // AÑADIDO
#include "system_status.h"


#define USE_STATE_SPACE_CONTROLLER 1

/*typedef struct
{
  int num_pulses;
  int frequency;
  int direction;
} motor_command_t;*/

QueueHandle_t motor_command_queue;

void app_main(void) {
  // Inicialización de NVS (necesario para Bluetooth)
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  pwm_init();           // inicializa y configura pines del driver
  pulse_counter_init(); // inicializa y configura pines del encoder
  lcd_init();           // Inicializar la pantalla
  bluetooth_telemetry_init(); // Inicializar servicio de telemetría Bluetooth

  // Mensaje de bienvenida en la pantalla
  lcd_clear();
  lcd_printf_line(0, "Bienvenidos...");
  lcd_printf_line(1, "Iniciando");
  vTaskDelay(pdMS_TO_TICKS(1000));

  // Creacion de la cola de tareas
  motor_command_queue = xQueueCreate(1, sizeof(motor_command_t));
  if (motor_command_queue == NULL) {
    ESP_LOGE("MAIN", "Error al crear la cola del motor.");
    return;
  }

  // --------- Cada tarea se ejecutará de forma independiente y concurrente.
  // ---------

  // Crear la tarea del controlador (prioridad mas alta)
  xTaskCreate(state_space_controller_task, "StateSpace",
              configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);
  xTaskCreate(state_space_reducido_task, "StateSpaceRed",
              configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);
  xTaskCreate(state_space_funcional_task, "StateSpaceFunc",
              configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);
  xTaskCreate(pid_controller_task, "PID_Controller",
              configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);

  // Crear la tarea del control del motor
  xTaskCreate(motor_control_task, "Motor_Control", configMINIMAL_STACK_SIZE * 3,
              NULL, 4, NULL);

  // Inicializar telemetría Simulink (UART0, 115200 baud, 6 variables TX, 0 RX)
  // NOTA: Los logs de ESP_LOG están desactivados para mantener el stream
  // binario limpio.
  // simulink_comms_init(UART_NUM_0, 115200, 6, 0);
  // simulink_comms_start_tasks(3, tskNO_AFFINITY,
  //                            10); // Prioridad 3, 10ms por paquete (100 Hz)

  // Tarea que inicializa el PCNT y reporta la posición del encoder para
  // depuración xTaskCreate(pulse_counter_task, "pulse_counter_task",
  // configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);

  // Tarea que monitorea el botón BOOT y envía comandos de "repetir"
  xTaskCreate(button_handler_task, "button_handler_task",
              configMINIMAL_STACK_SIZE * 3, NULL, 4, NULL);

  // TAREA DEL SWING-UP (Prioridad alta para control en tiempo real)
  xTaskCreate(swing_up_task, "SwingUp_Controller",
              configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);

  // TAREA DE LA PANTALLA (Prioridad baja, no es crítica)
  xTaskCreate(lcd_display_task, "LCDDisplay", 3072, NULL, 3, NULL);

  ESP_LOGI("MAIN", "Arranque del sistema completado.");
}
