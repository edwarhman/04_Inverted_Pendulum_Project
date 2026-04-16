#ifndef BUTTON_HANDLER_H
#define BUTTON_HANDLER_H

#include <stdbool.h>

/**
 * @brief Tarea que inicializa y monitorea el botón BOOT (GPIO 0).
 * 
 * Al detectar una pulsación, envía un comando para que se repita
 * la última acción de movimiento.
 * @param arg Argumentos de la tarea (no se usan).
 */
void button_handler_task(void *arg);

/**
 * @brief Inicia la rutina de calibración (homing) del sistema.
 * Puede ser llamada desde la tarea de botones o desde comandos externos (Bluetooth/UART).
 */
void button_handler_start_calibration(void);

#include "system_status.h"
bool is_any_controller_enabled(void);
void control_switch_mode(control_mode_t new_mode);
void control_toggle_current(void);

#endif // BUTTON_HANDLER_H