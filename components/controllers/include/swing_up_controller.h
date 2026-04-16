#ifndef SWING_UP_CONTROLLER_H
#define SWING_UP_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Tarea principal del controlador de Swing-Up.
 */
void swing_up_task(void *arg);

/**
 * @brief Habilita la rutina de Swing-Up.
 */
void swing_up_enable(void);

/**
 * @brief Deshabilita la rutina de Swing-Up.
 */
void swing_up_disable(void);

/**
 * @brief Indica si la rutina de Swing-Up está activa.
 */
bool swing_up_is_active(void);

/**
 * @brief Parada de emergencia para el swing-up.
 */
void swing_up_force_disable(void);

#endif // SWING_UP_CONTROLLER_H
