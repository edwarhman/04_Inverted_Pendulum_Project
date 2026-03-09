// src/pulse_counter.h
#ifndef PULSE_COUNTER_H
#define PULSE_COUNTER_H

#include <stdint.h>
#include <math.h>

#define ENCODER_RESOLUTION 4096 // 1024 pulsos/rev * 4

/**
 * @brief Obtiene el valor actual del contador del encoder.
 * 
 * @return Valor del contador (cuentas).
 */
int16_t pulse_counter_get_value(void);

/**
 * @brief Obtiene el ángulo actual del encoder en grados.
 * 
 * @return Ángulo en grados (float). 0° = punto de equilibrio.
 */
float pulse_counter_get_angle_degrees(void);

/**
 * @brief Obtiene el ángulo actual del encoder en radianes.
 * 
 * @return Ángulo en radianes (float). 0 = punto de equilibrio.
 */
float pulse_counter_get_angle_radians(void);

/**
 * @brief Inicializa el hardware del contador de pulsos (PCNT y GPIOs).
 * 
 * Debe ser llamada una sola vez al inicio del programa.
 */
void pulse_counter_init(void);

#endif