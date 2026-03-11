#ifndef CAR_POSITION_H
#define CAR_POSITION_H

#include <stdint.h>
#include <stdbool.h>

#define CAR_POSITION_PULSES_PER_METER  37200
#define CAR_POSITION_MM_PER_REV        12

/**
 * @brief Inicializa el módulo de posición del carro.
 */
void car_position_init(void);

/**
 * @brief Obtiene la posición actual en pulsos del encoder.
 * @return Posición en pulsos (cruda).
 */
int32_t car_position_get_pulses(void);

/**
 * @brief Obtiene la posición actual en metros.
 * @return Posición en metros.
 */
float car_position_get_meters(void);

/**
 * @brief Obtiene la posición actual en centímetros.
 * @return Posición en centímetros.
 */
float car_position_get_centimeters(void);

/**
 * @brief Obtiene la posición actual en milímetros.
 * @return Posición en milímetros.
 */
float car_position_get_millimeters(void);

/**
 * @brief Agrega pulsos a la posición actual (movimiento hacia adelante).
 * @param pulses Cantidad de pulsos a agregar.
 */
void car_position_add_pulses(int32_t pulses);

/**
 * @brief Resta pulsos a la posición actual (movimiento hacia atrás).
 * @param pulses Cantidad de pulsos a restar.
 */
void car_position_subtract_pulses(int32_t pulses);

/**
 * @brief Resetea la posición a cero.
 */
void car_position_reset(void);

/**
 * @brief Establece una posición absoluta en pulsos.
 * @param pulses Posición absoluta en pulsos.
 */
void car_position_set_pulses(int32_t pulses);

/**
 * @brief Establece la posición actual como home (resetea a cero).
 */
void car_position_set_home(void);

/**
 * @brief Establece el rango máximo de recorrido del carro.
 * 
 * Representa la distancia total entre los dos finales de carrera,
 * es decir, el recorrido máximo que puede desplazarse el carro.
 * 
 * @param max_range_meters Rango máximo en metros.
 */
void car_position_set_max_range_meters(float max_range_meters);

/**
 * @brief Obtiene el rango máximo de recorrido del carro.
 * @return Rango máximo en metros.
 */
float car_position_get_max_range_meters(void);

/**
 * @brief Obtiene la velocidad actual del carro.
 * @return Velocidad en metros por segundo.
 */
float car_position_get_velocity_mps(void);

/**
 * @brief Obtiene la velocidad actual del carro.
 * @return Velocidad en centímetros por segundo.
 */
float car_position_get_velocity_cmps(void);

#endif
