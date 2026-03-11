#ifndef CAR_POSITION_H
#define CAR_POSITION_H

#include <stdint.h>
#include <stdbool.h>

#define CAR_POSITION_PULSES_PER_METER  37200
#define CAR_POSITION_MM_PER_REV        12

void car_position_init(void);

int32_t car_position_get_pulses(void);
float car_position_get_meters(void);
float car_position_get_centimeters(void);
float car_position_get_millimeters(void);

void car_position_add_pulses(int32_t pulses);
void car_position_subtract_pulses(int32_t pulses);
void car_position_reset(void);
void car_position_set_pulses(int32_t pulses);

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
 * 
 * @return Rango máximo en metros.
 */
float car_position_get_max_range_meters(void);

float car_position_get_velocity_mps(void);
float car_position_get_velocity_cmps(void);

#endif
