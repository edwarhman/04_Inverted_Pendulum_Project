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
float car_position_get_range_meters(void);
void car_position_set_range_meters(float range_m);

float car_position_get_velocity_mps(void);
float car_position_get_velocity_cmps(void);

#endif
