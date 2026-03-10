// src/car_position.c
#include "car_position.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "CAR_POSITION";

static volatile int32_t s_car_position_pulses = 0;
static float s_range_meters = 0.0f;

static int32_t s_last_position_pulses = 0;
static TickType_t s_last_update_tick = 0;
static float s_velocity_mps = 0.0f;

void car_position_init(void) {
    s_car_position_pulses = 0;
    s_range_meters = 0.0f;
    s_last_position_pulses = 0;
    s_last_update_tick = xTaskGetTickCount();
    s_velocity_mps = 0.0f;
    ESP_LOGI(TAG, "Modulo de posicion del carro inicializado");
}

int32_t car_position_get_pulses(void) {
    return s_car_position_pulses;
}

float car_position_get_meters(void) {
    return (float)s_car_position_pulses / CAR_POSITION_PULSES_PER_METER;
}

float car_position_get_centimeters(void) {
    return ((float)s_car_position_pulses / CAR_POSITION_PULSES_PER_METER) * 100.0f;
}

float car_position_get_millimeters(void) {
    return ((float)s_car_position_pulses / CAR_POSITION_PULSES_PER_METER) * 1000.0f;
}

void car_position_add_pulses(int32_t pulses) {
    s_car_position_pulses += pulses;
}

void car_position_subtract_pulses(int32_t pulses) {
    s_car_position_pulses -= pulses;
}

void car_position_reset(void) {
    s_car_position_pulses = 0;
    s_last_position_pulses = 0;
    s_velocity_mps = 0.0f;
    ESP_LOGI(TAG, "Posicion del carro reseteada a 0");
}

void car_position_set_pulses(int32_t pulses) {
    s_car_position_pulses = pulses;
    ESP_LOGI(TAG, "Posicion del carro establecida a %d pulsos", pulses);
}

void car_position_set_home(void) {
    car_position_reset();
    ESP_LOGI(TAG, "Home establecido en posicion actual (0)");
}

float car_position_get_range_meters(void) {
    return s_range_meters;
}

void car_position_set_range_meters(float range_m) {
    s_range_meters = range_m;
    ESP_LOGI(TAG, "Rango del carro establecido a %.3f metros", range_m);
}

float car_position_get_velocity_mps(void) {
    return s_velocity_mps;
}

float car_position_get_velocity_cmps(void) {
    return s_velocity_mps * 100.0f;
}

void car_position_update_velocity(void) {
    TickType_t current_tick = xTaskGetTickCount();
    TickType_t tick_diff = current_tick - s_last_update_tick;
    
    if (tick_diff == 0) {
        return;
    }
    
    float time_s = (float)tick_diff / configTICK_RATE_HZ;
    
    int32_t position_diff = s_car_position_pulses - s_last_position_pulses;
    s_velocity_mps = ((float)position_diff / CAR_POSITION_PULSES_PER_METER) / time_s;
    
    s_last_position_pulses = s_car_position_pulses;
    s_last_update_tick = current_tick;
}
