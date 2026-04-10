// src/pulse_counter.c
#include "pulse_counter.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define PCNT_INPUT_A_PIN    14
#define PCNT_INPUT_B_PIN    12
#define ENCODER_INDEX_Z_PIN 13

static pcnt_unit_handle_t pcnt_unit = NULL;
static pcnt_channel_handle_t pcnt_chan_a = NULL;
static pcnt_channel_handle_t pcnt_chan_b = NULL;

static void IRAM_ATTR encoder_index_z_isr_handler(void* arg) {
    pcnt_unit_clear_count(pcnt_unit);
}

void pulse_counter_init(void) {
    pcnt_unit_config_t unit_config = {
        .low_limit = -ENCODER_RESOLUTION,
        .high_limit = ENCODER_RESOLUTION,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = PCNT_INPUT_A_PIN,
        .level_gpio_num = PCNT_INPUT_B_PIN,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));

    pcnt_channel_edge_action_t act_edge = PCNT_CHANNEL_EDGE_ACTION_DECREASE;
    pcnt_channel_level_action_t act_level_high = PCNT_CHANNEL_LEVEL_ACTION_KEEP;
    pcnt_channel_level_action_t act_level_low = PCNT_CHANNEL_LEVEL_ACTION_INVERSE;
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, act_edge, act_edge));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, act_level_high, act_level_low));

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = PCNT_INPUT_B_PIN,
        .level_gpio_num = PCNT_INPUT_A_PIN,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, act_level_low, act_level_high));

    gpio_config_t z_pin_config = {
        .pin_bit_mask = (1ULL << ENCODER_INDEX_Z_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&z_pin_config);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(ENCODER_INDEX_Z_PIN, encoder_index_z_isr_handler, NULL);

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

int16_t pulse_counter_get_value(void) {
    int count = 0;
    pcnt_unit_get_count(pcnt_unit, &count);
    return (int16_t)count;
}

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

float pulse_counter_get_angle_rad(void) {
    int16_t pulses = pulse_counter_get_value();
    return ((float)pulses / ENCODER_RESOLUTION) * 2.0f * M_PI;
}

float pulse_counter_get_angle_deg(void) {
    int16_t pulses = pulse_counter_get_value();
    return ((float)pulses / ENCODER_RESOLUTION) * 360.0f;
}

int16_t pulse_counter_get_angle_pulses(void) {
    return pulse_counter_get_value();
}
