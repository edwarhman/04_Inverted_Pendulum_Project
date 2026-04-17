// src/pulse_counter.c
#include "pulse_counter.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/pulse_cnt.h" // NUEVO: API de Pulse Counter moderna
#include "driver/gpio.h"
#include "esp_log.h"

// --- CONFIGURACIÓN DE PINES (Tus pines específicos) ---
#define PCNT_INPUT_A_PIN    14 // Fase A del encoder
#define PCNT_INPUT_B_PIN    12 // Fase B del encoder
#define ENCODER_INDEX_Z_PIN 13 // Fase Z (índice) del encoder

static pcnt_unit_handle_t pcnt_unit = NULL;

/**
 * @brief ISR para la señal Z. Resetea el contador del PCNT a cero.
 */
static void IRAM_ATTR encoder_index_z_isr_handler(void* arg) {
    if (pcnt_unit) {
        pcnt_unit_clear_count(pcnt_unit);
    }
}

/**
 * @brief Inicializa el PCNT en modo cuadratura y la interrupción de la señal Z.
 */
void pulse_counter_init(void) {
    ESP_LOGI("PULSE_COUNTER", "Inicializando PCNT con la nueva API pulse_cnt.h...");

    // 1. Configuración de la unidad PCNT
    pcnt_unit_config_t unit_config = {
        .high_limit = 32767,
        .low_limit = -32768,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    // 2. Configuración del filtro de ruido (glitch filter)
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    // 3. Configuración de Canales (Modo Cuadratura x4)
    
    // Canal A
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = PCNT_INPUT_A_PIN,
        .level_gpio_num = PCNT_INPUT_B_PIN,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));

    // Canal B
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = PCNT_INPUT_B_PIN,
        .level_gpio_num = PCNT_INPUT_A_PIN,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    // 4. Establecer acciones para los canales (Lógica de cuadratura)
    // Canal A: Reacciona a flancos de A según el nivel de B
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // Canal B: Reacciona a flancos de B según el nivel de A
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    // 5. Instalar pull-ups para los pines (opcional si ya tienen externos, pero recomendado)
    gpio_set_pull_mode(PCNT_INPUT_A_PIN, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PCNT_INPUT_B_PIN, GPIO_PULLUP_ONLY);

    // 6. Configurar la interrupción para la señal Z (GPIO 13)
    gpio_config_t z_pin_config = {
        .pin_bit_mask = (1ULL << ENCODER_INDEX_Z_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&z_pin_config);

    // Notar: gpio_install_isr_service(0) se suele llamar una sola vez en el sistema.
    // Lo dejamos aquí por compatibilidad si es el primer uso de GPIO ISR.
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ENCODER_INDEX_Z_PIN, encoder_index_z_isr_handler, NULL);

    // 7. Habilitar e iniciar la unidad
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    ESP_LOGI("PULSE_COUNTER", "PCNT inicializado exitosamente en modo cuadratura.");
}

/**
 * @brief Obtiene el valor actual del contador del PCNT.
 */
int16_t pulse_counter_get_value(void) {
    int count = 0;
    if (pcnt_unit) {
        pcnt_unit_get_count(pcnt_unit, &count);
    }
    return (int16_t)count;
}

// --- Abstracción de Unidades Físicas ---
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

float pulse_counter_get_angle_rad(void) {
    int16_t pulses = pulse_counter_get_value();
    // 4096 pulsos corresponden a 2*PI radianes
    return ((float)pulses / ENCODER_RESOLUTION) * 2.0f * M_PI;
}

float pulse_counter_get_angle_deg(void) {
    int16_t pulses = pulse_counter_get_value();
    // 4096 pulsos corresponden a 360 grados
    return ((float)pulses / ENCODER_RESOLUTION) * 360.0f;
}

int16_t pulse_counter_get_angle_pulses(void) {
    return pulse_counter_get_value();
}

