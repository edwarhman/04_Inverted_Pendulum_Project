// components/simulink_comms/simulink_comms.c
//
// Telemetry bridge: sends 6 float variables to Simulink over UART0 at a fixed rate.
// Packet format (binary):
//   [0x56 'V'] [float0] [float1] [float2] [float3] [float4] [float5] [0x0A '\n']
//   Header(1) + 6 floats * 4 bytes = 25 bytes total
//
// Variables transmitted (same order as Bluetooth CSV):
//   [0] time_ms       - uint64 cast to float (ms since control enabled)
//   [1] theta         - pendulum angle (rad)
//   [2] x_pos         - cart position (m)
//   [3] u_control     - control action
//   [4] x_dot         - cart velocity (m/s)
//   [5] theta_dot     - angular velocity (rad/s)

#include "simulink_comms.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

// Application-layer includes to read sensor/controller state
#include "pid_controller.h"
#include "pulse_counter.h"
#include "state_space_controller.h"
#include "state_space_reducido.h"
#include "system_status.h"

static const char *TAG = "SIM_COMM";

// ──────────────────────────────────────────────────────────────────────────────
// Private state
// ──────────────────────────────────────────────────────────────────────────────
#define TX_VARS  7          // floats we transmit to Simulink
#define RX_VARS  0          // floats we expect from Simulink (future use)

// Packet: 1 header byte + TX_VARS floats + 1 tail byte
#define PACKET_SIZE (1 + TX_VARS * sizeof(float) + 1)

static uart_port_t s_uart_num  = UART_NUM_0;
static int         s_rate_ms   = 10;        // transmit every 10 ms (100 Hz)

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// ──────────────────────────────────────────────────────────────────────────────
// Public API
// ──────────────────────────────────────────────────────────────────────────────

void simulink_comms_init(uart_port_t uart_num, uint32_t baud_rate,
                         int tx_vars, int rx_vars)
{
    (void)tx_vars; // We always use TX_VARS = 7
    (void)rx_vars; // RX not implemented yet

    s_uart_num = uart_num;

    uart_config_t uart_cfg = {
        .baud_rate  = baud_rate,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(s_uart_num, &uart_cfg);
    uart_set_pin(s_uart_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(s_uart_num, 2048, 2048, 0, NULL, 0);

    ESP_LOGI(TAG, "Simulink UART%d init @ %lu baud. Packet size: %d bytes",
             uart_num, baud_rate, PACKET_SIZE);
}

// ──────────────────────────────────────────────────────────────────────────────
// TX task - sends telemetry packet every s_rate_ms milliseconds
// ──────────────────────────────────────────────────────────────────────────────
static void tx_task(void *pvParameters)
{
    const TickType_t period = pdMS_TO_TICKS(s_rate_ms);
    TickType_t last_wake   = xTaskGetTickCount();

    uint8_t pkt[PACKET_SIZE];
    pkt[0]             = 'V';           // header
    pkt[PACKET_SIZE-1] = '\n';          // tail

    while (1) {
        vTaskDelayUntil(&last_wake, period);

        // ── Read current system state ─────────────────────────────────────
        float vars[TX_VARS];

        if (ss_is_enabled()) {
            // State-space controller active
            vars[0] = (float)pid_get_run_time_ms();
            vars[1] = ss_get_theta();
            vars[2] = ss_get_x_pos();
            vars[3] = ss_get_u_control();
            vars[4] = ss_get_x_dot();
            vars[5] = ss_get_theta_dot_hat();
            vars[6] = status_get_ref_position();
        } else if (ss_red_is_enabled()) {
            // State-space REDUCED controller active
            vars[0] = (float)pid_get_run_time_ms();
            vars[1] = ss_red_get_theta();
            vars[2] = ss_red_get_x_pos();
            vars[3] = ss_red_get_u_control();
            vars[4] = ss_red_get_x_dot();
            vars[5] = ss_red_get_theta_dot_hat();
            vars[6] = status_get_ref_position();
        } else {
            // PID controller (or idle) - now exposes all 6 variables
            vars[0] = (float)pid_get_run_time_ms();
            vars[1] = pulse_counter_get_angle_rad();  // theta (rad)
            vars[2] = pid_get_car_position_m();        // x_pos (m)
            vars[3] = pid_get_acceleration();          // u_control = aceleración (m/s²)
            vars[4] = pid_get_velocity();              // x_dot = velocidad carro (m/s)
            vars[5] = pid_get_angular_velocity();      // theta_dot (rad/s) — diferencia finita
            vars[6] = status_get_ref_position();       // ref_pos (m)
        }


        // ── Pack floats into packet (little-endian, native) ───────────────
        portENTER_CRITICAL(&s_mux);
        memcpy(&pkt[1], vars, TX_VARS * sizeof(float));
        portEXIT_CRITICAL(&s_mux);

        uart_write_bytes(s_uart_num, (const char *)pkt, PACKET_SIZE);
    }
}

// ──────────────────────────────────────────────────────────────────────────────
// Start tasks
// ──────────────────────────────────────────────────────────────────────────────
void simulink_comms_start_tasks(int priority, int core_id, int tx_rate_ms)
{
    s_rate_ms = tx_rate_ms;
    xTaskCreatePinnedToCore(tx_task, "SimTX", 4096, NULL, priority, NULL, core_id);
    ESP_LOGI(TAG, "SimTX task started (core %d, prio %d, %d ms/pkt)",
             core_id, priority, tx_rate_ms);
}

// ──────────────────────────────────────────────────────────────────────────────
// Stub implementations kept for API compatibility (not used in this build)
// ──────────────────────────────────────────────────────────────────────────────
void simulink_comms_set_tx_data(const float *tx_data) { (void)tx_data; }
void simulink_comms_get_rx_data(float       *rx_data) { (void)rx_data; }
