#include "lcd_controller.h"
#include <stdio.h>   // Para vsnprintf
#include <stdarg.h>  // Para va_list y macros relacionadas
#include <string.h>  // Necesario para memset
#include "hd44780.h" // La nueva librería tiene el mismo nombre de cabecera
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "pid_controller.h"
#include "pulse_counter.h"
#include "system_status.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "state_space_controller.h" // AÑADIDO para el estado SS
#include "state_space_reducido.h"   // AÑADIDO para SS_RED
#include "state_space_funcional.h" // AÑADIDO para SS_FUNC

// --- CONFIGURACIÓN DEL BUS I2C Y PANTALLA ---
#define I2C_MASTER_PORT I2C_NUM_0      // Puerto I2C a usar (0 o 1)
#define I2C_MASTER_SDA_PIN GPIO_NUM_21 // Pin GPIO para SDA
#define I2C_MASTER_SCL_PIN GPIO_NUM_22 // Pin GPIO para SCL
#define I2C_MASTER_FREQ_HZ 100000      // Frecuencia del bus (100kHz es estándar)
#define LCD_ADDR 0x27                  // Dirección I2C del módulo. O Puede ser 0x3F.

// Etiqueta para los logs
static const char *TAG = "LCD_CONTROLLER_I2C";

// Variable estática para el descriptor del dispositivo LCD
static hd44780_t lcd_handle;

// --- FUNCIÓN CALLBACK: El Puente entre hd44780 y I2C ---
// La librería hd44780 no sabe hablar I2C. Nosotros le "enseñamos" cómo
// enviar un byte usando esta función.
static esp_err_t write_lcd_data_i2c(const hd44780_t *lcd, uint8_t data)
{
    // Prepara una transacción I2C
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // Envía la dirección del esclavo (la pantalla) con el bit de escritura
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);
    // Envía el byte de datos. La librería hd44780 ya ha preparado este byte
    // con el formato correcto para el expansor PCF8574.
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);

    // Ejecuta la transacción y espera a que termine
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return ret;
}

// --- FUNCIÓN DE INICIALIZACIÓN (Ahora para I2C) ---
void lcd_init(void)
{
    // 1. Inicializar el hardware del bus I2C del ESP32
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_PIN,
        .scl_io_num = I2C_MASTER_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_PORT, &i2c_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_PORT, I2C_MODE_MASTER, 0, 0, 0));

    ESP_LOGI(TAG, "Driver I2C inicializado. Buscando LCD en 0x%X...", LCD_ADDR);

    // 2. Limpiar y configurar la estructura de la librería para modo I2C
    memset(&lcd_handle, 0, sizeof(hd44780_t));

    // --- ¡LA MAGIA ESTÁ AQUÍ! ---
    // a. Le decimos a la librería que use nuestra función de escritura I2C
    lcd_handle.write_cb = write_lcd_data_i2c;
    // b. Mapeamos los pines lógicos del LCD a los bits del expansor PCF8574
    lcd_handle.pins.rs = 0; // El pin RS está conectado al bit P0 del PCF8574
    lcd_handle.pins.e = 2;  // El pin E está conectado al bit P2
    lcd_handle.pins.d4 = 4; // El pin D4 está conectado al bit P4
    lcd_handle.pins.d5 = 5;
    lcd_handle.pins.d6 = 6;
    lcd_handle.pins.d7 = 7;
    lcd_handle.pins.bl = 3; // El backlight está conectado al bit P3

    // c. Configuramos las propiedades de la pantalla
    lcd_handle.lines = 2; // Para una pantalla 16x2
    lcd_handle.font = HD44780_FONT_5X8;
    lcd_handle.backlight = true; // Queremos el backlight encendido por defecto

    // 3. Dejamos que la librería se inicialice a sí misma
    // La librería ahora llamará a 'write_lcd_data_i2c' para enviar todos
    // los comandos de inicialización.
    ESP_ERROR_CHECK(hd44780_init(&lcd_handle));

    ESP_LOGI(TAG, "LCD inicializada correctamente vía I2C.");
}

// --- EL RESTO DE FUNCIONES NO NECESITAN CAMBIOS ---
// La belleza de esta arquitectura es que el resto de las funciones
// (clear, gotoxy, puts, printf_line, y la tarea) no necesitan saber
// si la comunicación es por GPIO directo o I2C. Simplemente llaman
// a las funciones de la librería, y la librería usa el 'write_cb'
// que le hemos configurado.

void lcd_clear(void)
{
    // La función para limpiar la pantalla ahora se llama hd44780_clear()
    hd44780_clear(&lcd_handle);
}

void lcd_set_cursor(uint8_t col, uint8_t row)
{
    /* // La función para posicionar el cursor es la misma
    hd44780_set_cursor(&lcd_handle, col, row);
    */
    // --- CORREGIDO: La función se llama hd44780_gotoxy ---
    hd44780_gotoxy(&lcd_handle, col, row);
}

void lcd_write_string(const char *str)
{
    // La función para escribir una cadena ahora se llama hd44780_puts()
    hd44780_puts(&lcd_handle, str);
}

void lcd_printf_line(uint8_t row, const char *format, ...)
{
// --- CORRECCIÓN 2: Ajustar al tamaño real de la pantalla ---
// Definimos el ancho de la pantalla para no usar "números mágicos"
#define LCD_LINE_WIDTH 16 // Cambia a 20 si usas una 20x4

    // El búfer debe ser 1 más grande que el ancho para el terminador nulo '\0'
    char buffer[LCD_LINE_WIDTH + 1];

    // Procesar los argumentos variables
    va_list args;
    va_start(args, format);
    // vsnprintf es seguro y no escribirá más allá del tamaño del búfer
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // Si vsnprintf escribió más caracteres de los que caben, lo truncamos.
    if (len > LCD_LINE_WIDTH)
    {
        len = LCD_LINE_WIDTH;
    }

    // Rellenar el resto del buffer con espacios
    for (int i = len; i < LCD_LINE_WIDTH; i++)
    {
        buffer[i] = ' ';
    }
    // Asegurar el terminador nulo al final
    buffer[LCD_LINE_WIDTH] = '\0';

    // --- CORRECCIÓN 1: Posicionar el cursor correctamente ---
    // La función es lcd_set_cursor(columna, fila)
    lcd_set_cursor(0, row); // Siempre empezamos en la columna 0 de la fila deseada
    lcd_write_string(buffer);
}

// --- Tarea dedicada para actualizar la pantalla (es la pieza clave que faltaba iniciar) ---
void lcd_display_task(void *pvParameters)
{
    // vTaskDelay(pdMS_TO_TICKS(500)); // Dar tiempo a que otros módulos se inicien
    ESP_LOGI("LCD_TASK", "Tarea de visualización iniciada.");

    while (1)
    {
        // 1. Obtener la vista que debemos mostrar
        lcd_view_state_t current_view = status_get_lcd_view();

        // 2. Usar un 'switch' para decidir qué dibujar
        switch (current_view)
        {
        case VIEW_MAIN_STATUS:
        {
            bool is_pid_on = pid_is_enabled();
            bool is_ss_on = ss_is_enabled();
            bool is_ss_red_on = ss_red_is_enabled();
            bool is_ss_func_on = ss_func_is_enabled();
            manual_move_state_t move_state = status_get_manual_move_state();

            // Línea 1: Estado prioritario
            if (move_state == MANUAL_MOVE_LEFT)
            {
                lcd_printf_line(0, "<-- Izquierda");
            }
            else if (move_state == MANUAL_MOVE_RIGHT)
            {
                lcd_printf_line(0, "Derecha -->");
            }
            else
            {
                if (is_pid_on) {
                    lcd_printf_line(0, "PID: ACTIVO");
                } else if (is_ss_on) {
                    lcd_printf_line(0, "LQR: ACTIVO");
                } else if (is_ss_red_on) {
                    lcd_printf_line(0, "LQR_RED: ACTIVO");
                } else if (is_ss_func_on) {
                    lcd_printf_line(0, "LQR_FUNC: ACTIVO");
                } else {
                    lcd_printf_line(0, "CTRL: INACTIVO");
                }
            }

            // Línea 2: Posición en grados
            float degrees = pulse_counter_get_angle_deg();
            float sp_rad = pid_get_dynamic_angle_setpoint_rad();
            float sp_deg = sp_rad * (180.0f / 3.14159265f);
            lcd_printf_line(1, "A:%.0f SP:%.0f*", degrees - 180, sp_deg - 180);
            break;
        }

        case VIEW_POSITION:
        {
            float pos_sp_m = pid_get_position_setpoint();
            float pos_sp_cm = pos_sp_m * 100.0f;
            lcd_printf_line(0, "SP:%.1fcm", pos_sp_cm);
            
            float car_position_cm = pid_get_car_position_cm();
            int32_t car_position_pulses = pid_get_car_position_pulses();
            lcd_printf_line(1, "%.1fcm P:%ld", car_position_cm, (long)car_position_pulses);
            break;
        }

        case VIEW_VELOCITY:
        {
            float vel = pid_get_velocity();
            lcd_printf_line(0, "Velocidad PID:");
            lcd_printf_line(1, "%.2f", vel);
            break;
        }

        case VIEW_PID_GAINS:
        {
            // Obtenemos los valores actuales del módulo PID
            float kp = pid_get_kp();
            float ki = pid_get_ki();
            float kd = pid_get_kd();
            
            // Obtenemos qué parámetro estamos editando actualmente
            pid_param_select_t selected_param = status_get_pid_param();
            
            // Efecto de parpadeo (500ms on, 500ms off)
            bool blink_show = (xTaskGetTickCount() * portTICK_PERIOD_MS) % 1000 < 500;
            
            // Renderizamos Kp (Línea 0)
            if (selected_param == SELECT_KP && !blink_show) {
                // Si el Kp está seleccionado y toca "apagar", mostramos espacios
                // Asumimos un máximo de 5 caracteres para "Kp:XX.X" para simplificar y alinear después con " Ki:"
                char temp_line[21];
                snprintf(temp_line, sizeof(temp_line), "Kp:     Ki:%.2f", ki);
                lcd_printf_line(0, temp_line);
            } else if (selected_param == SELECT_KI && !blink_show) {
                // Si el Ki está seleccionado y toca apagar
                char temp_line[21];
                snprintf(temp_line, sizeof(temp_line), "Kp:%.2f Ki:    ", kp);
                lcd_printf_line(0, temp_line);
            } else {
                // Normal
                lcd_printf_line(0, "Kp:%.2f Ki:%.2f", kp, ki);
            }
            
            // Renderizamos Kd (Línea 1)
            if (selected_param == SELECT_KD && !blink_show) {
                lcd_printf_line(1, "Kd:     ");
            } else {
                lcd_printf_line(1, "Kd: %.2f", kd);
            }
            break;
        }

        case VIEW_CALIBRATION:
        {
            lcd_printf_line(0, "Calibrando...");
            lcd_printf_line(1, "Espere");
            break;
        }

        case VIEW_CONTROL_MODE:
        {
            lcd_printf_line(0, "Modo de Control:");
            control_mode_t mode = status_get_control_mode();
            if (mode == MODE_PID) {
                lcd_printf_line(1, "< PID >");
            } else if (mode == MODE_STATE_SPACE) {
                lcd_printf_line(1, "< Est(LQR Id) >");
            } else if (mode == MODE_STATE_SPACE_RED) {
                lcd_printf_line(1, "< Est(LQR Red) >");
            } else {
                lcd_printf_line(1, "< Est(LQR Fun) >");
            }
            break;
        }

        case VIEW_ROD_SELECTION:
        {
            lcd_printf_line(0, "Barra fisica:");
            pendulum_rod_t active_rod = status_get_pendulum_rod();
            if (active_rod == ROD_LONG) {
                lcd_printf_line(1, "< Larga >");
            } else {
                lcd_printf_line(1, "< Corta >");
            }
            break;
        }

        default:
            // Vista por defecto en caso de error
            lcd_printf_line(0, "Vista Invalida");
            lcd_printf_line(1, "");
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

