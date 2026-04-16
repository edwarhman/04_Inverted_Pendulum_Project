// include/system_status.h

#ifndef SYSTEM_STATUS_H
#define SYSTEM_STATUS_H

#include <stdint.h>

// Definimos los posibles estados de movimiento manual
typedef enum
{
    MANUAL_MOVE_NONE,
    MANUAL_MOVE_LEFT,
    MANUAL_MOVE_RIGHT
} manual_move_state_t;

/**
 * @brief Establece el estado actual del movimiento manual.
 *
 * Esta función es segura para ser llamada desde cualquier tarea (es "thread-safe"
 * porque la operación es atómica en una variable volátil).
 *
 * @param state El nuevo estado de movimiento (NONE, LEFT, o RIGHT).
 */
void status_set_manual_move_state(manual_move_state_t state);

/**
 * @brief Obtiene el estado actual del movimiento manual.
 *
 * @return El estado de movimiento actual.
 */
manual_move_state_t status_get_manual_move_state(void);

// --- AÑADIDO: ESTADO DE LA VISTA DE LA PANTALLA ---
typedef enum
{
    VIEW_MAIN_STATUS,    // Vista principal: Estado PID y Posición en Grados
    VIEW_POSITION, // Vista secundaria: Posición del carro en cm
    VIEW_PID_GAINS,      // Vista terciaria: Valores de las ganancias Kp, Ki, Kd
    VIEW_VELOCITY,       // Cuarta vista: Velocidad
    VIEW_CONTROL_MODE,   // Quinta vista: Selección de modo de control (PID/LQR)
    VIEW_ROD_SELECTION,  // Sexta vista: Selección de barra (Larga/Corta)
    VIEW_COUNT,           // ¡Importante! Siempre al final, para saber cuántas vistas hay
    VIEW_CALIBRATION //++
} lcd_view_state_t;

// --- AÑADIDO: ESTADO DEL MODO DE CONTROL ---
typedef enum
{
    MODE_PID,
    MODE_STATE_SPACE,
    MODE_STATE_SPACE_RED,
    MODE_STATE_SPACE_FUNC
} control_mode_t;

void status_set_control_mode(control_mode_t mode);
control_mode_t status_get_control_mode(void);
const char* status_get_control_mode_str(void);

// --- AÑADIDO: ESTADO DE LA BARRA DEL PÉNDULO ---
typedef enum
{
    ROD_LONG,
    ROD_SHORT
} pendulum_rod_t;

void status_set_pendulum_rod(pendulum_rod_t rod);
pendulum_rod_t status_get_pendulum_rod(void);

// --- AÑADIDO: ESTADO DE SELECCIÓN DE PARÁMETROS PID ---
typedef enum
{
    SELECT_NONE,
    SELECT_KP,
    SELECT_KI,
    SELECT_KD,
    SELECT_COUNT
} pid_param_select_t;

extern volatile lcd_view_state_t g_lcd_view_state;
extern volatile pid_param_select_t g_pid_param_select;

/**
 * @brief Cambia al siguiente parámetro del PID a sintonizar.
 */
void status_cycle_pid_param(void);

/**
 * @brief Obtiene el parámetro del PID actualmente seleccionado.
 */
pid_param_select_t status_get_pid_param(void);

/**
 * @brief Cambia a la siguiente vista de la pantalla.
 *
 * Rota cíclicamente entre las vistas disponibles.
 */
void status_cycle_lcd_view(void);

/**
 * @brief Obtiene la vista actual de la pantalla.
 *
 * @return La vista que debe ser mostrada.
 */
lcd_view_state_t status_get_lcd_view(void);

/**
 * @brief Obtiene la posición de referencia compartida (en metros).
 */
float status_get_ref_position(void);

/**
 * @brief Establece la posición de referencia compartida (en metros).
 */
void status_set_ref_position(float ref);

#endif // SYSTEM_STATUS_H