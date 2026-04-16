// src/system_status.c

#include "system_status.h"

// La variable de estado. 'volatile' es importante para que el compilador
// siempre la lea de la memoria, ya que puede ser modificada por otra tarea.
static volatile manual_move_state_t g_manual_move_state = MANUAL_MOVE_NONE;

void status_set_manual_move_state(manual_move_state_t state)
{
    g_manual_move_state = state;
}

manual_move_state_t status_get_manual_move_state(void)
{
    return g_manual_move_state;
}

// --- AÑADIDO: Lógica para el Estado de Modo de Control ---
static volatile control_mode_t g_control_mode = MODE_PID;

void status_set_control_mode(control_mode_t mode)
{
    g_control_mode = mode;
}

control_mode_t status_get_control_mode(void)
{
    return g_control_mode;
}

const char* status_get_control_mode_str(void)
{
    switch (g_control_mode)
    {
        case MODE_PID: return "PID";
        case MODE_STATE_SPACE: return "IDEN";
        case MODE_STATE_SPACE_RED: return "REDU";
        case MODE_STATE_SPACE_FUNC: return "FUNC";
        case MODE_SWING_UP: return "SWNG";
        default: return "UKNOWN";
    }
}

// --- AÑADIDO: Lógica para la Selección de Barra del Péndulo ---
static volatile pendulum_rod_t g_pendulum_rod = ROD_LONG;

void status_set_pendulum_rod(pendulum_rod_t rod)
{
    g_pendulum_rod = rod;
}

pendulum_rod_t status_get_pendulum_rod(void)
{
    return g_pendulum_rod;
}



// --- AÑADIDO: Lógica para el estado de la vista de la pantalla ---
volatile lcd_view_state_t g_lcd_view_state = VIEW_MAIN_STATUS;

void status_cycle_lcd_view(void)
{
    // Convertimos el enum a un entero para hacer la suma
    int next_view_int = (int)g_lcd_view_state + 1;

    // Si hemos pasado la última vista, volvemos a la primera (0)
    if (next_view_int >= VIEW_COUNT)
    {
        next_view_int = 0;
    }

    // Convertimos de nuevo al tipo enum y lo guardamos
    g_lcd_view_state = (lcd_view_state_t)next_view_int;

    // Resetear la selección de parámetro al cambiar de vista
    g_pid_param_select = SELECT_NONE;
}

lcd_view_state_t status_get_lcd_view(void)
{
    return g_lcd_view_state;
}

volatile pid_param_select_t g_pid_param_select = SELECT_NONE;

void status_cycle_pid_param(void)
{
    int next_param_int = (int)g_pid_param_select + 1;
    if (next_param_int >= SELECT_COUNT)
    {
        next_param_int = 0; // Vuelve a SELECT_NONE
    }
    g_pid_param_select = (pid_param_select_t)next_param_int;
}

pid_param_select_t status_get_pid_param(void)
{
    return g_pid_param_select;
}

// --- AÑADIDO: Lógica para la posición de referencia compartida ---
static volatile float g_ref_posicion = 0.1f;

float status_get_ref_position(void)
{
    return g_ref_posicion;
}

void status_set_ref_position(float ref)
{
    g_ref_posicion = ref;
}