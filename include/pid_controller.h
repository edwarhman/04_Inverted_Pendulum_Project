// src/pid_controller.h
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdbool.h>
#include <stdint.h>

// la posición del carro
extern volatile int32_t g_car_position_pulses;

typedef struct
{
    // Parámetros de configuración
    float kp, ki, kd;
    float dt; // Tiempo de muestreo constante (en segundos)

    // Límites de Saturación
    float out_max;
    float out_min;

    // Estado interno (Memoria)
    float integral;
    float ultimo_error;
} PIDController;

/**
 * @brief Inicializa el controlador PID con los parámetros dados.
 */
void PID_Init(PIDController *pid, float p, float i, float d, float dt, float min, float max);

/**
 * @brief Calcula la salida del controlador PID.
 */
float PID_Compute(PIDController *pid, float objetivo, float medicion_actual);

/**
 * @brief Resetea el estado interno del controlador PID (integral y último error).
 */
void PID_Reset(PIDController *pid);

/**
 * @brief Tarea principal del controlador PID.
 * Se ejecuta a una frecuencia fija para leer el sensor y controlar el motor.
 */
void pid_controller_task(void *arg);
void motor_control_task(void *arg);

/**
 * @brief Habilita o deshabilita el bucle de control del PID.
 * Función segura para ser llamada desde otras tareas (ej. el manejador del botón).
 */
void pid_toggle_enable(void);

/**
 * @brief Establece el valor de la ganancia Proporcional (Kp).
 */
void pid_set_kp(float kp);

/**
 * @brief Establece el valor de la ganancia Integral (Ki). (Para uso futuro)
 */
void pid_set_ki(float ki);

/**
 * @brief Establece el valor de la ganancia Derivativa (Kd). (Para uso futuro)
 */
void pid_set_kd(float kd);

/**
 * @brief Devuelve el punto de consigna (setpoint) actual del controlador.
 * @return El setpoint actual en cuentas del encoder.
 */
int16_t pid_get_setpoint(void);
void pid_set_absolute_setpoint(int16_t new_setpoint);

/**
 * @brief Devuelve si el bucle de control del PID está actualmente habilitado.
 * @return true si está habilitado, false si no.
 */
bool pid_is_enabled(void);

// para parada de emergencia.

void pid_force_disable(void);

// --- AÑADIDO: Funciones para obtener los valores de las ganancias ---
float pid_get_kp(void);
float pid_get_ki(void);
float pid_get_kd(void);

#endif // PID_CONTROLLER_H