// src/pid_controller.c
#include "pid_controller.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pulse_counter.h" // Para leer la posición del encoder
#include "pwm_generator.h" // Para la función que mueve el motor
#include "uart_echo.h"
#include <math.h> // Para la función de valor absoluto fabs()
#include <stdint.h>
#include <stdio.h>

/************************************************************************************
 *                                                                                  *
 *                   CONFIGURACIÓN DE PARÁMETROS DEL CONTROLADOR     *
 *                                                                                  *
 ************************************************************************************/

// --- PARÁMETROS DE TEMPORIZACIÓN ---
// Frecuencia a la que se ejecutará el bucle de control. 50 Hz es un buen punto
// de partida para sistemas mecánicos. Un valor más bajo es menos reactivo, uno
// más alto consume más CPU y puede ser más sensible al ruido.
#define PID_LOOP_PERIOD_MS 10 // Frecuencia del bucle: 1000ms / 20ms = 50 Hz

// --- PARÁMETROS DE CONTROL ---
// El punto de equilibrio deseado. Gracias a la señal Z del encoder que resetea
// el contador, nuestro punto de equilibrio es siempre CERO.
// #define SETPOINT //   0 Este es para colocar el péndulo en 0  grados para
// dejarlo colgado en este caso queremos que empiece en la posición colocada

// Banda muerta (Dead Band). Si el error (en cuentas del encoder) es menor que
// este valor, lo consideramos cero. Esto es CRUCIAL para evitar que el motor
// vibre o "tiemble" constantemente tratando de corregir errores minúsculos.
#define DEAD_BAND_ANGLE                                                        \
  1.5f // 1.0f//1.6f // con factor de crecimiento de 0.088° o 0.1° 1.76

#define DEAD_BAND_X_CM 3 // 5

// Evita que el término integral crezca indefinidamente y desestabilice el
// sistema. Este valor debe ser menor o igual a MAX_OUTPUT_PULSES.
#define MAX_INTEGRAL 9000.0f

// --- PARÁMETROS DEL ACTUADOR (MOTOR) ---
// Límite máximo de pulsos que el PID puede ordenar en una sola corrección.
// Sirve como medida de seguridad para evitar que una reacción brusca del PID
// genere un movimiento demasiado violento.
#define MAX_OUTPUT_PULSES 10000 // Aumentado de 1700 a 3000 para mayor salida

// Frecuencia base (velocidad mínima) para los movimientos de corrección.
#define BASE_FREQUENCY                                                         \
  2000 // Aumentado de 1000 a 2000 para que arranque más rápido

// Factor de escalado de velocidad. Hace que la corrección sea más rápida
// para errores grandes. La frecuencia final será:
// Frecuencia = BASE_FREQUENCY + (Error * FREQ_PER_ERROR_PULSE)
#define FREQ_PER_ERROR_PULSE                                                   \
  250 // Aumentado de 80 a 250 para que acelere más rápido

// Ganancia Proporcional para la posición del carro. Convierte el error de
// posición en un pequeño ángulo de inclinación deseado (en cuentas del
// encoder). Este es tu nuevo "dial" para controlar qué tan rápido vuelve el
// carro al centro.
#define POSITION_CONTROL_GAIN 0.0003f // 0.001//0.0005f

// Límite máximo para el offset del setpoint. Evita que pida ángulos demasiado
// grandes.
#define MAX_SETPOINT_OFFSET 20 // 25 // Máximo offset de 20 cuentas

// máxima frecuencia
#define MAX_FRECUENCY_LIMIT 150000

// --- CONSTANTES FÍSICAS PARA CONVERSIÓN DE VELOCIDAD ---
// Estas constantes permiten convertir velocidad lineal (m/s) a frecuencia del
// motor
#define MOTOR_STEPS_PER_REV                                                    \
  200.0f // Pasos por revolución del motor (sin microstepping)
#define MICROSTEPPING_FACTOR 16.0f // Factor de microstepping (ej: 1/16)
#define PULLEY_RADIUS_MM 10.0f     // Radio de la polea en mm
#define TRANSMISSION_RATIO 1.0f    // Relación de transmisión (correa/polea)

/************************************************************************************
 *                        FIN DE LA CONFIGURACIÓN DE PARÁMETROS     *
 ************************************************************************************/

// Calculamos el tiempo del ciclo en segundos (dt) una sola vez.
// static const float PID_LOOP_PERIOD_S = PID_LOOP_PERIOD_MS / 1000.0f;
static const char *TAG = "PID_CONTROLLER";
float const loop_period_in_seconds = PID_LOOP_PERIOD_MS / 1000.0f;

// Variables de estado globales para el controlador
// para 3200pulse/rev kp=5, ki=1, kd=10, para 2000pulse/rev kp=70, ki=1, kd=10,
// para 10000pulse/rev kp=41, ki=0.4, kd=70
static volatile bool g_pid_enabled = false;
static PIDController
    g_angle_controller; // Controlador del ángulo (PID completo)
static PIDController g_position_controller; // Controlador de posición (solo P)
static PIDController g_velocity_integrator; // Integrador: convierte aceleración
                                            // a velocidad (solo I)

// --- Variable para el angulo del pendulo ---
static volatile int16_t g_absolute_setpoint =
    0; // Se inicializa en 0 por defecto

// --- Variable para la posición del carro ---
volatile int32_t g_car_position_pulses = 0; // pwm_generator puede actualizarla

// --- AÑADIDO: Implementación de las nuevas funciones ---
float pid_get_kp(void) { return g_angle_controller.kp; }
float pid_get_ki(void) { return g_angle_controller.ki; }
float pid_get_kd(void) { return g_angle_controller.kd; }

// --- Implementación de funciones públicas ---

// --- establecer el setpoint externamente ---
void pid_set_absolute_setpoint(int16_t new_setpoint) {
  g_absolute_setpoint = new_setpoint;
  ESP_LOGW(TAG, "Setpoint absoluto establecido en: %d", g_absolute_setpoint);
}

void pid_toggle_enable(void) {
  g_pid_enabled = !g_pid_enabled;
  if (g_pid_enabled) {
    // --- LÓGICA PARA ESTABLECER EL SETPOINT DINÁMICO ---
    // 1. Leer la posición actual del encoder en el momento de la habilitación.
    // int16_t current_position = pulse_counter_get_value();
    // 2. Establecer esa posición como nuestro nuevo punto de equilibrio.
    // g_absolute_setpoint = current_position;

    // Inicializar el controlador PID si no está inicializado
    if (g_angle_controller.dt == 0.0f) {
      PID_Init(&g_angle_controller, 400.0f, 0.4f, 300.0f,
               PID_LOOP_PERIOD_MS / 1000.0f, -MAX_OUTPUT_PULSES,
               MAX_OUTPUT_PULSES);
    } else {
      // Resetear el estado interno
      PID_Reset(&g_angle_controller);
    }

    // Resetear también el controlador de posición
    PID_Reset(&g_position_controller);

    // Resetear el integrador de velocidad
    PID_Reset(&g_velocity_integrator);

    ESP_LOGW(TAG, "Control PID HABILITADO");
  } else {
    motor_command_t stop_cmd = {
        .num_pulses = 0, .frequency = 0, .direction = 0};
    xQueueOverwrite(motor_command_queue, &stop_cmd);
    ESP_LOGW(TAG, "Control PID DESHABILITADO");
  }
}

void pid_set_kp(float kp) {
  g_angle_controller.kp = kp;
  ESP_LOGI(TAG, "Kp actualizado a: %f", g_angle_controller.kp);
}
void pid_set_ki(float ki) {
  g_angle_controller.ki = ki;
  ESP_LOGI(TAG, "Ki actualizado a: %f", g_angle_controller.ki);
}
void pid_set_kd(float kd) {
  g_angle_controller.kd = kd;
  ESP_LOGI(TAG, "Kd actualizado a: %f", g_angle_controller.kd);
}

// --- AÑADIDO: Implementación de la nueva función 'getter' ---
int16_t pid_get_setpoint(void) { return g_absolute_setpoint; }

// --- AÑADIDO: Implementación de la función de deshabilitación forzada ---
void pid_force_disable(void) {
  if (g_pid_enabled) { // Solo actúa y muestra el mensaje si estaba habilitado
    g_pid_enabled = false;
    // Reseteamos el estado para un futuro arranque limpio
    PID_Reset(&g_angle_controller);
    PID_Reset(&g_position_controller);
    PID_Reset(&g_velocity_integrator);

    motor_command_t stop_cmd = {
        .num_pulses = 0, .frequency = 0, .direction = 0};
    xQueueOverwrite(motor_command_queue, &stop_cmd);

    ESP_LOGE(TAG, "¡PARADA DE EMERGENCIA! Control PID DESHABILITADO.");
  }
}

bool pid_is_enabled(void) { return g_pid_enabled; }

// --- Tarea Principal del Controlador ---

void pid_controller_task(void *arg) {
  TickType_t last_wake_time = xTaskGetTickCount();

  // Inicializar el controlador PID
  PID_Init(&g_angle_controller, 400.0f, 0.4f, 300.0f, loop_period_in_seconds,
           -MAX_OUTPUT_PULSES, MAX_OUTPUT_PULSES);

  // Inicializar el controlador de posición (solo proporcional)
  PID_Init(&g_position_controller, POSITION_CONTROL_GAIN, 0.0f, 0.0f,
           loop_period_in_seconds, -MAX_SETPOINT_OFFSET, MAX_SETPOINT_OFFSET);

  // Inicializar el integrador de velocidad (solo I, ganancia 1)
  // Convierte la aceleración (salida del PID) a velocidad
  PID_Init(&g_velocity_integrator, 0.0f, 1, 0.0f, loop_period_in_seconds,
           -MAX_OUTPUT_PULSES, MAX_OUTPUT_PULSES);

  // Reseteamos el error anterior al habilitar para evitar un pico inicial en D
  pid_toggle_enable(); // Habilita y resetea
  pid_toggle_enable(); // Deshabilita de nuevo, pero el estado está limpio

  while (1) {
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(PID_LOOP_PERIOD_MS));

    if (!g_pid_enabled) {
      PID_Reset(&g_angle_controller);
      PID_Reset(&g_position_controller);
      PID_Reset(&g_velocity_integrator);
      continue;
    }

    // 1. MEDIR estado actual
    int16_t current_angle = pulse_counter_get_value();

    // --- Lógica de control de posición usando PID_Compute ---
    // El setpoint de posición es 0 (centro del riel)
    float position_setpoint = 0.0f;

    // Calcular el offset usando el controlador de posición (solo proporcional)
    float angle_setpoint_offset = PID_Compute(
        &g_position_controller, position_setpoint, g_car_position_pulses);

    // Calcular el setpoint dinámico
    int16_t dynamic_angle_setpoint =
        g_absolute_setpoint + (int16_t)angle_setpoint_offset;

    // 2. CALCULAR ERROR de ángulo usando el setpoint dinámico
    // La salida del PID es aceleración, se integra a velocidad
    float acceleration =
        PID_Compute(&g_angle_controller, dynamic_angle_setpoint, current_angle);

    // Integrador: convierte aceleración a velocidad (PID con solo Ki=1)
    // El "error" para el integrador es la aceleración de salida del PID de
    // ángulo
    float velocity = PID_Compute(&g_velocity_integrator, acceleration, 0.0f);

    // 6. ACTUAR: Establecer la velocidad del motor
    set_motor_velocity(velocity);
  }
}

// --- Implementación de las nuevas funciones del controlador PID ---

void PID_Init(PIDController *pid, float p, float i, float d, float dt,
              float min, float max) {
  pid->kp = p;
  pid->ki = i;
  pid->kd = d;
  pid->dt = dt;

  pid->out_min = min;
  pid->out_max = max;

  // Inicializar memoria en cero
  pid->integral = 0.0f;
  pid->ultimo_error = 0.0f;
}

float PID_Compute(PIDController *pid, float objetivo, float medicion_actual) {
  float error = objetivo - medicion_actual;

  // Proporcional
  float P = pid->kp * error;

  // Derivativo (Usando el dt interno)
  float D = pid->kd * (error - pid->ultimo_error) / pid->dt;

  // Cálculo temporal de la integral
  float nueva_integral = pid->integral + (error * pid->dt);
  float I = pid->ki * nueva_integral;

  float salida_total = P + I + D;
  float salida_saturada = salida_total;

  // Saturación del actuador
  if (salida_saturada > pid->out_max)
    salida_saturada = pid->out_max;
  else if (salida_saturada < pid->out_min)
    salida_saturada = pid->out_min;

  // Anti-Windup: Solo actualizamos la integral si no hay saturación
  if (salida_total == salida_saturada) {
    pid->integral = nueva_integral;
  }

  pid->ultimo_error = error;
  return salida_saturada;
}

/**
 * @brief Resetea el estado interno del controlador PID.
 */
void PID_Reset(PIDController *pid) {
  pid->integral = 0.0f;
  pid->ultimo_error = 0.0f;
}

/**
 * @brief Calcula la frecuencia del motor basada en una velocidad deseada del
 * carro.
 */
int calculate_motor_frequency(float desired_velocity) {
  int frequency =
      BASE_FREQUENCY + (int)(fabs(desired_velocity) * FREQ_PER_ERROR_PULSE);

  if (frequency > MAX_FRECUENCY_LIMIT) {
    frequency = MAX_FRECUENCY_LIMIT;
  }

  return frequency;
}

/**
 * @brief Calcula la frecuencia del motor para lograr una velocidad lineal
 * específica del carro.
 *
 * Conversión física:
 * 1. La velocidad lineal del carro depende del radio de la polea
 * 2. La frecuencia del motor determina la velocidad de rotación
 * 3. Fórmula: f = (v_linear * steps_per_rev_effective) / (2π * radius)
 */
int velocity_to_motor_frequency(float linear_velocity) {
  // Calcular pasos efectivos por revolución (considerando microstepping)
  float effective_steps_per_rev = MOTOR_STEPS_PER_REV * MICROSTEPPING_FACTOR;

  // Radio en metros
  float radius_meters = PULLEY_RADIUS_MM / 1000.0f;

  // Circunferencia efectiva considerando transmisión
  float effective_circumference =
      2.0f * M_PI * radius_meters / TRANSMISSION_RATIO;

  // Frecuencia = velocidad_lineal * pasos_efectivos / circunferencia
  float frequency =
      fabs(linear_velocity) * effective_steps_per_rev / effective_circumference;

  // Aplicar límites de seguridad
  if (frequency > MAX_FRECUENCY_LIMIT) {
    frequency = MAX_FRECUENCY_LIMIT;
  }
  if (frequency < BASE_FREQUENCY) {
    frequency = BASE_FREQUENCY;
  }

  return (int)frequency;
}

/**
 * @brief Establece la velocidad del motor a una velocidad objetivo.
 * Si la velocidad es cero o cercana a cero, detiene el motor.
 * De lo contrario, calcula la dirección y frecuencia y envía el comando al
 * motor.
 * @param velocity La velocidad deseada
 */
void set_motor_velocity(float velocity) {
  // 6. ACTUAR: Si la salida no es cero, enviar comando al motor
  if (fabs(velocity) > 0.01) {
    int direction = (velocity > 0) ? 1 : 0;
    int frequency = calculate_motor_frequency(velocity);

    // Estimación de odometría: ¿Cuántos pulsos se van a dar en este ciclo
    // (aprox)?
    float pulses_this_cycle =
        (float)frequency * (PID_LOOP_PERIOD_MS / 1000.0f);
    if (direction == 1) {
      g_car_position_pulses += (int32_t)pulses_this_cycle;
    } else {
      g_car_position_pulses -= (int32_t)pulses_this_cycle;
    }

    // Enviar el comando de velocidad continua
    motor_command_t cmd = {.num_pulses = 0, // Ignorado por el nuevo enfoque
                           .frequency = frequency,
                           .direction = direction};
    xQueueOverwrite(motor_command_queue, &cmd);
  } else {
    // Si la salida del PID es cercana a cero, apagamos el motor.
    motor_command_t stop_cmd = {
        .num_pulses = 0, .frequency = 0, .direction = 0};
    xQueueOverwrite(motor_command_queue, &stop_cmd);
  }
}
