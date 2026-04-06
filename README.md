# Inverted Pendulum Project (ESP32 & ESP-IDF)

Este repositorio contiene el código fuente para el control de un **Péndulo Invertido** montado sobre un carro deslizante, desarrollado utilizando un microcontrolador ESP32 y el framework de desarrollo oficial de Espressif (ESP-IDF).

## 🚀 Arquitectura del Proyecto

El proyecto está diseñado bajo una arquitectura de sistema en tiempo real (RTOS) usando FreeRTOS. Se divide en varias tareas concurrentes que garantizan que el control (PID) tenga la máxima prioridad, mientras que las métricas y la interfaz de usuario se manejan en un segundo plano.

### Flujo de Tareas (FreeRTOS Tasks)
1. **`PID_Controller` (Prioridad 5):** Es el corazón del proyecto. Se ejecuta a una frecuencia fija (100 Hz / cada 10ms) y está encargado de leer la inclinación del péndulo, calcular el error respecto al punto de equilibrio (Setpoint 0) y generar un comando de velocidad y dirección usando un lazo de control PID continuo.
2. **`Motor_Control` (Prioridad 4):** Escucha permanentemente una cola de mensajes (`motor_command_queue`). Al recibir un comando del PID, inyecta la señal PWM directamente al driver del motor paso a paso con la nueva frecuencia (velocidad) exigida de forma ininterrumpida, permitiendo un movimiento suave y sin tirones.
3. **`button_handler_task` (Prioridad 4):** Se encarga de la interfaz física. Monitorea:
   - Botón de Activar/Desactivar PID.
   - Botones de movimiento manual (Jogging Izquierda / Derecha).
   - Sensores de Final de Carrera (Paradas de Emergencia).
   - Botón de cambio de vistas en la pantalla LCD.
   - Ejecución de la rutina automática de Calibración/Homing.
4. **`uart_echo_task` (Prioridad 3):** Permite recibir comandos y enviar telemetría a través del cable USB/Serial para graficar o depurar en el PC.
5. **`LCDDisplay` (Prioridad 3):** Tarea no crítica que actualiza una pantalla LCD 16x2 a través de I2C, mostrando vistas del estado del PID, posición, errores y menús de calibración.

---

## ⚙️ Componentes Principales e Implementación

* **Controlador PID (`pid_controller.c`):** 
  Utiliza una banda muerta (Dead Band) para evitar vibraciones minúsculas cuando el péndulo está virtualmente balanceado. También incluye un control en cascada leve para intentar mantener el carro (Odometría) cerca del centro geométrico del riel, sumando un pequeño "offset" al balance del péndulo para forzar al carro a retroceder lentamente.
* **Generador PWM (`pwm_generator.c`):** 
  Utiliza el periférico de hardware **LEDC** del ESP32. Envía pulsos asíncronos y continuos con un ciclo de trabajo del 50%. La velocidad se controla ajustando la frecuencia (`ledc_set_freq`) al vuelo sin bloquear la tarea, reaccionando instantáneamente a las exigencias del PID.
* **Contador de Pulsos (`pulse_counter.c`):** 
  Utiliza el hardware **PCNT** del ESP32 dedicado a leer el encoder incremental (cuadratura) acoplado al eje del péndulo rotativo. Ofrece lecturas precisas sub-milimétricas sin gastar recursos de la CPU.
* **Interfaz y Emergencias (`button_handler.c`):** 
  Garantiza la seguridad mecánica. Si alguno de los interruptores de límite de carrera es alcanzado, el PID se desactiva enviando y obligando a la frecuencia PWM ir a 0 Hz.

---

## 🛠️ Pines y Conexiones Físicas (Hardware Setup)

*(Basado en las definiciones del código)*

### Encoders (PCNT)
* **Pin de Fase A:** Definido internamente en `pulse_counter.h`
* **Pin de Fase B:** Definido internamente en `pulse_counter.h`

### Motor Driver (Señales Step / Dir)
* **Pin PWM (Pulsos de paso - STEP):** GPIO 32
* **Pin de Dirección (DIR):** GPIO 33

### Interfaz Física (Botones y Sensores)
* **Activar/Desactivar PID:** GPIO 18
* **Movimiento Manual Izquierda:** GPIO 16
* **Movimiento Manual Derecha:** GPIO 17
* **Cambiar Vista LCD:** GPIO 19
* **Botón de Calibración:** GPIO 15
* **Final de Carrera Izquierdo (Emergencia):** GPIO 34
* **Final de Carrera Derecho (Emergencia):** GPIO 35

---

## 🎯 Instrucciones de Uso y Puesta en Marcha

1. **Encendido:** Al iniciar, la pantalla LCD mostrará un mensaje de inicialización. El sistema arranca con el control PID **deshabilitado** por seguridad.
2. **Calibración (Homing):** Es el paso fundamental. Mantén el péndulo colgado (hacia abajo). Presiona el botón de **Calibración (GPIO 15)**.
   * El carro se moverá lentamente hasta tocar un final de carrera, luego se moverá al opuesto midiendo la distancia total del riel.
   * Tras chocar con ambos, el sistema calculará el centro exacto y el carro viajará a esa posición media.
   * El sistema asumirá automáticamente que esta posición de reposo del encoder mirando hacia el suelo es "-180°" y establecerá su punto de equilibrio (Setpoint) 180 grados en dirección opuesta (arriba puramente vertical).
3. **Movimiento Manual (Jogging):** Si el PID está apagado, puedes usar los botones manuales para desplazar el carro a conveniencia y despegarlo de las paredes.
4. **Activación del Péndulo:**
   * Levanta el péndulo manualmente hasta su punto aproximado de equilibrio superior (vertical).
   * Presiona el botón de **Activar PID (GPIO 18)**. 
   * El motor comenzará a rastrear la posición instantáneamente y mantendrá el péndulo balanceado. 
   * Volver a presionar el botón desactivará el motor y dejará caer el péndulo.
5. **Parada de Emergencia:** Si en cualquier momento de descontrol agresivo el marco del carro golpea uno de los límites izquierdo (34) o derecho (35), el motor se detendrá al instante protegiendo la estructura.

---

*Proyecto configurado y compilable exclusivamente bajo el entorno ESP-IDF Extension (No PlatformIO). Funciona sobre CMake Tools.*

---

## 💡 Posibles Mejoras (Roadmap)

A continuación, se enumeran las optimizaciones técnicas identificadas para mejorar la estabilidad y limpieza del código.

### 1. Mejoras Críticas para el Controlador PID (Resolución de Estabilidad)

El péndulo invertido es un sistema inestable de alta dinámica. Para estabilizarlo con solidez, el control necesita precisión matemática estricta:

* **Hardware Timer (`esp_timer`) vs FreeRTOS (`vTaskDelayUntil`)**: 
  Actualmente, el PID vive dentro de un bucle de RTOS que duerme mediante `vTaskDelayUntil(10ms)`. Sin embargo, el "Scheduler" de RTOS tiene latencias y bloqueos aleatorios generados por otras tareas concurrentes (LCD, UART, Interrupciones Wifi/Bluetooth). A este error de ejecución milimétrico se le conoce como **"Jitter"**.
  **Mejora:** Eliminar el `vTaskDelayUntil` y trasladar la lógica del PID a un callback de **`esp_timer`**. Este es un hardware interno en el silicio que interrumpe brutal y atómicamente todo lo demás (con resolución de microsegundos) para clavar la ejecución matemática exactamente en el período correcto sin Jitter RTOS.
  
* **Cálculo Real del Diferencial de Tiempo (`dt`)**:
  Al asumir que el ciclo dura siempre `10ms` constantes (`loop_period_in_seconds`), cualquier Jitter del RTOS convierte el cálculo del factor Derivativo (`D = (Error - LastError)/dt`) en puro ruido o latigazos virtuales hacia el motor.
  **Mejora:** Capturar el tiempo real pasado mediante `esp_timer_get_time()` y alimentar el verdadero delta `dt` en cada iteración de `PID_Compute()`.

* **Fijación de Tareas (Core Affinity)**: 
  Usar `xTaskCreatePinnedToCore` para asegurar que las tareas de control de hardware puro (`Motor_Control`) vivan permanentemente sin obstrucciones en el Núcleo 1 (PRO_CPU), arrinconando tareas superficiales como Pantallas I2C y Serial Data hacia el Núcleo 0 (APP_CPU).

### 2. Limpieza de Deuda Técnica
* **Migración del driver PCNT**: Actualizar el driver obsoleto `driver/pcnt.h` usado en `pulse_counter.c` al nuevo estándar de ESP-IDF v5 `driver/pulse_cnt.h`, que es robusto en concurrencia (thread-safe) y maneja index callbacks en hardware puro.
* **Limpieza de variables**: Remover variables lógicas sin aplicación directa en `pid_controller.c` (`velocity`), aliviando los warnings de compilación (Clean Build).

### 3. Solución al Bucle Abierto de la Odometría
Actualmente, el software estima la posición de la base del péndulo calculándola teóricamente ("Si envié una frecuencia Y durante X milisegundos, di Z pasos."). A esto se le llama **Control de Lazo Abierto**. Si la inercia del movimiento hace que el motor eléctrico "salte" o pierda un paso real, el ESP32 no lo sabrá nunca y su idea virtual del punto de centro/cero del riel irá "derivando".

* **Solución de Código (Re-homing Dinámico)**: Dar inteligencia a los Fines de Carrera (GPIO 34, 35). Más que simple "Botón de Parada de Emergencia", si el carro deriva tanto como para rosar uno, capturar ese evento y sobreescribir con dureza la variable `g_car_position_pulses` forzando la sincronización de las coordenadas cero de vuelta a la realidad.
* **Solución Fija de Hardware (Loopback de Pulsos)**: Cablear externamente el propio pin emisor PWM del Motor (STEP - GPIO 32) hacia un módulo PCNT de lectura sobrante del ESP32. Esto cambiaría el cálculo teórico por un conteo absoluto 100% puro de pulsos emitidos físicamente.

### 4. Robustez Mecánica y de Software
* **Apagado de Emergencia por Caída Irreversible**: Implementar una condición en el código que detenga el PID inmediatamente si el péndulo supera un ángulo crítico de inclinación (ej. > 45°). En el estado actual, si el péndulo cae irrevocablemente, el control enviará la máxima potencia sin fin para intentar recuperarlo, logrando únicamente chocar el carro con violencia contra las poleas.
* **Prevención de 'Stack Overflow'**: Aumentar la huella de memoria (Stack) asignada a las tareas de FreeRTOS. Trabajar con constantes mínimas (`configMINIMAL_STACK_SIZE`) en un ecosistema que usa rutinas de log (`ESP_LOG`) y matemática flotante termina rutinariamente en cuelgues y reinicios repentinos de hardware.

### 5. Modularidad y Persistencia (Tuning)
* **Memoria No Volátil (NVS)**: Guardar las constantes sintonizadas del PID (Kp, Ki, Kd) en la memoria Flash (NVS) del ESP32. Esto evitaría el fastidio de perder los valores arduamente ajustados a cada apagado / reseteo de la tarjeta.
* **Archivo de Configuración Centralizado (`config.h`)**: Agrupar constantes, distribución de pines y dimensiones mecánicas que ahora mismo se encuentran dispersas a lo largo de los archivos (`main.c`, `pulse_counter.h`, etc.) dentro de un único cabecero maestro de configuración.

### 6. Mejoras "Con Precaución" (Alertas Técnicas)
Ciertas aproximaciones clásicas de software, si bien son buenas ideas en proyectos regulares, podrían atentar críticamente contra la rapidez que demanda un péndulo invertido si no se aíslan apropiadamente:
* ⚠️ **Suavizado de Motor (Ramping Trapezoidal)**: Añadir curvas S de aceleración/desaceleración para la corrección suaviza el deslizamiento, pero le introduce muchísima pre-latencia al mandato del actuador. El PID de un péndulo exige correcciones torques instantáneos, no curvos, o se desbalanceará. (Usarlo solo para desplazamientos manuales de "Homing").
* ⚠️ **Filtros de Software en los Encoders (Media Móvil)**: Suavizar la señal del sensor óptico con algoritmos predictivos o generacionales inyecta el temido *Phase Lag* (ver o proyectar un evento milisegundos más tarde de su instante real). Se prefiere utilizar exclusivamente el filtro de ruido del Hardware de Silicio original (`pcnt_filter_enable`).
* ⚠️ **Módulos Inalámbricos (Wi-Fi/Bluetooth)**: Muy tentador para calibración visual o graficado. Lamentablemente, invocar rutinas criptográficas y de red inalámbricas en el modelo ESP32 levantan masivas Interrupciones (ISR). Si las tareas de control de motores no se clavan estrictamente en el otro núcleo usando "Core Affinity", el procesador dejará "mudo" el motor repetidamente causando caídas esporádicas.

### 7. Testing y Aseguramiento de Calidad (QA)
Para evitar que un error de software estrelle la máquina física, se recomienda el uso de **Tests Unitarios** enfocados exclusivamente en la matemática de control, ignorando los periféricos o la simulación de física (Cualquier simulación física de robótica debe hacerse en MATLAB/Python, nunca dentro del ESP32).

Se recomienda usar el framework **Unity** (integrado nativamente en ESP-IDF) para someter a pruebas mecánicas a la siguiente función crítica:
* **Función Objetivo:** `PID_Compute()` (Ubicada en `pid_controller.c`).
* **Casos de Prueba a Implementar:**
    1. **Test del Lazo Proporcional:** Inyectar un "Error" constante y verificar que la salida aumente exactamente proporcional a `Kp`.
    2. **Freno de Integral (Anti-Windup):** Enviar un error gigantesco y mantenido en el tiempo para ver si el valor de la sumatoria Integral sigue creciendo hasta estallar la memoria RAM, o si respeta correctamente los límites de saturación (`MAX_OUTPUT_PULSES`).
    3. **Resistencia a Banda Muerta (Dead Band):** Inyectar errores sumamente diminutos y menores a `DEAD_BAND_ANGLE`. El test debe pasar únicamente si la función resetea la integral a 0 ($0.0f$), comprobando que el péndulo no intentará auto-corregirse con un microrruido.
    4. **Caos de Derivada:** Enviar una señal muy picuda (mucho error, luego nada, luego mucho error opuesto) simulando un tirón. Verificar que los cálculos de `(error - ultimo_error)/dt` generen bien sus signos negativos frenando virtualmente el movimiento, descartando que hayan desbordamientos `NaN` en las divisiones.

### 8. Control Avanzado del Sistema
* **Control por Variables de Estado**: Desarrollar e implementar un módulo de control avanzado mediante realimentación de variables de estado. Esto incluirá la utilización de un **Observador de Luenberger** para estimar aquellos estados dinámicos del sistema que no pueden ser medidos directamente por los sensores, logrando una respuesta más completa y estable.

### 9. Parametrización y Unidades Físicas
* **Abstracción de Variables Físicas**: Transformar las unidades de hardware en bruto a magnitudes físicas reales. Es necesario mapear los pulsos leídos por el encoder rotativo transformándolos a **Radianes (rad) y Grados (°)**. Asimismo, mapear y traducir los pulsos enviados al motor paso a paso para medir la **posición métrica (mm/cm)** física de desplazamiento que tiene el carrito sobre el riel.
