# Observador de Luenberger — Plan de Implementación (con valores MATLAB)

## Estado Actual vs. Estado Objetivo

| Estado | Fuente actual | Fuente nueva |
|---|---|---|
| `x` (posición carro) | `pid_get_car_position_m()` ✅ | Sin cambio — medición directa |
| `ẋ` (vel. carro) | `vel_control` (integral de `u`) ❌ | Observador: `x_hat[1]` + Δx/Δt como medición |
| `θ` (ángulo) | Encoder PCNT ✅ | Sin cambio — medición directa |
| `θ̇` (vel. angular) | Diferencia finita ruidosa ⚠️ | Observador: `x_hat[3]` |

---

## Clarificación sobre dimensiones — LQI vs. Observador

El integrador LQI **no afecta las dimensiones del observador**. Son dos subsistemas independientes:

```
Observador (4×4):   x̂[k+1] = Ad·x̂[k] + Bd·u[k-1] + L·(y[k] − Cd·x̂[k])
Controlador (LQI):  u[k]   = -(K_x·x̂  + K_xdot·ẋ̂ + K_theta·θ̂ + K_w·θ̂̇ + K_i·x_i)
```

El 5to estado `x_i` (integrador de posición) vive sólo en el controlador. MATLAB construye `A_aug (5×5)` para calcular `K_aug`, pero `L` se calcula sobre el sistema original `Ad (4×4)` sin aumentar.

---

## Matrices del Sistema (MATLAB, Ts=10ms, ZOH)

Estado: `x = [x_pos, x_dot, theta, theta_dot]`

### Ad (4×4)
```
1.0000  0.0100  0.0000  0.0000
0.0000  1.0000  0.0000  0.0000
0.0000  0.0000  1.0057  0.0100
0.0000  0.0000  1.1341  1.0057
```

### Bd (4×1)
```
 0.0001
 0.0100
-0.0012
-0.2312
```

### Cd (3×4) — salidas medibles: [x, ẋ, θ]
```c
{{1.0f, 0.0f, 0.0f, 0.0f},   // mide x_pos
 {0.0f, 1.0f, 0.0f, 0.0f},   // mide x_dot (diferencia finita)
 {0.0f, 0.0f, 1.0f, 0.0f}}   // mide theta
```

### L_obs (4×3) — polos del observador: {0.40, 0.45, 0.50, 0.55}
```
0.5000   0.0100   0.0000
0.0000   0.4500   0.0000
0.0000   0.0000   1.1613
0.0000   0.0000  34.7253
```
Físicamente: θ̂̇ tiene ganancia 34.73 sobre el error de θ — alta corrección hacia el ángulo medido.

### Ganancias LQI
```c
K_x     = -8.328162f
K_xdot  = -6.777184f
K_theta = -19.903805f
K_w     = -2.760653f   // actúa sobre theta_dot_hat del observador
K_i     = -4.754309f   // actúa sobre el integrador LQI
```

---

## Cambios al Código

### ELIMINAR (obsoleto)
- `F_obs`, `G_obs`, `H_obs` — observador reducido anterior (bypasseado)
- `g_Z_estado` — estado interno del observador reducido
- `g_theta_dot` — diferencia finita local (reemplazada por observador)
- `g_ss_accel_integrator`, `g_ss_vel_integrator` — integración por PID (simplificada)
- Variables `vel_control`, `pos_control`

### AÑADIR
- `Ad[4][4]`, `Bd[4]`, `Cd[3][4]`, `L_obs[4][3]` — matrices del observador
- `x_hat[4]` — vector de estado estimado
- `g_x_pos_prev` — para diferencia finita de ẋ como medición del observador
- `g_u_prev` — u[k-1] requerido para la predicción del observador
- `g_vel_cmd` — velocidad integrada para comandar el motor
- `luenberger_update(y, u_prev)` — función privada del observador

### MODIFICAR
- `SS_Reset()` — añadir `memset(x_hat, 0)`, reset de `g_x_pos_prev`, `g_u_prev`, `g_vel_cmd`
- `state_space_controller_task()` — nuevo flujo con observador

---

## Flujo del Loop de Control (cada 10 ms)

```
PASO 1  Leer sensores:
        theta = encoder_rad - π
        x_pos = -pid_get_car_position_m()
        x_dot_meas = (x_pos - x_pos_prev) / DT   ← nueva medición de ẋ
        x_pos_prev = x_pos

PASO 2  Observador de Luenberger:
        y = [x_pos, x_dot_meas, theta]
        luenberger_update(y, u_prev)   → actualiza x_hat[4]
        x_dot_hat     = x_hat[1]       ← ẋ estimada
        theta_dot_hat = x_hat[3]       ← θ̇ estimada

PASO 3  Integrador LQI (5to estado):
        x_i = PID_Compute(&integrador, ref, x_pos)   → acumula error de posición

PASO 4  Ley de control:
        u = -(K_x·x_pos + K_xdot·x_dot_hat + K_theta·theta +
              K_w·theta_dot_hat + K_i·x_i)
        saturar u en ±10000

PASO 5  Integrar u → velocidad de motor:
        vel_cmd += u * DT
        saturar vel_cmd en ±0.66 m/s
        set_motor_velocity(-vel_cmd)

PASO 6  Guardar u para próximo ciclo:
        u_prev = u
```

---

## Plan de Verificación

1. `idf.py build` — sin errores ni warnings.
2. Simulink: comparar `theta_dot_hat` (canal 5) vs. diferencia finita anterior — debe ser más suave.
3. Pendulum en equilibrio: `u_control` (canal 3) no debe oscilar más que antes.
4. Comparar `x_hat[1]` (canal 4, vel. carro) con la integral manual previa — deben converger pasados ~200 ms de arranque.

| Estado | Fuente actual | Calidad |
|---|---|---|
| `θ` (ángulo) | Encoder PCNT | ✅ Confiable |
| `x` (posición carro) | Odometría paso a paso | ✅ Aceptable |
| `θ̇` (vel. angular) | Diferencia finita `Δθ/Δt` | ⚠️ Muy ruidosa |
| `ẋ` (vel. carro) | Integral del actuador `u` | ❌ Acoplado con la señal de control |

El código ya tiene definidas las constantes `F_obs`, `G_obs`, `H_obs` de un **observador de orden reducido** mono-variable (sólo estima `θ̇`), pero está **bypasseado** en la línea 148 (`g_theta_dot_hat = g_theta_dot`). La velocidad del carro `ẋ` se saca de `vel_control` (integral del actuador), lo que introduce realimentación espuria.

## Solución: Observador de Luenberger de Orden Completo

Reemplazar el bypass y la estimación de `ẋ` por un **observador de Luenberger en tiempo discreto** que estime los **4 estados simultáneamente** a partir de las dos salidas medibles `y = [θ, x]ᵀ`.

### Ecuaciones del observador (tiempo discreto, T=10 ms)

```
x̂[k+1] = Ad · x̂[k] + Bd · u[k] + L · (y[k] − C · x̂[k])
```

Donde:
- `x̂ = [θ̂, θ̂̇, x̂, ẋ̂]ᵀ` — vector de estado estimado (4×1)
- `y  = [θ, x]ᵀ`            — mediciones reales (2×1)
- `Ad` — matriz de sistema discreta (4×4)
- `Bd` — vector de entrada discreta (4×1)
- `C`  — matriz de salida (2×4): `C = [[1,0,0,0],[0,0,1,0]]`
- `L`  — ganancia del observador (4×2), calculada en MATLAB

### Criterio de diseño de los polos del observador

Los polos del observador deben ser **5–10 veces más rápidos** que los del controlador LQI para garantizar convergencia antes de que el control reaccione. Con T=10 ms los polos del controlador están aproximadamente en `|z| ≈ 0.85–0.95`; los del observador se ubicarán en `|z| ≈ 0.5–0.65`.

> [!IMPORTANT]  
> Las matrices `Ad`, `Bd` y la ganancia `L` deben ser calculadas en MATLAB usando el modelo linealizado del péndulo con los parámetros físicos reales del sistema. Los valores de ejemplo en el plan son **placeholders**; el usuario debe sustituirlos con los valores propios de su planta.

---

## Cambios Propuestos

### [MODIFY] `state_space_controller.c`

#### 1. Eliminar variables redundantes
- Eliminar `g_theta_dot` (diferencia finita) — ya no se necesita.
- Eliminar el uso de `vel_control` como estimación de `ẋ`.

#### 2. Añadir matrices del observador (constantes, calculadas en MATLAB)

```c
// Matriz de sistema discreta Ad (4×4) — T=10ms
static const float Ad[4][4] = {
    { ... },  // calculada en MATLAB con c2d()
    { ... },
    { ... },
    { ... }
};

// Vector de entrada discreta Bd (4×1)
static const float Bd[4] = { ..., ..., ..., ... };

// Ganancia del observador L (4×2) — polos en z ≈ {0.55, 0.60, 0.58, 0.62}
static const float L_obs[4][2] = {
    { ... , ... },   // ganancias para θ
    { ... , ... },   // ganancias para θ̇
    { ... , ... },   // ganancias para x
    { ... , ... }    // ganancias para ẋ
};
```

#### 3. Variables del vector de estado estimado (4 escalares)

```c
static float x_hat[4] = {0}; // [theta_hat, theta_dot_hat, x_hat, x_dot_hat]
```

#### 4. Función `luenberger_update()` (privada en el .c)

```c
static void luenberger_update(float theta_meas, float x_meas, float u)
{
    float y_err[2] = {
        theta_meas - x_hat[0],   // error de observación en θ
        x_meas     - x_hat[2]    // error de observación en x
    };

    float x_hat_next[4];
    for (int i = 0; i < 4; i++) {
        x_hat_next[i] = 0.0f;
        for (int j = 0; j < 4; j++)
            x_hat_next[i] += Ad[i][j] * x_hat[j];
        x_hat_next[i] += Bd[i] * u;
        x_hat_next[i] += L_obs[i][0] * y_err[0]
                       + L_obs[i][1] * y_err[1];
    }
    memcpy(x_hat, x_hat_next, sizeof(x_hat));
}
```

#### 5. Modificar `state_space_controller_task()`

- **Paso 1 (Lectura):** Leer `theta` y `x_pos` desde sensores.
- **Paso 2 (Observador):** Llamar `luenberger_update(theta, x_pos, g_u_control_prev)` para obtener `x_hat`.
- **Paso 3 (Extracción de estados estimados):** `g_theta_dot_hat = x_hat[1]`, `g_x_dot = x_hat[3]`.
- **Paso 4 (Integracion LQI):** Igual que ahora.
- **Paso 5 (Ley de control):** Usar estados estimados en lugar de valores crudos.
- **Paso 6 (Actuación):** Enviar velocidad al motor.
- **Paso 7 (Guardar `u` anterior):** `g_u_control_prev = g_u_control` para el próximo ciclo del observador.

#### 6. Reset del observador en `SS_Reset()`

```c
memset(x_hat, 0, sizeof(x_hat));
```

### [MODIFY] `state_space_controller.h`

Añadir getter para `x_dot_hat` del observador (actualmente retorna `g_x_dot` que es la integral del actuador):

```c
float ss_get_x_dot_hat(void);  // velocidad del carro estimada por el observador
```

---

## Open Questions

> [!IMPORTANT]
> **¿Tienes las matrices del modelo linealizado?**  
> Para sustituir los placeholders `Ad`, `Bd` y `L_obs` necesito los parámetros físicos del sistema o las matrices continuas `A`, `B` que hayas obtenido en MATLAB. Puedes compartir el script de MATLAB o los valores directamente.  
> Si no los tienes, puedo dejar el archivo con los **placeholders comentados claramente** y una función `luenberger_update()` completamente implementada para que sólo necesites sustituir los números.

> [!NOTE]
> El observador de orden reducido existente (`F_obs`, `G_obs`, `H_obs`) sólo estima `θ̇`. Con el observador de orden completo esas constantes quedan obsoletas y se pueden eliminar para limpiar el código.

---

## Plan de Verificación

1. **Build limpio:** `idf.py build` sin errores ni warnings.
2. **Simulink:** Con el modelo `matlab/simulacion_serial.slx` graficar `theta_dot_hat` vs. la diferencia finita anterior — el estimador debe ser más suave sin perder seguimiento de la señal real.
3. **Estabilidad:** Con el péndulo en posición vertical, verificar que la señal de control `u_control` no oscile más que antes en condiciones de equilibrio (indicaría que el observador está inyectando ruido en lugar de filtrarlo).
