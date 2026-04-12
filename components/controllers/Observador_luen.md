# Observador de Luenberger en `state_space_controller.c`

## Contexto y Problema

El controlador LQI actual usa el vector de estado `[x, ẋ, θ, θ̇]` pero sólo puede **medir** dos de los cuatro estados directamente:

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
