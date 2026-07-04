# Simulación — Control de Velocidad de Ventiladores en Rack de Servidores (PID)

Trabajo Final Integrador — Teoría de Control (UTN-FRBA, K4011, Prof. Omar Civale)
Ale Marino, Santiago | Molina, Nicolás Ariel

Este repositorio simula el lazo cerrado de control descripto en el informe: un
controlador PID (implementado sobre un PLC Siemens S7-1200) que regula la
velocidad de los ventiladores de un rack de servidores para mantener la
temperatura del procesador en un setpoint de 65 °C, a pesar de las
perturbaciones de carga de CPU y temperatura ambiente.

| Script | Qué es | Cuándo usarlo |
|---|---|---|
| `modelo.py` | Módulo con la física de la planta y el PID (fuente única de verdad de constantes y del paso de integración) | No se ejecuta solo; lo importa el tablero. Tocar acá cualquier parámetro o la ley de control |
| `tablero_qt.py` | **Tablero interactivo en vivo**: UI de escritorio (PySide6 + pyqtgraph) con sliders y botones en tiempo real | Para la demo en la defensa oral y para explorar el sistema |

## 1. Instalación y ejecución (paso a paso)

Si ya se tiene Python 3.9+ y el código descargado, ir directo al Paso 3.

> **Comandos:** en macOS/Linux suelen ser `python3` / `pip3`; en Windows,
> `python` / `pip`. Si uno devuelve _"command not found"_, usar la otra
> variante (la forma `python -m pip ...` siempre funciona). En este README se
> escribe `python` / `pip`.

### Paso 1 — Python 3.9 o superior

Verificar con `python --version` (o `python3 --version`). Si es 3.9+, continuar.
Si no está instalado o es una versión menor:

- **Windows:** instalador de <https://www.python.org/downloads/>, marcar
  _"Add python.exe to PATH"_ al instalar.
- **macOS:** `brew install python` (o el instalador de python.org).
- **Linux (Debian/Ubuntu):** `sudo apt install python3 python3-venv python3-pip`

### Paso 2 — Obtener el código

- **Con git:** `git clone <URL-del-repo>` y luego `cd tdc_tfi`.
- **Sin git:** en GitHub, **Code → Download ZIP**, descomprimir y abrir una
  terminal dentro de la carpeta `tdc_tfi`.

### Paso 3 — Entorno virtual (venv) — recomendado

Aísla las librerías del proyecto y evita el error
`externally-managed-environment` (típico en macOS con Homebrew). Crear una vez:

```bash
python -m venv .venv
```

Activar en cada sesión (el prompt mostrará `(.venv)`):

```bash
# macOS / Linux
source .venv/bin/activate

# Windows (PowerShell)
.venv\Scripts\Activate.ps1

# Windows (CMD)
.venv\Scripts\activate.bat
```

Salir: `deactivate`.

### Paso 4 — Instalar dependencias

Con el venv activado:

```bash
pip install -r requirements.txt
```

Instala `numpy`, `PySide6` y `pyqtgraph`.

### Paso 5 — Ejecutar

```bash
python tablero_qt.py
```

Detalle del tablero en la sección 2.

### Alternativa: instalación sin entorno virtual

Si no se desea utilizar un venv, ordenadas de mayor a menor recomendación:

1. **`pip install --user -r requirements.txt`** — instala solo para el usuario
   actual. En macOS con Homebrew puede continuar mostrando
   `externally-managed-environment`.
2. **`pip install --break-system-packages -r requirements.txt`** — último
   recurso. Funciona, pero puede dejar en estado inconsistente el Python de
   Homebrew. **No recomendado.** El venv (Paso 3) no presenta esta desventaja.

### Problemas comunes

| Error / síntoma | Solución |
|---|---|
| `error: externally-managed-environment` | Utilizar un entorno virtual (Paso 3). Es exactamente lo que resuelve este error. |
| `command not found: python` | Probar `python3` (macOS/Linux) o `py` (Windows). |
| `command not found: pip` | Utilizar `python -m pip ...` (o `pip3`). |
| `ModuleNotFoundError: No module named 'numpy'` (o `PySide6`, etc.) | Las dependencias no fueron instaladas, o el entorno virtual no está activo. Activar el venv y ejecutar `pip install -r requirements.txt`. |
| El tablero no abre / `could not load platform plugin "xcb"` | Ejecutar en una máquina con pantalla (no funciona por SSH sin X11). En Linux puede faltar: `sudo apt install libxcb-cursor0`. |
| En Windows PowerShell, `Activate.ps1` da error de permisos | Ejecutar una vez `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned` y volver a activar. |

## 2. `tablero_qt.py` — tablero interactivo en vivo

```bash
python tablero_qt.py
```

Se abre una única ventana con 5 gráficos a la izquierda que se van dibujando
en tiempo real (ventana deslizante de los últimos 400 s) y un panel de
controles a la derecha. El tiempo avanza mientras la ventana está abierta, y
todo lo que toques en el panel de control se aplica de inmediato, sin
reiniciar nada.

### Qué se ve en cada gráfico

| Gráfico | Señales | Corresponde a |
|---|---|---|
| Temperatura | Referencia `Tref` · Salida real de la planta `Tcpu` · Realimentación `LM35` (lo que efectivamente "ve" el PLC, con el ruido del sensor) | Valor de referencia, señal de salida y señal de realimentación (informe 4.13) |
| Error | `e(t) = Tref − Tmedido` | Señal de error, calculada sobre la medición real (no sobre el valor "verdadero" de la planta, que el controlador no puede conocer) |
| Señal de control | `u(t)` = PWM en % | Salida del PID ya saturada 0–100 % |
| Velocidad del ventilador | RPM comandada vs. RPM medida por el sensor Hall (con ruido) | Las dos ramas de realimentación: la de control (temperatura) y la de verificación del actuador (RPM) |
| Perturbaciones | Carga de CPU (%) y Tamb (°C), en ejes distintos | Las dos perturbaciones externas que podés disparar desde los botones |

Cada gráfico lleva sus referencias de color a la altura del título. Además, un
recuadro de texto (arriba a la derecha) muestra en vivo los valores numéricos
actuales de `t`, `Tref`, `Tcpu`, `Tmedido`, error, PWM, RPM, y el aporte
individual de cada término **P**, **I** y **D** a la señal de control antes de
saturar — útil para ver, sintonización a sintonización, cuál de las tres
acciones está dominando la respuesta.

### Panel de control

**Sliders (se pueden mover en cualquier momento, incluso a mitad de un
transitorio):**

- `Kp`, `Ki`, `Kd`: ganancias del PID (arrancan en 5,0 / 0,3 / 2,0, la
  sintonización adoptada en el informe). Podés, por ejemplo, bajar `Kp` y ver
  cómo la respuesta se vuelve más lenta y oscilatoria (la acción integral igual
  lleva la temperatura al setpoint), o subir mucho `Kd` y ver cómo se vuelve más
  sensible al ruido del sensor.
- `Setpoint [°C]`: mueve la referencia en caliente.
- `Carga CPU [%]`: perturbación manual de carga; podés arrastrarla lentamente
  (rampa) o de un salto (escalón), además de usar los botones de abajo.
- `Tamb [°C]`: temperatura ambiente base.
- `Vel. sim [s/frame]`: cuántos segundos de tiempo simulado avanzan por cada
  actualización de pantalla (por defecto 5). Subila para ver transitorios
  largos más rápido, bajala a 1 para mirar en detalle un cambio brusco.

**Checkbox:**

- `Ruido de sensor (LM35 ±0,5 °C)`: activa/desactiva el ruido de medición y el
  del sensor Hall. Destildado, `Tcpu` y `Tmedido` coinciden exactamente
  (realimentación ideal); activo se ve la diferencia real que introduce el
  sensor, y por qué la rama derivativa necesita el filtro pasa-bajos (informe
  4.13).

**Botones — perturbaciones puntuales, se aplican al instante:**

- `Pico 90%`: salto de carga de CPU a 90 % (escenario "pico de tráfico").
- `DoS 100%`: salto de carga de CPU a 100 % (escenario "ataque DoS").
- `Carga 30%`: vuelve la carga a un valor bajo, para ver la recuperación.
- `Falla HVAC`: dispara una rampa de +10 °C en la temperatura ambiente a lo
  largo de 100 s (al terminar, deja el slider de Tamb en el nuevo valor).
- `Pausar` / `Reanudar`: congela el tiempo (los sliders se pueden seguir
  moviendo, pero no tienen efecto hasta reanudar).
- `Reset`: vuelve a `t=0`, `Tcpu=22 °C`, con todos los sliders en sus valores
  iniciales.

### Los escenarios del informe y cómo reproducirlos

Los 6 escenarios de la sección 6.2 del informe se pueden reproducir en vivo:

| Escenario (informe) | Cómo reproducirlo en el tablero |
|---|---|
| Arranque (cold start, 22 °C) | `Reset` (arranca en 22 °C con carga 30 %) |
| Operación nominal 60 % | slider `Carga CPU` = 60 |
| Pico de tráfico 90 % | botón `Pico 90%` |
| Ataque DoS 100 % | botón `DoS 100%` |
| Cambio de setpoint 65 → 55 °C | slider `Setpoint` |
| Falla parcial de HVAC | botón `Falla HVAC` |

### Cómo usarlo para explorar el sistema

Un recorrido sugerido: dejalo correr desde `t=0` con los valores por defecto
(carga 30 %, converge a un valor bajo); apretá `Pico 90%` y mirá cómo sube el
PWM y la temperatura se estabiliza cerca de 65 °C; apretá `DoS 100%` y observá
el nuevo transitorio; bajá `Kd` a 0 con el slider y repetí el golpe de carga
para ver cómo aumenta el sobreimpulso sin la acción derivativa; subí mucho `Ki`
y mirá cómo aparecen oscilaciones. Todo esto sin reiniciar el script ni tocar
código.

## 3. Parámetros del modelo

Todos viven en `modelo.py` (fuente única de verdad):

| Parámetro | Valor | Fuente |
|---|---|---|
| `Kp`, `Ki`, `Kd` | 5,0 / 0,3 / 2,0 (ajustables en vivo en el tablero) | Sintonización adoptada (informe 4.6), **no cambiar sin recalcular Routh-Hurwitz (informe 4.11)** |
| `C_TH` | 50 J/°C | Informe 6.1 |
| `Q_IDLE`, `Q_TDP` | 30 W / 150 W | Informe 6.1 |
| `RPM_MIN`, `RPM_MAX` | 800 / 3500 RPM | Informe 6.1 |
| `R_TH_BASE` | **0,6 °C/W** | Ver sección 4 — valor ajustado, no es el que figura literal en la tabla 6.1 del informe |
| `T_SETPOINT_DEFAULT` | 65 °C (ajustable en vivo) | Informe 6.1 |

## 4. Nota importante: por qué `R_TH_BASE` se fijó en 0,6 y no en 0,3

Este es el hallazgo más relevante del armado de la simulación, y vale la pena
que quede documentado para la defensa del TP.

El informe (sección 6.1) lista `Rth base = 0,3 °C/W ("sin ventilador")`. Con
ese valor tal cual, el sistema **nunca ejerce control activo** en los
escenarios de operación nominal (60 %) ni de pico de tráfico (90 %): la
temperatura de equilibrio pasivo a RPM mínima (800 RPM, PWM = 0 %) ya queda por
debajo del setpoint de 65 °C para cualquier carga menor al ~95 %, porque el
ventilador solo puede enfriar, nunca calentar. Concretamente, con
`Rth_base = 0,3`:

| Carga CPU | Temperatura de equilibrio a RPM mínima |
|---|---|
| 30 % | ≈ 42 °C |
| 60 % | ≈ 53 °C |
| 90 % | ≈ 63 °C |
| 100 % | ≈ 67 °C (recién acá se supera el setpoint) |

Es decir, con los parámetros literales del informe, el PID se queda saturado en
el mínimo casi todo el tiempo y la temperatura jamás converge a 65 °C salvo en
el escenario de DoS — lo cual contradice la narrativa de las secciones 6.2 y
6.4 del informe ("mantiene 65 °C ± 2 °C con ciclo de trabajo estable" en
operación nominal al 60 %).

Además, esto es inconsistente con el propio informe: la sección 4.8 linealiza la
planta en el punto de operación (65 °C, 2000 RPM, 90 W) y obtiene
`Rth ≈ 0,6 °C/W`. Físicamente, `Rth` tiene que ser **mayor** a menor RPM (peor
enfriamiento) y **menor** a mayor RPM — es decir, `Rth(800 RPM)` debería ser
_mayor_ que `Rth(2000 RPM)`, no menor. Con `Rth_base = 0,3` ocurre lo contrario.

**Decisión adoptada (confirmada con el equipo):** se fijó `R_TH_BASE = 0,6 °C/W`
en `modelo.py`, el mismo valor que ya usa el informe para la linealización en
4.8. Con este valor el sistema sí requiere modulación activa del ventilador
desde el 60 % de carga, reproduciendo la idea central del informe (el PID
mantiene la temperatura cerca del setpoint en operación normal, no solo durante
un ataque DoS). Esta constante es independiente de `Kp/Ki/Kd` y de la tabla de
Routh-Hurwitz (que usa el `Kt` linealizado, no `Rth_base`), así que el cambio no
invalida el análisis de estabilidad del informe.

**Pendiente para ustedes:** para que el informe y el código queden 100 %
alineados, conviene actualizar la tabla de parámetros de la sección 6.1 del
documento (`Rth base = 0,3 °C/W` → `0,6 °C/W`).

## 5. Convención de signo del error (fijada, no cambiar sin propagar)

```
e(t) = Tref − Tcpu(t)
```

Positivo cuando falta calentar (Tcpu < setpoint), negativo cuando hay
sobretemperatura (Tcpu > setpoint). Por eso la salida cruda del PID se usa
**negada** para construir la señal de control (sube el PWM con
sobretemperatura). Ver el cálculo de `u_pre` en `modelo.py`.

## 6. Estructura del repositorio

```
modelo.py          # Fisica de la planta + PID (fuente unica de verdad)
tablero_qt.py      # Tablero interactivo en vivo con PySide6 + pyqtgraph
requirements.txt   # Dependencias (numpy, PySide6, pyqtgraph)
```
