# Simulación — Control de Velocidad de Ventiladores en Rack de Servidores (PID)

Trabajo Final Integrador — Teoría de Control (UTN-FRBA, K4011, Prof. Omar Civale)
Ale Marino, Santiago | Molina, Nicolás Ariel

Este repositorio contiene dos formas de simular el lazo cerrado de control
descripto en el informe: un controlador PID (implementado sobre un PLC
Siemens S7-1200) que regula la velocidad de los ventiladores de un rack de
servidores para mantener la temperatura del procesador en un setpoint de
65 °C, a pesar de las perturbaciones de carga de CPU y temperatura ambiente.

| Script | Qué es | Cuándo usarlo |
|---|---|---|
| `simulacion_pid.py` | Corre los 6 escenarios del informe de una sola vez y guarda gráficos `.png` | Para reproducir los resultados de las secciones 6.2/6.4 del informe, o para el anexo de código |
| `tablero_control.py` | **Tablero interactivo en vivo**: el tiempo corre en tiempo real, con sliders para los parámetros y botones para disparar perturbaciones cuando quieras | Para explorar el sistema, probar sintonizaciones distintas de Kp/Ki/Kd, o hacer una demo en la defensa oral |

## 1. Requisitos

- Python 3.9 o superior.
- Paquetes: `numpy`, `matplotlib`.
- Para `tablero_control.py`: un backend gráfico interactivo de matplotlib
  (el que viene por defecto con una instalación normal de Python en Windows,
  típicamente TkAgg, alcanza). No funciona en un entorno 100% headless/sin
  pantalla — en ese caso usá `simulacion_pid.py`, que sí guarda `.png`.

Instalación de dependencias:

```bash
pip install -r requirements.txt
```

## 2. `simulacion_pid.py` — corrida única de los 6 escenarios

```bash
python simulacion_pid.py
```

Al ejecutarlo vas a ver, en este orden:

1. Una tabla en consola con el resumen numérico de los 6 escenarios (temperatura
   final y máxima de cada ventana, y error final).
2. Dos ventanas de gráficos (si corrés el script en un entorno con interfaz
   gráfica — VS Code, terminal local con Tk/Qt, Jupyter, etc.). Si tu entorno
   no tiene pantalla disponible, los gráficos igual se guardan como archivos
   `.png` en la carpeta `resultados/` (se crea automáticamente).

No hace falta tocar nada del código para verlo funcionar; los 6 escenarios
están encadenados en una única corrida de 500 segundos, tal como se describe
en la sección 6.2 del informe.

### Gráfico 1 — `resultados/01_lazo_de_control.png` (4 paneles)

| Panel | Qué muestra | Qué mirar |
|---|---|---|
| Temperatura | `Tcpu(t)` (rojo) contra el `Setpoint` (negro, punteado), con líneas de referencia en 75 °C (alerta) y 85 °C (throttling) | Que la temperatura nunca cruce la línea roja de throttling, y que se "pegue" al setpoint después de cada perturbación |
| PWM | Ciclo de trabajo de la señal de control, 0–100 % | Cómo el controlador sube el PWM ante una perturbación y lo relaja cuando el error se corrige |
| RPM | Velocidad del ventilador, 800–3500 RPM (mapeo directo del PWM) | La correlación 1 a 1 con el panel de PWM |
| Error | `e(t) = Tref − Tcpu(t)` | Que el error converja a ~0 después de cada escalón (error nulo en estado estable, acción integral) |

Las líneas grises verticales punteadas marcan los cambios de escenario en
t = 50, 100, 200, 300 y 400 s.

### Gráfico 2 — `resultados/02_perturbaciones.png` (2 paneles)

Muestra las perturbaciones que se le aplican al sistema para generar el
gráfico anterior: el perfil de carga de CPU (%) y la rampa de temperatura
ambiente. Sirve para leer el gráfico 1 en contexto: cada quiebre en la
temperatura/PWM/RPM/error corresponde a un quiebre en alguna de estas dos
curvas.

### Resumen en consola

Por cada escenario, se imprime la temperatura al final de la ventana, la
temperatura máxima alcanzada dentro de esa ventana, y el error al final de la
ventana. Al final se informa la temperatura máxima y mínima de toda la
corrida, con una verificación explícita de que nunca se cruzan los 85 °C de
throttling (el objetivo de control más crítico del sistema).

### Los 6 escenarios simulados

Todos encadenados en una sola línea de tiempo de 500 s (`dt = 1 s`, igual al
`Tscan` del PLC):

| Ventana | Escenario | Carga CPU | Setpoint | Tamb |
|---|---|---|---|---|
| 0–50 s | Arranque (cold start, Tcpu inicial 22 °C) | 30 % | 65 °C | 22 °C |
| 50–100 s | Operación nominal | 60 % | 65 °C | 22 °C |
| 100–200 s | Pico de tráfico | 60 % → 90 % (escalón) | 65 °C | 22 °C |
| 200–300 s | Ataque DoS | 90 % → 100 % (escalón) | 65 °C | 22 °C |
| 300–400 s | Recuperación + cambio de setpoint | 100 % → 40 % | 65 °C → 55 °C | 22 °C |
| 400–500 s | Falla parcial de HVAC | 40 % | 55 °C | rampa 22 °C → 32 °C |

## 3. `tablero_control.py` — tablero interactivo en vivo

```bash
python tablero_control.py
```

Se abre una única ventana con 5 gráficos a la izquierda que se van dibujando
en tiempo real (ventana deslizante de los últimos 400 s) y un panel de
controles a la derecha. A diferencia de `simulacion_pid.py`, acá **no hay una
corrida fija**: el tiempo avanza mientras la ventana está abierta, y todo lo
que toques en el panel de control se aplica de inmediato, sin reiniciar nada.

### Qué se ve en cada gráfico

| Gráfico | Señales | Corresponde a |
|---|---|---|
| Temperatura | Referencia `Tref` (negro, punteado) · Salida real de la planta `Tcpu` (rojo) · Realimentación `Tmedido` (naranja punteado, lo que efectivamente "ve" el PLC a través del LM35) | Valor de referencia, señal de salida y señal de realimentación (con el ruido del sensor, informe 4.13) |
| Error | `e(t) = Tref − Tmedido` | Señal de error, calculada sobre la medición real (no sobre el valor "verdadero" de la planta, que el controlador no puede conocer) |
| Señal de control | `u(t)` = PWM en % | Salida del PID ya saturada 0–100 % |
| Velocidad del ventilador | RPM comandada (verde) vs. RPM medida por el sensor Hall (verde oliva, punteado, con ruido) | Las dos ramas de realimentación del sistema: la de control (temperatura) y la de verificación del actuador (RPM) |
| Perturbaciones | Carga de CPU (%) y Tamb (°C), en ejes distintos | Las dos perturbaciones externas que podés disparar desde los botones |

Además, un recuadro de texto (arriba a la derecha) muestra en vivo los
valores numéricos actuales de `t`, `Tref`, `Tcpu`, `Tmedido`, error, PWM, RPM,
y el aporte individual de cada término **P**, **I** y **D** a la señal de
control antes de saturar — útil para ver, sintonización a sintonización, cuál
de las tres acciones está dominando la respuesta.

### Panel de control

**Sliders (se pueden mover en cualquier momento, incluso a mitad de un
transitorio):**

- `Kp`, `Ki`, `Kd`: ganancias del PID (arrancan en 5,0 / 0,3 / 2,0, la
  sintonización adoptada en el informe). Podés probar, por ejemplo, bajar `Kp`
  y ver cómo el sistema deja de acercarse al setpoint, o subir mucho `Kd` y
  ver cómo se vuelve más sensible al ruido del sensor.
- `Setpoint [°C]`: mueve la referencia en caliente (equivale al escenario de
  "cambio de setpoint" del informe, pero a cualquier valor y en cualquier
  momento).
- `Carga CPU [%]`: perturbación manual de carga; podés arrastrarla lentamente
  (rampa) o de un salto (escalón), además de usar los botones de abajo.
- `Tamb [°C]`: temperatura ambiente base.
- `Vel. sim [s/frame]`: cuántos segundos de tiempo simulado avanzan por
  cada actualización de pantalla (por defecto 5). Subila para ver transitorios
  largos más rápido, bajala a 1 para mirar en detalle un cambio brusco.

**Checkbox:**

- `Ruido de sensor (LM35 ±0,5 °C)`: activa/desactiva el ruido de medición y
  el ruido del sensor Hall. Con el checkbox destildado, `Tcpu` y `Tmedido`
  coinciden exactamente (realimentación ideal); con el checkbox activo se ve
  la diferencia real que introduce el sensor, y por qué la rama derivativa
  necesita el filtro pasa-bajos (informe 4.13).

**Botones — perturbaciones puntuales, se aplican en el instante que los
apretás:**

- `Pico 90%`: salto de carga de CPU a 90 % (escenario "pico de tráfico").
- `DoS 100%`: salto de carga de CPU a 100 % (escenario "ataque DoS").
- `Carga 30%`: vuelve la carga a un valor bajo, para ver la recuperación.
- `Falla HVAC`: dispara una rampa de +10 °C en la temperatura ambiente a lo
  largo de 100 s, igual que en el escenario 6 del informe, mueve el slider de
  Tamb en vivo mientras la rampa corre.
- `Pausar` / `Reanudar`: congela el tiempo (los sliders se pueden seguir
  moviendo, pero no van a tener efecto hasta reanudar).
- `Reset`: vuelve la simulación a `t=0`, `Tcpu=22°C`, con todos los sliders
  en sus valores iniciales.

### Cómo usarlo para explorar el sistema

Un recorrido sugerido: dejalo correr desde `t=0` con los valores por defecto
(carga 30 %, va a converger a un valor bajo); apretá `Pico 90%` y mirá cómo
sube el PWM y la temperatura se estabiliza cerca de 65 °C; apretá `DoS 100%`
y observá el nuevo transitorio; bajá `Kd` a 0 con el slider y repetí el golpe
de carga para ver cómo aumenta el sobreimpulso sin la acción derivativa;
subí mucho `Ki` y mirá cómo aparecen oscilaciones. Todo esto sin reiniciar el
script ni tocar código.

## 4. Parámetros del modelo

| Parámetro | Valor | Fuente |
|---|---|---|
| `Kp`, `Ki`, `Kd` | 5,0 / 0,3 / 2,0 (ajustables en vivo en `tablero_control.py`) | Sintonización adoptada (informe 4.6), **no modificar en `simulacion_pid.py` sin recalcular Routh-Hurwitz (informe 4.11)** |
| `C_TH` | 50 J/°C | Informe 6.1 |
| `Q_IDLE`, `Q_TDP` | 30 W / 150 W | Informe 6.1 |
| `RPM_MIN`, `RPM_MAX` | 800 / 3500 RPM | Informe 6.1 |
| `R_TH_BASE` | **0,6 °C/W** | Ver nota de la sección 5 de este README — valor ajustado, no es el que figura literal en la tabla 6.1 del informe |
| `T_setpoint` | 65 °C (→ 55 °C desde t=300s en `simulacion_pid.py`; ajustable en vivo en el tablero) | Informe 6.1 |

## 5. Nota importante: por qué `R_TH_BASE` se fijó en 0,6 y no en 0,3

Este es el hallazgo más relevante del trabajo de armar esta simulación, y vale
la pena que quede documentado para la defensa del TP.

El informe (sección 6.1) lista `Rth base = 0,3 °C/W ("sin ventilador")`, valor
que también aparece en el contexto que armaron para esta tarea. Al correr la
simulación con ese valor tal cual, el sistema **nunca ejerce control activo**
en los escenarios de operación nominal (60 %) ni de pico de tráfico (90 %):
la temperatura de equilibrio pasivo a RPM mínima (800 RPM, PWM = 0 %) ya queda
por debajo del setpoint de 65 °C para cualquier carga menor al ~95 %, porque
el ventilador solo puede enfriar, nunca calentar. Concretamente, con
`Rth_base = 0,3`:

| Carga CPU | Temperatura de equilibrio a RPM mínima |
|---|---|
| 30 % | ≈ 42 °C |
| 60 % | ≈ 53 °C |
| 90 % | ≈ 63 °C |
| 100 % | ≈ 67 °C (recién acá se supera el setpoint) |

Es decir, con los parámetros literales del informe, el PID se queda saturado
en el mínimo durante casi toda la corrida y la temperatura jamás converge a
65 °C salvo en el escenario de DoS — lo cual contradice la narrativa de las
secciones 6.2 y 6.4 del informe ("mantiene 65 °C ± 2 °C con ciclo de trabajo
estable" en operación nominal al 60 %).

Además, esto es inconsistente con el propio informe: la sección 4.8 linealiza
la planta en el punto de operación (65 °C, 2000 RPM, 90 W) y obtiene
`Rth ≈ 0,6 °C/W` para ese punto. Físicamente, `Rth` tiene que ser **mayor** a
menor RPM (peor enfriamiento) y **menor** a mayor RPM (mejor enfriamiento) —
es decir, `Rth(800 RPM)` debería ser *mayor* que `Rth(2000 RPM)`, no menor.
Con `Rth_base = 0,3` ocurre lo contrario.

**Decisión adoptada (confirmada con el equipo):** se fijó `R_TH_BASE = 0,6 °C/W`
en ambos scripts, tomando el mismo valor que ya usa el informe para la
linealización en 4.8. Con este valor el sistema sí requiere modulación activa
del ventilador desde el 60 % de carga en adelante, reproduciendo fielmente la
idea central del informe (el PID mantiene la temperatura cerca del setpoint
en operación normal, no solo durante un ataque DoS). Esta constante es
independiente de `Kp/Ki/Kd` y de la tabla de Routh-Hurwitz (que usa el `Kt`
linealizado, no `Rth_base` directamente), así que el cambio no invalida el
análisis de estabilidad ya hecho en el informe.

**Pendiente para ustedes:** si quieren que el informe y el código queden
100 % alineados, conviene actualizar la tabla de parámetros de la sección 6.1
del documento (`Rth base = 0,3 °C/W` → `0,6 °C/W`) para que coincida con lo
que efectivamente corre esta simulación.

## 6. Otras diferencias menores frente a la sección 6.2/6.4 del informe

Con el modelo recalibrado, la temperatura converge correctamente al setpoint
y nunca se acerca a la zona de throttling (85 °C) en ningún escenario, que es
el resultado central que el informe necesita sostener. Sin embargo, dos
magnitudes puntuales del texto no se reproducen exactamente en la corrida
fija de `simulacion_pid.py`, y vale mencionarlo para que no genere sorpresas
si comparan número contra número:

- **Escenario de pico/DoS:** en el informe, el ataque DoS se describe como si
  produjera la mayor desviación de temperatura (~8 °C, pico ≈73 °C). En la
  simulación, como los escenarios están encadenados (60 %→90 %→100 %), el
  controlador ya viene compensando gran parte de la carga cuando llega el
  100 %, así que el salto de 90 % a 100 % genera una desviación chica. La
  mayor desviación real termina siendo la del arranque en frío y la del
  cambio combinado de carga+setpoint en t=300s (baja abrupta de 65 °C a
  55 °C de referencia). Esto no afecta la validez del diseño (el sistema
  responde correctamente a cualquier escalón), pero en `tablero_control.py`
  podés reproducir un DoS "aislado" y más dramático apretando directamente
  `DoS 100%` desde una carga baja, sin pasar antes por `Pico 90%`.
- Los valores puntuales de "ciclo de trabajo ≈45–50 %" y "RPM ≈1400" que
  cita la sección 6.4 para la operación nominal tampoco son mutuamente
  consistentes entre sí en el propio texto (45–50 % de PWM equivale a
  ≈2015–2150 RPM, no a 1400 RPM), así que no se tomaron como referencia para
  la calibración.

## 7. Convención de signo del error (fijada, no cambiar sin propagar)

```
e(t) = Tref − Tcpu(t)
```

Positivo cuando falta calentar (Tcpu < setpoint), negativo cuando hay
sobretemperatura (Tcpu > setpoint). Por eso la salida cruda del PID se usa
**negada** para construir la señal de control (sube el PWM con
sobretemperatura). Ver los comentarios junto al cálculo de `u_pre` en
`simulacion_pid.py` y `tablero_control.py`.

## 8. Estructura del repositorio

```
simulacion_pid.py    # Corrida unica de los 6 escenarios, guarda graficos .png
tablero_control.py   # Tablero interactivo en vivo (sliders + botones)
requirements.txt     # Dependencias (numpy, matplotlib)
resultados/           # Se genera al correr simulacion_pid.py (no versionado)
```
