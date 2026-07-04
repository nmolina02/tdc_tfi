"""
Tablero de control interactivo del lazo cerrado PID
(Control de velocidad de ventiladores en rack de servidores).

Trabajo Final Integrador - Teoria de Control (UTN-FRBA, K4011)
Ale Marino, Santiago | Molina, Nicolas Ariel

A diferencia de simulacion_pid.py (que corre los 6 escenarios de una vez y
guarda graficos estaticos), este script simula el lazo EN VIVO: el tiempo
avanza en tiempo real mientras la ventana esta abierta, y desde la misma
ventana se puede:

  - Mover los sliders de Kp, Ki, Kd, Setpoint, Carga de CPU, Temperatura
    ambiente y Velocidad de simulacion en cualquier momento, sin reiniciar.
  - Disparar perturbaciones puntuales con un boton (pico de trafico, ataque
    DoS, carga normal, falla de HVAC) en el instante exacto que se quiera.
  - Activar/desactivar el ruido del sensor LM35 para ver la diferencia entre
    la salida real de la planta y la senal de realimentacion que efectivamente
    usa el controlador.
  - Pausar/reanudar y resetear la simulacion.

Requiere un backend grafico interactivo (TkAgg, QtAgg, etc.), no funciona
con backends no interactivos como Agg. Ejecutar con:

    python tablero_control.py
"""

from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, CheckButtons
from matplotlib.animation import FuncAnimation

# ==============================================================
# Parametros fisicos de la planta (identicos a simulacion_pid.py)
# ==============================================================
C_TH = 50.0        # J/°C
Q_IDLE = 30.0      # W
Q_TDP = 150.0      # W
RTH_BASE = 0.6     # °C/W (ver README, seccion 6, para la justificacion de este valor)
RPM_MIN = 800.0
RPM_MAX = 3500.0

DT = 1.0            # s (Tscan del PLC S7-1200)
TAU_FILTRO_D = 5.0  # s - constante del filtro pasa-bajos de la rama derivativa (informe 4.13)

T_ALERTA = 75.0
T_THROTTLE = 85.0
T_MAX = 96.0

VENTANA_S = 400     # segundos de historia visibles en pantalla (ventana deslizante)

HVAC_RAMPA_DURACION = 100.0  # s, tiempo en el que la rampa de HVAC suma +10 °C
HVAC_RAMPA_DELTA = 10.0      # °C


def q_cpu(cpu_pct):
    return Q_IDLE + (Q_TDP - Q_IDLE) * (cpu_pct / 100.0)


def r_th(rpm):
    rpm_clamp = max(RPM_MIN, min(RPM_MAX, rpm))
    return RTH_BASE * (RPM_MIN / rpm_clamp)


class EstadoSimulacion:
    """Guarda el estado del lazo (planta + PID) y el historial para graficar."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.t = 0.0
        self.T_cpu = 22.0
        self.integral = 0.0
        self.error_filt = 0.0
        self.error_filt_prev = 0.0

        self.hvac_rampa_activa = False
        self.hvac_rampa_t0 = 0.0
        self.hvac_rampa_val0 = 22.0

        n = VENTANA_S
        claves = ["t", "tref", "tcpu", "tmed", "error", "pwm",
                  "rpm_cmd", "rpm_med", "cpu", "tamb", "p", "i", "d"]
        self.hist = {k: deque(maxlen=n) for k in claves}

    def paso(self, kp, ki, kd, tref, cpu_pct, tamb, ruido_on):
        """Avanza un paso de tiempo DT del lazo cerrado con los parametros
        actuales (leidos en vivo desde los sliders del tablero)."""
        ruido_sensor = np.random.normal(0.0, 0.5) if ruido_on else 0.0
        T_medido = self.T_cpu + ruido_sensor

        # e(t) = Tref - Tcpu_medido: convencion fijada en todo el TP.
        # positivo = falta calentar, negativo = sobretemperatura.
        error = tref - T_medido

        # Filtro pasa-bajos de 1er orden en la rama derivativa (informe 4.13),
        # para no amplificar el ruido del LM35 con la accion D.
        alpha = DT / (TAU_FILTRO_D + DT)
        self.error_filt += alpha * (error - self.error_filt)
        derivative = (self.error_filt - self.error_filt_prev) / DT

        p_term = kp * error
        i_term = ki * self.integral
        d_term = kd * derivative

        u_pre = -(p_term + i_term + d_term)
        u_sat = max(0.0, min(100.0, u_pre))

        # Anti-windup por clamping.
        if (0.0 < u_sat < 100.0) or (u_sat == 100.0 and error > 0) or (u_sat == 0.0 and error < 0):
            self.integral += error * DT

        rpm_cmd = RPM_MIN + (u_sat / 100.0) * (RPM_MAX - RPM_MIN)
        ruido_rpm = np.random.normal(0.0, 15.0) if ruido_on else 0.0
        rpm_medida = max(0.0, rpm_cmd + ruido_rpm)  # segunda rama: sensor Hall

        # tcpu_actual y T_medido corresponden al mismo instante (antes de
        # integrar la planta un paso mas), para que ambas curvas queden
        # alineadas en el tiempo y solo difieran por el ruido del sensor.
        tcpu_actual = self.T_cpu
        dT = (q_cpu(cpu_pct) - (tcpu_actual - tamb) / r_th(rpm_cmd)) * DT / C_TH
        self.T_cpu += dT

        self.error_filt_prev = self.error_filt
        self.t += DT

        h = self.hist
        h["t"].append(self.t)
        h["tref"].append(tref)
        h["tcpu"].append(tcpu_actual)
        h["tmed"].append(T_medido)
        h["error"].append(error)
        h["pwm"].append(u_sat)
        h["rpm_cmd"].append(rpm_cmd)
        h["rpm_med"].append(rpm_medida)
        h["cpu"].append(cpu_pct)
        h["tamb"].append(tamb)
        h["p"].append(p_term)
        h["i"].append(i_term)
        h["d"].append(d_term)


# ==============================================================
# Figura y ejes
# ==============================================================
estado = EstadoSimulacion()
corriendo = [True]

fig = plt.figure(figsize=(16, 9.5))
fig.suptitle("Tablero de control - Lazo PID de velocidad de ventiladores (rack de servidores)",
             fontsize=12, fontweight="bold")

gs = fig.add_gridspec(5, 1, left=0.06, right=0.70, top=0.92, bottom=0.06, hspace=0.55)
ax_temp = fig.add_subplot(gs[0])
ax_err = fig.add_subplot(gs[1], sharex=ax_temp)
ax_pwm = fig.add_subplot(gs[2], sharex=ax_temp)
ax_rpm = fig.add_subplot(gs[3], sharex=ax_temp)
ax_pert = fig.add_subplot(gs[4], sharex=ax_temp)

(l_tref,) = ax_temp.plot([], [], "--", color="black", label="Referencia (Tref)")
(l_tcpu,) = ax_temp.plot([], [], "-", color="tab:red", label="Salida real (Tcpu)")
(l_tmed,) = ax_temp.plot([], [], ":", color="tab:orange", linewidth=1.2, label="Realimentación (LM35)")
ax_temp.axhline(T_ALERTA, linestyle=":", color="orange", linewidth=0.8)
ax_temp.axhline(T_THROTTLE, linestyle=":", color="darkred", linewidth=0.8)
ax_temp.set_ylabel("Temp. [°C]")
ax_temp.set_title("Temperatura: referencia / salida / realimentación", fontsize=9)
ax_temp.legend(loc="upper left", fontsize=7, ncol=3)
ax_temp.grid(True, alpha=0.4)
ax_temp.set_ylim(15, 90)

(l_err,) = ax_err.plot([], [], color="tab:purple", label="e(t) = Tref - Tmedido")
ax_err.axhline(0, color="black", linewidth=0.7)
ax_err.set_ylabel("Error [°C]")
ax_err.set_title("Señal de error", fontsize=9)
ax_err.grid(True, alpha=0.4)

(l_pwm,) = ax_pwm.plot([], [], color="tab:blue", label="u(t) PWM")
ax_pwm.set_ylabel("PWM [%]")
ax_pwm.set_title("Señal de control (ciclo de trabajo)", fontsize=9)
ax_pwm.set_ylim(-5, 105)
ax_pwm.grid(True, alpha=0.4)

(l_rpmc,) = ax_rpm.plot([], [], color="tab:green", label="RPM comandada")
(l_rpmm,) = ax_rpm.plot([], [], ":", color="tab:olive", linewidth=1.0, label="RPM medida (sensor Hall)")
ax_rpm.set_ylabel("RPM")
ax_rpm.set_title("Velocidad del ventilador (verificación por 2ª realimentación)", fontsize=9)
ax_rpm.legend(loc="upper left", fontsize=7, ncol=2)
ax_rpm.grid(True, alpha=0.4)

(l_cpu,) = ax_pert.plot([], [], color="tab:brown", label="Carga CPU [%]")
ax_pert2 = ax_pert.twinx()
(l_tamb,) = ax_pert2.plot([], [], color="tab:cyan", label="Tamb [°C]")
ax_pert.set_ylabel("Carga CPU [%]", color="tab:brown")
ax_pert2.set_ylabel("Tamb [°C]", color="tab:cyan")
ax_pert.set_xlabel("Tiempo [s]")
ax_pert.set_title("Perturbaciones aplicadas", fontsize=9)
ax_pert.grid(True, alpha=0.4)

texto_estado = fig.text(0.72, 0.90, "", fontsize=9, family="monospace", va="top")

# ==============================================================
# Panel de control (sliders, botones, checkbox)
# ==============================================================
def eje_control(bottom, height=0.025):
    return fig.add_axes([0.76, bottom, 0.20, height])


fig.text(0.76, 0.60, "Parámetros del PID", fontsize=10, fontweight="bold")
s_kp = Slider(eje_control(0.565), "Kp", 0.0, 20.0, valinit=5.0, valstep=0.1)
s_ki = Slider(eje_control(0.530), "Ki", 0.0, 2.0, valinit=0.3, valstep=0.01)
s_kd = Slider(eje_control(0.495), "Kd", 0.0, 10.0, valinit=2.0, valstep=0.1)

fig.text(0.76, 0.455, "Referencia y perturbaciones manuales", fontsize=10, fontweight="bold")
s_tref = Slider(eje_control(0.420), "Setpoint [°C]", 40.0, 80.0, valinit=65.0, valstep=1.0)
s_cpu = Slider(eje_control(0.385), "Carga CPU [%]", 0.0, 100.0, valinit=30.0, valstep=1.0)
s_tamb = Slider(eje_control(0.350), "Tamb [°C]", 15.0, 35.0, valinit=22.0, valstep=0.5)
s_vel = Slider(eje_control(0.315), "Vel. sim [s/frame]", 1.0, 20.0, valinit=5.0, valstep=1.0)

s_tamb.on_changed(lambda val: setattr(estado, "hvac_rampa_activa", False))

chk_ruido = CheckButtons(fig.add_axes([0.76, 0.265, 0.20, 0.035]),
                          ["Ruido de sensor (LM35 ±0.5°C)"], [True])

fig.text(0.76, 0.235, "Perturbaciones puntuales (botón)", fontsize=10, fontweight="bold")
b_pico = Button(fig.add_axes([0.76, 0.195, 0.09, 0.035]), "Pico 90%")
b_dos = Button(fig.add_axes([0.87, 0.195, 0.09, 0.035]), "DoS 100%")
b_normal = Button(fig.add_axes([0.76, 0.150, 0.09, 0.035]), "Carga 30%")
b_hvac = Button(fig.add_axes([0.87, 0.150, 0.09, 0.035]), "Falla HVAC")

b_pausa = Button(fig.add_axes([0.76, 0.095, 0.09, 0.04]), "Pausar")
b_reset = Button(fig.add_axes([0.87, 0.095, 0.09, 0.04]), "Reset")


def on_pico(_event):
    s_cpu.set_val(90.0)


def on_dos(_event):
    s_cpu.set_val(100.0)


def on_normal(_event):
    s_cpu.set_val(30.0)


def on_hvac(_event):
    estado.hvac_rampa_activa = True
    estado.hvac_rampa_t0 = estado.t
    estado.hvac_rampa_val0 = s_tamb.val


def on_pausa(_event):
    corriendo[0] = not corriendo[0]
    b_pausa.label.set_text("Reanudar" if not corriendo[0] else "Pausar")


def on_reset(_event):
    estado.reset()
    s_kp.reset(); s_ki.reset(); s_kd.reset()
    s_tref.reset(); s_cpu.reset(); s_tamb.reset(); s_vel.reset()


b_pico.on_clicked(on_pico)
b_dos.on_clicked(on_dos)
b_normal.on_clicked(on_normal)
b_hvac.on_clicked(on_hvac)
b_pausa.on_clicked(on_pausa)
b_reset.on_clicked(on_reset)


def tamb_actual():
    if estado.hvac_rampa_activa:
        delta = min(HVAC_RAMPA_DELTA, (estado.t - estado.hvac_rampa_t0) * (HVAC_RAMPA_DELTA / HVAC_RAMPA_DURACION))
        if estado.t - estado.hvac_rampa_t0 >= HVAC_RAMPA_DURACION:
            estado.hvac_rampa_activa = False
            s_tamb.set_val(estado.hvac_rampa_val0 + HVAC_RAMPA_DELTA)
        return estado.hvac_rampa_val0 + delta
    return s_tamb.val


def actualizar(_frame):
    if corriendo[0]:
        n_pasos = int(s_vel.val)
        ruido_on = chk_ruido.get_status()[0]
        for _ in range(n_pasos):
            estado.paso(s_kp.val, s_ki.val, s_kd.val, s_tref.val, s_cpu.val, tamb_actual(), ruido_on)

    h = estado.hist
    t = list(h["t"])
    if t:
        l_tref.set_data(t, h["tref"])
        l_tcpu.set_data(t, h["tcpu"])
        l_tmed.set_data(t, h["tmed"])
        l_err.set_data(t, h["error"])
        l_pwm.set_data(t, h["pwm"])
        l_rpmc.set_data(t, h["rpm_cmd"])
        l_rpmm.set_data(t, h["rpm_med"])
        l_cpu.set_data(t, h["cpu"])
        l_tamb.set_data(t, h["tamb"])

        for ax in (ax_err, ax_pwm, ax_rpm, ax_pert, ax_pert2):
            ax.relim()
            ax.autoscale_view()
        ax_temp.set_xlim(t[0], t[-1] if t[-1] > t[0] else t[0] + 1)

        estado_texto = (
            f"t     = {estado.t:7.1f} s\n"
            f"Tref  = {h['tref'][-1]:7.2f} C\n"
            f"Tcpu  = {h['tcpu'][-1]:7.2f} C\n"
            f"Tmed  = {h['tmed'][-1]:7.2f} C\n"
            f"error = {h['error'][-1]:7.2f} C\n"
            f"PWM   = {h['pwm'][-1]:7.1f} %\n"
            f"RPM   = {h['rpm_cmd'][-1]:7.0f}\n"
            f"---- aporte PID (antes de saturar) ----\n"
            f"P = {h['p'][-1]:7.2f}\n"
            f"I = {h['i'][-1]:7.2f}\n"
            f"D = {h['d'][-1]:7.2f}\n"
        )
        if h["tcpu"][-1] >= T_THROTTLE:
            estado_texto += "\n*** THROTTLING ***"
        elif h["tcpu"][-1] >= T_ALERTA:
            estado_texto += "\n*** ALERTA ***"
        texto_estado.set_text(estado_texto)

    return (l_tref, l_tcpu, l_tmed, l_err, l_pwm, l_rpmc, l_rpmm, l_cpu, l_tamb)


ani = FuncAnimation(fig, actualizar, interval=100, blit=False, cache_frame_data=False)

if __name__ == "__main__":
    plt.show()
