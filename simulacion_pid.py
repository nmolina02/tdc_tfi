"""
Simulacion del lazo cerrado de control de velocidad de ventiladores
en un rack de servidores mediante controlador PID.

Trabajo Final Integrador - Teoria de Control (UTN-FRBA, K4011)
Ale Marino, Santiago | Molina, Nicolas Ariel

Modelo: cascada motor (PWM->RPM) + proceso termico (RPM->Tcpu), ambos
de primer orden, con PID discreto (Tscan = 1 s) y anti-windup por
clamping. Reproduce los 6 escenarios descriptos en la seccion 6.2 del
informe (arranque, nominal, pico de trafico, ataque DoS, cambio de
setpoint y falla parcial de HVAC) en una unica linea de tiempo continua
de 500 s, tal como se valido en el documento.

Convencion de signo del error (fijada en todo el trabajo):
    e(t) = Tref - Tcpu(t)
    positivo  -> falta calentar (se puede bajar RPM)
    negativo  -> sobretemperatura (hay que subir RPM)
Por eso la salida cruda del PID se USA NEGADA para construir la senal
de control (sube el ciclo de trabajo cuando e(t) < 0).
"""

import os

import numpy as np
import matplotlib.pyplot as plt

# ==============================================================
# Parametros de la planta (seccion 4.8 / 6.1 del informe)
# ==============================================================
C_TH = 50.0        # J/°C   - capacidad termica CPU + disipador
Q_IDLE = 30.0      # W      - disipacion en idle
Q_TDP = 150.0      # W      - disipacion a carga plena (TDP)
# Resistencia termica de referencia a RPM_MIN (peor caso de enfriamiento).
# NOTA: se fija en 0.6 °C/W para que sea consistente con el Rth ≈ 0.6 °C/W
# ya usado en la seccion 4.8 del informe para linealizar Gt(s) en el punto
# de operacion (65 °C, 2000 RPM, 90 W). Con R_th_base = 0.3 °C/W (valor que
# figura en la tabla de parametros de la seccion 6.1) la temperatura de
# equilibrio pasivo a RPM minima ya queda por debajo del setpoint para
# cualquier carga de CPU menor al ~95%, y el PID nunca sale de saturacion
# minima: el sistema jamas ejerce control activo en los escenarios de
# operacion nominal o pico de trafico, lo cual contradice la seccion 6.2/6.4
# del informe (ver discusion en el README). Con 0.6 °C/W el ventilador debe
# modular activamente su velocidad ya desde el 60% de carga.
RTH_BASE = 0.6     # °C/W
RPM_MIN = 800.0    # RPM minimas (ventiladores nunca se apagan)
RPM_MAX = 3500.0   # RPM maximas

T_AMB_INICIAL = 22.0   # °C
T_CPU_INICIAL = 22.0   # °C (cold start)

# ==============================================================
# Parametros del controlador PID, en notacion paralela
# (Gc(s) = Kp + Ki/s + Kd*s), que es la que usa directamente el
# codigo. Equivale a la notacion de catedra Kp=5.0, Ti≈3.33s,
# Td=2.0s usada en el informe (seccion 4.6) para el analisis de
# Routh-Hurwitz. Sintonizacion validada: NO modificar sin volver a
# correr todos los escenarios y recalcular Routh-Hurwitz (4.11).
# ==============================================================
KP = 5.0
KI = 0.3
KD = 2.0

T_SETPOINT_INICIAL = 65.0
T_SETPOINT_FINAL = 55.0

# ==============================================================
# Limites operativos de temperatura (seccion 4.1)
# ==============================================================
T_ALERTA = 75.0     # °C - inicio de zona de alerta
T_THROTTLE = 85.0   # °C - thermal throttling
T_MAX = 96.0        # °C - Tj max / apagado de emergencia

# ==============================================================
# Linea de tiempo de simulacion y limites de cada escenario
# ==============================================================
DT = 1.0        # s  (Tscan del PLC S7-1200)
T_TOTAL = 500   # s

T_ARRANQUE_FIN = 50    # 0-50s:   arranque / baja carga (30% CPU)
T_NOMINAL_FIN = 100    # 50-100s: operacion nominal (60% CPU)
T_PICO_FIN = 200        # 100-200s: pico de trafico (90% CPU)
T_DOS_FIN = 300         # 200-300s: ataque DoS (100% CPU)
T_HVAC_INICIO = 400     # 300-400s: recuperacion + cambio de setpoint (40% CPU, setpoint 55°C)
                        # 400-500s: falla parcial de HVAC (rampa Tamb 22->32°C)

ESCENARIOS = [
    (0, T_ARRANQUE_FIN, "1. Arranque (cold start)"),
    (T_ARRANQUE_FIN, T_NOMINAL_FIN, "2. Operacion nominal (60% CPU)"),
    (T_NOMINAL_FIN, T_PICO_FIN, "3. Pico de trafico (60%->90% CPU)"),
    (T_PICO_FIN, T_DOS_FIN, "4. Ataque DoS (100% CPU)"),
    (T_DOS_FIN, T_HVAC_INICIO, "5. Recuperacion + cambio setpoint (65->55 C)"),
    (T_HVAC_INICIO, T_TOTAL, "6. Falla parcial de HVAC (rampa Tamb 22->32 C)"),
]


def q_cpu(cpu_pct: float) -> float:
    """Potencia disipada por la CPU en funcion del % de utilizacion."""
    return Q_IDLE + (Q_TDP - Q_IDLE) * (cpu_pct / 100.0)


def r_th(rpm: float) -> float:
    """Resistencia termica efectiva: disminuye al aumentar el flujo de aire."""
    rpm_clamp = max(RPM_MIN, min(RPM_MAX, rpm))
    return RTH_BASE * (RPM_MIN / rpm_clamp)


def perfil_cpu(t: float) -> float:
    """Perfil de carga de CPU [%] a lo largo de los 6 escenarios."""
    if t < T_ARRANQUE_FIN:
        return 30.0
    if t < T_NOMINAL_FIN:
        return 60.0
    if t < T_PICO_FIN:
        return 90.0
    if t < T_DOS_FIN:
        return 100.0
    return 40.0


def perfil_setpoint(t: float) -> float:
    """Setpoint de temperatura [°C]: cae a 55 C junto con la recuperacion de carga."""
    return T_SETPOINT_INICIAL if t < T_DOS_FIN else T_SETPOINT_FINAL


def perfil_tamb(t: float) -> float:
    """Temperatura ambiente [°C]: rampa de falla parcial de HVAC desde t=400s."""
    if t < T_HVAC_INICIO:
        return T_AMB_INICIAL
    return T_AMB_INICIAL + min(10.0, (t - T_HVAC_INICIO) * 0.1)


def simular():
    """Corre el lazo cerrado completo y devuelve las series temporales."""
    tiempo = np.arange(0, T_TOTAL, DT)
    n = len(tiempo)

    T_cpu = np.zeros(n)
    pwm = np.zeros(n)
    rpm_hist = np.zeros(n)
    error_hist = np.zeros(n)
    setpoint_hist = np.zeros(n)
    tamb_hist = np.zeros(n)
    cpu_hist = np.zeros(n)

    T_cpu[0] = T_CPU_INICIAL
    integral = 0.0
    error_prev = 0.0

    setpoint_hist[0] = perfil_setpoint(0)
    tamb_hist[0] = perfil_tamb(0)
    cpu_hist[0] = perfil_cpu(0)

    for k in range(1, n):
        t = tiempo[k]
        setpoint = perfil_setpoint(t)
        tamb = perfil_tamb(t)
        cpu = perfil_cpu(t)

        # e(t) = Tref - Tcpu: positivo = falta calentar, negativo = sobretemperatura
        error = setpoint - T_cpu[k - 1]
        derivative = (error - error_prev) / DT

        # Salida cruda del PID negada: sube el ciclo de trabajo con sobretemperatura
        u_pre = -(KP * error + KI * integral + KD * derivative)
        u_sat = max(0.0, min(100.0, u_pre))

        # Anti-windup por clamping (logica ajustada a la convencion de signo de arriba)
        if (0.0 < u_sat < 100.0) or (u_sat == 100.0 and error > 0) or (u_sat == 0.0 and error < 0):
            integral += error * DT

        rpm = RPM_MIN + (u_sat / 100.0) * (RPM_MAX - RPM_MIN)
        dT = (q_cpu(cpu) - (T_cpu[k - 1] - tamb) / r_th(rpm)) * DT / C_TH

        T_cpu[k] = T_cpu[k - 1] + dT
        pwm[k] = u_sat
        rpm_hist[k] = rpm
        error_hist[k] = error
        setpoint_hist[k] = setpoint
        tamb_hist[k] = tamb
        cpu_hist[k] = cpu
        error_prev = error

    rpm_hist[0] = RPM_MIN
    return {
        "t": tiempo, "T_cpu": T_cpu, "pwm": pwm, "rpm": rpm_hist,
        "error": error_hist, "setpoint": setpoint_hist, "tamb": tamb_hist,
        "cpu": cpu_hist,
    }


def imprimir_metricas(res):
    """Resumen numerico por escenario para contrastar contra la seccion 6.4 del informe."""
    t, T_cpu, error = res["t"], res["T_cpu"], res["error"]

    print("=" * 72)
    print("RESUMEN DE LA SIMULACION (500 s, 6 escenarios)")
    print("=" * 72)
    print(f"{'Escenario':<45}{'T fin [C]':>10}{'T max [C]':>10}{'e fin [C]':>10}")
    print("-" * 72)
    for t_ini, t_fin, nombre in ESCENARIOS:
        mask = (t >= t_ini) & (t < t_fin)
        t_final_ventana = T_cpu[mask][-1]
        t_max_ventana = T_cpu[mask].max()
        e_final_ventana = error[mask][-1]
        print(f"{nombre:<45}{t_final_ventana:>10.2f}{t_max_ventana:>10.2f}{e_final_ventana:>10.2f}")
    print("-" * 72)
    print(f"Temperatura maxima global: {T_cpu.max():.2f} C  "
          f"(umbral throttling: {T_THROTTLE:.0f} C, Tj max: {T_MAX:.0f} C)")
    print(f"Temperatura minima global: {T_cpu.min():.2f} C")
    if T_cpu.max() < T_THROTTLE:
        print("OK: el sistema nunca alcanza la zona de throttling en ningun escenario.")
    else:
        print("ALERTA: el sistema supero el umbral de throttling en algun instante.")
    print("=" * 72)


def graficar(res, carpeta_salida="resultados"):
    os.makedirs(carpeta_salida, exist_ok=True)
    t = res["t"]

    def marcar_escenarios(ax):
        for t_ini, _, _ in ESCENARIOS[1:]:
            ax.axvline(t_ini, color="grey", linestyle=":", linewidth=0.8)

    fig, axs = plt.subplots(4, 1, figsize=(11, 13), sharex=True)

    axs[0].plot(t, res["T_cpu"], label="Tcpu", color="tab:red")
    axs[0].plot(t, res["setpoint"], label="Setpoint", linestyle="--", color="black")
    axs[0].axhline(T_ALERTA, linestyle=":", color="orange", label="Alerta (75 C)")
    axs[0].axhline(T_THROTTLE, linestyle=":", color="darkred", label="Throttling (85 C)")
    axs[0].set_ylabel("Temperatura [°C]")
    axs[0].set_title("Temperatura del procesador vs. setpoint")
    axs[0].legend(loc="upper right", fontsize=8)
    axs[0].grid(True, alpha=0.4)
    marcar_escenarios(axs[0])

    axs[1].plot(t, res["pwm"], color="tab:blue")
    axs[1].set_ylabel("PWM [%]")
    axs[1].set_title("Senal de control (ciclo de trabajo)")
    axs[1].grid(True, alpha=0.4)
    marcar_escenarios(axs[1])

    axs[2].plot(t, res["rpm"], color="tab:green")
    axs[2].set_ylabel("RPM")
    axs[2].set_title("Velocidad del ventilador")
    axs[2].grid(True, alpha=0.4)
    marcar_escenarios(axs[2])

    axs[3].plot(t, res["error"], color="tab:purple")
    axs[3].axhline(0, color="black", linewidth=0.8)
    axs[3].set_ylabel("Error [°C]")
    axs[3].set_xlabel("Tiempo [s]")
    axs[3].set_title("Senal de error e(t) = Tref - Tcpu")
    axs[3].grid(True, alpha=0.4)
    marcar_escenarios(axs[3])

    fig.tight_layout()
    fig.savefig(os.path.join(carpeta_salida, "01_lazo_de_control.png"), dpi=150)

    fig2, ax2 = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
    ax2[0].plot(t, res["cpu"], color="tab:brown")
    ax2[0].set_ylabel("Carga CPU [%]")
    ax2[0].set_title("Perturbaciones aplicadas")
    ax2[0].grid(True, alpha=0.4)
    marcar_escenarios(ax2[0])

    ax2[1].plot(t, res["tamb"], color="tab:cyan")
    ax2[1].set_ylabel("Tamb [°C]")
    ax2[1].set_xlabel("Tiempo [s]")
    ax2[1].grid(True, alpha=0.4)
    marcar_escenarios(ax2[1])

    fig2.tight_layout()
    fig2.savefig(os.path.join(carpeta_salida, "02_perturbaciones.png"), dpi=150)

    print(f"Graficos guardados en: {os.path.abspath(carpeta_salida)}")
    plt.show()


if __name__ == "__main__":
    resultados = simular()
    imprimir_metricas(resultados)
    graficar(resultados)
