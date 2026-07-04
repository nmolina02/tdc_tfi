"""
Modelo compartido del lazo cerrado PID (planta + controlador).

Trabajo Final Integrador - Teoria de Control (UTN-FRBA, K4011)
Ale Marino, Santiago | Molina, Nicolas Ariel

Este modulo es la UNICA fuente de verdad de la fisica y del controlador:
las constantes y el paso de integracion viven aca y los consume el tablero
interactivo (tablero_qt.py). Asi no hay constantes duplicadas que puedan
divergir entre archivos.

Convencion de signo del error (fijada en todo el trabajo):
    e(t) = Tref - Tcpu(t)
    positivo  -> falta calentar (se puede bajar RPM)
    negativo  -> sobretemperatura (hay que subir RPM)
Por eso la salida cruda del PID se USA NEGADA para construir la senal de
control (sube el ciclo de trabajo cuando e(t) < 0).
"""

# ==============================================================
# Parametros fisicos de la planta (seccion 4.8 / 6.1 del informe)
# ==============================================================
C_TH = 50.0        # J/°C   - capacidad termica CPU + disipador
Q_IDLE = 30.0      # W      - disipacion en idle
Q_TDP = 150.0      # W      - disipacion a carga plena (TDP)
# Resistencia termica de referencia a RPM_MIN (peor caso de enfriamiento).
# Se fija en 0.6 °C/W para ser consistente con el Rth ≈ 0.6 usado en la
# seccion 4.8 del informe al linealizar Gt(s) en el punto de operacion
# (65 °C, 2000 RPM, 90 W). Con 0.3 (valor de la tabla 6.1) el PID nunca
# sale de saturacion minima por debajo del ~95% de carga. Ver README, sec. 5.
RTH_BASE = 0.6     # °C/W
RPM_MIN = 800.0    # RPM minimas (ventiladores nunca se apagan)
RPM_MAX = 3500.0   # RPM maximas

T_AMB_INICIAL = 22.0   # °C
T_CPU_INICIAL = 22.0   # °C (cold start)

# ==============================================================
# Controlador PID, notacion paralela (Gc = Kp + Ki/s + Kd*s).
# Sintonizacion validada: NO modificar sin recalcular Routh-Hurwitz (4.11).
# ==============================================================
KP = 5.0
KI = 0.3
KD = 2.0
T_SETPOINT_DEFAULT = 65.0

# ==============================================================
# Limites operativos de temperatura (seccion 4.1)
# ==============================================================
T_ALERTA = 75.0     # °C - inicio de zona de alerta
T_THROTTLE = 85.0   # °C - thermal throttling
T_MAX = 96.0        # °C - Tj max / apagado de emergencia

# ==============================================================
# Discretizacion y ruido de medicion
# ==============================================================
DT = 1.0             # s  (Tscan del PLC S7-1200)
TAU_FILTRO_D = 5.0   # s  - filtro pasa-bajos de la rama derivativa (informe 4.13)
RUIDO_LM35_STD = 0.5     # °C  - desvio del ruido del sensor de temperatura
RUIDO_HALL_STD = 15.0    # RPM - desvio del ruido del sensor Hall


def q_cpu(cpu_pct: float) -> float:
    """Potencia disipada por la CPU en funcion del % de utilizacion."""
    return Q_IDLE + (Q_TDP - Q_IDLE) * (cpu_pct / 100.0)


def r_th(rpm: float) -> float:
    """Resistencia termica efectiva: disminuye al aumentar el flujo de aire."""
    rpm_clamp = max(RPM_MIN, min(RPM_MAX, rpm))
    return RTH_BASE * (RPM_MIN / rpm_clamp)


class Simulador:
    """Estado del lazo cerrado (planta termica + PID discreto con anti-windup).

    Cada llamada a paso() avanza DT segundos: lee la temperatura actual,
    calcula la accion de control con los parametros dados (que pueden variar
    en vivo) e integra la planta un paso. Con ruido=False el lazo es
    determinista. La rama derivativa siempre lleva filtro pasa-bajos (4.13).
    """

    def __init__(self, t_cpu_inicial: float = T_CPU_INICIAL):
        self.reset(t_cpu_inicial)

    def reset(self, t_cpu_inicial: float = T_CPU_INICIAL):
        self.T_cpu = t_cpu_inicial
        self.integral = 0.0
        self.error_filt = 0.0
        self.error_filt_prev = 0.0

    def paso(self, setpoint, cpu_pct, tamb, kp, ki, kd,
             dt=DT, ruido=False, rng=None):
        # --- Rama de realimentacion principal: sensor LM35 ---
        ruido_sensor = rng.normal(0.0, RUIDO_LM35_STD) if (ruido and rng is not None) else 0.0
        T_medido = self.T_cpu + ruido_sensor

        # e(t) = Tref - Tcpu_medido (positivo = falta calentar)
        error = setpoint - T_medido

        # Filtro pasa-bajos de 1er orden en la rama D (informe 4.13), para no
        # amplificar el ruido del LM35 con la accion derivativa.
        alpha = dt / (TAU_FILTRO_D + dt)
        self.error_filt += alpha * (error - self.error_filt)
        derivative = (self.error_filt - self.error_filt_prev) / dt

        p_term = kp * error
        i_term = ki * self.integral
        d_term = kd * derivative

        # Salida cruda del PID negada: sube el ciclo de trabajo con sobretemperatura
        u_pre = -(p_term + i_term + d_term)
        u_sat = max(0.0, min(100.0, u_pre))

        # Anti-windup por clamping (logica ajustada a e = Tref - Tcpu)
        if (0.0 < u_sat < 100.0) or (u_sat == 100.0 and error > 0) or (u_sat == 0.0 and error < 0):
            self.integral += error * dt

        rpm_cmd = RPM_MIN + (u_sat / 100.0) * (RPM_MAX - RPM_MIN)

        # --- Segunda rama de realimentacion: sensor Hall (verificacion) ---
        ruido_rpm = rng.normal(0.0, RUIDO_HALL_STD) if (ruido and rng is not None) else 0.0
        rpm_medida = max(0.0, rpm_cmd + ruido_rpm)

        # Integracion de la planta termica un paso (Euler hacia adelante)
        dT = (q_cpu(cpu_pct) - (self.T_cpu - tamb) / r_th(rpm_cmd)) * dt / C_TH
        self.T_cpu += dT

        self.error_filt_prev = self.error_filt

        return {
            "T_cpu": self.T_cpu,     # temperatura ya integrada (nuevo valor)
            "T_medido": T_medido,    # lo que "ve" el PLC (con ruido si esta activo)
            "error": error,
            "u_sat": u_sat,
            "rpm_cmd": rpm_cmd,
            "rpm_medida": rpm_medida,
            "p": p_term, "i": i_term, "d": d_term,
        }
