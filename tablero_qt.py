"""
Tablero de control interactivo del lazo PID (version PySide6 + pyqtgraph).

Trabajo Final Integrador - Teoria de Control (UTN-FRBA, K4011)
Ale Marino, Santiago | Molina, Nicolas Ariel

UI de escritorio fluida: graficos en tiempo real acelerados por pyqtgraph y
controles nativos de Qt. El tiempo avanza en vivo mientras la ventana esta
abierta; los sliders y botones se aplican al instante, sin reiniciar.

Toda la fisica y el PID viven en modelo.py (fuente unica de verdad); este
archivo es solo la capa de UI.

Ejecutar:
    python tablero_qt.py
"""

import sys
from collections import deque

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtCore import Qt

from modelo import (
    T_AMB_INICIAL, T_CPU_INICIAL,
    KP, KI, KD, T_SETPOINT_DEFAULT,
    T_ALERTA, T_THROTTLE, DT,
    Simulador,
)

# ==============================================================
# Apariencia global
# ==============================================================
pg.setConfigOptions(antialias=False, useOpenGL=False)
pg.setConfigOption("background", "#12131a")
pg.setConfigOption("foreground", "#c8ccd8")

VENTANA_S = 400          # segundos de historia visibles (ventana deslizante)
FRAME_MS = 40            # periodo de refresco de pantalla (~25 FPS)
HVAC_RAMPA_DURACION = 100.0
HVAC_RAMPA_DELTA = 10.0

COL_TREF = "#e0e0e0"
COL_TCPU = "#ff5d5d"
COL_TMED = "#ffb347"
COL_ERR = "#b48ead"
COL_PWM = "#5aa9e6"
COL_RPMC = "#8bd450"
COL_RPMM = "#c9d94a"
COL_CPU = "#c98a5a"
COL_TAMB = "#4ec9c9"


def _ref(color, texto):
    """Chip de referencia con color, para incrustar en el título del gráfico."""
    return f"<span style='color:{color}'>&#9632; {texto}</span>"


QSS = """
QWidget { background-color: #12131a; color: #c8ccd8;
          font-family: 'Segoe UI', 'Helvetica Neue', sans-serif; font-size: 11px; }
QGroupBox { border: 1px solid #2a2d3a; border-radius: 6px; margin-top: 7px;
            padding: 5px 8px 5px 8px; font-weight: 600; }
QGroupBox::title { subcontrol-origin: margin; left: 9px; padding: 0 4px; color: #8ea0c0; }
QPushButton { background-color: #232637; border: 1px solid #333850; border-radius: 6px;
              padding: 4px 9px; }
QPushButton:hover { background-color: #2c3145; }
QPushButton:pressed { background-color: #3a4160; }
QPushButton#accent { background-color: #2f6f9f; border: none; }
QPushButton#accent:hover { background-color: #397fb3; }
QPushButton#danger { background-color: #8a3b45; border: none; }
QPushButton#danger:hover { background-color: #a04651; }
QSlider::groove:horizontal { height: 4px; background: #2a2d3a; border-radius: 2px; }
QSlider::handle:horizontal { width: 14px; margin: -6px 0; border-radius: 7px;
                             background: #5aa9e6; }
QCheckBox { spacing: 8px; }
QLabel#readout { font-family: 'JetBrains Mono','Menlo','Consolas',monospace; font-size: 11px; }
"""


class ParamSlider(QtWidgets.QWidget):
    """Slider de valor flotante con etiqueta de nombre y valor actual."""

    changed = QtCore.Signal(float)

    def __init__(self, nombre, lo, hi, step, init, fmt="{:.2f}", unidad=""):
        super().__init__()
        self._lo, self._step, self._fmt, self._unidad = lo, step, fmt, unidad
        self._n = int(round((hi - lo) / step))

        self.slider = QtWidgets.QSlider(Qt.Horizontal)
        self.slider.setRange(0, self._n)
        self.slider.setValue(int(round((init - lo) / step)))
        self.slider.valueChanged.connect(self._on_change)

        self.lbl_nombre = QtWidgets.QLabel(nombre)
        self.lbl_valor = QtWidgets.QLabel()
        self.lbl_valor.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.lbl_valor.setMinimumWidth(64)
        self.lbl_valor.setStyleSheet("color:#e6e9f2; font-weight:600;")

        fila = QtWidgets.QHBoxLayout()
        fila.setContentsMargins(0, 0, 0, 0)
        fila.addWidget(self.lbl_nombre)
        fila.addStretch(1)
        fila.addWidget(self.lbl_valor)

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(2, 1, 2, 1)
        lay.setSpacing(1)
        lay.addLayout(fila)
        lay.addWidget(self.slider)
        self._refresh_label()

    def value(self) -> float:
        return self._lo + self.slider.value() * self._step

    def set_value(self, v: float):
        self.slider.setValue(int(round((v - self._lo) / self._step)))

    def _refresh_label(self):
        self.lbl_valor.setText(self._fmt.format(self.value()) + self._unidad)

    def _on_change(self, _):
        self._refresh_label()
        self.changed.emit(self.value())


class Tablero(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Tablero PID — Control de ventiladores de rack")
        self.resize(1500, 900)

        self.sim = Simulador(T_CPU_INICIAL)
        self.rng = np.random.default_rng()
        self.t = 0.0
        self.corriendo = True

        # rampa de falla de HVAC
        self.hvac_activa = False
        self.hvac_t0 = 0.0
        self.hvac_val0 = T_AMB_INICIAL

        # buffers de historial (una muestra por segundo simulado, ventana deslizante)
        self.buf = {k: deque(maxlen=VENTANA_S) for k in
                    ("t", "tref", "tcpu", "tmed", "error", "pwm", "rpm_cmd",
                     "rpm_med", "cpu", "tamb", "p", "i", "d")}

        self._build_ui()

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(FRAME_MS)

    # ---------------------------------------------------------- UI
    def _build_ui(self):
        raiz = QtWidgets.QHBoxLayout(self)
        raiz.setContentsMargins(12, 12, 12, 12)
        raiz.setSpacing(10)

        raiz.addWidget(self._panel_graficos(), stretch=3)
        raiz.addWidget(self._panel_control(), stretch=0)

    def _nuevo_plot(self, titulo, ylabel):
        p = pg.PlotWidget()
        p.setTitle(titulo, size="9pt")
        p.setLabel("left", ylabel, **{"font-size": "8pt"})
        p.showGrid(x=True, y=True, alpha=0.15)
        p.setMouseEnabled(x=False, y=False)
        p.setClipToView(True)
        p.setDownsampling(mode="peak", auto=True)
        tick_font = QtGui.QFont()
        tick_font.setPointSize(8)
        for eje in ("left", "bottom", "right"):
            p.getAxis(eje).setStyle(tickFont=tick_font)
        return p

    def _panel_graficos(self):
        cont = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(cont)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(4)

        # --- Temperatura ---
        titulo_temp = ("Temperatura&nbsp;&nbsp;&nbsp;&nbsp;"
                       + _ref(COL_TCPU, "Tcpu (variable controlada / salida)") + "&nbsp;&nbsp;&nbsp;"
                       + _ref(COL_TMED, "LM35 (realimentación)") + "&nbsp;&nbsp;&nbsp;"
                       + _ref(COL_TREF, "Tref (referencia)"))
        self.p_temp = self._nuevo_plot(titulo_temp, "Temp. [°C]")
        self.p_temp.setYRange(15, 90)
        self.p_temp.addLine(y=T_ALERTA, pen=pg.mkPen("#e6a23c", style=Qt.DotLine))
        self.p_temp.addLine(y=T_THROTTLE, pen=pg.mkPen("#c0392b", style=Qt.DotLine))
        self.c_tref = self.p_temp.plot(pen=pg.mkPen(COL_TREF, width=1.5, style=Qt.DashLine))
        self.c_tcpu = self.p_temp.plot(pen=pg.mkPen(COL_TCPU, width=2))
        self.c_tmed = self.p_temp.plot(pen=pg.mkPen(COL_TMED, width=1, style=Qt.DotLine))

        # --- Error ---
        self.p_err = self._nuevo_plot("Señal de error  e(t) = Tref − Tmedido", "Error [°C]")
        self.p_err.addLine(y=0, pen=pg.mkPen("#555", width=1))
        self.c_err = self.p_err.plot(pen=pg.mkPen(COL_ERR, width=2))

        # --- PWM ---
        self.p_pwm = self._nuevo_plot("Señal de control · u(t) (variable manipulada, ciclo de trabajo)", "PWM [%]")
        self.p_pwm.setYRange(-5, 105)
        self.c_pwm = self.p_pwm.plot(pen=pg.mkPen(COL_PWM, width=2),
                                     fillLevel=0, brush=pg.mkBrush(90, 169, 230, 40))

        # --- RPM ---
        titulo_rpm = ("Velocidad del ventilador&nbsp;&nbsp;&nbsp;&nbsp;"
                      + _ref(COL_RPMC, "comandada") + "&nbsp;&nbsp;&nbsp;"
                      + _ref(COL_RPMM, "medida (Hall)"))
        self.p_rpm = self._nuevo_plot(titulo_rpm, "RPM")
        self.c_rpmc = self.p_rpm.plot(pen=pg.mkPen(COL_RPMC, width=2))
        self.c_rpmm = self.p_rpm.plot(pen=pg.mkPen(COL_RPMM, width=1, style=Qt.DotLine))

        # --- Perturbaciones (doble eje) ---
        titulo_pert = ("Perturbaciones aplicadas&nbsp;&nbsp;&nbsp;&nbsp;"
                       + _ref(COL_CPU, "Carga CPU") + "&nbsp;&nbsp;&nbsp;"
                       + _ref(COL_TAMB, "Tamb"))
        self.p_pert = self._nuevo_plot(titulo_pert, "Carga CPU [%]")
        self.c_cpu = self.p_pert.plot(pen=pg.mkPen(COL_CPU, width=2))
        self.vb_tamb = pg.ViewBox()
        self.p_pert.showAxis("right")
        self.p_pert.scene().addItem(self.vb_tamb)
        self.p_pert.getAxis("right").linkToView(self.vb_tamb)
        self.p_pert.getAxis("right").setLabel("Tamb [°C]", color=COL_TAMB)
        self.vb_tamb.setXLink(self.p_pert)
        self.c_tamb = pg.PlotCurveItem(pen=pg.mkPen(COL_TAMB, width=1.5))
        self.vb_tamb.addItem(self.c_tamb)
        self.p_pert.getViewBox().sigResized.connect(self._sync_tamb_view)

        for p in (self.p_temp, self.p_err, self.p_pwm, self.p_rpm, self.p_pert):
            lay.addWidget(p)
        return cont

    def _sync_tamb_view(self):
        self.vb_tamb.setGeometry(self.p_pert.getViewBox().sceneBoundingRect())
        self.vb_tamb.linkedViewChanged(self.p_pert.getViewBox(), self.vb_tamb.XAxis)

    def _panel_control(self):
        panel = QtWidgets.QWidget()
        panel.setFixedWidth(320)
        lay = QtWidgets.QVBoxLayout(panel)
        lay.setContentsMargins(0, 0, 0, 0)
        # spacing 0 + un stretch igual entre cada container: el primero queda
        # pegado arriba, el ultimo abajo, y los huecos quedan repartidos parejo.
        lay.setSpacing(0)

        # lectura en vivo
        self.readout = QtWidgets.QLabel()
        self.readout.setObjectName("readout")
        self.readout.setTextFormat(Qt.RichText)
        box_read = QtWidgets.QGroupBox("Estado en vivo")
        l0 = QtWidgets.QVBoxLayout(box_read)
        l0.addWidget(self.readout)
        lay.addWidget(box_read)
        lay.addStretch(1)

        # PID
        box_pid = QtWidgets.QGroupBox("Parámetros del PID")
        lp = QtWidgets.QVBoxLayout(box_pid)
        self.s_kp = ParamSlider("Kp", 0.0, 20.0, 0.1, KP)
        self.s_ki = ParamSlider("Ki", 0.0, 2.0, 0.01, KI)
        self.s_kd = ParamSlider("Kd", 0.0, 10.0, 0.1, KD)
        for s in (self.s_kp, self.s_ki, self.s_kd):
            lp.addWidget(s)
        lay.addWidget(box_pid)
        lay.addStretch(1)

        # referencia y perturbaciones manuales
        box_ref = QtWidgets.QGroupBox("Referencia y perturbaciones")
        lr = QtWidgets.QVBoxLayout(box_ref)
        self.s_tref = ParamSlider("Setpoint", 40.0, 80.0, 1.0, T_SETPOINT_DEFAULT, "{:.0f}", " °C")
        self.s_cpu = ParamSlider("Carga CPU", 0.0, 100.0, 1.0, 30.0, "{:.0f}", " %")
        self.s_tamb = ParamSlider("Tamb", 15.0, 35.0, 0.5, T_AMB_INICIAL, "{:.1f}", " °C")
        self.s_vel = ParamSlider("Vel. sim", 1.0, 20.0, 1.0, 5.0, "{:.0f}", " s/frame")
        self.s_tamb.changed.connect(lambda _: setattr(self, "hvac_activa", False))
        for s in (self.s_tref, self.s_cpu, self.s_tamb, self.s_vel):
            lr.addWidget(s)
        self.chk_ruido = QtWidgets.QCheckBox("Ruido de sensor (LM35 ±0,5 °C)")
        self.chk_ruido.setChecked(True)
        lr.addWidget(self.chk_ruido)
        lay.addWidget(box_ref)
        lay.addStretch(1)

        # perturbaciones puntuales
        box_bt = QtWidgets.QGroupBox("Perturbaciones puntuales")
        gb = QtWidgets.QGridLayout(box_bt)
        gb.setSpacing(6)
        b_pico = QtWidgets.QPushButton("Pico 90%")
        b_dos = QtWidgets.QPushButton("DoS 100%")
        b_normal = QtWidgets.QPushButton("Carga 30%")
        b_hvac = QtWidgets.QPushButton("Falla HVAC")
        b_pico.clicked.connect(lambda: self.s_cpu.set_value(90))
        b_dos.clicked.connect(lambda: self.s_cpu.set_value(100))
        b_normal.clicked.connect(lambda: self.s_cpu.set_value(30))
        b_hvac.clicked.connect(self._disparar_hvac)
        gb.addWidget(b_pico, 0, 0); gb.addWidget(b_dos, 0, 1)
        gb.addWidget(b_normal, 1, 0); gb.addWidget(b_hvac, 1, 1)
        lay.addWidget(box_bt)
        lay.addStretch(1)

        # transporte
        fila = QtWidgets.QHBoxLayout()
        fila.setSpacing(8)
        self.b_pausa = QtWidgets.QPushButton("Pausar")
        self.b_pausa.setObjectName("accent")
        self.b_pausa.clicked.connect(self._toggle_pausa)
        b_reset = QtWidgets.QPushButton("Reset")
        b_reset.setObjectName("danger")
        b_reset.clicked.connect(self._reset)
        fila.addWidget(self.b_pausa)
        fila.addWidget(b_reset)
        lay.addLayout(fila)
        return panel

    # ---------------------------------------------------------- acciones
    def _disparar_hvac(self):
        self.hvac_activa = True
        self.hvac_t0 = self.t
        self.hvac_val0 = self.s_tamb.value()

    def _toggle_pausa(self):
        self.corriendo = not self.corriendo
        self.b_pausa.setText("Reanudar" if not self.corriendo else "Pausar")

    def _reset(self):
        self.sim.reset(T_CPU_INICIAL)
        self.t = 0.0
        self.hvac_activa = False
        for k in self.buf:
            self.buf[k].clear()
        self.s_kp.set_value(KP); self.s_ki.set_value(KI); self.s_kd.set_value(KD)
        self.s_tref.set_value(T_SETPOINT_DEFAULT); self.s_cpu.set_value(30)
        self.s_tamb.set_value(T_AMB_INICIAL); self.s_vel.set_value(5)

    def _tamb_actual(self):
        if self.hvac_activa:
            transcurrido = self.t - self.hvac_t0
            delta = min(HVAC_RAMPA_DELTA, transcurrido * (HVAC_RAMPA_DELTA / HVAC_RAMPA_DURACION))
            if transcurrido >= HVAC_RAMPA_DURACION:
                self.hvac_activa = False
                self.s_tamb.set_value(self.hvac_val0 + HVAC_RAMPA_DELTA)
            return self.hvac_val0 + delta
        return self.s_tamb.value()

    # ---------------------------------------------------------- loop
    def _tick(self):
        if self.corriendo:
            ruido = self.chk_ruido.isChecked()
            kp, ki, kd = self.s_kp.value(), self.s_ki.value(), self.s_kd.value()
            tref, cpu = self.s_tref.value(), self.s_cpu.value()
            for _ in range(int(self.s_vel.value())):
                tamb = self._tamb_actual()
                r = self.sim.paso(tref, cpu, tamb, kp, ki, kd,
                                  dt=DT, ruido=ruido, rng=self.rng)
                self.t += DT
                self._push(tref, cpu, tamb, r)

        self._redibujar()

    def _push(self, tref, cpu, tamb, r):
        b = self.buf
        b["t"].append(self.t)
        b["tref"].append(tref)
        b["tcpu"].append(r["T_cpu"])
        b["tmed"].append(r["T_medido"])
        b["error"].append(r["error"])
        b["pwm"].append(r["u_sat"])
        b["rpm_cmd"].append(r["rpm_cmd"])
        b["rpm_med"].append(r["rpm_medida"])
        b["cpu"].append(cpu)
        b["tamb"].append(tamb)
        b["p"].append(r["p"]); b["i"].append(r["i"]); b["d"].append(r["d"])
        # el recorte de la ventana lo hace deque(maxlen=VENTANA_S) automaticamente

    def _redibujar(self):
        b = self.buf
        if not b["t"]:
            return
        a = {k: np.fromiter(v, float) for k, v in b.items()}
        t = a["t"]
        self.c_tref.setData(t, a["tref"])
        self.c_tcpu.setData(t, a["tcpu"])
        self.c_tmed.setData(t, a["tmed"])
        self.c_err.setData(t, a["error"])
        self.c_pwm.setData(t, a["pwm"])
        self.c_rpmc.setData(t, a["rpm_cmd"])
        self.c_rpmm.setData(t, a["rpm_med"])
        self.c_cpu.setData(t, a["cpu"])
        self.c_tamb.setData(t, a["tamb"])

        x0, x1 = t[0], max(t[-1], t[0] + 1)
        for p in (self.p_temp, self.p_err, self.p_pwm, self.p_rpm, self.p_pert):
            p.setXRange(x0, x1, padding=0)

        tcpu = b["tcpu"][-1]
        estado = ""
        if tcpu >= T_THROTTLE:
            estado = "<br><b style='color:#ff5d5d'>*** THROTTLING ***</b>"
        elif tcpu >= T_ALERTA:
            estado = "<br><b style='color:#e6a23c'>*** ALERTA ***</b>"
        self.readout.setText(
            f"<span style='color:#8ea0c0'>t</span>&nbsp;&nbsp;&nbsp;&nbsp;= {self.t:7.1f} s<br>"
            f"Tref&nbsp;= {b['tref'][-1]:7.2f} °C<br>"
            f"<span style='color:{COL_TCPU}'>Tcpu</span>&nbsp;= {tcpu:7.2f} °C<br>"
            f"<span style='color:{COL_TMED}'>Tmed</span>&nbsp;= {b['tmed'][-1]:7.2f} °C<br>"
            f"error = {b['error'][-1]:7.2f} °C<br>"
            f"<span style='color:{COL_PWM}'>PWM</span>&nbsp;&nbsp;= {b['pwm'][-1]:7.1f} %<br>"
            f"<span style='color:{COL_RPMC}'>RPM</span>&nbsp;&nbsp;= {b['rpm_cmd'][-1]:7.0f}<br>"
            f"<br>"
            f"<span style='color:#666'>── aporte PID (pre-saturación) ──</span><br>"
            f"<br>"
            f"P = {b['p'][-1]:8.2f}<br>"
            f"I = {b['i'][-1]:8.2f}<br>"
            f"D = {b['d'][-1]:8.2f}"
            f"{estado}"
        )


def main():
    app = QtWidgets.QApplication(sys.argv)
    app.setStyleSheet(QSS)
    w = Tablero()
    # Ajustar al area disponible y centrar, dejando aire arriba y abajo
    # (usa ~80-90% de la pantalla para no pegarse a los bordes).
    disp = app.primaryScreen().availableGeometry()
    ancho = min(1480, int(disp.width() * 0.9))
    alto = min(800, int(disp.height() * 0.80))
    w.resize(ancho, alto)
    w.move(disp.left() + (disp.width() - ancho) // 2,
           disp.top() + (disp.height() - alto) // 2)
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
