"""
Sonar Kontrol Paneli — ana pencere.

Düzen:
  - Üst araç çubuğu : bağlantı, LED, sıcaklık, ses hızı, frekans
  - Merkez sol      : kontrol paneli (tarama aralığı, engel tespiti)
  - Merkez orta     : sekmeli harita — "2D Harita" (polar az+el) / "3D Harita"
  - Merkez sağ      : cluster listesi
  - Alt dock        : kontrol sekmeleri (Steering / Frekans / Kazanç / Eylem)
"""

import sys
import os
import time
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from PyQt6.QtCore import (
    QTimer,
    Qt,
    pyqtSignal,
)
from PyQt6.QtGui import QColor, QPalette
from PyQt6.QtWidgets import (
    QCheckBox,
    QDial,
    QDockWidget,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSlider,
    QSpinBox,
    QStatusBar,
    QTabWidget,
    QToolBar,
    QVBoxLayout,
    QWidget,
    QFrame,
)

from socket_client import SonarClient, SonarConnectionError, SonarTimeoutError
from ui.sonar_map import SonarMap
from ui.map_3d import Map3D
from ui.toggle_switch import ToggleSwitch
from processing.obstacle_detector import ObstacleDetector


# ─── Global QSS ──────────────────────────────────────────────────────────────

_QSS = """
QMainWindow, QWidget {
    background-color: #0a1128;
    color: #b0cce0;
    font-family: "Segoe UI", "DejaVu Sans", Arial, sans-serif;
    font-size: 11px;
}
QGroupBox {
    background: rgba(10, 24, 54, 0.80);
    border: 1px solid #1e3d6e;
    border-radius: 8px;
    margin-top: 8px;
    padding: 10px 8px 8px 8px;
    font-weight: bold;
    font-size: 10px;
    color: #5eaeff;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 12px;
    padding: 0 4px;
}
QPushButton {
    background-color: #0b1e3d;
    border: 1px solid #1e4a7a;
    border-radius: 6px;
    color: #7ac8ff;
    padding: 5px 12px;
}
QPushButton:hover {
    background-color: #102545;
    border-color: #00e5ff;
    color: #00e5ff;
}
QPushButton:pressed {
    background-color: #050f22;
    border-color: #00b8d9;
    color: #00b8d9;
}
QPushButton:disabled {
    color: #3a5570;
    border-color: #162233;
}
QSlider::groove:horizontal {
    height: 5px;
    background: #152236;
    border-radius: 2px;
    margin: 2px 0;
}
QSlider::sub-page:horizontal {
    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
        stop:0 #0066aa, stop:1 #00ccff);
    border-radius: 2px;
}
QSlider::add-page:horizontal {
    background: #152236;
    border-radius: 2px;
}
QSlider::handle:horizontal {
    width: 14px;
    height: 14px;
    margin: -5px 0;
    background: #00ccff;
    border-radius: 7px;
    border: 2px solid #050f22;
}
QSlider::handle:horizontal:hover { background: #00ffff; }
QDial {
    background: #0a1128;
    color: #5eaeff;
}
QSpinBox, QDoubleSpinBox {
    background-color: #0b1a30;
    border: 1px solid #1e3d6e;
    border-radius: 4px;
    color: #7affcc;
    padding: 2px 6px;
    selection-background-color: #0066aa;
}
QSpinBox::up-button, QDoubleSpinBox::up-button,
QSpinBox::down-button, QDoubleSpinBox::down-button {
    background: #0b1e3d;
    border: none;
    width: 14px;
}
QLabel {
    color: #8cb8d0;
    background: transparent;
}
QLabel#telemetry {
    color: #7affcc;
    font-family: "Consolas", "Courier New", "DejaVu Sans Mono", monospace;
    font-size: 11px;
    font-weight: bold;
}
QLabel#scan_status {
    color: #ffdd88;
    font-family: "Consolas", "Courier New", "DejaVu Sans Mono", monospace;
    font-size: 11px;
}
QToolBar {
    background: #060e20;
    border-bottom: 1px solid #152236;
    padding: 4px;
    spacing: 8px;
}
QStatusBar {
    background: #040c1c;
    color: #5eaeff;
    font-family: "Consolas", "Courier New", monospace;
    font-size: 10px;
    border-top: 1px solid #152236;
}
QTabWidget::pane {
    border: 1px solid #1e3d6e;
    border-radius: 0 4px 4px 4px;
    background: #0a1128;
}
QTabBar::tab {
    background: #060e20;
    border: 1px solid #1a2d48;
    border-bottom: none;
    padding: 4px 14px;
    color: #4a7090;
    border-top-left-radius: 4px;
    border-top-right-radius: 4px;
    font-size: 10px;
}
QTabBar::tab:selected {
    background: #0e1e3a;
    color: #00e5ff;
    border-color: #00e5ff;
    border-bottom-color: #0e1e3a;
}
QTabBar::tab:hover:!selected { color: #7ac8ff; background: #0a1628; }
QDockWidget {
    color: #5eaeff;
    font-size: 11px;
    font-weight: bold;
}
QDockWidget::title {
    background: #060e20;
    padding: 4px 8px;
    border-bottom: 1px solid #152236;
    text-align: left;
}
QListWidget {
    background: #050d1e;
    border: 1px solid #1e3d6e;
    border-radius: 4px;
    color: #7affcc;
    font-family: "Consolas", "Courier New", "DejaVu Sans Mono", monospace;
    font-size: 10px;
}
QListWidget::item { padding: 2px 4px; }
QListWidget::item:hover { background: #0e1e38; }
QListWidget::item:selected { background: #143055; color: #00e5ff; }
QScrollBar:vertical {
    background: #050d1e;
    width: 8px;
    border-radius: 4px;
}
QScrollBar::handle:vertical {
    background: #1e3d6e;
    border-radius: 4px;
    min-height: 20px;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QSplitter::handle { background: #1e3d6e; }
QCheckBox { color: #8cb8d0; spacing: 6px; }
QCheckBox::indicator {
    width: 14px; height: 14px;
    border-radius: 3px;
    border: 1px solid #1e4a7a;
    background: #0b1a30;
}
QCheckBox::indicator:checked { background: #0066aa; border-color: #00ccff; }
"""


# ─── LED Widget ──────────────────────────────────────────────────────────────

class LedIndicator(QFrame):
    """Yuvarlak LED göstergesi — yeşil (bağlı) / kırmızı (bağlı değil)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(16, 16)
        self._set_color("#cc0000")

    def set_connected(self, connected: bool):
        self._set_color("#00cc44" if connected else "#cc0000")

    def _set_color(self, hex_color: str):
        self.setStyleSheet(
            f"background-color: {hex_color};"
            "border-radius: 8px;"
            "border: 1px solid rgba(0,0,0,0.4);"
        )


# ─── Ana Pencere ──────────────────────────────────────────────────────────────

class MainWindow(QMainWindow):
    """Sonar kontrol paneli ana penceresi."""

    doppler_received        = pyqtSignal(dict)
    scan_progress_received  = pyqtSignal(dict)
    temperature_received    = pyqtSignal(dict)
    error_received          = pyqtSignal(dict)
    scan_range_set_received = pyqtSignal(dict)
    sweep_complete_received = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Sonar Kontrol Paneli — Hexagonal Phased Array")
        self.resize(1100, 560)
        self.setMinimumSize(900, 480)

        self._burst_active  = False
        self._scan_active   = False
        self._last_scan_step: int = -1

        self.client = SonarClient(auto_reconnect=False)

        self._detector = ObstacleDetector()
        self._detector.clusters_updated.connect(self._on_clusters_updated)

        self._pending_echoes: deque = deque(maxlen=20000)
        self._last_status_ts: float = 0.0
        self._status_throttle_s     = 0.2

        self._steer_timer = QTimer(self)
        self._steer_timer.setSingleShot(True)
        self._steer_timer.timeout.connect(self._send_steer)

        self.setStyleSheet(_QSS)

        self._build_ui()
        self._connect_signals()
        self._start_timers()
        self._try_connect()

    # ─── UI inşası ───────────────────────────────────────────────────────────

    def _build_ui(self):
        self._build_toolbar()
        self._build_center()
        self._build_control_dock()
        self.setStatusBar(QStatusBar(self))

    def _build_toolbar(self):
        tb = QToolBar("Bağlantı", self)
        tb.setMovable(False)
        self.addToolBar(tb)

        self._btn_connect = QPushButton("Baglan")
        self._btn_connect.setFixedWidth(95)
        self._btn_connect.clicked.connect(self._toggle_connection)
        tb.addWidget(self._btn_connect)

        self._led = LedIndicator()
        tb.addWidget(self._led)
        tb.addSeparator()

        tb.addWidget(QLabel(" T:"))
        self._lbl_temp = QLabel("—")
        self._lbl_temp.setObjectName("telemetry")
        self._lbl_temp.setMinimumWidth(50)
        tb.addWidget(self._lbl_temp)

        tb.addWidget(QLabel(" c:"))
        self._lbl_sound = QLabel("—")
        self._lbl_sound.setObjectName("telemetry")
        self._lbl_sound.setMinimumWidth(60)
        tb.addWidget(self._lbl_sound)

        tb.addWidget(QLabel(" f:"))
        self._lbl_freq_display = QLabel("—")
        self._lbl_freq_display.setObjectName("telemetry")
        self._lbl_freq_display.setMinimumWidth(70)
        tb.addWidget(self._lbl_freq_display)

    def _build_center(self):
        central  = QWidget()
        h_layout = QHBoxLayout(central)
        h_layout.setContentsMargins(0, 0, 0, 0)
        h_layout.setSpacing(0)

        # ── Sol kontrol paneli ───────────────────────────────────────────────
        left = QWidget()
        left.setFixedWidth(260)
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(6, 6, 6, 6)
        left_layout.setSpacing(8)
        left_layout.addWidget(self._build_scan_range_group())
        left_layout.addWidget(self._build_obstacle_group())
        left_layout.addWidget(self._build_clear_group())
        left_layout.addStretch(1)

        # ── İki polar map yan yana ───────────────────────────────────────────
        self._sonar_map_az = SonarMap(plane='azimuth')
        self._sonar_map_el = SonarMap(plane='elevation')

        polar_widget  = QWidget()
        polar_h       = QHBoxLayout(polar_widget)
        polar_h.setContentsMargins(0, 0, 0, 0)
        polar_h.setSpacing(2)
        polar_h.addWidget(self._sonar_map_az)
        polar_h.addWidget(self._sonar_map_el)

        # ── 3D harita ────────────────────────────────────────────────────────
        self._map_3d = Map3D()

        # ── Sekmeli harita: 2D polar ve 3D ayrı sekmelerde ───────────────────
        map_tabs = QTabWidget()
        map_tabs.addTab(polar_widget,  "2D Harita")
        map_tabs.addTab(self._map_3d,  "3D Harita")

        # ── En sağ: cluster listesi paneli ──────────────────────────────────
        cluster_panel = self._build_cluster_panel()

        h_layout.addWidget(left)
        h_layout.addWidget(map_tabs, stretch=1)
        h_layout.addWidget(cluster_panel)

        self.setCentralWidget(central)

    def _build_cluster_panel(self) -> QWidget:
        panel        = QWidget()
        panel.setMinimumWidth(340)
        panel.setMaximumWidth(420)
        p_layout     = QVBoxLayout(panel)
        p_layout.setContentsMargins(4, 4, 4, 4)
        p_layout.setSpacing(4)

        group   = QGroupBox("Tespit Edilen Engeller")
        g_layout = QVBoxLayout(group)
        g_layout.setContentsMargins(4, 4, 4, 4)
        g_layout.setSpacing(4)

        self._cluster_list = QListWidget()
        self._cluster_list.setMaximumHeight(280)
        self._cluster_list.setStyleSheet(
            "font-family: monospace; font-size: 10px;"
        )

        self._cluster_summary = QLabel("0 engel tespit edildi")
        self._cluster_summary.setStyleSheet("color: #aaffaa; font-size: 11px;")
        self._cluster_summary.setAlignment(Qt.AlignmentFlag.AlignCenter)

        g_layout.addWidget(self._cluster_list)
        g_layout.addWidget(self._cluster_summary)

        p_layout.addWidget(group)
        p_layout.addStretch(1)
        return panel

    def _build_scan_range_group(self) -> QGroupBox:
        group = QGroupBox("Tarama Aralığı")
        grid  = QGridLayout(group)
        grid.setSpacing(6)

        grid.addWidget(QLabel("Azimuth ±°"), 0, 0)
        self._az_spin = QSpinBox()
        self._az_spin.setRange(5, 90)
        self._az_spin.setSingleStep(5)
        self._az_spin.setValue(30)
        grid.addWidget(self._az_spin, 0, 1)

        grid.addWidget(QLabel("Elevation ±°"), 1, 0)
        self._el_spin = QSpinBox()
        self._el_spin.setRange(5, 90)
        self._el_spin.setSingleStep(5)
        self._el_spin.setValue(30)
        grid.addWidget(self._el_spin, 1, 1)

        grid.addWidget(QLabel("Step °"), 2, 0)
        self._step_spin = QSpinBox()
        self._step_spin.setRange(1, 30)
        self._step_spin.setSingleStep(1)
        self._step_spin.setValue(5)
        grid.addWidget(self._step_spin, 2, 1)

        btn = QPushButton("Uygula")
        btn.clicked.connect(self._on_apply_scan_range)
        grid.addWidget(btn, 3, 0, 1, 2)

        self._scan_range_status = QLabel("")
        self._scan_range_status.setWordWrap(True)
        self._scan_range_status.setStyleSheet("color: #aaffaa; font-size: 11px;")
        grid.addWidget(self._scan_range_status, 4, 0, 1, 2)

        return group

    def _build_obstacle_group(self) -> QGroupBox:
        group = QGroupBox("Engel Tespiti")
        grid  = QGridLayout(group)
        grid.setSpacing(6)

        self._chk_obstacle = ToggleSwitch("Aktif")
        self._chk_obstacle.setChecked(True)
        self._chk_obstacle.toggled.connect(self._on_obstacle_toggle)
        grid.addWidget(self._chk_obstacle, 0, 0, 1, 2)

        grid.addWidget(QLabel("Eps (m):"), 1, 0)
        self._eps_spin = QDoubleSpinBox()
        self._eps_spin.setRange(0.05, 5.0)
        self._eps_spin.setSingleStep(0.05)
        self._eps_spin.setDecimals(2)
        self._eps_spin.setValue(0.30)
        self._eps_spin.valueChanged.connect(self._on_obstacle_params_changed)
        grid.addWidget(self._eps_spin, 1, 1)

        grid.addWidget(QLabel("Min Nokta:"), 2, 0)
        self._min_pts_spin = QSpinBox()
        self._min_pts_spin.setRange(2, 20)
        self._min_pts_spin.setValue(3)
        self._min_pts_spin.valueChanged.connect(self._on_obstacle_params_changed)
        grid.addWidget(self._min_pts_spin, 2, 1)

        return group

    def _build_clear_group(self) -> QGroupBox:
        group   = QGroupBox("Harita")
        layout  = QVBoxLayout(group)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(6)

        btn = QPushButton("Haritalari Temizle")
        btn.setStyleSheet(
            "background-color: #1a0a2e;"
            "border: 1px solid #6633aa;"
            "color: #cc99ff;"
            "border-radius: 6px;"
            "padding: 5px 10px;"
        )
        btn.clicked.connect(self._on_clear_maps)
        layout.addWidget(btn)
        return group

    def _build_control_dock(self):
        dock = QDockWidget("Kontrol", self)
        dock.setAllowedAreas(Qt.DockWidgetArea.BottomDockWidgetArea)
        dock.setFeatures(QDockWidget.DockWidgetFeature.NoDockWidgetFeatures)
        dock.setMaximumHeight(140)

        self._ctrl_tabs = QTabWidget()
        self._ctrl_tabs.addTab(self._build_steering_widget(), "Steering")
        self._ctrl_tabs.addTab(self._build_freq_widget(),     "Frekans")
        self._ctrl_tabs.addTab(self._build_gain_widget(),     "Kazanc")
        self._ctrl_tabs.addTab(self._build_action_widget(),   "Eylem")

        dock.setWidget(self._ctrl_tabs)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, dock)

    def _build_steering_widget(self) -> QWidget:
        w = QWidget()
        layout = QHBoxLayout(w)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(20)

        az_box = QVBoxLayout()
        az_box.setSpacing(4)
        az_box.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        self._lbl_az = QLabel("Azimuth: 0°")
        self._lbl_az.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._dial_az = QDial()
        self._dial_az.setRange(-30, 30)
        self._dial_az.setValue(0)
        self._dial_az.setNotchesVisible(True)
        self._dial_az.setNotchTarget(10.0)
        self._dial_az.setWrapping(False)
        self._dial_az.setFixedSize(80, 80)
        self._dial_az.valueChanged.connect(self._on_az_changed)
        az_box.addWidget(self._lbl_az)
        az_box.addWidget(self._dial_az, alignment=Qt.AlignmentFlag.AlignHCenter)

        el_box = QVBoxLayout()
        el_box.setSpacing(4)
        el_box.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        self._lbl_el = QLabel("Elevasyon: 0°")
        self._lbl_el.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self._dial_el = QDial()
        self._dial_el.setRange(-30, 30)
        self._dial_el.setValue(0)
        self._dial_el.setNotchesVisible(True)
        self._dial_el.setNotchTarget(10.0)
        self._dial_el.setWrapping(False)
        self._dial_el.setFixedSize(80, 80)
        self._dial_el.valueChanged.connect(self._on_el_changed)
        el_box.addWidget(self._lbl_el)
        el_box.addWidget(self._dial_el, alignment=Qt.AlignmentFlag.AlignHCenter)

        layout.addLayout(az_box)
        layout.addLayout(el_box)
        layout.addStretch(1)
        return w

    def _build_freq_widget(self) -> QWidget:
        w = QWidget()
        layout = QHBoxLayout(w)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(12)

        # ── Manuel frekans ───────────────────────────────────────────────────
        layout.addWidget(QLabel("Manuel:"))
        self._spin_freq = QSpinBox()
        self._spin_freq.setRange(38000, 42000)
        self._spin_freq.setSingleStep(100)
        self._spin_freq.setValue(40000)
        self._spin_freq.setSuffix(" Hz")
        self._spin_freq.valueChanged.connect(self._on_freq_changed)
        layout.addWidget(self._spin_freq)

        # ── Dikey ayırıcı ────────────────────────────────────────────────────
        sep = QFrame()
        sep.setFrameShape(QFrame.Shape.VLine)
        sep.setStyleSheet("color: #1e3d6e;")
        layout.addWidget(sep)

        # ── Frekans sweep ────────────────────────────────────────────────────
        layout.addWidget(QLabel("Sweep:"))

        self._spin_sweep_min = QSpinBox()
        self._spin_sweep_min.setRange(38000, 42000)
        self._spin_sweep_min.setSingleStep(100)
        self._spin_sweep_min.setValue(39000)
        self._spin_sweep_min.setSuffix(" Hz")
        self._spin_sweep_min.setToolTip("Min Hz")
        layout.addWidget(QLabel("Min"))
        layout.addWidget(self._spin_sweep_min)

        self._spin_sweep_max = QSpinBox()
        self._spin_sweep_max.setRange(38000, 42000)
        self._spin_sweep_max.setSingleStep(100)
        self._spin_sweep_max.setValue(41000)
        self._spin_sweep_max.setSuffix(" Hz")
        self._spin_sweep_max.setToolTip("Max Hz")
        layout.addWidget(QLabel("Max"))
        layout.addWidget(self._spin_sweep_max)

        self._spin_sweep_step = QSpinBox()
        self._spin_sweep_step.setRange(10, 1000)
        self._spin_sweep_step.setSingleStep(10)
        self._spin_sweep_step.setValue(100)
        self._spin_sweep_step.setSuffix(" Hz")
        self._spin_sweep_step.setToolTip("Adım Hz")
        layout.addWidget(QLabel("Adım"))
        layout.addWidget(self._spin_sweep_step)

        self._spin_sweep_dwell = QSpinBox()
        self._spin_sweep_dwell.setRange(1, 500)
        self._spin_sweep_dwell.setSingleStep(5)
        self._spin_sweep_dwell.setValue(31)
        self._spin_sweep_dwell.setSuffix(" ms")
        self._spin_sweep_dwell.setToolTip("Her frekansta bekleme süresi")
        layout.addWidget(QLabel("Bekleme"))
        layout.addWidget(self._spin_sweep_dwell)

        self._btn_sweep_start = QPushButton("Sweep Başlat")
        self._btn_sweep_start.clicked.connect(self._on_sweep_start)
        layout.addWidget(self._btn_sweep_start)

        self._btn_sweep_stop = QPushButton("Sweep Durdur")
        self._btn_sweep_stop.clicked.connect(self._on_sweep_stop)
        layout.addWidget(self._btn_sweep_stop)

        layout.addStretch()
        return w

    def _build_gain_widget(self) -> QWidget:
        w = QWidget()
        main_layout = QVBoxLayout(w)
        main_layout.setContentsMargins(8, 4, 8, 4)
        main_layout.setSpacing(6)

        all_layout = QHBoxLayout()
        self._lbl_gain_all      = QLabel("Tum: 0 dB")
        self._lbl_gain_all.setMinimumWidth(90)
        self._slider_gain_all   = QSlider(Qt.Orientation.Horizontal)
        self._slider_gain_all.setRange(0, 15)
        self._slider_gain_all.setValue(0)
        self._slider_gain_all.valueChanged.connect(self._on_gain_all_changed)
        all_layout.addWidget(self._lbl_gain_all)
        all_layout.addWidget(self._slider_gain_all, 1)
        main_layout.addLayout(all_layout)

        grid = QGridLayout()
        grid.setSpacing(4)
        self._gain_sliders: list[QSlider] = []

        for kanal in range(7):
            row        = kanal // 2
            col_offset = (kanal % 2) * 2
            lbl = QLabel(f"K{kanal}: 0dB")
            lbl.setMinimumWidth(60)
            sl = QSlider(Qt.Orientation.Horizontal)
            sl.setRange(0, 15)
            sl.setValue(0)
            sl.setFixedHeight(16)
            sl.valueChanged.connect(
                lambda db, k=kanal, l=lbl: self._on_gain_channel_changed(k, db, l)
            )
            self._gain_sliders.append(sl)
            grid.addWidget(lbl, row, col_offset)
            grid.addWidget(sl,  row, col_offset + 1)

        main_layout.addLayout(grid)
        return w

    def _build_action_widget(self) -> QWidget:
        w = QWidget()
        layout = QVBoxLayout(w)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(6)

        burst_row = QHBoxLayout()
        self._btn_burst_start = QPushButton("Burst Baslat")
        self._btn_burst_stop  = QPushButton("Burst Durdur")
        self._btn_burst_start.clicked.connect(self._on_burst_start)
        self._btn_burst_stop.clicked.connect(self._on_burst_stop)
        burst_row.addWidget(self._btn_burst_start)
        burst_row.addWidget(self._btn_burst_stop)

        scan_row = QHBoxLayout()
        self._btn_scan_start = QPushButton("Tarama Baslat")
        self._btn_scan_stop  = QPushButton("Tarama Durdur")
        self._btn_scan_start.clicked.connect(self._on_scan_start)
        self._btn_scan_stop.clicked.connect(self._on_scan_stop)
        scan_row.addWidget(self._btn_scan_start)
        scan_row.addWidget(self._btn_scan_stop)

        dwell_row = QHBoxLayout()
        dwell_row.addWidget(QLabel("Bekleme:"))
        self._spin_dwell = QSpinBox()
        self._spin_dwell.setRange(32, 2000)
        self._spin_dwell.setSingleStep(10)
        self._spin_dwell.setValue(50)
        self._spin_dwell.setSuffix(" ms")
        self._spin_dwell.setToolTip(
            "Her pozisyonda bekleme süresi.\n"
            "Min 32 ms (1 tam burst+echo penceresi = 31.5 ms).\n"
            "±30°/5° adım: ~9.6 sn/sweep  |  ±30°/1°: ~3.5 dk/sweep"
        )
        dwell_row.addWidget(self._spin_dwell)
        dwell_row.addStretch()

        self._lbl_scan_status = QLabel("Tarama: —")
        self._lbl_scan_status.setObjectName("scan_status")
        self._lbl_scan_status.setAlignment(Qt.AlignmentFlag.AlignCenter)

        layout.addLayout(burst_row)
        layout.addLayout(scan_row)
        layout.addLayout(dwell_row)
        layout.addWidget(self._lbl_scan_status)
        return w

    # ─── Sinyal / Slot bağlantıları ──────────────────────────────────────────

    def _connect_signals(self):
        self.doppler_received.connect(self._handle_doppler)
        self.scan_progress_received.connect(self._handle_scan_progress)
        self.temperature_received.connect(self._handle_temperature)
        self.error_received.connect(self._handle_error)
        self.scan_range_set_received.connect(self._handle_scan_range_set)
        self.sweep_complete_received.connect(self._handle_sweep_complete)

    # ─── Timer'lar ───────────────────────────────────────────────────────────

    def _start_timers(self):
        self._conn_timer = QTimer(self)
        self._conn_timer.timeout.connect(self._update_connection_ui)
        self._conn_timer.start(1000)

        self._temp_timer = QTimer(self)
        self._temp_timer.timeout.connect(self._poll_temperature)
        self._temp_timer.start(5000)

        self._render_timer = QTimer(self)
        self._render_timer.timeout.connect(self._drain_pending_echoes)
        self._render_timer.start(33)

    # ─── Bağlantı yönetimi ───────────────────────────────────────────────────

    def _try_connect(self):
        ok = self.client.connect()
        if ok:
            self._register_event_listeners()
            self.statusBar().showMessage("Baglandi.")
        else:
            self.statusBar().showMessage("Baglanti kurulamadi. C tarafi calisiyor mu?")
            QMessageBox.warning(
                self,
                "Baglanti Hatasi",
                "C sonar sunucusuna baglanamadi.\n"
                "Lutfen sunucunun calistigini dogrulayin ve tekrar deneyin.",
            )
        self._update_connection_ui()

    def _toggle_connection(self):
        if self.client.is_connected():
            self.client.disconnect()
            self.statusBar().showMessage("Baglanti kesildi.")
        else:
            self._try_connect()
        self._update_connection_ui()

    def _update_connection_ui(self):
        connected = self.client.is_connected()
        self._led.set_connected(connected)
        self._btn_connect.setText("Baglantiyi Kes" if connected else "Baglan")

    def _register_event_listeners(self):
        self.client.on_event("echo",            self.on_echo_event)
        self.client.on_event("echo_batch",      self.on_echo_batch_event)
        self.client.on_event("doppler",          self.on_doppler_event)
        self.client.on_event("scan_progress",    self.on_scan_progress_event)
        self.client.on_event("temperature",      self.on_temperature_event)
        self.client.on_event("error",            self.on_error_event)
        self.client.on_event("scan_range_set",   self.on_scan_range_set_event)
        self.client.on_event("sweep_complete",   self.on_sweep_complete_event)

    # ─── Arka plan thread → sinyal/queue köprüleri ───────────────────────────

    def on_echo_event(self, data: dict):
        az   = float(data.get("az",   0.0))
        el   = float(data.get("el",   0.0))
        dist = float(data.get("dist", 0.0))
        self._pending_echoes.append((az, el, dist))

    def on_echo_batch_event(self, data: dict):
        az = float(data.get("az", 0.0))
        el = float(data.get("el", 0.0))
        for item in data.get("echoes", ()):
            try:
                dist = float(item.get("dist", 0.0))
            except (TypeError, AttributeError):
                continue
            self._pending_echoes.append((az, el, dist))

    def on_doppler_event(self, data: dict):
        self.doppler_received.emit(data)

    def on_scan_progress_event(self, data: dict):
        self.scan_progress_received.emit(data)

    def on_temperature_event(self, data: dict):
        self.temperature_received.emit(data)

    def on_error_event(self, data: dict):
        self.error_received.emit(data)

    def on_scan_range_set_event(self, data: dict):
        self.scan_range_set_received.emit(data)

    def on_sweep_complete_event(self, data: dict):
        self.sweep_complete_received.emit(data)

    # ─── UI thread sinyal işleyicileri ───────────────────────────────────────

    def _drain_pending_echoes(self):
        """Main-thread timer (30 FPS): deque'yi boşalt, widget'lara ilet."""
        if not self._pending_echoes:
            return

        items = []
        try:
            while True:
                items.append(self._pending_echoes.popleft())
        except IndexError:
            pass

        if not items:
            return

        # items: (az, el, dist) — add_echoes_batch 3-tuple imzasını destekliyor
        self._sonar_map_az.add_echoes_batch(items)
        self._sonar_map_el.add_echoes_batch(items)
        self._map_3d.add_echoes_batch(items)
        self._detector.add_echoes_batch(items)

        now = time.time()
        if now - self._last_status_ts >= self._status_throttle_s:
            az, el, dist = items[-1]
            self.statusBar().showMessage(
                f"Echo: {dist:.2f} m  |  az={az:.1f} el={el:.1f}  ({len(items)} pkt)"
            )
            self._last_status_ts = now

    def _handle_doppler(self, data: dict):
        freq = data.get("freq", 0.0)
        vel  = data.get("velocity", 0.0)
        self.statusBar().showMessage(
            f"Doppler: {freq:.1f} Hz  |  hiz={vel:.2f} m/s"
        )

    def _handle_scan_progress(self, data: dict):
        step  = data.get("step", "?")
        total = data.get("total", "?")
        az    = data.get("az", 0.0)
        el    = data.get("el", 0.0)

        if isinstance(step, int):
            prev        = self._last_scan_step
            # Yeni tarama döngüsü: step 1'e sıfırlandığında (ilk başlatma hariç).
            # Çift yönlü taramada her iki yön de 1..N adımlarını paylaşır;
            # step < prev tek tek kontrol EDİLMEZ — her adımda temizleme yapar.
            new_sweep = (step == 1 and prev > 1)
            if new_sweep:
                self._sonar_map_az.clear_echoes()
                self._sonar_map_el.clear_echoes()
                self._map_3d.clear_echoes()
                self._detector.reset_buffer()
                self._cluster_list.clear()
                self._cluster_summary.setText("0 engel tespit edildi")
            self._last_scan_step = step

        self._sonar_map_az.set_beam_direction(az, el)
        self._sonar_map_el.set_beam_direction(az, el)
        self._map_3d.set_beam_direction(az, el)

        text = f"Tarama: {step}/{total}  az={az:.1f} el={el:.1f}"
        self._lbl_scan_status.setText(text)
        self.statusBar().showMessage(text)

    def _handle_temperature(self, data: dict):
        celsius     = data.get("celsius", 0.0)
        sound_speed = data.get("sound_speed", 0.0)
        self._lbl_temp.setText(f"{celsius:.1f} °C")
        self._lbl_sound.setText(f"{sound_speed:.1f} m/s")

    def _handle_error(self, data: dict):
        module  = data.get("module", "?")
        message = data.get("message", "")
        self.statusBar().showMessage(f"[HATA] {module}: {message}")

    def _handle_sweep_complete(self, data: dict):
        """Yeni tarama pass'i başladığında tüm haritaları sıfırla."""
        direction = data.get("direction", "?")
        count     = data.get("count", 0)
        self._lbl_scan_status.setText(f"Sweep #{count} ({direction}) başladı")

        # Haritalar ve cluster state'ini sıfırla
        self._sonar_map_az.clear_echoes()
        self._sonar_map_el.clear_echoes()
        self._map_3d.clear_echoes()
        self._detector.reset_buffer()
        self._cluster_list.clear()
        self._cluster_summary.setText("0 engel tespit edildi")

    def _on_clusters_updated(self, clusters: list):
        self._sonar_map_az.set_clusters(clusters)
        self._sonar_map_el.set_clusters(clusters)
        self._map_3d.set_clusters(clusters)

        self._cluster_list.clear()
        for c in clusters:
            cx, cy, cz = c.centroid_xyz
            az, el, r  = c.centroid_polar
            text = (
                f"C{c.cluster_id:2d}: "
                f"x={cx:+.2f}m y={cy:+.2f}m z={cz:+.2f}m  "
                f"az={az:+5.1f}° el={el:+5.1f}° r={r:.2f}m "
                f"(n={c.n_points})"
            )
            self._cluster_list.addItem(text)

        self._cluster_summary.setText(f"{len(clusters)} engel tespit edildi")

    def _on_obstacle_toggle(self, enabled: bool):
        self._detector.set_enabled(enabled)

    def _on_obstacle_params_changed(self):
        self._detector.set_params(
            eps=self._eps_spin.value(),
            min_samples=self._min_pts_spin.value(),
        )

    def _handle_scan_range_set(self, data: dict):
        az   = data.get("az",   "?")
        el   = data.get("el",   "?")
        step = data.get("step", "?")
        self._scan_range_status.setText(f"OK: ±{az}°/±{el}° step={step}°")
        self._scan_range_status.setStyleSheet("color: #aaffaa; font-size: 11px;")

    # ─── Tarama Aralığı ──────────────────────────────────────────────────────

    def _on_apply_scan_range(self):
        az   = self._az_spin.value()
        el   = self._el_spin.value()
        step = self._step_spin.value()

        self._sonar_map_az.set_scan_range(az, el, float(step))
        self._sonar_map_el.set_scan_range(az, el, float(step))
        self._map_3d.set_scan_range(az, el, float(step))

        self._dial_az.setRange(-az, az)
        self._dial_el.setRange(-el, el)

        self._scan_range_status.setText("…uygulanıyor")
        self._scan_range_status.setStyleSheet("color: #ffdd88; font-size: 11px;")

        if self.client.is_connected():
            try:
                resp = self.client.scan_range(float(az), float(el), float(step))
                if resp.get("status") == "ok":
                    self._scan_range_status.setText(f"OK: ±{az}°/±{el}° step={step}°")
                    self._scan_range_status.setStyleSheet(
                        "color: #aaffaa; font-size: 11px;")
                else:
                    msg = resp.get("message", "bilinmeyen hata")
                    self._scan_range_status.setText(f"HATA: {msg}")
                    self._scan_range_status.setStyleSheet(
                        "color: #ff6666; font-size: 11px;")
            except (SonarConnectionError, SonarTimeoutError) as exc:
                self._scan_range_status.setText(f"Hata: {exc}")
                self._scan_range_status.setStyleSheet("color: #ff6666; font-size: 11px;")
        else:
            self._scan_range_status.setText("C tarafı bağlı değil (UI güncellendi)")
            self._scan_range_status.setStyleSheet("color: #ffdd88; font-size: 11px;")

    # ─── Steering ────────────────────────────────────────────────────────────

    def _on_az_changed(self, value: int):
        self._lbl_az.setText(f"Azimuth: {value}°")
        self._steer_timer.start(200)

    def _on_el_changed(self, value: int):
        self._lbl_el.setText(f"Elevasyon: {value}°")
        self._steer_timer.start(200)

    def _send_steer(self):
        if not self.client.is_connected():
            return
        az = float(self._dial_az.value())
        el = float(self._dial_el.value())

        self._sonar_map_az.set_beam_direction(az, el)
        self._sonar_map_el.set_beam_direction(az, el)
        self._map_3d.set_beam_direction(az, el)

        try:
            self.client.steer(az, el)
        except (SonarConnectionError, SonarTimeoutError) as exc:
            self.statusBar().showMessage(f"Steering hatasi: {exc}")

    # ─── Frekans ─────────────────────────────────────────────────────────────

    def _on_freq_changed(self, hz: int):
        self._lbl_freq_display.setText(f"{hz} Hz")
        if not self.client.is_connected():
            return
        try:
            self.client.set_freq(hz)
        except (SonarConnectionError, SonarTimeoutError) as exc:
            self.statusBar().showMessage(f"Frekans hatasi: {exc}")

    def _on_sweep_start(self):
        if not self.client.is_connected():
            self.statusBar().showMessage("Bagli degil.")
            return
        if self._scan_active:
            self.statusBar().showMessage(
                "Uyari: Tarama aktifken freq sweep çalıştırma — beamforming bozulur.")
            return
        min_hz   = self._spin_sweep_min.value()
        max_hz   = self._spin_sweep_max.value()
        step_hz  = self._spin_sweep_step.value()
        dwell_ms = self._spin_sweep_dwell.value()
        if min_hz >= max_hz:
            self.statusBar().showMessage("Sweep: min_hz < max_hz olmali.")
            return
        try:
            self.client.freq_sweep_start(min_hz, max_hz, step_hz, dwell_ms)
            self._btn_sweep_start.setStyleSheet(
                "background-color: #cc2222; color: white;")
            self.statusBar().showMessage(
                f"Freq sweep: {min_hz}–{max_hz} Hz, adim={step_hz} Hz, bekleme={dwell_ms} ms")
        except (SonarConnectionError, SonarTimeoutError) as exc:
            self.statusBar().showMessage(f"Sweep baslatilamadi: {exc}")

    def _on_sweep_stop(self):
        if not self.client.is_connected():
            return
        try:
            self.client.freq_sweep_stop()
            self._btn_sweep_start.setStyleSheet("")
            self.statusBar().showMessage("Freq sweep durduruldu.")
        except (SonarConnectionError, SonarTimeoutError) as exc:
            self.statusBar().showMessage(f"Sweep durdurulamadi: {exc}")

    # ─── Kazanç ──────────────────────────────────────────────────────────────

    def _on_gain_all_changed(self, db: int):
        self._lbl_gain_all.setText(f"Tum: {db} dB")
        if not self.client.is_connected():
            return
        try:
            self.client.set_gain_all(float(db))
        except (SonarConnectionError, SonarTimeoutError) as exc:
            self.statusBar().showMessage(f"Kazanc hatasi: {exc}")

    def _on_gain_channel_changed(self, kanal: int, db: int, label: QLabel):
        label.setText(f"K{kanal}: {db}dB")
        if not self.client.is_connected():
            return
        try:
            self.client.set_gain(kanal, float(db))
        except (SonarConnectionError, SonarTimeoutError) as exc:
            self.statusBar().showMessage(f"Kanal {kanal} kazanc hatasi: {exc}")

    # ─── Burst ───────────────────────────────────────────────────────────────

    def _on_burst_start(self):
        if not self.client.is_connected():
            self.statusBar().showMessage("Bagli degil.")
            return
        try:
            self.client.burst_start()
            self._burst_active = True
            self._btn_burst_start.setStyleSheet("background-color: #cc2222; color: white;")
            self.statusBar().showMessage("Burst baslatildi.")
        except (SonarConnectionError, SonarTimeoutError) as exc:
            self.statusBar().showMessage(f"Burst baslatilamadi: {exc}")

    def _on_burst_stop(self):
        if not self.client.is_connected():
            return
        try:
            self.client.burst_stop()
            self._burst_active = False
            self._btn_burst_start.setStyleSheet("")
            self.statusBar().showMessage("Burst durduruldu.")
        except (SonarConnectionError, SonarTimeoutError) as exc:
            self.statusBar().showMessage(f"Burst durdurulamadi: {exc}")

    # ─── Tarama ──────────────────────────────────────────────────────────────

    def _on_scan_start(self):
        if not self.client.is_connected():
            self.statusBar().showMessage("Bagli degil.")
            return
        try:
            self.client.scan_start(self._spin_dwell.value())
            self._scan_active = True
            self._btn_scan_start.setStyleSheet("background-color: #cc2222; color: white;")
            self._lbl_scan_status.setText("Tarama baslatildi...")
            self.statusBar().showMessage("Tarama baslatildi.")
        except (SonarConnectionError, SonarTimeoutError) as exc:
            self.statusBar().showMessage(f"Tarama baslatilamadi: {exc}")

    def _on_scan_stop(self):
        if not self.client.is_connected():
            return
        try:
            self.client.scan_stop()
            self._scan_active = False
            self._btn_scan_start.setStyleSheet("")
            self._lbl_scan_status.setText("Tarama durduruldu.")
            self.statusBar().showMessage("Tarama durduruldu.")
        except (SonarConnectionError, SonarTimeoutError) as exc:
            self.statusBar().showMessage(f"Tarama durdurulamadi: {exc}")

    def _on_clear_maps(self):
        self._sonar_map_az.clear_echoes()
        self._sonar_map_el.clear_echoes()
        self._map_3d.clear_echoes()
        self._detector.reset_buffer()
        self._pending_echoes.clear()
        self._cluster_list.clear()
        self._cluster_summary.setText("0 engel tespit edildi")
        self._last_scan_step = -1
        self.statusBar().showMessage("Haritalar temizlendi.")

    # ─── Sıcaklık polling ────────────────────────────────────────────────────

    def _poll_temperature(self):
        if not self.client.is_connected():
            return
        try:
            resp = self.client.read_temp()
            if resp.get("status") == "ok":
                celsius     = resp.get("celsius", 0.0)
                sound_speed = resp.get("sound_speed", 0.0)
                self._lbl_temp.setText(f"{celsius:.1f} °C")
                self._lbl_sound.setText(f"{sound_speed:.1f} m/s")
        except (SonarConnectionError, SonarTimeoutError):
            pass

    # ─── Kapanış ─────────────────────────────────────────────────────────────

    def closeEvent(self, event):
        if self._scan_active:
            try:
                self.client.scan_stop()
            except Exception:
                pass

        if self._burst_active:
            try:
                self.client.burst_stop()
            except Exception:
                pass

        self._detector.shutdown()
        self.client.disconnect()
        event.accept()
