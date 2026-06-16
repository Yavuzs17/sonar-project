"""
Polar harita test scripti.

Sahte echo verileri üretir, polar haritanın doğru çizip çizmediğini test eder.
C tarafına ihtiyaç YOKTUR — bağımsız çalışır.

Kullanım:
    cd ~/sonar-project/python
    python3 test_polar_echo.py
    python3 test_polar_echo.py --pattern scan      # tarama deseni
    python3 test_polar_echo.py --pattern wall      # düz duvar
    python3 test_polar_echo.py --pattern targets   # belirli hedefler
    python3 test_polar_echo.py --pattern random    # rastgele
"""

import sys
import os
import argparse
import math
import random
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QComboBox,
    QSlider,
)

# Path ayarı — ui modülünü import edebilmek için
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ui.sonar_map import PolarSonarMap


class EchoSimulator:
    """Sahte echo üretici. Farklı desenler destekler."""

    def __init__(self, polar_map: PolarSonarMap):
        self.polar_map = polar_map
        self.pattern = "scan"
        self.tick = 0

        # Tarama deseni için pozisyon
        self._scan_az = -30
        self._scan_el = 0  # şimdilik sadece yatay

    def step(self):
        """Bir tick ileri al, echo üret."""
        self.tick += 1

        if self.pattern == "scan":
            self._pattern_scan()
        elif self.pattern == "wall":
            self._pattern_wall()
        elif self.pattern == "targets":
            self._pattern_targets()
        elif self.pattern == "random":
            self._pattern_random()
        elif self.pattern == "moving":
            self._pattern_moving()

    # ─── Tarama deseni ────────────────────────────────────────────
    def _pattern_scan(self):
        """13×1 yatay tarama, her yönde sahte echo (bir nesne 2.5m'de)."""
        # Beam yönünü güncelle
        self.polar_map.set_beam_direction(self._scan_az, 0)

        # %50 ihtimalle echo at (her yönden değil)
        if random.random() < 0.5:
            # Sahte hedef: az değerine bağlı mesafe (sallanan duvar)
            mesafe = 2.5 + 0.5 * math.sin(math.radians(self._scan_az * 3))
            self.polar_map.add_echo(self._scan_az, mesafe)

        # Sonraki yöne geç
        self._scan_az += 5
        if self._scan_az > 30:
            self._scan_az = -30

    # ─── Düz duvar ───────────────────────────────────────────────
    def _pattern_wall(self):
        """Yaklaşık 3 metrede dümdüz duvar — her yöne echo at."""
        for az in range(-30, 31, 5):
            mesafe = 3.0 + random.uniform(-0.1, 0.1)  # gürültü
            self.polar_map.add_echo(az, mesafe)

    # ─── Belirli hedefler ────────────────────────────────────────
    def _pattern_targets(self):
        """3 sabit hedef — farklı yönlerde, farklı mesafelerde."""
        targets = [
            (-20, 1.5),  # sol yakın
            (5,   3.0),  # önde uzak
            (15,  2.0),  # sağ orta
        ]
        for az, mesafe in targets:
            # Pozisyona az gürültü ekle
            self.polar_map.add_echo(
                az + random.uniform(-1, 1),
                mesafe + random.uniform(-0.05, 0.05)
            )

    # ─── Rastgele ────────────────────────────────────────────────
    def _pattern_random(self):
        """Tamamen rastgele noktalar."""
        for _ in range(3):
            az = random.uniform(-30, 30)
            mesafe = random.uniform(0.5, 4.5)
            self.polar_map.add_echo(az, mesafe)

    # ─── Hareket eden hedef ──────────────────────────────────────
    def _pattern_moving(self):
        """Tek hedef sağa-sola gidip geliyor, mesafe yavaşça yaklaşıyor."""
        t = self.tick * 0.1
        az = 25 * math.sin(t)
        mesafe = 3.5 - 0.05 * (self.tick % 30)
        if mesafe < 0.5:
            self.tick = 0
            mesafe = 3.5

        self.polar_map.set_beam_direction(az, 0)
        self.polar_map.add_echo(az, mesafe)


class TestWindow(QMainWindow):
    """Test penceresi — kontroller + polar harita."""

    def __init__(self, initial_pattern="scan"):
        super().__init__()
        self.setWindowTitle("Polar Harita Test — Sahte Echo Üretici")
        self.resize(800, 600)

        # Ana widget
        central = QWidget()
        layout = QVBoxLayout(central)

        # Üst kontrol şeridi
        ctrl = QHBoxLayout()

        ctrl.addWidget(QLabel("Desen:"))
        self.combo = QComboBox()
        self.combo.addItems(["scan", "wall", "targets", "random", "moving"])
        self.combo.setCurrentText(initial_pattern)
        self.combo.currentTextChanged.connect(self._on_pattern_change)
        ctrl.addWidget(self.combo)

        ctrl.addWidget(QLabel("Hız:"))
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(50, 1000)  # ms
        self.speed_slider.setValue(200)
        self.speed_slider.valueChanged.connect(self._on_speed_change)
        self.speed_label = QLabel("200 ms")
        ctrl.addWidget(self.speed_slider)
        ctrl.addWidget(self.speed_label)

        self.btn_clear = QPushButton("Temizle")
        self.btn_clear.clicked.connect(self._on_clear)
        ctrl.addWidget(self.btn_clear)

        self.btn_pause = QPushButton("Duraklat")
        self.btn_pause.setCheckable(True)
        self.btn_pause.toggled.connect(self._on_pause)
        ctrl.addWidget(self.btn_pause)

        layout.addLayout(ctrl)

        # Polar harita
        self.polar_map = PolarSonarMap()
        layout.addWidget(self.polar_map, 1)

        self.setCentralWidget(central)

        # Echo simulator
        self.simulator = EchoSimulator(self.polar_map)
        self.simulator.pattern = initial_pattern

        # Tick timer
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.simulator.step)
        self.timer.start(200)

    def _on_pattern_change(self, text):
        self.simulator.pattern = text
        self.polar_map.clear_echoes()

    def _on_speed_change(self, value):
        self.speed_label.setText(f"{value} ms")
        self.timer.setInterval(value)

    def _on_clear(self):
        self.polar_map.clear_echoes()

    def _on_pause(self, checked):
        if checked:
            self.timer.stop()
            self.btn_pause.setText("Devam")
        else:
            self.timer.start()
            self.btn_pause.setText("Duraklat")


def main():
    parser = argparse.ArgumentParser(description="Polar harita test scripti")
    parser.add_argument(
        "--pattern",
        default="scan",
        choices=["scan", "wall", "targets", "random", "moving"],
        help="Başlangıç deseni"
    )
    args = parser.parse_args()

    app = QApplication(sys.argv)
    win = TestWindow(initial_pattern=args.pattern)
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()