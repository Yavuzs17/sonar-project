"""
3D sonar haritası widget'ı — PyQtGraph OpenGL (GLViewWidget) tabanlı.

matplotlib kaldırıldı; render yükü GPU'ya taşındı.

Koordinat sistemi:
    x = r * sin(az) * cos(el)   (sağa pozitif)
    y = r * sin(el)              (yukarı pozitif)
    z = r * cos(az) * cos(el)   (ileri pozitif)
"""

import math
import time
from dataclasses import dataclass

import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QVBoxLayout, QWidget


# ─── Sabitler ────────────────────────────────────────────────────────────────

DEFAULT_MAX_RANGE_M     = 5.0
DEFAULT_AZIMUTH_RANGE   = 30.0
DEFAULT_ELEVATION_RANGE = 30.0

MAX_RANGE_M     = DEFAULT_MAX_RANGE_M
AZIMUTH_RANGE   = DEFAULT_AZIMUTH_RANGE
ELEVATION_RANGE = DEFAULT_ELEVATION_RANGE

MAX_ECHOES = 50000
REFRESH_MS = 50

BG_COLOR     = "#0d1b2a"
GRID_COLOR   = "#2a3a4a"
BEAM_COLOR   = "#3399ff"
SENSOR_COLOR = "#ff3366"
TEXT_COLOR   = "#ccffcc"

_ECHO_RGB = (0, 210, 100)


# ─── Yardımcılar ─────────────────────────────────────────────────────────────

def _hex_rgba(hex_color: str, alpha: float = 1.0) -> tuple:
    h = hex_color.lstrip("#")
    r, g, b = (int(h[i:i + 2], 16) / 255.0 for i in (0, 2, 4))
    return (r, g, b, alpha)


# ─── Veri yapısı ─────────────────────────────────────────────────────────────

@dataclass
class EchoPoint3D:
    azimuth_deg:   float
    elevation_deg: float
    range_m:       float
    timestamp:     float
    strength:      float = 1.0

    def to_cartesian(self) -> tuple:
        az = math.radians(self.azimuth_deg)
        el = math.radians(self.elevation_deg)
        r  = self.range_m
        x  = r * math.sin(az) * math.cos(el)
        y  = r * math.sin(el)
        z  = r * math.cos(az) * math.cos(el)
        return x, y, z


# ─── Widget ───────────────────────────────────────────────────────────────────

class Map3D(QWidget):
    """
    3D sonar haritası — PyQtGraph OpenGL tabanlı PyQt6 widget.

    Echo'lar sweep boyunca birikir; clear_echoes() ile silinir.
    Public API, matplotlib sürümüyle aynıdır.
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        self._echoes:   list[EchoPoint3D] = []
        self._clusters: list              = []
        self._beam_az  = 0.0
        self._beam_el  = 0.0
        self._dirty    = True

        self._az_range  = DEFAULT_AZIMUTH_RANGE
        self._el_range  = DEFAULT_ELEVATION_RANGE
        self._max_range = DEFAULT_MAX_RANGE_M
        self._step_deg  = 5.0

        # ── GLViewWidget ─────────────────────────────────────────────────────
        self._glw = gl.GLViewWidget()
        self._glw.setBackgroundColor(BG_COLOR)
        self._glw.opts['elevation'] = 22
        self._glw.opts['azimuth']   = -60
        self._glw.opts['distance']  = 12
        self._glw.opts['center']    = pg.Vector(0, 0, self._max_range * 0.4)

        self._build_grid()
        self._build_artists()

        # ── Layout ───────────────────────────────────────────────────────────
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._glw)

        # ── Refresh timer ────────────────────────────────────────────────────
        self._refresh_timer = QTimer(self)
        self._refresh_timer.timeout.connect(self._redraw)
        self._refresh_timer.start(REFRESH_MS)

    # ─── Grid ────────────────────────────────────────────────────────────────

    def _make_grid_pos(self) -> np.ndarray:
        """Tüm yay ve radyal çizgi segmentlerini GL_LINES için (N,3) float32 üret."""
        az  = self._az_range
        el  = self._el_range
        r   = self._max_range
        stp = self._step_deg
        pairs: list = []

        for radius in np.arange(stp, r + 1e-9, stp):
            # Azimuth yayı (el=0 düzlemi)
            az_angles = np.deg2rad(np.linspace(-az, az, 61))
            ax  = radius * np.sin(az_angles)
            ay  = np.zeros(61)
            az_ = radius * np.cos(az_angles)
            for i in range(60):
                pairs.append([ax[i], ay[i], az_[i]])
                pairs.append([ax[i + 1], ay[i + 1], az_[i + 1]])

            # Elevation yayı (az=0 düzlemi)
            el_angles = np.deg2rad(np.linspace(-el, el, 61))
            ex  = np.zeros(61)
            ey  = radius * np.sin(el_angles)
            ez  = radius * np.cos(el_angles)
            for i in range(60):
                pairs.append([ex[i], ey[i], ez[i]])
                pairs.append([ex[i + 1], ey[i + 1], ez[i + 1]])

        # Azimuth radyal çizgiler
        for az_deg in np.arange(-az, az + 1e-9, stp):
            x, y, z = EchoPoint3D(float(az_deg), 0.0, r, 0).to_cartesian()
            pairs.append([0.0, 0.0, 0.0])
            pairs.append([x, y, z])

        # Elevation radyal çizgiler
        for el_deg in np.arange(-el, el + 1e-9, stp):
            x, y, z = EchoPoint3D(0.0, float(el_deg), r, 0).to_cartesian()
            pairs.append([0.0, 0.0, 0.0])
            pairs.append([x, y, z])

        return np.array(pairs, dtype=np.float32)

    def _build_grid(self):
        pos = self._make_grid_pos()
        self._grid_item = gl.GLLinePlotItem(
            pos=pos,
            color=_hex_rgba(GRID_COLOR, 0.5),
            width=1.0,
            mode='lines',
            antialias=False,
        )
        self._glw.addItem(self._grid_item)

    # ─── Dinamik artist'ler ───────────────────────────────────────────────────

    def _build_artists(self):
        # Sensör noktası (orijin, statik)
        self._sensor_item = gl.GLScatterPlotItem(
            pos=np.array([[0.0, 0.0, 0.0]], dtype=np.float32),
            color=_hex_rgba(SENSOR_COLOR),
            size=10,
            pxMode=True,
        )
        self._glw.addItem(self._sensor_item)

        # Beam çizgisi (orijinden uca)
        x, y, z = EchoPoint3D(0.0, 0.0, self._max_range, 0).to_cartesian()
        self._beam_item = gl.GLLinePlotItem(
            pos=np.array([[0.0, 0.0, 0.0], [x, y, z]], dtype=np.float32),
            color=_hex_rgba(BEAM_COLOR),
            width=2.0,
            mode='line_strip',
            antialias=True,
        )
        self._glw.addItem(self._beam_item)

        # Echo scatter — başlangıçta gizli (pos=None → paint() erken döner)
        self._echo_item = gl.GLScatterPlotItem(
            color=(_ECHO_RGB[0] / 255.0, _ECHO_RGB[1] / 255.0, _ECHO_RGB[2] / 255.0, 0.85),
            size=6,
            pxMode=True,
        )
        self._echo_item.setVisible(False)
        self._glw.addItem(self._echo_item)

        # Cluster çapraz işaretleri (turuncu) — başlangıçta gizli
        self._cluster_item = gl.GLLinePlotItem(
            color=(1.0, 0.6, 0.0, 0.85),
            width=2.0,
            mode='lines',
            antialias=False,
        )
        self._cluster_item.setVisible(False)
        self._glw.addItem(self._cluster_item)

    # ─── Render ──────────────────────────────────────────────────────────────

    def _redraw(self):
        if not self.isVisible():
            self._dirty = True  # görünür olduğunda çizilmeli
            return
        if not self._dirty:
            return
        try:
            self._update_beam()
            self._update_echoes()
            self._update_clusters()
            self._dirty = False
        except Exception:
            pass

    def _beam_xyz(self) -> tuple:
        az = math.radians(self._beam_az)
        el = math.radians(self._beam_el)
        r  = self._max_range
        return (
            r * math.sin(az) * math.cos(el),
            r * math.sin(el),
            r * math.cos(az) * math.cos(el),
        )

    def _update_beam(self):
        x, y, z = self._beam_xyz()
        self._beam_item.setData(
            pos=np.array([[0.0, 0.0, 0.0], [x, y, z]], dtype=np.float32)
        )

    def _update_echoes(self):
        if not self._echoes:
            self._echo_item.setVisible(False)
            return
        az_rad = np.deg2rad([e.azimuth_deg   for e in self._echoes])
        el_rad = np.deg2rad([e.elevation_deg for e in self._echoes])
        rs     = np.array  ([e.range_m       for e in self._echoes])
        cos_el = np.cos(el_rad)
        xs = rs * np.sin(az_rad) * cos_el
        ys = rs * np.sin(el_rad)
        zs = rs * np.cos(az_rad) * cos_el
        pos = np.column_stack([xs, ys, zs]).astype(np.float32)
        self._echo_item.setData(pos=pos)
        self._echo_item.setVisible(True)

    def _update_clusters(self):
        if not self._clusters:
            self._cluster_item.setVisible(False)
            return
        arm   = self._max_range * 0.05
        pairs = []
        for c in self._clusters:
            cx, cy, cz = c.centroid_xyz
            pairs += [
                [cx - arm, cy, cz], [cx + arm, cy, cz],
                [cx, cy - arm, cz], [cx, cy + arm, cz],
                [cx, cy, cz - arm], [cx, cy, cz + arm],
            ]
        self._cluster_item.setData(pos=np.array(pairs, dtype=np.float32))
        self._cluster_item.setVisible(True)

    # ─── Izgara / view yeniden oluşturma ─────────────────────────────────────

    def _rebuild(self):
        self._grid_item.setData(pos=self._make_grid_pos())
        x, y, z = self._beam_xyz()
        self._beam_item.setData(
            pos=np.array([[0.0, 0.0, 0.0], [x, y, z]], dtype=np.float32)
        )
        self._glw.opts['center'] = pg.Vector(0, 0, self._max_range * 0.4)
        self._dirty = True

    # ─── Public API ──────────────────────────────────────────────────────────

    def add_echo(
        self,
        azimuth_deg:   float,
        elevation_deg: float,
        range_m:       float,
        strength:      float = 1.0,
    ):
        if abs(azimuth_deg)   > self._az_range:  return
        if abs(elevation_deg) > self._el_range:  return
        if range_m < 0 or range_m > self._max_range: return
        self._echoes.append(EchoPoint3D(
            azimuth_deg=azimuth_deg, elevation_deg=elevation_deg,
            range_m=range_m, timestamp=time.time(), strength=strength,
        ))
        if len(self._echoes) > MAX_ECHOES:
            self._echoes = self._echoes[-MAX_ECHOES:]
        self._dirty = True

    def add_echoes_batch(self, items: list):
        """items: (azimuth_deg, elevation_deg, range_m) tuple listesi."""
        if not items:
            return
        now      = time.time()
        appended = 0
        for az, el, dist in items:
            if abs(az) > self._az_range or abs(el) > self._el_range:
                continue
            if dist < 0 or dist > self._max_range:
                continue
            self._echoes.append(EchoPoint3D(
                azimuth_deg=az, elevation_deg=el, range_m=dist, timestamp=now,
            ))
            appended += 1
        if appended:
            if len(self._echoes) > MAX_ECHOES:
                self._echoes = self._echoes[-MAX_ECHOES:]
            self._dirty = True

    def set_beam_direction(self, azimuth_deg: float, elevation_deg: float = 0.0):
        new_az = float(azimuth_deg)
        new_el = float(elevation_deg)
        if new_az != self._beam_az or new_el != self._beam_el:
            self._beam_az = new_az
            self._beam_el = new_el
            self._dirty   = True

    def set_scan_range(
        self,
        azimuth_range_deg:   float,
        elevation_range_deg: float,
        step_deg:            float = 5.0,
    ):
        self._az_range  = float(azimuth_range_deg)
        self._el_range  = float(elevation_range_deg)
        self._step_deg  = float(step_deg)
        self._rebuild()

    def set_max_range(self, max_range_m: float):
        self._max_range = float(max_range_m)
        self._rebuild()

    def set_clusters(self, clusters: list):
        self._clusters = list(clusters)
        self._dirty    = True

    def clear_echoes(self):
        self._echoes   = []
        self._clusters = []
        self._dirty    = True

    def showEvent(self, event):
        super().showEvent(event)
        self._dirty = True
        self._glw.update()
