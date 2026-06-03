"""
Polar sonar haritası widget'ı — PyQtGraph tabanlı PyQt6 bileşeni.

Matplotlib yerine PyQtGraph kullanılır; render yükü GPU'ya taşınır.
PyQtGraph native polar desteği olmadığından koordinatlar manuel
Kartezyen'e dönüştürülür:
    x = r * sin(az_rad)
    y = r * cos(az_rad)   (y = ileri / yukarı)

plane='azimuth'   → θ ekseni azimuth açısı
plane='elevation' → θ ekseni elevation açısı
"""

import math
import time
from dataclasses import dataclass

import numpy as np
import pyqtgraph as pg
from PyQt6.QtCore import Qt, QPointF, QRectF, QTimer
from PyQt6.QtGui  import QBrush, QColor, QPainter, QPen, QPolygonF
from PyQt6.QtWidgets import QWidget, QVBoxLayout

pg.setConfigOption('antialias', True)
pg.setConfigOption('background', '#0d1b2a')
pg.setConfigOption('foreground', '#ccffcc')


# ─── Sabitler ────────────────────────────────────────────────────────────────

MAX_RANGE_M     = 5.0
AZIMUTH_RANGE   = 30.0
ELEVATION_RANGE = 30.0
MAX_ECHOES      = 50000
BEAM_HALF_DEG   = 5.0
REFRESH_MS      = 50     # 20 FPS

BG_COLOR     = "#0d1b2a"
GRID_COLOR   = "#2a3a4a"
BEAM_COLOR   = "#3399ff"
SENSOR_COLOR = "#ff3366"
TEXT_COLOR   = "#ccffcc"

_ECHO_RGB   = (0, 210, 100)
_ECHO_BRUSH = pg.mkBrush(_ECHO_RGB[0], _ECHO_RGB[1], _ECHO_RGB[2], 220)


# ─── Veri yapısı ─────────────────────────────────────────────────────────────

@dataclass
class EchoPoint:
    azimuth_deg: float
    range_m:     float
    timestamp:   float
    strength:    float = 1.0


# ─── Dolu fan çokgeni (beam) — pg.GraphicsObject alt sınıfı ─────────────────

class _FilledPolygon(pg.GraphicsObject):
    """Veri koordinatlarında çizilen dolu çokgen."""

    def __init__(self, pen: QPen, brush: QBrush):
        super().__init__()
        self._pen   = pen
        self._brush = brush
        self._poly  = QPolygonF()

    def setPolygon(self, pts: list):
        self._poly = QPolygonF([QPointF(float(x), float(y)) for x, y in pts])
        self.prepareGeometryChange()
        self.update()

    def boundingRect(self) -> QRectF:
        if self._poly.isEmpty():
            return QRectF()
        r = self._poly.boundingRect()
        # küçük kenar boşluğu — Qt kırpma sorununu önler
        return r.adjusted(-0.1, -0.1, 0.1, 0.1)

    def paint(self, p: QPainter, *_):
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setPen(self._pen)
        p.setBrush(self._brush)
        p.drawPolygon(self._poly)


# ─── Widget ───────────────────────────────────────────────────────────────────

class SonarMap(QWidget):
    """
    Polar sonar haritası — PyQtGraph tabanlı.

    Echo'lar sweep boyunca birikir; clear_echoes() ile silinir.
    plane: 'azimuth' | 'elevation'
    """

    def __init__(self, parent=None, plane: str = 'azimuth'):
        assert plane in ('azimuth', 'elevation'), f"Geçersiz plane: {plane}"
        super().__init__(parent)

        self._plane   = plane
        self._echoes: list[EchoPoint] = []
        self._clusters: list         = []
        self._visible_clusters: list = []
        self._current_beam_az = 0.0
        self._max_range   = MAX_RANGE_M
        self._az_range    = AZIMUTH_RANGE
        self._el_range    = ELEVATION_RANGE
        self._step_deg    = 5.0
        self._dirty       = True

        # ── PyQtGraph PlotWidget ─────────────────────────────────────────────
        self._pw = pg.PlotWidget(background=BG_COLOR)
        self._pw.setAspectLocked(True)
        self._pw.setMouseEnabled(x=False, y=False)
        self._pw.hideAxis('left')
        self._pw.hideAxis('bottom')
        self._pw.setMenuEnabled(False)

        self._setup_view()
        self._build_grid()
        self._build_artists()

        # ── Layout ──────────────────────────────────────────────────────────
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._pw)

        # ── Refresh timer ────────────────────────────────────────────────────
        self._refresh_timer = QTimer(self)
        self._refresh_timer.timeout.connect(self._redraw)
        self._refresh_timer.start(REFRESH_MS)

    # ─── Yardımcılar ─────────────────────────────────────────────────────────

    @property
    def _active_range(self) -> float:
        return self._el_range if self._plane == 'elevation' else self._az_range

    @staticmethod
    def _to_xy(az_deg, r):
        t = np.deg2rad(np.asarray(az_deg, dtype=float))
        r = np.asarray(r, dtype=float)
        return r * np.sin(t), r * np.cos(t)

    # ─── View kurulumu ────────────────────────────────────────────────────────

    def _setup_view(self):
        rng    = self._active_range
        r      = self._max_range
        margin = r * 0.15
        x_max  = r * math.sin(math.radians(rng)) + margin
        self._pw.setXRange(-x_max, x_max, padding=0)
        self._pw.setYRange(-margin * 0.5, r + margin, padding=0)
        title = "Azimuth (θ)" if self._plane == 'azimuth' else "Elevation (φ)"
        self._pw.setTitle(title, color=TEXT_COLOR, size='10pt')

    # ─── Statik ızgara ────────────────────────────────────────────────────────

    def _build_grid(self):
        rng      = self._active_range
        grid_pen = pg.mkPen(GRID_COLOR, width=0.8,
                            style=Qt.PenStyle.DashLine)

        xs: list = []
        ys: list = []

        r_ticks = np.linspace(self._max_range / 5, self._max_range, 5)
        theta   = np.deg2rad(np.linspace(-rng, rng, 80))

        for r in r_ticks:
            xs.extend((r * np.sin(theta)).tolist()); xs.append(float('nan'))
            ys.extend((r * np.cos(theta)).tolist()); ys.append(float('nan'))

        az_ticks = np.arange(-rng, rng + 1e-6, self._step_deg)
        for az in az_ticks:
            t = math.radians(az)
            xs += [0.0, self._max_range * math.sin(t), float('nan')]
            ys += [0.0, self._max_range * math.cos(t), float('nan')]

        self._pw.addItem(pg.PlotCurveItem(
            np.array(xs, dtype=float), np.array(ys, dtype=float),
            pen=grid_pen,
        ))

        # Mesafe etiketleri
        for r in r_ticks:
            lbl = pg.TextItem(f"{r:.1f}m", color=TEXT_COLOR, anchor=(0, 0.5))
            lbl.setPos(0.05, float(r))
            self._pw.addItem(lbl)

        # Açı etiketleri
        for az in az_ticks:
            lx = float(self._max_range * 1.08 * math.sin(math.radians(az)))
            ly = float(self._max_range * 1.08 * math.cos(math.radians(az)))
            lbl = pg.TextItem(f"{int(az)}°", color=TEXT_COLOR, anchor=(0.5, 0.5))
            lbl.setPos(lx, ly)
            self._pw.addItem(lbl)

    # ─── Dinamik artist'ler ───────────────────────────────────────────────────

    def _build_artists(self):
        # Beam dolgu (dolu fan)
        self._beam_poly = _FilledPolygon(
            pen=pg.mkPen(BEAM_COLOR, width=1),
            brush=QBrush(QColor(51, 153, 255, 38)),
        )
        self._pw.addItem(self._beam_poly)

        # Beam merkez çizgisi
        self._beam_line = pg.PlotCurveItem(
            pen=pg.mkPen(BEAM_COLOR, width=1)
        )
        self._pw.addItem(self._beam_line)

        # Echo scatter
        self._echo_scatter = pg.ScatterPlotItem(
            size=5, pen=None, brush=_ECHO_BRUSH,
        )
        self._pw.addItem(self._echo_scatter)

        # Cluster scatter
        self._cluster_scatter = pg.ScatterPlotItem(
            size=12, symbol='d',
            pen=pg.mkPen('white', width=0.8),
            brush=pg.mkBrush('#ff9900'),
        )
        self._pw.addItem(self._cluster_scatter)

        # Sensör noktası (statik)
        self._pw.addItem(pg.ScatterPlotItem(
            [0], [0], size=10, symbol='o',
            pen=pg.mkPen('white', width=1),
            brush=pg.mkBrush(SENSOR_COLOR),
        ))

    # ─── Render ──────────────────────────────────────────────────────────────

    def _redraw(self):
        if not self._dirty:
            return
        self._update_beam()
        self._update_echoes()
        self._update_clusters()
        self._dirty = False

    def _update_beam(self):
        az    = self._current_beam_az
        lo    = math.radians(az - BEAM_HALF_DEG)
        hi    = math.radians(az + BEAM_HALF_DEG)
        sweep = np.linspace(lo, hi, 16)
        r     = self._max_range

        fan_x = [0.0] + (r * np.sin(sweep)).tolist() + [0.0]
        fan_y = [0.0] + (r * np.cos(sweep)).tolist() + [0.0]
        self._beam_poly.setPolygon(list(zip(fan_x, fan_y)))

        cx = r * math.sin(math.radians(az))
        cy = r * math.cos(math.radians(az))
        self._beam_line.setData([0.0, cx], [0.0, cy])

    def _update_echoes(self):
        if not self._echoes:
            self._echo_scatter.setData([], [])
            return
        azs    = np.array([e.azimuth_deg for e in self._echoes])
        ranges = np.array([e.range_m     for e in self._echoes])
        xs, ys = self._to_xy(azs, ranges)
        self._echo_scatter.setData(xs.tolist(), ys.tolist())

    def _update_clusters(self):
        visible = []
        for c in self._clusters:
            az, el, r = c.centroid_polar
            angle = el if self._plane == 'elevation' else az
            if abs(angle) <= self._active_range and r <= self._max_range:
                visible.append((angle, r, c))

        self._visible_clusters = [v[2] for v in visible]

        if not visible:
            self._cluster_scatter.setData([], [])
            return

        angles = np.array([v[0] for v in visible])
        ranges = np.array([v[1] for v in visible])
        xs, ys = self._to_xy(angles, ranges)
        self._cluster_scatter.setData(xs.tolist(), ys.tolist())

    # ─── Izgara / view yeniden oluşturma ─────────────────────────────────────

    def _rebuild(self):
        self._pw.clear()
        self._setup_view()
        self._build_grid()
        self._build_artists()
        self._dirty = True

    # ─── Public API ──────────────────────────────────────────────────────────

    def add_echo(self, azimuth_deg: float, range_m: float,
                 elevation_deg: float = None, strength: float = 1.0):
        plot_angle = (
            elevation_deg
            if (self._plane == 'elevation' and elevation_deg is not None)
            else azimuth_deg
        )
        if abs(plot_angle) > self._active_range:
            return
        if range_m < 0 or range_m > self._max_range:
            return
        self._echoes.append(EchoPoint(
            azimuth_deg=plot_angle, range_m=range_m,
            timestamp=time.time(), strength=strength,
        ))
        if len(self._echoes) > MAX_ECHOES:
            self._echoes = self._echoes[-MAX_ECHOES:]
        self._dirty = True

    def add_echoes_batch(self, items: list):
        """
        items: (azimuth_deg, elevation_deg, range_m)  ← 3-tuple
               (azimuth_deg, range_m)                 ← geriye dönük uyumluluk
        """
        if not items:
            return
        now         = time.time()
        three_tuple = len(items[0]) >= 3
        for item in items:
            if three_tuple:
                az, el, dist = item[0], item[1], item[2]
            else:
                az, dist = item[0], item[1]
                el = None
            plot_angle = el if (self._plane == 'elevation' and el is not None) else az
            if abs(plot_angle) > self._active_range:
                continue
            if dist < 0 or dist > self._max_range:
                continue
            self._echoes.append(EchoPoint(
                azimuth_deg=plot_angle, range_m=dist, timestamp=now,
            ))
        if len(self._echoes) > MAX_ECHOES:
            self._echoes = self._echoes[-MAX_ECHOES:]
        self._dirty = True

    def set_beam_direction(self, azimuth_deg: float, elevation_deg: float = 0.0):
        beam_angle = elevation_deg if self._plane == 'elevation' else azimuth_deg
        if float(beam_angle) != self._current_beam_az:
            self._current_beam_az = float(beam_angle)
            self._dirty = True

    def set_clusters(self, clusters: list):
        self._clusters = list(clusters)
        self._dirty = True

    def clear_echoes(self):
        self._echoes           = []
        self._clusters         = []
        self._visible_clusters = []
        self._dirty            = True

    def set_max_range(self, max_range_m: float):
        self._max_range = float(max_range_m)
        self._rebuild()

    def set_scan_range(self, azimuth_range_deg: float,
                       elevation_range_deg: float = None,
                       step_deg: float = 5.0):
        self._az_range = float(azimuth_range_deg)
        if elevation_range_deg is not None:
            self._el_range = float(elevation_range_deg)
        self._step_deg = float(step_deg)
        self._echoes   = [e for e in self._echoes
                          if abs(e.azimuth_deg) <= self._active_range]
        self._rebuild()


# ─── Geriye dönük uyumluluk ───────────────────────────────────────────────────
PolarSonarMap = SonarMap
