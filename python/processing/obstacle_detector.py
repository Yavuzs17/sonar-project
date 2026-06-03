"""
DBSCAN tabanlı engel tespit modülü.

Echo noktaları Kartezyen uzaya dönüştürülür, her 500 ms'de bir DBSCAN
kümeleme çalıştırılır ve cluster listesi `clusters_updated` sinyali ile
yayınlanır.

DBSCAN hesaplaması _DbscanWorker üzerinde ayrı bir QThread'de çalışır —
GUI thread bloke olmaz.

Koordinat dönüşümü map_3d.py'daki EchoPoint3D.to_cartesian() ile aynı:
    x = r * sin(az) * cos(el)
    y = r * sin(el)
    z = r * cos(az) * cos(el)
"""

import math
import time
from dataclasses import dataclass

import numpy as np
from PyQt6.QtCore import QObject, QThread, QTimer, pyqtSignal


MAX_BUFFER = 10000  # saklanacak maksimum echo noktası sayısı
RUN_MS     = 500    # DBSCAN çalıştırma aralığı (ms)

# Kayan zaman penceresi: sweep süresinden uzun tutulmalı.
# 5°/±30° sweep ≈ 6 sn → 20 sn 3 sweep verisini kapsar.
# 1° adımda sweep ~210 sn sürer; reset_buffer() sweep başında çağrıldığından
# bu window sadece çok yavaş taramalarda etkili güvenlik sınırı olur.
WINDOW_SEC = 20.0


@dataclass
class Cluster:
    cluster_id:    int
    cx:            float
    cy:            float
    cz:            float
    n_points:      int
    azimuth_deg:   float
    elevation_deg: float
    range_m:       float

    @property
    def count(self) -> int:
        return self.n_points

    @property
    def centroid_xyz(self) -> tuple:
        return (self.cx, self.cy, self.cz)

    @property
    def centroid_polar(self) -> tuple:
        return (self.azimuth_deg, self.elevation_deg, self.range_m)


def _to_cartesian(az_deg: float, el_deg: float, r: float) -> tuple:
    az = math.radians(az_deg)
    el = math.radians(el_deg)
    x = r * math.sin(az) * math.cos(el)
    y = r * math.sin(el)
    z = r * math.cos(az) * math.cos(el)
    return x, y, z


# ─── Worker — arka plan QThread'inde çalışır ─────────────────────────────────

class _DbscanWorker(QObject):
    """DBSCAN hesaplamasını arka planda yürütür. Main thread'i bloke etmez."""

    result = pyqtSignal(list)   # list[Cluster]

    def process(self, recent: list, eps: float, min_samples: int):
        """
        recent: [(az, el, r), ...] — main thread'den kopyalanmış anlık liste.
        Qt cross-thread signal ile çağrılır; worker thread'de yürür.
        """
        try:
            from sklearn.cluster import DBSCAN
        except ImportError:
            self.result.emit([])
            return

        if len(recent) < min_samples:
            self.result.emit([])
            return

        pts = np.array([_to_cartesian(az, el, r) for az, el, r in recent], dtype=float)

        labels = DBSCAN(eps=eps, min_samples=min_samples).fit_predict(pts)

        clusters: list[Cluster] = []
        for label in set(labels):
            if label == -1:
                continue
            mask    = labels == label
            members = pts[mask]
            cx, cy, cz = members.mean(axis=0)
            count      = int(mask.sum())
            r_mean  = float(np.linalg.norm([cx, cy, cz]))
            az_mean = float(np.degrees(np.arctan2(cx, cz)))
            el_mean = float(np.degrees(np.arcsin(cy / r_mean))) if r_mean > 1e-6 else 0.0
            clusters.append(Cluster(
                cluster_id=int(label),
                cx=float(cx), cy=float(cy), cz=float(cz),
                n_points=count,
                azimuth_deg=az_mean, elevation_deg=el_mean, range_m=r_mean,
            ))

        self.result.emit(clusters)


# ─── Ana sınıf ───────────────────────────────────────────────────────────────

class ObstacleDetector(QObject):
    """
    Echo tamponunu toplar, periyodik DBSCAN kümeleme uygular.

    DBSCAN worker ayrı bir QThread'de çalışır; GUI thread bloke olmaz.
    _busy bayrağı ile worker meşgulken yeni iş gönderilmez.

    Sinyaller:
        clusters_updated(list[Cluster]): yeni küme listesi hazır.
    """

    clusters_updated = pyqtSignal(list)
    _dispatch        = pyqtSignal(list, float, int)   # → worker.process

    def __init__(
        self,
        eps:         float = 0.30,
        min_samples: int   = 3,
        enabled:     bool  = True,
        parent=None,
    ):
        super().__init__(parent)

        self._eps         = eps
        self._min_samples = min_samples
        self._enabled     = enabled
        self._buffer: list[tuple] = []
        self._busy = False   # worker meşgul bayrağı — main thread'de okunup yazılır

        # ── Worker thread kurulumu ───────────────────────────────────────────
        self._worker = _DbscanWorker()
        self._thread = QThread()
        self._worker.moveToThread(self._thread)

        # Çapraz thread bağlantılar (Qt queued connection — thread-safe)
        self._dispatch.connect(self._worker.process)
        self._worker.result.connect(self._on_worker_result)

        self._thread.start()

        # ── Zamanlayıcı — main thread'de sadece iş gönderir ─────────────────
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._schedule_dbscan)
        if self._enabled:
            self._timer.start(RUN_MS)

    # ─── Zamanlayıcı callback (main thread) ─────────────────────────────────

    def _schedule_dbscan(self):
        """500 ms'de bir çağrılır. Worker meşgulse veya buffer boşsa atla."""
        if self._busy:
            return

        if not self._buffer:
            self.clusters_updated.emit([])
            return

        # Eski noktaları at (buffer'ı da temizle)
        cutoff = time.time() - WINDOW_SEC
        self._buffer = [(az, el, r, ts) for az, el, r, ts in self._buffer if ts >= cutoff]
        recent = [(az, el, r) for az, el, r, _ in self._buffer]

        if not recent:
            self.clusters_updated.emit([])
            return

        self._busy = True
        self._dispatch.emit(recent, self._eps, self._min_samples)

    def _on_worker_result(self, clusters: list):
        """Worker'dan gelen sonucu ilet (main thread'de çalışır — Qt queued)."""
        self._busy = False
        self.clusters_updated.emit(clusters)

    # ─── Public API ──────────────────────────────────────────────────────────

    def add_echo(self, azimuth_deg: float, elevation_deg: float, range_m: float):
        if not self._enabled:
            return
        self._buffer.append((azimuth_deg, elevation_deg, range_m, time.time()))
        if len(self._buffer) > MAX_BUFFER:
            self._buffer = self._buffer[-MAX_BUFFER:]

    def add_echoes_batch(self, items: list):
        """items: (az, el, dist) tuple listesi."""
        if not self._enabled or not items:
            return
        ts = time.time()
        self._buffer.extend((az, el, dist, ts) for az, el, dist in items)
        if len(self._buffer) > MAX_BUFFER:
            self._buffer = self._buffer[-MAX_BUFFER:]

    def notify_scan_start(self):
        """Yeni tarama döngüsü başladığında çağrılır — buffer temizlenir."""
        self._buffer.clear()

    def reset_buffer(self):
        """sweep_complete event'i geldiğinde çağrılır."""
        self._buffer.clear()
        self._busy = False
        self.clusters_updated.emit([])

    def set_params(self, eps: float, min_samples: int):
        self._eps         = float(eps)
        self._min_samples = int(min_samples)

    def set_enabled(self, enabled: bool):
        self._enabled = enabled
        if enabled:
            self._timer.start(RUN_MS)
        else:
            self._timer.stop()
            self.clusters_updated.emit([])

    def clear(self):
        self._buffer.clear()

    def shutdown(self):
        """Uygulama kapanırken çağrılır — thread'i düzgün kapatır."""
        self._timer.stop()
        self._thread.quit()
        self._thread.wait()
