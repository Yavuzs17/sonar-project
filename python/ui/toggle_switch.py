"""
ToggleSwitch — QCheckBox drop-in replacement with painted knob.

Signal: toggled(bool)  — same as QCheckBox.toggled
API   : isChecked(), setChecked(bool), text(), setText(str)
"""

from PyQt6.QtCore import Qt, QSize, pyqtSignal
from PyQt6.QtGui  import QColor, QPainter, QPen
from PyQt6.QtWidgets import QWidget


class ToggleSwitch(QWidget):
    toggled = pyqtSignal(bool)

    _TRACK_ON  = QColor("#00aa66")
    _TRACK_OFF = QColor("#1a2e46")
    _BORDER    = QColor("#1e4a7a")
    _KNOB_ON   = QColor("#00e5ff")
    _KNOB_OFF  = QColor("#3a6080")
    _TEXT_CLR  = QColor("#8cb8d0")

    def __init__(self, text: str = "", parent=None):
        super().__init__(parent)
        self._checked = False
        self._text    = text
        self.setFixedHeight(26)
        self.setCursor(Qt.CursorShape.PointingHandCursor)

    # ── API ──────────────────────────────────────────────────────────────────

    def isChecked(self) -> bool:
        return self._checked

    def setChecked(self, checked: bool):
        if self._checked != bool(checked):
            self._checked = bool(checked)
            self.update()

    def text(self) -> str:
        return self._text

    def setText(self, text: str):
        self._text = text
        self.update()

    # ── Events ───────────────────────────────────────────────────────────────

    def mousePressEvent(self, _):
        self._checked = not self._checked
        self.toggled.emit(self._checked)
        self.update()

    def sizeHint(self) -> QSize:
        extra = 8 + self.fontMetrics().horizontalAdvance(self._text) if self._text else 0
        return QSize(44 + extra, 26)

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        tw, th = 42, 18
        ty = (self.height() - th) // 2

        # Track
        p.setPen(QPen(self._BORDER, 1))
        p.setBrush(self._TRACK_ON if self._checked else self._TRACK_OFF)
        p.drawRoundedRect(0, ty, tw, th, th // 2, th // 2)

        # Knob
        kd = th - 4
        ky = ty + 2
        kx = tw - kd - 2 if self._checked else 2
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(self._KNOB_ON if self._checked else self._KNOB_OFF)
        p.drawEllipse(kx, ky, kd, kd)

        # Label
        if self._text:
            p.setPen(self._TEXT_CLR)
            p.drawText(
                tw + 8, 0,
                self.width() - tw - 8, self.height(),
                Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft,
                self._text,
            )
        p.end()
