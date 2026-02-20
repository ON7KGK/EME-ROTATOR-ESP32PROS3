"""
Compass Widget — Affichage azimut circulaire 0-360 degres.
Aiguille verte = position courante, marqueur rouge = cible.
"""

import math
from PySide6.QtWidgets import QWidget
from PySide6.QtCore import Qt, QRectF, QPointF
from PySide6.QtGui import QPainter, QPen, QColor, QFont, QBrush, QConicalGradient


class CompassWidget(QWidget):
    """Compass circulaire pour l'affichage azimut."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._az = 0.0
        self._target_az = 0.0
        self._moving = False
        self.setMinimumSize(200, 200)

    def setAzimuth(self, az: float):
        self._az = az
        self.update()

    def setTargetAzimuth(self, az: float):
        self._target_az = az
        self.update()

    def setMoving(self, moving: bool):
        self._moving = moving
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        side = min(self.width(), self.height())
        painter.translate(self.width() / 2, self.height() / 2)
        painter.scale(side / 240.0, side / 240.0)

        # Cercle exterieur
        painter.setPen(QPen(QColor(80, 80, 80), 2))
        painter.setBrush(QBrush(QColor(20, 20, 30)))
        painter.drawEllipse(QRectF(-105, -105, 210, 210))

        # Graduations et labels
        font = QFont("Consolas", 9)
        painter.setFont(font)
        for deg in range(0, 360, 10):
            painter.save()
            painter.rotate(deg)
            if deg % 30 == 0:
                painter.setPen(QPen(QColor(200, 200, 200), 2))
                painter.drawLine(0, -95, 0, -80)
            else:
                painter.setPen(QPen(QColor(100, 100, 100), 1))
                painter.drawLine(0, -95, 0, -88)
            painter.restore()

        # Labels cardinaux
        painter.setPen(QColor(220, 220, 220))
        labels = {0: "N", 90: "E", 180: "S", 270: "W"}
        for deg, lbl in labels.items():
            angle_rad = math.radians(deg - 90)
            x = 70 * math.cos(angle_rad)
            y = 70 * math.sin(angle_rad)
            painter.drawText(QRectF(x - 10, y - 8, 20, 16),
                             Qt.AlignCenter, lbl)

        # Marqueur cible (triangle rouge)
        painter.save()
        painter.rotate(self._target_az)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(220, 50, 50)))
        painter.drawPolygon([
            QPointF(0, -100),
            QPointF(-6, -88),
            QPointF(6, -88),
        ])
        painter.restore()

        # Aiguille position courante
        painter.save()
        painter.rotate(self._az)
        color = QColor(50, 220, 50) if not self._moving else QColor(255, 160, 0)
        painter.setPen(QPen(color, 3))
        painter.drawLine(0, 15, 0, -85)
        # Pointe
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(color))
        painter.drawPolygon([
            QPointF(0, -85),
            QPointF(-5, -70),
            QPointF(5, -70),
        ])
        painter.restore()

        # Point central
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(180, 180, 180)))
        painter.drawEllipse(QRectF(-5, -5, 10, 10))

        # Valeur numerique au centre
        painter.setPen(QColor(255, 255, 255))
        font_big = QFont("Consolas", 14, QFont.Bold)
        painter.setFont(font_big)
        painter.drawText(QRectF(-50, 20, 100, 25),
                         Qt.AlignCenter, f"{self._az:.1f}\u00b0")

        painter.end()
