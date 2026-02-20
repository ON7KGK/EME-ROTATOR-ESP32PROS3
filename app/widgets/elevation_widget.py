"""
Elevation Widget — Jauge arc -30 a 90 degres pour l'elevation.
Arc vert = position courante, marqueur rouge = cible.
Echelle etendue pour mode maintenance (angles negatifs).
"""

import math
from PySide6.QtWidgets import QWidget
from PySide6.QtCore import Qt, QRectF, QPointF
from PySide6.QtGui import QPainter, QPen, QColor, QFont, QBrush


class ElevationWidget(QWidget):
    """Jauge en arc pour l'affichage elevation -30 a 90 degres."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._el = 0.0
        self._target_el = 0.0
        self._moving = False
        self.setMinimumSize(200, 180)

    def setElevation(self, el: float):
        self._el = el
        self.update()

    def setTargetElevation(self, el: float):
        self._target_el = el
        self.update()

    def setMoving(self, moving: bool):
        self._moving = moving
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        w = self.width()
        h = self.height()

        # Centre de l'arc — decale pour accueillir -30° sous l'horizontale
        # Au-dessus du centre : radius (pour 90°)
        # En dessous du centre : radius * sin(30°) = 0.5 * radius
        margin_top = 25
        margin_bottom = 30
        radius = min(w / 2 - 25, (h - margin_top - margin_bottom) / 1.5)
        cx = w / 2
        cy = margin_top + radius

        # Arc de fond (-30 a 90 degres)
        arc_rect = QRectF(cx - radius, cy - radius, radius * 2, radius * 2)
        painter.setPen(QPen(QColor(80, 80, 80), 2))
        painter.setBrush(Qt.NoBrush)
        # drawArc : 1/16 degre, 0 = 3h, positif = anti-horaire
        # -30° a 90° = startAngle -30*16, span 120*16
        painter.drawArc(arc_rect, -30 * 16, 120 * 16)

        # Ligne horizontale 0° en pointille (reference)
        painter.setPen(QPen(QColor(60, 60, 80), 1, Qt.DashLine))
        painter.drawLine(QPointF(cx, cy), QPointF(cx + radius + 10, cy))

        # Graduations de -30 a 90
        font = QFont("Consolas", 8)
        painter.setFont(font)
        for deg in range(-30, 91, 10):
            angle_rad = math.radians(deg)
            cos_a = math.cos(angle_rad)
            sin_a = math.sin(angle_rad)

            x1 = cx + (radius - 5) * cos_a
            y1 = cy - (radius - 5) * sin_a
            x2 = cx + (radius + 5) * cos_a
            y2 = cy - (radius + 5) * sin_a

            if deg % 30 == 0:
                painter.setPen(QPen(QColor(200, 200, 200), 2))
                # Label
                lx = cx + (radius + 18) * cos_a
                ly = cy - (radius + 18) * sin_a
                painter.drawText(QRectF(lx - 18, ly - 8, 36, 16),
                                 Qt.AlignCenter, f"{deg}\u00b0")
            else:
                painter.setPen(QPen(QColor(100, 100, 100), 1))

            painter.drawLine(QPointF(x1, y1), QPointF(x2, y2))

        # Marqueur cible (triangle rouge)
        tgt_clamped = max(-30.0, min(90.0, self._target_el))
        tgt_rad = math.radians(tgt_clamped)
        tx = cx + (radius + 2) * math.cos(tgt_rad)
        ty = cy - (radius + 2) * math.sin(tgt_rad)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(220, 50, 50)))
        # Petit triangle pointant vers le centre
        perp_x = -math.sin(tgt_rad) * 5
        perp_y = -math.cos(tgt_rad) * 5
        inward_x = -math.cos(tgt_rad) * 10
        inward_y = math.sin(tgt_rad) * 10
        painter.drawPolygon([
            QPointF(tx, ty),
            QPointF(tx + perp_x + inward_x, ty + perp_y + inward_y),
            QPointF(tx - perp_x + inward_x, ty - perp_y + inward_y),
        ])

        # Aiguille position courante
        el_clamped = max(-30.0, min(90.0, self._el))
        el_rad = math.radians(el_clamped)
        nx = cx + (radius - 15) * math.cos(el_rad)
        ny = cy - (radius - 15) * math.sin(el_rad)
        color = QColor(50, 220, 50) if not self._moving else QColor(255, 160, 0)
        painter.setPen(QPen(color, 3))
        painter.drawLine(QPointF(cx, cy), QPointF(nx, ny))

        # Point central
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(180, 180, 180)))
        painter.drawEllipse(QPointF(cx, cy), 4, 4)

        # Valeur numerique
        painter.setPen(QColor(255, 255, 255))
        font_big = QFont("Consolas", 14, QFont.Bold)
        painter.setFont(font_big)
        painter.drawText(QRectF(cx - 50, cy - radius / 2 - 10, 100, 25),
                         Qt.AlignCenter, f"{self._el:.1f}\u00b0")

        painter.end()
