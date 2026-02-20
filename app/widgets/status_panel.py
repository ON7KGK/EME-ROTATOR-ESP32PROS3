"""
Status Panel — Indicateurs LED et texte pour l'etat du rotateur.
"""

from PySide6.QtWidgets import QWidget, QHBoxLayout, QLabel
from PySide6.QtCore import Qt
from PySide6.QtGui import QColor


class LedIndicator(QLabel):
    """Petit indicateur LED colore avec label."""

    def __init__(self, text: str, parent=None):
        super().__init__(parent)
        self._on = False
        self._on_color = "#00cc00"
        self._off_color = "#444444"
        self._text = text
        self._updateStyle()

    def setOnColor(self, color: str):
        self._on_color = color
        self._updateStyle()

    def setOn(self, on: bool):
        self._on = on
        self._updateStyle()

    def _updateStyle(self):
        color = self._on_color if self._on else self._off_color
        self.setText(f"  {self._text}  ")
        self.setStyleSheet(
            f"background-color: {color}; color: white; "
            f"border-radius: 3px; padding: 2px 6px; font-size: 11px; "
            f"font-family: Consolas;"
        )


class StatusPanel(QWidget):
    """Barre d'indicateurs de status."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        self.ledConnected = LedIndicator("TCP")
        self.ledMoving = LedIndicator("MOV")
        self.ledMoving.setOnColor("#ff8800")
        self.ledGps = LedIndicator("GPS")
        self.ledGps.setOnColor("#0088ff")
        self.ledStop = LedIndicator("STOP")
        self.ledStop.setOnColor("#ff0000")

        self.labelState = QLabel("---")
        self.labelState.setStyleSheet(
            "color: #cccccc; font-size: 13px; font-family: Consolas; "
            "font-weight: bold; padding: 0 10px;"
        )

        layout.addWidget(self.ledConnected)
        layout.addWidget(self.ledMoving)
        layout.addWidget(self.ledGps)
        layout.addWidget(self.ledStop)
        layout.addWidget(self.labelState)
        layout.addStretch()

    def updateFromStatus(self, status: dict):
        """Met a jour tous les indicateurs depuis un dict JSON status."""
        self.ledMoving.setOn(status.get("moving", False))
        self.ledGps.setOn(status.get("gps_fix", False))
        self.ledStop.setOn(status.get("stop_pressed", False))

        state = status.get("state", "---")
        stale = status.get("stale", False)
        if stale:
            self.labelState.setText(f"({state})")
            self.labelState.setStyleSheet(
                "color: #888888; font-size: 13px; font-family: Consolas; "
                "font-weight: bold; padding: 0 10px;"
            )
        else:
            self.labelState.setText(state)
            self.labelState.setStyleSheet(
                "color: #cccccc; font-size: 13px; font-family: Consolas; "
                "font-weight: bold; padding: 0 10px;"
            )
