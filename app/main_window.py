"""
EME Rotator Controller - Fenetre principale PySide6
Monitoring temps reel + controle via JSON TCP port 4534.
"""

from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QLineEdit, QPushButton, QGroupBox, QMessageBox,
    QDoubleSpinBox,
)
from PySide6.QtCore import Qt, Slot
from PySide6.QtGui import QFont

from rotator_client import RotatorClient
from widgets import CompassWidget, ElevationWidget, StatusPanel
from widgets.settings_dialog import SettingsDialog


class MainWindow(QMainWindow):
    """Fenetre principale de l'application EME Rotator."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("EME Rotator Controller - ON7KGK")
        self.setMinimumSize(750, 500)

        # Theme sombre
        self.setStyleSheet("""
            QMainWindow { background-color: #1e1e2e; }
            QWidget { background-color: #1e1e2e; color: #cccccc; }
            QGroupBox {
                border: 1px solid #444;
                border-radius: 4px;
                margin-top: 8px;
                padding-top: 12px;
                font-weight: bold;
                color: #aaaaaa;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
            }
            QLineEdit, QDoubleSpinBox {
                background-color: #2a2a3e;
                border: 1px solid #555;
                border-radius: 3px;
                padding: 4px;
                color: #ffffff;
                font-family: Consolas;
            }
            QPushButton {
                background-color: #3a3a5e;
                border: 1px solid #555;
                border-radius: 4px;
                padding: 6px 12px;
                color: #ffffff;
                font-family: Consolas;
            }
            QPushButton:hover { background-color: #4a4a7e; }
            QPushButton:pressed { background-color: #2a2a4e; }
        """)

        # Client TCP
        self.client = RotatorClient()
        self.client.statusReceived.connect(self.onStatusReceived)
        self.client.connectionChanged.connect(self.onConnectionChanged)
        self.client.errorOccurred.connect(self.onError)
        self.client.configReceived.connect(self._onConfigReceived)
        self.client.configAck.connect(self._onConfigAck)
        self.client.configSaved.connect(self._onConfigSaved)
        self.client.configReset.connect(self._onConfigReset)

        # Dialogue configuration (cree a la demande)
        self._settingsDialog = None

        self._buildUI()

    def _buildUI(self):
        central = QWidget()
        self.setCentralWidget(central)
        mainLayout = QVBoxLayout(central)
        mainLayout.setSpacing(8)

        # ── Barre de connexion ──
        connLayout = QHBoxLayout()
        connLayout.addWidget(QLabel("IP:"))
        self.editHost = QLineEdit("192.168.0.200")
        self.editHost.setMaximumWidth(150)
        connLayout.addWidget(self.editHost)
        connLayout.addWidget(QLabel("Port:"))
        self.editPort = QLineEdit("4534")
        self.editPort.setMaximumWidth(60)
        connLayout.addWidget(self.editPort)
        self.btnConnect = QPushButton("Connect")
        self.btnConnect.clicked.connect(self.onConnectClicked)
        connLayout.addWidget(self.btnConnect)

        # Bouton engrenage (configuration)
        self.btnSettings = QPushButton("\u2699")
        self.btnSettings.setToolTip("Configuration ESP32")
        self.btnSettings.setStyleSheet(
            "QPushButton { font-size: 20px; padding: 2px 8px; }"
            "QPushButton:hover { background-color: #4a4a7e; }"
        )
        self.btnSettings.clicked.connect(self.onSettingsClicked)
        connLayout.addWidget(self.btnSettings)
        connLayout.addStretch()
        mainLayout.addLayout(connLayout)

        # ── Zone centrale : compass + elevation + controles ──
        centerLayout = QHBoxLayout()

        # Compass AZ
        azGroup = QGroupBox("Azimut")
        azLayout = QVBoxLayout(azGroup)
        self.compass = CompassWidget()
        azLayout.addWidget(self.compass)
        # Valeurs numeriques
        azNumLayout = QHBoxLayout()
        azNumLayout.addWidget(QLabel("Cur:"))
        self.labelAz = QLabel("---.-")
        self.labelAz.setFont(QFont("Consolas", 12, QFont.Bold))
        azNumLayout.addWidget(self.labelAz)
        azNumLayout.addWidget(QLabel("Tgt:"))
        self.labelAzTgt = QLabel("---.-")
        self.labelAzTgt.setFont(QFont("Consolas", 12))
        self.labelAzTgt.setStyleSheet("color: #ff6666;")
        azNumLayout.addWidget(self.labelAzTgt)
        azLayout.addLayout(azNumLayout)
        centerLayout.addWidget(azGroup)

        # Elevation
        elGroup = QGroupBox("Elevation")
        elLayout = QVBoxLayout(elGroup)
        self.elevation = ElevationWidget()
        elLayout.addWidget(self.elevation)
        # Valeurs numeriques
        elNumLayout = QHBoxLayout()
        elNumLayout.addWidget(QLabel("Cur:"))
        self.labelEl = QLabel("---.-")
        self.labelEl.setFont(QFont("Consolas", 12, QFont.Bold))
        elNumLayout.addWidget(self.labelEl)
        elNumLayout.addWidget(QLabel("Tgt:"))
        self.labelElTgt = QLabel("---.-")
        self.labelElTgt.setFont(QFont("Consolas", 12))
        self.labelElTgt.setStyleSheet("color: #ff6666;")
        elNumLayout.addWidget(self.labelElTgt)
        elLayout.addLayout(elNumLayout)
        centerLayout.addWidget(elGroup)

        # Controles
        ctrlGroup = QGroupBox("Controls")
        ctrlLayout = QVBoxLayout(ctrlGroup)

        # Go To
        gotoLayout = QGridLayout()
        gotoLayout.addWidget(QLabel("AZ:"), 0, 0)
        self.spinAz = QDoubleSpinBox()
        self.spinAz.setRange(0.0, 360.0)
        self.spinAz.setDecimals(1)
        self.spinAz.setSuffix("\u00b0")
        gotoLayout.addWidget(self.spinAz, 0, 1)
        gotoLayout.addWidget(QLabel("EL:"), 1, 0)
        self.spinEl = QDoubleSpinBox()
        self.spinEl.setRange(-30.0, 90.0)
        self.spinEl.setDecimals(1)
        self.spinEl.setSuffix("\u00b0")
        gotoLayout.addWidget(self.spinEl, 1, 1)
        self.btnGoto = QPushButton("GO TO")
        self.btnGoto.setStyleSheet(
            "QPushButton { background-color: #2255aa; font-weight: bold; }"
            "QPushButton:hover { background-color: #3366cc; }"
        )
        self.btnGoto.clicked.connect(self.onGoto)
        gotoLayout.addWidget(self.btnGoto, 0, 2, 2, 1)
        ctrlLayout.addLayout(gotoLayout)

        # Jog buttons
        jogLayout = QGridLayout()
        self.btnUp = QPushButton("\u25b2 UP")
        self.btnDown = QPushButton("\u25bc DOWN")
        self.btnCW = QPushButton("CW \u25b6")
        self.btnCCW = QPushButton("\u25c0 CCW")

        jogLayout.addWidget(self.btnUp, 0, 1)
        jogLayout.addWidget(self.btnCCW, 1, 0)
        jogLayout.addWidget(self.btnCW, 1, 2)
        jogLayout.addWidget(self.btnDown, 2, 1)

        # Jog : press = start, release = stop
        for btn, direction in [
            (self.btnUp, "up"), (self.btnDown, "down"),
            (self.btnCW, "cw"), (self.btnCCW, "ccw"),
        ]:
            btn.pressed.connect(lambda d=direction: self.onJogStart(d))
            btn.released.connect(self.onJogStop)

        ctrlLayout.addLayout(jogLayout)

        # STOP
        self.btnStop = QPushButton("STOP")
        self.btnStop.setStyleSheet(
            "QPushButton { background-color: #cc2222; font-size: 16px; "
            "font-weight: bold; padding: 12px; }"
            "QPushButton:hover { background-color: #ee3333; }"
        )
        self.btnStop.clicked.connect(self.onStop)
        ctrlLayout.addWidget(self.btnStop)

        # Maintenance (position NVS)
        self.btnMaint = QPushButton("\u2699 MAINTENANCE")
        self.btnMaint.setToolTip("Aller a la position de maintenance (configuree dans NVS)")
        self.btnMaint.setStyleSheet(
            "QPushButton { background-color: #886600; font-weight: bold; }"
            "QPushButton:hover { background-color: #aa8800; }"
        )
        self.btnMaint.clicked.connect(self.onMaintenance)
        ctrlLayout.addWidget(self.btnMaint)

        ctrlLayout.addStretch()
        centerLayout.addWidget(ctrlGroup)

        mainLayout.addLayout(centerLayout)

        # ── Monitor 10 GHz (telemetrie Nano R4) ──
        monGroup = QGroupBox("Monitor 10 GHz")
        monLayout = QHBoxLayout(monGroup)
        monLayout.setSpacing(20)

        # LED online Nano R4
        self.ledNano = QLabel("  Nano R4  ")
        self.ledNano.setStyleSheet(
            "background-color: #444444; color: white; "
            "border-radius: 3px; padding: 2px 6px; font-size: 11px; "
            "font-family: Consolas;"
        )
        monLayout.addWidget(self.ledNano)

        # A0
        monLayout.addWidget(QLabel("A0:"))
        self.labelA0 = QLabel("---- mV")
        self.labelA0.setFont(QFont("Consolas", 14, QFont.Bold))
        self.labelA0.setStyleSheet("color: #00ddff;")
        monLayout.addWidget(self.labelA0)

        # A1
        monLayout.addWidget(QLabel("A1:"))
        self.labelA1 = QLabel("---- mV")
        self.labelA1.setFont(QFont("Consolas", 14, QFont.Bold))
        self.labelA1.setStyleSheet("color: #ffdd00;")
        monLayout.addWidget(self.labelA1)

        # Uptime
        monLayout.addWidget(QLabel("Uptime:"))
        self.labelNanoUptime = QLabel("---")
        self.labelNanoUptime.setFont(QFont("Consolas", 11))
        monLayout.addWidget(self.labelNanoUptime)

        monLayout.addStretch()
        mainLayout.addWidget(monGroup)

        # ── Status bar ──
        self.statusPanel = StatusPanel()
        mainLayout.addWidget(self.statusPanel)

    # ════════════════════════════════════════════
    # Slots
    # ════════════════════════════════════════════

    def onConnectClicked(self):
        if self.client.isRunning():
            self.client.disconnectFrom()
            self.client.wait(2000)
            self.btnConnect.setText("Connect")
        else:
            host = self.editHost.text().strip()
            port = int(self.editPort.text().strip())
            self.client.connectTo(host, port)
            self.btnConnect.setText("Disconnect")

    @Slot(dict)
    def onStatusReceived(self, status: dict):
        az = status.get("az", 0.0)
        el = status.get("el", 0.0)
        az_tgt = status.get("az_target", 0.0)
        el_tgt = status.get("el_target", 0.0)
        moving = status.get("moving", False)

        self.compass.setAzimuth(az)
        self.compass.setTargetAzimuth(az_tgt)
        self.compass.setMoving(moving)

        self.elevation.setElevation(el)
        self.elevation.setTargetElevation(el_tgt)
        self.elevation.setMoving(moving)

        self.labelAz.setText(f"{az:.1f}\u00b0")
        self.labelEl.setText(f"{el:.1f}\u00b0")
        self.labelAzTgt.setText(f"{az_tgt:.1f}\u00b0")
        self.labelElTgt.setText(f"{el_tgt:.1f}\u00b0")

        self.statusPanel.updateFromStatus(status)

        # Telemetrie Nano R4
        nano = status.get("nano")
        if nano:
            online = nano.get("online", False)
            self.ledNano.setStyleSheet(
                f"background-color: {'#00cc00' if online else '#444444'}; "
                f"color: white; border-radius: 3px; padding: 2px 6px; "
                f"font-size: 11px; font-family: Consolas;"
            )
            self.labelA0.setText(f"{nano.get('a0_mv', 0)} mV")
            self.labelA1.setText(f"{nano.get('a1_mv', 0)} mV")
            uptime_s = nano.get("uptime", 0)
            if uptime_s >= 3600:
                self.labelNanoUptime.setText(
                    f"{uptime_s // 3600}h{(uptime_s % 3600) // 60:02d}m")
            elif uptime_s >= 60:
                self.labelNanoUptime.setText(f"{uptime_s // 60}m{uptime_s % 60:02d}s")
            else:
                self.labelNanoUptime.setText(f"{uptime_s}s")

    @Slot(bool)
    def onConnectionChanged(self, connected: bool):
        self.statusPanel.ledConnected.setOn(connected)
        if connected:
            self.btnConnect.setText("Disconnect")
        else:
            self.btnConnect.setText("Connect")

    @Slot(str)
    def onError(self, msg: str):
        self.statusBar().showMessage(msg, 5000)

    def onGoto(self):
        az = self.spinAz.value()
        el = self.spinEl.value()
        self.client.sendCommand({"cmd": "goto", "az": az, "el": el})

    def onJogStart(self, direction: str):
        self.client.sendCommand({"cmd": "jog", "dir": direction})

    def onJogStop(self):
        self.client.sendCommand({"cmd": "jog_stop"})

    def onStop(self):
        self.client.sendCommand({"cmd": "stop"})

    def onMaintenance(self):
        self.client.sendCommand({"cmd": "goto_maint"})

    # ════════════════════════════════════════════
    # Configuration (engrenage)
    # ════════════════════════════════════════════

    def onSettingsClicked(self):
        """Ouvre le dialogue de configuration et demande la config a l'ESP32."""
        if not self.client.isRunning():
            QMessageBox.warning(self, "Non connecte",
                                "Connectez-vous d'abord a l'ESP32.")
            return

        if self._settingsDialog is None:
            self._settingsDialog = SettingsDialog(self)
            self._settingsDialog.sendCommand.connect(self.client.sendCommand)
        self._settingsDialog.show()
        self._settingsDialog.raise_()
        # Demander la config actuelle
        self.client.sendCommand({"cmd": "get_config"})

    @Slot(dict)
    def _onConfigReceived(self, cfg: dict):
        """Config recue de l'ESP32 → remplir le dialogue."""
        if self._settingsDialog is not None:
            self._settingsDialog.loadFromConfig(cfg)

    @Slot(dict)
    def _onConfigAck(self, data: dict):
        """Reponse a set_config."""
        if self._settingsDialog is not None:
            self._settingsDialog.onConfigAck(data)

    @Slot()
    def _onConfigSaved(self):
        """Reponse a save_config."""
        if self._settingsDialog is not None:
            self._settingsDialog.onConfigSaved()

    @Slot()
    def _onConfigReset(self):
        """Reponse a reset_config."""
        if self._settingsDialog is not None:
            self._settingsDialog.onConfigReset()

    def closeEvent(self, event):
        self.client.disconnectFrom()
        self.client.wait(2000)
        event.accept()
