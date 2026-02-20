"""
EME Rotator Controller - Dialogue de configuration
Accessible via l'icone engrenage, communique avec l'ESP32 via JSON port 4534.
Commandes : get_config, set_config, save_config, reset_config, goto_maint.
"""

from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QLabel, QDoubleSpinBox, QCheckBox, QComboBox, QPushButton,
    QMessageBox, QFrame, QScrollArea, QWidget,
)
from PySide6.QtCore import Qt, Signal


class SettingsDialog(QDialog):
    """Dialogue de configuration du rotateur ESP32."""

    # Signal pour envoyer une commande JSON au client TCP
    sendCommand = Signal(dict)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("\u2699  Configuration")
        self.setMinimumWidth(460)
        self.setMinimumHeight(500)
        self.setStyleSheet("""
            QDialog { background-color: #1e1e2e; }
            QWidget { color: #cccccc; }
            QGroupBox {
                border: 1px solid #444;
                border-radius: 4px;
                margin-top: 8px;
                padding-top: 14px;
                font-weight: bold;
                color: #aaaaaa;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
            }
            QDoubleSpinBox, QComboBox {
                background-color: #2a2a3e;
                border: 1px solid #555;
                border-radius: 3px;
                padding: 4px;
                color: #ffffff;
                font-family: Consolas;
                min-width: 120px;
                min-height: 24px;
            }
            QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
                width: 20px;
                border: 1px solid #555;
                background-color: #3a3a5e;
            }
            QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover {
                background-color: #4a4a7e;
            }
            QDoubleSpinBox::up-arrow {
                image: none;
                border-left: 5px solid transparent;
                border-right: 5px solid transparent;
                border-bottom: 6px solid #cccccc;
                width: 0; height: 0;
            }
            QDoubleSpinBox::down-arrow {
                image: none;
                border-left: 5px solid transparent;
                border-right: 5px solid transparent;
                border-top: 6px solid #cccccc;
                width: 0; height: 0;
            }
            QCheckBox {
                spacing: 6px;
                font-family: Consolas;
            }
            QCheckBox::indicator {
                width: 16px;
                height: 16px;
            }
            QPushButton {
                background-color: #3a3a5e;
                border: 1px solid #555;
                border-radius: 4px;
                padding: 6px 14px;
                color: #ffffff;
                font-family: Consolas;
            }
            QPushButton:hover { background-color: #4a4a7e; }
            QPushButton:pressed { background-color: #2a2a4e; }
            QLabel { font-family: Consolas; font-size: 12px; }
            QScrollArea { border: none; }
        """)

        self._buildUI()

    def _buildUI(self):
        outerLayout = QVBoxLayout(self)
        outerLayout.setContentsMargins(4, 4, 4, 4)

        # Scroll area pour tout le contenu
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scrollWidget = QWidget()
        layout = QVBoxLayout(scrollWidget)
        layout.setSpacing(6)

        # ── Moteur (effet immediat) ──
        motGroup = QGroupBox("Moteur  (effet immediat)")
        motGrid = QGridLayout(motGroup)

        motGrid.addWidget(QLabel("Max duty (%):"), 0, 0)
        self.spinMaxDuty = QDoubleSpinBox()
        self.spinMaxDuty.setRange(10.0, 100.0)
        self.spinMaxDuty.setDecimals(1)
        self.spinMaxDuty.setSuffix(" %")
        motGrid.addWidget(self.spinMaxDuty, 0, 1)

        motGrid.addWidget(QLabel("Min duty (%):"), 1, 0)
        self.spinMinDuty = QDoubleSpinBox()
        self.spinMinDuty.setRange(5.0, 100.0)
        self.spinMinDuty.setDecimals(1)
        self.spinMinDuty.setSuffix(" %")
        motGrid.addWidget(self.spinMinDuty, 1, 1)

        motGrid.addWidget(QLabel("Rampe (\u00b0):"), 2, 0)
        self.spinRamp = QDoubleSpinBox()
        self.spinRamp.setRange(1.0, 90.0)
        self.spinRamp.setDecimals(1)
        self.spinRamp.setSuffix(" \u00b0")
        motGrid.addWidget(self.spinRamp, 2, 1)

        motGrid.addWidget(QLabel("Deadband (\u00b0):"), 3, 0)
        self.spinDeadband = QDoubleSpinBox()
        self.spinDeadband.setRange(0.1, 10.0)
        self.spinDeadband.setDecimals(1)
        self.spinDeadband.setSuffix(" \u00b0")
        motGrid.addWidget(self.spinDeadband, 3, 1)

        layout.addWidget(motGroup)

        # ── Limites mecaniques (effet immediat) ──
        limGroup = QGroupBox("Limites mecaniques  (effet immediat)")
        limGrid = QGridLayout(limGroup)

        limGrid.addWidget(QLabel("AZ CCW min:"), 0, 0)
        self.spinAzMin = QDoubleSpinBox()
        self.spinAzMin.setRange(-180.0, 540.0)
        self.spinAzMin.setDecimals(1)
        self.spinAzMin.setSuffix(" \u00b0")
        limGrid.addWidget(self.spinAzMin, 0, 1)

        limGrid.addWidget(QLabel("AZ CW max:"), 0, 2)
        self.spinAzMax = QDoubleSpinBox()
        self.spinAzMax.setRange(-180.0, 540.0)
        self.spinAzMax.setDecimals(1)
        self.spinAzMax.setSuffix(" \u00b0")
        limGrid.addWidget(self.spinAzMax, 0, 3)

        limGrid.addWidget(QLabel("EL DOWN min:"), 1, 0)
        self.spinElMin = QDoubleSpinBox()
        self.spinElMin.setRange(-90.0, 180.0)
        self.spinElMin.setDecimals(1)
        self.spinElMin.setSuffix(" \u00b0")
        limGrid.addWidget(self.spinElMin, 1, 1)

        limGrid.addWidget(QLabel("EL UP max:"), 1, 2)
        self.spinElMax = QDoubleSpinBox()
        self.spinElMax.setRange(-90.0, 180.0)
        self.spinElMax.setDecimals(1)
        self.spinElMax.setSuffix(" \u00b0")
        limGrid.addWidget(self.spinElMax, 1, 3)

        layout.addWidget(limGroup)

        # ── Offset elevation (effet immediat) ──
        offGroup = QGroupBox("Offset elevation  (parabole offset-fed)")
        offGrid = QGridLayout(offGroup)

        self.chkElOffset = QCheckBox("Activer offset")
        offGrid.addWidget(self.chkElOffset, 0, 0)

        offGrid.addWidget(QLabel("Offset (\u00b0):"), 0, 1)
        self.spinElOffset = QDoubleSpinBox()
        self.spinElOffset.setRange(-90.0, 90.0)
        self.spinElOffset.setDecimals(1)
        self.spinElOffset.setSuffix(" \u00b0")
        offGrid.addWidget(self.spinElOffset, 0, 2)

        layout.addWidget(offGroup)

        # ── Position maintenance ──
        maintGroup = QGroupBox("Position maintenance")
        maintGrid = QGridLayout(maintGroup)

        maintGrid.addWidget(QLabel("AZ:"), 0, 0)
        self.spinMaintAz = QDoubleSpinBox()
        self.spinMaintAz.setRange(-180.0, 540.0)
        self.spinMaintAz.setDecimals(1)
        self.spinMaintAz.setSuffix(" \u00b0")
        maintGrid.addWidget(self.spinMaintAz, 0, 1)

        maintGrid.addWidget(QLabel("EL:"), 0, 2)
        self.spinMaintEl = QDoubleSpinBox()
        self.spinMaintEl.setRange(-90.0, 180.0)
        self.spinMaintEl.setDecimals(1)
        self.spinMaintEl.setSuffix(" \u00b0")
        maintGrid.addWidget(self.spinMaintEl, 0, 3)

        layout.addWidget(maintGroup)

        # ── Modules (reboot requis) ──
        modGroup = QGroupBox("Modules  (reboot requis)")
        modGrid = QGridLayout(modGroup)

        self.chkOled = QCheckBox("OLED SSD1306")
        self.chkGps = QCheckBox("GPS NEO-6M")
        self.chkNanoR4 = QCheckBox("Nano R4 monitor")
        self.chkDebug = QCheckBox("Debug Serial")

        modGrid.addWidget(self.chkOled, 0, 0)
        modGrid.addWidget(self.chkGps, 0, 1)
        modGrid.addWidget(self.chkNanoR4, 1, 0)
        modGrid.addWidget(self.chkDebug, 1, 1)

        layout.addWidget(modGroup)

        # ── Encodeurs / capteurs (reboot requis) ──
        encGroup = QGroupBox("Encodeurs / Capteurs  (reboot requis)")
        encGrid = QGridLayout(encGroup)

        encGrid.addWidget(QLabel("Azimut:"), 0, 0)
        self.comboAzEnc = QComboBox()
        self.comboAzEnc.addItems(["sim", "hh12", "as5048a"])
        encGrid.addWidget(self.comboAzEnc, 0, 1)

        encGrid.addWidget(QLabel("Elevation:"), 1, 0)
        self.comboElSens = QComboBox()
        self.comboElSens.addItems(["sim", "hh12", "witmotion", "as5048a"])
        encGrid.addWidget(self.comboElSens, 1, 1)

        layout.addWidget(encGroup)

        scroll.setWidget(scrollWidget)
        outerLayout.addWidget(scroll)

        # ── Separateur ──
        sep = QFrame()
        sep.setFrameShape(QFrame.HLine)
        sep.setStyleSheet("color: #444;")
        outerLayout.addWidget(sep)

        # ── Boutons d'action ──
        btnLayout = QHBoxLayout()

        self.btnApply = QPushButton("Appliquer")
        self.btnApply.setToolTip("Envoie set_config (moteur/limites = immediat, modules = reboot)")
        self.btnApply.setStyleSheet(
            "QPushButton { background-color: #2255aa; font-weight: bold; }"
            "QPushButton:hover { background-color: #3366cc; }"
        )
        self.btnApply.clicked.connect(self._onApply)
        btnLayout.addWidget(self.btnApply)

        self.btnSave = QPushButton("Sauver NVS")
        self.btnSave.setToolTip("Sauvegarde dans la flash ESP32 (persistant)")
        self.btnSave.setStyleSheet(
            "QPushButton { background-color: #227722; font-weight: bold; }"
            "QPushButton:hover { background-color: #339933; }"
        )
        self.btnSave.clicked.connect(self._onSave)
        btnLayout.addWidget(self.btnSave)

        self.btnReset = QPushButton("Reset")
        self.btnReset.setToolTip("Remet les valeurs par defaut (compile-time)")
        self.btnReset.setStyleSheet(
            "QPushButton { background-color: #883322; }"
            "QPushButton:hover { background-color: #aa4433; }"
        )
        self.btnReset.clicked.connect(self._onReset)
        btnLayout.addWidget(self.btnReset)

        self.btnReboot = QPushButton("Reboot ESP32")
        self.btnReboot.setToolTip("Sauvegarde NVS puis redemarre l'ESP32")
        self.btnReboot.setStyleSheet(
            "QPushButton { background-color: #cc6600; font-weight: bold; }"
            "QPushButton:hover { background-color: #ee7700; }"
        )
        self.btnReboot.clicked.connect(self._onReboot)
        btnLayout.addWidget(self.btnReboot)

        btnLayout.addStretch()

        self.btnClose = QPushButton("Fermer")
        self.btnClose.clicked.connect(self.close)
        btnLayout.addWidget(self.btnClose)

        outerLayout.addLayout(btnLayout)

        # Label feedback
        self.labelFeedback = QLabel("")
        self.labelFeedback.setStyleSheet("color: #888; font-size: 11px;")
        outerLayout.addWidget(self.labelFeedback)

    # ════════════════════════════════════════════
    # Remplir les champs depuis la config recue
    # ════════════════════════════════════════════

    def loadFromConfig(self, cfg: dict):
        """Remplit le dialogue avec les valeurs JSON recues de get_config."""
        self.spinMaxDuty.setValue(cfg.get("mot_max_duty", 90.0))
        self.spinMinDuty.setValue(cfg.get("mot_min_duty", 20.0))
        self.spinRamp.setValue(cfg.get("mot_ramp_deg", 10.0))
        self.spinDeadband.setValue(cfg.get("mot_deadband", 0.2))

        self.spinAzMin.setValue(cfg.get("az_min", 0.0))
        self.spinAzMax.setValue(cfg.get("az_max", 360.0))
        self.spinElMin.setValue(cfg.get("el_min", 0.0))
        self.spinElMax.setValue(cfg.get("el_max", 90.0))

        self.chkElOffset.setChecked(cfg.get("el_offset_active", False))
        self.spinElOffset.setValue(cfg.get("el_offset", 0.0))

        self.spinMaintAz.setValue(cfg.get("maint_az", 180.0))
        self.spinMaintEl.setValue(cfg.get("maint_el", 0.0))

        self.chkDebug.setChecked(cfg.get("debug", True))
        self.chkOled.setChecked(cfg.get("oled", True))
        self.chkGps.setChecked(cfg.get("gps", True))
        self.chkNanoR4.setChecked(cfg.get("nano_r4", False))

        az_enc = cfg.get("az_encoder", "sim")
        idx = self.comboAzEnc.findText(az_enc)
        if idx >= 0:
            self.comboAzEnc.setCurrentIndex(idx)

        el_sens = cfg.get("el_sensor", "sim")
        idx = self.comboElSens.findText(el_sens)
        if idx >= 0:
            self.comboElSens.setCurrentIndex(idx)

        self.labelFeedback.setText("Config chargee depuis ESP32")
        self.labelFeedback.setStyleSheet("color: #888; font-size: 11px;")

    # ════════════════════════════════════════════
    # Actions
    # ════════════════════════════════════════════

    def _onApply(self):
        """Envoie set_config avec toutes les valeurs."""
        cmd = {
            "cmd": "set_config",
            "mot_max_duty": self.spinMaxDuty.value(),
            "mot_min_duty": self.spinMinDuty.value(),
            "mot_ramp_deg": self.spinRamp.value(),
            "mot_deadband": self.spinDeadband.value(),
            "az_min": self.spinAzMin.value(),
            "az_max": self.spinAzMax.value(),
            "el_min": self.spinElMin.value(),
            "el_max": self.spinElMax.value(),
            "el_offset_active": self.chkElOffset.isChecked(),
            "el_offset": self.spinElOffset.value(),
            "maint_az": self.spinMaintAz.value(),
            "maint_el": self.spinMaintEl.value(),
            "debug": self.chkDebug.isChecked(),
            "oled": self.chkOled.isChecked(),
            "gps": self.chkGps.isChecked(),
            "nano_r4": self.chkNanoR4.isChecked(),
            "az_encoder": self.comboAzEnc.currentText(),
            "el_sensor": self.comboElSens.currentText(),
        }
        self.sendCommand.emit(cmd)
        self.labelFeedback.setText("set_config envoye...")

    def _onSave(self):
        """Envoie save_config pour persister dans NVS."""
        self.sendCommand.emit({"cmd": "save_config"})
        self.labelFeedback.setText("save_config envoye...")

    def _onReset(self):
        """Remet les defauts apres confirmation."""
        reply = QMessageBox.question(
            self,
            "Reset configuration",
            "Remettre toutes les valeurs par defaut ?\n"
            "(les defauts compile-time seront restaures et sauves)",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply == QMessageBox.Yes:
            self.sendCommand.emit({"cmd": "reset_config"})
            self.labelFeedback.setText("reset_config envoye...")

    def _onReboot(self):
        """Sauvegarde NVS puis redemarre l'ESP32."""
        reply = QMessageBox.question(
            self,
            "Reboot ESP32",
            "Sauvegarder la config et redemarrer l'ESP32 ?\n"
            "(la connexion TCP sera perdue)",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply == QMessageBox.Yes:
            self.sendCommand.emit({"cmd": "save_config"})
            self.sendCommand.emit({"cmd": "reboot"})
            self.labelFeedback.setText("Sauvegarde + reboot en cours...")
            self.labelFeedback.setStyleSheet("color: #ff8800; font-size: 11px;")

    # ════════════════════════════════════════════
    # Slots pour les reponses ESP32
    # ════════════════════════════════════════════

    def onConfigAck(self, data: dict):
        """Reponse a set_config."""
        if data.get("reboot"):
            self.labelFeedback.setText(
                "Config appliquee. Reboot requis pour les modules/encodeurs."
            )
            self.labelFeedback.setStyleSheet("color: #ff8800; font-size: 11px;")
        else:
            self.labelFeedback.setText("Config appliquee (effet immediat).")
            self.labelFeedback.setStyleSheet("color: #00cc00; font-size: 11px;")

    def onConfigSaved(self):
        """Reponse a save_config."""
        self.labelFeedback.setText("Config sauvegardee dans NVS (flash).")
        self.labelFeedback.setStyleSheet("color: #00cc00; font-size: 11px;")

    def onConfigReset(self):
        """Reponse a reset_config — la config mise a jour arrive ensuite."""
        self.labelFeedback.setText("Defauts restaures et sauves.")
        self.labelFeedback.setStyleSheet("color: #00cc00; font-size: 11px;")
