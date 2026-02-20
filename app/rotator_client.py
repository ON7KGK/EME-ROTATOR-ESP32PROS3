"""
EME Rotator Controller - Client TCP JSON
Connexion au port 4534 de l'ESP32, reception status JSON, envoi commandes.
"""

import json
import socket
from PySide6.QtCore import QThread, Signal, Slot, QMutex


class RotatorClient(QThread):
    """Thread reseau dedie : reception JSON status, envoi commandes."""

    # Signaux vers la GUI
    statusReceived = Signal(dict)       # Status JSON recu
    connectionChanged = Signal(bool)    # True=connecte, False=deconnecte
    errorOccurred = Signal(str)         # Message d'erreur

    def __init__(self, parent=None):
        super().__init__(parent)
        self._host = ""
        self._port = 4534
        self._sock = None
        self._running = False
        self._mutex = QMutex()

    def connectTo(self, host: str, port: int = 4534):
        """Demarre la connexion dans le thread."""
        self._host = host
        self._port = port
        self._running = True
        if not self.isRunning():
            self.start()

    def disconnectFrom(self):
        """Arrete proprement la connexion."""
        self._running = False

    def run(self):
        """Boucle principale du thread reseau."""
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(2.0)
            self._sock.connect((self._host, self._port))
            self._sock.settimeout(0.5)  # Timeout court pour le read
            self.connectionChanged.emit(True)
        except Exception as e:
            self.errorOccurred.emit(f"Connexion impossible: {e}")
            self.connectionChanged.emit(False)
            return

        buf = ""
        while self._running:
            try:
                data = self._sock.recv(1024)
                if not data:
                    # Serveur a ferme la connexion
                    break
                buf += data.decode("utf-8", errors="replace")

                # Traiter les lignes completes
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        msg = json.loads(line)
                        if msg.get("type") == "status":
                            self.statusReceived.emit(msg)
                    except json.JSONDecodeError:
                        pass

            except socket.timeout:
                continue
            except Exception as e:
                if self._running:
                    self.errorOccurred.emit(f"Erreur reseau: {e}")
                break

        # Nettoyage
        try:
            self._sock.close()
        except Exception:
            pass
        self._sock = None
        self.connectionChanged.emit(False)

    @Slot(dict)
    def sendCommand(self, cmd: dict):
        """Envoie une commande JSON au serveur (thread-safe)."""
        self._mutex.lock()
        try:
            if self._sock:
                data = json.dumps(cmd) + "\n"
                self._sock.sendall(data.encode("utf-8"))
        except Exception as e:
            self.errorOccurred.emit(f"Erreur envoi: {e}")
        finally:
            self._mutex.unlock()
