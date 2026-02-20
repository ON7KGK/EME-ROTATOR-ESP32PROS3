"""
Script PlatformIO pour ESP32-S3 USB CDC (ProS3).

Gère deux problèmes :
1. PRE-upload : le port COM change quand la carte passe en bootloader
   (ex: COM24 → COM25). On détecte et redirige automatiquement.
2. POST-upload : après le flash, la carte reste en bootloader.
   On attend que le port CDC réapparaisse = carte redémarrée.
"""
import time
import serial
import serial.tools.list_ports

Import("env")


def get_serial_ports():
    """Retourne l'ensemble des ports COM actuellement visibles."""
    return {p.device for p in serial.tools.list_ports.comports()}


def find_esp32s3_port():
    """Trouve le port de l'ESP32-S3 (CDC ou bootloader)."""
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if "usb" in desc or "esp" in desc or "303a" in hwid:
            return p.device
    return None


def trigger_1200bps_reset(port):
    """Envoie un reset 1200bps pour passer en mode bootloader."""
    try:
        ser = serial.Serial(port, baudrate=1200)
        ser.dtr = False
        time.sleep(0.1)
        ser.dtr = True
        time.sleep(0.1)
        ser.close()
        return True
    except Exception as e:
        print(f"  [upload] Erreur reset sur {port}: {e}")
        return False


def wait_for_new_port(ports_before, timeout=10):
    """Attend qu'un nouveau port COM apparaisse."""
    start = time.time()
    while time.time() - start < timeout:
        time.sleep(0.5)
        ports_now = get_serial_ports()
        new_ports = ports_now - ports_before
        if new_ports:
            return new_ports.pop()
    return None


def wait_for_port(port, timeout=10):
    """Attend qu'un port spécifique (ré)apparaisse."""
    start = time.time()
    while time.time() - start < timeout:
        time.sleep(0.5)
        if port in get_serial_ports():
            return True
    return False


def before_upload(source, target, env):
    """Hook pre-upload : reset CDC → détecte port bootloader → redirige."""
    print("=" * 60)
    print("  [upload] Auto-reset ESP32-S3 USB CDC → bootloader")
    print("=" * 60)

    cdc_port = find_esp32s3_port()
    if not cdc_port:
        print("  [upload] Aucun port ESP32-S3 détecté, upload par défaut...")
        return

    # Sauvegarder le port CDC pour le post-upload
    env["CDC_PORT"] = cdc_port
    print(f"  [upload] Port CDC : {cdc_port}")

    ports_before = get_serial_ports()

    print(f"  [upload] Reset 1200bps sur {cdc_port}...")
    if not trigger_1200bps_reset(cdc_port):
        return

    # Attendre disparition du port CDC
    time.sleep(1)
    ports_after_reset = get_serial_ports()

    # Attendre le port bootloader
    print("  [upload] Attente du port bootloader...")
    bootloader_port = wait_for_new_port(ports_after_reset, timeout=10)

    if bootloader_port:
        print(f"  [upload] Port bootloader : {bootloader_port}")
        env.Replace(UPLOAD_PORT=bootloader_port)
    elif cdc_port in get_serial_ports():
        print(f"  [upload] Port {cdc_port} toujours présent, upload direct.")
        env.Replace(UPLOAD_PORT=cdc_port)
    else:
        print("  [upload] ERREUR : aucun port trouvé ! Essaie BOOT + RESET.")

    print("=" * 60)


def after_upload(source, target, env):
    """Hook post-upload : attend que la carte redémarre en mode CDC."""
    cdc_port = env.get("CDC_PORT", "")
    if not cdc_port:
        return

    print("=" * 60)
    print("  [upload] Attente du redémarrage de la carte...")

    # Attendre que le port CDC réapparaisse (la carte a redémarré)
    if wait_for_port(cdc_port, timeout=15):
        print(f"  [upload] Carte redémarrée sur {cdc_port}")
        # Petit délai pour laisser le firmware démarrer
        time.sleep(1)
        print("  [upload] Firmware démarré !")
    else:
        # Vérifier si un autre port est apparu
        new_port = find_esp32s3_port()
        if new_port:
            print(f"  [upload] Carte redémarrée sur {new_port}")
        else:
            print("  [upload] Port CDC non retrouvé. Appuie sur RESET si nécessaire.")

    print("=" * 60)


# Désactiver le reset automatique de PlatformIO (on le gère nous-mêmes)
env.Replace(UPLOAD_PORT="")
env.AddPreAction("upload", before_upload)
env.AddPostAction("upload", after_upload)
