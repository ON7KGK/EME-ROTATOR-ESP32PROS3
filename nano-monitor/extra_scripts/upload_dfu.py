"""
Script PlatformIO pour Arduino UNO R4 Minima (DFU upload).

Problème : dfu-util envoie un DFU detach, la carte redémarre,
mais Windows perd le device USB ("Lost device after RESET?").

Solution :
1. On remplace la commande upload de PlatformIO
2. On demande un double-tap RESET (la carte entre en DFU, LED pulse)
3. On lance dfu-util avec UNIQUEMENT le PID DFU (0x0374 pour Minima)
   → pas de detach, pas de reset USB, upload direct
"""
import time
import subprocess
import sys
import os

Import("env")

platform = env.PioPlatform()


def get_dfu_util():
    """Chemin vers dfu-util dans les packages PlatformIO."""
    pkg_dir = platform.get_package_dir("tool-dfuutil-arduino") or ""
    exe = "dfu-util.exe" if sys.platform == "win32" else "dfu-util"
    path = os.path.join(pkg_dir, exe)
    return path if os.path.exists(path) else "dfu-util"


def check_dfu_device(dfu_util):
    """Vérifie si un device en mode DFU (PID 0x0369) est présent."""
    try:
        result = subprocess.run(
            [dfu_util, "--list"],
            capture_output=True, text=True, timeout=5
        )
        return "0374" in result.stdout
    except Exception:
        return False


def prompt_and_wait_dfu(dfu_util):
    """Demande le double-tap RESET et attend le device DFU."""
    # Déjà en DFU ?
    if check_dfu_device(dfu_util):
        print("  Device DFU (0x0369) déjà présent !")
        return True

    print("")
    print("  >>> DOUBLE-TAP sur le bouton RESET de l'UNO R4 <<<")
    print("  >>> La LED doit PULSER (mode bootloader)        <<<")
    print("")

    timeout = 30
    start = time.time()
    while time.time() - start < timeout:
        if check_dfu_device(dfu_util):
            print("\r  Device DFU détecté !                        ")
            return True
        remaining = timeout - int(time.time() - start)
        sys.stdout.write(f"\r  Attente du device DFU... ({remaining}s) ")
        sys.stdout.flush()
        time.sleep(1)

    print("\r  TIMEOUT : device DFU non trouvé !             ")
    return False


def do_upload(source, target, env):
    """Upload custom : prompt RESET + dfu-util sur PID DFU uniquement."""
    dfu_util = get_dfu_util()
    firmware = str(source[0])

    print("")
    print("=" * 60)
    print("  UPLOAD UNO R4 MINIMA — DFU (PID 0x0374)")
    print("=" * 60)

    if not prompt_and_wait_dfu(dfu_util):
        print("  Abandon.")
        env.Exit(1)

    # Upload : on cible UNIQUEMENT le PID 0x0369 (DFU mode)
    # → dfu-util ne tente PAS de detach/reset
    cmd = [
        dfu_util,
        "-d", "0x2341:0x0374",   # DFU mode PID (UNO R4 Minima)
        "-a", "0",               # alt setting 0
        "-D", firmware           # download firmware
    ]

    print(f"  {' '.join(cmd)}")
    print("=" * 60)

    ret = subprocess.call(cmd)

    if ret != 0:
        print(f"\n  *** Upload échoué (code {ret}) ***")
        env.Exit(1)

    print("")
    print("=" * 60)
    print("  Upload OK ! La carte redémarre automatiquement.")
    print("=" * 60)
    print("")


# ── Remplacer complètement l'action upload de PlatformIO ──
# On vire le UPLOADCMD string et on met notre fonction Python
env.Replace(UPLOADCMD=do_upload)
