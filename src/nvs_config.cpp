// ════════════════════════════════════════════════════════════════
// EME ROTATOR CONTROLLER — Configuration runtime NVS (Impl.)
// ════════════════════════════════════════════════════════════════
// Stockage key-value dans la flash interne ESP32
// Namespace NVS : "rotcfg" — clés limitées à 15 caractères
// ════════════════════════════════════════════════════════════════

#include "nvs_config.h"
#include "config.h"
#include <Preferences.h>

// Instance globale
RuntimeConfig cfg;

// ════════════════════════════════════════════════════════════════
// VALEURS PAR DÉFAUT (depuis les #define de config.h)
// ════════════════════════════════════════════════════════════════

void cfgResetDefaults() {
    // Moteur
    cfg.motMaxDuty  = MOT_MAX_DUTY;
    cfg.motMinDuty  = MOT_MIN_DUTY;
    cfg.motRampDeg  = MOT_RAMP_DEG;
    cfg.motDeadband = MOT_DEADBAND_DEG;

    // Modules — défauts = ce qui est activé à la compilation
    cfg.oledActive     = (bool)ENABLE_OLED;
    cfg.ethernetActive = (bool)ENABLE_ETHERNET;
    cfg.gpsActive      = (bool)ENABLE_GPS;
    cfg.mcp23017Active = (bool)ENABLE_MCP23017;
    cfg.nanoR4Active   = (bool)ENABLE_NANO_R4;

    // AZ encoder — défaut selon flags compile
    #if ENABLE_AZ_HH12
        cfg.azEncoder = AZ_ENC_HH12;
    #elif ENABLE_AZ_AS5048A
        cfg.azEncoder = AZ_ENC_AS5048A;
    #else
        cfg.azEncoder = AZ_ENC_SIM;
    #endif

    // EL sensor — défaut selon flags compile
    #if ENABLE_EL_HWT901B
        cfg.elSensor = EL_ENC_WITMOTION;
    #elif ENABLE_EL_HH12
        cfg.elSensor = EL_ENC_HH12;
    #else
        cfg.elSensor = EL_ENC_SIM;
    #endif
}

// ════════════════════════════════════════════════════════════════
// CHARGEMENT DEPUIS NVS
// ════════════════════════════════════════════════════════════════

void cfgInit() {
    // Partir des défauts
    cfgResetDefaults();

    #if ENABLE_NVS_CONFIG
    {
        Preferences prefs;
        prefs.begin("rotcfg", true);  // lecture seule

        if (prefs.isKey("ver")) {
            // Config existe dans NVS → charger
            cfg.motMaxDuty  = prefs.getFloat("motMaxDuty", cfg.motMaxDuty);
            cfg.motMinDuty  = prefs.getFloat("motMinDuty", cfg.motMinDuty);
            cfg.motRampDeg  = prefs.getFloat("motRampDeg", cfg.motRampDeg);
            cfg.motDeadband = prefs.getFloat("motDeadb",   cfg.motDeadband);

            cfg.oledActive     = prefs.getBool("oled",     cfg.oledActive);
            cfg.ethernetActive = prefs.getBool("ethernet", cfg.ethernetActive);
            cfg.gpsActive      = prefs.getBool("gps",      cfg.gpsActive);
            cfg.mcp23017Active = prefs.getBool("mcp23017", cfg.mcp23017Active);
            cfg.nanoR4Active   = prefs.getBool("nanoR4",   cfg.nanoR4Active);

            cfg.azEncoder = (AzEncoderType)prefs.getUChar("azEnc",  (uint8_t)cfg.azEncoder);
            cfg.elSensor  = (ElSensorType) prefs.getUChar("elSens", (uint8_t)cfg.elSensor);

            DEBUG_PRINTLN("[CFG] Config chargée depuis NVS");
        } else {
            DEBUG_PRINTLN("[CFG] NVS vierge, utilisation des défauts");
        }

        prefs.end();
    }
    #endif

    // ── Validation : cohérence des dépendances ──

    // Moteurs nécessitent MCP23017
    #if ENABLE_MOTORS
    if (!cfg.mcp23017Active) {
        cfg.mcp23017Active = true;
        DEBUG_PRINTLN("[CFG] WARN: MCP23017 forcé ON (requis par moteurs)");
    }
    #endif

    // Vérifier que l'encodeur AZ sélectionné est compilé
    #if !ENABLE_AZ_HH12
    if (cfg.azEncoder == AZ_ENC_HH12) {
        cfg.azEncoder = AZ_ENC_SIM;
        DEBUG_PRINTLN("[CFG] WARN: HH-12 AZ non compilé → SIM");
    }
    #endif
    #if !ENABLE_AZ_AS5048A
    if (cfg.azEncoder == AZ_ENC_AS5048A) {
        cfg.azEncoder = AZ_ENC_SIM;
        DEBUG_PRINTLN("[CFG] WARN: AS5048A AZ non compilé → SIM");
    }
    #endif

    // Vérifier que le capteur EL sélectionné est compilé
    #if !ENABLE_EL_HH12
    if (cfg.elSensor == EL_ENC_HH12) {
        cfg.elSensor = EL_ENC_SIM;
        DEBUG_PRINTLN("[CFG] WARN: HH-12 EL non compilé → SIM");
    }
    #endif
    #if !ENABLE_EL_HWT901B
    if (cfg.elSensor == EL_ENC_WITMOTION) {
        cfg.elSensor = EL_ENC_SIM;
        DEBUG_PRINTLN("[CFG] WARN: WitMotion EL non compilé → SIM");
    }
    #endif

    // Clamp valeurs moteur
    if (cfg.motMaxDuty < 10.0f)  cfg.motMaxDuty = 10.0f;
    if (cfg.motMaxDuty > 100.0f) cfg.motMaxDuty = 100.0f;
    if (cfg.motMinDuty < 5.0f)   cfg.motMinDuty = 5.0f;
    if (cfg.motMinDuty > cfg.motMaxDuty) cfg.motMinDuty = cfg.motMaxDuty;
    if (cfg.motRampDeg < 1.0f)   cfg.motRampDeg = 1.0f;
    if (cfg.motDeadband < 0.1f)  cfg.motDeadband = 0.1f;

    // ── Log résumé ──
    DEBUG_PRINTLN("[CFG] ── Config active ──");
    DEBUG_PRINT("  Moteur: max="); DEBUG_PRINT(String(cfg.motMaxDuty, 0));
    DEBUG_PRINT("% min=");         DEBUG_PRINT(String(cfg.motMinDuty, 0));
    DEBUG_PRINT("% rampe=");       DEBUG_PRINT(String(cfg.motRampDeg, 0));
    DEBUG_PRINT("° dead=");        DEBUG_PRINT(String(cfg.motDeadband, 1));
    DEBUG_PRINTLN("°");

    const char *azNames[] = {"SIM", "HH-12", "AS5048A"};
    const char *elNames[] = {"SIM", "HH-12", "WitMotion", "AS5048A"};
    DEBUG_PRINT("  AZ: "); DEBUG_PRINT(azNames[cfg.azEncoder]);
    DEBUG_PRINT("  EL: "); DEBUG_PRINTLN(elNames[cfg.elSensor]);

    DEBUG_PRINT("  OLED=");     DEBUG_PRINT(cfg.oledActive);
    DEBUG_PRINT(" ETH=");       DEBUG_PRINT(cfg.ethernetActive);
    DEBUG_PRINT(" GPS=");       DEBUG_PRINT(cfg.gpsActive);
    DEBUG_PRINT(" MCP=");       DEBUG_PRINT(cfg.mcp23017Active);
    DEBUG_PRINT(" NanoR4=");    DEBUG_PRINTLN(cfg.nanoR4Active);
}

// ════════════════════════════════════════════════════════════════
// SAUVEGARDE DANS NVS
// ════════════════════════════════════════════════════════════════

void cfgSave() {
    #if ENABLE_NVS_CONFIG
    Preferences prefs;
    prefs.begin("rotcfg", false);  // lecture-écriture

    prefs.putUChar("ver", 1);

    prefs.putFloat("motMaxDuty", cfg.motMaxDuty);
    prefs.putFloat("motMinDuty", cfg.motMinDuty);
    prefs.putFloat("motRampDeg", cfg.motRampDeg);
    prefs.putFloat("motDeadb",   cfg.motDeadband);

    prefs.putBool("oled",     cfg.oledActive);
    prefs.putBool("ethernet", cfg.ethernetActive);
    prefs.putBool("gps",      cfg.gpsActive);
    prefs.putBool("mcp23017", cfg.mcp23017Active);
    prefs.putBool("nanoR4",   cfg.nanoR4Active);

    prefs.putUChar("azEnc",  (uint8_t)cfg.azEncoder);
    prefs.putUChar("elSens", (uint8_t)cfg.elSensor);

    prefs.end();
    DEBUG_PRINTLN("[CFG] Sauvegardé dans NVS");
    #else
    DEBUG_PRINTLN("[CFG] NVS désactivé, config non sauvegardée");
    #endif
}
