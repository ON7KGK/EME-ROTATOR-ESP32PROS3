// ════════════════════════════════════════════════════════════════
// EME ROTATOR CONTROLLER — Configuration runtime (NVS)
// ════════════════════════════════════════════════════════════════
// Paramètres modifiables à chaud via JSON (port 4534)
// Persistés dans NVS (flash interne ESP32)
// Valeurs par défaut = #define de config.h
// ════════════════════════════════════════════════════════════════

#ifndef NVS_CONFIG_H
#define NVS_CONFIG_H

#include <Arduino.h>

// ── Sélection encodeur AZ ──
enum AzEncoderType : uint8_t {
    AZ_ENC_SIM     = 0,   // Simulation software
    AZ_ENC_HH12    = 1,   // HH-12 SSI 12-bit
    AZ_ENC_AS5048A = 2    // AS5048A SPI 14-bit
};

// ── Sélection capteur EL ──
enum ElSensorType : uint8_t {
    EL_ENC_SIM       = 0,  // Simulation software
    EL_ENC_HH12      = 1,  // HH-12 SSI 12-bit
    EL_ENC_WITMOTION = 2,  // WitMotion WT901C485 RS-485
    EL_ENC_AS5048A   = 3   // AS5048A SPI 14-bit
};

// ── Configuration runtime ──
struct RuntimeConfig {
    // Moteur — modifiables à chaud
    float motMaxDuty;       // % duty max
    float motMinDuty;       // % duty min (stall threshold)
    float motRampDeg;       // ° zone de rampe proportionnelle
    float motDeadband;      // ° zone morte

    // Limites mécaniques — modifiables à chaud
    float azMin;            // ° limite CCW (défaut 0)
    float azMax;            // ° limite CW (défaut 360)
    float elMin;            // ° limite DOWN (défaut 0)
    float elMax;            // ° limite UP (défaut 90)

    // Offset élévation (parabole offset-fed) — modifiable à chaud
    bool  elOffsetActive;   // appliquer l'offset au faisceau
    float elOffset;         // ° décalage (ex: 25° pour offset dish)

    // Position maintenance — stockée NVS
    float maintAz;          // ° AZ position maintenance
    float maintEl;          // ° EL position maintenance

    // Debug série — modifiable à chaud
    bool debugSerial;       // activer/désactiver messages Serial

    // Modules actifs — prennent effet au reboot
    bool oledActive;
    bool ethernetActive;
    bool gpsActive;
    bool mcp23017Active;
    bool nanoR4Active;

    // Sélection encodeur/capteur — prennent effet au reboot
    AzEncoderType azEncoder;
    ElSensorType  elSensor;
};

// Instance globale (accessible partout)
extern RuntimeConfig cfg;

// Charge config depuis NVS (ou défauts si première fois)
void cfgInit();

// Sauvegarde config dans NVS
void cfgSave();

// Remet les valeurs par défaut (sans sauver)
void cfgResetDefaults();

#endif // NVS_CONFIG_H
