// ════════════════════════════════════════════════════════════════
// EME ROTATOR CONTROLLER — Encodeurs HH-12 SSI (Implementation)
// ════════════════════════════════════════════════════════════════
// AZ : HH-12 absolu sur axe final — mapping direct 0-4095 = 0-360°
// EL : HH-12 absolu sur axe final — mapping direct 0-4095 = 0-90°
// Pas de multi-tours, pas de calibration, pas d'EEPROM
// ════════════════════════════════════════════════════════════════

#include "encoder.h"

// ════════════════════════════════════════════════════════════════
// VARIABLES GLOBALES
// ════════════════════════════════════════════════════════════════

float currentAz = 0.0f;
float currentEl = 0.0f;

int rawCountsAz = 0;
int rawCountsEl = 0;

// Throttle lecture
static unsigned long lastEncoderReadTime = 0;

// ════════════════════════════════════════════════════════════════
// VARIABLES SIMULATION (si SIMULATE_ENCODERS == 1)
// ════════════════════════════════════════════════════════════════

#if SIMULATE_ENCODERS
    // targetAz/targetEl sont définis dans main.cpp
    extern float targetAz;
    extern float targetEl;
#endif

// ════════════════════════════════════════════════════════════════
// INITIALISATION
// ════════════════════════════════════════════════════════════════

void setupEncoders() {
    #if !SIMULATE_ENCODERS
        // Configuration pins SSI (bit-bang)
        pinMode(PIN_HH12_SCLK, OUTPUT);
        pinMode(PIN_HH12_MISO, INPUT);
        pinMode(PIN_HH12_CS_AZ, OUTPUT);
        pinMode(PIN_HH12_CS_EL, OUTPUT);

        // État repos : CLK et CS à HIGH
        digitalWrite(PIN_HH12_SCLK, HIGH);
        digitalWrite(PIN_HH12_CS_AZ, HIGH);
        digitalWrite(PIN_HH12_CS_EL, HIGH);
    #endif

    #if !SIMULATE_ENCODERS
        // Première lecture position
        delay(10);
        rawCountsAz = readSSI(PIN_HH12_CS_AZ, REVERSE_AZ);
        rawCountsEl = readSSI(PIN_HH12_CS_EL, REVERSE_EL);

        // AZ : mapping direct 0-4095 → 0-360°
        currentAz = (float)rawCountsAz * 360.0f / 4096.0f;

        // EL : mapping direct 0-4095 → 0-360° (limité ensuite par EL_MIN/EL_MAX)
        currentEl = (float)rawCountsEl * 360.0f / 4096.0f;
    #else
        // Simulation : position initiale au centre
        currentAz = 180.0f;
        currentEl = 45.0f;
    #endif

    DEBUG_PRINTLN("=== ENCODEURS INITIALISÉS ===");
    #if SIMULATE_ENCODERS
        DEBUG_PRINTLN("Mode : SIMULATION");
    #else
        DEBUG_PRINTLN("AZ : HH-12 absolu (direct sur axe)");
        DEBUG_PRINTLN("EL : HH-12 absolu (direct sur axe)");
    #endif
    DEBUG_PRINT("Position initiale AZ=");
    DEBUG_PRINT(currentAz);
    DEBUG_PRINT(" EL=");
    DEBUG_PRINTLN(currentEl);
}

// ════════════════════════════════════════════════════════════════
// MISE À JOUR POSITION (appelé dans loop)
// ════════════════════════════════════════════════════════════════

void updateEncoders() {
    unsigned long now = millis();
    if (now - lastEncoderReadTime < ENCODER_READ_INTERVAL) {
        return;
    }
    lastEncoderReadTime = now;

    #if SIMULATE_ENCODERS
        // ─── Simulation de mouvement réaliste ───
        // Vitesse basée sur le temps réel écoulé (dt en secondes)
        float dt = (float)ENCODER_READ_INTERVAL / 1000.0f;
        float stepAz = SIM_AZ_SPEED * dt;  // °/s × s = ° par update
        float stepEl = SIM_EL_SPEED * dt;

        if (abs(currentAz - targetAz) > 0.01f) {
            float diff = targetAz - currentAz;
            if (abs(diff) <= stepAz) {
                currentAz = targetAz;
            } else {
                currentAz += (diff > 0) ? stepAz : -stepAz;
            }
            currentAz = constrain(currentAz, AZ_MIN, AZ_MAX);
        }
        if (abs(currentEl - targetEl) > 0.01f) {
            float diff = targetEl - currentEl;
            if (abs(diff) <= stepEl) {
                currentEl = targetEl;
            } else {
                currentEl += (diff > 0) ? stepEl : -stepEl;
            }
            currentEl = constrain(currentEl, EL_MIN, EL_MAX);
        }

    #else
        // ═══ AZIMUT : HH-12 absolu, mapping direct ═══
        rawCountsAz = readSSI(PIN_HH12_CS_AZ, REVERSE_AZ);
        currentAz = (float)rawCountsAz * 360.0f / 4096.0f;

        // ═══ ÉLÉVATION : HH-12 absolu, mapping direct ═══
        rawCountsEl = readSSI(PIN_HH12_CS_EL, REVERSE_EL);
        currentEl = (float)rawCountsEl * 360.0f / 4096.0f;
    #endif
}

// ════════════════════════════════════════════════════════════════
// LECTURE SSI (bit-bang, protocole HH-12)
// ════════════════════════════════════════════════════════════════

int readSSI(int csPin, bool reverse) {
    // Protocole SSI 12-bit (HH-12):
    // - CS LOW active la transmission
    // - 18 pulses CLK (12 bits data + 6 bits status/parity)
    // - Data valide sur front montant CLK (MSB first)

    unsigned long data = 0;

    // Activer transmission
    digitalWrite(csPin, LOW);
    delayMicroseconds(5);

    // 18 pulses CLK
    for (int i = 0; i < 18; i++) {
        digitalWrite(PIN_HH12_SCLK, LOW);
        delayMicroseconds(5);
        digitalWrite(PIN_HH12_SCLK, HIGH);
        delayMicroseconds(5);

        // Lire les 12 premiers bits (data utile)
        if (i < 12 && digitalRead(PIN_HH12_MISO)) {
            data |= (1UL << (11 - i));  // MSB first
        }
    }

    // Fin transmission
    digitalWrite(csPin, HIGH);

    int val = (int)data;
    return reverse ? (4095 - val) : val;
}
