// ════════════════════════════════════════════════════════════════
// EME ROTATOR CONTROLLER — Module Encodeurs (Implementation)
// ════════════════════════════════════════════════════════════════
// Phase 1 : tous les encodeurs désactivés (ENABLE_SIM_POSITION = 1)
// Phase 4a : AS5048A AZ (SPI bit-bang, IO35/IO36/IO37)
// Phase 4b : Hall AZ PCNT (IO3/IO4)
// Legacy   : HH-12 AZ SSI (mêmes pins que AS5048A)
// ════════════════════════════════════════════════════════════════

#include "encoder.h"

// ════════════════════════════════════════════════════════════════
// AS5048A — SPI bit-bang Mode 1 (CPOL=0, CPHA=1), 500 kHz
// ════════════════════════════════════════════════════════════════

#if ENABLE_AZ_AS5048A

void as5048aInit() {
    pinMode(PIN_ENC_AZ_CS, OUTPUT);
    pinMode(PIN_ENC_AZ_CLK, OUTPUT);
    pinMode(PIN_ENC_AZ_MISO, INPUT);
    digitalWrite(PIN_ENC_AZ_CS, HIGH);
    digitalWrite(PIN_ENC_AZ_CLK, LOW);   // CPOL=0, repos LOW
}

int16_t as5048aReadRaw() {
    // TODO Phase 4a : implémenter lecture SPI pipelined
    // 1. CS LOW → send 0x3FFF (read angle) → CS HIGH
    // 2. CS LOW → read 16-bit response → CS HIGH
    // 3. Check parity (bit 15) and error flag (bit 14)
    // 4. Return bits[13:0] = angle 0-16383
    return -1;
}

float as5048aReadAngle() {
    int16_t raw = as5048aReadRaw();
    if (raw < 0) return -1.0f;
    return (float)raw * 360.0f / 16384.0f;
}

#endif // ENABLE_AZ_AS5048A

// ════════════════════════════════════════════════════════════════
// HH-12 — SSI bit-bang 12-bit (legacy/fallback)
// ════════════════════════════════════════════════════════════════

#if ENABLE_AZ_HH12

void hh12Init() {
    pinMode(PIN_ENC_AZ_CS, OUTPUT);
    pinMode(PIN_ENC_AZ_CLK, OUTPUT);
    pinMode(PIN_ENC_AZ_MISO, INPUT);
    digitalWrite(PIN_ENC_AZ_CS, HIGH);   // CS repos HIGH
    digitalWrite(PIN_ENC_AZ_CLK, LOW);   // CLK repos LOW
    DEBUG_PRINT("[HH12] Init : CS=IO");
    DEBUG_PRINT(PIN_ENC_AZ_CS);
    DEBUG_PRINT(" CLK=IO");
    DEBUG_PRINT(PIN_ENC_AZ_CLK);
    DEBUG_PRINT(" DATA=IO");
    DEBUG_PRINTLN(PIN_ENC_AZ_MISO);
}

int hh12ReadRaw(int csPin, bool reverse) {
    // Protocole SSI 12-bit (HH-12) :
    // CS LOW → 18 pulses CLK → 12 bits data (MSB first) → CS HIGH
    unsigned long data = 0;

    digitalWrite(csPin, LOW);
    delayMicroseconds(20);

    for (int i = 0; i < 18; i++) {
        digitalWrite(PIN_ENC_AZ_CLK, LOW);
        delayMicroseconds(10);
        digitalWrite(PIN_ENC_AZ_CLK, HIGH);
        delayMicroseconds(10);

        if (i < 12 && digitalRead(PIN_ENC_AZ_MISO)) {
            data |= (1UL << (11 - i));
        }
    }

    digitalWrite(csPin, HIGH);
    delayMicroseconds(20);

    int val = (int)data;
    return reverse ? (4095 - val) : val;
}

float hh12ReadAngle(int csPin, bool reverse) {
    int raw = hh12ReadRaw(csPin, reverse);
    return (float)raw * 360.0f / 4096.0f;
}

#endif // ENABLE_AZ_HH12

// ════════════════════════════════════════════════════════════════
// PCNT Hall AZ — Hardware quadrature counter
// ════════════════════════════════════════════════════════════════

#if ENABLE_AZ_HALL_PCNT

// TODO Phase 4b : implémenter avec driver/pulse_cnt.h
// pcnt_unit_config_t, pcnt_channel_config_t, etc.

void pcntAzInit() {
    // TODO : configurer PCNT unit 0, pins IO3/IO4, quadrature 4x
}

int32_t pcntAzGetCount() {
    // TODO : pcnt_unit_get_count()
    return 0;
}

void pcntAzSetCount(int32_t value) {
    // TODO : pcnt_unit_clear_count() + offset
    (void)value;
}

#endif // ENABLE_AZ_HALL_PCNT
