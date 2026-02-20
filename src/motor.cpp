// ════════════════════════════════════════════════════════════════
// EME ROTATOR CONTROLLER — Module MC33926 Motor Driver (Impl.)
// ════════════════════════════════════════════════════════════════
// LEDC PWM — remplace MCPWM legacy qui ne fonctionne pas sur
// ESP32-S3 (mcpwm_set_signal_low bloque la sortie)
// ════════════════════════════════════════════════════════════════

#include "config.h"

#if ENABLE_MOTORS

#include "motor.h"

// Canaux LEDC (0–15 disponibles sur ESP32-S3)
#define LEDC_CH_AZ  0
#define LEDC_CH_EL  1
#define LEDC_BITS   8              // 8-bit résolution (0-255)
#define LEDC_MAX    ((1 << LEDC_BITS) - 1)  // 255

// ════════════════════════════════════════════════════════════════
// INITIALISATION
// ════════════════════════════════════════════════════════════════

void motorInit() {
    // Configurer canaux LEDC : fréquence + résolution
    ledcSetup(LEDC_CH_AZ, MOT_PWM_FREQ, LEDC_BITS);
    ledcSetup(LEDC_CH_EL, MOT_PWM_FREQ, LEDC_BITS);

    // Attacher GPIO aux canaux
    ledcAttachPin(PIN_MOT_AZ_PWM, LEDC_CH_AZ);
    ledcAttachPin(PIN_MOT_EL_PWM, LEDC_CH_EL);

    // Démarrer à 0% (D2 = LOW → MC33926 désactivé)
    ledcWrite(LEDC_CH_AZ, 0);
    ledcWrite(LEDC_CH_EL, 0);

    // ADC pour lecture courant
    pinMode(PIN_MOT_AZ_FB, INPUT);
    pinMode(PIN_MOT_EL_FB, INPUT);

    DEBUG_PRINT("[MOT] LEDC init: AZ=IO");
    DEBUG_PRINT(PIN_MOT_AZ_PWM);
    DEBUG_PRINT(" (ch");
    DEBUG_PRINT(LEDC_CH_AZ);
    DEBUG_PRINT(") EL=IO");
    DEBUG_PRINT(PIN_MOT_EL_PWM);
    DEBUG_PRINT(" (ch");
    DEBUG_PRINT(LEDC_CH_EL);
    DEBUG_PRINT(") @ ");
    DEBUG_PRINT(MOT_PWM_FREQ / 1000);
    DEBUG_PRINTLN(" kHz, 8-bit");
}

// ════════════════════════════════════════════════════════════════
// CONTRÔLE PWM
// ════════════════════════════════════════════════════════════════

void motorSetDutyAz(float duty) {
    if (duty <= 0.0f) {
        ledcWrite(LEDC_CH_AZ, 0);
    } else {
        if (duty > 100.0f) duty = 100.0f;
        uint32_t raw = (uint32_t)(duty * LEDC_MAX / 100.0f);
        ledcWrite(LEDC_CH_AZ, raw);
    }
}

void motorSetDutyEl(float duty) {
    if (duty <= 0.0f) {
        ledcWrite(LEDC_CH_EL, 0);
    } else {
        if (duty > 100.0f) duty = 100.0f;
        uint32_t raw = (uint32_t)(duty * LEDC_MAX / 100.0f);
        ledcWrite(LEDC_CH_EL, raw);
    }
}

void motorBrakeAll() {
    ledcWrite(LEDC_CH_AZ, 0);
    ledcWrite(LEDC_CH_EL, 0);
}

// ════════════════════════════════════════════════════════════════
// LECTURE COURANT (ADC → mV, 0.525 V/A)
// ════════════════════════════════════════════════════════════════

uint16_t motorReadCurrentAzMv() {
    return (uint16_t)((analogRead(PIN_MOT_AZ_FB) * 3300UL) / 4095);
}

uint16_t motorReadCurrentElMv() {
    return (uint16_t)((analogRead(PIN_MOT_EL_FB) * 3300UL) / 4095);
}

#endif // ENABLE_MOTORS
