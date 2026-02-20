// ════════════════════════════════════════════════════════════════
// EME ROTATOR CONTROLLER — Module MC33926 Motor Driver (Impl.)
// ════════════════════════════════════════════════════════════════
// API MCPWM legacy (ESP-IDF 4.4, framework Arduino ESP32 2.0.x)
// ════════════════════════════════════════════════════════════════

#include "config.h"

#if ENABLE_MOTORS

#include "motor.h"
#include "driver/mcpwm.h"

// ════════════════════════════════════════════════════════════════
// INITIALISATION
// ════════════════════════════════════════════════════════════════

void motorInit() {
    // Assigner les GPIO aux sorties MCPWM
    // AZ = MCPWM unit 0, operator 0, sortie A
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_MOT_AZ_PWM);
    // EL = MCPWM unit 0, operator 1, sortie A
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PIN_MOT_EL_PWM);

    // Configuration commune : 20 kHz, duty 0%
    mcpwm_config_t cfg;
    cfg.frequency = MOT_PWM_FREQ;
    cfg.cmpr_a = 0.0f;
    cfg.cmpr_b = 0.0f;
    cfg.duty_mode = MCPWM_DUTY_MODE_0;
    cfg.counter_mode = MCPWM_UP_COUNTER;

    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &cfg);  // AZ
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &cfg);  // EL

    // Démarrer avec sortie LOW (pas de PWM = frein)
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);

    // Configurer ADC pour lecture courant (analogRead suffit)
    pinMode(PIN_MOT_AZ_FB, INPUT);
    pinMode(PIN_MOT_EL_FB, INPUT);

    DEBUG_PRINT("[MOT] MCPWM init: AZ=IO");
    DEBUG_PRINT(PIN_MOT_AZ_PWM);
    DEBUG_PRINT(" EL=IO");
    DEBUG_PRINT(PIN_MOT_EL_PWM);
    DEBUG_PRINT(" @ ");
    DEBUG_PRINT(MOT_PWM_FREQ / 1000);
    DEBUG_PRINTLN(" kHz");
}

// ════════════════════════════════════════════════════════════════
// CONTRÔLE PWM
// ════════════════════════════════════════════════════════════════

void motorSetDutyAz(float duty) {
    if (duty <= 0.0f) {
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    } else {
        if (duty > 100.0f) duty = 100.0f;
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,
                            MCPWM_DUTY_MODE_0);
    }
}

void motorSetDutyEl(float duty) {
    if (duty <= 0.0f) {
        mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
    } else {
        if (duty > 100.0f) duty = 100.0f;
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty);
        mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A,
                            MCPWM_DUTY_MODE_0);
    }
}

void motorBrakeAll() {
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
}

// ════════════════════════════════════════════════════════════════
// LECTURE COURANT (ADC → mV, 0.525 V/A)
// ════════════════════════════════════════════════════════════════

uint16_t motorReadCurrentAzMv() {
    // ADC ESP32-S3 : 12 bits, 0-3.3V par défaut
    return (uint16_t)((analogRead(PIN_MOT_AZ_FB) * 3300UL) / 4095);
}

uint16_t motorReadCurrentElMv() {
    return (uint16_t)((analogRead(PIN_MOT_EL_FB) * 3300UL) / 4095);
}

#endif // ENABLE_MOTORS
