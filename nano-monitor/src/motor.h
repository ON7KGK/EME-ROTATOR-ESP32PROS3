#ifndef MOTOR_H
#define MOTOR_H

// ════════════════════════════════════════════════════════════════
// EME ROTATOR — Contrôle moteur MC33926
// ════════════════════════════════════════════════════════════════
// PWM via pins directes (D3, D5)
// Direction via MCP23017 I2C (GPB0-3 = IN1/IN2)
// Current feedback via ADC (A0, A1)
// Status flag via pin directe (A2)
// ════════════════════════════════════════════════════════════════

#include <Arduino.h>

// Courant mesuré en ampères
extern float motorCurrentAz;
extern float motorCurrentEl;

// ADC brut pour Modbus
extern uint16_t motorFbRawAz;
extern uint16_t motorFbRawEl;

// Status flag MC33926
extern bool motorSfFault;

// Initialise les pins moteur et le MCP23017
// Doit être appelé APRÈS Wire.begin()
void motorInit();

// Applique un duty cycle (-255 à +255, signe = direction)
// Respecte les sécurités locales (limits, STOP, SF)
void motorSetDuty(int16_t dutyAz, int16_t dutyEl);

// Arrêt immédiat des deux moteurs
void motorStop();

// Lit le courant et le status flag
void motorUpdateFeedback();

// Lit le SF directement (pour vérification rapide)
bool motorReadSF();

#endif
