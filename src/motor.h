// ════════════════════════════════════════════════════════════════
// EME ROTATOR CONTROLLER — Module MC33926 Motor Driver
// ════════════════════════════════════════════════════════════════
// 2× MC33926 pont H via MCPWM ESP32-S3
// AZ : MCPWM unit 0, timer 0 → IO21 (D2 PWM)
// EL : MCPWM unit 0, timer 1 → IO38 (D2 PWM)
// Directions IN1/IN2 via MCP23017 (PB0-PB3)
// Courant FB via ADC : IO1 (AZ), IO2 (EL)
// ════════════════════════════════════════════════════════════════

#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

// Initialise MCPWM (2 timers, 20 kHz) + ADC courant
void motorInit();

// Applique un duty cycle (0.0 - 100.0%) sur moteur AZ
void motorSetDutyAz(float duty);

// Applique un duty cycle (0.0 - 100.0%) sur moteur EL
void motorSetDutyEl(float duty);

// Frein (PWM = 0) sur les deux moteurs
void motorBrakeAll();

// Lit le courant moteur en mV (FB pin, 0.525 V/A)
uint16_t motorReadCurrentAzMv();
uint16_t motorReadCurrentElMv();

#endif // MOTOR_H
