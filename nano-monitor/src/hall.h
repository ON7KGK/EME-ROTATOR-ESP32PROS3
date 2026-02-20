#ifndef HALL_H
#define HALL_H

// ════════════════════════════════════════════════════════════════
// EME ROTATOR — Encodeurs Hall quadrature
// ════════════════════════════════════════════════════════════════
// 2 encodeurs quadrature (12 PPR × 4 = 48 CPR)
// Comptage via interrupts sur 4 pins (A + B par encodeur)
// Compteurs int32_t signés (>44 millions de counts disponibles)
// ════════════════════════════════════════════════════════════════

#include <Arduino.h>

// Compteurs quadrature (signés, volatile pour accès ISR)
extern volatile int32_t hallCountAz;
extern volatile int32_t hallCountEl;

// Initialise les pins interrupt pour les 4 signaux Hall
void hallInit();

// Copie atomique des compteurs (désactive interrupts brièvement)
int32_t hallGetCountAz();
int32_t hallGetCountEl();

// Preset des compteurs (recalibration depuis HH-12)
void hallSetCountAz(int32_t value);
void hallSetCountEl(int32_t value);

// Reset des compteurs à zéro
void hallResetCounts();

#endif
