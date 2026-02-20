// ════════════════════════════════════════════════════════════════
// EME ROTATOR CONTROLLER — Module Encodeurs
// ════════════════════════════════════════════════════════════════
// AZ : AS5048A SPI 14-bit (ENABLE_AZ_AS5048A) ou HH-12 SSI 12-bit (ENABLE_AZ_HH12)
// EL : HWT901B RS-485 inclinomètre (via module Modbus, pas ici)
// Hall AZ : PCNT hardware (ENABLE_AZ_HALL_PCNT)
// ════════════════════════════════════════════════════════════════

#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include "config.h"

// ════════════════════════════════════════════════════════════════
// AS5048A — Encodeur magnétique AZ 14-bit (Phase 4a)
// ════════════════════════════════════════════════════════════════
#if ENABLE_AZ_AS5048A

// Initialise les pins SPI bit-bang AS5048A
void as5048aInit();

// Lit l'angle brut 14-bit (0-16383). Retourne -1 en cas d'erreur.
int16_t as5048aReadRaw();

// Lit l'angle en degrés (0.0 - 359.978)
float as5048aReadAngle();

#endif

// ════════════════════════════════════════════════════════════════
// HH-12 — Encodeur capacitif AZ 12-bit SSI (fallback/legacy)
// ════════════════════════════════════════════════════════════════
#if ENABLE_AZ_HH12

// Initialise les pins SSI pour le HH-12
void hh12Init();

// Lecture SSI brute HH-12 (0-4095)
int hh12ReadRaw(int csPin, bool reverse);

// Lit l'angle en degrés (0.0 - 359.912)
float hh12ReadAngle(int csPin, bool reverse);

#endif

// ════════════════════════════════════════════════════════════════
// PCNT Hall AZ — Compteur hardware quadrature (Phase 4b)
// ════════════════════════════════════════════════════════════════
#if ENABLE_AZ_HALL_PCNT

// Initialise le PCNT hardware (unit 0, pins IO3/IO4)
void pcntAzInit();

// Lit le compteur PCNT AZ (int32, positif = CW)
int32_t pcntAzGetCount();

// Remet le compteur à une valeur donnée (recalibration AS5048A)
void pcntAzSetCount(int32_t value);

#endif

#endif // ENCODER_H
