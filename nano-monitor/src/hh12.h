#ifndef HH12_H
#define HH12_H

// ════════════════════════════════════════════════════════════════
// EME ROTATOR — Encodeurs HH-12 SSI (absolus, 12-bit)
// ════════════════════════════════════════════════════════════════
// 2 encodeurs indépendants (pins séparés CS + SCLK + DATA)
// Protocole SSI bit-bang, 5V natif (pas de level shifter)
// ════════════════════════════════════════════════════════════════

#include <Arduino.h>

// Position brute (0-4095)
extern int hh12RawAz;
extern int hh12RawEl;

// Angle en degrés × 10 (ex: 1805 = 180.5°)
extern uint16_t hh12AngleAzX10;
extern uint16_t hh12AngleElX10;

// Flag de validité (lectures cohérentes)
extern bool hh12AzOk;
extern bool hh12ElOk;

// Initialise les pins SSI pour les deux encodeurs
void hh12Init();

// Lit les deux encodeurs (appelé périodiquement)
void hh12Update();

// Lecture SSI bas niveau (12-bit, MSB first)
int readSSI(int csPin, int sclkPin, int dataPin, bool reverse);

#endif
