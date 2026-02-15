// ════════════════════════════════════════════════════════════════
// EME ROTATOR CONTROLLER — Encodeurs HH-12 SSI (absolus)
// ════════════════════════════════════════════════════════════════
// AZ : HH-12 absolu sur axe final — mapping direct 0-4095 = 0-360°
// EL : HH-12 absolu sur axe final — mapping direct 0-4095 = 0-90°
// Pins SPI2 du ProS3 : SCLK=IO36, DATA=IO37, CS_AZ=IO35, CS_EL=IO34
// Via level shifter TXS0108E (3.3V ProS3 ↔ 5V HH-12)
// ════════════════════════════════════════════════════════════════

#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include "config.h"

// ════════════════════════════════════════════════════════════════
// VARIABLES GLOBALES POSITION
// ════════════════════════════════════════════════════════════════

// Position courante en degrés (mise à jour par updateEncoders)
extern float currentAz;
extern float currentEl;

// Valeurs brutes encodeurs (0-4095)
extern int rawCountsAz;
extern int rawCountsEl;

// ════════════════════════════════════════════════════════════════
// FONCTIONS PUBLIQUES
// ════════════════════════════════════════════════════════════════

// Initialise les pins SSI et effectue la première lecture
void setupEncoders();

// Met à jour currentAz/currentEl (appelé dans loop, throttled)
// En mode simulation, déplace la position vers targetAz/targetEl
void updateEncoders();

// Lecture SSI brute d'un encodeur HH-12
// @param csPin   Pin Chip Select
// @param reverse Inverser le sens (true/false)
// @return Valeur 0-4095
int readSSI(int csPin, bool reverse);

#endif // ENCODER_H
