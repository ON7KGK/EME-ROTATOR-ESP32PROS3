// ════════════════════════════════════════════════════════════════
// EME ROTATOR CONTROLLER — Module MCP23017 I2C Expander
// ════════════════════════════════════════════════════════════════
// Carte CQROBOT, adresse 0x27 (A0=A1=A2=VCC)
// Port A : 4 fins de course + 4 boutons manuels (entrées)
// Port B : 4 directions moteur + 2× SF + 2 LEDs (sorties + 2 entrées)
// Interrupt : INTA=INTB miroir → IO16 (PIN_MCP_INT)
// ════════════════════════════════════════════════════════════════

#ifndef MCP23017_H
#define MCP23017_H

#include <Arduino.h>

// ── Port A : entrées (active LOW) ──
#define MCP_LIMIT_CW        0  // PA0 — Fin de course CW  (via opto VMA452)
#define MCP_LIMIT_CCW       1  // PA1 — Fin de course CCW (via opto VMA452)
#define MCP_LIMIT_UP        2  // PA2 — Fin de course UP  (via opto VMA452)
#define MCP_LIMIT_DOWN      3  // PA3 — Fin de course DOWN(via opto VMA452)
#define MCP_BTN_CW          4  // PA4 — Bouton CW   (pull-up interne)
#define MCP_BTN_CCW         5  // PA5 — Bouton CCW  (pull-up interne)
#define MCP_BTN_UP          6  // PA6 — Bouton UP   (pull-up interne)
#define MCP_BTN_DOWN        7  // PA7 — Bouton DOWN (pull-up interne)

// ── Port B : sorties + 2× SF entrées ──
#define MCP_MOT_AZ_IN1      0  // PB0 — MC33926 M1_IN1 (direction AZ)
#define MCP_MOT_AZ_IN2      1  // PB1 — MC33926 M1_IN2 (direction AZ)
#define MCP_MOT_EL_IN1      2  // PB2 — MC33926 M2_IN1 (direction EL)
#define MCP_MOT_EL_IN2      3  // PB3 — MC33926 M2_IN2 (direction EL)
#define MCP_MC_SF_AZ         4  // PB4 — MC33926 M1_SF (open-drain, LOW=fault AZ)
#define MCP_MC_SF_EL         5  // PB5 — MC33926 M2_SF (open-drain, LOW=fault EL)
#define MCP_LED_FAULT        6  // PB6 — LED rouge (fault)
#define MCP_LED_MOVING       7  // PB7 — LED jaune (moving)

// ── Masques pratiques ──
#define MCP_MASK_LIMITS     0x0F  // PA0-PA3
#define MCP_MASK_BUTTONS    0xF0  // PA4-PA7
#define MCP_MASK_DIRS       0x0F  // PB0-PB3 (directions moteur)
#define MCP_MASK_SF         0x30  // PB4-PB5 (status flags)
#define MCP_MASK_LEDS       0xC0  // PB6-PB7 (LEDs)

// ── Directions moteur (nibble bas de port B) ──
#define MCP_DIR_AZ_CW       0x01  // IN1=H, IN2=L
#define MCP_DIR_AZ_CCW      0x02  // IN1=L, IN2=H
#define MCP_DIR_AZ_BRAKE    0x00
#define MCP_DIR_EL_UP       0x04  // IN1=H, IN2=L
#define MCP_DIR_EL_DOWN     0x08  // IN1=L, IN2=H
#define MCP_DIR_EL_BRAKE    0x00

// Initialise le MCP23017 (registres, directions, pull-ups, interrupts)
// Retourne true si le chip répond sur I2C
bool mcpInit();

// Lit le port A complet (limites + boutons), retourne 8 bits
// Bit à 0 = actif (active LOW)
uint8_t mcpReadPortA();

// Lit le port B complet, retourne 8 bits
uint8_t mcpReadPortB();

// Écrit les directions moteur (nibble bas PB0-PB3)
// Préserve les bits hauts (LEDs + SF)
void mcpSetMotorDir(uint8_t dirBits);

// Allume/éteint les LEDs (PB6-PB7)
void mcpSetLed(uint8_t ledBit, bool on);

// Lit les flags SF du MC33926, true = fault actif (LOW)
bool mcpReadFaultAz();  // M1_SF sur PB4
bool mcpReadFaultEl();  // M2_SF sur PB5

// Lit le registre INTCAP A (capture au moment de l'interrupt)
uint8_t mcpReadIntCapA();

#endif // MCP23017_H
