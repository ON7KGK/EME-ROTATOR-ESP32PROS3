#ifndef SAFETY_H
#define SAFETY_H

// ════════════════════════════════════════════════════════════════
// EME ROTATOR — Sécurités locales (UNO R4 Motor Box)
// ════════════════════════════════════════════════════════════════
// Sécurités appliquées AVANT chaque commande moteur :
//   1. Watchdog Modbus (500ms sans commande → stop)
//   2. STOP button (NF, hardware EN + software read via MCP23017)
//   3. MC33926 SF fault (overcurrent/overtemp hardware)
//   4. Current FB overcurrent (software threshold)
//   5. Limit switches directionnels (bloque la direction, autorise l'inverse)
// ════════════════════════════════════════════════════════════════

#include <Arduino.h>

// ═══════ État des sécurités ═══════

// Flags limites individuels
extern bool limitCW;
extern bool limitCCW;
extern bool limitUP;
extern bool limitDOWN;

// Bouton STOP pressé
extern bool stopActive;

// Fault flags
extern bool sfFault;
extern bool fbOvercurrent;

// Status bitmap (combinaison de tous les flags pour Modbus)
extern uint16_t safetyStatus;

// ═══════ Fonctions ═══════

// Initialise le MCP23017 (limits + STOP + direction outputs)
// Doit être appelé APRÈS Wire.begin()
void safetyInit();

// Met à jour les flags de sécurité (lire MCP23017, SF, FB)
void safetyUpdate();

// Applique les sécurités sur les duty cycles demandés
// Modifie dutyAz/dutyEl en place (met à 0 si sécurité active)
void safetyApply(int16_t &dutyAz, int16_t &dutyEl);

// Reset le watchdog Modbus (appelé à chaque commande Modbus reçue)
void safetyResetWatchdog();

// Vérifie si le watchdog a expiré
bool safetyWatchdogExpired();

// Clear les flags de fault (après résolution)
void safetyClearFaults();

#endif
