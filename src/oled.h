// ════════════════════════════════════════════════════════════════
// EME ROTATOR CONTROLLER — Module OLED SSD1306 128x32
// ════════════════════════════════════════════════════════════════
// Midas MDOB128032GV-WI — 0.91" — I2C 0x3C
// 3 lignes avec font 6x10 :
//   Ligne 1 : Position courante  C:AZxxx.xx EL xx.xx
//   Ligne 2 : Position cible     T:AZxxx.xx EL xx.xx  (ou parenthèses si stale)
//   Ligne 3 : Statut mouvement   TRACK CW UP / IDLE / STOP
// ════════════════════════════════════════════════════════════════

#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include <Arduino.h>
#include "config.h"

#if ENABLE_OLED

// Initialise le display, affiche splash screen 2s
void oledInit();

// Rafraîchit l'affichage (appeler toutes les 500ms depuis loop Core 0)
//   curAz/curEl : position courante
//   tgtAz/tgtEl : position cible
//   stale       : true si PSTRotator n'a rien envoyé depuis STALE_TARGET_MS
//                 → cible entre parenthèses au lieu de "T:"
//   status      : chaîne statut ("TRACK CW UP", "IDLE", "** STOP **", etc.)
void oledUpdate(float curAz, float curEl,
                float tgtAz, float tgtEl,
                bool stale,
                const char *status);

#endif // ENABLE_OLED
#endif // OLED_DISPLAY_H
