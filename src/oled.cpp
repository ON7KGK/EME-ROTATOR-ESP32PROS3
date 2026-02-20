// ════════════════════════════════════════════════════════════════
// EME ROTATOR CONTROLLER — Module OLED SSD1306 128x32
// ════════════════════════════════════════════════════════════════
// Midas MDOB128032GV-WI — 0.91" blanc sur noir
// Bibliothèque U8g2 — mode full-buffer (512 octets RAM)
// Rafraîchissement : 500 ms depuis loop() Core 0
// ════════════════════════════════════════════════════════════════

#include "config.h"

#if ENABLE_OLED

#include "oled.h"
#include <U8g2lib.h>
#include <Wire.h>

// ── Constructeur U8g2 — SSD1306 128x32, I2C hardware ──
// Wire.begin() est déjà appelé dans main.cpp avant oledInit()
static U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ── Positions Y des 3 lignes (baseline, font 6x10, ascent=7) ──
#define LINE_Y1   9    // Position courante
#define LINE_Y2  20    // Position cible
#define LINE_Y3  31    // Statut

void oledInit() {
    // Forcer l'adresse I2C (8-bit = 7-bit << 1)
    u8g2.setI2CAddress(I2C_ADDR_OLED * 2);
    u8g2.setBusClock(I2C_SPEED);
    u8g2.begin();
    u8g2.setFont(u8g2_font_6x10_tf);

    // ── Splash screen 2 secondes ──
    u8g2.clearBuffer();
    u8g2.drawStr(4, LINE_Y1, "EME ROTATOR v7.4");
    u8g2.drawStr(4, LINE_Y2, "ON7KGK  JO20BM");
    u8g2.drawStr(4, LINE_Y3, "10 GHz EME");
    u8g2.sendBuffer();
    delay(2000);

    DEBUG_PRINTLN("[OLED] SSD1306 128x32 initialisé");
}

void oledUpdate(float curAz, float curEl,
                float tgtAz, float tgtEl,
                bool stale,
                const char *status) {
    char line[22];

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x10_tf);

    // Ligne 1 : Position courante
    snprintf(line, sizeof(line), "C:AZ%05.1f EL%04.1f", curAz, curEl);
    u8g2.drawStr(0, LINE_Y1, line);

    // Ligne 2 : Position cible — parenthèses si PSTRotator silencieux
    if (stale) {
        snprintf(line, sizeof(line), "(AZ%05.1f EL%04.1f)", tgtAz, tgtEl);
    } else {
        snprintf(line, sizeof(line), "T:AZ%05.1f EL%04.1f", tgtAz, tgtEl);
    }
    u8g2.drawStr(0, LINE_Y2, line);

    // Ligne 3 : Statut mouvement
    u8g2.drawStr(0, LINE_Y3, status);

    u8g2.sendBuffer();
}

#endif // ENABLE_OLED
