// ════════════════════════════════════════════════════════════════
// EME ROTATOR CONTROLLER — Module EEPROM/FRAM
// ════════════════════════════════════════════════════════════════
// Double buffering avec CRC16-CCITT pour survie aux coupures
// Block A : 0x0000  |  Block B : 0x0020
// Sélection au boot : bloc avec le seq_counter le plus élevé + CRC valide
// ════════════════════════════════════════════════════════════════

#ifndef EEPROM_STORAGE_H
#define EEPROM_STORAGE_H

#include <Arduino.h>
#include "config.h"

#if ENABLE_EEPROM

// ════════════════════════════════════════════════════════════════
// Structure de données — 30 octets par bloc
// ════════════════════════════════════════════════════════════════

#define EEPROM_BLOCK_A      0x0000
#define EEPROM_BLOCK_B      0x0020
#define EEPROM_BLOCK_SIZE   30      // sizeof(EepromPositionBlock)

typedef struct __attribute__((packed)) {
    float    pos_az_deg;     // Position azimut en degrés (4 octets)
    float    pos_el_deg;     // Position élévation en degrés (4 octets)
    uint16_t enc_az_raw;     // AS5048A AZ raw au moment du stockage (2 octets)
    float    hwt901b_el;     // HWT901B élévation au moment du stockage (4 octets)
    float    target_az_deg;  // Cible azimut (4 octets)
    float    target_el_deg;  // Cible élévation (4 octets)
    uint32_t seq_counter;    // Compteur séquentiel, incrémenté à chaque écriture (4 octets)
    uint16_t crc16;          // CRC16-CCITT sur les 28 octets précédents (2 octets)
} EepromPositionBlock;       // Total : 30 octets

// ════════════════════════════════════════════════════════════════
// API publique
// ════════════════════════════════════════════════════════════════

// Initialise le bus I2C (si pas déjà fait) et vérifie la présence de l'EEPROM
// Retourne true si l'EEPROM répond à I2C_ADDR_EEPROM
bool eepromInit();

// Lit les deux blocs, sélectionne le plus récent avec CRC valide
// Remplit `block` avec le bloc choisi. Retourne true si au moins un bloc valide.
bool eepromReadBest(EepromPositionBlock &block);

// Écrit un bloc dans le slot le plus ancien (alternance A/B)
// Incrémente automatiquement seq_counter et calcule le CRC16
bool eepromWritePosition(float az, float el, uint16_t encAzRaw, float hwt901bEl,
                         float targetAz, float targetEl);

// Diagnostic : lit un bloc brut à l'adresse donnée (EEPROM_BLOCK_A ou _B)
bool eepromReadBlock(uint16_t addr, EepromPositionBlock &block);

// Test complet : écriture → relecture → vérification CRC
// Affiche les résultats sur Serial. Retourne true si tout OK.
bool eepromSelfTest();

// CRC16-CCITT (polynôme 0x1021, init 0xFFFF)
uint16_t crc16_ccitt(const uint8_t *data, size_t len);

#endif // ENABLE_EEPROM
#endif // EEPROM_STORAGE_H
