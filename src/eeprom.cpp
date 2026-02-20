// ════════════════════════════════════════════════════════════════
// EME ROTATOR CONTROLLER — Module EEPROM/FRAM (Implementation)
// ════════════════════════════════════════════════════════════════
// AT24C256 (1M cycles) ou FM24C64 FRAM (10^14 cycles) à 0x50
// Double buffering : Block A (0x0000) + Block B (0x0020)
// CRC16-CCITT sur 20 octets → détection corruption
// ════════════════════════════════════════════════════════════════

#include "config.h"

#if ENABLE_EEPROM

#include "eeprom.h"
#include <Wire.h>

// ════════════════════════════════════════════════════════════════
// CRC16-CCITT (polynôme 0x1021, init 0xFFFF)
// ════════════════════════════════════════════════════════════════

uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// ════════════════════════════════════════════════════════════════
// I2C bas niveau — lecture/écriture EEPROM à adresse 16-bit
// ════════════════════════════════════════════════════════════════

// Écriture d'un buffer à une adresse 16-bit dans l'EEPROM
// L'AT24C256 supporte les pages de 64 octets, notre bloc fait 22 → une seule page
static bool eepromI2CWrite(uint16_t memAddr, const uint8_t *data, size_t len) {
    Wire.beginTransmission(I2C_ADDR_EEPROM);
    Wire.write((uint8_t)(memAddr >> 8));    // Adresse haute
    Wire.write((uint8_t)(memAddr & 0xFF));  // Adresse basse
    for (size_t i = 0; i < len; i++) {
        Wire.write(data[i]);
    }
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        DEBUG_PRINT("[EEPROM] Erreur écriture I2C: ");
        DEBUG_PRINTLN(err);
        return false;
    }
    // Attendre la fin du cycle d'écriture (5ms typique AT24C256, instantané FRAM)
    delay(6);
    return true;
}

// Lecture d'un buffer depuis une adresse 16-bit dans l'EEPROM
static bool eepromI2CRead(uint16_t memAddr, uint8_t *data, size_t len) {
    // Positionner le pointeur d'adresse
    Wire.beginTransmission(I2C_ADDR_EEPROM);
    Wire.write((uint8_t)(memAddr >> 8));
    Wire.write((uint8_t)(memAddr & 0xFF));
    uint8_t err = Wire.endTransmission(false);  // repeated start
    if (err != 0) {
        DEBUG_PRINT("[EEPROM] Erreur adressage I2C: ");
        DEBUG_PRINTLN(err);
        return false;
    }

    // Lire les données
    size_t received = Wire.requestFrom((uint8_t)I2C_ADDR_EEPROM, len);
    if (received != len) {
        DEBUG_PRINT("[EEPROM] Lecture partielle: ");
        DEBUG_PRINT(received);
        DEBUG_PRINT("/");
        DEBUG_PRINTLN(len);
        return false;
    }
    for (size_t i = 0; i < len; i++) {
        data[i] = Wire.read();
    }
    return true;
}

// ════════════════════════════════════════════════════════════════
// API publique
// ════════════════════════════════════════════════════════════════

bool eepromInit() {
    // Vérifier que l'EEPROM répond sur le bus I2C
    Wire.beginTransmission(I2C_ADDR_EEPROM);
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        DEBUG_PRINTLN("[EEPROM] ERREUR: EEPROM non trouvée à 0x50 !");
        return false;
    }
    DEBUG_PRINTLN("[EEPROM] EEPROM détectée à 0x50");
    return true;
}

bool eepromReadBlock(uint16_t addr, EepromPositionBlock &block) {
    return eepromI2CRead(addr, (uint8_t *)&block, sizeof(EepromPositionBlock));
}

// Valider le CRC d'un bloc (CRC sur les 20 premiers octets)
static bool blockCrcValid(const EepromPositionBlock &block) {
    uint16_t computed = crc16_ccitt((const uint8_t *)&block,
                                     sizeof(EepromPositionBlock) - sizeof(uint16_t));
    return (computed == block.crc16);
}

bool eepromReadBest(EepromPositionBlock &block) {
    EepromPositionBlock blockA, blockB;
    bool readA = eepromReadBlock(EEPROM_BLOCK_A, blockA);
    bool readB = eepromReadBlock(EEPROM_BLOCK_B, blockB);

    bool validA = readA && blockCrcValid(blockA);
    bool validB = readB && blockCrcValid(blockB);

    DEBUG_PRINT("[EEPROM] Block A: ");
    if (validA) {
        DEBUG_PRINT("CRC OK, seq=");
        DEBUG_PRINTLN(blockA.seq_counter);
    } else {
        DEBUG_PRINTLN(readA ? "CRC INVALIDE" : "lecture échouée");
    }

    DEBUG_PRINT("[EEPROM] Block B: ");
    if (validB) {
        DEBUG_PRINT("CRC OK, seq=");
        DEBUG_PRINTLN(blockB.seq_counter);
    } else {
        DEBUG_PRINTLN(readB ? "CRC INVALIDE" : "lecture échouée");
    }

    // Sélectionner le bloc avec le seq_counter le plus élevé et CRC valide
    if (validA && validB) {
        block = (blockA.seq_counter >= blockB.seq_counter) ? blockA : blockB;
        DEBUG_PRINT("[EEPROM] Sélection: bloc ");
        DEBUG_PRINT((blockA.seq_counter >= blockB.seq_counter) ? "A" : "B");
        DEBUG_PRINT(" (seq=");
        DEBUG_PRINT(block.seq_counter);
        DEBUG_PRINTLN(")");
        return true;
    }
    if (validA) {
        block = blockA;
        DEBUG_PRINTLN("[EEPROM] Sélection: bloc A (seul valide)");
        return true;
    }
    if (validB) {
        block = blockB;
        DEBUG_PRINTLN("[EEPROM] Sélection: bloc B (seul valide)");
        return true;
    }

    DEBUG_PRINTLN("[EEPROM] AUCUN bloc valide — EEPROM vierge ou corrompue");
    return false;
}

bool eepromWritePosition(float az, float el, uint16_t encAzRaw, float hwt901bEl,
                         float targetAz, float targetEl) {
    // Lire les deux blocs pour déterminer le prochain seq_counter et le slot cible
    EepromPositionBlock blockA, blockB;
    bool readA = eepromReadBlock(EEPROM_BLOCK_A, blockA);
    bool readB = eepromReadBlock(EEPROM_BLOCK_B, blockB);

    bool validA = readA && blockCrcValid(blockA);
    bool validB = readB && blockCrcValid(blockB);

    // Prochain compteur = max des deux + 1
    uint32_t nextSeq = 1;
    if (validA && validB) {
        nextSeq = ((blockA.seq_counter > blockB.seq_counter)
                   ? blockA.seq_counter : blockB.seq_counter) + 1;
    } else if (validA) {
        nextSeq = blockA.seq_counter + 1;
    } else if (validB) {
        nextSeq = blockB.seq_counter + 1;
    }

    // Écrire dans le bloc le plus ancien (alternance)
    uint16_t targetAddr;
    const char *targetLabel;
    if (!validA && !validB) {
        // Aucun valide : commencer par A
        targetAddr = EEPROM_BLOCK_A;
        targetLabel = "A";
    } else if (!validA) {
        targetAddr = EEPROM_BLOCK_A;
        targetLabel = "A";
    } else if (!validB) {
        targetAddr = EEPROM_BLOCK_B;
        targetLabel = "B";
    } else {
        // Les deux valides : écrire dans le plus ancien
        targetAddr = (blockA.seq_counter <= blockB.seq_counter)
                     ? EEPROM_BLOCK_A : EEPROM_BLOCK_B;
        targetLabel = (blockA.seq_counter <= blockB.seq_counter) ? "A" : "B";
    }

    // Préparer le nouveau bloc
    EepromPositionBlock newBlock;
    newBlock.pos_az_deg    = az;
    newBlock.pos_el_deg    = el;
    newBlock.enc_az_raw    = encAzRaw;
    newBlock.hwt901b_el    = hwt901bEl;
    newBlock.target_az_deg = targetAz;
    newBlock.target_el_deg = targetEl;
    newBlock.seq_counter   = nextSeq;
    newBlock.crc16 = crc16_ccitt((const uint8_t *)&newBlock,
                                  sizeof(EepromPositionBlock) - sizeof(uint16_t));

    // Écrire
    bool ok = eepromI2CWrite(targetAddr, (const uint8_t *)&newBlock,
                             sizeof(EepromPositionBlock));

    if (ok) {
        DEBUG_PRINT("[EEPROM] Écrit bloc ");
        DEBUG_PRINT(targetLabel);
        DEBUG_PRINT(" seq=");
        DEBUG_PRINT(nextSeq);
        DEBUG_PRINT(" AZ=");
        DEBUG_PRINT(String(az, 1));
        DEBUG_PRINT(" EL=");
        DEBUG_PRINTLN(String(el, 1));
    }

    return ok;
}

// ════════════════════════════════════════════════════════════════
// SELF-TEST — test complet écriture/lecture/CRC
// ════════════════════════════════════════════════════════════════

bool eepromSelfTest() {
    DEBUG_PRINTLN("═══════════════════════════════════════");
    DEBUG_PRINTLN("  EEPROM SELF-TEST");
    DEBUG_PRINTLN("═══════════════════════════════════════");

    // 1. Lire l'état initial des deux blocs
    DEBUG_PRINTLN("[TEST] Lecture état initial...");
    EepromPositionBlock best;
    bool hasBest = eepromReadBest(best);

    if (hasBest) {
        DEBUG_PRINT("[TEST] Position stockée: AZ=");
        DEBUG_PRINT(String(best.pos_az_deg, 1));
        DEBUG_PRINT(" EL=");
        DEBUG_PRINT(String(best.pos_el_deg, 1));
        DEBUG_PRINT(" seq=");
        DEBUG_PRINTLN(best.seq_counter);
    } else {
        DEBUG_PRINTLN("[TEST] EEPROM vierge — premier test");
    }

    // 2. Écrire une position de test (valeurs distinctives)
    float testAz = 123.4f;
    float testEl = 56.7f;
    uint16_t testRaw = 5678;
    float testHwt = 56.5f;

    DEBUG_PRINTLN("[TEST] Écriture position test AZ=123.4 EL=56.7...");
    if (!eepromWritePosition(testAz, testEl, testRaw, testHwt, 200.0f, 30.0f)) {
        DEBUG_PRINTLN("[TEST] ÉCHEC écriture !");
        return false;
    }

    // 3. Relire et vérifier
    DEBUG_PRINTLN("[TEST] Relecture + vérification CRC...");
    EepromPositionBlock verify;
    if (!eepromReadBest(verify)) {
        DEBUG_PRINTLN("[TEST] ÉCHEC relecture !");
        return false;
    }

    bool pass = true;

    if (fabsf(verify.pos_az_deg - testAz) > 0.01f) {
        DEBUG_PRINT("[TEST] ÉCHEC AZ: attendu ");
        DEBUG_PRINT(String(testAz, 1));
        DEBUG_PRINT(" lu ");
        DEBUG_PRINTLN(String(verify.pos_az_deg, 1));
        pass = false;
    }

    if (fabsf(verify.pos_el_deg - testEl) > 0.01f) {
        DEBUG_PRINT("[TEST] ÉCHEC EL: attendu ");
        DEBUG_PRINT(String(testEl, 1));
        DEBUG_PRINT(" lu ");
        DEBUG_PRINTLN(String(verify.pos_el_deg, 1));
        pass = false;
    }

    if (verify.enc_az_raw != testRaw) {
        DEBUG_PRINT("[TEST] ÉCHEC enc_az_raw: attendu ");
        DEBUG_PRINT(testRaw);
        DEBUG_PRINT(" lu ");
        DEBUG_PRINTLN(verify.enc_az_raw);
        pass = false;
    }

    if (fabsf(verify.hwt901b_el - testHwt) > 0.01f) {
        DEBUG_PRINT("[TEST] ÉCHEC hwt901b_el: attendu ");
        DEBUG_PRINT(String(testHwt, 1));
        DEBUG_PRINT(" lu ");
        DEBUG_PRINTLN(String(verify.hwt901b_el, 1));
        pass = false;
    }

    // 4. Deuxième écriture pour tester l'alternance A/B
    float testAz2 = 270.5f;
    float testEl2 = 15.3f;
    DEBUG_PRINTLN("[TEST] 2ème écriture AZ=270.5 EL=15.3 (test alternance)...");
    if (!eepromWritePosition(testAz2, testEl2, 12345, 15.2f, 270.5f, 15.3f)) {
        DEBUG_PRINTLN("[TEST] ÉCHEC 2ème écriture !");
        return false;
    }

    // Vérifier que le meilleur bloc est bien le nouveau
    EepromPositionBlock verify2;
    if (!eepromReadBest(verify2)) {
        DEBUG_PRINTLN("[TEST] ÉCHEC relecture 2 !");
        return false;
    }

    if (fabsf(verify2.pos_az_deg - testAz2) > 0.01f) {
        DEBUG_PRINTLN("[TEST] ÉCHEC alternance : mauvais bloc sélectionné");
        pass = false;
    }

    if (verify2.seq_counter != verify.seq_counter + 1) {
        DEBUG_PRINT("[TEST] ÉCHEC seq_counter: attendu ");
        DEBUG_PRINT(verify.seq_counter + 1);
        DEBUG_PRINT(" lu ");
        DEBUG_PRINTLN(verify2.seq_counter);
        pass = false;
    }

    // 5. Dump des deux blocs pour diagnostic
    DEBUG_PRINTLN("[TEST] Dump final:");
    EepromPositionBlock dumpA, dumpB;
    if (eepromReadBlock(EEPROM_BLOCK_A, dumpA)) {
        DEBUG_PRINT("  A: AZ=");
        DEBUG_PRINT(String(dumpA.pos_az_deg, 1));
        DEBUG_PRINT(" EL=");
        DEBUG_PRINT(String(dumpA.pos_el_deg, 1));
        DEBUG_PRINT(" seq=");
        DEBUG_PRINT(dumpA.seq_counter);
        DEBUG_PRINT(" crc=0x");
        DEBUG_PRINT(String(dumpA.crc16, HEX));
        DEBUG_PRINT(blockCrcValid(dumpA) ? " OK" : " BAD");
        DEBUG_PRINTLN("");
    }
    if (eepromReadBlock(EEPROM_BLOCK_B, dumpB)) {
        DEBUG_PRINT("  B: AZ=");
        DEBUG_PRINT(String(dumpB.pos_az_deg, 1));
        DEBUG_PRINT(" EL=");
        DEBUG_PRINT(String(dumpB.pos_el_deg, 1));
        DEBUG_PRINT(" seq=");
        DEBUG_PRINT(dumpB.seq_counter);
        DEBUG_PRINT(" crc=0x");
        DEBUG_PRINT(String(dumpB.crc16, HEX));
        DEBUG_PRINT(blockCrcValid(dumpB) ? " OK" : " BAD");
        DEBUG_PRINTLN("");
    }

    DEBUG_PRINTLN("═══════════════════════════════════════");
    if (pass) {
        DEBUG_PRINTLN("  EEPROM SELF-TEST : PASS ✓");
    } else {
        DEBUG_PRINTLN("  EEPROM SELF-TEST : FAIL ✗");
    }
    DEBUG_PRINTLN("═══════════════════════════════════════");

    return pass;
}

#endif // ENABLE_EEPROM
