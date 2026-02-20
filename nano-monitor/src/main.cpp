// ════════════════════════════════════════════════════════════════
// MODBUS RTU SLAVE MANUEL — UNO R4 (sans bibliothèque)
// ════════════════════════════════════════════════════════════════
// Slave ID 1, RS-485 auto-direction, Serial1 @ 9600
// FC 0x03 : Read Holding Registers
//   Reg 0 : A0 raw ADC
//   Reg 1 : A1 raw ADC
//   Reg 2 : A0 millivolts
//   Reg 3 : A1 millivolts
//   Reg 4 : uptime secondes
//   Reg 5 : Vref mV (5000)
// ════════════════════════════════════════════════════════════════

#include <Arduino.h>

#define RS485_BAUD      9600
#define SLAVE_ID        1
#define BUF_SIZE        64
#define FRAME_SILENCE   4     // ms (3.5 char @ 9600 = ~4 ms)
#define TURNAROUND_MS   2     // délai avant réponse

static uint8_t rxBuf[BUF_SIZE];
static int     rxLen = 0;
static unsigned long lastByteTime = 0;
static unsigned long reqCount = 0;

// ── CRC16 Modbus (polynôme 0xA001, init 0xFFFF) ──
uint16_t crc16_modbus(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else         crc >>= 1;
        }
    }
    return crc;
}

// ── Envoyer réponse FC 0x03 ──
void sendReadResponse(const uint16_t *regs, uint8_t startAddr, uint8_t qty) {
    uint8_t byteCount = qty * 2;
    uint8_t resp[3 + byteCount + 2];  // ID + FC + BC + data + CRC

    resp[0] = SLAVE_ID;
    resp[1] = 0x03;
    resp[2] = byteCount;
    for (int i = 0; i < qty; i++) {
        resp[3 + i * 2]     = regs[startAddr + i] >> 8;
        resp[3 + i * 2 + 1] = regs[startAddr + i] & 0xFF;
    }
    uint16_t crc = crc16_modbus(resp, 3 + byteCount);
    resp[3 + byteCount]     = crc & 0xFF;        // CRC lo
    resp[3 + byteCount + 1] = (crc >> 8) & 0xFF; // CRC hi

    delay(TURNAROUND_MS);
    Serial1.write(resp, 3 + byteCount + 2);
    Serial1.flush();

    // Debug
    Serial.print("[TX] Resp: ");
    for (int i = 0; i < 3 + byteCount + 2; i++) {
        if (resp[i] < 0x10) Serial.print("0");
        Serial.print(resp[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
}

// ── Traiter une trame Modbus complète ──
void processFrame() {
    // Dump de la trame reçue
    Serial.print("[RX] Frame(");
    Serial.print(rxLen);
    Serial.print("): ");
    for (int i = 0; i < rxLen; i++) {
        if (rxBuf[i] < 0x10) Serial.print("0");
        Serial.print(rxBuf[i], HEX);
        Serial.print(" ");
    }
    Serial.println();

    // Taille minimum : ID(1) + FC(1) + data(2+) + CRC(2) = 6
    if (rxLen < 6) {
        Serial.println("[ERR] Trame trop courte");
        return;
    }

    // Vérifier slave ID
    if (rxBuf[0] != SLAVE_ID) {
        Serial.print("[IGN] Pas pour nous (ID=");
        Serial.print(rxBuf[0]);
        Serial.println(")");
        return;
    }

    // Vérifier CRC
    uint16_t calcCrc = crc16_modbus(rxBuf, rxLen - 2);
    uint16_t rxCrc   = rxBuf[rxLen - 2] | ((uint16_t)rxBuf[rxLen - 1] << 8);
    if (calcCrc != rxCrc) {
        Serial.print("[ERR] CRC mismatch: calc=");
        Serial.print(calcCrc, HEX);
        Serial.print(" rx=");
        Serial.println(rxCrc, HEX);
        return;
    }

    reqCount++;
    Serial.print("[OK] CRC valide, FC=0x");
    Serial.print(rxBuf[1], HEX);
    Serial.print("  req #");
    Serial.println(reqCount);

    // ── FC 0x03 : Read Holding Registers ──
    if (rxBuf[1] == 0x03 && rxLen == 8) {
        uint16_t startAddr = ((uint16_t)rxBuf[2] << 8) | rxBuf[3];
        uint16_t quantity  = ((uint16_t)rxBuf[4] << 8) | rxBuf[5];

        if (startAddr + quantity > 6) {
            Serial.println("[ERR] Adresse hors limites");
            return;
        }

        // Lire ADC et remplir registres
        uint16_t regs[6];
        uint16_t a0 = analogRead(A0);
        uint16_t a1 = analogRead(A1);
        regs[0] = a0;
        regs[1] = a1;
        regs[2] = (uint16_t)((a0 * 5000UL) / 1023);
        regs[3] = (uint16_t)((a1 * 5000UL) / 1023);
        regs[4] = (uint16_t)(millis() / 1000);
        regs[5] = 5000;

        sendReadResponse(regs, startAddr, quantity);
    } else {
        Serial.print("[ERR] FC non supporté: 0x");
        Serial.println(rxBuf[1], HEX);
    }
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000);

    Serial.println();
    Serial.println("═══════════════════════════════════");
    Serial.println("  MODBUS SLAVE MANUEL (ID=1)");
    Serial.println("  Serial1 @ 9600, pas de lib");
    Serial.println("═══════════════════════════════════");

    Serial1.begin(RS485_BAUD);

    Serial.println("[DIAG] En attente de trames Modbus...");
}

void loop() {
    // ── Accumuler octets entrants ──
    while (Serial1.available()) {
        if (rxLen < BUF_SIZE) {
            rxBuf[rxLen++] = Serial1.read();
        } else {
            Serial1.read();  // overflow, jeter
        }
        lastByteTime = millis();
    }

    // ── Silence > 4ms = fin de trame ──
    if (rxLen > 0 && (millis() - lastByteTime > FRAME_SILENCE)) {
        processFrame();
        rxLen = 0;
    }
}
