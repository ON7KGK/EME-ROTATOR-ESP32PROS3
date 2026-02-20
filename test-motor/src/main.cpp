// ════════════════════════════════════════════════════════════════
// TEST DIAGNOSTIC — MC33926 Dual Motor Driver
// ════════════════════════════════════════════════════════════════
// Phase 1 : GPIO direct (digitalWrite) — pas de MCPWM
// Phase 2 : MCPWM si phase 1 OK
// Diagnostics : relecture MCP23017, SF, courant ADC
// ════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <Wire.h>

// ── Pins ──
#define PIN_I2C_SDA     8
#define PIN_I2C_SCL     9
#define PIN_MOT_AZ_D2   21   // D2 du MC33926 M1 (AZ)
#define PIN_MOT_EL_D2   38   // D2 du MC33926 M2 (EL)
#define PIN_MOT_AZ_FB   1    // IO1 — ADC courant AZ
#define PIN_MOT_EL_FB   2    // IO2 — ADC courant EL

// ── MCP23017 registres ──
#define MCP_ADDR        0x27
#define REG_IODIRA      0x00
#define REG_IODIRB      0x01
#define REG_GPPUB       0x0D
#define REG_GPIOA       0x12
#define REG_GPIOB       0x13

// Port B : PB0=M1_IN1, PB1=M1_IN2, PB2=M2_IN1, PB3=M2_IN2
//          PB4=M1_SF,  PB5=M2_SF (entrées, active LOW = fault)
#define DIR_AZ_CW       0x01  // IN1=H, IN2=L → forward
#define DIR_AZ_CCW      0x02  // IN1=L, IN2=H → reverse
#define DIR_EL_UP       0x04
#define DIR_EL_DOWN     0x08

// ── MCP23017 accès direct ──
static void mcpWrite(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MCP_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static uint8_t mcpRead(uint8_t reg) {
    Wire.beginTransmission(MCP_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MCP_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}

// ── Diagnostic : lire et afficher état complet ──
static void printDiag(const char *phase) {
    uint8_t portB = mcpRead(REG_GPIOB);
    bool sf1 = !(portB & 0x10);  // M1_SF, active LOW
    bool sf2 = !(portB & 0x20);  // M2_SF, active LOW

    uint16_t fbAz = (uint16_t)((analogRead(PIN_MOT_AZ_FB) * 3300UL) / 4095);
    uint16_t fbEl = (uint16_t)((analogRead(PIN_MOT_EL_FB) * 3300UL) / 4095);

    Serial.print("  [DIAG] ");
    Serial.print(phase);
    Serial.print(" | PortB=0b");
    for (int i = 7; i >= 0; i--) Serial.print((portB >> i) & 1);
    Serial.print(" | IN: AZ_IN1=");
    Serial.print((portB >> 0) & 1);
    Serial.print(" AZ_IN2=");
    Serial.print((portB >> 1) & 1);
    Serial.print(" EL_IN1=");
    Serial.print((portB >> 2) & 1);
    Serial.print(" EL_IN2=");
    Serial.print((portB >> 3) & 1);
    Serial.print(" | SF: M1=");
    Serial.print(sf1 ? "FAULT!" : "ok");
    Serial.print(" M2=");
    Serial.print(sf2 ? "FAULT!" : "ok");
    Serial.print(" | FB: AZ=");
    Serial.print(fbAz);
    Serial.print("mV EL=");
    Serial.print(fbEl);
    Serial.println("mV");
}

// ── Test une direction avec digitalWrite (pas de PWM) ──
static void testDirect(const char *name, uint8_t dirBits, int d2pin) {
    Serial.println();
    Serial.print(">>> TEST DIRECT: ");
    Serial.print(name);
    Serial.println(" — D2 = digitalWrite HIGH (100%)");

    // Direction via MCP23017
    uint8_t portB = mcpRead(REG_GPIOB);
    portB = (portB & 0xF0) | (dirBits & 0x0F);
    mcpWrite(REG_GPIOB, portB);
    delay(10);

    printDiag("avant D2");

    // D2 = HIGH → moteur ON à 100%
    digitalWrite(d2pin, HIGH);
    Serial.println("  D2 = HIGH — moteur devrait tourner pendant 15s");

    // Diagnostics pendant le fonctionnement
    for (int t = 0; t < 15; t++) {
        delay(1000);
        char buf[16];
        snprintf(buf, sizeof(buf), "t=%ds", t + 1);
        printDiag(buf);
    }

    // D2 = LOW → moteur OFF
    digitalWrite(d2pin, LOW);
    Serial.println("  D2 = LOW — arrêt");

    // Remettre directions à 0
    portB = mcpRead(REG_GPIOB);
    mcpWrite(REG_GPIOB, portB & 0xF0);
    delay(500);
    printDiag("repos");
}

void setup() {
    pinMode(17, OUTPUT);
    digitalWrite(17, HIGH);

    Serial.begin(115200);
    unsigned long t0 = millis();
    while (!Serial && millis() - t0 < 3000) delay(10);

    Serial.println();
    Serial.println("════════════════════════════════════════");
    Serial.println("  DIAGNOSTIC MC33926 — Phase GPIO");
    Serial.println("════════════════════════════════════════");

    // ── D2 pins en sortie simple (pas MCPWM) ──
    pinMode(PIN_MOT_AZ_D2, OUTPUT);
    pinMode(PIN_MOT_EL_D2, OUTPUT);
    digitalWrite(PIN_MOT_AZ_D2, LOW);
    digitalWrite(PIN_MOT_EL_D2, LOW);
    Serial.println("[OK] D2 pins: IO21=LOW, IO38=LOW (moteurs off)");

    // ── ADC courant ──
    pinMode(PIN_MOT_AZ_FB, INPUT);
    pinMode(PIN_MOT_EL_FB, INPUT);

    // ── I2C ──
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 100000);
    delay(500);

    Serial.println("[I2C] Scan bus...");
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("[I2C]   0x");
            if (addr < 16) Serial.print("0");
            Serial.println(String(addr, HEX));
        }
    }

    Wire.setClock(400000);

    // MCP23017 check
    bool mcpFound = false;
    for (int attempt = 0; attempt < 3; attempt++) {
        Wire.beginTransmission(MCP_ADDR);
        if (Wire.endTransmission() == 0) { mcpFound = true; break; }
        Serial.print("[I2C] MCP tentative ");
        Serial.println(attempt + 1);
        delay(500);
    }
    if (!mcpFound) {
        Serial.println("[ERREUR] MCP23017 absent !");
        while (1) delay(1000);
    }
    Serial.println("[OK] MCP23017 à 0x27");

    // MCP23017 config
    mcpWrite(REG_IODIRB, 0x30);  // PB0-3 output, PB4-5 input (SF), PB6-7 output
    mcpWrite(REG_GPPUB, 0x30);   // Pull-up sur SF
    mcpWrite(REG_GPIOB, 0x00);   // Tout à 0

    // Vérification relecture
    uint8_t iodirb = mcpRead(REG_IODIRB);
    uint8_t portb  = mcpRead(REG_GPIOB);
    Serial.print("[MCP] IODIRB=0x");
    Serial.print(String(iodirb, HEX));
    Serial.print("  GPIOB=0x");
    Serial.println(String(portb, HEX));

    // Lire SF au repos
    printDiag("repos initial");

    Serial.println();
    Serial.println("Câblage attendu :");
    Serial.println("  D1  → 3.3V     EN → 3.3V (ou flottant)");
    Serial.println("  D2  → IO21/38  SLEW → VDD");
    Serial.println("  IN1 → MCP PB0/PB2   IN2 → MCP PB1/PB3");
    Serial.println();
    // ════════════════════════════════════
    // AZ CW permanent — prends ton temps pour mesurer
    // ════════════════════════════════════
    Serial.println(">>> AZ CW PERMANENT — D2 (IO21) = HIGH, IN1=1, IN2=0");
    Serial.println("    Mesurer : IO21, M1D1, M1D2, M1IN1, M1IN2, M1OUT1, M1OUT2");
    Serial.println();

    // Direction AZ CW via MCP23017
    uint8_t portB = mcpRead(REG_GPIOB);
    portB = (portB & 0xF0) | DIR_AZ_CW;
    mcpWrite(REG_GPIOB, portB);
    delay(10);

    // D2 = HIGH
    digitalWrite(PIN_MOT_AZ_D2, HIGH);
    Serial.println("    IO21 = HIGH, direction AZ CW active");
    Serial.println("    Diag toutes les 5s — CTRL+C pour arrêter");
    Serial.println();
}

void loop() {
    // Diag toutes les 5 secondes, indéfiniment
    printDiag("AZ CW actif");
    delay(5000);
}
