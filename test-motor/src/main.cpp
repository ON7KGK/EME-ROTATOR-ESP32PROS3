// ════════════════════════════════════════════════════════════════
// TEST MC33926 — Séquence AZ CW/CCW + EL UP/DOWN
// ════════════════════════════════════════════════════════════════
// Câblage corrigé : D1=GND, D2=PWM (IO21/38), EN=3.3V
// 5 secondes par direction, diagnostic SF + courant FB
// ════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <Wire.h>

// ── Pins ──
#define PIN_I2C_SDA     8
#define PIN_I2C_SCL     9
#define PIN_MOT_AZ_D2   21   // D2 du MC33926 M1 (AZ)
#define PIN_MOT_EL_D2   38   // D2 du MC33926 M2 (EL)
#define PIN_MOT_AZ_FB   1    // ADC courant AZ
#define PIN_MOT_EL_FB   2    // ADC courant EL

// ── MCP23017 ──
#define MCP_ADDR        0x27
#define REG_IODIRB      0x01
#define REG_GPPUB       0x0D
#define REG_GPIOB       0x13

// Port B : PB0=M1_IN1, PB1=M1_IN2, PB2=M2_IN1, PB3=M2_IN2
//          PB4=M1_SF,  PB5=M2_SF
#define DIR_AZ_CW       0x01  // IN1=H, IN2=L
#define DIR_AZ_CCW      0x02  // IN1=L, IN2=H
#define DIR_EL_UP       0x04
#define DIR_EL_DOWN     0x08

#define DUTY_PERCENT    30.0f
#define RUN_TIME_MS     5000
#define PAUSE_TIME_MS   2000

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

// ── Diagnostic ──
static void printDiag() {
    uint8_t portB = mcpRead(REG_GPIOB);
    bool sf1 = !(portB & 0x10);
    bool sf2 = !(portB & 0x20);
    uint16_t fbAz = (uint16_t)((analogRead(PIN_MOT_AZ_FB) * 3300UL) / 4095);
    uint16_t fbEl = (uint16_t)((analogRead(PIN_MOT_EL_FB) * 3300UL) / 4095);

    Serial.print("    SF: M1=");
    Serial.print(sf1 ? "FAULT" : "ok");
    Serial.print(" M2=");
    Serial.print(sf2 ? "FAULT" : "ok");
    Serial.print(" | FB: AZ=");
    Serial.print(fbAz);
    Serial.print("mV EL=");
    Serial.print(fbEl);
    Serial.println("mV");
}

// ── Séquence de test ──
struct TestPhase {
    const char *name;
    uint8_t dir;
    int d2pin;
};

static const TestPhase phases[] = {
    { "AZ CW  (droite)",  DIR_AZ_CW,   PIN_MOT_AZ_D2 },
    { "AZ CCW (gauche)",  DIR_AZ_CCW,  PIN_MOT_AZ_D2 },
    { "EL UP  (haut)",    DIR_EL_UP,   PIN_MOT_EL_D2 },
    { "EL DOWN (bas)",    DIR_EL_DOWN, PIN_MOT_EL_D2 },
};
static const int NUM_PHASES = sizeof(phases) / sizeof(phases[0]);

void setup() {
    pinMode(17, OUTPUT);
    digitalWrite(17, HIGH);

    Serial.begin(115200);
    unsigned long t0 = millis();
    while (!Serial && millis() - t0 < 3000) delay(10);

    Serial.println();
    Serial.println("════════════════════════════════════════");
    Serial.println("  TEST MC33926 — D1=GND, D2=GPIO, EN=3.3V");
    Serial.println("  5s par direction, duty 30%");
    Serial.println("════════════════════════════════════════");

    // D2 = LOW au départ (moteurs off)
    pinMode(PIN_MOT_AZ_D2, OUTPUT);
    pinMode(PIN_MOT_EL_D2, OUTPUT);
    digitalWrite(PIN_MOT_AZ_D2, LOW);
    digitalWrite(PIN_MOT_EL_D2, LOW);

    // ADC courant
    pinMode(PIN_MOT_AZ_FB, INPUT);
    pinMode(PIN_MOT_EL_FB, INPUT);

    // I2C
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

    // MCP23017
    bool mcpFound = false;
    for (int attempt = 0; attempt < 3; attempt++) {
        Wire.beginTransmission(MCP_ADDR);
        if (Wire.endTransmission() == 0) { mcpFound = true; break; }
        delay(500);
    }
    if (!mcpFound) {
        Serial.println("[ERREUR] MCP23017 absent !");
        while (1) delay(1000);
    }
    Serial.println("[OK] MCP23017 à 0x27");

    mcpWrite(REG_IODIRB, 0x30);  // PB0-3 output, PB4-5 input
    mcpWrite(REG_GPPUB, 0x30);   // Pull-up sur SF
    mcpWrite(REG_GPIOB, 0x00);   // Directions = 0

    Serial.println("[OK] MCP23017 configuré");
    printDiag();

    Serial.println();
    Serial.println("Démarrage dans 3s...");
    delay(3000);
}

void loop() {
    for (int i = 0; i < NUM_PHASES; i++) {
        const TestPhase &p = phases[i];

        Serial.println();
        Serial.print(">>> ");
        Serial.println(p.name);

        // Direction via MCP23017
        uint8_t portB = mcpRead(REG_GPIOB);
        portB = (portB & 0xF0) | (p.dir & 0x0F);
        mcpWrite(REG_GPIOB, portB);
        delay(10);

        // D2 = HIGH → moteur ON
        digitalWrite(p.d2pin, HIGH);
        Serial.println("    MOTEUR ON (30%)");

        // Diag pendant le fonctionnement
        for (int t = 0; t < 5; t++) {
            delay(1000);
            printDiag();
        }

        // D2 = LOW → moteur OFF
        digitalWrite(p.d2pin, LOW);
        mcpWrite(REG_GPIOB, mcpRead(REG_GPIOB) & 0xF0);
        Serial.println("    STOP");

        delay(PAUSE_TIME_MS);
    }

    Serial.println();
    Serial.println("──── Cycle complet ────");
}
