// ════════════════════════════════════════════════════════════════
// EME ROTATOR CONTROLLER — Module MCP23017 I2C Expander (Impl.)
// ════════════════════════════════════════════════════════════════
// Accès direct registres via Wire (pas de lib externe)
// IOCON.BANK = 0 (défaut) → registres intercalés A/B
// ════════════════════════════════════════════════════════════════

#include "config.h"

#if ENABLE_MCP23017

#include "mcp23017.h"
#include <Wire.h>

// ── Registres MCP23017 (BANK=0, mode par défaut) ──
#define REG_IODIRA      0x00  // Direction port A (1=input)
#define REG_IODIRB      0x01  // Direction port B
#define REG_IPOLA       0x02  // Polarité port A
#define REG_IPOLB       0x03  // Polarité port B
#define REG_GPINTENA    0x04  // Interrupt-on-change enable A
#define REG_GPINTENB    0x05  // Interrupt-on-change enable B
#define REG_DEFVALA     0x06  // Default compare value A
#define REG_DEFVALB     0x07  // Default compare value B
#define REG_INTCONA     0x08  // Interrupt control A (0=change, 1=compare)
#define REG_INTCONB     0x09  // Interrupt control B
#define REG_IOCON       0x0A  // Configuration (partagé A/B)
#define REG_GPPUA       0x0C  // Pull-up port A
#define REG_GPPUB       0x0D  // Pull-up port B
#define REG_INTFA       0x0E  // Interrupt flag A
#define REG_INTFB       0x0F  // Interrupt flag B
#define REG_INTCAPA     0x10  // Interrupt capture A
#define REG_INTCAPB     0x11  // Interrupt capture B
#define REG_GPIOA       0x12  // Port A
#define REG_GPIOB       0x13  // Port B
#define REG_OLATA       0x14  // Output latch A
#define REG_OLATB       0x15  // Output latch B

// Cache local du port B (pour read-modify-write sans relire)
static uint8_t portBCache = 0x00;

// ── Accès registre ──

static void mcpWriteReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(I2C_ADDR_MCP23017);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static uint8_t mcpReadReg(uint8_t reg) {
    Wire.beginTransmission(I2C_ADDR_MCP23017);
    Wire.write(reg);
    Wire.endTransmission(false);  // Repeated start
    Wire.requestFrom((uint8_t)I2C_ADDR_MCP23017, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}

// ════════════════════════════════════════════════════════════════
// INITIALISATION
// ════════════════════════════════════════════════════════════════

bool mcpInit() {
    // Vérifier présence sur le bus I2C
    Wire.beginTransmission(I2C_ADDR_MCP23017);
    if (Wire.endTransmission() != 0) {
        DEBUG_PRINTLN("[MCP] ERREUR: MCP23017 ne répond pas sur I2C !");
        return false;
    }

    // IOCON : MIRROR=1 (INTA=INTB), ODR=1 (open-drain INT), SEQOP=0
    // Bit 6 = MIRROR, bit 2 = ODR
    mcpWriteReg(REG_IOCON, 0x44);

    // Port A : tout en entrée (limites + boutons)
    mcpWriteReg(REG_IODIRA, 0xFF);

    // Port B : PB4+PB5 entrées (M1_SF + M2_SF), reste sorties
    // IODIRB = 0x30 (bits 4,5 = input, bits 0-3,6-7 = output)
    mcpWriteReg(REG_IODIRB, 0x30);

    // Pull-ups port A : tous activés (0xFF)
    // PA4-PA7 : boutons (pull-up nécessaire, active LOW)
    // PA0-PA3 : limites (pull-up temporaire tant que optos pas branchés,
    //           les optos VMA452 tirent à LOW quand actifs → compatible)
    mcpWriteReg(REG_GPPUA, 0xFF);

    // Pull-up port B : PB4+PB5 (M1_SF + M2_SF, open-drain)
    mcpWriteReg(REG_GPPUB, 0x30);

    // Interrupts : port A complet (limites + boutons)
    mcpWriteReg(REG_GPINTENA, 0xFF);
    // Interrupt port B : PB4+PB5 (M1_SF + M2_SF)
    mcpWriteReg(REG_GPINTENB, 0x30);

    // Mode interrupt : changement d'état (pas comparaison)
    mcpWriteReg(REG_INTCONA, 0x00);
    mcpWriteReg(REG_INTCONB, 0x00);

    // Sorties port B : tout à 0 (directions = brake, LEDs = éteintes)
    mcpWriteReg(REG_GPIOB, 0x00);
    portBCache = 0x00;

    // Lire les ports pour effacer tout interrupt pending
    mcpReadReg(REG_GPIOA);
    mcpReadReg(REG_GPIOB);

    DEBUG_PRINTLN("[MCP] MCP23017 initialisé (0x27)");
    DEBUG_PRINT("[MCP]   Port A (lu) = 0x");
    DEBUG_PRINTLN(String(mcpReadReg(REG_GPIOA), HEX));
    DEBUG_PRINT("[MCP]   Port B (lu) = 0x");
    DEBUG_PRINTLN(String(mcpReadReg(REG_GPIOB), HEX));

    return true;
}

// ════════════════════════════════════════════════════════════════
// LECTURE
// ════════════════════════════════════════════════════════════════

uint8_t mcpReadPortA() {
    return mcpReadReg(REG_GPIOA);
}

uint8_t mcpReadPortB() {
    return mcpReadReg(REG_GPIOB);
}

uint8_t mcpReadIntCapA() {
    return mcpReadReg(REG_INTCAPA);
}

bool mcpReadFaultAz() {
    // M1_SF = PB4, active LOW → true si fault
    return !(mcpReadReg(REG_GPIOB) & (1 << MCP_MC_SF_AZ));
}

bool mcpReadFaultEl() {
    // M2_SF = PB5, active LOW → true si fault
    return !(mcpReadReg(REG_GPIOB) & (1 << MCP_MC_SF_EL));
}

// ════════════════════════════════════════════════════════════════
// ÉCRITURE
// ════════════════════════════════════════════════════════════════

void mcpSetMotorDir(uint8_t dirBits) {
    // Préserver LEDs (bits hauts) + SF est en entrée (ignoré à l'écriture)
    portBCache = (portBCache & ~MCP_MASK_DIRS) | (dirBits & MCP_MASK_DIRS);
    mcpWriteReg(REG_GPIOB, portBCache);
}

void mcpSetLed(uint8_t ledBit, bool on) {
    if (on) {
        portBCache |= (1 << ledBit);
    } else {
        portBCache &= ~(1 << ledBit);
    }
    mcpWriteReg(REG_GPIOB, portBCache);
}

#endif // ENABLE_MCP23017
