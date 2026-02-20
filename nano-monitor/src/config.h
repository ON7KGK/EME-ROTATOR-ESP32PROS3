#ifndef CONFIG_H
#define CONFIG_H

// ════════════════════════════════════════════════════════════════
// EME ROTATOR — UNO R4 Motor Box — Configuration
// ════════════════════════════════════════════════════════════════
// Arduino UNO R4 Minima (Renesas RA4M1, 5V natif)
// 20/20 GPIO utilisées — voir plan pour détails
// ════════════════════════════════════════════════════════════════

// === Debug ===
#define DEBUG 1
#if DEBUG
  #define DEBUG_PRINT(x)   Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// ═══════ RS-485 (Serial1, modules auto-direction TTL) ═══════
// D0 = RX (Serial1), D1 = TX (Serial1)
// Pas de pin DE/RE — module auto-direction
#define RS485_BAUD          115200
#define MODBUS_SLAVE_ID     2       // Slave ID sur le bus RS-485

// ═══════ HH-12 SSI — Azimut (pins indépendants) ═══════
#define PIN_HH12_AZ_CS      2      // D2 — Chip Select AZ
#define PIN_HH12_AZ_SCLK    4      // D4 — Clock AZ
#define PIN_HH12_AZ_DATA    7      // D7 — Data AZ (MISO)

// ═══════ HH-12 SSI — Élévation (pins indépendants) ═══════
#define PIN_HH12_EL_CS      8      // D8 — Chip Select EL
#define PIN_HH12_EL_SCLK   12      // D12 — Clock EL
#define PIN_HH12_EL_DATA   13      // D13 — Data EL (MISO)

// ═══════ MC33926 — Motor PWM (pins directes) ═══════
#define PIN_MOT_AZ_PWM       3     // D3 — PWM azimut (timer)
#define PIN_MOT_EL_PWM       5     // D5 — PWM élévation (timer)

// ═══════ Hall encoders (pins interrupt) ═══════
#define PIN_HALL_AZ_A        6     // D6 — Hall AZ signal A
#define PIN_HALL_AZ_B        9     // D9 — Hall AZ signal B
#define PIN_HALL_EL_A       10     // D10 — Hall EL signal A
#define PIN_HALL_EL_B       11     // D11 — Hall EL signal B

// ═══════ MC33926 — Analog feedback (pins ADC) ═══════
#define PIN_MOT_AZ_FB       A0     // A0 — Current feedback AZ (0.525 V/A)
#define PIN_MOT_EL_FB       A1     // A1 — Current feedback EL (0.525 V/A)

// ═══════ MC33926 — Status Flag ═══════
#define PIN_MC_SF            A2    // A2 — nSF open-drain, LOW=fault

// ═══════ MCP23017 — Interrupt ═══════
#define PIN_MCP_INT          A3    // A3 — MCP23017 INTA (limits + STOP)

// ═══════ I2C (MCP23017) — A4=SDA, A5=SCL (hardware) ═══════
#define I2C_ADDR_MCP23017   0x20

// ═══════ MCP23017 — Pin mapping ═══════
// Port A (GPA) — Inputs (limits + STOP)
#define MCP_LIMIT_CW         0    // GPA0 — Fin de course CW
#define MCP_LIMIT_CCW        1    // GPA1 — Fin de course CCW
#define MCP_LIMIT_UP         2    // GPA2 — Fin de course UP
#define MCP_LIMIT_DOWN       3    // GPA3 — Fin de course DOWN
#define MCP_STOP_BUTTON      4    // GPA4 — Bouton STOP (NF, aussi hardware → EN)

// Port B (GPB) — Outputs (direction moteur)
#define MCP_MOT_AZ_IN1       8    // GPB0 — Direction AZ bit 1
#define MCP_MOT_AZ_IN2       9    // GPB1 — Direction AZ bit 2
#define MCP_MOT_EL_IN1      10    // GPB2 — Direction EL bit 1
#define MCP_MOT_EL_IN2      11    // GPB3 — Direction EL bit 2

// ═══════ Paramètres HH-12 SSI ═══════
#define SSI_COUNTS_PER_REV  4096   // 12-bit = 4096 counts par tour
#define REVERSE_AZ          false  // Inverser sens azimut
#define REVERSE_EL          false  // Inverser sens élévation
#define HH12_READ_INTERVAL  50     // Lecture HH-12 toutes les 50ms

// ═══════ Paramètres moteur ═══════
#define FB_VOLTS_PER_AMP    0.525f // MC33926 FB gain
#define ADC_VREF            5.0f   // UNO R4 = 5V reference
#define ADC_RESOLUTION      1023   // 10-bit ADC
#define CURRENT_WARN        2.00f  // Seuil alerte courant (A)
#define CURRENT_EMERGENCY   2.50f  // Seuil arrêt d'urgence (A)
#define PWM_FREQUENCY       20000  // 20 kHz PWM pour MC33926

// ═══════ Sécurité ═══════
#define MODBUS_WATCHDOG_MS  500    // Timeout sans commande → duty=0
#define SAFETY_CHECK_INTERVAL 5    // Vérifier sécurités toutes les 5ms

#endif
