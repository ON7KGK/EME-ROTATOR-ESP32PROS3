#ifndef CONFIG_H
#define CONFIG_H

// === Debug ===
#define DEBUG 1
#if DEBUG
  #define DEBUG_PRINT(x)   Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// ═══════════════════════════════════════════════════════
// GPIO MAPPING ProS3 — VALIDATED vs schematic Rev P4
// 25 GPIO + 2 I2C Qwiic = 27 pins used out of 27 available
// ═══════════════════════════════════════════════════════

// ═══════ MC33926 — Current feedback (ADC1) ═══════
#define PIN_MOT_AZ_FB       1    // IO1  — ADC1_CH0, 0.525 V/A
#define PIN_MOT_EL_FB       2    // IO2  — ADC1_CH1, 0.525 V/A

// ═══════ PCNT — Hall encoders (via VMA452 + 74HC14) ═══════
#define PIN_HALL_AZ_A       3    // IO3  — PCNT unit 0, signal
#define PIN_HALL_AZ_B       4    // IO4  — PCNT unit 0, control
#define PIN_HALL_EL_A       5    // IO5  — PCNT unit 1, signal
#define PIN_HALL_EL_B       6    // IO6  — PCNT unit 1, control

// ═══════ MC33926 — Status Flag + STOP button ═══════
#define PIN_MC_SF           0    // IO0  — Input, open drain, LOW=fault
#define PIN_STOP_BUTTON     7    // IO7  — Input, interrupt, NF fail-safe

// ═══════ I2C Qwiic (fixed in hardware on ProS3) ═══════
#define PIN_I2C_SDA         8    // IO8  — STEMMA Qwiic SDA
#define PIN_I2C_SCL         9    // IO9  — STEMMA Qwiic SCL

// ═══════ SPI3 — W5500 Ethernet ═══════
#define PIN_W5500_CS        11   // IO11 — Chip Select W5500
#define PIN_W5500_SCLK      12   // IO12 — SPI3 Clock
#define PIN_W5500_MISO      13   // IO13 — SPI3 MISO
#define PIN_W5500_MOSI      14   // IO14 — SPI3 MOSI

// ═══════ LED RGB intégrée (NeoPixel WS2812B) ═══════
#define PIN_RGB_LED         18   // IO18 — LED RGB ProS3
#define PIN_LDO2_EN         17   // IO17 — LDO2 Enable (alimente la LED RGB)

// ═══════ MCPWM — Motor PWM (via MC33926 D2) ═══════
#define PIN_MOT_AZ_PWM      21   // IO21 — MCPWM timer 0, operator A
#define PIN_MOT_EL_PWM      38   // IO38 — MCPWM timer 1, operator A

// ═══════ MCP23017 — Interrupt (limit switches + buttons) ═══════
#define PIN_MCP_INT         33   // IO33 — Input, interrupt

// ═══════ SPI2 — HH-12 encoders (via TXS0108E) ═══════
#define PIN_HH12_CS_EL      34   // IO34 — SPI2 CS elevation
#define PIN_HH12_CS_AZ      35   // IO35 — SPI2 CS azimuth
#define PIN_HH12_SCLK       36   // IO36 — SPI2 SCLK
#define PIN_HH12_MISO       37   // IO37 — SPI2 MISO (DATA SSI)

// ═══════ MC33926 — Direction (IN1/IN2) via JTAG pins ═══════
#define PIN_MOT_AZ_IN1      39   // IO39 — Direction AZ bit 1
#define PIN_MOT_AZ_IN2      40   // IO40 — Direction AZ bit 2
#define PIN_MOT_EL_IN1      41   // IO41 — Direction EL bit 1
#define PIN_MOT_EL_IN2      42   // IO42 — Direction EL bit 2

// ═══════ UART1 — RS-485 to Arduino Nano R4 ═══════
#define PIN_RS485_TX        43   // IO43 — UART1 TX → MAX485 DI
#define PIN_RS485_RX        44   // IO44 — UART1 RX ← MAX485 RO

// ═══════ I2C Addresses ═══════
#define I2C_ADDR_EEPROM     0x50
#define I2C_ADDR_MCP23017   0x20
#define I2C_ADDR_OLED       0x3F
#define I2C_SPEED           400000  // 400 kHz Fast Mode

// ═══════ Protocole rotator ═══════
// Décommenter UN SEUL des deux :
#define PROTOCOL_GS232      // Yaesu GS-232B (PSTRotator, N1MM, etc.)
// #define PROTOCOL_EASYCOM    // EasyCom II (HRD, certains logiciels)

// ═══════ TCP Ports ═══════
#define GS232_TCP_PORT      4533    // PSTRotator GS-232
#define APP_TCP_PORT        4534    // Application Windows (JSON)

// ═══════ Paramètres rotor ═══════
#define AZ_MIN 0.0
#define AZ_MAX 360.0
#define EL_MIN 0.0
#define EL_MAX 90.0

#endif
