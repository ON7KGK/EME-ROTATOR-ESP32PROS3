#ifndef CONFIG_H
#define CONFIG_H

// === Debug ===
// Mettre à 0 pour tester avec PSTRotator (supprime les messages debug sur Serial)
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
// GPIO interdits (internes ProS3, pas sur headers) :
//   IO10 = BATTERY VOLTAGE (VBAT)
//   IO11 = pas routé vers les headers
//   IO33 = DETECT 5V PRESENT (VBUS)
//   IO17 = LDO2 Enable (LED RGB)
//   IO18 = RGB LED WS2812B
//   IO19/IO20 = USB D-/D+
//   IO26-IO32 = SPI Flash + PSRAM
//   IO45/IO46 = Strapping pins
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
#define PIN_W5500_CS        15   // IO15 — Chip Select W5500 (cristal 32 kHz non peuplé)
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
#define PIN_MCP_INT         16   // IO16 — Input, interrupt (cristal 32 kHz non peuplé)

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
//#define PROTOCOL_GS232      // Yaesu GS-232B (PSTRotator, N1MM, etc.)
#define PROTOCOL_EASYCOM    // EasyCom II (HRD, certains logiciels)

// ═══════ Connexion rotateur ═══════
// ETHERNET_ENABLED = 1 : protocole rotateur via TCP Ethernet (W5500)
//                        Serial USB = upload uniquement
// ETHERNET_ENABLED = 0 : protocole rotateur via Serial USB (PSTRotator)
//                        pas d'Ethernet
#define ETHERNET_ENABLED    1

// ═══════ TCP Ports ═══════
#define ROTATOR_TCP_PORT    4533    // Protocole rotateur (GS-232B ou EasyCom)
#define APP_TCP_PORT        4534    // Application Windows (JSON)

// ═══════ Configuration réseau ═══════
// DHCP_ENABLED = 1 : essayer DHCP d'abord, puis fallback IP statique
// DHCP_ENABLED = 0 : IP statique directe (évite double-init W5500 = 1 min de délai)
#define DHCP_ENABLED        1

#define STATIC_IP           192, 168, 0, 200
#define STATIC_GATEWAY      192, 168, 0, 1
#define STATIC_SUBNET       255, 255, 255, 0
#define STATIC_DNS          192, 168, 0, 1
#define STATIC_DNS2         8, 8, 8, 8
#define DHCP_TIMEOUT_MS     5000    // Timeout DHCP en ms (si DHCP_ENABLED)

// ═══════ Paramètres rotor ═══════
#define AZ_MIN 0.0
#define AZ_MAX 360.0
#define EL_MIN 0.0
#define EL_MAX 90.0

// ═══════ Mode simulation (sans hardware) ═══════
// 1 = simuler les encodeurs (pas de hardware), 0 = vrais HH-12
#define SIMULATE_ENCODERS   1

// Vitesse moteur simulée (°/s)
// AZ : 360° en 3 min = 2.0 °/s
// EL : 90° en 2 min  = 0.75 °/s
#define SIM_AZ_SPEED        2.0f
#define SIM_EL_SPEED        0.75f

// ═══════ Encodeurs HH-12 SSI (absolus) ═══════
#define SSI_COUNTS_PER_REV  4096    // 12-bit = 4096 counts par tour
// AZ : HH-12 absolu sur axe final — mapping direct 0-4095 = 0-360°
// EL : HH-12 absolu sur axe final — mapping direct 0-4095 = 0-90°
#define REVERSE_AZ          false   // Inverser sens azimut
#define REVERSE_EL          false   // Inverser sens élévation
#define ENCODER_READ_INTERVAL 20    // Lecture toutes les 20ms

#endif
