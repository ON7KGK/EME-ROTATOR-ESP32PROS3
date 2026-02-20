#ifndef CONFIG_H
#define CONFIG_H

// === Debug ===
// Mettre à 0 pour éliminer tout code debug à la compilation
// Si DEBUG=1, le flag runtime g_debugSerial contrôle l'activation
#define DEBUG 1
extern bool g_debugSerial;
#if DEBUG
  #define DEBUG_PRINT(x)   do { if (g_debugSerial) Serial.print(x); } while(0)
  #define DEBUG_PRINTLN(x) do { if (g_debugSerial) Serial.println(x); } while(0)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// ═══════════════════════════════════════════════════════════════
// FEATURE FLAGS — Développement incrémental v7.4
// Activer un module APRÈS avoir testé les précédents.
// ═══════════════════════════════════════════════════════════════

// ── ETHERNET + PROTOCOLES ──
#define ENABLE_ETHERNET       1   // W5500 Ethernet (SPI3) — ✅ TESTÉ
#define ENABLE_EASYCOM        1   // EasyCom II TCP port 4533 — ✅ TESTÉ
#define ENABLE_APP_TCP        1   // JSON TCP server port 4534 (app Windows) — ✅ ACTIF
#define ENABLE_AZELDAT        0   // Autonomous mode WSJT-X azel.dat

// ── I/O EXPANDER ──
#define ENABLE_MCP23017       1   // MCP23017 I2C (limites, boutons, direction moteur, LEDs) — ✅ TEST

// ── MOTEURS ──
#define ENABLE_MOTORS         1   // MC33926 (LEDC PWM + ADC courant) — ✅ TEST
#define ENABLE_PID            0   // Boucle PID 100 Hz Core 1

// ── ENCODEUR AZ — choisir UN seul ──
#define ENABLE_AZ_AS5048A     0   // AS5048A SPI 14-bit (hardware cible)
#define ENABLE_AZ_HH12        1   // HH-12 SSI 12-bit — ✅ TEST IO35/36/37
#define ENABLE_AZ_HALL_PCNT   0   // Hall 12 PPR → PCNT hardware

// ── CAPTEUR EL — choisir UN seul ──
#define ENABLE_EL_HWT901B     0   // HWT901B-RS485 inclinomètre (hardware cible)
#define ENABLE_EL_HH12        0   // HH-12 SSI 12-bit (legacy/fallback)

// ── RS-485 / MODBUS ──
#define ENABLE_RS485          1   // RS-485 Modbus master (UART1) — ✅ TEST
#define ENABLE_NANO_R4        1   // Nano R4 télémétrie (addr 1) — ✅ ACTIF
#define ENABLE_HWT901B_BUS    0   // HWT901B sur bus Modbus (addr 2)

// ── GPS ──
#define ENABLE_GPS            1   // GPS NEO-6M (UART2 + PPS) — ✅ DIAGNOSTIC

// ── CALIBRATION ──
#define ENABLE_SOLAR_CAL      0   // Calibration solaire RF (section 27 specs)
#define ENABLE_CAL_OFFSETS    0   // Appliquer offsets calibration aux positions

// ── AFFICHAGE ──
#define ENABLE_OLED           1   // OLED SSD1306 128x32 I2C 0.91" — ✅ I2C scan 0x3C

// ── STOCKAGE ──
#define ENABLE_EEPROM         1   // EEPROM/FRAM position (double buffering + CRC) — ✅ TESTÉ I2C
#define ENABLE_NVS_CONFIG     1   // NVS configuration persistante (runtime via JSON) — ✅ ACTIF

// ── SÉCURITÉ ──
#define ENABLE_STOP_BUTTON    1   // ISR bouton STOP (recommandé toujours actif)
#define ENABLE_WATCHDOG       0   // Watchdog Core 1 (5s timeout)
#define ENABLE_ANOMALY_DET    0   // Détection anomalies (surcourant, découplage)

// ── SIMULATION ──
#define ENABLE_SIM_POSITION   1   // Simuler position AZ/EL (test protocole)
                                  // Désactiver quand vrais encodeurs activés

// ═══════════════════════════════════════════════════════════════
// VALIDATION COMPILE-TIME
// ═══════════════════════════════════════════════════════════════

// Note : AZ/EL encoder mutual exclusion supprimée — sélection runtime via NVS
#if ENABLE_MOTORS && !ENABLE_MCP23017
  #error "MOTORS requires MCP23017 (motor direction via I2C)"
#endif
#if ENABLE_PID && !ENABLE_MOTORS
  #error "PID requires MOTORS"
#endif
#if ENABLE_AZ_HALL_PCNT && !ENABLE_AZ_AS5048A && !ENABLE_AZ_HH12
  #error "PCNT AZ requires an absolute AZ encoder for recalibration"
#endif
#if ENABLE_HWT901B_BUS && !ENABLE_RS485
  #error "HWT901B Modbus requires RS485"
#endif
#if ENABLE_SOLAR_CAL && !ENABLE_GPS
  #error "Solar calibration requires GPS (UTC time for ephemeris)"
#endif
#if ENABLE_AZELDAT && !ENABLE_APP_TCP
  #error "AZELDAT autonomous mode requires APP_TCP (JSON port 4534)"
#endif

// ═══════════════════════════════════════════════════════════════
// GPIO MAPPING ProS3 — v7.4 VALIDATED vs pinout card 2023
// 22 GPIO used + 5 reserve (IO0, IO39, IO40, IO41, IO42)
// ═══════════════════════════════════════════════════════════════

// ── MC33926 — Current feedback (ADC1) ──
#define PIN_MOT_AZ_FB       1    // IO1  — ADC1_CH0, 0.525 V/A
#define PIN_MOT_EL_FB       2    // IO2  — ADC1_CH1, 0.525 V/A

// ── PCNT — Hall encoder AZ uniquement (via VMA452 + 74HC14) ──
#define PIN_HALL_AZ_A       3    // IO3  — PCNT unit 0, signal
#define PIN_HALL_AZ_B       4    // IO4  — PCNT unit 0, control

// ── GPS NEO-6M — Source UTC ──
#define PIN_GPS_RX          5    // IO5  — UART2 RX ← GPS TX (NMEA 9600)
#define PIN_GPS_TX          6    // IO6  — UART2 TX → GPS RX (config)

// ── Bouton STOP (NF fail-safe) ──
#define PIN_STOP_BUTTON     7    // IO7  — Input, interrupt

// ── I2C Qwiic (fixé en hardware sur ProS3) ──
#define PIN_I2C_SDA         8    // IO8  — STEMMA Qwiic SDA
#define PIN_I2C_SCL         9    // IO9  — STEMMA Qwiic SCL

// ── SPI3 — W5500 Ethernet ──
#define PIN_W5500_CS        15   // IO15 — Chip Select W5500
#define PIN_W5500_SCLK      12   // IO12 — SPI3 Clock
#define PIN_W5500_MISO      13   // IO13 — SPI3 MISO
#define PIN_W5500_MOSI      14   // IO14 — SPI3 MOSI

// ── MCP23017 — Interrupt ──
#define PIN_MCP_INT         16   // IO16 — Input, interrupt (limites + boutons + SF)

// ── LEDC — Motor PWM (via MC33926 D2) ──
#define PIN_MOT_AZ_PWM      21   // IO21 — LEDC ch0 (D2 AZ)
#define PIN_MOT_EL_PWM      38   // IO38 — LEDC ch1 (D2 EL)

// ── GPS PPS ──
#define PIN_GPS_PPS         34   // IO34 — PPS interrupt (1 Hz, rising edge = UTC)

// ── AS5048A Azimut — SPI bit-bang dédié (3.3V natif) ──
#define PIN_ENC_AZ_CS       35   // IO35 — CS azimut
#define PIN_ENC_AZ_CLK      36   // IO36 — CLK azimut
#define PIN_ENC_AZ_MISO     37   // IO37 — MISO azimut (data du capteur)

// ── UART1 — RS-485 bus (Nano R4 addr 1 + HWT901B addr 2) ──
#define PIN_RS485_TX        43   // IO43 — UART1 TX → MAX485 DI
#define PIN_RS485_RX        44   // IO44 — UART1 RX ← MAX485 RO

// ── Reserve (5 pins disponibles) ──
// IO0  — Strapping pin, utilisable après boot
// IO39 — Libre (ex AS5048A EL CLK)
// IO40 — Libre (ex AS5048A EL MISO)
// IO41 — Libre (ex MTDI)
// IO42 — Libre (ex MTMS)

// ── LED RGB intégrée ProS3 (debug visuel) ──
#define PIN_RGB_LED         18   // IO18 — WS2812B NeoPixel
#define PIN_LDO2_EN         17   // IO17 — LDO2 Enable (alimente LED RGB)

// ═══════════════════════════════════════════════════════════════
// I2C Addresses
// ═══════════════════════════════════════════════════════════════
#define I2C_ADDR_MCP23017   0x27   // A0=A1=A2=VCC (carte CQROBOT)
#define I2C_ADDR_OLED       0x3C   // SSD1306 128x32 0.91" Midas MDOB128032GV-WI
#define I2C_ADDR_EEPROM     0x50   // AT24C256 ou FM24C64
#define I2C_SPEED           400000 // 400 kHz Fast Mode

// ═══════════════════════════════════════════════════════════════
// TCP Ports
// ═══════════════════════════════════════════════════════════════
#define EASYCOM_TCP_PORT    4533   // EasyCom II (PSTRotator)
#define APP_TCP_PORT        4534   // Application Windows (JSON)

// ═══════════════════════════════════════════════════════════════
// Configuration réseau
// ═══════════════════════════════════════════════════════════════
#define DHCP_ENABLED        1
#define STATIC_IP           192, 168, 0, 200
#define STATIC_GATEWAY      192, 168, 0, 1
#define STATIC_SUBNET       255, 255, 255, 0
#define STATIC_DNS          192, 168, 0, 1
#define STATIC_DNS2         8, 8, 8, 8
#define DHCP_TIMEOUT_MS     5000

// ═══════════════════════════════════════════════════════════════
// RS-485 / Modbus RTU
// ═══════════════════════════════════════════════════════════════
#define RS485_BAUD              9600    // 9600 pour premier test (robuste)
#define NANO_MODBUS_ID          1       // Slave ID du Nano R4 (télémétrie)
#define MODBUS_POLL_INTERVAL_MS 2000    // Polling toutes les 2s (test)

// ═══════════════════════════════════════════════════════════════
// Paramètres rotor
// ═══════════════════════════════════════════════════════════════
#define AZ_MIN 0.0f
#define AZ_MAX 360.0f
#define EL_MIN 0.0f
#define EL_MAX 90.0f

// ═══════════════════════════════════════════════════════════════
// Persistance cible EEPROM + indicateur stale
// ═══════════════════════════════════════════════════════════════
#define STALE_TARGET_MS         300000  // 5 min sans commande → cible "stale"

// ═══════════════════════════════════════════════════════════════
// Moteurs MC33926 (quand ENABLE_MOTORS = 1)
// ═══════════════════════════════════════════════════════════════
#define MOT_PWM_FREQ        20000   // 20 kHz (fast slew mode)
#define MOT_MAX_DUTY        90.0f   // % duty max (jamais 100% → marge thermique)
#define MOT_MIN_DUTY        20.0f   // % duty min (en dessous le moteur cale)
#define MOT_RAMP_DEG        10.0f   // ° début rampe (proportionnel en dessous)
#define MOT_DEADBAND_DEG    0.2f    // ° zone morte (encodeur 0.1° → deadband 2×)
#define MOT_CONTROL_MS      100     // Période contrôle moteur (ms)

// ═══════════════════════════════════════════════════════════════
// Simulation (quand ENABLE_SIM_POSITION = 1)
// ═══════════════════════════════════════════════════════════════
#define SIM_AZ_SPEED        2.0f    // °/s (360° en 3 min)
#define SIM_EL_SPEED        0.75f   // °/s (90° en 2 min)

// ═══════════════════════════════════════════════════════════════
// FreeRTOS — Configuration tâches
// ═══════════════════════════════════════════════════════════════
#define TASK_PID_PERIOD_MS      10      // 100 Hz PID loop
#define TASK_PID_STACK_SIZE     4096
#define TASK_PID_PRIORITY       (configMAX_PRIORITIES - 1)

#define TASK_ENC_PERIOD_MS      1000    // 1 Hz encodeur absolu
#define TASK_ENC_STACK_SIZE     4096
#define TASK_ENC_PRIORITY       (configMAX_PRIORITIES - 2)

#endif // CONFIG_H
