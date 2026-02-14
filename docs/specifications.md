# EME Rotator Controller — Firmware Specification
# Station ON7KGK — JO20BM — 10 GHz EME
# Document Version: 5.0 — Validated against ProS3 schematic Rev P4

---

## 1. SYSTEM OVERVIEW

Dual-axis antenna rotator controller for 10 GHz EME (Earth-Moon-Earth) communication.
Two microcontrollers: ESP32-S3 (ProS3 by Unexpected Maker) as main controller,
Arduino Nano R4 as monitoring/telemetry slave.

### 1.1 Main Controller — ProS3 (ESP32-S3)
- **Board**: Unexpected Maker ProS3 (ESP32-S3FN16R8)
- **CPU**: Dual-core Xtensa LX7 @ 240 MHz
- **Flash**: 16 MB QSPI, **PSRAM**: 8 MB QSPI
- **GPIO**: 27 available (25 general + 2 I2C Qwiic fixed)
- **USB**: CDC (console via IO19/IO20), UART0 freed for RS-485
- **Framework**: Arduino + ESP-IDF APIs (MCPWM, PCNT, ADC)
- **RTOS**: FreeRTOS dual-core

### 1.2 Monitoring Module — Arduino Nano R4
- **Role**: Telemetry acquisition + autonomous anti-freeze thermostat
- **Sensors**: Voltages (24V, 12V), PA power (directional coupler), 3× DS18B20
- **Output**: Relay for anti-freeze lamp 25-40W (thermostat 5-10°C)
- **Comm**: RS-485 Modbus RTU slave (address 1) → ESP32-S3 master

### 1.3 Mechanical System
- **Slewing drives**: 2× Coresun SVH3-62-RC-24H3033 (AZ + EL)
- **Gear ratio**: 34,224:1
- **Speed**: 0.048 RPM at 24VDC
- **Torque**: 716 N.m driving, 2200 N.m holding (self-locking worm gear)
- **Motor**: 24VDC, 1.9A max

---

## 2. HARDWARE COMPONENTS

### 2.1 Motor Driver — Pololu Dual MC33926
- Voltage: 5-28V, 3A per channel
- Current feedback: FB pin, 0.525 V/A
- Status flag: SF pin, open-drain, LOW = fault (overcurrent or overtemp)
- Enable: EN pin, active HIGH (connected to STOP button via pull-up + NF contact)
- PWM input: D2 pin, up to 20 kHz
- Direction: IN1 + IN2 per motor
- SLEW: tied to VDD (fast slew rate for >10 kHz PWM)

### 2.2 Absolute Encoders — 2× HH-12 (SSI, 12-bit, 5V)
- Interface: SSI over SPI (bit-bang or SPI peripheral)
- Resolution: 12 bits = 4096 positions = 0.088°/step
- Protocol: CS LOW → 16 clocks → read 16 bits → CS HIGH
- Data format: bits[15:4] = 12-bit position, bits[3:2] = status (00=OK)
- Supply: 5V (level shifting required via TXS0108E)
- Read frequency: 1 Hz (for PCNT recalibration)
- Cable: Dedicated CAT6 per encoder, max 2m, SPI speed 500 kHz

### 2.3 Incremental Encoders — 2× Hall 12 PPR (quadrature, 5V)
- Interface: Quadrature A/B → PCNT hardware counters
- Resolution: 12 PPR × 4 (quadrature) × 34,224 (gear) = 1,642,752 counts/rev
- Effective resolution: 0.000219°/count
- Level shifting: VMA452 optocoupler + 74HC14 Schmitt trigger (5V → 3.3V)
- Cable: Shared CAT6 with motor wires

### 2.4 Limit Switches — 4× Siemens 3RG4013-0AG33 (inductive, PNP NO)
- Supply: 10-30V (powered from 24V motor supply)
- Output: PNP NO, 4mm sensing range
- Interface: PNP 24V → [1.5kΩ] → LED PC817 (VMA452) → phototransistor 3.3V → MCP23017
- Detection = LOW on MCP23017 input (no detection = HIGH via pull-up)
- **CRITICAL**: Limit switches are SOFTWARE ONLY (not wired to EN). See safety rules.

### 2.5 Manual Buttons — 4× (CW, CCW, UP, DOWN)
- Type: Momentary pushbutton, active LOW
- Interface: MCP23017 port A pins PA4-PA7 with internal pull-ups
- Pressing a button → MCP23017 interrupt → GPIO 33 → ESP32 ISR

### 2.6 Emergency Stop — 1× NF (normally closed) button
- **HARDWARE safety**: Wired in series with MC33926 EN pin
- Rest (NF closed): EN = HIGH via pull-up → motors enabled
- Pressed (NF open): EN = LOW → motors cut HARDWARE
- Cable cut: EN = LOW → motors cut (FAIL-SAFE)
- GPIO 7 reads the state for software reaction (save position, log)

### 2.7 I/O Expander — MCP23017 (I2C, address 0x20)
- Port A (PA0-PA7): 4 limit switches + 4 manual buttons (all inputs)
- Port B (PB0-PB7): 8 GPIO reserve (LEDs, relays, future expansion)
- Interrupt: INTA/INTB mirrored, open-drain, connected to GPIO 33

### 2.8 Ethernet — W5500 module (SPI)
- Integrated TCP/IP stack (offloads ESP32)
- Two TCP servers: port 4533 (PSTRotator GS-232) + port 4534 (custom app)
- No interrupt pin needed (polled by Ethernet library)

### 2.9 Display — OLED I2C 128×64 (SSD1306, address 0x3F)
- Connected via STEMMA Qwiic bus
- Shows: position, target, state, current, errors

### 2.10 Storage — EEPROM/FRAM I2C (address 0x50)
- AT24C256 (1M write cycles) or FM24C64 (10¹⁴ cycles)
- Stores fine position with double buffering + CRC16
- Connected via STEMMA Qwiic bus

### 2.11 Level Shifting
- **HH-12 → SPI2**: TXS0108E bidirectional 3.3V ↔ 5V (SCLK, MISO, CS_AZ, CS_EL)
- **Halls → PCNT**: VMA452 optocoupler + 74HC14 Schmitt trigger (5V → 3.3V)
- **Limit switches → MCP23017**: VMA452 #2 optocouplers (24V → 3.3V)
- **MC33926, W5500**: Native 3.3V logic, direct connection

---

## 3. PLATFORMIO CONFIGURATION

```ini
[env:pros3]
platform = espressif32
board = um_pros3
framework = arduino
monitor_speed = 115200
board_build.partitions = min_spiffs.csv
lib_deps =
    SPI
    Wire
    arduino-libraries/Ethernet
    adafruit/Adafruit MCP23X17
    adafruit/Adafruit SSD1306
    adafruit/Adafruit GFX Library
    4-20ma/ModbusMaster
    paulstoffregen/OneWire
build_flags =
    -DARDUINO_USB_MODE=1
    -DARDUINO_USB_CDC_ON_BOOT=1
```

ESP-IDF headers available directly from Arduino framework:
```cpp
#include "driver/mcpwm_prelude.h"  // MCPWM motor control
#include "driver/pulse_cnt.h"       // PCNT hardware counter
#include "esp_adc/adc_oneshot.h"    // ADC one-shot mode
#include "esp_timer.h"              // High-resolution timer
```

---

## 4. GPIO MAPPING — VALIDATED vs ProS3 Schematic Rev P4

### 4.1 Forbidden GPIOs (used internally by ProS3)

| GPIO | Internal Function | Status |
|------|-------------------|--------|
| IO15, IO16 | 32 kHz crystal (XTAL_32K_P/N) | FORBIDDEN |
| IO17 | LDO2 Enable (controls 2nd 3.3V regulator + RGB LED power) | FORBIDDEN |
| IO18 | RGB LED WS2812B (DI) | FORBIDDEN |
| IO19, IO20 | USB D−/D+ (USB-C connector) | FORBIDDEN |
| IO26–IO32 | Internal SPI Flash + PSRAM | FORBIDDEN |
| IO45, IO46 | Strapping pins (not on headers) | FORBIDDEN |

### 4.2 GPIOs with internal connections (usable with conditions)

| GPIO | Internal Connection | Usage Condition |
|------|---------------------|-----------------|
| IO0 | Strapping pin (boot mode) | Usable after boot. HIGH at boot = normal SPI Flash boot. |
| IO10 | VBUS sense via R15 (2kΩ to 5V USB) | DANGEROUS if USB connected. NOT USED. |
| IO33 | VBAT sense via R14 (3.3kΩ) | OK if no LiPo battery. Used for MCP23017 INT. |

### 4.3 Fixed I2C (STEMMA Qwiic connector)
```cpp
// I2C fixed in hardware on ProS3 STEMMA Qwiic connector
// IO8 = SDA, IO9 = SCL (not remappable, dedicated to I2C bus)
```

### 4.4 UART0 freed by USB CDC
The ProS3 uses USB CDC for console (via IO19/IO20). UART0 (IO43=U0TXD, IO44=U0RXD)
is freed and available. These pins are used for UART1 (RS-485) via the ESP32-S3 GPIO matrix.

### 4.5 Complete pin definitions — pins_config.h

```cpp
// ═══════════════════════════════════════════════════════
// GPIO MAPPING ProS3 — VALIDATED vs schematic Rev P4
// 25 GPIO + 2 I2C Qwiic = 27 pins used out of 27 available
// ═══════════════════════════════════════════════════════

// ═══════ MC33926 — Current feedback (ADC1 required if WiFi used) ═══════
#define PIN_MOT_AZ_FB       1    // IO1  — ADC1_CH0, 0.525 V/A
#define PIN_MOT_EL_FB       2    // IO2  — ADC1_CH1, 0.525 V/A

// ═══════ PCNT — Hall encoders (via VMA452 + 74HC14) ═══════
#define PIN_HALL_AZ_A       3    // IO3  — PCNT unit 0, signal
#define PIN_HALL_AZ_B       4    // IO4  — PCNT unit 0, control
#define PIN_HALL_EL_A       5    // IO5  — PCNT unit 1, signal
#define PIN_HALL_EL_B       6    // IO6  — PCNT unit 1, control

// ═══════ MC33926 — Status Flag + STOP button ═══════
#define PIN_MC_SF           0    // IO0  — Input, open drain, LOW=fault
                                 //        HIGH at rest = normal SPI boot
#define PIN_STOP_BUTTON     7    // IO7  — Input, interrupt, NF fail-safe

// ═══════ I2C Qwiic (fixed in hardware on ProS3) ═══════
#define PIN_I2C_SDA         8    // IO8  — STEMMA Qwiic SDA (not remappable)
#define PIN_I2C_SCL         9    // IO9  — STEMMA Qwiic SCL (not remappable)

// ═══════ SPI3 — W5500 Ethernet ═══════
#define PIN_W5500_CS        11   // IO11 — Chip Select W5500
#define PIN_W5500_SCLK      12   // IO12 — SPI3 Clock
#define PIN_W5500_MISO      13   // IO13 — SPI3 MISO
#define PIN_W5500_MOSI      14   // IO14 — SPI3 MOSI
// Note: No W5500 INT pin needed. Arduino Ethernet library uses
//       polling (W5500 has integrated TCP/IP stack).

// ═══════ MCPWM — Motor PWM (via MC33926 D2) ═══════
#define PIN_MOT_AZ_PWM      21   // IO21 — MCPWM timer 0, operator A
#define PIN_MOT_EL_PWM      38   // IO38 — MCPWM timer 1, operator A

// ═══════ MCP23017 — Interrupt (limit switches + buttons) ═══════
#define PIN_MCP_INT         33   // IO33 — Input, interrupt
                                 //        Note: R14 (3.3kΩ) to VBAT internally.
                                 //        OK because no LiPo in this application.

// ═══════ SPI2 — HH-12 encoders (via TXS0108E) ═══════
#define PIN_HH12_CS_EL      34   // IO34 — SPI2 CS elevation
#define PIN_HH12_CS_AZ      35   // IO35 — SPI2 CS azimuth
#define PIN_HH12_SCLK       36   // IO36 — SPI2 SCLK
#define PIN_HH12_MISO       37   // IO37 — SPI2 MISO (DATA SSI)

// ═══════ MC33926 — Direction (IN1/IN2) via JTAG pins ═══════
// Note: JTAG disabled (debug via USB CDC). Pins free for GPIO.
#define PIN_MOT_AZ_IN1      39   // IO39 — Direction AZ bit 1 [ex-MTCK]
#define PIN_MOT_AZ_IN2      40   // IO40 — Direction AZ bit 2 [ex-MTDO]
#define PIN_MOT_EL_IN1      41   // IO41 — Direction EL bit 1 [ex-MTDI]
#define PIN_MOT_EL_IN2      42   // IO42 — Direction EL bit 2 [ex-MTMS]

// ═══════ UART1 — RS-485 to Arduino Nano R4 ═══════
// Note: IO43/IO44 are the UART0 (U0TXD/U0RXD) pins of the chip.
//       Freed because console uses USB CDC (IO19/IO20).
//       UART1 is assigned to these pins via ESP32-S3 GPIO matrix.
#define PIN_RS485_TX        43   // IO43 — UART1 TX → MAX485 DI [ex-U0TXD]
#define PIN_RS485_RX        44   // IO44 — UART1 RX ← MAX485 RO [ex-U0RXD]

// ═══════ Unused pins ═══════
// IO10 — VBUS sense (R15 2kΩ to USB 5V). DANGEROUS, do not use.
// IO18 — RGB LED WS2812B of ProS3. Could be used as diagnostic LED
//         but WS2812B will interpret the signals.
```

### 4.6 Summary table

| # | GPIO | Peripheral | Function |
|---|------|-----------|----------|
| 1 | IO0 | GPIO In (strap) | MC33926 Status Flag SF (open drain, HIGH=OK) |
| 2 | IO1 | ADC1_CH0 | Current feedback AZ (M1_FB, 0.525 V/A) |
| 3 | IO2 | ADC1_CH1 | Current feedback EL (M2_FB, 0.525 V/A) |
| 4 | IO3 | PCNT 0 sig | Hall AZ signal A (via VMA452 + 74HC14) |
| 5 | IO4 | PCNT 0 ctrl | Hall AZ signal B (via VMA452 + 74HC14) |
| 6 | IO5 | PCNT 1 sig | Hall EL signal A (via VMA452 + 74HC14) |
| 7 | IO6 | PCNT 1 ctrl | Hall EL signal B (via VMA452 + 74HC14) |
| 8 | IO7 | GPIO In (INT) | STOP button NF (fail-safe, interrupt) |
| 9 | IO8 | I2C SDA | Qwiic: EEPROM (0x50) + MCP23017 (0x20) + OLED (0x3F) |
| 10 | IO9 | I2C SCL | Qwiic: I2C bus 400 kHz |
| 11 | IO11 | SPI3 CS | W5500 Ethernet Chip Select |
| 12 | IO12 | SPI3 SCLK | W5500 Ethernet Clock |
| 13 | IO13 | SPI3 MISO | W5500 Ethernet Data In |
| 14 | IO14 | SPI3 MOSI | W5500 Ethernet Data Out |
| 15 | IO21 | MCPWM T0 | PWM motor AZ (MC33926 M1_D2) |
| 16 | IO33 | GPIO In (INT) | MCP23017 INTA (limit switches + buttons) |
| 17 | IO34 | SPI2 CS | HH-12 Elevation Chip Select |
| 18 | IO35 | SPI2 CS | HH-12 Azimuth Chip Select |
| 19 | IO36 | SPI2 SCLK | HH-12 Clock (via TXS0108E) |
| 20 | IO37 | SPI2 MISO | HH-12 Data SSI (via TXS0108E) |
| 21 | IO38 | MCPWM T1 | PWM motor EL (MC33926 M2_D2) |
| 22 | IO39 | GPIO Out | Direction AZ IN1 [ex-MTCK] |
| 23 | IO40 | GPIO Out | Direction AZ IN2 [ex-MTDO] |
| 24 | IO41 | GPIO Out | Direction EL IN1 [ex-MTDI] |
| 25 | IO42 | GPIO Out | Direction EL IN2 [ex-MTMS] |
| 26 | IO43 | UART1 TX | RS-485 TX to Nano R4 [ex-U0TXD] |
| 27 | IO44 | UART1 RX | RS-485 RX from Nano R4 [ex-U0RXD] |

---

## 5. I2C BUS CONFIGURATION

```cpp
#define I2C_ADDR_EEPROM     0x50   // AT24C256 or FM24C64
#define I2C_ADDR_MCP23017   0x20   // A0=A1=A2=GND
#define I2C_ADDR_OLED       0x3F   // SSD1306 128x64
#define I2C_SPEED           400000 // 400 kHz Fast Mode
```

---

## 6. MCP23017 — Register configuration and pin allocation

```cpp
// Port A (PA0–PA7): Inputs
#define MCP_LIMIT_CW        0  // PA0 — Limit switch CW  (via VMA452 opto, active LOW)
#define MCP_LIMIT_CCW       1  // PA1 — Limit switch CCW (via VMA452 opto, active LOW)
#define MCP_LIMIT_UP        2  // PA2 — Limit switch UP  (via VMA452 opto, active LOW)
#define MCP_LIMIT_DOWN      3  // PA3 — Limit switch DOWN(via VMA452 opto, active LOW)
#define MCP_BTN_CW          4  // PA4 — Button CW   (internal pull-up, active LOW)
#define MCP_BTN_CCW         5  // PA5 — Button CCW  (internal pull-up, active LOW)
#define MCP_BTN_UP          6  // PA6 — Button UP   (internal pull-up, active LOW)
#define MCP_BTN_DOWN        7  // PA7 — Button DOWN (internal pull-up, active LOW)

// Port B (PB0–PB7): Reserve (8 free GPIOs for LEDs, relays, etc.)

// MCP23017 Register Configuration:
//   IODIRA   = 0xFF (port A all inputs)
//   GPPUA    = 0xF0 (pull-up on PA4–PA7 buttons only, PA0–PA3 via optocouplers)
//   GPINTENA = 0xFF (interrupt on all port A pins)
//   INTCONA  = 0x00 (interrupt on change)
//   IOCON.MIRROR = 1 (INTA = INTB, single wire to GPIO 33)
//   IOCON.ODR = 1 (open drain, external pull-up to 3.3V)
```

---

## 7. MOTOR CONTROL — MC33926

### 7.1 Direction truth table

| IN1 | IN2 | D2 (PWM) | Result |
|-----|-----|----------|--------|
| HIGH | LOW | PWM | Rotation direction 1 (CW / UP) |
| LOW | HIGH | PWM | Rotation direction 2 (CCW / DOWN) |
| LOW | LOW | X | Brake (low-side) |
| HIGH | HIGH | X | Brake (high-side) |

### 7.2 MCPWM configuration

```cpp
// Timer config
#define MCPWM_FREQ_HZ       20000  // 20 kHz (ultrasonic, inaudible)
#define MCPWM_RESOLUTION_HZ 10000000  // 10 MHz resolution
#define MCPWM_PERIOD_TICKS  (MCPWM_RESOLUTION_HZ / MCPWM_FREQ_HZ)  // 500 ticks

// Two MCPWM timers, one per axis:
// Timer 0 + Operator 0 + Comparator A → PIN_MOT_AZ_PWM
// Timer 1 + Operator 1 + Comparator A → PIN_MOT_EL_PWM

// Duty cycle: 0 = stopped, 500 = 100% (MCPWM_PERIOD_TICKS)
// Ramps managed by software timer (esp_timer) modifying duty
```

### 7.3 Acceleration profile

```cpp
#define RAMP_ACCEL_MS       2000   // Acceleration ramp duration (ms)
#define RAMP_DECEL_MS       1500   // Deceleration ramp duration (ms)
#define RAMP_UPDATE_MS      20     // Ramp update period (ms)
#define RAMP_MIN_DUTY       50     // Minimum duty to overcome inertia (10%)
#define RAMP_MAX_DUTY       500    // Maximum duty = 100%

// S-curve profile (sigmoid) recommended:
// duty(t) = duty_max * (1 / (1 + exp(-k * (t - t_mid))))
// with k = 6.0 / ramp_duration and t_mid = ramp_duration / 2
// Alternative trapezoidal: duty(t) = duty_max * t / ramp_duration
```

---

## 8. PCNT CONFIGURATION

```cpp
// Unit 0: Azimuth
#define PCNT_AZ_HIGH_LIMIT   1000000   // High limit (wrap)
#define PCNT_AZ_LOW_LIMIT   -1000000   // Low limit (wrap)

// Unit 1: Elevation
#define PCNT_EL_HIGH_LIMIT   500000
#define PCNT_EL_LOW_LIMIT   -500000

// Full quadrature 4× decoding: signal A and B
// Glitch filter: glitch_ns = 1000 (1 µs)
// Resolution: 360.0 / (12 * 4 * 34224) = 0.000219° per count
#define COUNTS_PER_REV      1642752  // 12 PPR * 4 * 34224 gear ratio
#define DEG_PER_COUNT       (360.0 / COUNTS_PER_REV)  // 0.000219°
```

---

## 9. HH-12 READING (SSI over SPI2)

```cpp
#define HH12_SPI_SPEED      500000  // 500 kHz
#define HH12_BITS           12      // 12-bit resolution
#define HH12_DEG_PER_BIT    (360.0 / 4096)  // 0.0879°

// SSI Protocol:
// 1. CS LOW
// 2. Send 16 clocks (SCLK), read 16 bits on MISO
// 3. CS HIGH
// 4. Bits [15:4] = 12-bit position, Bits [3:2] = status
//    Status 00 = OK, 01 = alarm, 10 = alarm, 11 = error
// 5. Raw position = (raw >> 4) & 0x0FFF
// 6. Angle = position * 360.0 / 4096.0

// Read frequency: 1 Hz (PCNT recalibration)
// Validate status bits before using position
```

---

## 10. MOTOR CURRENT READING (ADC)

```cpp
#define ADC_ATTEN           ADC_ATTEN_DB_12    // Range 0–3.3V
#define ADC_BITWIDTH        ADC_BITWIDTH_12    // 12 bits (0–4095)
#define FB_VOLTS_PER_AMP    0.525              // MC33926 FB gain
#define ADC_VREF            3.3                // Reference voltage

// Conversion:
// voltage = adc_raw * ADC_VREF / 4095.0
// current_amps = voltage / FB_VOLTS_PER_AMP

// Thresholds (adjustable after calibration):
#define CURRENT_IDLE        0.20   // No-load current (A)
#define CURRENT_WARN        2.00   // Warning threshold (A)
#define CURRENT_EMERGENCY   2.50   // Emergency stop threshold (A)
#define CURRENT_DECOUPLE    0.10   // Under-current = mechanical decoupling (A)

// Read frequency: 100 Hz (in PID loop)
// Moving average over 10 samples recommended
```

---

## 11. EEPROM STRUCTURE (double buffering)

```cpp
// Two alternating identical blocks (A and B)
// The block with the highest counter AND valid CRC is the most recent

#define EEPROM_BLOCK_A      0x0000
#define EEPROM_BLOCK_B      0x0020
#define EEPROM_BLOCK_SIZE   20     // bytes per block

typedef struct __attribute__((packed)) {
    float    pos_az_deg;     // Azimuth position in degrees (float32, 4 bytes)
    float    pos_el_deg;     // Elevation position in degrees (float32, 4 bytes)
    uint16_t hh12_az_raw;    // HH-12 AZ raw value at storage time
    uint16_t hh12_el_raw;    // HH-12 EL raw value at storage time
    uint32_t seq_counter;    // Sequential counter (incremented each write)
    uint16_t crc16;          // CRC16-CCITT over the preceding 18 bytes
} EepromPositionBlock;       // Total: 20 bytes

// Write moments:
// - At each motor stop (ramp finished)
// - Periodically during movement (every 5–10 seconds)
// - On position change > 0.01°
// - Before shutdown (if detected)
```

---

## 12. MODBUS RTU REGISTERS (Nano R4)

```cpp
#define MODBUS_SLAVE_ADDR   1
#define MODBUS_BAUD         19200
#define MODBUS_SERIAL       Serial1    // UART1 on ESP32-S3
#define MODBUS_POLL_MS      1000       // Polling 1×/second

// Holding registers (read function 03)
// All registers are uint16 unless otherwise noted
#define REG_VOLTAGE_24V     0x0001  // Voltage 24V * 10 (ex: 240 = 24.0V)
#define REG_VOLTAGE_12V     0x0002  // Voltage 12V * 10
#define REG_PA_POWER_FWD    0x0003  // Forward power (mW)
#define REG_PA_POWER_REF    0x0004  // Reflected power (mW)
#define REG_TEMP_BOX        0x0010  // Box temp * 10 (int16, ex: 215 = 21.5°C)
#define REG_TEMP_PA         0x0011  // PA temp * 10
#define REG_TEMP_EXT        0x0012  // Outdoor temp * 10
#define REG_RELAY_WG        0x0020  // Waveguide relay (0/1)
#define REG_SEQ_PA          0x0021  // PA sequencer (0=RX, 1=TX)
#define REG_RELAY_HEAT      0x0022  // Anti-freeze lamp relay (0/1)
#define REG_NANO_STATUS     0x0030  // General status bitmap
```

---

## 13. PROTOCOL — Yaesu GS-232A/B (TCP port 4533)

```cpp
#define GS232_TCP_PORT      4533
#define GS232_LINE_TERM     "\r\n"

// Commands to implement:
// Rx: "C2\r\n"       → Tx: "+0xxx+0yyy\r\n"  (AZ xxx°, EL yyy°)
// Rx: "Mxxx\r\n"     → Point azimuth xxx° (AZ movement only)
// Rx: "Wxxx yyy\r\n" → Point AZ xxx° and EL yyy°
// Rx: "S\r\n"        → Immediate stop all motors (deceleration ramp)
// Rx: "A\r\n"        → Stop azimuth
// Rx: "E\r\n"        → Stop elevation
// Rx: "R\r\n"        → Continuous CW rotation
// Rx: "L\r\n"        → Continuous CCW rotation
// Rx: "U\r\n"        → Continuous elevation UP
// Rx: "D\r\n"        → Continuous elevation DOWN

// C2 response format:
// Position AZ: 000–450 (overlap possible)
// Position EL: 000–180
// Example: "+0180+0045" = AZ 180°, EL 45°

// PSTRotator typically sends C2 every 200–500 ms
// Parser must be tolerant (ignore empty lines, spaces)
```

---

## 14. PROTOCOL — Windows Application (TCP port 4534)

```cpp
#define APP_TCP_PORT        4534

// Format: JSON over TCP, one line per message, terminated by \n
```

### 14.1 Automatic status push (2×/second)

```json
{
  "type": "status",
  "az": 180.05,  "el": 45.12,
  "az_target": 185.00, "el_target": 50.00,
  "moving": true,
  "current_az": 0.85, "current_el": 1.20,
  "limit_cw": false, "limit_ccw": false,
  "limit_up": false, "limit_down": false,
  "stop_pressed": false,
  "mc_fault": false,
  "hh12_az": 2048, "hh12_el": 512,
  "pcnt_az": 820000, "pcnt_el": 205000,
  "nano": {
    "v24": 24.1, "v12": 12.05,
    "pa_fwd": 850, "pa_ref": 12,
    "temp_box": 22.5, "temp_pa": 35.1, "temp_ext": 8.3,
    "wg_relay": 0, "sequencer": 0, "heater": 0
  }
}
```

### 14.2 Incoming commands (from app)

```json
{"cmd": "goto", "az": 185.0, "el": 50.0}
{"cmd": "stop"}
{"cmd": "jog", "dir": "cw"}
{"cmd": "jog_stop"}
{"cmd": "calibrate"}
{"cmd": "set_pid", "kp": 2.0, "ki": 0.1, "kd": 0.05}
{"cmd": "set_limits", "az_min": 0, "az_max": 360, "el_min": 0, "el_max": 90}
{"cmd": "get_config"}
{"cmd": "reboot"}
```

---

## 15. PID CONTROL LOOP

```cpp
#define PID_LOOP_HZ         100    // 100 Hz = 10 ms per cycle
#define PID_DEADBAND_DEG    0.02   // Dead zone (no correction below 0.02°)
#define PID_APPROACH_DEG    2.0    // Slow-down threshold on approach

// Default coefficients (adjustable via NVS and Windows app):
typedef struct {
    float kp;     // Proportional (default: 2.0)
    float ki;     // Integral (default: 0.1)
    float kd;     // Derivative (default: 0.05)
    float max_i;  // Anti-windup integral limit (default: 100.0)
} PidParams;

// One PID per axis (AZ and EL, potentially different coefficients)
// Elevation may need higher Ki due to gravity

// PID output → MCPWM duty cycle (with clamp 0–RAMP_MAX_DUTY)
// Acceleration ramp limits maximum duty variation per cycle
```

---

## 16. STATE MACHINE

```cpp
typedef enum {
    STATE_BOOT,          // Startup, hardware init
    STATE_CALIBRATING,   // Initial calibration (HH-12 read, EEPROM)
    STATE_IDLE,          // Ready, waiting for command
    STATE_MOVING,        // Moving toward target
    STATE_JOGGING,       // Manual movement (button or jog command)
    STATE_TRACKING,      // Continuous tracking (PSTRotator sends targets)
    STATE_DECELERATING,  // Deceleration ramp in progress
    STATE_FAULT,         // Error detected (overcurrent, SF, etc.)
    STATE_STOPPED,       // Emergency stop (STOP button)
} SystemState;

// Transitions:
// BOOT → CALIBRATING → IDLE
// IDLE → MOVING (goto cmd) | JOGGING (button/jog) | TRACKING (C2 polling)
// MOVING → DECELERATING → IDLE (target reached)
// JOGGING → DECELERATING → IDLE (button released)
// TRACKING → IDLE (timeout without C2)
// * → FAULT (overcurrent, SF, anomaly) → IDLE (after correction)
// * → STOPPED (hardware STOP button) → IDLE (button released)
```

---

## 17. BOOT SEQUENCE (15 steps)

```
1. Init hardware: GPIO, SPI2, SPI3, I2C, UART1, ADC
2. Init MCP23017: configure ports, interrupts, pull-ups
3. Init OLED: splash screen "ON7KGK EME Rotator"
4. Init W5500: DHCP or static IP, open TCP ports 4533 + 4534
5. Read HH-12 AZ + EL: absolute raw position (0.088°)
6. Read EEPROM blocks A and B: choose most recent with valid CRC
7. Cross-validation:
   IF |hh12_now - hh12_stored| <= 1 step (0.088°):
      → Fine position restored (precision ~0.01°)
   ELSE:
      → Degraded mode (precision 0.088°), flag "uncalibrated"
      → Auto-recovery on first movement
8. Init PCNT units 0 and 1 with restored position
9. Init MCPWM timers 0 and 1 (off, duty = 0)
10. Read limit switch state + STOP button (IO7)
11. Idle current calibration (read FB for 1 second, motors off)
12. Start PID loop on Core 1 (xTaskCreatePinnedToCore)
13. Start Core 0 tasks: TCP servers, Modbus, OLED
14. Enable Core 1 watchdog
15. State → IDLE
```

---

## 18. FREERTOS TASK ALLOCATION

| Task | Core | Priority | Period | Description |
|------|:----:|:--------:|:------:|-------------|
| task_pid_loop | 1 | configMAX-1 | 10 ms | PID + MCPWM + ADC FB + anomaly detection |
| task_hh12_read | 1 | configMAX-2 | 1000 ms | HH-12 read + PCNT recalibration + EEPROM |
| task_tcp_gs232 | 0 | 5 | Event | TCP server PSTRotator (GS-232) |
| task_tcp_app | 0 | 5 | Event | TCP server Windows app (JSON) |
| task_modbus | 0 | 4 | 1000 ms | Modbus polling Nano R4 |
| task_oled | 0 | 2 | 500 ms | OLED display update |
| ISR stop_btn | 1 | ISR | Event | STOP button interrupt (GPIO 7) |
| ISR mcp_int | 1 | ISR | Event | MCP23017 interrupt (GPIO 33) |

All Core 1 tasks (real-time) have higher priority than Core 0 tasks (communication).
ISRs must be short (set flag + xTaskNotify, no I2C in ISR).

---

## 19. SAFETY RULES (pseudocode)

```cpp
// ─── Rule 1: STOP button (hardware + software) ───
// ISR IO7 (PIN_STOP_BUTTON):
//   stop_flag = true
//   // EN already cut by hardware (pull-up + NF contact)
//   // Software saves position and notifies

// ─── Rule 2: Limit switches (SOFTWARE ONLY) ───
// ISR IO33 (PIN_MCP_INT, MCP23017 INT):
//   mcp_flag = true
//   xTaskNotifyGive(task_pid_loop)
//
// In task_pid_loop, after notification:
//   pins = mcp.readGPIOA()
//   if (pins & (1 << MCP_LIMIT_CW) == 0)  limit_flags |= LIMIT_CW
//   if (pins & (1 << MCP_LIMIT_CCW) == 0) limit_flags |= LIMIT_CCW
//   // ... same for UP, DOWN
//
//   // DIRECTIONAL SAFETY LOGIC:
//   // Before applying PWM:
//   if (direction_az == CW  && (limit_flags & LIMIT_CW))  duty_az = 0
//   if (direction_az == CCW && (limit_flags & LIMIT_CCW)) duty_az = 0
//   if (direction_el == UP  && (limit_flags & LIMIT_UP))  duty_el = 0
//   if (direction_el == DOWN&& (limit_flags & LIMIT_DOWN))duty_el = 0
//
//   // CRITICAL: Movement in OPPOSITE direction REMAINS ALLOWED
//   // This allows the rotator to back off from a limit switch
//   // When sensor disengages: flag is cleared automatically
//   // by the next MCP23017 read

// ─── Rule 3: Overcurrent (stop affected motor) ───
// In task_pid_loop (100 Hz):
//   if (current_az > CURRENT_EMERGENCY) {
//     duty_az = 0; state = STATE_FAULT; log("AZ overcurrent")
//   }

// ─── Rule 4: MC33926 Status Flag ───
// In task_pid_loop:
//   if (digitalRead(PIN_MC_SF) == LOW) {
//     duty_az = 0; duty_el = 0; state = STATE_FAULT
//     log("MC33926 fault (SF LOW)")
//   }

// ─── Rule 5: Mechanical decoupling ───
// If PCNT changes but HH-12 stationary for > 2 seconds
// AND current < CURRENT_IDLE * 1.5:
//   log("Mechanical decoupling detected")
//   state = STATE_FAULT

// ─── Rule 6: Watchdog ───
// Watchdog on Core 1, timeout 5 seconds
// Reset in task_pid_loop at each cycle
// If triggered: reboot ESP32, recover from EEPROM
```

---

## 20. BACKLASH COMPENSATION

```cpp
#define BACKLASH_DEFAULT_DEG  0.1    // Default value (datasheet)
#define BACKLASH_OVERSHOOT    1.2    // Overshoot factor (120% of backlash)

// Storage in NVS:
// backlash_az_deg (float): measured backlash for azimuth axis
// backlash_el_deg (float): measured backlash for elevation axis

// Automatic calibration:
// 1. Move in one direction (e.g., CW) until position stable
// 2. Reverse direction (CCW)
// 3. Count PCNT pulses before HH-12 starts moving
// 4. backlash_deg = pcnt_counts * DEG_PER_COUNT
// 5. Store in NVS

// Application during pointing:
// If direction change detected:
//   target_adj = target + (backlash * BACKLASH_OVERSHOOT * direction)
// Or: unidirectional approach (always CW for AZ final approach)
```

---

## 21. NVS PARAMETERS (persistent configuration)

```cpp
// Namespace: "rotator"
// Keys and default values:

// PID
//   pid_az_kp (float)  = 2.0
//   pid_az_ki (float)  = 0.1
//   pid_az_kd (float)  = 0.05
//   pid_el_kp (float)  = 2.5
//   pid_el_ki (float)  = 0.15
//   pid_el_kd (float)  = 0.05

// Backlash
//   backlash_az (float) = 0.1
//   backlash_el (float) = 0.1

// Limits
//   az_min (float)  = 0.0
//   az_max (float)  = 360.0
//   el_min (float)  = 0.0
//   el_max (float)  = 90.0     // or 180.0 if flip possible

// Calibration offsets
//   hh12_az_offset (float) = 0.0   // Mechanical zero correction AZ
//   hh12_el_offset (float) = 0.0   // Mechanical zero correction EL

// Station
//   callsign (str)    = "ON7KGK"
//   locator (str)     = "JO20BM"
//   latitude (float)  = 50.41
//   longitude (float) = 3.87

// Network
//   ip_mode (uint8)   = 0        // 0=DHCP, 1=static
//   static_ip (str)   = "192.168.1.100"
//   gateway (str)     = "192.168.1.1"
//   subnet (str)      = "255.255.255.0"
//   gs232_port (uint16) = 4533
//   app_port (uint16)   = 4534

// Current thresholds
//   current_idle_az (float) = 0.20   // Calibrated at startup
//   current_idle_el (float) = 0.20
//   current_warn (float)    = 2.00
//   current_emerg (float)   = 2.50
```

---

## 22. SENSOR FUSION AND PRECISION

| Source | Resolution | Limitation | Role |
|--------|-----------|------------|------|
| HH-12 alone | 0.088° | 12 bits | Absolute truth |
| Hall PCNT alone | 0.0002° | Cumulative drift | Real-time precision |
| HH-12 + PCNT fusion | ~0.001° | Periodic recalibration | Optimal position |
| Mechanical backlash | n/a | ~0.1° | Physical limit |
| **Effective precision** | **~0.01°** | **Backlash compensated** | **Target** |

Precision 0.01° (36 arc-seconds) is more than sufficient for 10 GHz EME
(3dB beamwidth of a 90cm dish ≈ 2.5°, 1.8m dish ≈ 1.2°).

### Fusion algorithm
1. At startup: HH-12 provides absolute position (0.088° resolution)
2. EEPROM provides fine position from last session (if CRC valid and HH-12 matches)
3. PCNT provides continuous high-resolution tracking (0.0002°)
4. Every 1 second: HH-12 is re-read, PCNT is recalibrated to match
5. Discrepancy between PCNT and HH-12 → anomaly detection (mechanical decoupling)

---

## 23. ANOMALY DETECTION — TRIPLE REDUNDANCY

### 23.1 Electrical (MC33926 FB + SF)
- No-load current: ~200 mA → ~0.10V on FB (reference)
- 90cm dish: 400-800 mA → 0.2-0.4V (normal)
- 1.8m dish: 800-1500 mA → 0.4-0.8V (normal)
- Full load: 1.9A → ~1.0V (alert)
- Stall: >2.5A → >1.3V (emergency stop)
- SF pin LOW: hardware overtemp or overcurrent (independent of software)

### 23.2 Mechanical (HH-12 vs PCNT)
- PCNT counting + HH-12 stationary → mechanical decoupling
- PCNT stationary + HH-12 moving → Hall/PCNT problem
- Progressive drift → wear, slipping

### 23.3 Software (current profile)
After calibration: "normal" current profile known per axis/direction.
Elevation draws more than azimuth (gravity).
Significant deviation from expected profile → alert.

---

## 24. CAT6 CABLE WIRING

### 24.1 HH-12 (2× dedicated cables, max 2m)
- Pair 1: SCLK + GND
- Pair 2: MISO (DATA) + GND
- Pair 3: CS + GND
- Pair 4: +5V + +5V
- Termination: 100Ω at HH-12 end, 100nF decoupling capacitor

### 24.2 Halls + Motors (2× dedicated cables)
- Pair 1: Hall A + GND
- Pair 2: Hall B + GND
- Pair 3: Hall +5V + GND
- Pair 4: Motor+ + Motor−
- AWG23-24 OK for 1.9A / 2m. Use FTP/STP shielded if near power wires.

---

## 25. BOM SUMMARY (31 items)

### Main controller
1. ProS3 ESP32-S3 (1)
2. Pololu Dual MC33926 (1)
3. W5500 Ethernet module (1)
4. TXS0108E Level Shifter 3.3V↔5V (1)
5. VMA452 Optocoupler 4-channel (2: halls + limit switches)
6. 74HC14 Schmitt Trigger 3.3V (1)
7. EEPROM/FRAM I2C AT24C256 or FM24C64 (1)
8. MCP23017 I2C GPIO expander (1)
9. OLED I2C 128×64 (1)
10. HH-12 SSI 12-bit 5V encoder (2)
11. RS-485 module MAX485/MAX3485 (1)
12. NF STOP button + waterproof enclosure (1)
13. Pushbuttons CW/CCW/UP/DOWN (4)

### Limit switches
14. Siemens 3RG4013-0AG33 inductive PNP NO 10-30V (4)
15. 1.5 kΩ 1/4W resistors (4)
16. Metal target plates (4)

### Monitoring module
17. Arduino Nano R4 (1)
18. RS-485 module MAX485/MAX3485 (1)
19. DS18B20 OneWire (3+)
20. 5V relay module (1)
21. Incandescent bulb 25-40W (1)
22. ADC voltage divider resistors (2×)

### Wiring and passives
23. CAT6 cable 2m (4: 2 HH-12, 2 halls+motor)
24. 100nF ceramic capacitors (6+)
25. 100µF electrolytic capacitor (1)
26. 100Ω termination resistors (2)
27. 10kΩ pull-up resistors (2: STOP + EN)
28. 24V >5A power supply (1)
29. 5V >1A regulator (1)
