# EME Rotator Controller — Firmware Specification
# Station ON7KGK — JO20BM — 10 GHz EME
# Document Version: 7.4 — GPS NEO-6M + Solar RF Calibration
# Hybrid: AS5048A AZ + HWT901B-RS485 EL, Hall PCNT AZ only, Nano R4 at parabola arm

---

## 1. SYSTEM OVERVIEW

Dual-axis antenna rotator controller for 10 GHz EME (Earth-Moon-Earth) communication.
Two microcontrollers in separate locations.

### 1.1 Architecture

```
┌─────────────────────────────────────────────────┐
│  AT ROTATOR (weatherproof enclosure)            │
│                                                  │
│  ESP32-S3 ProS3 (main controller)               │
│  ├── MC33926 dual motor driver                   │
│  ├── 1× AS5048A magnetic encoder AZ (SPI, 14-bit)│
│  ├── 1× Hall 12 PPR incremental encoder AZ (PCNT) │
│  ├── GPS NEO-6M (UTC time + PPS)                │
│  │   ├── UART2 (NMEA sentences)                 │
│  │   └── PPS → GPIO34 interrupt (µs precision)    │
│  ├── MCP23017 I2C expander                       │
│  │   ├── 4× limit switches (inductive)           │
│  │   ├── 4× manual buttons                       │
│  │   ├── Motor directions IN1/IN2 (4 outputs)    │
│  │   ├── MC33926 SF status flag (1 input)         │
│  │   └── 3× status LEDs (reserve)                │
│  ├── W5500 Ethernet                              │
│  ├── OLED I2C display                            │
│  ├── EEPROM/FRAM I2C position storage            │
│  └── RS-485 Modbus master → Nano R4 + HWT901B   │
│                                                  │
│  Connections to shack:                           │
│  ├── Ethernet CAT6 (control, 50-100m OK)         │
│  └── 24VDC power cable                           │
└─────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────┐
│  AT PARABOLA ARM (2-3m from ESP32)              │
│  Inside the dish arm with transverter/PA/LNA    │
│                                                  │
│  Arduino Nano R4 (telemetry + thermostat)        │
│  ├── Voltage monitoring (24V, 12V)               │
│  ├── PA power monitoring (directional coupler)   │
│  ├── 3× DS18B20 temperature sensors              │
│  ├── Anti-freeze thermostat (relay + lamp)        │
│  └── RS-485 Modbus slave (address 1)             │
│                                                  │
│  WitMotion HWT901B-RS485 (elevation sensor)      │
│  ├── 9-axis AHRS IMU (accel + gyro + magneto)   │
│  ├── Measures TRUE elevation angle via gravity    │
│  ├── 0.05° static accuracy, Kalman filtered      │
│  ├── Mounted at end of dish arm                   │
│  └── RS-485 Modbus slave (address 2)             │
└─────────────────────────────────────────────────┘
```

### 1.2 Main Controller — ProS3 (ESP32-S3) — At rotator
- **Board**: Unexpected Maker ProS3 (ESP32-S3FN16R8)
- **CPU**: Dual-core Xtensa LX7 @ 240 MHz
- **Flash**: 16 MB QSPI, **PSRAM**: 8 MB QSPI
- **GPIO**: 27 available on headers, 22 used + 5 reserve (IO0, IO39, IO40, IO41, IO42)
- **USB**: CDC (console via IO19/IO20), UART0 freed for RS-485
- **Framework**: Arduino + ESP-IDF APIs (MCPWM, PCNT, ADC)
- **RTOS**: FreeRTOS dual-core
- **Time source**: GPS NEO-6M with PPS (µs UTC precision)

### 1.3 Monitoring Module — Arduino Nano R4 — At parabola arm (2-3m from ESP32)
- **Location**: Inside the dish arm with transverter, PA, LNA, and 10 GHz equipment
- **Role**: Telemetry acquisition + autonomous anti-freeze thermostat
- **Sensors**: Voltages (24V, 12V), PA power (directional coupler), 3× DS18B20
- **Output**: Relay for anti-freeze lamp 25-40W (thermostat 5-10°C)
- **Comm**: RS-485 Modbus RTU slave (address 1) → ESP32-S3 master (2-3m cable)
- **Autonomy**: Thermostat operates independently even if ESP32 is offline

### 1.4 Mechanical System
- **Slewing drives**: 2× Coresun SVH3-62-RC-24H3033 (AZ + EL)
- **Gear ratio**: 34,224:1
- **Speed**: 0.048 RPM at 24VDC
- **Torque**: 716 N.m driving, 2200 N.m holding (self-locking worm gear)
- **Motor**: 24VDC, 1.9A max

---

## 2. HARDWARE COMPONENTS

### 2.1 Motor Driver — Pololu Dual MC33926
- Voltage: 5-28V, 3A per channel
- Current feedback: FB pin, 0.525 V/A → ESP32 ADC (real-time)
- Status flag: SF pin, open-drain, LOW = fault → MCP23017 PB4 (polled)
- Enable: EN pin, active HIGH (connected to STOP button via pull-up + NF contact)
- PWM input: D2 pin, up to 20 kHz → ESP32 MCPWM (real-time)
- Direction: IN1 + IN2 per motor → MCP23017 PB0-PB3 (low frequency)
- SLEW: tied to VDD (fast slew rate for >10 kHz PWM)

### 2.2 Absolute Encoder AZ — 1× AS5048A (SPI, 14-bit, 3.3V, magnetic contactless)
- **Sensor**: ams-OSRAM AS5048A, Hall-effect magnetic rotary position sensor
- **Principle**: Contactless — diametrically magnetized magnet (6×3mm) rotates over IC
- **Interface**: SPI (hardware-compatible) — dedicated CLK + MISO + CS
- **Resolution**: 14 bits = 16384 positions = 0.022°/step
- **Accuracy**: 0.05° nominal (may degrade to ~0.5° due to surrounding steel on SVH3)
- **Repeatability**: Excellent even with static metal distortion (fixed error = calibratable)
- **Supply**: 3.3V native (NO level shifting required, direct connection to ESP32)
- **Temperature**: -40°C to +150°C (ideal for outdoor use)
- **Magnet**: 6mm diameter × 3mm height, diametrically magnetized, N42+ neodymium
- **Air gap**: 0.5 to 2.5mm between magnet and IC surface (tolerant to misalignment ±1mm)
- **Mounting**: AZ slewing drive central bore — magnet on rotating part, PCB on mât (fixed)
- **Note on steel proximity**: SVH3 steel structure distorts field → fixed sinusoidal error on
  full rotation. Acceptable because AS5048A is only used as absolute reference for PCNT
  recalibration, not as primary position sensor. Repeatability matters more than linearity.
- **Read frequency**: 1 Hz (for PCNT recalibration)
- **Cable**: Dedicated CAT6, short run at rotator (~2m max, 500 kHz bit-bang)
- **AZIMUTH ONLY** — elevation uses HWT901B inclinometer instead (see 2.2b)

### 2.2b Absolute Sensor EL — WitMotion HWT901B-RS485 (9-axis AHRS inclinometer)
- **Sensor**: MPU9250 9-axis IMU + RM3100 magnetometer, Kalman filtered
- **Principle**: Measures true inclination angle relative to gravity (accelerometer + gyroscope fusion)
- **Interface**: RS-485 Modbus RTU, slave address 0x02 (on same bus as Nano R4 address 1)
- **Factory default address**: 0x50 — MUST be reconfigured to 0x02 (see section 9c)
- **Accuracy**: 0.05° static (X/Y axis), 0.1° dynamic
- **Output**: 3-axis angle + angular velocity + acceleration + magnetic field + air pressure
- **Gyroscope**: Fixed range 2000°/s, provides EL angular velocity for PID
- **Supply**: 5V (from Nano R4 or dedicated 5V regulator at parabola arm)
- **Temperature**: -40°C to +85°C
- **Mounting**: End of dish arm, near Nano R4 and 10 GHz equipment. Bolted or strapped to
  arm structure. No axis alignment needed — measures angle relative to gravity.
- **Advantage over AS5048A for elevation**: No mechanical coupling to slewing drive shaft,
  immune to backlash/play, measures REAL dish angle, trivial mounting on SVH3
- **Limitation**: CANNOT measure azimuth (gravity invariant to horizontal rotation)
- **Baud rate**: 9600 (default, register BAUD = 0x02)
- **Data rate**: 10 Hz recommended (configurable 0.2-200 Hz via BANDWIDTH register)
- **Response delay**: Factory 3000 µs, reduce to 1000 µs (MODDELAY register 0x74)
- **Algorithm**: Set to 6-axis mode (AXIS6 = 0x01) to disable magnetometer influence
- **Write protocol**: Unlock (KEY=0xB588) → Write (function 0x06) → Save (SAVE=0x0000)
- **Calibration**: Accelerometer calibration on level surface + set angle reference at 0° elevation
- **Housing**: Robust industrial enclosure, suitable for outdoor use
- **NO extra GPIO required** — uses existing RS-485 bus (UART1, IO43/IO44)
- **GitHub SDK**: https://github.com/WITMOTION/WitStandardModbus_WT901C485

### 2.3 Incremental Encoder — 1× Hall 12 PPR (quadrature, 5V) — AZIMUTH ONLY
- Interface: Quadrature A/B → PCNT hardware counter (Unit 0)
- Resolution: 12 PPR × 4 (quadrature) × 34,224 (gear) = 1,642,752 counts/rev
- Effective resolution: 0.000219°/count
- Level shifting: VMA452 optocoupler + 74HC14 Schmitt trigger (5V → 3.3V)
- Cable: Short runs at rotator (ESP32 is co-located)
- **AZIMUTH ONLY** — elevation does NOT use Hall encoder (HWT901B gyroscope
  provides angular velocity for PID, and accelerometer provides absolute angle)

### 2.4 Limit Switches — 4× Siemens 3RG4013-0AG33 (inductive, PNP NO)
- Supply: 10-30V (powered from 24V motor supply)
- Output: PNP NO, 4mm sensing range
- Interface: PNP 24V → [1.5kΩ] → LED PC817 (VMA452) → phototransistor 3.3V → MCP23017
- Detection = LOW on MCP23017 input (no detection = HIGH via pull-up)
- **CRITICAL**: Limit switches are SOFTWARE ONLY (not wired to EN). See safety rules.

### 2.5 Manual Buttons — 4× (CW, CCW, UP, DOWN)
- Type: Momentary pushbutton, active LOW
- Interface: MCP23017 port A pins PA4-PA7 with internal pull-ups
- Pressing a button → MCP23017 interrupt → GPIO 16 → ESP32 ISR

### 2.6 Emergency Stop — 1× NF (normally closed) button
- **HARDWARE safety**: Wired in series with MC33926 EN pin
- Rest (NF closed): EN = HIGH via pull-up → motors enabled
- Pressed (NF open): EN = LOW → motors cut HARDWARE
- Cable cut: EN = LOW → motors cut (FAIL-SAFE)
- GPIO 7 reads the state for software reaction (save position, log)

### 2.7 I/O Expander — MCP23017 (I2C, address 0x20)
- Port A (PA0-PA7): 4 limit switches + 4 manual buttons (all inputs)
- Port B (PB0-PB4): 4 motor directions + SF input
- Port B (PB5-PB7): 3 status LEDs (reserve)
- Interrupt: INTA/INTB mirrored, open-drain, connected to GPIO 16
- **Purely digital** — no ADC, no PWM capability

### 2.8 Ethernet — W5500 module (SPI)
- Integrated TCP/IP stack (offloads ESP32)
- Two TCP servers: port 4533 (PSTRotator GS-232) + port 4534 (custom app)
- No interrupt pin needed (polled by Ethernet library)
- Long-distance link to shack (50-100m CAT6 OK)

### 2.9 Display — OLED I2C 128×64 (SSD1306, address 0x3F)
- Connected via STEMMA Qwiic bus
- Shows: position, target, state, current, errors

### 2.10 Storage — EEPROM/FRAM I2C (address 0x50)
- AT24C256 (1M write cycles) or FM24C64 (10¹⁴ cycles)
- Stores fine position with double buffering + CRC16
- Connected via STEMMA Qwiic bus

### 2.11 Level Shifting
- **AS5048A AZ → SPI**: Native 3.3V — **NO level shifter needed** (direct connection to ESP32)
- **HWT901B EL → RS-485**: Connected via MAX485 transceiver (shared bus with Nano R4)
- **Halls AZ → PCNT**: VMA452 optocoupler + 74HC14 Schmitt trigger (5V → 3.3V)
- **Limit switches → MCP23017**: VMA452 #2 optocouplers (24V → 3.3V)
- **MC33926 PWM/FB, W5500**: Native 3.3V logic, direct connection

### 2.12 GPS — u-blox NEO-6M (UTC time source + PPS)
- **Module**: u-blox NEO-6M (or NEO-8M, pin-compatible, either works)
- **Purpose**: Precise UTC time for solar/lunar ephemeris calculation + PPS sync
- **Position**: Station is FIXED at JO20BM85DP (50.41°N, 3.87°E), coded in NVS.
  GPS position used only for initial verification, NOT for ongoing tracking.
- **Interface**: UART2 (NMEA sentences at 9600 baud)
- **PPS output**: 1 pulse per second, rising edge aligned to UTC second boundary
  - Precision: ±50 ns (when GPS has fix)
  - Connected to ESP32 GPIO34 with hardware interrupt
  - IO34 chosen over IO0 to avoid strapping pin conflict at boot
  - Used to discipline internal microsecond clock
- **Supply**: 3.3V (NEO-6M accepts 2.7-3.6V), ~45 mA during acquisition, ~30 mA tracking
- **Antenna**: Active ceramic patch antenna (included with module), mounted with sky view
- **Fix time**: Cold start ~30s, warm start ~5s (with battery backup on module)
- **NMEA sentences used**: $GPRMC (time + date + fix status), $GPGGA (fix quality + satellites)
- **Mounting**: Inside rotator weatherproof enclosure, antenna cable routed to exterior
  with sky view. Or antenna mounted on top of enclosure.
- **Critical for**: Solar calibration (section 27), lunar tracking ephemeris, OLED time display
- **NOT critical for**: Position (fixed, known, stored in NVS)

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
    mikalhart/TinyGPSPlus
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

## 4. GPIO MAPPING — VALIDATED vs ProS3 pinout card 2023

### 4.1 Forbidden GPIOs (used internally by ProS3)

| GPIO | Internal Function | Status |
|------|-------------------|--------|
| IO10 | BATTERY VOLTAGE (VBAT sense via divider) | FORBIDDEN — not on headers |
| IO11 | Not routed to headers | FORBIDDEN — not accessible |
| IO17 | LDO2 Enable (controls 2nd 3.3V regulator + RGB LED power) | FORBIDDEN |
| IO18 | RGB LED WS2812B (DI) | FORBIDDEN |
| IO19, IO20 | USB D−/D+ (USB-C connector) | FORBIDDEN |
| IO26–IO32 | Internal SPI Flash + PSRAM | FORBIDDEN |
| IO33 | DETECT 5V PRESENT (VBUS sense) | FORBIDDEN — not on headers |
| IO45, IO46 | Strapping pins (not on headers) | FORBIDDEN |

### 4.2 GPIOs with internal connections (usable with conditions)

| GPIO | Internal Connection | Usage Condition |
|------|---------------------|-----------------|
| IO0 | Strapping pin (boot mode) | Usable after boot. HIGH at boot = normal SPI Flash boot. |
| IO3 | Strapping pin (JTAG select) | LOW at boot disables JTAG → frees IO39-42. |
| IO15 | XTAL_32K_P (32 kHz crystal pad, NOT populated) | Available on header. Safe for general use. |
| IO16 | XTAL_32K_N (32 kHz crystal pad, NOT populated) | Available on header. Safe for general use. |

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
// ═══════════════════════════════════════════════════════════════
// GPIO MAPPING ProS3 — VALIDATED vs pinout card 2023
// 22 GPIO + 2 I2C Qwiic = 22 pins used, 5 reserve (IO0, IO39, IO40, IO41, IO42)
// ═══════════════════════════════════════════════════════════════

// ═══════ MC33926 — Current feedback (ADC1 required if WiFi used) ═══════
#define PIN_MOT_AZ_FB       1    // IO1  — ADC1_CH0, 0.525 V/A
#define PIN_MOT_EL_FB       2    // IO2  — ADC1_CH1, 0.525 V/A

// ═══════ PCNT — Hall encoder AZ only (via VMA452 + 74HC14) ═══════
#define PIN_HALL_AZ_A       3    // IO3  — PCNT unit 0, signal
                                 //        Strapping pin: LOW at boot → JTAG disabled (OK)
#define PIN_HALL_AZ_B       4    // IO4  — PCNT unit 0, control

// ═══════ GPS NEO-6M — UTC time source ═══════
#define PIN_GPS_RX          5    // IO5  — UART2 RX ← GPS TX (NMEA 9600 baud)
#define PIN_GPS_TX          6    // IO6  — UART2 TX → GPS RX (config commands, optional)
#define PIN_GPS_PPS         34   // IO34 — PPS interrupt (1 Hz, rising edge = UTC second)
                                 //        No strapping constraints, safe for always-active signal

// ═══════ STOP button ═══════
#define PIN_STOP_BUTTON     7    // IO7  — Input, interrupt, NF fail-safe

// ═══════ I2C Qwiic (fixed in hardware on ProS3) ═══════
#define PIN_I2C_SDA         8    // IO8  — STEMMA Qwiic SDA (not remappable)
#define PIN_I2C_SCL         9    // IO9  — STEMMA Qwiic SCL (not remappable)

// ═══════ SPI3 — W5500 Ethernet ═══════
#define PIN_W5500_CS        15   // IO15 — Chip Select W5500 (XTAL_32K_P, not populated)
#define PIN_W5500_SCLK      12   // IO12 — SPI3 Clock
#define PIN_W5500_MISO      13   // IO13 — SPI3 MISO
#define PIN_W5500_MOSI      14   // IO14 — SPI3 MOSI

// ═══════ MCP23017 — Interrupt ═══════
#define PIN_MCP_INT         16   // IO16 — Input, interrupt (XTAL_32K_N, not populated)

// ═══════ MCPWM — Motor PWM (via MC33926 D2) ═══════
#define PIN_MOT_AZ_PWM      21   // IO21 — MCPWM timer 0, operator A
#define PIN_MOT_EL_PWM      38   // IO38 — MCPWM timer 1, operator A

// ═══════ AS5048A Azimut — Dedicated SPI (3 pins, 3.3V native) ═══════
#define PIN_ENC_AZ_CS       35   // IO35 — CS azimut
#define PIN_ENC_AZ_CLK      36   // IO36 — CLK azimut
#define PIN_ENC_AZ_MISO     37   // IO37 — MISO azimut (input, data from sensor)

// ═══════ UART1 — RS-485 bus (Nano R4 addr 1 + HWT901B addr 2, at parabola arm) ═══════
#define PIN_RS485_TX        43   // IO43 — UART1 TX → MAX485 DI [ex-U0TXD]
#define PIN_RS485_RX        44   // IO44 — UART1 RX ← MAX485 RO [ex-U0RXD]

// ═══════ Reserve (5 pins available) ═══════
// IO0  — Strapping pin (HIGH at boot = normal SPI Flash). Usable after boot, reserved.
// IO39 — Free [ex-MTCK] (was AS5048A EL CLK in v7.0)
// IO40 — Free [ex-MTDO] (was AS5048A EL MISO in v7.0)
// IO41 — ex-MTDI, JTAG disabled, free GPIO
// IO42 — ex-MTMS, JTAG disabled, free GPIO

// ═══════ Not on headers ═══════
// IO10 — BATTERY VOLTAGE (VBAT sense). Not on headers.
// IO11 — Not routed to headers. Not accessible.
// IO33 — DETECT 5V PRESENT (VBUS sense). Not on headers.
// IO18 — RGB LED WS2812B of ProS3 (diagnostic LED possible).

// ═══════ MC33926 direction + SF — via MCP23017 (I2C) ═══════
// Motor directions (IN1/IN2) and Status Flag (SF) are on the MCP23017
// expander, NOT on direct ESP32 GPIO. See section 6 for MCP23017 mapping.
// Direction changes are infrequent (~50 µs I2C write, negligible in 10 ms PID).
```

### 4.6 Summary table

| # | GPIO | Peripheral | Function |
|---|------|-----------|----------|
| 1 | IO1 | ADC1_CH0 | Current feedback AZ (M1_FB, 0.525 V/A) |
| 2 | IO2 | ADC1_CH1 | Current feedback EL (M2_FB, 0.525 V/A) |
| 3 | IO3 | PCNT 0 sig | Hall AZ signal A (via VMA452 + 74HC14) |
| 4 | IO4 | PCNT 0 ctrl | Hall AZ signal B (via VMA452 + 74HC14) |
| 5 | IO5 | UART2 RX | GPS NEO-6M TX → ESP32 (NMEA 9600) |
| 6 | IO6 | UART2 TX | ESP32 → GPS NEO-6M RX (config, optional) |
| 7 | IO34 | GPIO In (INT) | GPS PPS (1 Hz, rising edge = UTC second) |
| 8 | IO7 | GPIO In (INT) | STOP button NF (fail-safe, interrupt) |
| 9 | IO8 | I2C SDA | Qwiic: MCP23017 (0x20) + OLED (0x3F) + EEPROM (0x50) |
| 10 | IO9 | I2C SCL | Qwiic: I2C bus 400 kHz |
| 11 | IO12 | SPI3 SCLK | W5500 Ethernet Clock |
| 12 | IO13 | SPI3 MISO | W5500 Ethernet Data In |
| 13 | IO14 | SPI3 MOSI | W5500 Ethernet Data Out |
| 14 | IO15 | SPI3 CS | W5500 Ethernet Chip Select |
| 15 | IO16 | GPIO In (INT) | MCP23017 INTA (limits + buttons + SF) |
| 16 | IO21 | MCPWM T0 | PWM motor AZ (MC33926 M1_D2) |
| 17 | IO35 | GPIO Out | AS5048A AZ CS (SPI, 3.3V native) |
| 18 | IO36 | GPIO Out | AS5048A AZ CLK (SPI, 3.3V native) |
| 19 | IO37 | GPIO In | AS5048A AZ MISO (SPI, 3.3V native) |
| 20 | IO38 | MCPWM T1 | PWM motor EL (MC33926 M2_D2) |
| 21 | IO43 | UART1 TX | RS-485 TX (Nano R4 + HWT901B) [ex-U0TXD] |
| 22 | IO44 | UART1 RX | RS-485 RX (Nano R4 + HWT901B) [ex-U0RXD] |
| — | IO0 | Reserve | Strapping pin, usable after boot |
| — | IO39 | Reserve | Free [ex-MTCK] (was AS5048A EL CLK) |
| — | IO40 | Reserve | Free [ex-MTDO] (was AS5048A EL MISO) |
| — | IO41 | Reserve | ex-MTDI, free GPIO |
| — | IO42 | Reserve | ex-MTMS, free GPIO |

---

## 5. I2C BUS CONFIGURATION

```cpp
#define I2C_ADDR_MCP23017   0x20   // A0=A1=A2=GND
#define I2C_ADDR_OLED       0x3F   // SSD1306 128x64
#define I2C_ADDR_EEPROM     0x50   // AT24C256 or FM24C64
#define I2C_SPEED           400000 // 400 kHz Fast Mode
```

---

## 6. MCP23017 — Pin allocation and register configuration

### 6.1 Pin allocation

```cpp
// ═══════ Port A (PA0–PA7): ALL INPUTS ═══════
#define MCP_LIMIT_CW        0  // PA0 — Limit switch CW  (via VMA452 opto, active LOW)
#define MCP_LIMIT_CCW       1  // PA1 — Limit switch CCW (via VMA452 opto, active LOW)
#define MCP_LIMIT_UP        2  // PA2 — Limit switch UP  (via VMA452 opto, active LOW)
#define MCP_LIMIT_DOWN      3  // PA3 — Limit switch DOWN(via VMA452 opto, active LOW)
#define MCP_BTN_CW          4  // PA4 — Button CW   (internal pull-up, active LOW)
#define MCP_BTN_CCW         5  // PA5 — Button CCW  (internal pull-up, active LOW)
#define MCP_BTN_UP          6  // PA6 — Button UP   (internal pull-up, active LOW)
#define MCP_BTN_DOWN        7  // PA7 — Button DOWN (internal pull-up, active LOW)

// ═══════ Port B (PB0–PB7): OUTPUTS + 1 INPUT ═══════
#define MCP_MOT_AZ_IN1      8  // PB0 — Output: MC33926 M1_IN1 (direction AZ)
#define MCP_MOT_AZ_IN2      9  // PB1 — Output: MC33926 M1_IN2 (direction AZ)
#define MCP_MOT_EL_IN1     10  // PB2 — Output: MC33926 M2_IN1 (direction EL)
#define MCP_MOT_EL_IN2     11  // PB3 — Output: MC33926 M2_IN2 (direction EL)
#define MCP_MC_SF           12  // PB4 — Input:  MC33926 SF (open-drain, LOW=fault)
#define MCP_LED_STATUS      13  // PB5 — Output: LED status (green)
#define MCP_LED_FAULT       14  // PB6 — Output: LED fault (red)
#define MCP_LED_MOVING      15  // PB7 — Output: LED moving (yellow)
```

### 6.2 Register configuration

```cpp
// Register configuration:
//   IODIRA   = 0xFF   Port A all inputs
//   IODIRB   = 0x10   Port B: PB4 input (SF), PB0-PB3+PB5-PB7 outputs
//   GPPUA    = 0xF0   Pull-up on PA4–PA7 (buttons only, PA0–PA3 via optos)
//   GPPUB    = 0x10   Pull-up on PB4 (SF is open-drain)
//   GPINTENA = 0xFF   Interrupt on all port A pins (limits + buttons)
//   GPINTENB = 0x10   Interrupt on PB4 (SF fault detection)
//   INTCONA  = 0x00   Interrupt on change (port A)
//   INTCONB  = 0x00   Interrupt on change (port B)
//   IOCON    = MIRROR | ODR   (INTA = INTB mirrored, open-drain output)
//                              Single wire to ESP32 GPIO 16
```

### 6.3 Motor direction via MCP23017

```cpp
// Direction changes are INFREQUENT (only at start of movement or reversal).
// One I2C write at 400 kHz takes ~50 µs — negligible in a 10 ms PID cycle.
//
// To set direction:
//   mcp.writeGPIOB((mcp.readGPIOB() & 0xF0) | direction_bits)
//
// Direction truth table (same as MC33926):
//   CW  / UP   : IN1=HIGH, IN2=LOW   → PBx = 0b01
//   CCW / DOWN  : IN1=LOW,  IN2=HIGH  → PBx = 0b10
//   Brake       : IN1=LOW,  IN2=LOW   → PBx = 0b00
//
// Combined byte for Port B lower nibble:
//   AZ_CW  + EL_UP   = 0b00000101 = 0x05
//   AZ_CCW + EL_DOWN = 0b00001010 = 0x0A
//   AZ_CW  + EL_DOWN = 0b00001001 = 0x09
//   All brake         = 0b00000000 = 0x00
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
// Timer 0 + Operator 0 + Comparator A → PIN_MOT_AZ_PWM (IO21)
// Timer 1 + Operator 1 + Comparator A → PIN_MOT_EL_PWM (IO38)

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

## 8. PCNT CONFIGURATION — AZIMUTH ONLY

```cpp
// Unit 0: Azimuth (only PCNT unit used)
#define PCNT_AZ_HIGH_LIMIT   1000000   // High limit (wrap)
#define PCNT_AZ_LOW_LIMIT   -1000000   // Low limit (wrap)

// NO PCNT Unit 1 for elevation — HWT901B provides both:
//   - Absolute angle via accelerometer/gravity (0.05° static)
//   - Angular velocity via gyroscope (for PID derivative term)
// This eliminates Hall EL wiring, level shifting, and PCNT complexity.

// Full quadrature 4x decoding: signal A and B
// Glitch filter: glitch_ns = 1000 (1 us)
// Resolution: 360.0 / (12 * 4 * 34224) = 0.000219 deg per count
#define COUNTS_PER_REV      1642752  // 12 PPR * 4 * 34224 gear ratio
#define DEG_PER_COUNT       (360.0 / COUNTS_PER_REV)  // 0.000219 deg
```

---

## 9. AS5048A READING — AZIMUTH ONLY (SPI bit-bang, 3.3V native)

```cpp
#define AS5048A_CMD_ANGLE   0x3FFF   // Read angle register (14-bit)
#define AS5048A_BITS        14       // 14-bit resolution
#define AS5048A_DEG_PER_BIT (360.0 / 16384)  // 0.02197 deg
#define AS5048A_CLK_HZ      500000   // 500 kHz SPI clock (safe for 2m cable)

// AZIMUTH ONLY — elevation uses HWT901B inclinometer on RS-485 bus (section 9b).
// AS5048A is 3.3V NATIVE — no level shifter needed.
//
// SPI protocol (Mode 1: CPOL=0, CPHA=1):
// 1. CS LOW
// 2. Send 16-bit command (0xFFFF for NOP, 0x3FFF for angle read)
// 3. CS HIGH
// 4. CS LOW
// 5. Read 16-bit response (previous command result)
// 6. CS HIGH
// 7. Bit [15] = parity (even), Bit [14] = error flag
// 8. Bits [13:0] = 14-bit angle (0 = 0°, 16383 = 359.978°)
//
// NOTE: AS5048A returns data on NEXT transaction (pipeline).
//       First read sends NOP, second read gets angle.
//
// Error handling:
// - If bit[14] = 1: read error register (0x0001) and clear
// - Errors: framing, command invalid, parity
//
// Read frequency: 1 Hz (PCNT AZ recalibration)
// Multiple reads + averaging recommended for best accuracy (0.05°)
//
// Azimut uses: PIN_ENC_AZ_CS (IO35), PIN_ENC_AZ_CLK (IO36), PIN_ENC_AZ_MISO (IO37)
//
// Note on steel proximity: SVH3 steel structure creates fixed sinusoidal error.
// Acceptable for PCNT recalibration (repeatability > linearity).
// Bit-bang preferred for flexibility.
```

### 9b. HWT901B READING — ELEVATION ONLY (RS-485 Modbus RTU)

```cpp
#define HWT901B_MODBUS_ADDR    0x02    // Target Modbus address (AFTER reconfiguration)
#define HWT901B_FACTORY_ADDR   0x50    // Factory default address — MUST be changed!
#define HWT901B_BAUD           9600    // Default baud rate (register BAUD = 0x02)
#define HWT901B_POLL_MS        1000    // Poll every 1 second (same cycle as Modbus)

// ═══════════════════════════════════════════════════════════════
// IMPORTANT: HWT901B ships with Modbus address 0x50 (factory default)
// It must be reconfigured to 0x02 ONCE before use on the shared bus.
// See section 9c for the full initial configuration procedure.
// GitHub reference: https://github.com/WITMOTION/WitStandardModbus_WT901C485
// ═══════════════════════════════════════════════════════════════

// HWT901B is on the SAME RS-485 bus as Arduino Nano R4.
// ESP32 is Modbus master, polls Nano R4 (addr 1) then HWT901B (addr 2) alternately.
//
// === MODBUS PROTOCOL DETAILS (from official WitMotion documentation) ===
//
// Read: Function code 0x03 (Read Holding Registers)
//   TX: [ADDR] [0x03] [REG_H] [REG_L] [LEN_H] [LEN_L] [CRC_L] [CRC_H]
//   RX: [ADDR] [0x03] [BYTE_COUNT] [DATA1_H] [DATA1_L] ... [CRC_L] [CRC_H]
//
// Write: Function code 0x06 (Write Single Register)
//   TX: [ADDR] [0x06] [REG_H] [REG_L] [DATA_H] [DATA_L] [CRC_L] [CRC_H]
//   RX: echo of TX (confirmation)
//
// CRC: Standard Modbus CRC16 (polynomial 0xA001)
//
// === DATA REGISTERS (read-only, function 0x03) ===
//
// Modbus registers (function 03, holding registers):
// 0x34: Accel X  (int16, accel_g = raw / 32768.0 * 16.0)
// 0x35: Accel Y
// 0x36: Accel Z
// 0x37: Gyro X   (int16, dps = raw / 32768.0 * 2000.0)
// 0x38: Gyro Y   — ELEVATION ANGULAR VELOCITY for PID
// 0x39: Gyro Z
// 0x3A: Mag X    (int16, raw LSB units)
// 0x3B: Mag Y
// 0x3C: Mag Z
// 0x3D: Roll     (int16, deg = raw / 32768.0 * 180.0)
// 0x3E: Pitch    — ELEVATION ANGLE (gravity-based)
// 0x3F: Yaw      (magnetic heading, NOT used in our application)
// 0x40: Temp     (int16, celsius = raw / 100.0)
//
// Optimized read: function 03, start 0x34, count 13 → reads Accel through Temp
// Minimal read:   function 03, start 0x38, count 7  → reads Gyro Y through Pitch
// This gets both EL velocity (gyro Y at 0x38) and EL angle (pitch at 0x3E).
//
// EL POSITION: Pitch register (0x3E) → true elevation angle via gravity
//   int16_t raw = response[offset_pitch];
//   float elevation_deg = (float)raw / 32768.0f * 180.0f;
//
// EL VELOCITY: Gyro Y register (0x38) → angular velocity for PID derivative term
//   int16_t raw_gy = response[offset_gy];
//   float el_velocity_dps = (float)raw_gy / 32768.0f * 2000.0f;
//
// TEMPERATURE: register 0x40
//   float temp_celsius = (float)raw_temp / 100.0f;
//
// === CONFIGURATION REGISTERS (R/W, function 0x06) ===
// CRITICAL: Before ANY write, must unlock with KEY register!
//
// 0x00: SAVE      — 0x0000=save, 0x00FF=reboot, 0x0001=factory reset
// 0x01: CALSW     — Calibration mode (0x08 = set angle reference/zero)
// 0x04: BAUD      — 0x02=9600, 0x03=19200, 0x06=115200
// 0x1A: IICADDR   — Device address (0x01-0x7F), factory default 0x50
// 0x1B: LEDOFF    — 0x01=LED off, 0x00=LED on
// 0x1F: BANDWIDTH — 0x04=20Hz (default), 0x03=42Hz, 0x02=98Hz
// 0x23: ORIENT    — 0x00=horizontal (default), 0x01=vertical
// 0x24: AXIS6     — 0x00=9-axis (default), 0x01=6-axis (no magnetometer)
// 0x25: FILTK     — Dynamic filter K (default 30, range 1-10000)
// 0x2A: ACCFILT   — Accel filter (default 500, range 1-10000)
// 0x69: KEY       — UNLOCK: write 0xB588 before any other write!
// 0x74: MODDELAY  — RS-485 response delay in µs (default 3000 = 3ms)
//
// === RESPONSE DELAY ===
// Default MODDELAY = 3000 µs (3 ms) between receiving query and sending response.
// Consider reducing to 1000 µs to speed up polling cycle.
// At 9600 baud, a 7-register read takes ~15ms total (TX + delay + RX).
//
// NO extra GPIO needed — shares UART1 (IO43/IO44) with Nano R4.
// NO Hall encoder EL needed — HWT901B provides both position AND velocity.
```

### 9c. HWT901B INITIAL CONFIGURATION (one-time setup)

```cpp
// ═══════════════════════════════════════════════════════════════
// ONE-TIME SETUP PROCEDURE — Run BEFORE integrating on shared bus
// Can be done via ESP32 or USB-to-RS485 adapter + PC
//
// The HWT901B ships with address 0x50. All commands below use 0x50.
// After changing address to 0x02, subsequent commands must use 0x02.
// ═══════════════════════════════════════════════════════════════
//
// WRITE PROTOCOL: For each write operation:
//   1. Unlock:  TX: [0x50] [0x06] [0x00] [0x69] [0xB5] [0x88] [CRC_L] [CRC_H]
//   2. Write:   TX: [0x50] [0x06] [REG_H] [REG_L] [DATA_H] [DATA_L] [CRC_L] [CRC_H]
//   3. Save:    TX: [0x50] [0x06] [0x00] [0x00] [0x00] [0x00] [CRC_L] [CRC_H]
//   Must complete unlock + write + save within 10 seconds (auto-lock timeout)
//
// CONFIGURATION SEQUENCE:
//
// Step 1: Change Modbus address from 0x50 to 0x02
//   Unlock: 50 06 00 69 B5 88 [CRC]
//   Write:  50 06 00 1A 00 02 [CRC]  (IICADDR = 0x02)
//   Save:   50 06 00 00 00 00 [CRC]
//   → From now on, sensor responds on address 0x02
//
// Step 2: Set 6-axis algorithm (disable magnetometer influence)
//   Unlock: 02 06 00 69 B5 88 [CRC]
//   Write:  02 06 00 24 00 01 [CRC]  (AXIS6 = 1, 6-axis mode)
//   Save:   02 06 00 00 00 00 [CRC]
//   → Magnetometer disabled for angle calculation (immune to steel/motors)
//
// Step 3: Set horizontal installation mode
//   Unlock: 02 06 00 69 B5 88 [CRC]
//   Write:  02 06 00 23 00 00 [CRC]  (ORIENT = 0, horizontal)
//   Save:   02 06 00 00 00 00 [CRC]
//
// Step 4: Set bandwidth to 20Hz (default, good balance for rotator)
//   Unlock: 02 06 00 69 B5 88 [CRC]
//   Write:  02 06 00 1F 00 04 [CRC]  (BANDWIDTH = 0x04 = 20Hz)
//   Save:   02 06 00 00 00 00 [CRC]
//
// Step 5: Reduce RS-485 response delay from 3000µs to 1000µs
//   Unlock: 02 06 00 69 B5 88 [CRC]
//   Write:  02 06 00 74 03 E8 [CRC]  (MODDELAY = 1000 µs)
//   Save:   02 06 00 00 00 00 [CRC]
//   → Faster bus turnaround, more time for Nano R4 queries
//
// Step 6: Turn off LED (save power, no visual disturbance)
//   Unlock: 02 06 00 69 B5 88 [CRC]
//   Write:  02 06 00 1B 00 01 [CRC]  (LEDOFF = 1)
//   Save:   02 06 00 00 00 00 [CRC]
//
// Step 7: Set angle reference (zero current position as reference)
//   Unlock: 02 06 00 69 B5 88 [CRC]
//   Write:  02 06 00 01 00 08 [CRC]  (CALSW = 0x08, set angle reference)
//   Save:   02 06 00 00 00 00 [CRC]
//   → Do this with dish at known 0° elevation
//
// VERIFICATION: Read back key registers
//   Read address:   02 03 00 1A 00 01 [CRC] → expect 0x0002
//   Read algorithm: 02 03 00 24 00 01 [CRC] → expect 0x0001
//   Read pitch:     02 03 00 3E 00 01 [CRC] → expect ~0 if level
//   Read version:   02 03 00 2E 00 01 [CRC] → firmware version
//
// OPTIONAL: Accelerometer calibration (for best accuracy)
//   Place sensor on level surface, then:
//   Unlock: 02 06 00 69 B5 88 [CRC]
//   Write:  02 06 00 01 00 01 [CRC]  (CALSW = 0x01, auto accel calibration)
//   Save:   02 06 00 00 00 00 [CRC]
//   Wait 5 seconds for calibration to complete.
//
// GitHub SDK: https://github.com/WITMOTION/WitStandardModbus_WT901C485
// Contains Arduino, ESP32, C++, Python examples for reference.
```

---

### 9d. HH-12 FALLBACK — AZIMUTH (SSI bit-bang, 5V with level shifter)

```cpp
// ═══════════════════════════════════════════════════════════════
// LEGACY/FALLBACK: HH-12 SSI 12-bit capacitive encoder
// Enabled by: #define ENABLE_AZ_HH12  1  (in config.h)
// Uses SAME GPIO pins as AS5048A: IO35 (CS), IO36 (CLK), IO37 (DATA)
// BUT requires TXS0108E level shifter (5V ↔ 3.3V) — NOT in current BOM!
// Add TXS0108E if using HH-12.
// ═══════════════════════════════════════════════════════════════

#define HH12_BITS            12       // 12-bit resolution
#define HH12_DEG_PER_BIT     (360.0 / 4096)  // 0.088 deg
#define HH12_CLK_HZ          500000   // 500 kHz SSI clock

// SSI protocol (different from SPI):
// 1. CS LOW (start conversion)
// 2. Wait 40 µs (conversion time)
// 3. Clock in 13 bits: 1 status + 12 data (MSB first)
//    Bit [12] = status (1 = valid)
//    Bits [11:0] = 12-bit angle (0 = 0°, 4095 = 359.912°)
// 4. CS HIGH
//
// Same pin mapping as AS5048A (IO35/IO36/IO37) but different protocol.
// The config.h compile-time check prevents enabling both simultaneously.
//
// Use case: Michaël may have a working HH-12 for initial AZ testing
// before AS5048A module arrives or is mounted.

// For ELEVATION HH-12 fallback (ENABLE_EL_HH12):
// Requires 3 additional GPIOs from reserve (e.g., IO39=CS, IO40=CLK, IO41=DATA)
// + TXS0108E level shifter
// This is a temporary solution until HWT901B is configured and tested.
```

---

## 10. GPS NEO-6M — TIME SOURCE AND PPS

```cpp
#define GPS_SERIAL          Serial2    // UART2 on ESP32-S3
#define GPS_BAUD            9600       // Default NEO-6M baud rate
#define GPS_RX_PIN          PIN_GPS_RX // IO5 ← GPS TX
#define GPS_TX_PIN          PIN_GPS_TX // IO6 → GPS RX

// ═══════════════════════════════════════════════════════════════
// GPS serves TWO purposes:
//   1. UTC time (±50 ns via PPS) for solar/lunar ephemeris
//   2. Position verification at startup (compare vs NVS stored coords)
//
// Station position is FIXED and stored in NVS:
//   Latitude:  50.41°N
//   Longitude:  3.87°E
//   Locator:   JO20BM85DP
// GPS position is NOT used for tracking — only for sanity check.
// ═══════════════════════════════════════════════════════════════

// === PPS INTERRUPT ===
// The PPS signal rising edge marks the exact UTC second boundary.
// NMEA sentences arrive ~200-500 ms AFTER the PPS pulse.
// Strategy:
//   1. PPS ISR: capture esp_timer_get_time() → pps_micros
//   2. NMEA parser: extract UTC time from $GPRMC sentence
//   3. Combine: NMEA gives date+hour+minute+second, PPS gives sub-second sync
//   4. Internal clock: micros since last PPS = sub-second offset
//
// ISR (keep minimal):
//   volatile uint64_t pps_utc_micros = 0;
//   volatile bool pps_flag = false;
//   void IRAM_ATTR gps_pps_isr() {
//     pps_utc_micros = esp_timer_get_time();
//     pps_flag = true;
//   }
//
// Time query (anywhere in firmware):
//   uint64_t now_micros = esp_timer_get_time();
//   double seconds_since_pps = (now_micros - pps_utc_micros) / 1000000.0;
//   double utc_seconds = gps_utc_second + seconds_since_pps;
//   // utc_seconds is precise to ~1 µs

// === NMEA PARSING ===
// Using TinyGPS++ library:
//   TinyGPSPlus gps;
//   while (GPS_SERIAL.available()) gps.encode(GPS_SERIAL.read());
//   if (gps.time.isUpdated()) {
//     utc_hour = gps.time.hour();
//     utc_minute = gps.time.minute();
//     utc_second = gps.time.second();
//     utc_year = gps.date.year();
//     utc_month = gps.date.month();
//     utc_day = gps.date.day();
//     fix_valid = gps.location.isValid();
//     sat_count = gps.satellites.value();
//   }

// === GPS STATUS ===
#define GPS_FIX_TIMEOUT_S   120    // Max time to wait for fix at boot (seconds)
#define GPS_MIN_SATELLITES   4      // Minimum satellites for valid fix
// If no fix after timeout: use last known time from EEPROM + warn on OLED.
// Solar calibration requires valid GPS fix (time accuracy critical).

// === UART2 INIT ===
// ESP32-S3 GPIO matrix allows any GPIO for UART2:
//   GPS_SERIAL.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
//   pinMode(PIN_GPS_PPS, INPUT);
//   attachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS), gps_pps_isr, RISING);
```

---

## 11. MOTOR CURRENT READING (ADC)

```cpp
#define ADC_ATTEN           ADC_ATTEN_DB_12    // Range 0-3.3V
#define ADC_BITWIDTH        ADC_BITWIDTH_12    // 12 bits (0-4095)
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

## 12. EEPROM STRUCTURE (double buffering)

```cpp
// Two alternating identical blocks (A and B)
// The block with the highest counter AND valid CRC is the most recent

#define EEPROM_BLOCK_A      0x0000
#define EEPROM_BLOCK_B      0x0020
#define EEPROM_BLOCK_SIZE   22     // bytes per block

typedef struct __attribute__((packed)) {
    float    pos_az_deg;     // Azimuth position in degrees (float32, 4 bytes)
    float    pos_el_deg;     // Elevation position in degrees (float32, 4 bytes)
    uint16_t enc_az_raw;     // AS5048A AZ raw value at storage time (14-bit)
    float    hwt901b_el;     // HWT901B elevation angle at storage time (for cross-check)
    uint32_t seq_counter;    // Sequential counter (incremented each write)
    uint16_t crc16;          // CRC16-CCITT over the preceding 20 bytes
} EepromPositionBlock;       // Total: 22 bytes

// Write moments:
// - At each motor stop (ramp finished)
// - Periodically during movement (every 5-10 seconds)
// - On position change > 0.01 deg
// - Before shutdown (if detected)
```

---

## 13. MODBUS RTU — RS-485 BUS (Nano R4 addr 1 + HWT901B addr 2)

```cpp
#define MODBUS_NANO_ADDR    1         // Nano R4 telemetry slave
#define MODBUS_HWT901B_ADDR 0x02      // HWT901B (reconfigured from factory 0x50)
#define MODBUS_BAUD         9600      // Shared baud rate (HWT901B default = 9600)
#define MODBUS_SERIAL       Serial1   // UART1 on ESP32-S3
#define MODBUS_POLL_MS      1000      // Polling cycle: Nano R4 then HWT901B

// NOTE: HWT901B factory default address is 0x50 — MUST be changed to 0x02
// before connecting to the shared bus (see section 9c for procedure).
// Baud rate 9600 is HWT901B default (register BAUD = 0x02).
// Nano R4 firmware must also be configured for 9600 baud.
// Alternative: reconfigure both to 19200 (HWT901B BAUD = 0x03).

// NOTE: HWT901B has a RS-485 response delay (MODDELAY register 0x74).
// Factory default = 3000 µs (3 ms). Recommended to reduce to 1000 µs.
// This delay is ADDED to normal Modbus turnaround time.

// --- Nano R4 registers (address 1, function 03) ---
// All registers are uint16 unless otherwise noted
#define REG_VOLTAGE_24V     0x0001  // Voltage 24V * 10 (ex: 240 = 24.0V)
#define REG_VOLTAGE_12V     0x0002  // Voltage 12V * 10
#define REG_PA_POWER_FWD    0x0003  // Forward power (mW)
#define REG_PA_POWER_REF    0x0004  // Reflected power (mW)
#define REG_TEMP_BOX        0x0010  // Box temp * 10 (int16, ex: 215 = 21.5 deg C)
#define REG_TEMP_PA         0x0011  // PA temp * 10
#define REG_TEMP_EXT        0x0012  // Outdoor temp * 10
#define REG_RELAY_WG        0x0020  // Waveguide relay (0/1)
#define REG_SEQ_PA          0x0021  // PA sequencer (0=RX, 1=TX)
#define REG_RELAY_HEAT      0x0022  // Anti-freeze lamp relay (0/1)
#define REG_NANO_STATUS     0x0030  // General status bitmap

// --- HWT901B registers (address 0x02, function 03) ---
// WitMotion standard Modbus register map (from official documentation):
#define HWT_REG_AX          0x34    // Accel X (int16, g = raw / 32768.0 * 16.0)
#define HWT_REG_AY          0x35    // Accel Y
#define HWT_REG_AZ          0x36    // Accel Z
#define HWT_REG_GX          0x37    // Gyro X (int16, dps = raw / 32768.0 * 2000.0)
#define HWT_REG_GY          0x38    // Gyro Y — EL ANGULAR VELOCITY for PID
#define HWT_REG_GZ          0x39    // Gyro Z
#define HWT_REG_HX          0x3A    // Mag X (int16, raw LSB)
#define HWT_REG_HY          0x3B    // Mag Y
#define HWT_REG_HZ          0x3C    // Mag Z
#define HWT_REG_ROLL        0x3D    // Roll  angle (int16, deg = raw / 32768.0 * 180.0)
#define HWT_REG_PITCH       0x3E    // Pitch angle = ELEVATION POSITION
#define HWT_REG_YAW         0x3F    // Yaw   angle (magnetic heading, NOT used)
#define HWT_REG_TEMP        0x40    // Temperature (int16, °C = raw / 100.0)

// --- HWT901B configuration registers (function 06, write single) ---
#define HWT_REG_SAVE        0x00    // Save=0x0000, Reboot=0x00FF, Reset=0x0001
#define HWT_REG_CALSW       0x01    // Calibration: 0x01=accel, 0x08=set angle ref
#define HWT_REG_BAUD        0x04    // Baud: 0x02=9600, 0x03=19200
#define HWT_REG_IICADDR     0x1A    // Device address (0x01-0x7F)
#define HWT_REG_BANDWIDTH   0x1F    // Filter BW: 0x04=20Hz, 0x03=42Hz
#define HWT_REG_ORIENT      0x23    // 0x00=horizontal, 0x01=vertical
#define HWT_REG_AXIS6       0x24    // 0x00=9-axis, 0x01=6-axis (no mag)
#define HWT_REG_KEY         0x69    // UNLOCK: must write 0xB588 before ANY write!
#define HWT_REG_MODDELAY    0x74    // RS-485 response delay (µs, default 3000)

// Elevation reading:
//   int16_t raw = modbus.readHoldingRegister(HWT901B_ADDR, HWT_REG_PITCH);
//   float elevation_deg = (float)raw / 32768.0f * 180.0f;
//
// Elevation velocity (for PID, replaces Hall PCNT):
//   int16_t raw_gy = modbus.readHoldingRegister(HWT901B_ADDR, HWT_REG_GY);
//   float el_velocity_dps = (float)raw_gy / 32768.0f * 2000.0f;
//
// Temperature:
//   int16_t raw_temp = modbus.readHoldingRegister(HWT901B_ADDR, HWT_REG_TEMP);
//   float temp_celsius = (float)raw_temp / 100.0f;
//
// Optimized: read registers 0x38 to 0x3E (7 regs) in one Modbus transaction.
//
// WRITE SEQUENCE (for runtime config changes):
//   1. Unlock: modbus.writeSingleRegister(addr, 0x69, 0xB588);
//   2. Write:  modbus.writeSingleRegister(addr, reg, value);
//   3. Save:   modbus.writeSingleRegister(addr, 0x00, 0x0000);
//   Must complete within 10 seconds (auto-lock timeout)!
//
// Polling sequence per cycle (1 Hz):
//   1. Read Nano R4 (addr 1): voltages, temps, PA, relays
//   2. Read HWT901B (addr 2): gyro Y + pitch → EL velocity + EL angle
//   Total ~25ms for both at 9600 baud (including 1ms MODDELAY per query)
```

---

## 14. PROTOCOL — Yaesu GS-232A/B (TCP port 4533)

```cpp
#define GS232_TCP_PORT      4533
#define GS232_LINE_TERM     "\r\n"

// Commands to implement:
// Rx: "C2\r\n"       -> Tx: "+0xxx+0yyy\r\n"  (AZ xxx deg, EL yyy deg)
// Rx: "Mxxx\r\n"     -> Point azimuth xxx deg (AZ movement only)
// Rx: "Wxxx yyy\r\n" -> Point AZ xxx deg and EL yyy deg
// Rx: "S\r\n"        -> Immediate stop all motors (deceleration ramp)
// Rx: "A\r\n"        -> Stop azimuth
// Rx: "E\r\n"        -> Stop elevation
// Rx: "R\r\n"        -> Continuous CW rotation
// Rx: "L\r\n"        -> Continuous CCW rotation
// Rx: "U\r\n"        -> Continuous elevation UP
// Rx: "D\r\n"        -> Continuous elevation DOWN

// C2 response format:
// Position AZ: 000-450 (overlap possible)
// Position EL: 000-180
// Example: "+0180+0045" = AZ 180 deg, EL 45 deg

// IMPORTANT: Positions reported on port 4533 and 4534 are CORRECTED positions.
// The solar calibration offsets (section 27) are applied BEFORE sending to clients.
// PSTRotator and the custom app always see TRUE azimuth/elevation.
// Internal encoder values are offset-corrected transparently.

// PSTRotator typically sends C2 every 200-500 ms
// Parser must be tolerant (ignore empty lines, spaces)
```

---

## 15. PROTOCOL — Windows Application (TCP port 4534)

```cpp
#define APP_TCP_PORT        4534

// Format: JSON over TCP, one line per message, terminated by \n
```

### 15.1 Automatic status push (2x/second)

```json
{
  "type": "status",
  "az": 180.05,  "el": 45.12,
  "az_target": 185.00, "el_target": 50.00,
  "az_raw": 179.85, "el_raw": 45.10,
  "state": "MOVING",
  "tracking_source": "gs232",
  "moving": true,
  "current_az": 0.85, "current_el": 1.20,
  "limit_cw": false, "limit_ccw": false,
  "limit_up": false, "limit_down": false,
  "stop_pressed": false,
  "mc_fault": false,
  "enc_az": 8192,
  "hwt901b_el": 45.12,
  "hwt901b_el_gyro": 0.02,
  "pcnt_az": 820000,
  "gps": {
    "fix": true, "satellites": 8,
    "utc": "2026-02-18T14:23:45Z"
  },
  "cal": {
    "az_offset": 1.35,
    "el_offset": -0.08,
    "points": 3,
    "last_cal": "2026-02-15T12:00:00Z"
  },
  "nano": {
    "v24": 24.1, "v12": 12.05,
    "pa_fwd": 850, "pa_ref": 12,
    "temp_box": 22.5, "temp_pa": 35.1, "temp_ext": 8.3,
    "wg_relay": 0, "sequencer": 0, "heater": 0
  }
}
```

### 15.2 Incoming commands (from app)

```json
{"cmd": "goto", "az": 185.0, "el": 50.0}
{"cmd": "stop"}
{"cmd": "track", "source": "azeldat", "az": 180.3, "el": 45.7}
{"cmd": "track", "source": "azeldat", "az": 180.3, "el": 45.7, "doppler": -1234, "body": "Moon"}
{"cmd": "jog", "dir": "cw"}
{"cmd": "jog_stop"}
{"cmd": "calibrate"}
{"cmd": "solar_cal_start"}
{"cmd": "solar_cal_point"}
{"cmd": "solar_cal_clear"}
{"cmd": "set_pid", "axis": "az", "kp": 2.0, "ki": 0.1, "kd": 0.05}
{"cmd": "set_limits", "az_min": 0, "az_max": 360, "el_min": 0, "el_max": 90}
{"cmd": "get_config"}
{"cmd": "reboot"}
```

### 15.3 AZELDAT autonomous tracking mode (replaces PSTRotator)

```cpp
// ═══════════════════════════════════════════════════════════════
// AUTONOMOUS TRACKING — WSJT-X azel.dat → Windows App → ESP32
// Enabled by: #define ENABLE_AZELDAT 1  (requires ENABLE_APP_TCP)
//
// WSJT-X writes a file called "azel.dat" in its config directory:
//   Windows: C:\Users\<user>\AppData\Local\WSJT-X\azel.dat
//   Linux:   ~/.local/share/WSJT-X/azel.dat
//
// File format (fixed-width text, updated every ~1 second by WSJT-X):
//   Line content includes: body name, azimuth, elevation, Doppler shift, etc.
//   Example: "Moon  180.3  45.7  -1234  ..."
//   Fields are space-separated, Moon line starts with "Moon".
//
// The Windows application:
//   1. Monitors azel.dat for changes (file watcher or polling every 1s)
//   2. Parses Moon azimuth and elevation from the "Moon" line
//   3. Sends tracking command to ESP32 via TCP port 4534:
//      {"cmd": "track", "source": "azeldat", "az": 180.3, "el": 45.7}
//   4. Optionally sends Doppler info for display:
//      {"cmd": "track", "source": "azeldat", "az": 180.3, "el": 45.7,
//       "doppler": -1234, "body": "Moon"}
//
// The ESP32:
//   - Receives "track" commands on port 4534
//   - Updates target AZ/EL (same as receiving W command on GS-232)
//   - State machine enters STATE_TRACKING
//   - If no "track" command received for 10 seconds → STATE_IDLE (timeout)
//
// ADVANTAGE over PSTRotator chain:
//   - Eliminates one software layer (PSTRotator)
//   - WSJT-X is already required for Q65 decoding
//   - Windows app can display more info (Doppler, Moon rise/set, etc.)
//   - Windows app can implement additional logic (avoid obstructions, etc.)
//   - Single application for monitoring AND control
//
// COEXISTENCE with GS-232:
//   When ENABLE_AZELDAT=1 AND ENABLE_GS232=1 (both active):
//   - Both TCP servers run simultaneously
//   - Last received target wins (either source)
//   - GS-232 W command overrides azeldat tracking
//   - azeldat "track" command overrides last GS-232 target
//   - Status push on port 4534 includes "tracking_source": "azeldat" or "gs232"
//   This allows PSTRotator to be used as backup/manual override.
// ═══════════════════════════════════════════════════════════════

#define AZELDAT_TIMEOUT_MS   10000  // Timeout: stop tracking if no update for 10s
```

---

## 16. PID CONTROL LOOP

```cpp
#define PID_LOOP_HZ         100    // 100 Hz = 10 ms per cycle
#define PID_DEADBAND_DEG    0.02   // Dead zone (no correction below 0.02 deg)
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

// PID output -> MCPWM duty cycle (with clamp 0-RAMP_MAX_DUTY)
// Acceleration ramp limits maximum duty variation per cycle

// IMPORTANT: PID works on CORRECTED positions.
// az_current = az_raw + cal_az_offset
// el_current = el_raw + cal_el_offset
// Error = target - corrected_position
// This ensures the dish points at the TRUE sky coordinates.
```

---

## 17. STATE MACHINE

```cpp
typedef enum {
    STATE_BOOT,          // Startup, hardware init
    STATE_CALIBRATING,   // Initial calibration (AS5048A read, EEPROM)
    STATE_IDLE,          // Ready, waiting for command
    STATE_MOVING,        // Moving toward target
    STATE_JOGGING,       // Manual movement (button or jog command)
    STATE_TRACKING,      // Continuous tracking (PSTRotator sends targets)
    STATE_DECELERATING,  // Deceleration ramp in progress
    STATE_SOLAR_CAL,     // Solar calibration scan in progress (section 27)
    STATE_FAULT,         // Error detected (overcurrent, SF, etc.)
    STATE_STOPPED,       // Emergency stop (STOP button)
} SystemState;

// Transitions:
// BOOT -> CALIBRATING -> IDLE
// IDLE -> MOVING (goto cmd) | JOGGING (button/jog) | TRACKING (C2 polling)
// IDLE -> SOLAR_CAL (solar_cal_start cmd) -> IDLE (scan complete or cancel)
// MOVING -> DECELERATING -> IDLE (target reached)
// JOGGING -> DECELERATING -> IDLE (button released)
// TRACKING -> IDLE (timeout without C2)
// * -> FAULT (overcurrent, SF, anomaly) -> IDLE (after correction)
// * -> STOPPED (hardware STOP button) -> IDLE (button released)
```

---

## 18. BOOT SEQUENCE (16 steps)

```
1. Init hardware: GPIO, SPI3 (W5500), I2C, UART1 (RS-485), UART2 (GPS), ADC
2. Init GPS: start UART2 at 9600, attach PPS interrupt on IO34
3. Init MCP23017: configure ports A/B, interrupts, pull-ups, directions to BRAKE
4. Init OLED: splash screen "ON7KGK EME Rotator"
5. Wait GPS fix: parse NMEA, wait for valid fix (timeout 120s)
   Display satellite count on OLED during wait.
   If timeout: warn "NO GPS FIX", use last EEPROM time (degraded mode).
6. Init W5500: DHCP or static IP, open TCP ports 4533 + 4534
7. Read AS5048A AZ (SPI, 14-bit): absolute AZ position (0.022 deg)
7b. Read HWT901B EL (Modbus addr 0x02): absolute EL position (0.05 deg)
    NOTE: HWT901B must have been configured to addr 0x02 before first use (see 9c)
8. Read EEPROM blocks A and B: choose most recent with valid CRC
9. Cross-validation:
   AZ: IF |as5048a_now - enc_stored| <= 1 step (0.022 deg):
      -> Fine AZ position restored from EEPROM
   ELSE:
      -> Degraded AZ (0.022 deg precision), flag "uncalibrated"
   EL: IF |hwt901b_now - hwt901b_stored| <= 0.1 deg:
      -> Fine EL position restored from EEPROM
   ELSE:
      -> Use HWT901B directly (0.05 deg, already good enough)
10. Load calibration offsets from NVS (cal_az_offset, cal_el_offset)
11. Init PCNT unit 0 (AZ only) with restored AZ position
12. Init MCPWM timers 0 and 1 (off, duty = 0)
13. Read limit switch state + STOP button (IO7) + SF via MCP23017
14. Idle current calibration (read FB for 1 second, motors off)
15. Start PID loop on Core 1 (xTaskCreatePinnedToCore)
16. Start Core 0 tasks: TCP servers, Modbus, GPS, OLED, Solar cal
17. Enable Core 1 watchdog
18. State -> IDLE
```

---

## 19. FREERTOS TASK ALLOCATION

| Task | Core | Priority | Period | Description |
|------|:----:|:--------:|:------:|-------------|
| task_pid_loop | 1 | configMAX-1 | 10 ms | PID + MCPWM + ADC FB + MCP direction write + anomaly |
| task_enc_read | 1 | configMAX-2 | 1000 ms | AS5048A AZ read (SPI) + PCNT AZ recalib + EEPROM |
| task_tcp_gs232 | 0 | 5 | Event | TCP server PSTRotator (GS-232) |
| task_tcp_app | 0 | 5 | Event | TCP server Windows app (JSON) |
| task_modbus | 0 | 4 | 1000 ms | Modbus: Nano R4 (addr 1) + HWT901B EL (addr 2, angle + gyro) |
| task_gps | 0 | 4 | 100 ms | GPS NMEA parsing (UART2) + PPS time sync |
| task_oled | 0 | 2 | 500 ms | OLED display update |
| ISR stop_btn | 1 | ISR | Event | STOP button interrupt (GPIO 7) |
| ISR mcp_int | 1 | ISR | Event | MCP23017 interrupt (GPIO 16) |
| ISR gps_pps | 1 | ISR | Event | GPS PPS interrupt (GPIO 0, 1 Hz) |

All Core 1 tasks (real-time) have higher priority than Core 0 tasks (communication).
ISRs must be short (set flag + xTaskNotify, no I2C in ISR).

Note: Motor direction is set via MCP23017 I2C write in task_pid_loop.
This takes ~50 us at 400 kHz — negligible in a 10 ms cycle.
Direction only changes at start of movement or reversal (not every cycle).

Note: Solar calibration scan (section 27) runs within task_pid_loop as a special
movement mode (STATE_SOLAR_CAL). The scan pattern and peak detection logic
execute in the PID loop context on Core 1.

---

## 20. SAFETY RULES (pseudocode)

```cpp
// --- Rule 1: STOP button (hardware + software) ---
// ISR IO7 (PIN_STOP_BUTTON):
//   stop_flag = true
//   // EN already cut by hardware (pull-up + NF contact)
//   // Software saves position and notifies

// --- Rule 2: Limit switches (SOFTWARE ONLY) ---
// ISR IO16 (PIN_MCP_INT, MCP23017 INT):
//   mcp_flag = true
//   xTaskNotifyGive(task_pid_loop)
//
// In task_pid_loop, after notification:
//   pins = mcp.readGPIOA()
//   if (pins & (1 << MCP_LIMIT_CW) == 0)  limit_flags |= LIMIT_CW
//   if (pins & (1 << MCP_LIMIT_CCW) == 0) limit_flags |= LIMIT_CCW
//   if (pins & (1 << MCP_LIMIT_UP) == 0)  limit_flags |= LIMIT_UP
//   if (pins & (1 << MCP_LIMIT_DOWN) == 0) limit_flags |= LIMIT_DOWN
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

// --- Rule 3: Overcurrent (stop affected motor) ---
// In task_pid_loop (100 Hz):
//   if (current_az > CURRENT_EMERGENCY) {
//     duty_az = 0; state = STATE_FAULT; log("AZ overcurrent")
//   }

// --- Rule 4: MC33926 Status Flag (via MCP23017 PB4) ---
// In task_pid_loop (polled via MCP23017 or on interrupt):
//   portb = mcp.readGPIOB()
//   if (!(portb & (1 << 4))) {
//     duty_az = 0; duty_el = 0; state = STATE_FAULT
//     log("MC33926 fault (SF LOW)")
//   }

// --- Rule 5: Mechanical decoupling ---
// AZ: If PCNT changes but AS5048A stationary for > 2 seconds
// AND current < CURRENT_IDLE * 1.5:
//   log("AZ mechanical decoupling detected")
//   state = STATE_FAULT
// EL: If motor current > idle BUT HWT901B angle AND gyro both stationary > 2s:
//   log("EL mechanical decoupling detected — motor spinning, dish not moving")
//   state = STATE_FAULT
// EL: If HWT901B gyro shows movement BUT motor current is zero:
//   log("EL external force or sensor error")
//   state = STATE_FAULT

// --- Rule 6: Watchdog ---
// Watchdog on Core 1, timeout 5 seconds
// Reset in task_pid_loop at each cycle
// If triggered: reboot ESP32, recover from EEPROM
```

---

## 21. BACKLASH COMPENSATION

```cpp
#define BACKLASH_DEFAULT_DEG  0.1    // Default value (datasheet)
#define BACKLASH_OVERSHOOT    1.2    // Overshoot factor (120% of backlash)

// Storage in NVS:
// backlash_az_deg (float): measured backlash for azimuth axis
// backlash_el_deg (float): measured backlash for elevation axis

// Automatic calibration:
// AZ: Move CW until stable → reverse CCW → count PCNT before AS5048A moves
// EL: Move UP until stable → reverse DOWN → measure HWT901B angle change delay
//     (HWT901B measures true dish angle, so backlash = motor reversal time before
//      HWT901B detects movement. Measured in degrees via gyro integration.)
// 4. backlash_deg = measured dead zone
// 5. Store in NVS

// Application during pointing:
// If direction change detected:
//   target_adj = target + (backlash * BACKLASH_OVERSHOOT * direction)
// Or: unidirectional approach (always CW for AZ final approach)
```

---

## 22. NVS PARAMETERS (persistent configuration)

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

// Calibration offsets (from solar calibration, section 27)
//   cal_az_offset (float) = 0.0   // Azimuth offset: true_az = raw_az + cal_az_offset
//   cal_el_offset (float) = 0.0   // Elevation offset: true_el = raw_el + cal_el_offset
//   cal_base_tilt (float) = 0.0   // Base tilt magnitude (degrees, optional model)
//   cal_base_tilt_dir (float) = 0.0  // Base tilt direction (degrees, optional model)
//   cal_point_count (uint8) = 0   // Number of calibration points used
//   cal_timestamp (uint32) = 0    // Unix timestamp of last calibration

// Station
//   callsign (str)    = "ON7KGK"
//   locator (str)     = "JO20BM85DP"
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

## 23. SENSOR FUSION AND PRECISION

| Source | Resolution | Limitation | Role |
|--------|-----------|------------|------|
| AS5048A AZ | 0.022 deg | 14 bits, steel distortion ~0.5° | AZ absolute reference |
| HWT901B EL angle | 0.05 deg static | Gravity-based, 0.1° dynamic | EL absolute position |
| HWT901B EL gyro | ~0.01 deg/s | Drift over time (Kalman corrected) | EL angular velocity for PID |
| Hall PCNT AZ | 0.0002 deg | Cumulative drift | AZ real-time precision |
| AS5048A + PCNT fusion (AZ) | ~0.001 deg | Periodic recalibration | Optimal AZ position |
| HWT901B standalone (EL) | 0.05 deg | No PCNT fusion needed | EL position (immune to backlash) |
| Mechanical backlash | n/a | ~0.1 deg | Physical limit |
| **Effective precision AZ** | **~0.01 deg** | **Backlash compensated** | **Target** |
| **Effective precision EL** | **~0.05 deg** | **True dish angle, no backlash** | **Target** |

Precision 0.05 deg (EL) and 0.01 deg (AZ) are more than sufficient for 10 GHz EME
(3dB beamwidth of a 90cm dish = approx 2.5 deg, 1.8m dish = approx 1.2 deg).

### Fusion algorithm
1. At startup: AS5048A provides absolute AZ position (0.022 deg resolution)
2. At startup: HWT901B provides absolute EL position (0.05 deg, gravity-based)
3. EEPROM provides fine AZ position from last session (if CRC valid and AS5048A matches)
4. AZ running: PCNT provides continuous high-resolution AZ tracking (0.0002 deg)
5. Every 1 second: AS5048A re-read, PCNT AZ recalibrated to match
6. EL running: HWT901B polled every 1s → direct EL position (no PCNT fusion needed)
7. EL PID uses HWT901B gyroscope for angular velocity (derivative term)
8. AZ discrepancy PCNT vs AS5048A → anomaly detection (mechanical decoupling)
9. EL anomaly: motor current vs HWT901B gyro/angle mismatch → decoupling detection
10. HWT901B measures TRUE dish angle → inherently immune to EL backlash
11. Solar calibration offsets (NVS) applied to both axes before reporting to clients

---

## 24. ANOMALY DETECTION — TRIPLE REDUNDANCY

### 24.1 Electrical (MC33926 FB + SF)
- No-load current: ~200 mA -> ~0.10V on FB (reference)
- 90cm dish: 400-800 mA -> 0.2-0.4V (normal)
- 1.8m dish: 800-1500 mA -> 0.4-0.8V (normal)
- Full load: 1.9A -> ~1.0V (alert)
- Stall: >2.5A -> >1.3V (emergency stop)
- SF pin LOW (via MCP23017 PB4): hardware overtemp or overcurrent

### 24.2 Mechanical (sensor cross-checks)
- AZ: PCNT counting + AS5048A stationary → mechanical decoupling
- AZ: PCNT stationary + AS5048A moving → Hall/PCNT problem
- EL: Motor current > idle + HWT901B angle/gyro stationary → mechanical decoupling
- EL: HWT901B gyro detects movement + motor current zero → external force or sensor error
- Progressive drift -> wear, slipping

### 24.3 Software (current profile)
After calibration: "normal" current profile known per axis/direction.
Elevation draws more than azimuth (gravity).
Significant deviation from expected profile -> alert.

---

## 25. CAT6 CABLE WIRING

Note: ESP32 is co-located at the rotator. Encoder and motor cables are SHORT runs.
Only the Ethernet cable runs to the shack (long distance).
Nano R4 is at the parabola arm (2-3m from ESP32), NOT at the shack.

### 25.1 AS5048A Azimut (1x dedicated CAT6, short run ~2m max)
- Pair 1: CLK (IO36) + GND
- Pair 2: MISO (IO37) + GND
- Pair 3: CS (IO35) + GND
- Pair 4: +3.3V + +3.3V
- 100nF decoupling at AS5048A end
- NOTE: 3.3V native, NO level shifter needed
- AZIMUTH ONLY — elevation uses HWT901B on RS-485 bus (section 25.6)

### 25.2 Halls + Motor AZ (1x dedicated cable, short run)
- Pair 1: Hall A + GND
- Pair 2: Hall B + GND
- Pair 3: Hall +5V + GND
- Pair 4: Motor+ + Motor-
- AWG23-24 OK for 1.9A on short runs.

### 25.3 Motor EL (1x dedicated cable, short run)
- Pair 1: Motor+ + Motor-
- AWG23-24 OK for 1.9A on short runs.
- NOTE: SVH3 EL motor has built-in Hall sensors but they are NOT connected to ESP32.
  EL position/velocity is provided by HWT901B inclinometer instead.

### 25.4 Ethernet to shack (1x CAT6, up to 100m)
- Standard Ethernet (W5500 to switch/router)
- Carries all control traffic (PSTRotator + custom app)

### 25.5 RS-485 bus to parabola arm (1x twisted pair, 2-3m)
- 2 wires: A + B (in shielded cable with power)
- Bus carries: Nano R4 (addr 1) + HWT901B (addr 2)
- HWT901B powered by 5V at parabola arm (from Nano R4 regulator or dedicated)
- Short distance: RS-485 provides noise immunity near motors and transverter

### 25.6 GPS antenna cable
- GPS NEO-6M module inside rotator weatherproof enclosure
- Active ceramic patch antenna on exterior of enclosure (sky view required)
- Or: antenna on short cable (10-20 cm) mounted on top of enclosure lid
- Keep GPS antenna away from 10 GHz equipment to avoid desense

---

## 26. BOM SUMMARY

### At rotator — Main controller
1. ProS3 ESP32-S3 (1)
2. Pololu Dual MC33926 (1)
3. W5500 Ethernet module (1)
4. VMA452 Optocoupler 4-channel (2: 1× limit switches 24V, 1× hall AZ 5V)
5. 74HC14 Schmitt Trigger 3.3V (1)
6. EEPROM/FRAM I2C AT24C256 or FM24C64 (1)
7. MCP23017 I2C GPIO expander (1)
8. OLED I2C 128x64 (1)
9. AS5048A magnetic encoder module 14-bit 3.3V (1, AZ only)
10. Diametrically magnetized neodymium magnet 6×3mm N42+ (1, AZ only)
11. RS-485 module MAX485/MAX3485 (1)
12. GPS u-blox NEO-6M module with active antenna (1)
13. NF STOP button + waterproof enclosure (1)
14. Pushbuttons CW/CCW/UP/DOWN (4)
15. Siemens 3RG4013-0AG33 inductive PNP NO 10-30V (4)
16. 1.5 kohm 1/4W resistors for optos (4)
17. Metal target plates for limit switches (4)
18. Weatherproof enclosure (1)
19. 3D printed bracket for AS5048A AZ mounting (1)

### At parabola arm — Monitoring module + EL sensor (2-3m from ESP32)
20. Arduino Nano R4 (1)
21. RS-485 module MAX485/MAX3485 (1, for Nano R4)
22. WitMotion HWT901B-RS485 (1, elevation inclinometer, Modbus addr 2)
23. DS18B20 OneWire temperature sensors (3+)
24. 5V relay module (1)
25. Incandescent bulb 25-40W anti-freeze (1)
26. ADC voltage divider resistors (2x)

### Wiring and passives
27. CAT6 cable short (2: 1× AS5048A AZ, 1× hall+motor AZ)
28. CAT6 cable long to shack (1: Ethernet)
29. Motor EL cable short (1: motor wires only, no hall connection)
30. Twisted pair + power cable to parabola arm (1: RS-485 bus, 2-3m)
31. 24VDC power cable to shack (1)
32. 100nF ceramic capacitors (8+)
33. 100uF electrolytic capacitor (1)
34. 10kohm pull-up resistors (2: STOP + EN)
35. 24V >5A power supply (1, at shack)
36. 5V >1A regulator at rotator (1)

### REMOVED from previous BOMs (no longer needed)
- ~~TXS0108E Level Shifter 3.3V to 5V (were 2, for HH-12 5V encoders)~~
- ~~HH-12 SSI 12-bit 5V encoders (were 2, replaced v6→v7)~~
- ~~AS5048A EL (was 1, replaced by HWT901B in v7.1)~~
- ~~DFRobot SEN0386 / WT61PC (was optional, replaced by HWT901B-RS485)~~
- ~~Hall encoder EL connection to ESP32 (was IO5/IO6, removed in v7.2)~~

---

## 27. SOLAR RF CALIBRATION — AZIMUTH ALIGNMENT

### 27.1 Purpose and principle

The solar calibration procedure determines the azimuth mounting offset (true north vs
encoder zero) by using the Sun as a 10 GHz RF reference source.

The Sun is an extremely strong broadband noise source at 10 GHz (~10,000 SFU quiet sun,
much more during solar activity). The 10 GHz receiver on the parabola will detect a
clear signal increase when the dish points at the Sun.

**What the calibration corrects:**
- **Azimuth offset** (primary): Rotation of the mount relative to true north.
  The AS5048A encoder reads 0° at an arbitrary mechanical position.
  Solar calibration maps encoder-zero to true-north.
- **Elevation offset** (minor verification): The HWT901B is gravity-referenced,
  so EL offset should be near zero. Solar calibration verifies this.
  Any non-zero EL offset indicates HWT901B mounting misalignment.
- **Optional: Base tilt model** (advanced): If the azimuth axis isn't perfectly vertical,
  azimuth error varies with elevation. With 3+ calibration points at different
  elevations, this tilt can be characterized and compensated.

**What the calibration does NOT correct:**
- Errors that change over time (requires periodic recalibration)
- Non-linear encoder errors (these are handled by AS5048A+PCNT fusion)
- Backlash (handled separately in section 21)

### 27.2 Why calibration is valid year-round

The offsets corrected are **properties of the mechanical installation**, not the sky.
The mount's orientation relative to true north doesn't change with seasons.
Therefore:
- A calibration done in winter is valid in summer (and vice versa).
- Recalibrate only if: the mount is physically disturbed, settling occurs,
  or periodically (e.g., every 3-6 months) to detect drift.

The Sun's path changes with seasons (declination varies ±23.4°), but this is
fully accounted for in the solar position algorithm. Any day of the year works.

### 27.3 Solar position algorithm

Solar ephemeris computed from GPS UTC time and station coordinates (NVS).
Algorithm: Jean Meeus, "Astronomical Algorithms" (simplified for 0.01° accuracy).

```cpp
// Input: UTC date/time (from GPS), latitude/longitude (from NVS)
// Output: sun_az (0-360°, from north clockwise), sun_el (0-90°)
//
// Key steps:
// 1. Julian Day Number from UTC date
// 2. Solar mean longitude and mean anomaly
// 3. Equation of center → ecliptic longitude
// 4. Obliquity of ecliptic
// 5. Right Ascension and Declination
// 6. Greenwich Hour Angle → Local Hour Angle
// 7. Altitude (elevation) and Azimuth via spherical trigonometry
//
// Atmospheric refraction correction (optional):
// refraction_deg = 1.02 / tan(radians(el + 10.3/(el + 5.11))) / 60.0
// At 10 GHz: multiply by 0.8 (radio refraction ≈ 80% of optical)
// Significant only below 10° elevation.
//
// Library option: SolarPosition (Arduino lib) or custom implementation.
// Both give <0.01° accuracy, well within our needs.

typedef struct {
    float azimuth;    // degrees, 0=North, 90=East, 180=South, 270=West
    float elevation;  // degrees above horizon
} SunPosition;

SunPosition computeSunPosition(
    int year, int month, int day,
    int hour, int minute, int second,
    float latitude, float longitude
);
```

### 27.4 Multi-point calibration procedure

The procedure collects 3 calibration points at different times of day, providing
good azimuth coverage. Each point measures the offset between theoretical sun
position and actual encoder reading at peak RF signal.

**Prerequisites:**
- GPS fix valid (UTC time accurate)
- 10 GHz receiver active (preamp ON, waveguide relay to RX)
- Sun above horizon (elevation > 5°) — check before starting
- No obstructions between dish and sun
- PA transmitter OFF (safety!)

**Manual procedure (via Windows app or buttons):**

```
CALIBRATION POINT ACQUISITION:

1. Operator initiates "solar_cal_start" (via app or button sequence)
2. ESP32 computes current sun position: (sun_az_theo, sun_el_theo)
3. ESP32 slews dish to theoretical sun position (coarse pointing)
4. AUTOMATIC SCAN (STATE_SOLAR_CAL):
   a. Perform azimuth scan: sweep ±5° around theoretical position
      at constant elevation, in 0.1° steps
   b. At each step: pause 2 seconds, read PA_POWER_FWD from Nano R4
      (or dedicated 10 GHz power detector if available)
   c. Record azimuth with maximum received power → peak_az_encoder
   d. Perform elevation scan: sweep ±3° around theoretical position
      at peak azimuth, in 0.1° steps
   e. Record elevation with maximum received power → peak_el_encoder
5. Compute offsets for this point:
   delta_az = sun_az_theo - peak_az_encoder
   delta_el = sun_el_theo - peak_el_encoder (should be ~0)
6. Store calibration point in RAM:
   cal_points[n] = { time, sun_az_theo, sun_el_theo, peak_az, peak_el, delta_az, delta_el }
7. Operator repeats at different times of day (morning, noon, evening)
8. After 3 points: "solar_cal_finish" computes final offsets
```

**Recommended timing for 3 calibration points (JO20BM, Belgium):**

| Point | Time (approx) | Sun azimuth | Sun elevation | Coverage |
|-------|---------------|-------------|---------------|----------|
| Morning | 09:00-10:00 | ~120-140° (ESE) | 15-30° | East sector |
| Noon | 12:00-13:00 | ~180° (South) | 25-60° (season) | South sector |
| Afternoon | 15:00-16:00 | ~220-240° (WSW) | 15-30° | West sector |

These three points span approximately 120° of azimuth, providing excellent
coverage for offset determination and tilt model fitting.

### 27.5 Offset computation from calibration points

```cpp
// === SIMPLE MODEL (recommended): Constant AZ offset ===
// With N calibration points:
//   cal_az_offset = mean(delta_az[0..N-1])
//   cal_el_offset = mean(delta_el[0..N-1])
// Standard deviation of delta_az values indicates calibration quality.
// If stdev(delta_az) < 0.5°: simple offset model is sufficient.
// If stdev(delta_az) > 0.5°: base tilt model may be needed.

// === ADVANCED MODEL (optional): AZ offset + base tilt ===
// If azimuth axis isn't perfectly vertical, error varies with pointing:
//   az_error(az, el) = az_offset + tilt_mag * tan(el) * sin(az - tilt_dir)
//
// With 3 calibration points, solve for 3 unknowns:
//   az_offset  = constant azimuth offset (rotation from true north)
//   tilt_mag   = base tilt magnitude (degrees)
//   tilt_dir   = base tilt direction (degrees from north)
//
// Least-squares fit:
//   For each point i:
//     delta_az[i] = az_offset + tilt_mag * tan(el_theo[i]) * sin(az_theo[i] - tilt_dir)
//   Solve iteratively (Newton-Raphson) or linearize for small tilt.
//
// For Michaël's application: start with simple model. If pointing accuracy
// is insufficient at high elevation, add tilt model.

// === ELEVATION VERIFICATION ===
// The HWT901B is gravity-referenced, so delta_el SHOULD be near zero.
// Expected: |mean(delta_el)| < 0.2° (mounting alignment tolerance)
// If > 0.5°: HWT901B mounting is significantly misaligned → physical fix needed.
// The small residual cal_el_offset compensates any remaining misalignment.
//
// NOTE: Atmospheric refraction at 10 GHz causes the Sun to appear slightly
// higher than its true position, especially at low elevation:
//   ~0.15° at 10° elevation
//   ~0.06° at 20° elevation
//   ~0.02° at 45° elevation
// This is within the beamwidth and can be ignored for practical purposes.
// For maximum accuracy, apply radio refraction correction to sun_el_theo.

typedef struct {
    float timestamp;        // Unix time of measurement
    float sun_az_theo;      // Theoretical sun azimuth at measurement time
    float sun_el_theo;      // Theoretical sun elevation at measurement time
    float peak_az_encoder;  // Encoder azimuth at RF peak
    float peak_el_sensor;   // HWT901B elevation at RF peak
    float delta_az;         // sun_az_theo - peak_az_encoder
    float delta_el;         // sun_el_theo - peak_el_sensor
    float peak_power;       // RF power at peak (for quality assessment)
} SolarCalPoint;

#define SOLAR_CAL_MAX_POINTS  6    // Max stored calibration points
```

### 27.6 Applying calibration offsets

```cpp
// Offsets are applied TRANSPARENTLY to all position reports.
// Internal encoder values are raw; all external interfaces see corrected values.

// Corrected position (reported to PSTRotator on port 4533 and app on port 4534):
//   az_corrected = az_raw + cal_az_offset
//   el_corrected = el_raw + cal_el_offset
//
// If advanced tilt model is enabled:
//   az_corrected = az_raw + cal_az_offset
//                + cal_base_tilt * tan(el_raw) * sin(az_raw - cal_base_tilt_dir)
//   el_corrected = el_raw + cal_el_offset
//
// Incoming targets (from PSTRotator W command) are in TRUE coordinates.
// They must be REVERSE-corrected to encoder coordinates for PID:
//   az_target_raw = az_target_true - cal_az_offset
//   el_target_raw = el_target_true - cal_el_offset
//
// This way:
//   - PSTRotator sends "W180 045" (point south, 45° up)
//   - ESP32 converts to raw encoder target
//   - PID moves dish to raw position
//   - C2 response reports corrected position back → PSTRotator sees 180/045
//   - The dish is physically pointing at true south, 45° elevation

// NVS storage:
//   nvs_set_float("rotator", "cal_az_offset", cal_az_offset);
//   nvs_set_float("rotator", "cal_el_offset", cal_el_offset);
//   nvs_set_u8("rotator", "cal_point_count", n_points);
//   nvs_set_u32("rotator", "cal_timestamp", unix_time);
```

### 27.7 RF power measurement for solar scan

```cpp
// Option A: Use PA forward power detector (Nano R4 REG_PA_POWER_FWD)
//   The directional coupler on the PA measures forward power.
//   In RX mode (PA off), the coupled port sees the incoming RF noise.
//   The sun noise is strong enough at 10 GHz to produce a measurable
//   increase on the detector, especially through a 90cm+ dish.
//   Advantage: no extra hardware, uses existing Modbus telemetry.
//   Disadvantage: depends on coupler sensitivity in reverse direction.

// Option B: Dedicated 10 GHz power detector (future expansion)
//   A simple Schottky diode detector on the IF output of the transverter
//   would give a better signal. Connected to Nano R4 ADC or ESP32 reserve GPIO.
//   Could use one of the 5 reserve GPIOs (IO0, IO39-42) with ADC.

// Option C: Use SDR (LimeSDR on NUC) to measure band power
//   Advantage: most sensitive measurement
//   Disadvantage: requires NUC + software coordination

// For initial implementation: use Option A (Nano R4 power detector).
// The scan routine reads REG_PA_POWER_FWD via Modbus after each step.
// Sun noise at 10 GHz through a 90cm dish gives ~3-6 dB increase
// over cold sky background — clearly detectable.

#define SOLAR_SCAN_AZ_RANGE    5.0    // ±5° scan range around theoretical
#define SOLAR_SCAN_EL_RANGE    3.0    // ±3° scan range around theoretical
#define SOLAR_SCAN_STEP_DEG    0.1    // Step size (< half beamwidth)
#define SOLAR_SCAN_DWELL_MS    2000   // Dwell time per step (ms)
#define SOLAR_SCAN_MIN_SNR_DB  2.0    // Minimum SNR to validate peak (dB)
```

---

## 28. CLAUDE CODE — PROJECT BRIEFING

This section provides context for AI-assisted firmware development (Claude Code, VS Code).

### 28.1 Project summary

Dual-axis antenna rotator controller for amateur radio EME (Earth-Moon-Earth) communication
at 10 GHz. Station ON7KGK, locator JO20BM85DP, Belgium.

The system tracks the Moon automatically via PSTRotator software and allows manual fine-tuning.
Two Coresun SVH3 slewing drives (34,224:1 ratio) move a parabolic dish in azimuth and elevation.

### 28.2 Architecture overview

```
ESP32-S3 ProS3 (main controller, at rotator base)
  ├── MC33926 dual motor driver (24V DC motors, PWM via MCPWM)
  ├── 1× AS5048A magnetic encoder AZ (14-bit absolute, SPI, 3.3V native)
  ├── 1× Hall 12 PPR incremental encoder AZ (quadrature → PCNT hardware)
  ├── GPS NEO-6M (UART2: NMEA + PPS on GPIO interrupt → UTC time)
  ├── MCP23017 I2C expander (limits, buttons, motor dir, LEDs, SF)
  ├── W5500 Ethernet (SPI3, TCP servers: GS-232 port 4533 + JSON port 4534)
  ├── OLED SSD1306 I2C display
  ├── EEPROM/FRAM I2C (position storage, double buffered)
  └── RS-485 UART1 → bus to parabola arm (2-3m):
      ├── Arduino Nano R4 Modbus addr 1 (telemetry + thermostat)
      └── WitMotion HWT901B-RS485 Modbus addr 2 (EL angle + gyro velocity)
```

### 28.3 Key hardware decisions to respect

1. **Hybrid absolute position**: AS5048A (SPI, 14-bit) for AZ, HWT901B-RS485 (inclinometer) for EL.
   AS5048A: pipelined SPI read, 3.3V native, mounted on AZ slewing drive central bore.
   HWT901B: gravity-based, 0.05° static, RS-485 Modbus addr 2, mounted at end of dish arm.
   HWT901B gyroscope provides EL angular velocity for PID (replaces Hall encoder PCNT).
2. **Motor directions** are on MCP23017 Port B (I2C), NOT direct GPIO.
   One I2C write ~50µs at 400 kHz, negligible in 10 ms PID cycle.
3. **Limit switches are SOFTWARE ONLY** (not wired to EN). Directional: block movement
   toward limit, allow retreat in opposite direction.
4. **STOP button is HARDWARE** (NF in series with MC33926 EN). Always works.
5. **Nano R4 is at the parabola arm** (2-3m from ESP32), NOT at the shack.
   Connected via RS-485 Modbus addr 1. Monitors PA, temperatures, voltages.
6. **HWT901B shares RS-485 bus** with Nano R4 (Modbus addr 2). No extra GPIO needed.
7. **Hall encoder AZ only** — EL does NOT use hall encoder. HWT901B provides both
   EL position (accelerometer) and EL velocity (gyroscope). Saves 2 GPIO + PCNT unit.
8. **GPS NEO-6M** on UART2 (IO5 RX, IO6 TX) + PPS on IO34. Provides UTC time for
   solar/lunar ephemeris. Station position is fixed in NVS (JO20BM85DP).
9. **Solar calibration** uses Sun as 10 GHz RF reference to determine AZ mounting offset.
   Offsets stored in NVS, applied transparently to all position reports.
10. **5 reserve GPIOs**: IO0, IO39, IO40, IO41, IO42.

### 28.4 Development framework

- **PlatformIO** with Arduino framework + ESP-IDF APIs
- Board: `um_pros3` (Unexpected Maker ProS3)
- Build flags: `-DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=1`
- FreeRTOS dual-core: Core 1 = real-time (PID, encoders), Core 0 = comms (TCP, Modbus, OLED)
- ESP-IDF APIs: MCPWM (motor PWM), PCNT (AZ pulse counting), ADC oneshot

### 28.5 Firmware modules (development order)

```
Step 1:  GPIO init + MCP23017 + LEDs + buttons + STOP button
Step 2:  MC33926 motor control (MCPWM + direction via MCP23017 + current ADC)
Step 3:  AS5048A AZ encoder reading (SPI bit-bang, azimuth only)
Step 4:  PCNT Hall encoder AZ setup (quadrature 4x decoding, AZ only — no EL PCNT)
Step 5:  GPS NEO-6M (UART2 NMEA parsing + PPS interrupt + UTC clock)
Step 6:  Modbus RTU master (RS-485: Nano R4 addr 1 + HWT901B EL addr 2)
Step 7:  Sensor fusion (AS5048A + PCNT for AZ, HWT901B angle + gyro for EL, EEPROM)
Step 8:  PID control loop (10 ms, AZ uses PCNT velocity, EL uses HWT901B gyro velocity)
Step 9:  W5500 Ethernet + GS-232 TCP server (PSTRotator compatibility)
Step 10: JSON TCP server (Windows monitoring app)
Step 11: Solar ephemeris calculator (Jean Meeus algorithm, GPS time input)
Step 12: Solar RF calibration scan (AZ/EL scan, peak detection, offset computation)
Step 13: OLED display
Step 14: Safety rules + anomaly detection + watchdog
Step 15: NVS persistent configuration
```

Each module should be independently testable with `#define ENABLE_xxx` switches.

### 28.5b Feature flags — config.h (incremental development)

```cpp
// ═══════════════════════════════════════════════════════════════
// FEATURE FLAGS — config.h
// Enable/disable modules for incremental development and testing.
// Only enable a feature AFTER the previous ones are tested and working.
// ═══════════════════════════════════════════════════════════════

// ─── CURRENT STATUS (as of v7.4 development) ───
// ✅ TESTED & WORKING:
//   - ESP32-S3 ProS3 boots and runs
//   - W5500 Ethernet (Adafruit 3201) connects to network
//   - GS-232 TCP server on port 4533 dialogues with PSTRotator
//   - Positions are currently SIMULATED (hardcoded/random values)
//
// ⏳ NEXT TO TEST: MCP23017, then motor control, then encoders...

// ═══════ CORE (always enabled) ═══════
// These are the foundation — serial console, state machine, basic GPIO.
// No #define needed, always compiled.

// ═══════ ETHERNET + PROTOCOLS ═══════
#define ENABLE_ETHERNET       1   // W5500 Ethernet (SPI3) — ✅ TESTED
#define ENABLE_GS232          1   // GS-232 TCP server port 4533 — ✅ TESTED
#define ENABLE_APP_TCP        0   // JSON TCP server port 4534 (custom Windows app)
                                  // Enables: status push 2x/s, incoming JSON commands,
                                  // runtime config, solar cal control, telemetry display

// ═══════ TRACKING SOURCE — choose tracking input ═══════
#define ENABLE_PSTROTATOR     1   // PSTRotator sends targets via GS-232 (port 4533)
                                  // This is the DEFAULT tracking source.
#define ENABLE_AZELDAT        0   // Autonomous mode: Windows app reads WSJT-X azel.dat
                                  // and sends Moon AZ/EL targets via JSON (port 4534).
                                  // Eliminates PSTRotator from the chain entirely.
                                  // Requires: ENABLE_APP_TCP
                                  // See section 15.3 for azel.dat protocol details.
                                  //
                                  // With ENABLE_AZELDAT:
                                  //   WSJT-X → azel.dat → Windows app → TCP 4534 → ESP32
                                  // Without (classic chain):
                                  //   WSJT-X → PSTRotator → TCP 4533 → ESP32
                                  //
                                  // Both can coexist: GS-232 still accepts manual overrides
                                  // even when azel.dat tracking is active.
                                  // Priority: last received target wins (either source).

// ═══════ I/O EXPANDER ═══════
#define ENABLE_MCP23017       0   // MCP23017 I2C expander
                                  // When 0: no buttons, no limit switches, no LEDs,
                                  // no motor direction control (motors disabled)
                                  // When 1: enables buttons, limits, LEDs, motor direction

// ═══════ MOTORS ═══════
#define ENABLE_MOTORS         0   // MC33926 motor control (MCPWM + ADC current)
                                  // Requires: ENABLE_MCP23017 (for direction IN1/IN2)
                                  // When 0: PWM outputs disabled, no motor movement
                                  // When 1: MCPWM init, current reading, ramps

#define ENABLE_PID            0   // PID control loop (10 ms, Core 1)
                                  // Requires: ENABLE_MOTORS + at least one encoder
                                  // When 0: motors only via manual jog (open loop)
                                  // When 1: closed-loop PID positioning

// ═══════ AZ ENCODER — choose ONE ═══════
#define ENABLE_AZ_AS5048A     0   // AS5048A SPI 14-bit magnetic encoder (target hardware)
#define ENABLE_AZ_HH12        0   // HH-12 SSI 12-bit capacitive encoder (legacy/fallback)
                                  // IMPORTANT: Enable only ONE of these at a time!
                                  // Both provide absolute AZ position, different protocols.
                                  // HH-12 uses same pins (IO35=CS, IO36=CLK, IO37=DATA)
                                  // but SSI protocol instead of SPI.

#define ENABLE_AZ_HALL_PCNT   0   // Hall 12 PPR quadrature → PCNT hardware counter
                                  // Requires: at least one AZ absolute encoder above
                                  // When 0: AZ position from absolute encoder only (0.022°)
                                  // When 1: PCNT provides fine AZ tracking (0.0002°)

// ═══════ EL SENSOR — choose ONE ═══════
#define ENABLE_EL_HWT901B     0   // HWT901B-RS485 inclinometer (target hardware)
                                  // Uses RS-485 Modbus bus (shared with Nano R4)
#define ENABLE_EL_HH12        0   // HH-12 SSI 12-bit for elevation (legacy/fallback)
                                  // Would need dedicated GPIO pins (IO39=CS, IO40=CLK, IO41=DATA)
                                  // IMPORTANT: Enable only ONE of these at a time!

// ═══════ RS-485 / MODBUS ═══════
#define ENABLE_RS485          0   // RS-485 Modbus master (UART1)
                                  // When 0: no Modbus communication at all
#define ENABLE_NANO_R4        0   // Nano R4 telemetry (Modbus addr 1)
                                  // Requires: ENABLE_RS485
#define ENABLE_HWT901B_BUS    0   // HWT901B on Modbus bus (addr 2)
                                  // Requires: ENABLE_RS485 + ENABLE_EL_HWT901B

// ═══════ GPS ═══════
#define ENABLE_GPS            0   // GPS NEO-6M (UART2 + PPS)
                                  // When 0: time from NTP or manual, position from NVS
                                  // When 1: UTC from GPS, PPS sync, satellite info

// ═══════ CALIBRATION ═══════
#define ENABLE_SOLAR_CAL      0   // Solar RF calibration (section 27)
                                  // Requires: ENABLE_GPS + ENABLE_MOTORS + encoder(s)
                                  // When 0: cal offsets loaded from NVS but no scan capability
#define ENABLE_CAL_OFFSETS    0   // Apply calibration offsets to position reports
                                  // Can be enabled independently of SOLAR_CAL
                                  // (offsets can be entered manually via app or NVS)

// ═══════ DISPLAY ═══════
#define ENABLE_OLED           0   // OLED SSD1306 I2C display
                                  // When 0: no display output (saves I2C bandwidth)
                                  // When 1: position, target, state, GPS on OLED

// ═══════ STORAGE ═══════
#define ENABLE_EEPROM         0   // EEPROM/FRAM position storage (double buffered)
                                  // When 0: position lost at reboot (re-read from encoders)
                                  // When 1: fine position preserved across reboots
#define ENABLE_NVS_CONFIG     0   // NVS persistent configuration (PID, limits, cal offsets)
                                  // When 0: all config values are compile-time defaults
                                  // When 1: config stored in NVS, adjustable at runtime

// ═══════ SAFETY ═══════
#define ENABLE_STOP_BUTTON    1   // STOP button ISR (always recommended!)
                                  // The hardware safety (EN cut) works regardless.
                                  // This flag controls the SOFTWARE reaction (save pos, log)
#define ENABLE_WATCHDOG       0   // Core 1 watchdog (5s timeout)
                                  // Enable only when PID loop is stable
#define ENABLE_ANOMALY_DET    0   // Anomaly detection (overcurrent, decoupling, etc.)
                                  // Requires: ENABLE_MOTORS + encoders

// ═══════ SIMULATION ═══════
#define ENABLE_SIM_POSITION   1   // Simulate AZ/EL position (for protocol testing)
                                  // When 1: position slowly moves toward target (fake PID)
                                  // When 0: position comes from real encoders
                                  // IMPORTANT: Disable this when real encoders are enabled!

// ═══════════════════════════════════════════════════════════════
// RECOMMENDED DEVELOPMENT PHASES:
//
// Phase 1 (DONE): ETHERNET + GS232 + SIM_POSITION + STOP_BUTTON
//   Flags: ENABLE_ETHERNET=1, ENABLE_GS232=1, ENABLE_SIM_POSITION=1,
//          ENABLE_STOP_BUTTON=1
//   Test: PSTRotator sees rotator, sends W commands, C2 returns
//         simulated position moving toward target.
//   Validate: STOP button ISR fires (check serial log).
//
// Phase 2: + I2C bus: MCP23017 + OLED + EEPROM
//   Flags: + ENABLE_MCP23017=1, ENABLE_OLED=1, ENABLE_EEPROM=1
//   Test MCP23017: buttons trigger interrupt, limit switches read,
//         LEDs blink on command, SF pin reads correctly.
//   Test OLED: splash screen, AZ/EL display updates.
//   Test EEPROM: write block A, read back, verify CRC. Write block B,
//         power cycle, verify most recent block is restored.
//         Test double-buffering: corrupt block A, verify B is used.
//   Note: I2C bus test — all 3 devices on same bus, check no conflicts.
//
// Phase 3: + MOTORS (open loop, manual jog via buttons)
//   Flags: + ENABLE_MOTORS=1
//   Test: Buttons CW/CCW/UP/DOWN spin motors via MCP23017 directions.
//         Current reading on ADC (FB pins). Ramp up/down profiles.
//         Overcurrent threshold test (block motor shaft gently).
//         STOP button cuts EN hardware (verify motor stops instantly).
//   Validate: Motor current values match MC33926 datasheet (0.525 V/A).
//
// Phase 4a: + AZ encoder (AS5048A or HH12) — disable SIM for AZ
//   Flags: + ENABLE_AZ_AS5048A=1 (or ENABLE_AZ_HH12=1)
//          Partially disable SIM: AZ from real encoder, EL still simulated
//   Test: Read AS5048A raw value, rotate by hand, verify angle changes.
//         Check parity and error flag handling.
//         Compare multiple reads for noise/repeatability.
//   Validate: Position reported to PSTRotator matches physical AZ.
//
// Phase 4b: + AZ Hall PCNT
//   Flags: + ENABLE_AZ_HALL_PCNT=1
//   Test: Rotate motor, verify PCNT counts increase/decrease correctly.
//         Verify quadrature 4x decoding (direction detection).
//         Test PCNT recalibration: AS5048A re-read corrects PCNT drift.
//   Validate: PCNT resolution matches theoretical 0.0002°/count.
//
// Phase 5: + RS-485 bus: Nano R4 telemetry
//   Flags: + ENABLE_RS485=1, ENABLE_NANO_R4=1
//   Test: ESP32 polls Nano R4 addr 1, reads voltage registers.
//         Verify 24V, 12V readings match multimeter.
//         Verify temperature DS18B20 readings are plausible.
//         Test anti-freeze relay command (toggle on/off).
//         Test communication at 2-3m cable length.
//         Test error recovery: disconnect cable, verify timeout handling.
//   Validate: Telemetry appears in serial log at 1 Hz rate.
//
// Phase 6: + EL sensor (HWT901B or HH12) — disable SIM for EL
//   Flags: + ENABLE_EL_HWT901B=1, ENABLE_HWT901B_BUS=1
//          ENABLE_SIM_POSITION=0 (both axes now real)
//   Prerequisite: HWT901B reconfigured to addr 0x02 (section 9c).
//   Test: Read pitch register, tilt sensor by hand, verify angle changes.
//         Read gyro Y, shake sensor, verify angular velocity.
//         Verify coexistence on RS-485 bus with Nano R4 (alternating polls).
//   Validate: EL position matches inclinometer at known angles (0°, 45°, 90°).
//
// Phase 7: + PID closed-loop positioning
//   Flags: + ENABLE_PID=1
//   Test: Send goto command, verify dish moves to target and stops.
//         Test approach/deceleration near target.
//         Test deadband (no oscillation at target).
//         Test each axis independently first, then both.
//         Tune PID coefficients (start conservative: low Kp, no Ki, no Kd).
//   Validate: Pointing accuracy matches expected precision (0.01° AZ, 0.05° EL).
//
// Phase 8: + APP_TCP (Windows monitoring app)
//   Flags: + ENABLE_APP_TCP=1
//   Test: Connect TCP client to port 4534, verify JSON status push 2x/s.
//         Send JSON commands (goto, stop, jog, get_config).
//         Verify all telemetry fields populated (encoders, current, nano, GPS).
//   Validate: App displays real-time position, target, state, telemetry.
//
// Phase 9: + GPS
//   Flags: + ENABLE_GPS=1
//   Test: Verify NMEA parsing (lat, lon, time, fix quality, sat count).
//         Verify PPS interrupt fires 1x/s (check with scope or serial log).
//         Verify UTC time accuracy vs known reference.
//         Compare GPS position with known JO20BM85DP coordinates.
//   Validate: UTC time on OLED matches actual UTC (within 1 second visual).
//
// Phase 10: + NVS persistent configuration
//   Flags: + ENABLE_NVS_CONFIG=1
//   Test: Change PID coefficients via app, reboot, verify they persist.
//         Change limits, cal offsets, network config — all persist.
//   Validate: All NVS keys listed in section 22 read/write correctly.
//
// Phase 11: + SOLAR_CAL + CAL_OFFSETS
//   Flags: + ENABLE_SOLAR_CAL=1, ENABLE_CAL_OFFSETS=1
//   Test: Run solar scan, verify dish moves in scan pattern.
//         Verify peak detection finds sun maximum.
//         Take 3 cal points, verify offset computation.
//         Verify corrected positions reported to PSTRotator and app.
//   Validate: After calibration, PSTRotator "south" command points dish true south.
//
// Phase 12: + AZELDAT autonomous mode (replaces PSTRotator)
//   Flags: + ENABLE_AZELDAT=1
//   Test: Windows app reads WSJT-X azel.dat, sends targets to ESP32 via TCP 4534.
//         Verify tracking works without PSTRotator running.
//         Verify GS-232 port still accepts manual overrides.
//   Validate: Full EME QSO cycle without PSTRotator in the chain.
//
// Phase 13: + WATCHDOG + ANOMALY_DET
//   Flags: + ENABLE_WATCHDOG=1, ENABLE_ANOMALY_DET=1
//   Test: Verify watchdog doesn't trigger during normal operation.
//         Simulate fault conditions (block motor, disconnect encoder).
//         Verify anomaly detection triggers STATE_FAULT correctly.
//   Validate: System recovers gracefully from all tested fault conditions.
//
// ═══════════════════════════════════════════════════════════════
```

### 28.5c Compile-time validation

```cpp
// In config.h — sanity checks:

#if ENABLE_AZ_AS5048A && ENABLE_AZ_HH12
  #error "Cannot enable both AS5048A and HH-12 for AZ — choose one!"
#endif

#if ENABLE_EL_HWT901B && ENABLE_EL_HH12
  #error "Cannot enable both HWT901B and HH-12 for EL — choose one!"
#endif

#if ENABLE_MOTORS && !ENABLE_MCP23017
  #error "MOTORS requires MCP23017 (motor direction via I2C)"
#endif

#if ENABLE_PID && !ENABLE_MOTORS
  #error "PID requires MOTORS"
#endif

#if ENABLE_PID && !ENABLE_AZ_AS5048A && !ENABLE_AZ_HH12 && !ENABLE_SIM_POSITION
  #error "PID requires at least one AZ encoder or SIM_POSITION"
#endif

#if ENABLE_AZ_HALL_PCNT && !ENABLE_AZ_AS5048A && !ENABLE_AZ_HH12
  #error "PCNT AZ requires an absolute AZ encoder for recalibration"
#endif

#if ENABLE_HWT901B_BUS && !ENABLE_RS485
  #error "HWT901B Modbus requires RS485"
#endif

#if ENABLE_NANO_R4 && !ENABLE_RS485
  #error "Nano R4 Modbus requires RS485"
#endif

#if ENABLE_SOLAR_CAL && !ENABLE_GPS
  #error "Solar calibration requires GPS (UTC time for ephemeris)"
#endif

#if ENABLE_AZELDAT && !ENABLE_APP_TCP
  #error "AZELDAT autonomous mode requires APP_TCP (JSON port 4534)"
#endif

#if ENABLE_SIM_POSITION && (ENABLE_AZ_AS5048A || ENABLE_AZ_HH12) && (ENABLE_EL_HWT901B || ENABLE_EL_HH12)
  #warning "SIM_POSITION is enabled but real encoders are also enabled — sim will be ignored"
#endif
```

### 28.6 Critical pin mapping (quick reference)

```
ADC:    IO1 (FB AZ), IO2 (FB EL)
PCNT:   IO3/IO4 (Hall AZ A/B) — AZ only, no EL PCNT
GPS:    IO5 (UART2 RX ← GPS TX), IO6 (UART2 TX → GPS RX), IO34 (PPS interrupt)
STOP:   IO7 (interrupt, NF fail-safe)
I2C:    IO8 (SDA), IO9 (SCL) — Qwiic fixed
SPI3:   IO12 (SCLK), IO13 (MISO), IO14 (MOSI), IO15 (CS W5500)
INT:    IO16 (MCP23017 interrupt)
PWM:    IO21 (AZ MCPWM), IO38 (EL MCPWM)
ENC AZ: IO35 (CS), IO36 (CLK), IO37 (MISO) — AS5048A, 3.3V, bit-bang
RS485:  IO43 (TX), IO44 (RX) — UART1 bus: Nano R4 (addr 1) + HWT901B EL (addr 2)
FREE:   IO0, IO39, IO40, IO41, IO42 — 5 reserve GPIOs
```

### 28.7 AS5048A SPI read protocol (AZ encoder — essential for encoder module)

```
SPI Mode 1 (CPOL=0, CPHA=1), MSB first, 500 kHz bit-bang
1. CS LOW → send 16-bit command (0x3FFF = read angle) → CS HIGH
2. CS LOW → read 16-bit response (previous command result) → CS HIGH
3. Bit[15] = even parity, Bit[14] = error flag
4. Bits[13:0] = 14-bit angle (0..16383 = 0°..359.978°)
5. If error flag set: read register 0x0001 to clear
6. Average multiple reads for best accuracy (0.05°)
AZIMUTH ONLY — pins IO35/IO36/IO37
```

### 28.7b HWT901B Modbus read protocol (EL inclinometer — essential for Modbus module)

```
RS-485 Modbus RTU, 9600 baud, slave address 0x02 (reconfigured from factory 0x50)
Gyroscope range: fixed 2000°/s (cannot be changed)
Response delay: 1000 µs (reduced from factory 3000 µs via MODDELAY register)

ELEVATION ANGLE (position):
1. Send: 02 03 00 3E 00 01 [CRC16]  (read 1 register at 0x3E = Pitch)
2. Receive: 02 03 02 [MSB] [LSB] [CRC16]
3. int16_t raw = (MSB << 8) | LSB
4. float elevation_deg = raw / 32768.0f * 180.0f

ELEVATION ANGULAR VELOCITY (for PID derivative term):
1. Send: 02 03 00 38 00 01 [CRC16]  (read 1 register at 0x38 = Gyro Y)
2. Receive: 02 03 02 [MSB] [LSB] [CRC16]
3. int16_t raw = (MSB << 8) | LSB
4. float el_velocity_dps = raw / 32768.0f * 2000.0f  (degrees per second)

Optimized: read both in one Modbus transaction:
1. Send: 02 03 00 38 00 07 [CRC16]  (read 7 registers 0x38-0x3E)
2. Parse: GY at offset 0 (gyro Y), Pitch at offset 6 (angle Y/EL)

WRITE PROTOCOL (for config changes, function 0x06):
1. Unlock: 02 06 00 69 B5 88 [CRC16]  (write 0xB588 to KEY register)
2. Write:  02 06 [REG_H] [REG_L] [DATA_H] [DATA_L] [CRC16]
3. Save:   02 06 00 00 00 00 [CRC16]  (write 0x0000 to SAVE register)
Must complete within 10 seconds (auto-lock timeout)!

INITIAL SETUP (one-time, see section 9c):
- Change address from factory 0x50 to 0x02
- Set 6-axis algorithm (disable magnetometer)
- Reduce MODDELAY from 3000 to 1000 µs
- Set angle reference at known 0° elevation

Polled every 1 second in task_modbus, after Nano R4 query.
NO extra GPIO — shares UART1 (IO43/IO44) with Nano R4.
```

### 28.8 Sensor fusion strategy

```
AZIMUTH:
  Startup:  AS5048A → absolute AZ position (0.022° resolution)
            + EEPROM → fine AZ position from last session (if CRC valid & encoder matches)
  Running:  PCNT → continuous AZ tracking (0.0002° resolution)
            Every 1s: AS5048A re-read → PCNT AZ recalibrated to absolute reference
            Discrepancy AS5048A vs PCNT → anomaly flag (mechanical decoupling)

ELEVATION (no PCNT — HWT901B is sole sensor):
  Startup:  HWT901B → absolute EL position (0.05° static, gravity-based)
            + EEPROM → EL position from last session (for cross-check only)
  Running:  HWT901B polled every 1s → direct EL position update
            HWT901B gyroscope → EL angular velocity for PID derivative term
            HWT901B immune to backlash → measures TRUE dish angle
            Anomaly: motor current vs HWT901B gyro mismatch → decoupling

CALIBRATION OFFSETS (from solar calibration, section 27):
  All positions reported externally = raw + offset
  Incoming targets reverse-corrected = target - offset
  Offsets stored in NVS, loaded at boot (step 10 in boot sequence)

Target AZ: ~0.01° effective precision (AS5048A + PCNT fusion)
Target EL: ~0.05° effective precision (HWT901B alone, sufficient for EME)
```

### 28.9 Coding conventions

- All pin definitions in `pins_config.h` (see section 4.5)
- Module-level `#define ENABLE_xxx` for compile-time feature selection
- FreeRTOS tasks pinned to specific cores (see section 19)
- ISRs: set flag + xTaskNotify only, NO I2C/SPI in ISR context
- Angles in float degrees internally, integer for protocol output
- Serial console via USB CDC (115200 baud) for debug logging
- Use `ESP_LOGx()` macros with per-module tags
- All external position reports use CORRECTED values (raw + cal offset)

### 28.10 Version history

| Version | Date | Changes |
|---------|------|---------|
| 6.0 | 2026-02-16 | HH-12 separated SPI, motor dirs to MCP23017, 25 sections |
| 7.0 | 2026-02-16 | AS5048A replaces HH-12, Nano R4 at parabola arm (not shack), TXS0108E removed, WT61PC option added, Claude Code briefing section |
| 7.1 | 2026-02-17 | Hybrid architecture: AS5048A AZ only + HWT901B-RS485 EL (Modbus addr 2 on shared bus). WT61PC/SEN0386 removed. EL encoder GPIOs freed (IO34/39/40 → reserve). BOM updated. Firmware asymmetric: SPI for AZ, Modbus for EL |
| 7.2 | 2026-02-17 | Hall encoder EL removed (IO5/IO6 freed → 8 reserve GPIOs). PCNT AZ only. HWT901B provides both EL position (accelerometer/gravity) and EL velocity (gyroscope) — replaces PCNT for PID derivative. Simplified EL wiring (motor only, no hall connection). Anomaly detection EL uses motor current vs HWT901B instead of PCNT cross-check |
| 7.3 | 2026-02-18 | Integrated official WitMotion Modbus protocol documentation. Factory address 0x50 documented (must reconfigure to 0x02). Added section 9c: full HWT901B initial configuration procedure (unlock/write/save sequence, 6-axis mode, MODDELAY, angle reference). Corrected conversion formulas (raw/32768*scale). Added config registers (KEY, SAVE, AXIS6, ORIENT, BANDWIDTH, MODDELAY). GitHub SDK reference added |
| 7.4 | 2026-02-18 | **GPS NEO-6M integration**: UART2 (IO5 RX, IO6 TX) + PPS interrupt (IO34, avoids IO0 strapping risk). Section 2.12 (hardware), section 10 (GPS time/PPS), section 27 (solar RF calibration). Reserve GPIOs: IO0/39/40/41/42. **Solar RF calibration**: multi-point procedure using Sun as 10 GHz reference for azimuth alignment. Offset computation (constant + optional tilt model), stored in NVS, applied transparently to all position reports (GS-232 + JSON). EL offset verification (HWT901B gravity-based → should be ~0). New state STATE_SOLAR_CAL. New commands solar_cal_start/point/clear. **Feature flags system** (section 28.5b): modular #define ENABLE_xxx for incremental development. Current working baseline: Ethernet + GS-232 with simulated position. BOM: added GPS NEO-6M module. TinyGPSPlus added to PlatformIO lib_deps. |
