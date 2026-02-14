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

// === Carte : Unexpected Maker ProS3 (ESP32-S3) ===

// LED — ProS3 a une LED RGB (NeoPixel) sur GPIO 18
#define LED_PIN 18  // LED RGB intégrée ProS3

// Encodeur HH-12 (SPI)
// #define HH12_CLK_PIN   xx
// #define HH12_MISO_PIN  xx
// #define HH12_CS_AZ_PIN xx   // Chip Select encodeur azimut
// #define HH12_CS_EL_PIN xx   // Chip Select encodeur élévation

// Ethernet W5500 (SPI)
// #define W5500_CS_PIN   xx
// #define W5500_RST_PIN  xx
// #define W5500_INT_PIN  xx

// === Paramètres réseau ===
// #define EASYCOM_TCP_PORT 12000

// === Paramètres rotor ===
// #define AZ_MIN 0.0
// #define AZ_MAX 360.0
// #define EL_MIN 0.0
// #define EL_MAX 90.0

#endif
