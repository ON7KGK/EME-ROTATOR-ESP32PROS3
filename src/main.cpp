#include <Arduino.h>
#include "config.h"

void setup() {
  Serial.begin(115200);
  delay(1000);  // Attente stabilisation USB Serial
  DEBUG_PRINTLN("=== EME Rotator Controller ===");
  DEBUG_PRINTLN("Carte : Unexpected Maker ProS3 (ESP32-S3)");
  DEBUG_PRINTLN("Initialisation terminée.");
}

void loop() {
  // Boucle principale — sera complétée étape par étape
}
