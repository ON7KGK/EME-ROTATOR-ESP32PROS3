#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "config.h"

// LED RGB NeoPixel intégrée ProS3 (1 seule LED WS2812B sur IO18)
Adafruit_NeoPixel rgbLed(1, PIN_RGB_LED, NEO_GRB + NEO_KHZ800);

// Variables pour le clignotement non-bloquant
unsigned long previousMillis = 0;
const unsigned long BLINK_INTERVAL = 1000;  // 1 seconde
bool ledOn = false;

void setup() {
  Serial.begin(115200);
  delay(1000);  // Attente stabilisation USB CDC

  DEBUG_PRINTLN("=== EME Rotator Controller ===");
  DEBUG_PRINTLN("Carte : Unexpected Maker ProS3 (ESP32-S3)");
  DEBUG_PRINTLN("Étape 1 : Blink LED RGB");

  // Activer LDO2 pour alimenter la LED RGB
  pinMode(PIN_LDO2_EN, OUTPUT);
  digitalWrite(PIN_LDO2_EN, HIGH);
  DEBUG_PRINTLN("LDO2 activé (alimentation LED RGB)");

  // Initialiser le NeoPixel
  rgbLed.begin();
  rgbLed.setBrightness(30);  // Luminosité modérée (0-255)
  rgbLed.clear();
  rgbLed.show();

  DEBUG_PRINTLN("LED RGB initialisée. Clignotement vert toutes les secondes.");
  DEBUG_PRINTLN("Initialisation terminée.");
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= BLINK_INTERVAL) {
    previousMillis = currentMillis;
    ledOn = !ledOn;

    if (ledOn) {
      rgbLed.setPixelColor(0, rgbLed.Color(0, 255, 0));  // Vert
    } else {
      rgbLed.setPixelColor(0, rgbLed.Color(0, 0, 0));    // Éteint
    }
    rgbLed.show();

    DEBUG_PRINT("LED : ");
    DEBUG_PRINTLN(ledOn ? "VERT" : "OFF");
  }
}
