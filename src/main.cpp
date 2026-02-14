#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "config.h"
#include "protocol.h"

// LED RGB NeoPixel intégrée ProS3 (1 seule LED WS2812B sur IO18)
Adafruit_NeoPixel rgbLed(1, PIN_RGB_LED, NEO_GRB + NEO_KHZ800);

// Variables pour le clignotement non-bloquant
unsigned long previousMillis = 0;
const unsigned long BLINK_INTERVAL = 1000;
bool ledOn = false;

// Position simulée (sera remplacée par les encodeurs aux étapes suivantes)
float currentAz = 180.0f;
float currentEl = 45.0f;

// Position cible
float targetAz = 180.0f;
float targetEl = 45.0f;

// État de mouvement simulé
bool isMoving = false;

void setup() {
  // Activer LDO2 — alimente la LED RGB
  pinMode(PIN_LDO2_EN, OUTPUT);
  digitalWrite(PIN_LDO2_EN, HIGH);

  // LED bleue au boot
  rgbLed.begin();
  rgbLed.setBrightness(30);
  rgbLed.setPixelColor(0, rgbLed.Color(0, 0, 255));
  rgbLed.show();

  // Serial USB CDC
  Serial.begin(115200);
  unsigned long waitStart = millis();
  while (!Serial && (millis() - waitStart < 5000)) {
    delay(10);
  }

  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("=== EME Rotator Controller ===");
  DEBUG_PRINTLN("Carte : Unexpected Maker ProS3 (ESP32-S3)");
  DEBUG_PRINTLN("Étape 2 : Protocole série");

  // Initialiser le parser de protocole
  protocolInit();

  DEBUG_PRINT("Position simulée : AZ=");
  DEBUG_PRINT(currentAz);
  DEBUG_PRINT(" EL=");
  DEBUG_PRINTLN(currentEl);
  DEBUG_PRINTLN("Prêt — envoie des commandes via le moniteur série.");
  DEBUG_PRINTLN("(Configure le moniteur en 'Newline' ou 'CR+LF')");
}

void handleCommand(ParsedCommand &cmd) {
    switch (cmd.cmd) {
        case CMD_QUERY_POS:
            // Répondre avec la position actuelle
            protocolSendPosition(Serial, currentAz, currentEl);
            DEBUG_PRINT("[CMD] Position demandée → AZ=");
            DEBUG_PRINT(currentAz);
            DEBUG_PRINT(" EL=");
            DEBUG_PRINTLN(currentEl);
            break;

        case CMD_GOTO_AZ:
            targetAz = constrain(cmd.az, AZ_MIN, AZ_MAX);
            DEBUG_PRINT("[CMD] Goto AZ=");
            DEBUG_PRINTLN(targetAz);
            isMoving = true;
            break;

        case CMD_GOTO_AZEL:
            if (cmd.az != 0.0f || cmd.cmd == CMD_GOTO_AZEL) {
                targetAz = constrain(cmd.az, AZ_MIN, AZ_MAX);
            }
            targetEl = constrain(cmd.el, EL_MIN, EL_MAX);
            DEBUG_PRINT("[CMD] Goto AZ=");
            DEBUG_PRINT(targetAz);
            DEBUG_PRINT(" EL=");
            DEBUG_PRINTLN(targetEl);
            isMoving = true;
            break;

        case CMD_STOP_ALL:
            targetAz = currentAz;
            targetEl = currentEl;
            isMoving = false;
            DEBUG_PRINTLN("[CMD] STOP ALL");
            break;

        case CMD_STOP_AZ:
            targetAz = currentAz;
            DEBUG_PRINTLN("[CMD] STOP AZ");
            break;

        case CMD_STOP_EL:
            targetEl = currentEl;
            DEBUG_PRINTLN("[CMD] STOP EL");
            break;

        case CMD_JOG_CW:
            DEBUG_PRINTLN("[CMD] JOG CW");
            targetAz = AZ_MAX;
            isMoving = true;
            break;

        case CMD_JOG_CCW:
            DEBUG_PRINTLN("[CMD] JOG CCW");
            targetAz = AZ_MIN;
            isMoving = true;
            break;

        case CMD_JOG_UP:
            DEBUG_PRINTLN("[CMD] JOG UP");
            targetEl = EL_MAX;
            isMoving = true;
            break;

        case CMD_JOG_DOWN:
            DEBUG_PRINTLN("[CMD] JOG DOWN");
            targetEl = EL_MIN;
            isMoving = true;
            break;

        case CMD_VERSION:
            protocolSendVersion(Serial);
            DEBUG_PRINTLN("[CMD] Version demandée");
            break;

        case CMD_UNKNOWN:
            DEBUG_PRINTLN("[CMD] Commande inconnue");
            break;

        default:
            break;
    }
}

void simulateMovement() {
    // Simulation : déplace la position vers la cible à 1°/sec
    float step = 1.0f;  // degrés par seconde (ajusté par BLINK_INTERVAL)

    if (abs(currentAz - targetAz) > 0.1f) {
        currentAz += (targetAz > currentAz) ? step : -step;
        currentAz = constrain(currentAz, AZ_MIN, AZ_MAX);
    }
    if (abs(currentEl - targetEl) > 0.1f) {
        currentEl += (targetEl > currentEl) ? step : -step;
        currentEl = constrain(currentEl, EL_MIN, EL_MAX);
    }

    // Arrêter si cible atteinte
    if (abs(currentAz - targetAz) <= 0.1f && abs(currentEl - targetEl) <= 0.1f) {
        currentAz = targetAz;
        currentEl = targetEl;
        if (isMoving) {
            isMoving = false;
            DEBUG_PRINT("[SIM] Cible atteinte : AZ=");
            DEBUG_PRINT(currentAz);
            DEBUG_PRINT(" EL=");
            DEBUG_PRINTLN(currentEl);
        }
    }
}

void loop() {
    // Lire et traiter les commandes série
    ParsedCommand cmd;
    if (protocolProcessStream(Serial, cmd)) {
        handleCommand(cmd);
    }

    // Clignotement LED + simulation de mouvement (1 Hz)
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= BLINK_INTERVAL) {
        previousMillis = currentMillis;

        // Simulation de mouvement
        simulateMovement();

        // LED : vert clignotant si idle, orange si en mouvement
        ledOn = !ledOn;
        if (ledOn) {
            if (isMoving) {
                rgbLed.setPixelColor(0, rgbLed.Color(255, 100, 0));  // Orange
            } else {
                rgbLed.setPixelColor(0, rgbLed.Color(0, 255, 0));    // Vert
            }
        } else {
            rgbLed.setPixelColor(0, rgbLed.Color(0, 0, 0));          // Éteint
        }
        rgbLed.show();
    }
}
