#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "config.h"
#include "protocol.h"
#include "encoder.h"
#if ETHERNET_ENABLED
  #include "network.h"
#endif

// LED RGB NeoPixel intégrée ProS3 (1 seule LED WS2812B sur IO18)
Adafruit_NeoPixel rgbLed(1, PIN_RGB_LED, NEO_GRB + NEO_KHZ800);

// Variables pour le clignotement non-bloquant
unsigned long previousMillis = 0;
const unsigned long BLINK_INTERVAL = 1000;
bool ledOn = false;

// Position cible
float targetAz = 180.0f;
float targetEl = 45.0f;

// État de mouvement
bool isMoving = false;

// État du parser pour le port Série USB (mode sans Ethernet uniquement)
#if !ETHERNET_ENABLED
  ProtocolState serialState;
#endif

void setup() {
  // Activer LDO2 — alimente la LED RGB
  pinMode(PIN_LDO2_EN, OUTPUT);
  digitalWrite(PIN_LDO2_EN, HIGH);

  // LED bleue au boot
  rgbLed.begin();
  rgbLed.setBrightness(30);
  rgbLed.setPixelColor(0, rgbLed.Color(0, 0, 255));
  rgbLed.show();

  // Serial USB CDC (toujours initialisé pour upload + debug)
  Serial.begin(115200);
  unsigned long waitStart = millis();
  while (!Serial && (millis() - waitStart < 5000)) {
    delay(10);
  }

  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("=== EME Rotator Controller ===");
  DEBUG_PRINTLN("Carte : Unexpected Maker ProS3 (ESP32-S3)");
  #if ETHERNET_ENABLED
    DEBUG_PRINTLN("Mode : Ethernet W5500 (Serial USB = upload only)");
  #else
    DEBUG_PRINTLN("Mode : Serial USB (PSTRotator direct)");
  #endif

  // Initialiser encodeurs (ou simulation)
  setupEncoders();

  // Initialiser cibles à la position courante
  targetAz = currentAz;
  targetEl = currentEl;

  // Initialiser le parser de protocole
  protocolInit();

  #if ETHERNET_ENABLED
    // Initialiser Ethernet W5500 (DHCP avec fallback statique)
    networkInit();
    DEBUG_PRINTLN("Prêt — commandes via TCP Ethernet.");
  #else
    protocolStateInit(serialState);
    DEBUG_PRINTLN("Prêt — commandes via série USB.");
  #endif
}

void handleCommand(ParsedCommand &cmd, Stream &output) {
    switch (cmd.cmd) {
        case CMD_QUERY_POS:
        case CMD_QUERY_AZ:
        case CMD_QUERY_EL:
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

        case CMD_GOTO_EL:
            targetEl = constrain(cmd.el, EL_MIN, EL_MAX);
            DEBUG_PRINT("[CMD] Goto EL=");
            DEBUG_PRINTLN(targetEl);
            isMoving = true;
            break;

        case CMD_GOTO_AZEL:
            targetAz = constrain(cmd.az, AZ_MIN, AZ_MAX);
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
            protocolSendVersion(output);
            DEBUG_PRINTLN("[CMD] Version demandée");
            break;

        case CMD_UNKNOWN:
            DEBUG_PRINTLN("[CMD] Commande inconnue");
            break;

        default:
            break;
    }

    // Feedback position après chaque commande (comportement K3NG)
    // PSTRotator attend toujours une réponse position
    if (cmd.cmd != CMD_VERSION && cmd.cmd != CMD_UNKNOWN && cmd.cmd != CMD_NONE) {
        protocolSendPosition(output, currentAz, currentEl);
    }
}

void loop() {
    #if ETHERNET_ENABLED
        // Mode Ethernet : commandes via TCP uniquement
        #if DEBUG
            unsigned long t0 = millis();
        #endif
        networkLoop();
        #if DEBUG
            unsigned long dt = millis() - t0;
            if (dt > 10) {
                DEBUG_PRINT("WARN: networkLoop() bloqué ");
                DEBUG_PRINT(dt);
                DEBUG_PRINTLN(" ms");
            }
        #endif
    #else
        // Mode Serial USB : commandes via série
        ParsedCommand cmd;
        if (protocolProcessStream(Serial, serialState, cmd)) {
            handleCommand(cmd, Serial);
        }
    #endif

    // Mise à jour encodeurs (throttled à ENCODER_READ_INTERVAL)
    updateEncoders();

    // Détection cible atteinte (simulation)
    #if SIMULATE_ENCODERS
        if (isMoving && abs(currentAz - targetAz) <= 0.01f && abs(currentEl - targetEl) <= 0.01f) {
            isMoving = false;
            DEBUG_PRINT("[SIM] Cible atteinte : AZ=");
            DEBUG_PRINT(currentAz);
            DEBUG_PRINT(" EL=");
            DEBUG_PRINTLN(currentEl);
        }
    #endif

    // Clignotement LED (1 Hz)
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= BLINK_INTERVAL) {
        previousMillis = currentMillis;

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
