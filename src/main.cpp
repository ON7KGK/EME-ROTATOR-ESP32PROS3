// ════════════════════════════════════════════════════════════════
// EME ROTATOR CONTROLLER — v7.4 Dual-Core FreeRTOS
// ════════════════════════════════════════════════════════════════
// Core 1 : temps réel (PID + encodeurs + sécurités)
// Core 0 : communication (TCP + Modbus + GPS + OLED)
// Inter-core : variables atomiques, pas de mutex
// ════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <atomic>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "config.h"
#include "protocol.h"
#include "encoder.h"

#if ENABLE_ETHERNET
  #include "network.h"
#endif

#if ENABLE_EEPROM
  #include "eeprom.h"
#endif

#if ENABLE_OLED
  #include "oled.h"
#endif

#if ENABLE_MCP23017
  #include "mcp23017.h"
  static bool mcpOk = false;
#endif

#if ENABLE_MOTORS
  #include "motor.h"
#endif

#if ENABLE_RS485
  #include <ModbusMaster.h>
  static HardwareSerial rs485Serial(1);
  static ModbusMaster modbusNano;
#endif

#if ENABLE_GPS
  static HardwareSerial gpsSerial(2);
  static bool gpsFix = false;
  static char gpsLine[84];    // NMEA max 82 chars + \n + null
  static uint8_t gpsIdx = 0;

  // Parse minimale : extraire le status A/V de $GPRMC
  static void gpsProcessChar(char c) {
      if (c == '$') gpsIdx = 0;
      if (gpsIdx < sizeof(gpsLine) - 1) {
          gpsLine[gpsIdx++] = c;
          gpsLine[gpsIdx] = '\0';
      }
      if (c == '\n') {
          // $GPRMC,time,status,... → status = A (fix) ou V (void)
          if (strncmp(gpsLine, "$GPRMC,", 7) == 0) {
              char *p = strchr(gpsLine + 7, ',');  // Passer le champ time
              if (p) gpsFix = (*(p + 1) == 'A');
          }
          gpsIdx = 0;
      }
  }
#endif

// ════════════════════════════════════════════════════════════════
// DONNÉES INTER-CORES (variables atomiques)
// ════════════════════════════════════════════════════════════════

// Position courante (écrite par Core 1, lue par Core 0)
std::atomic<float> g_currentAz{180.0f};
std::atomic<float> g_currentEl{45.0f};

// Position cible (écrite par Core 0, lue par Core 1)
std::atomic<float> g_targetAz{180.0f};
std::atomic<float> g_targetEl{45.0f};

// Booléen mouvement en cours (écrit par Core 1, lu par Core 0)
std::atomic<bool> g_isMoving{false};

// ════════════════════════════════════════════════════════════════
// LED RGB NeoPixel intégrée ProS3
// ════════════════════════════════════════════════════════════════

static Adafruit_NeoPixel rgbLed(1, PIN_RGB_LED, NEO_GRB + NEO_KHZ800);

// ════════════════════════════════════════════════════════════════
// BOUTON STOP — ISR (Core 1)
// ════════════════════════════════════════════════════════════════

#if ENABLE_STOP_BUTTON
static volatile bool stopPressed = false;

void IRAM_ATTR isr_stop_button() {
    stopPressed = true;
}
#endif

// ════════════════════════════════════════════════════════════════
// CORE 1 — TÂCHE PID (temps réel, 100 Hz)
// ════════════════════════════════════════════════════════════════

void task_pid_loop(void *param) {
    (void)param;
    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        // Lire cibles (atomique, écrites par Core 0)
        float targetAz = g_targetAz.load(std::memory_order_relaxed);
        float targetEl = g_targetEl.load(std::memory_order_relaxed);
        float currentAz = g_currentAz.load(std::memory_order_relaxed);
        float currentEl = g_currentEl.load(std::memory_order_relaxed);

        #if ENABLE_STOP_BUTTON
            if (stopPressed) {
                // STOP pressé : figer la position, cible = position actuelle
                g_targetAz.store(currentAz, std::memory_order_relaxed);
                g_targetEl.store(currentEl, std::memory_order_relaxed);
                g_isMoving.store(false, std::memory_order_relaxed);
                // Ne pas remettre stopPressed à false ici —
                // Core 0 le lira et décidera quand réarmer
                vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TASK_PID_PERIOD_MS));
                continue;
            }
        #endif

        bool moving = false;

        // ── Position AZ ──
        #if ENABLE_AZ_HH12
            currentAz = hh12ReadAngle(PIN_ENC_AZ_CS, false);
            if (fabsf(currentAz - targetAz) > 0.5f) moving = true;
        #elif ENABLE_AZ_AS5048A
            currentAz = as5048aReadAngle();
            if (fabsf(currentAz - targetAz) > 0.5f) moving = true;
        #elif ENABLE_SIM_POSITION
        {
            float dt = (float)TASK_PID_PERIOD_MS / 1000.0f;
            float stepAz = SIM_AZ_SPEED * dt;
            if (fabsf(currentAz - targetAz) > 0.01f) {
                float diff = targetAz - currentAz;
                currentAz += (fabsf(diff) <= stepAz) ? diff
                           : ((diff > 0) ? stepAz : -stepAz);
                if (currentAz < AZ_MIN) currentAz = AZ_MIN;
                if (currentAz > AZ_MAX) currentAz = AZ_MAX;
                moving = true;
            }
        }
        #endif

        // ── Position EL (simulée tant que capteur EL pas actif) ──
        #if ENABLE_SIM_POSITION && !ENABLE_EL_HWT901B && !ENABLE_EL_HH12
        {
            float dt = (float)TASK_PID_PERIOD_MS / 1000.0f;
            float stepEl = SIM_EL_SPEED * dt;
            if (fabsf(currentEl - targetEl) > 0.01f) {
                float diff = targetEl - currentEl;
                currentEl += (fabsf(diff) <= stepEl) ? diff
                           : ((diff > 0) ? stepEl : -stepEl);
                if (currentEl < EL_MIN) currentEl = EL_MIN;
                if (currentEl > EL_MAX) currentEl = EL_MAX;
                moving = true;
            }
        }
        #endif

        // Écrire position courante + état (atomique, lu par Core 0)
        g_currentAz.store(currentAz, std::memory_order_relaxed);
        g_currentEl.store(currentEl, std::memory_order_relaxed);
        g_isMoving.store(moving, std::memory_order_relaxed);

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(TASK_PID_PERIOD_MS));
    }
}

// ════════════════════════════════════════════════════════════════
// CORE 0 — VARIABLES LOCALES (copies des atomiques)
// ════════════════════════════════════════════════════════════════

// Copies locales Core 0 — rafraîchies à chaque tour de loop()
float currentAz = 180.0f;
float currentEl = 45.0f;
float targetAz  = 180.0f;
float targetEl  = 45.0f;
bool  isMoving  = false;

// Timestamp dernière commande reçue (pour détection stale)
unsigned long lastCommandTime = 0;

// Télémétrie Nano R4 (mise à jour par Modbus polling)
#if ENABLE_RS485
  uint16_t nanoA0mV   = 0;   // Tension A0 en mV
  uint16_t nanoA1mV   = 0;   // Tension A1 en mV
  uint16_t nanoUptime = 0;   // Uptime Nano R4 en secondes
  bool     nanoOnline = false;
#endif

// Flag changement cible → écriture EEPROM immédiate
#if ENABLE_EEPROM
  bool targetChanged = false;
#endif

// Parser pour Serial USB (mode sans Ethernet)
#if !ENABLE_ETHERNET
  static ProtocolState serialState;
#endif

// ════════════════════════════════════════════════════════════════
// GESTION DES COMMANDES (appelé par networkLoop ou Serial)
// ════════════════════════════════════════════════════════════════

void handleCommand(ParsedCommand &cmd, Stream &output) {
    // Toute commande reçue rafraîchit le timestamp (détection stale)
    lastCommandTime = millis();

    switch (cmd.cmd) {
        case CMD_QUERY_POS:
        case CMD_QUERY_AZ:
        case CMD_QUERY_EL:
            // Position demandée — envoyée après le switch
            DEBUG_PRINT("[CMD] Query → AZ=");
            DEBUG_PRINT(currentAz);
            DEBUG_PRINT(" EL=");
            DEBUG_PRINTLN(currentEl);
            break;

        case CMD_GOTO_AZ:
            targetAz = constrain(cmd.az, AZ_MIN, AZ_MAX);
            g_targetAz.store(targetAz, std::memory_order_relaxed);
            #if ENABLE_EEPROM
                targetChanged = true;
            #endif
            DEBUG_PRINT("[CMD] Goto AZ=");
            DEBUG_PRINTLN(targetAz);
            break;

        case CMD_GOTO_EL:
            targetEl = constrain(cmd.el, EL_MIN, EL_MAX);
            g_targetEl.store(targetEl, std::memory_order_relaxed);
            #if ENABLE_EEPROM
                targetChanged = true;
            #endif
            DEBUG_PRINT("[CMD] Goto EL=");
            DEBUG_PRINTLN(targetEl);
            break;

        case CMD_GOTO_AZEL:
            targetAz = constrain(cmd.az, AZ_MIN, AZ_MAX);
            targetEl = constrain(cmd.el, EL_MIN, EL_MAX);
            g_targetAz.store(targetAz, std::memory_order_relaxed);
            g_targetEl.store(targetEl, std::memory_order_relaxed);
            #if ENABLE_EEPROM
                targetChanged = true;
            #endif
            DEBUG_PRINT("[CMD] Goto AZ=");
            DEBUG_PRINT(targetAz);
            DEBUG_PRINT(" EL=");
            DEBUG_PRINTLN(targetEl);
            break;

        case CMD_STOP_ALL:
            targetAz = currentAz;
            targetEl = currentEl;
            g_targetAz.store(targetAz, std::memory_order_relaxed);
            g_targetEl.store(targetEl, std::memory_order_relaxed);
            DEBUG_PRINTLN("[CMD] STOP ALL");
            break;

        case CMD_STOP_AZ:
            targetAz = currentAz;
            g_targetAz.store(targetAz, std::memory_order_relaxed);
            DEBUG_PRINTLN("[CMD] STOP AZ");
            break;

        case CMD_STOP_EL:
            targetEl = currentEl;
            g_targetEl.store(targetEl, std::memory_order_relaxed);
            DEBUG_PRINTLN("[CMD] STOP EL");
            break;

        case CMD_JOG_CW:
            targetAz = AZ_MAX;
            g_targetAz.store(targetAz, std::memory_order_relaxed);
            DEBUG_PRINTLN("[CMD] JOG CW");
            break;

        case CMD_JOG_CCW:
            targetAz = AZ_MIN;
            g_targetAz.store(targetAz, std::memory_order_relaxed);
            DEBUG_PRINTLN("[CMD] JOG CCW");
            break;

        case CMD_JOG_UP:
            targetEl = EL_MAX;
            g_targetEl.store(targetEl, std::memory_order_relaxed);
            DEBUG_PRINTLN("[CMD] JOG UP");
            break;

        case CMD_JOG_DOWN:
            targetEl = EL_MIN;
            g_targetEl.store(targetEl, std::memory_order_relaxed);
            DEBUG_PRINTLN("[CMD] JOG DOWN");
            break;

        case CMD_VERSION:
            protocolSendVersion(output);
            DEBUG_PRINTLN("[CMD] Version");
            break;

        case CMD_UNKNOWN:
            DEBUG_PRINTLN("[CMD] Commande inconnue");
            break;

        default:
            break;
    }

    // Réponse position EasyCom après chaque commande
    if (cmd.cmd != CMD_VERSION && cmd.cmd != CMD_UNKNOWN && cmd.cmd != CMD_NONE) {
        protocolSendPosition(output, currentAz, currentEl);
    }
}

// ════════════════════════════════════════════════════════════════
// SETUP (Core 0)
// ════════════════════════════════════════════════════════════════

void setup() {
    // Activer LDO2 pour LED RGB
    pinMode(PIN_LDO2_EN, OUTPUT);
    digitalWrite(PIN_LDO2_EN, HIGH);

    // LED bleue au boot
    rgbLed.begin();
    rgbLed.setBrightness(30);
    rgbLed.setPixelColor(0, rgbLed.Color(0, 0, 255));
    rgbLed.show();

    // Serial USB CDC (debug + upload)
    Serial.begin(115200);
    unsigned long waitStart = millis();
    while (!Serial && (millis() - waitStart < 5000)) {
        delay(10);
    }

    DEBUG_PRINTLN("");
    DEBUG_PRINTLN("════════════════════════════════════════");
    DEBUG_PRINTLN("  EME Rotator Controller v7.4");
    DEBUG_PRINTLN("  ProS3 (ESP32-S3) Dual-Core FreeRTOS");
    DEBUG_PRINTLN("  Protocole : EasyCom II");
    DEBUG_PRINTLN("════════════════════════════════════════");

    // ── Scan I2C bus (diagnostic) ──
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_SPEED);
    DEBUG_PRINTLN("[I2C] Scan bus...");
    int i2cCount = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            DEBUG_PRINT("[I2C]   0x");
            if (addr < 16) DEBUG_PRINT("0");
            DEBUG_PRINT(String(addr, HEX));
            if (addr == I2C_ADDR_MCP23017) DEBUG_PRINT(" = MCP23017");
            if (addr == I2C_ADDR_OLED)     DEBUG_PRINT(" = OLED SSD1306");
            if (addr == I2C_ADDR_EEPROM)   DEBUG_PRINT(" = EEPROM/FRAM");
            DEBUG_PRINTLN("");
            i2cCount++;
        }
    }
    DEBUG_PRINT("[I2C] ");
    DEBUG_PRINT(i2cCount);
    DEBUG_PRINTLN(" device(s) found");

    // ── EEPROM/FRAM — Restaurer position cible au boot ──
    #if ENABLE_EEPROM
        if (eepromInit()) {
            EepromPositionBlock savedBlock;
            if (eepromReadBest(savedBlock)) {
                targetAz = constrain(savedBlock.target_az_deg, AZ_MIN, AZ_MAX);
                targetEl = constrain(savedBlock.target_el_deg, EL_MIN, EL_MAX);
                g_targetAz.store(targetAz, std::memory_order_relaxed);
                g_targetEl.store(targetEl, std::memory_order_relaxed);
                DEBUG_PRINT("[EEPROM] Cible restaurée: AZ=");
                DEBUG_PRINT(String(targetAz, 1));
                DEBUG_PRINT(" EL=");
                DEBUG_PRINTLN(String(targetEl, 1));
            } else {
                DEBUG_PRINTLN("[EEPROM] Pas de cible sauvegardée (EEPROM vierge)");
            }
        }
    #endif

    // ── OLED SSD1306 128x32 — splash screen 2s ──
    #if ENABLE_OLED
        oledInit();
    #endif

    // ── MCP23017 I/O expander ──
    #if ENABLE_MCP23017
        mcpOk = mcpInit();
    #endif

    // ── MC33926 moteurs (MCPWM) ──
    #if ENABLE_MOTORS
        motorInit();
        // Directions en brake au démarrage
        #if ENABLE_MCP23017
            mcpSetMotorDir(MCP_DIR_AZ_BRAKE | MCP_DIR_EL_BRAKE);
        #endif
    #endif

    // ── Bouton STOP ISR ──
    #if ENABLE_STOP_BUTTON
        pinMode(PIN_STOP_BUTTON, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(PIN_STOP_BUTTON),
                        isr_stop_button, FALLING);
        DEBUG_PRINTLN("[INIT] STOP button ISR sur IO7");
    #endif

    // ── Protocole ──
    protocolInit();

    // ── Ethernet ──
    #if ENABLE_ETHERNET
        networkInit();
    #else
        protocolStateInit(serialState);
        DEBUG_PRINTLN("[INIT] Mode Serial USB (pas d'Ethernet)");
    #endif

    // ── RS-485 Modbus master (UART1) ──
    #if ENABLE_RS485
        rs485Serial.begin(RS485_BAUD, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);
        modbusNano.begin(NANO_MODBUS_ID, rs485Serial);
        DEBUG_PRINT("[RS485] UART1 init : TX=IO");
        DEBUG_PRINT(PIN_RS485_TX);
        DEBUG_PRINT(" RX=IO");
        DEBUG_PRINT(PIN_RS485_RX);
        DEBUG_PRINT(" @ ");
        DEBUG_PRINT(RS485_BAUD);
        DEBUG_PRINTLN(" baud");
    #endif

    // ── Encodeur HH-12 AZ (SSI bit-bang) ──
    #if ENABLE_AZ_HH12
        hh12Init();
    #endif

    // ── GPS NEO-6M (UART2, 9600 baud) ──
    #if ENABLE_GPS
        gpsSerial.begin(9600, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
        DEBUG_PRINT("[GPS] UART2 init : RX=IO");
        DEBUG_PRINT(PIN_GPS_RX);
        DEBUG_PRINT(" TX=IO");
        DEBUG_PRINT(PIN_GPS_TX);
        DEBUG_PRINTLN(" @ 9600 baud");
    #endif

    // ── Lancer tâche PID sur Core 1 ──
    xTaskCreatePinnedToCore(
        task_pid_loop,
        "pid_loop",
        TASK_PID_STACK_SIZE,
        NULL,
        TASK_PID_PRIORITY,
        NULL,
        1  // Core 1
    );
    DEBUG_PRINTLN("[INIT] Core 1 : task_pid_loop (10 ms)");

    #if ENABLE_SIM_POSITION
        DEBUG_PRINTLN("[INIT] Mode SIMULATION actif");
        DEBUG_PRINT("[INIT] Position initiale AZ=");
        DEBUG_PRINT(currentAz);
        DEBUG_PRINT(" EL=");
        DEBUG_PRINTLN(currentEl);
    #endif

    DEBUG_PRINTLN("════════════════════════════════════════");
    DEBUG_PRINTLN("  PRET");
    DEBUG_PRINTLN("════════════════════════════════════════");
}

// ════════════════════════════════════════════════════════════════
// LOOP (Core 0 — communication)
// ════════════════════════════════════════════════════════════════

void loop() {
    // ── Rafraîchir copies locales depuis Core 1 ──
    currentAz = g_currentAz.load(std::memory_order_relaxed);
    currentEl = g_currentEl.load(std::memory_order_relaxed);
    isMoving  = g_isMoving.load(std::memory_order_relaxed);

    // ── Bouton STOP : réaction software ──
    #if ENABLE_STOP_BUTTON
        if (stopPressed) {
            // Synchroniser cibles locales avec la position figée
            targetAz = currentAz;
            targetEl = currentEl;
            // Log une seule fois
            static bool stopLogged = false;
            if (!stopLogged) {
                DEBUG_PRINTLN("!!! STOP BUTTON PRESSED !!!");
                stopLogged = true;
            }
            // Réarmer quand le bouton est relâché (NF = HIGH au repos)
            if (digitalRead(PIN_STOP_BUTTON) == HIGH) {
                stopPressed = false;
                stopLogged = false;
                DEBUG_PRINTLN("[STOP] Bouton relâché, reprise possible");
            }
        }
    #endif

    // ── Contrôle moteur (simple bang-bang, Core 0) ──
    #if ENABLE_MOTORS
    {
        static unsigned long prevMotor = 0;
        static uint8_t prevDirBits = 0xFF;  // Force la première écriture
        unsigned long nowMotor = millis();
        if (nowMotor - prevMotor >= MOT_CONTROL_MS) {
            prevMotor = nowMotor;

            // Si STOP pressé, tout couper immédiatement
            #if ENABLE_STOP_BUTTON
            if (stopPressed) {
                motorBrakeAll();
                if (prevDirBits != 0) {
                    mcpSetMotorDir(0);
                    prevDirBits = 0;
                }
                goto motor_done;
            }
            #endif

            float errAz = targetAz - currentAz;
            float errEl = targetEl - currentEl;

            uint8_t dirBits = 0;
            float dutyAz = 0.0f;
            float dutyEl = 0.0f;

            // AZ — rampe proportionnelle
            if (fabsf(errAz) > MOT_DEADBAND_DEG) {
                dirBits |= (errAz > 0) ? MCP_DIR_AZ_CW : MCP_DIR_AZ_CCW;
                float t = fminf(fabsf(errAz) / MOT_RAMP_DEG, 1.0f);
                dutyAz = MOT_MIN_DUTY + t * (MOT_MAX_DUTY - MOT_MIN_DUTY);
            }

            // EL — rampe proportionnelle
            if (fabsf(errEl) > MOT_DEADBAND_DEG) {
                dirBits |= (errEl > 0) ? MCP_DIR_EL_UP : MCP_DIR_EL_DOWN;
                float t = fminf(fabsf(errEl) / MOT_RAMP_DEG, 1.0f);
                dutyEl = MOT_MIN_DUTY + t * (MOT_MAX_DUTY - MOT_MIN_DUTY);
            }

            // Debug moteur (toutes les 1s = 10 itérations)
            static uint8_t motDbgCnt = 0;
            if (++motDbgCnt >= 10) {
                motDbgCnt = 0;
                if (dirBits != 0) {
                    DEBUG_PRINT("[MOT] err AZ=");
                    DEBUG_PRINT(String(errAz, 1));
                    DEBUG_PRINT(" EL=");
                    DEBUG_PRINT(String(errEl, 1));
                    DEBUG_PRINT(" dir=0x");
                    DEBUG_PRINT(String(dirBits, HEX));
                    DEBUG_PRINT(" duty AZ=");
                    DEBUG_PRINT(String(dutyAz, 0));
                    DEBUG_PRINT(" EL=");
                    DEBUG_PRINTLN(String(dutyEl, 0));
                }
            }

            // Direction via MCP23017 (seulement si changée)
            #if ENABLE_MCP23017
            if (dirBits != prevDirBits) {
                // Couper le PWM avant de changer la direction (anti-shoot-through)
                motorBrakeAll();
                mcpSetMotorDir(dirBits);
                prevDirBits = dirBits;
                delayMicroseconds(50);  // Laisser le pont H se configurer
                DEBUG_PRINT("[MOT] Direction changée → 0x");
                DEBUG_PRINTLN(String(dirBits, HEX));
            }
            #endif

            motorSetDutyAz(dutyAz);
            motorSetDutyEl(dutyEl);
        }
        motor_done: ;
    }
    #endif

    // ── Traiter communication réseau ou série ──
    #if ENABLE_ETHERNET
        networkLoop();
    #else
        ParsedCommand cmd;
        if (protocolProcessStream(Serial, serialState, cmd)) {
            handleCommand(cmd, Serial);
        }
    #endif

    // ── Modbus polling Nano R4 (2s) ──
    #if ENABLE_RS485
    {
        static unsigned long prevModbus = 0;
        unsigned long nowModbus = millis();
        if (nowModbus - prevModbus >= MODBUS_POLL_INTERVAL_MS) {
            prevModbus = nowModbus;

            // Lire 6 holding registers (FC 0x03) du Nano R4
            uint8_t result = modbusNano.readHoldingRegisters(0, 6);
            if (result == modbusNano.ku8MBSuccess) {
                nanoA0mV   = modbusNano.getResponseBuffer(2);
                nanoA1mV   = modbusNano.getResponseBuffer(3);
                nanoUptime = modbusNano.getResponseBuffer(4);
                nanoOnline = true;
                DEBUG_PRINT("[MODBUS] Nano R4: A0=");
                DEBUG_PRINT(nanoA0mV);
                DEBUG_PRINT("mV  A1=");
                DEBUG_PRINT(nanoA1mV);
                DEBUG_PRINT("mV  uptime=");
                DEBUG_PRINT(nanoUptime);
                DEBUG_PRINTLN("s");
            } else {
                nanoOnline = false;
                DEBUG_PRINT("[MODBUS] Erreur Nano R4: 0x");
                DEBUG_PRINTLN(String(result, HEX));
            }
        }
    }
    #endif

    // ── GPS : parser NMEA pour détecter le fix ──
    #if ENABLE_GPS
    while (gpsSerial.available()) {
        gpsProcessChar(gpsSerial.read());
    }
    #endif

    // ── MCP23017 : polling boutons + limites (500ms diagnostic) ──
    #if ENABLE_MCP23017
    if (mcpOk) {
        static unsigned long prevMcp = 0;
        static uint8_t prevPortA = 0xFF;
        unsigned long nowMcp = millis();
        if (nowMcp - prevMcp >= 500) {
            prevMcp = nowMcp;
            uint8_t portA = mcpReadPortA();
            // Logger uniquement si changement
            if (portA != prevPortA) {
                prevPortA = portA;
                DEBUG_PRINT("[MCP] Port A = 0b");
                for (int i = 7; i >= 0; i--) {
                    DEBUG_PRINT((portA >> i) & 1);
                }
                DEBUG_PRINTLN("");
                // Détail lisible
                if (!(portA & (1 << MCP_BTN_CW)))   DEBUG_PRINTLN("[MCP]   BTN CW pressé");
                if (!(portA & (1 << MCP_BTN_CCW)))  DEBUG_PRINTLN("[MCP]   BTN CCW pressé");
                if (!(portA & (1 << MCP_BTN_UP)))   DEBUG_PRINTLN("[MCP]   BTN UP pressé");
                if (!(portA & (1 << MCP_BTN_DOWN))) DEBUG_PRINTLN("[MCP]   BTN DOWN pressé");
                if (!(portA & (1 << MCP_LIMIT_CW)))  DEBUG_PRINTLN("[MCP]   LIMIT CW actif");
                if (!(portA & (1 << MCP_LIMIT_CCW))) DEBUG_PRINTLN("[MCP]   LIMIT CCW actif");
                if (!(portA & (1 << MCP_LIMIT_UP)))  DEBUG_PRINTLN("[MCP]   LIMIT UP actif");
                if (!(portA & (1 << MCP_LIMIT_DOWN)))DEBUG_PRINTLN("[MCP]   LIMIT DOWN actif");
            }
        }
    }
    #endif

    // ── Sauvegarde EEPROM immédiate à chaque changement de cible ──
    #if ENABLE_EEPROM
    if (targetChanged) {
        targetChanged = false;
        eepromWritePosition(currentAz, currentEl, 0, 0.0f, targetAz, targetEl);
    }
    #endif

    // ── Refresh 500ms : OLED + JSON push app ──
    #if ENABLE_OLED || ENABLE_APP_TCP
    {
        static unsigned long prevRefresh = 0;
        unsigned long nowRefresh = millis();
        if (nowRefresh - prevRefresh >= 500) {
            prevRefresh = nowRefresh;

            // Détection stale : PSTRotator silencieux depuis STALE_TARGET_MS
            bool stale = (lastCommandTime > 0)
                       ? (nowRefresh - lastCommandTime >= STALE_TARGET_MS)
                       : true;  // Pas encore reçu de commande → stale

            // Déterminer l'état courant
            bool isStopped = false;
            #if ENABLE_STOP_BUTTON
                isStopped = stopPressed;
            #endif

            const char *stateStr = isStopped ? "STOP"
                                 : isMoving  ? "MOVING"
                                             : "IDLE";

            bool gpsOk = false;
            #if ENABLE_GPS
                gpsOk = gpsFix;
            #endif

            // ── OLED ──
            #if ENABLE_OLED
            {
                char oledStatus[22];
                if (isStopped) {
                    strcpy(oledStatus, "** STOP **");
                } else if (isMoving) {
                    float dAz = targetAz - currentAz;
                    float dEl = targetEl - currentEl;
                    const char *azDir = (fabsf(dAz) < 0.05f) ? "--"
                                      : (dAz > 0 ? "CW" : "CCW");
                    const char *elDir = (fabsf(dEl) < 0.05f) ? "--"
                                      : (dEl > 0 ? "UP" : "DN");
                    snprintf(oledStatus, sizeof(oledStatus), "TRACK %s %s", azDir, elDir);
                } else {
                    strcpy(oledStatus, "IDLE");
                }
                if (gpsOk) strcat(oledStatus, " GPS");
                oledUpdate(currentAz, currentEl, targetAz, targetEl, stale, oledStatus);
            }
            #endif

            // ── JSON push vers app Windows (port 4534) ──
            #if ENABLE_APP_TCP && ENABLE_ETHERNET
                appSendStatus(currentAz, currentEl, targetAz, targetEl,
                              stateStr, isMoving, isStopped, gpsOk, stale);
            #endif
        }
    }
    #endif

    // ── LED RGB (1 Hz) ──
    static unsigned long prevBlink = 0;
    static bool ledOn = false;
    unsigned long now = millis();

    if (now - prevBlink >= 1000) {
        prevBlink = now;
        ledOn = !ledOn;
        if (ledOn) {
            #if ENABLE_STOP_BUTTON
            if (stopPressed) {
                rgbLed.setPixelColor(0, rgbLed.Color(255, 0, 0));    // Rouge = STOP
            } else
            #endif
            if (isMoving) {
                rgbLed.setPixelColor(0, rgbLed.Color(255, 100, 0));  // Orange = mouvement
            } else {
                rgbLed.setPixelColor(0, rgbLed.Color(0, 255, 0));    // Vert = idle
            }
        } else {
            rgbLed.setPixelColor(0, 0);
        }
        rgbLed.show();
    }
}
