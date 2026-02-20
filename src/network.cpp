// ════════════════════════════════════════════════════════════════
// EME ROTATOR CONTROLLER — Module Ethernet W5500 (Implementation)
// ════════════════════════════════════════════════════════════════
// W5500 (Adafruit 3201) sur SPI3 du ProS3
// SPI3 : SCLK=IO12, MISO=IO13, MOSI=IO14, CS=IO15
// DHCP avec fallback IP statique
// 2 serveurs TCP : port 4533 (EasyCom) + port 4534 (app)
// ════════════════════════════════════════════════════════════════

#include "config.h"

#if ENABLE_ETHERNET

#include "network.h"
#include "protocol.h"

#include <SPI.h>
#include <Ethernet.h>

#if ENABLE_APP_TCP
  #include <ArduinoJson.h>
  #include "nvs_config.h"
#endif

// Wrapper pour contourner l'incompatibilité entre le Server ESP32
// (begin(uint16_t) pure virtual) et EthernetServer (begin() sans param)
class W5500Server : public EthernetServer {
public:
    W5500Server(uint16_t port) : EthernetServer(port) {}
    using EthernetServer::begin;
    void begin(uint16_t port) override { (void)port; EthernetServer::begin(); }
};

// ════════════════════════════════════════════════════════════════
// CONFIGURATION RÉSEAU
// ════════════════════════════════════════════════════════════════

// Adresse MAC pour le W5500 (pas de MAC intégrée, définie manuellement)
static byte mac[] = { 0xDE, 0xAD, 0xBE, 0x07, 0x4B, 0x47 };

// IP statique de fallback (depuis config.h)
static IPAddress staticIP(STATIC_IP);
static IPAddress gateway(STATIC_GATEWAY);
static IPAddress subnet(STATIC_SUBNET);
static IPAddress dns_server(STATIC_DNS);

// ════════════════════════════════════════════════════════════════
// VARIABLES MODULE
// ════════════════════════════════════════════════════════════════

static bool ethConnected = false;
static bool useDHCP = false;  // true si DHCP a réussi (pour maintain())

// Serveurs TCP
static W5500Server easycomServer(EASYCOM_TCP_PORT);
static W5500Server appServer(APP_TCP_PORT);

// Clients TCP actuels (1 par port)
static EthernetClient easycomClient;
static EthernetClient appClient;

// État du parser pour chaque client TCP
static ProtocolState easycomState;
static ProtocolState appState;

// handleCommand est défini dans main.cpp
extern void handleCommand(ParsedCommand &cmd, Stream &output);

// Télémétrie Nano R4 (définie dans main.cpp)
#if ENABLE_RS485
  extern uint16_t nanoA0mV;
  extern uint16_t nanoA1mV;
  extern uint16_t nanoUptime;
  extern bool     nanoOnline;
#endif

// ════════════════════════════════════════════════════════════════
// INITIALISATION
// ════════════════════════════════════════════════════════════════

void networkInit() {
    // Pré-configurer CS en output HIGH avant d'initialiser SPI
    pinMode(PIN_W5500_CS, OUTPUT);
    digitalWrite(PIN_W5500_CS, HIGH);

    // Configurer SPI pour le W5500 (pins SPI3 du ProS3)
    // SS = -1 : pas de hardware SS, le CS est géré par la lib Ethernet
    SPI.begin(PIN_W5500_SCLK, PIN_W5500_MISO, PIN_W5500_MOSI, -1);

    // Indiquer au Ethernet library quel pin CS utiliser
    Ethernet.init(PIN_W5500_CS);

    DEBUG_PRINTLN("=== ETHERNET W5500 ===");

    #if DHCP_ENABLED
        DEBUG_PRINTLN("Tentative DHCP...");

        // Essayer DHCP — c'est la seule init W5500
        if (Ethernet.begin(mac, DHCP_TIMEOUT_MS) == 0) {
            DEBUG_PRINTLN("DHCP échoué");

            // Vérifier que le hardware est présent (valide seulement après begin())
            if (Ethernet.hardwareStatus() == EthernetNoHardware) {
                DEBUG_PRINTLN("ERREUR: W5500 non détecté !");
                ethConnected = false;
                return;
            }

            if (Ethernet.linkStatus() == LinkOFF) {
                DEBUG_PRINTLN("WARN: Câble Ethernet non connecté");
            }

            // Fallback statique SANS ré-initialiser le W5500
            // (évite le double software reset → link bounce → 1 min STP)
            useDHCP = false;
            Ethernet.setLocalIP(staticIP);
            Ethernet.setSubnetMask(subnet);
            Ethernet.setGatewayIP(gateway);
            Ethernet.setDnsServerIP(dns_server);
            DEBUG_PRINT("Fallback IP statique : ");
        } else {
            useDHCP = true;
            DEBUG_PRINT("DHCP OK, IP : ");
        }
    #else
        // IP statique directe — une seule init W5500
        useDHCP = false;
        Ethernet.begin(mac, staticIP, dns_server, gateway, subnet);

        if (Ethernet.hardwareStatus() == EthernetNoHardware) {
            DEBUG_PRINTLN("ERREUR: W5500 non détecté !");
            ethConnected = false;
            return;
        }

        if (Ethernet.linkStatus() == LinkOFF) {
            DEBUG_PRINTLN("WARN: Câble Ethernet non connecté");
        }

        DEBUG_PRINT("IP statique : ");
    #endif

    DEBUG_PRINTLN(Ethernet.localIP());
    DEBUG_PRINT("Hardware status : ");
    DEBUG_PRINTLN(Ethernet.hardwareStatus());  // 3 = W5500
    ethConnected = true;

    // Initialiser les états de parser TCP
    protocolStateInit(easycomState);
    protocolStateInit(appState);

    // Démarrer les serveurs TCP
    easycomServer.begin();
    appServer.begin();

    DEBUG_PRINT("Serveur TCP EasyCom sur port ");
    DEBUG_PRINTLN(EASYCOM_TCP_PORT);
    DEBUG_PRINT("Serveur TCP app sur port ");
    DEBUG_PRINTLN(APP_TCP_PORT);
}

// ════════════════════════════════════════════════════════════════
// GESTION D'UN CLIENT TCP
// ════════════════════════════════════════════════════════════════

static void handleTcpClient(W5500Server &server, EthernetClient &client,
                            ProtocolState &state, const char *label) {
    // Écouter nouvelles connexions (server.available = client avec data prête)
    EthernetClient newClient = server.available();
    if (newClient) {
        if (!client || !client.connected()) {
            // Pas de client actuel → accepter
            client = newClient;
            protocolStateInit(state);
            DEBUG_PRINT("[TCP] Client connecté sur ");
            DEBUG_PRINTLN(label);
        } else if (newClient != client) {
            // Client différent → rejeter (un seul client par port)
            newClient.stop();
        }
    }

    // Traiter les données du client connecté
    if (client && client.connected()) {
        while (client.available() > 0) {
            ParsedCommand cmd;
            if (protocolProcessStream(client, state, cmd)) {
                handleCommand(cmd, client);
            }
        }
    }

    // Déconnecter un client qui a fermé la connexion
    if (client && !client.connected()) {
        DEBUG_PRINT("[TCP] Client déconnecté de ");
        DEBUG_PRINTLN(label);
        client.stop();
        protocolStateInit(state);
    }
}

// ════════════════════════════════════════════════════════════════
// APP TCP — Serveur JSON (port 4534)
// ════════════════════════════════════════════════════════════════

#if ENABLE_APP_TCP

#define APP_JSON_BUF_SIZE 512
static char appJsonBuf[APP_JSON_BUF_SIZE];
static uint16_t appJsonIdx = 0;

// ── Envoyer la config au client app ──
static void appSendConfig() {
    if (!appClient || !appClient.connected()) return;

    const char *azNames[] = {"sim", "hh12", "as5048a"};
    const char *elNames[] = {"sim", "hh12", "witmotion", "as5048a"};

    char buf[512];
    int n = snprintf(buf, sizeof(buf),
        "{\"type\":\"config\","
        "\"mot_max_duty\":%.1f,"
        "\"mot_min_duty\":%.1f,"
        "\"mot_ramp_deg\":%.1f,"
        "\"mot_deadband\":%.1f,"
        "\"oled\":%s,"
        "\"ethernet\":%s,"
        "\"gps\":%s,"
        "\"mcp23017\":%s,"
        "\"nano_r4\":%s,"
        "\"az_encoder\":\"%s\","
        "\"el_sensor\":\"%s\"}\n",
        cfg.motMaxDuty, cfg.motMinDuty, cfg.motRampDeg, cfg.motDeadband,
        cfg.oledActive ? "true" : "false",
        cfg.ethernetActive ? "true" : "false",
        cfg.gpsActive ? "true" : "false",
        cfg.mcp23017Active ? "true" : "false",
        cfg.nanoR4Active ? "true" : "false",
        azNames[cfg.azEncoder],
        elNames[cfg.elSensor]);
    appClient.write((const uint8_t*)buf, n);
}

// ── Parse encodeur AZ depuis string JSON ──
static bool parseAzEncoder(const char *s, AzEncoderType &out) {
    if (strcmp(s, "sim") == 0)     { out = AZ_ENC_SIM;     return true; }
    if (strcmp(s, "hh12") == 0)    { out = AZ_ENC_HH12;    return true; }
    if (strcmp(s, "as5048a") == 0) { out = AZ_ENC_AS5048A;  return true; }
    return false;
}

// ── Parse capteur EL depuis string JSON ──
static bool parseElSensor(const char *s, ElSensorType &out) {
    if (strcmp(s, "sim") == 0)       { out = EL_ENC_SIM;       return true; }
    if (strcmp(s, "hh12") == 0)      { out = EL_ENC_HH12;      return true; }
    if (strcmp(s, "witmotion") == 0)  { out = EL_ENC_WITMOTION; return true; }
    if (strcmp(s, "as5048a") == 0)   { out = EL_ENC_AS5048A;    return true; }
    return false;
}

// Parse une ligne JSON reçue du client app et exécute la commande
static void appParseLine(const char *line) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, line);
    if (err) {
        DEBUG_PRINT("[APP] JSON parse error: ");
        DEBUG_PRINTLN(err.c_str());
        return;
    }

    const char *cmd = doc["cmd"];
    if (!cmd) return;

    // ── Commandes de contrôle rotor ──
    ParsedCommand parsed;
    parsed.cmd = CMD_NONE;
    parsed.az = 0;
    parsed.el = 0;

    if (strcmp(cmd, "goto") == 0) {
        parsed.az = doc["az"] | 0.0f;
        parsed.el = doc["el"] | 0.0f;
        parsed.cmd = CMD_GOTO_AZEL;
    } else if (strcmp(cmd, "stop") == 0) {
        parsed.cmd = CMD_STOP_ALL;
    } else if (strcmp(cmd, "jog") == 0) {
        const char *dir = doc["dir"];
        if (dir) {
            if (strcmp(dir, "cw") == 0)   parsed.cmd = CMD_JOG_CW;
            else if (strcmp(dir, "ccw") == 0)  parsed.cmd = CMD_JOG_CCW;
            else if (strcmp(dir, "up") == 0)   parsed.cmd = CMD_JOG_UP;
            else if (strcmp(dir, "down") == 0) parsed.cmd = CMD_JOG_DOWN;
        }
    } else if (strcmp(cmd, "jog_stop") == 0) {
        parsed.cmd = CMD_STOP_ALL;
    }
    // ── Commandes de configuration ──
    else if (strcmp(cmd, "get_config") == 0) {
        appSendConfig();
        return;
    }
    else if (strcmp(cmd, "set_config") == 0) {
        bool reboot = false;

        // Moteur (hot)
        if (doc.containsKey("mot_max_duty"))  cfg.motMaxDuty  = doc["mot_max_duty"];
        if (doc.containsKey("mot_min_duty"))  cfg.motMinDuty  = doc["mot_min_duty"];
        if (doc.containsKey("mot_ramp_deg"))  cfg.motRampDeg  = doc["mot_ramp_deg"];
        if (doc.containsKey("mot_deadband"))  cfg.motDeadband = doc["mot_deadband"];

        // Modules (reboot)
        if (doc.containsKey("oled"))     { cfg.oledActive     = doc["oled"];     reboot = true; }
        if (doc.containsKey("ethernet")) { cfg.ethernetActive = doc["ethernet"]; reboot = true; }
        if (doc.containsKey("gps"))      { cfg.gpsActive      = doc["gps"];      reboot = true; }
        if (doc.containsKey("mcp23017")) { cfg.mcp23017Active = doc["mcp23017"]; reboot = true; }
        if (doc.containsKey("nano_r4"))  { cfg.nanoR4Active   = doc["nano_r4"];  reboot = true; }

        // Encodeurs (reboot)
        if (doc.containsKey("az_encoder")) {
            AzEncoderType az;
            if (parseAzEncoder(doc["az_encoder"], az)) { cfg.azEncoder = az; reboot = true; }
        }
        if (doc.containsKey("el_sensor")) {
            ElSensorType el;
            if (parseElSensor(doc["el_sensor"], el)) { cfg.elSensor = el; reboot = true; }
        }

        // Clamp moteur
        if (cfg.motMaxDuty < 10.0f)  cfg.motMaxDuty = 10.0f;
        if (cfg.motMaxDuty > 100.0f) cfg.motMaxDuty = 100.0f;
        if (cfg.motMinDuty < 5.0f)   cfg.motMinDuty = 5.0f;
        if (cfg.motMinDuty > cfg.motMaxDuty) cfg.motMinDuty = cfg.motMaxDuty;
        if (cfg.motRampDeg < 1.0f)   cfg.motRampDeg = 1.0f;
        if (cfg.motDeadband < 0.1f)  cfg.motDeadband = 0.1f;

        char ack[64];
        snprintf(ack, sizeof(ack),
            "{\"type\":\"config_ack\",\"reboot\":%s}\n",
            reboot ? "true" : "false");
        appClient.write((const uint8_t*)ack, strlen(ack));
        DEBUG_PRINTLN("[APP] Config modifiée");
        return;
    }
    else if (strcmp(cmd, "save_config") == 0) {
        cfgSave();
        const char *r = "{\"type\":\"config_saved\"}\n";
        appClient.write((const uint8_t*)r, strlen(r));
        return;
    }
    else if (strcmp(cmd, "reset_config") == 0) {
        cfgResetDefaults();
        cfgSave();
        const char *r = "{\"type\":\"config_reset\"}\n";
        appClient.write((const uint8_t*)r, strlen(r));
        appSendConfig();
        return;
    }

    if (parsed.cmd != CMD_NONE) {
        handleCommand(parsed, appClient);
    }
}

// Gère le client TCP app : connexion, réception JSON, déconnexion
static void handleAppTcpClient() {
    // Accepter nouvelle connexion (accept() détecte la connexion TCP
    // immédiatement, contrairement à available() qui attend des données)
    EthernetClient newClient = appServer.accept();
    if (newClient) {
        if (!appClient || !appClient.connected()) {
            appClient = newClient;
            appJsonIdx = 0;
            DEBUG_PRINTLN("[APP] Client connecté sur port 4534");
        } else if (newClient != appClient) {
            newClient.stop();
        }
    }

    // Lire données entrantes (JSON ligne par ligne)
    if (appClient && appClient.connected()) {
        while (appClient.available() > 0) {
            char c = appClient.read();
            if (c == '\n' || c == '\r') {
                if (appJsonIdx > 0) {
                    appJsonBuf[appJsonIdx] = '\0';
                    appParseLine(appJsonBuf);
                    appJsonIdx = 0;
                }
            } else if (appJsonIdx < APP_JSON_BUF_SIZE - 1) {
                appJsonBuf[appJsonIdx++] = c;
            } else {
                // Buffer overflow — ignorer la ligne
                appJsonIdx = 0;
            }
        }
    }

    // Nettoyer client déconnecté
    if (appClient && !appClient.connected()) {
        DEBUG_PRINTLN("[APP] Client déconnecté de port 4534");
        appClient.stop();
        appJsonIdx = 0;
    }
}

#endif // ENABLE_APP_TCP

// ════════════════════════════════════════════════════════════════
// BOUCLE RÉSEAU (appelée dans loop)
// ════════════════════════════════════════════════════════════════

void networkLoop() {
    if (!ethConnected) return;

    // Maintenir le bail DHCP (toutes les 60s, uniquement si DHCP actif)
    // En mode IP statique, maintain() bloque ~8s pour rien
    if (useDHCP) {
        static unsigned long lastMaintain = 0;
        if (millis() - lastMaintain > 60000) {
            Ethernet.maintain();
            lastMaintain = millis();
        }
    }

    // Traiter les serveurs TCP
    handleTcpClient(easycomServer, easycomClient, easycomState, "easycom");

    #if ENABLE_APP_TCP
        handleAppTcpClient();
    #else
        handleTcpClient(appServer, appClient, appState, "app");
    #endif
}

bool networkIsConnected() {
    return ethConnected;
}

String networkGetIP() {
    if (!ethConnected) return "N/A";
    IPAddress ip = Ethernet.localIP();
    return String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(ip[3]);
}

bool appTcpConnected() {
    #if ENABLE_APP_TCP
        return appClient && appClient.connected();
    #else
        return false;
    #endif
}

void appSendStatus(float az, float el, float tgtAz, float tgtEl,
                   const char *state, bool moving, bool stopBtn,
                   bool gpsFix, bool stale) {
    #if ENABLE_APP_TCP
    if (!appClient || !appClient.connected()) return;

    char buf[384];
    int n = snprintf(buf, sizeof(buf),
        "{\"type\":\"status\","
        "\"az\":%.1f,\"el\":%.1f,"
        "\"az_target\":%.1f,\"el_target\":%.1f,"
        "\"state\":\"%s\","
        "\"moving\":%s,"
        "\"stop_pressed\":%s,"
        "\"gps_fix\":%s,"
        "\"stale\":%s",
        az, el, tgtAz, tgtEl, state,
        moving ? "true" : "false",
        stopBtn ? "true" : "false",
        gpsFix ? "true" : "false",
        stale ? "true" : "false");

    // Ajouter télémétrie Nano R4 si RS-485 actif
    #if ENABLE_RS485
    n += snprintf(buf + n, sizeof(buf) - n,
        ",\"nano\":{\"online\":%s,\"a0_mv\":%u,\"a1_mv\":%u,\"uptime\":%u}",
        nanoOnline ? "true" : "false",
        nanoA0mV, nanoA1mV, nanoUptime);
    #endif

    n += snprintf(buf + n, sizeof(buf) - n, "}\n");
    appClient.write((const uint8_t*)buf, n);
    #else
    (void)az; (void)el; (void)tgtAz; (void)tgtEl;
    (void)state; (void)moving; (void)stopBtn; (void)gpsFix; (void)stale;
    #endif
}

#endif // ETHERNET_ENABLED
