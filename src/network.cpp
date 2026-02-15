// ════════════════════════════════════════════════════════════════
// EME ROTATOR CONTROLLER — Module Ethernet W5500 (Implementation)
// ════════════════════════════════════════════════════════════════
// W5500 (Adafruit 3201) sur SPI3 du ProS3
// SPI3 : SCLK=IO12, MISO=IO13, MOSI=IO14, CS=IO15
// DHCP avec fallback IP statique
// 2 serveurs TCP : port 4533 (rotateur) + port 4534 (app)
// ════════════════════════════════════════════════════════════════

#include "config.h"

#if ETHERNET_ENABLED

#include "network.h"
#include "protocol.h"

#include <SPI.h>
#include <Ethernet.h>

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
static W5500Server rotatorServer(ROTATOR_TCP_PORT);
static W5500Server appServer(APP_TCP_PORT);

// Clients TCP actuels (1 par port)
static EthernetClient rotatorClient;
static EthernetClient appClient;

// État du parser pour chaque client TCP
static ProtocolState rotatorState;
static ProtocolState appState;

// handleCommand est défini dans main.cpp
extern void handleCommand(ParsedCommand &cmd, Stream &output);

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
    protocolStateInit(rotatorState);
    protocolStateInit(appState);

    // Démarrer les serveurs TCP
    rotatorServer.begin();
    appServer.begin();

    DEBUG_PRINT("Serveur TCP rotateur sur port ");
    DEBUG_PRINTLN(ROTATOR_TCP_PORT);
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

    // Traiter les deux serveurs TCP
    handleTcpClient(rotatorServer, rotatorClient, rotatorState, "rotateur");
    handleTcpClient(appServer,   appClient,   appState,   "app");
}

bool networkIsConnected() {
    return ethConnected;
}

String networkGetIP() {
    if (!ethConnected) return "N/A";
    IPAddress ip = Ethernet.localIP();
    return String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(ip[3]);
}

#endif // ETHERNET_ENABLED
