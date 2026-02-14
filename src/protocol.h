#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>

// Types de commandes reçues par le parser
typedef enum {
    CMD_NONE,           // Pas de commande (ligne vide ou incomplète)
    CMD_QUERY_POS,      // Demande de position (C2 / AZ / EL / AZ EL)
    CMD_GOTO_AZ,        // Aller à un azimut cible
    CMD_GOTO_AZEL,      // Aller à un azimut + élévation cible
    CMD_STOP_ALL,       // Arrêt de tous les moteurs
    CMD_STOP_AZ,        // Arrêt azimut seul
    CMD_STOP_EL,        // Arrêt élévation seule
    CMD_JOG_CW,         // Rotation continue CW (droite)
    CMD_JOG_CCW,        // Rotation continue CCW (gauche)
    CMD_JOG_UP,         // Élévation continue UP
    CMD_JOG_DOWN,       // Élévation continue DOWN
    CMD_VERSION,        // Demande de version
    CMD_UNKNOWN         // Commande non reconnue
} ProtocolCommand;

// Résultat du parsing d'une ligne
typedef struct {
    ProtocolCommand cmd;
    float az;           // Azimut cible (si CMD_GOTO_AZ ou CMD_GOTO_AZEL)
    float el;           // Élévation cible (si CMD_GOTO_AZEL)
} ParsedCommand;

// Initialise le module protocole
void protocolInit();

// Lit les caractères disponibles sur un Stream (Serial, TCP client, etc.)
// Retourne true si une commande complète a été parsée
bool protocolProcessStream(Stream &input, ParsedCommand &result);

// Envoie la réponse de position sur le Stream de sortie
void protocolSendPosition(Stream &output, float az, float el);

// Envoie la réponse de version sur le Stream de sortie
void protocolSendVersion(Stream &output);

// Retourne le nom du protocole actif ("GS-232B" ou "EasyCom II")
const char* protocolGetName();

#endif
