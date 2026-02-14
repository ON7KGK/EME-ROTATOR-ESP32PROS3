#include "protocol.h"
#include "config.h"

// Vérification qu'un seul protocole est défini
#if defined(PROTOCOL_GS232) && defined(PROTOCOL_EASYCOM)
  #error "Définir un seul protocole : PROTOCOL_GS232 ou PROTOCOL_EASYCOM"
#endif
#if !defined(PROTOCOL_GS232) && !defined(PROTOCOL_EASYCOM)
  #error "Aucun protocole défini : activer PROTOCOL_GS232 ou PROTOCOL_EASYCOM dans config.h"
#endif

// Buffer de réception (partagé, un par Stream géré)
#define RX_BUFFER_SIZE 64
static char rxBuffer[RX_BUFFER_SIZE];
static uint8_t rxIndex = 0;

// ─────────────────────────────────────────────────
// Fonctions utilitaires
// ─────────────────────────────────────────────────

static void resetBuffer() {
    rxIndex = 0;
    rxBuffer[0] = '\0';
}

static void trimLine(char *line) {
    // Retirer \r \n et espaces en fin de ligne
    int len = strlen(line);
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n' || line[len - 1] == ' ')) {
        line[--len] = '\0';
    }
}

static void toUpper(char *str) {
    for (int i = 0; str[i]; i++) {
        if (str[i] >= 'a' && str[i] <= 'z') {
            str[i] -= 32;
        }
    }
}

// ─────────────────────────────────────────────────
// Parser GS-232B
// ─────────────────────────────────────────────────

#ifdef PROTOCOL_GS232

static ParsedCommand parseGS232(char *line) {
    ParsedCommand result = { CMD_NONE, 0.0f, 0.0f };

    trimLine(line);
    toUpper(line);

    int len = strlen(line);
    if (len == 0) return result;

    // C2 — Demande de position AZ + EL
    if (strcmp(line, "C2") == 0) {
        result.cmd = CMD_QUERY_POS;
        return result;
    }

    // C — Demande de position AZ seul (GS-232A compat)
    if (strcmp(line, "C") == 0) {
        result.cmd = CMD_QUERY_POS;
        return result;
    }

    // S — Arrêt tous moteurs
    if (strcmp(line, "S") == 0) {
        result.cmd = CMD_STOP_ALL;
        return result;
    }

    // A — Arrêt azimut
    if (strcmp(line, "A") == 0) {
        result.cmd = CMD_STOP_AZ;
        return result;
    }

    // E — Arrêt élévation
    if (strcmp(line, "E") == 0) {
        result.cmd = CMD_STOP_EL;
        return result;
    }

    // R — Rotation continue CW
    if (strcmp(line, "R") == 0) {
        result.cmd = CMD_JOG_CW;
        return result;
    }

    // L — Rotation continue CCW
    if (strcmp(line, "L") == 0) {
        result.cmd = CMD_JOG_CCW;
        return result;
    }

    // U — Élévation continue UP
    if (strcmp(line, "U") == 0) {
        result.cmd = CMD_JOG_UP;
        return result;
    }

    // D — Élévation continue DOWN
    if (strcmp(line, "D") == 0) {
        result.cmd = CMD_JOG_DOWN;
        return result;
    }

    // Mxxx — Pointer azimut (3 chiffres)
    if (line[0] == 'M' && len >= 4) {
        float az = atof(&line[1]);
        result.cmd = CMD_GOTO_AZ;
        result.az = az;
        return result;
    }

    // Wxxx yyy — Pointer AZ + EL
    if (line[0] == 'W' && len >= 7) {
        // Format : Wxxx yyy  ou  Wxxx.x yyy.y
        char *space = strchr(&line[1], ' ');
        if (space) {
            *space = '\0';
            result.az = atof(&line[1]);
            result.el = atof(space + 1);
            result.cmd = CMD_GOTO_AZEL;
            return result;
        }
    }

    result.cmd = CMD_UNKNOWN;
    return result;
}

void protocolSendPosition(Stream &output, float az, float el) {
    // Format GS-232B : +0xxx+0yyy\r\n
    // AZ sur 3 chiffres entiers (000-450), EL sur 3 chiffres (000-180)
    char buf[20];
    snprintf(buf, sizeof(buf), "+0%03d+0%03d\r\n", (int)(az + 0.5f), (int)(el + 0.5f));
    output.print(buf);
}

void protocolSendVersion(Stream &output) {
    output.print("GS-232B ON7KGK\r\n");
}

const char* protocolGetName() {
    return "GS-232B";
}

#endif // PROTOCOL_GS232

// ─────────────────────────────────────────────────
// Parser EasyCom II
// ─────────────────────────────────────────────────

#ifdef PROTOCOL_EASYCOM

static ParsedCommand parseEasyCom(char *line) {
    ParsedCommand result = { CMD_NONE, 0.0f, 0.0f };

    trimLine(line);
    toUpper(line);

    int len = strlen(line);
    if (len == 0) return result;

    // VE — Version
    if (strcmp(line, "VE") == 0) {
        result.cmd = CMD_VERSION;
        return result;
    }

    // SA — Stop azimut
    if (strcmp(line, "SA") == 0) {
        result.cmd = CMD_STOP_AZ;
        return result;
    }

    // SE — Stop élévation
    if (strcmp(line, "SE") == 0) {
        result.cmd = CMD_STOP_EL;
        return result;
    }

    // AZ — Requête ou commande azimut
    if (strncmp(line, "AZ", 2) == 0) {
        if (len == 2) {
            // "AZ" seul = demande de position
            result.cmd = CMD_QUERY_POS;
            return result;
        }
        // "AZxxx.x" = commande goto azimut
        result.az = atof(&line[2]);
        result.cmd = CMD_GOTO_AZ;
        return result;
    }

    // EL — Requête ou commande élévation
    if (strncmp(line, "EL", 2) == 0) {
        if (len == 2) {
            // "EL" seul = demande de position
            result.cmd = CMD_QUERY_POS;
            return result;
        }
        // "ELxxx.x" = commande goto élévation (traité comme AZEL avec AZ inchangé)
        result.el = atof(&line[2]);
        result.cmd = CMD_GOTO_AZEL;
        return result;
    }

    // MR — Jog CW (Move Right)
    if (strcmp(line, "MR") == 0) {
        result.cmd = CMD_JOG_CW;
        return result;
    }

    // ML — Jog CCW (Move Left)
    if (strcmp(line, "ML") == 0) {
        result.cmd = CMD_JOG_CCW;
        return result;
    }

    // MU — Jog UP (Move Up)
    if (strcmp(line, "MU") == 0) {
        result.cmd = CMD_JOG_UP;
        return result;
    }

    // MD — Jog DOWN (Move Down)
    if (strcmp(line, "MD") == 0) {
        result.cmd = CMD_JOG_DOWN;
        return result;
    }

    result.cmd = CMD_UNKNOWN;
    return result;
}

void protocolSendPosition(Stream &output, float az, float el) {
    // Format EasyCom II : AZxxx.x ELxxx.x\r\n
    char buf[30];
    snprintf(buf, sizeof(buf), "AZ%.1f EL%.1f\r\n", az, el);
    output.print(buf);
}

void protocolSendVersion(Stream &output) {
    output.print("VE002\r\n");  // EasyCom version 2
}

const char* protocolGetName() {
    return "EasyCom II";
}

#endif // PROTOCOL_EASYCOM

// ─────────────────────────────────────────────────
// Fonctions communes
// ─────────────────────────────────────────────────

void protocolInit() {
    resetBuffer();
    DEBUG_PRINT("Protocole actif : ");
    DEBUG_PRINTLN(protocolGetName());
}

bool protocolProcessStream(Stream &input, ParsedCommand &result) {
    while (input.available()) {
        char c = input.read();

        // Fin de ligne = commande complète
        if (c == '\n' || c == '\r') {
            if (rxIndex == 0) continue;  // Ignorer les lignes vides

            rxBuffer[rxIndex] = '\0';

            #ifdef PROTOCOL_GS232
                result = parseGS232(rxBuffer);
            #endif
            #ifdef PROTOCOL_EASYCOM
                result = parseEasyCom(rxBuffer);
            #endif

            resetBuffer();
            return (result.cmd != CMD_NONE);
        }

        // Ajouter le caractère au buffer
        if (rxIndex < RX_BUFFER_SIZE - 1) {
            rxBuffer[rxIndex++] = c;
        } else {
            // Buffer plein, reset
            DEBUG_PRINTLN("WARN: buffer protocole plein, reset");
            resetBuffer();
        }
    }
    return false;
}
