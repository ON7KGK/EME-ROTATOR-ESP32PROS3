#include "protocol.h"
#include "config.h"

// Vérification qu'un seul protocole est défini
#if defined(PROTOCOL_GS232) && defined(PROTOCOL_EASYCOM)
  #error "Définir un seul protocole : PROTOCOL_GS232 ou PROTOCOL_EASYCOM"
#endif
#if !defined(PROTOCOL_GS232) && !defined(PROTOCOL_EASYCOM)
  #error "Aucun protocole défini : activer PROTOCOL_GS232 ou PROTOCOL_EASYCOM dans config.h"
#endif

// ─────────────────────────────────────────────────
// Fonctions utilitaires
// ─────────────────────────────────────────────────

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

void protocolSendAzimuth(Stream &output, float az) {
    // GS-232B n'a pas de requête AZ seule, envoyer la position complète
    protocolSendPosition(output, az, 0.0f);
}

void protocolSendElevation(Stream &output, float el) {
    // GS-232B n'a pas de requête EL seule, envoyer la position complète
    protocolSendPosition(output, 0.0f, el);
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

// Extraire valeur numérique après un mot-clé (AZ, EL) dans la ligne
// Retourne true si une valeur a été trouvée
static bool extractValue(const char *line, const char *keyword, float &value) {
    const char *pos = strstr(line, keyword);
    if (!pos) return false;

    int keyLen = strlen(keyword);
    const char *start = pos + keyLen;

    // Sauter les espaces entre le mot-clé et la valeur (ex: "AZ 270.00")
    while (*start == ' ') start++;

    // Vérifier qu'il y a un chiffre, signe ou point après le mot-clé
    if (*start == '\0' || (*start != '-' && *start != '+' && *start != '.' && !(*start >= '0' && *start <= '9'))) {
        return false;  // Pas de valeur = query
    }

    value = atof(start);
    return true;
}

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

    // ── Commandes STOP (priorité absolue, style K3NG) ──
    // S, STOP, ; → stop all
    if (strcmp(line, "S") == 0 || strcmp(line, "STOP") == 0 || strcmp(line, ";") == 0) {
        result.cmd = CMD_STOP_ALL;
        return result;
    }
    // SA (seul ou suivi de SE), SE (seul ou suivi de SA)
    if (strstr(line, "SA") || strstr(line, "SE")) {
        // Vérifier que ce n'est pas un début de commande goto (ex: ligne contenant "AZ...SE...")
        if (line[0] == 'S') {
            result.cmd = CMD_STOP_ALL;
            return result;
        }
    }

    // ── Query position : "AZ" ou "EL" seuls (sans valeur) ──
    if (strcmp(line, "AZ") == 0 || strcmp(line, "EL") == 0 || strcmp(line, "AZ EL") == 0) {
        result.cmd = CMD_QUERY_POS;
        return result;
    }

    // ── Commandes GOTO : parser AZ et EL indépendamment ──
    // PSTRotator peut envoyer "AZ123.5 EL45.0" sur une seule ligne
    bool hasAz = false, hasEl = false;
    float azVal = 0.0f, elVal = 0.0f;

    hasAz = extractValue(line, "AZ", azVal);
    hasEl = extractValue(line, "EL", elVal);

    if (hasAz && hasEl) {
        result.az = azVal;
        result.el = elVal;
        result.cmd = CMD_GOTO_AZEL;
        return result;
    }
    if (hasAz) {
        result.az = azVal;
        result.cmd = CMD_GOTO_AZ;
        return result;
    }
    if (hasEl) {
        result.el = elVal;
        result.cmd = CMD_GOTO_EL;
        return result;
    }

    // ── Commandes Jog ──
    if (strcmp(line, "MR") == 0) { result.cmd = CMD_JOG_CW;   return result; }
    if (strcmp(line, "ML") == 0) { result.cmd = CMD_JOG_CCW;  return result; }
    if (strcmp(line, "MU") == 0) { result.cmd = CMD_JOG_UP;   return result; }
    if (strcmp(line, "MD") == 0) { result.cmd = CMD_JOG_DOWN; return result; }

    result.cmd = CMD_UNKNOWN;
    return result;
}

void protocolSendPosition(Stream &output, float az, float el) {
    // Format EasyCom II : AZxxx.x ELxxx.x\r\n (1 décimale, comme SVH3/K3NG)
    char buf[30];
    snprintf(buf, sizeof(buf), "AZ%.1f EL%.1f\r\n", az, el);
    output.print(buf);
}

void protocolSendAzimuth(Stream &output, float az) {
    // EasyCom : toujours répondre avec position combinée (pas utilisé)
    protocolSendPosition(output, az, 0.0f);
}

void protocolSendElevation(Stream &output, float el) {
    // EasyCom : toujours répondre avec position combinée (pas utilisé)
    protocolSendPosition(output, 0.0f, el);
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

void protocolStateInit(ProtocolState &state) {
    state.index = 0;
    state.buffer[0] = '\0';
}

void protocolInit() {
    DEBUG_PRINT("Protocole actif : ");
    DEBUG_PRINTLN(protocolGetName());
}

bool protocolProcessStream(Stream &input, ProtocolState &state, ParsedCommand &result) {
    while (input.available()) {
        char c = input.read();

        // Fin de ligne = commande complète
        if (c == '\n' || c == '\r') {
            if (state.index == 0) continue;  // Ignorer les lignes vides

            state.buffer[state.index] = '\0';

            DEBUG_PRINT("[RX] \"");
            DEBUG_PRINT(state.buffer);
            DEBUG_PRINTLN("\"");

            #ifdef PROTOCOL_GS232
                result = parseGS232(state.buffer);
            #endif
            #ifdef PROTOCOL_EASYCOM
                result = parseEasyCom(state.buffer);
            #endif

            // Reset du buffer pour la prochaine commande
            state.index = 0;
            state.buffer[0] = '\0';
            return (result.cmd != CMD_NONE);
        }

        // Ajouter le caractère au buffer
        if (state.index < RX_BUFFER_SIZE - 1) {
            state.buffer[state.index++] = c;
        } else {
            // Buffer plein, reset
            DEBUG_PRINTLN("WARN: buffer protocole plein, reset");
            state.index = 0;
            state.buffer[0] = '\0';
        }
    }
    return false;
}
