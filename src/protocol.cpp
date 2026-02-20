// ════════════════════════════════════════════════════════════════
// EME ROTATOR CONTROLLER — Protocole EasyCom II (Implementation)
// ════════════════════════════════════════════════════════════════
// Format position : AZxxx.x ELxxx.x (1 décimale, extensible à 2)
// Commandes : AZxxx.x ELyyy.y, AZ, EL, S, SA, SE, VE, MR/ML/MU/MD
// ════════════════════════════════════════════════════════════════

#include "protocol.h"
#include "config.h"

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
    if (*start == '\0' || (*start != '-' && *start != '+' && *start != '.'
        && !(*start >= '0' && *start <= '9'))) {
        return false;  // Pas de valeur = query
    }

    value = atof(start);
    return true;
}

// ─────────────────────────────────────────────────
// Parser EasyCom II
// ─────────────────────────────────────────────────

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

    // ── Commandes STOP (priorité absolue) ──
    if (strcmp(line, "S") == 0 || strcmp(line, "STOP") == 0 || strcmp(line, ";") == 0) {
        result.cmd = CMD_STOP_ALL;
        return result;
    }

    // SA, SE, SASE, SESA → stop
    if (line[0] == 'S' && len >= 2 && (line[1] == 'A' || line[1] == 'E')) {
        result.cmd = CMD_STOP_ALL;
        return result;
    }

    // ── Query position : "AZ" ou "EL" seuls (sans valeur) ──
    if (strcmp(line, "AZ") == 0 || strcmp(line, "EL") == 0 || strcmp(line, "AZ EL") == 0) {
        result.cmd = CMD_QUERY_POS;
        return result;
    }

    // ── Commandes GOTO : parser AZ et EL indépendamment ──
    // PSTRotator envoie "AZ123.5 EL45.0" sur une seule ligne
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

// ─────────────────────────────────────────────────
// Envoi de réponses EasyCom II
// ─────────────────────────────────────────────────

void protocolSendPosition(Stream &output, float az, float el) {
    // Format EasyCom II : AZxxx.x ELxxx.x\r\n (1 décimale)
    char buf[30];
    snprintf(buf, sizeof(buf), "AZ%.1f EL%.1f\r\n", az, el);
    output.print(buf);
}

void protocolSendVersion(Stream &output) {
    output.print("VE002\r\n");  // EasyCom version 2
}

// ─────────────────────────────────────────────────
// Fonctions communes
// ─────────────────────────────────────────────────

void protocolStateInit(ProtocolState &state) {
    state.index = 0;
    state.buffer[0] = '\0';
}

void protocolInit() {
    DEBUG_PRINTLN("[INIT] Protocole : EasyCom II (1 décimale)");
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

            result = parseEasyCom(state.buffer);

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
