# CLAUDE.md — ESP32 Rotator Controller Project

## Contexte du projet

Ce projet est un contrôleur de rotor d'antenne basé sur un ESP32. Il communique avec le logiciel **PstRotator** via le protocole **EasyCom**, lit la position angulaire de deux encodeurs absolus **HH-12** (azimut + élévation), et se connecte au réseau via un module **Ethernet W5500**.

Le fichier de spécifications complet se trouve dans `/docs/specifications.md` (ou le nom du fichier uploadé par l'utilisateur). **Lis toujours ce fichier avant de commencer une étape.**

---

## Architecture du projet

### Structure des fichiers

```
rotator-controller/
├── CLAUDE.md                # CE FICHIER — instructions pour Claude Code
├── TODO.md                  # Checklist de progression
├── platformio.ini           # Configuration PlatformIO
├── docs/
│   └── specifications.md    # Document de spécifications complet (uploadé par l'utilisateur)
├── src/
│   ├── main.cpp             # Setup/loop, orchestration générale
│   ├── config.h             # Définitions de pins, constantes, paramètres
│   ├── easycom.h            # Header protocole EasyCom
│   ├── easycom.cpp          # Implémentation protocole EasyCom
│   ├── encoder.h            # Header gestion encodeurs HH-12
│   ├── encoder.cpp          # Implémentation lecture encodeurs HH-12
│   ├── network.h            # Header Ethernet W5500
│   └── network.cpp          # Implémentation Ethernet W5500
└── test/                    # Tests unitaires (optionnel)
```

### Principes de code

- **Modulaire** : un fichier .h/.cpp par fonctionnalité, main.cpp ne fait qu'orchestrer
- **config.h centralise** toutes les définitions de pins et constantes
- **Pas de delay() bloquant** dans la loop principale — utiliser millis() pour le timing
- **Serial pour le debug** : toujours garder des messages de debug activables via un #define DEBUG
- **Commentaires en français** sauf les noms de fonctions/variables (en anglais)

---

## Plan de développement — Étapes séquentielles

Chaque étape correspond à une branche Git. **Ne passe à l'étape suivante que quand l'utilisateur confirme que l'étape actuelle fonctionne sur le hardware.**

### Étape 0 : Initialisation du projet

**Branche : `main`**

Actions à effectuer :
1. Initialiser un projet PlatformIO pour ESP32 dans le dossier courant
2. Configurer `platformio.ini` pour la carte ESP32 utilisée (demander le modèle exact à l'utilisateur si pas dans les specs)
3. Créer la structure de dossiers (`src/`, `docs/`, `test/`)
4. Créer `config.h` avec les définitions de base
5. Créer `TODO.md` avec la checklist complète
6. Faire un premier commit

**platformio.ini de base :**
```ini
[env:esp32]
platform = espressif32
board = esp32dev          ; À adapter selon la carte réelle
framework = arduino
monitor_speed = 115200
lib_deps =
    ; Les dépendances seront ajoutées au fur et à mesure
```

**config.h de base :**
```cpp
#ifndef CONFIG_H
#define CONFIG_H

// === Debug ===
#define DEBUG 1
#if DEBUG
  #define DEBUG_PRINT(x)   Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// === Pins — À COMPLÉTER SELON LES SPECS ===

// LED
#define LED_PIN 2  // LED intégrée ESP32 (vérifier selon la carte)

// Encodeur HH-12 (SPI)
// #define HH12_CLK_PIN   xx
// #define HH12_MISO_PIN  xx
// #define HH12_CS_AZ_PIN xx   // Chip Select encodeur azimut
// #define HH12_CS_EL_PIN xx   // Chip Select encodeur élévation

// Ethernet W5500 (SPI)
// #define W5500_CS_PIN   xx
// #define W5500_RST_PIN  xx
// #define W5500_INT_PIN  xx

// === Paramètres réseau ===
// #define EASYCOM_TCP_PORT 12000

// === Paramètres rotor ===
// #define AZ_MIN 0.0
// #define AZ_MAX 360.0
// #define EL_MIN 0.0
// #define EL_MAX 90.0

#endif
```

---

### Étape 1 : Blink LED

**Branche : `feature/blink`**

Objectif : valider que la chaîne de compilation → upload → exécution fonctionne.

Actions :
1. Créer `git checkout -b feature/blink`
2. Écrire un main.cpp minimal qui fait clignoter la LED intégrée
3. Utiliser `millis()` et non `delay()` (bonne habitude dès le départ)
4. Ajouter des messages Serial pour confirmer que le debug fonctionne
5. Compiler et vérifier qu'il n'y a pas d'erreur

**Critère de validation (par l'utilisateur) :** LED clignote + messages Serial visibles dans le moniteur.

Quand validé : `git checkout main && git merge feature/blink`

---

### Étape 2 : Protocole EasyCom (série)

**Branche : `feature/easycom`**

Objectif : implémenter le parsing du protocole EasyCom pour recevoir/envoyer des commandes de position.

Le protocole EasyCom utilise des commandes texte simples. **Consulter le document de spécifications pour les détails exacts du protocole.**

Commandes EasyCom typiques :
- `AZxxx.x` — positionner l'azimut
- `ELxxx.x` — positionner l'élévation
- `AZ EL` — demander la position actuelle (réponse : `AZxxx.x ELxxx.x`)

Actions :
1. Créer `easycom.h` et `easycom.cpp`
2. Implémenter le parsing des commandes entrantes (depuis Serial pour commencer)
3. Implémenter les réponses
4. Dans main.cpp, intégrer la boucle EasyCom
5. Pour cette étape, utiliser des valeurs d'angle simulées (variables globales)

**Critère de validation :** envoyer des commandes via le moniteur série et recevoir des réponses correctes.

Quand validé : merge dans main.

---

### Étape 3 : Un encodeur HH-12

**Branche : `feature/hh12-single`**

Objectif : lire l'angle d'un encodeur absolu HH-12 via SPI.

**Consulter les specs pour le protocole SPI du HH-12** (bit-banging ou SPI hardware, format des données 12 bits, etc.)

Actions :
1. Créer `encoder.h` et `encoder.cpp`
2. Implémenter une classe/struct `Encoder` avec init() et readAngle()
3. Connecter un seul encodeur (azimut)
4. Afficher l'angle en continu sur le Serial
5. Intégrer avec EasyCom : la commande `AZ EL` retourne maintenant l'angle réel de l'encodeur pour AZ

**Critère de validation :** tourner l'encodeur et voir l'angle changer correctement (0-360°).

Quand validé : merge dans main.

---

### Étape 4 : Deux encodeurs HH-12

**Branche : `feature/hh12-dual`**

Objectif : lire deux encodeurs sur le même bus SPI avec des CS (Chip Select) différents.

Actions :
1. Ajouter le deuxième encodeur (élévation) avec son propre CS pin
2. Vérifier qu'il n'y a pas de conflit SPI entre les deux
3. Intégrer avec EasyCom : `AZ EL` retourne maintenant les deux angles réels

**Critère de validation :** les deux encodeurs lisent correctement et indépendamment.

Quand validé : merge dans main.

---

### Étape 5 : Ethernet W5500

**Branche : `feature/ethernet-w5500`**

Objectif : connecter le W5500, obtenir une IP, et faire passer EasyCom sur TCP au lieu de Serial.

**Attention au SPI partagé** entre W5500 et les encodeurs HH-12. Vérifier la compatibilité dans les specs.

Actions :
1. Créer `network.h` et `network.cpp`
2. Ajouter la bibliothèque Ethernet dans platformio.ini
3. Implémenter l'initialisation W5500 (DHCP ou IP statique selon les specs)
4. Créer un serveur TCP sur le port EasyCom
5. Modifier easycom.cpp pour accepter les commandes depuis TCP ET Serial
6. Gérer la cohabitation SPI entre W5500 et encodeurs HH-12

**Critère de validation :** PstRotator se connecte en TCP et contrôle le rotor.

Quand validé : merge dans main.

---

## Commandes Git à utiliser

```bash
# Voir l'état actuel
git status

# Créer une nouvelle branche pour une étape
git checkout -b feature/nom-etape

# Commiter le travail en cours
git add .
git commit -m "description claire du changement"

# Pousser vers GitHub
git push origin feature/nom-etape

# Merger une étape validée
git checkout main
git merge feature/nom-etape
git push origin main
```

---

## Instructions pour Claude Code

### À chaque nouvelle session

1. **Lis ce fichier** (CLAUDE.md)
2. **Lis le document de spécifications** dans `/docs/`
3. **Lis TODO.md** pour savoir où on en est
4. **Vérifie la branche Git actuelle** (`git branch`)
5. **Lis le code existant** dans `src/` avant de proposer des modifications

### Règles de travail

- **Ne saute jamais une étape.** Demande toujours à l'utilisateur si l'étape précédente est validée sur le hardware.
- **Propose le code complet**, pas des extraits partiels. L'utilisateur doit pouvoir compiler directement.
- **Mets à jour TODO.md** après chaque changement significatif.
- **Commite régulièrement** avec des messages clairs en français.
- **En cas de doute sur un pin ou un paramètre**, demande à l'utilisateur plutôt que de deviner.
- **Vérifie que le code compile** (`pio run`) avant de dire que c'est prêt.
- **Quand tu modifies config.h**, montre toujours le résumé des pins utilisées.

### En cas de problème de compilation

1. Lis l'erreur complète
2. Vérifie les #include manquants
3. Vérifie les dépendances dans platformio.ini
4. Propose un fix et recompile

### En cas de problème hardware (rapporté par l'utilisateur)

1. Propose des étapes de diagnostic (messages debug, test de continuité, etc.)
2. Vérifie les niveaux logiques (3.3V vs 5V)
3. Vérifie les conflits de pins/SPI
4. Propose un code de test minimal pour isoler le problème

---

## Checklist de démarrage

Quand l'utilisateur te donne ce fichier pour la première fois, fais ceci :

1. [ ] Demande quel modèle exact d'ESP32 est utilisé (ESP32-DevKitC, ESP32-WROOM, etc.)
2. [ ] Demande si le document de spécifications est déjà dans `/docs/`. Si non, demande à l'utilisateur de le placer là.
3. [ ] Lis le document de spécifications
4. [ ] Crée la structure du projet (dossiers, fichiers de base)
5. [ ] Crée le TODO.md initial
6. [ ] Fais le premier commit
7. [ ] Passe à l'Étape 1 (Blink)
