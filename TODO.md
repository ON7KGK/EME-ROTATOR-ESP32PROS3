# TODO — EME Rotator Controller (ESP32 ProS3)

## Étape 0 : Initialisation du projet
- [x] Créer le repo GitHub
- [x] Initialiser le projet PlatformIO (um_pros3)
- [x] Créer la structure de dossiers (src/, docs/, test/)
- [x] Créer config.h avec les définitions de base
- [x] Créer CLAUDE.md
- [x] Créer ce TODO.md
- [ ] Ajouter le document de spécifications dans /docs/
- [ ] Premier commit

## Étape 1 : Blink LED
**Branche : `feature/blink`**
- [ ] Créer la branche feature/blink
- [ ] Écrire main.cpp avec clignotement LED (millis(), pas delay())
- [ ] Ajouter messages Serial de debug
- [ ] Compiler sans erreur (pio run)
- [ ] **Validation hardware** : LED clignote + messages Serial visibles

## Étape 2 : Protocole EasyCom (série)
**Branche : `feature/easycom`**
- [ ] Créer easycom.h et easycom.cpp
- [ ] Implémenter parsing des commandes (AZ, EL, AZ EL)
- [ ] Implémenter les réponses
- [ ] Intégrer dans main.cpp (boucle EasyCom)
- [ ] Tester avec valeurs simulées
- [ ] **Validation hardware** : commandes série fonctionnelles

## Étape 3 : Un encodeur HH-12
**Branche : `feature/hh12-single`**
- [ ] Créer encoder.h et encoder.cpp
- [ ] Implémenter lecture SPI d'un encodeur HH-12
- [ ] Afficher angle en continu sur Serial
- [ ] Intégrer avec EasyCom (AZ réel)
- [ ] **Validation hardware** : angle change correctement (0-360°)

## Étape 4 : Deux encodeurs HH-12
**Branche : `feature/hh12-dual`**
- [ ] Ajouter deuxième encodeur (élévation) avec CS séparé
- [ ] Vérifier absence de conflit SPI
- [ ] Intégrer avec EasyCom (AZ + EL réels)
- [ ] **Validation hardware** : deux encodeurs indépendants et corrects

## Étape 5 : Ethernet W5500
**Branche : `feature/ethernet-w5500`**
- [ ] Créer network.h et network.cpp
- [ ] Ajouter bibliothèque Ethernet dans platformio.ini
- [ ] Implémenter initialisation W5500 (DHCP ou IP statique)
- [ ] Créer serveur TCP sur port EasyCom
- [ ] Modifier easycom.cpp pour TCP + Serial
- [ ] Gérer cohabitation SPI (W5500 + encodeurs)
- [ ] **Validation hardware** : PstRotator se connecte en TCP
