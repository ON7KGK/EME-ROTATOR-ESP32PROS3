// ════════════════════════════════════════════════════════════════
// EME ROTATOR CONTROLLER — Module Ethernet W5500
// ════════════════════════════════════════════════════════════════
// W5500 (Adafruit 3201) sur SPI3 du ProS3
// DHCP avec fallback IP statique
// 2 serveurs TCP : port 4533 (rotateur) + port 4534 (app)
// ════════════════════════════════════════════════════════════════

#ifndef NETWORK_H
#define NETWORK_H

#include <Arduino.h>

// Initialise le module Ethernet W5500 (SPI3)
// DHCP avec fallback sur IP statique définie dans config.h
void networkInit();

// Boucle réseau : accepte les connexions TCP, lit les commandes
// Doit être appelée dans loop()
void networkLoop();

// Retourne true si Ethernet est connecté et a une IP
bool networkIsConnected();

// Retourne l'adresse IP sous forme de String (pour debug/OLED)
String networkGetIP();

#endif // NETWORK_H
