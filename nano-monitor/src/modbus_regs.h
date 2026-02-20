#ifndef MODBUS_REGS_H
#define MODBUS_REGS_H

// ════════════════════════════════════════════════════════════════
// EME ROTATOR — Registres Modbus (partagé entre UNO R4 et ESP32)
// ════════════════════════════════════════════════════════════════
// UNO R4 Motor Box = Slave ID 2
// ESP32 = Master
// ════════════════════════════════════════════════════════════════

// ═══════ Input Registers (FC 04, lecture seule) ═══════
// Le slave (UNO R4) met à jour ces registres, le master (ESP32) les lit
#define IREG_HH12_AZ_RAW        0   // Position brute AZ (0-4095)
#define IREG_HH12_EL_RAW        1   // Position brute EL (0-4095)
#define IREG_HH12_AZ_ANGLE_X10  2   // Angle AZ × 10 (ex: 1805 = 180.5°)
#define IREG_HH12_EL_ANGLE_X10  3   // Angle EL × 10
#define IREG_HALL_AZ_HI          4   // Compteur Hall AZ [31:16]
#define IREG_HALL_AZ_LO          5   // Compteur Hall AZ [15:0]
#define IREG_HALL_EL_HI          6   // Compteur Hall EL [31:16]
#define IREG_HALL_EL_LO          7   // Compteur Hall EL [15:0]
#define IREG_MOT_AZ_FB_RAW      8   // ADC brut courant AZ
#define IREG_MOT_EL_FB_RAW      9   // ADC brut courant EL
#define IREG_STATUS             10   // Bitmap status (voir ci-dessous)
#define IREG_LOOP_TIME_US       11   // Temps de boucle en µs

#define NUM_INPUT_REGS          12

// ═══════ STATUS bitmap (Input Register 10) ═══════
#define STATUS_LIMIT_CW      (1 << 0)   // Fin de course CW atteint
#define STATUS_LIMIT_CCW     (1 << 1)   // Fin de course CCW atteint
#define STATUS_LIMIT_UP      (1 << 2)   // Fin de course UP atteint
#define STATUS_LIMIT_DOWN    (1 << 3)   // Fin de course DOWN atteint
#define STATUS_STOP_ACTIVE   (1 << 4)   // Bouton STOP pressé
#define STATUS_SF_FAULT      (1 << 5)   // MC33926 fault (SF LOW)
#define STATUS_FB_OVERCURRENT (1 << 6)  // Courant FB > seuil
#define STATUS_HH12_AZ_OK   (1 << 7)   // Encodeur AZ OK
#define STATUS_HH12_EL_OK   (1 << 8)   // Encodeur EL OK

// ═══════ Holding Registers (FC 03/06, lecture/écriture) ═══════
// Le master (ESP32) écrit ces registres, le slave (UNO R4) les exécute
#define HREG_MOT_AZ_DUTY         0   // Duty AZ : -255 à +255 (int16_t signé)
#define HREG_MOT_EL_DUTY         1   // Duty EL : -255 à +255 (int16_t signé)
#define HREG_CMD                 2   // Commande : voir CMD_* ci-dessous
#define HREG_HALL_PRESET_AZ_HI   3   // Preset Hall AZ [31:16]
#define HREG_HALL_PRESET_AZ_LO   4   // Preset Hall AZ [15:0]
#define HREG_HALL_PRESET_EL_HI   5   // Preset Hall EL [31:16]
#define HREG_HALL_PRESET_EL_LO   6   // Preset Hall EL [15:0]

#define NUM_HOLDING_REGS         7

// ═══════ Commandes (Holding Register CMD) ═══════
#define CMD_NONE                 0x0000
#define CMD_RESET_HALL           0x0001  // Remettre compteurs Hall à zéro
#define CMD_CLEAR_FAULT          0x0002  // Effacer les flags de fault
#define CMD_APPLY_PRESET         0x0003  // Appliquer les presets Hall
#define CMD_EMERGENCY_STOP       0xFFFF  // Arrêt d'urgence immédiat

#endif
