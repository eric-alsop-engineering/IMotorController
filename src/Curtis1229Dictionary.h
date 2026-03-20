/*
  Curtis1229Dictionary.h — CANopen object dictionary definitions for Curtis 1229.

  Ported from Virgin Galactic BT project, adapted for Best Tugs differential
  drive (2 wheels: left and right). VG-specific 4-wheel/6-axis node definitions
  have been removed.

  Created by Eric Alsop, July 14, 2025.
  Ported to Best Tugs by Eric Alsop, March 20, 2026.
  Copyright 2025-2026 Best Tugs, LLC
*/
#ifndef CURTIS1229_DICTIONARY_H
#define CURTIS1229_DICTIONARY_H

// ─── Best Tugs Node IDs (differential drive, 2 wheels) ─────────────────────
// Keep node IDs under 16 to avoid platform addressing conflicts
#define CURTIS_DEFAULT_NODE_ID       1
#define CURTIS_DEFAULT_USER          2

#define CURTIS_LEFT_WHEEL_NODE_ID    1   // COB-ID = 0x200 + 1 = 0x201
#define CURTIS_RIGHT_WHEEL_NODE_ID   2   // COB-ID = 0x200 + 2 = 0x202

#define CURTIS_NEUTRAL               0

// ─── Speed Modes ────────────────────────────────────────────────────────────
#define CURTIS_SPEED_MODE_1          1
#define CURTIS_SPEED_MODE_2          2

// ─── Base COB-ID Values for CANopen Communication ───────────────────────────
#define CURTIS_NMT_COBID             0x000  // Network Management (NMT)
#define CURTIS_SYNC_COBID            0x080  // Sync Message
#define CURTIS_EMCY_COBID            0x080  // Emergency Message (EMCY)
#define CURTIS_TIME_COBID            0x100  // Time Stamp Message
#define CURTIS_HEARTBEAT_COBID       0x700  // Heartbeat Message

// ─── Base COB-ID Values for PDO Communication ───────────────────────────────
/*
  Transmit/Receive is from the perspective of the Curtis controller.
  When sending a PDO message TO the controller, use RPDO base COB-IDs.
*/
#define CURTIS_TPDO1_BASE_COBID      0x180  // Transmit PDO1 (TPDO1)
#define CURTIS_RPDO1_BASE_COBID      0x200  // Receive PDO1 (RPDO1)

#define CURTIS_TPDO2_BASE_COBID      0x280  // Transmit PDO2 (TPDO2)
#define CURTIS_RPDO2_BASE_COBID      0x300  // Receive PDO2 (RPDO2)

/*
  CAN message data byte mapping (8 bytes per message, 16 bits per user):

  PDO1 TX and RX:
    111-User1: Data bytes 0 & 1, Bits 15-0
    112-User2: Data bytes 2 & 3, Bits 15-0
    113-User3: Data bytes 4 & 5, Bits 15-0
    114-User4: Data bytes 6 & 7, Bits 15-0

  PDO2 TX and RX:
    115-User5: Data bytes 0 & 1, Bits 15-0
    116-User6: Data bytes 2 & 3, Bits 15-0
    117-User7: Data bytes 4 & 5, Bits 15-0
    118-User8: Data bytes 6 & 7, Bits 15-0
*/

// ─── Standard Object Dictionary Entries for Curtis 1229 Controller ──────────
#define CURTIS_TARGET_VELOCITY       0x604000
#define CURTIS_ACTUAL_VELOCITY       0x606C00
#define CURTIS_CONTROL_WORD          0x604000
#define CURTIS_STATUS_WORD           0x604100
#define CURTIS_MODE_OF_OPERATION     0x606000
#define CURTIS_TORQUE_SETPOINT       0x607100
#define CURTIS_ERROR_CODE            0x603F00
#define CURTIS_MOTOR_TEMPERATURE     0x22A000
#define CURTIS_VOLTAGE_SUPPLY        0x220000

// ─── User 1-8 Function Mappings ─────────────────────────────────────────────
#define CURTIS_USER_1                0x8003330  // User 1
#define CURTIS_USER_2                0x8003331  // User 2
#define CURTIS_USER_3                0x8003332  // User 3
#define CURTIS_USER_4                0x8003333  // User 4
#define CURTIS_USER_5                0x8003334  // User 5
#define CURTIS_USER_6                0x8003335  // User 6
#define CURTIS_USER_7                0x8003336  // User 7
#define CURTIS_USER_8                0x8003337  // User 8

// ─── PDO1 TX (TPDO1) Mapping ────────────────────────────────────────────────
#define CURTIS_TPDO1_USER_1_LSB      0x8003330
#define CURTIS_TPDO1_USER_1_MSB      0x8013330
#define CURTIS_TPDO1_USER_2_LSB      0x8003331
#define CURTIS_TPDO1_USER_2_MSB      0x8013331
#define CURTIS_TPDO1_USER_3_LSB      0x8003332
#define CURTIS_TPDO1_USER_3_MSB      0x8013332
#define CURTIS_TPDO1_USER_4_LSB      0x8003333
#define CURTIS_TPDO1_USER_4_MSB      0x8013333

// ─── PDO1 RX (RPDO1) Mapping ────────────────────────────────────────────────
#define CURTIS_RPDO1_USER_1_LSB      0x8003330
#define CURTIS_RPDO1_USER_1_MSB      0x8013330
#define CURTIS_RPDO1_USER_2_LSB      0x8003331
#define CURTIS_RPDO1_USER_2_MSB      0x8013331
#define CURTIS_RPDO1_USER_3_LSB      0x8003332
#define CURTIS_RPDO1_USER_3_MSB      0x8013332
#define CURTIS_RPDO1_USER_4_LSB      0x8003333
#define CURTIS_RPDO1_USER_4_MSB      0x8013333

#endif // CURTIS1229_DICTIONARY_H
