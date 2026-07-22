/*
  Curtis1229Controller.h — Low-level CAN PDO wrapper for a single Curtis 1229 controller.

  Generates and sends RPDO1 throttle messages to one Curtis 1229 motor controller
  node over FlexCAN. Each instance manages one wheel (one node ID).

  Ported from Virgin Galactic BT project. Adapted to own a FlexCAN reference
  and actually send messages (VG version only generated them).

  Original: Libraries/Curtis1229Controller/src/Curtis1229Controller.h
  Created by Eric Alsop, July 14, 2025.
  Ported to Best Tugs by Eric Alsop, March 20, 2026.
  Copyright 2025-2026 Best Tugs, LLC
*/
#ifndef CURTIS1229_CONTROLLER_H
#define CURTIS1229_CONTROLLER_H

#include <Arduino.h>
#include "Curtis1229Dictionary.h"
#include <FlexCAN_T4.h>

// RPDO1 User slot carrying the e-stop flag (User 1 is unused by throttle/mode, which live in
// User 2/3). Must match the slot the Curtis I/O map routes into 119-User Fault Estop.
#define CURTIS_ESTOP_USER_SLOT 1

// ─── TPDO data received from a single Curtis 1229 node ─────────────────────
struct CurtisTPDOData
{
    int16_t user1;       // Bytes 0-1 (little-endian)
    int16_t user2;       // Bytes 2-3
    int16_t user3;       // Bytes 4-5
    int16_t user4;       // Bytes 6-7
    unsigned long timestamp;  // millis() when last received
    bool valid;               // true after first successful parse
};

class Curtis1229Controller
{
public:
    // Constructor — takes the FlexCAN bus reference and a node ID
    Curtis1229Controller(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& canBus, uint8_t nodeId = CURTIS_DEFAULT_NODE_ID);

    // Send a throttle command to this controller's node
    // modeValue: 0 = Speed Mode 1, non-zero = Speed Mode 2 (sent via User 3, bytes 4-5)
    void sendThrottleCommand(int16_t throttle, int16_t modeValue = 0);

    // Generate throttle message without sending (for debugging/testing)
    CAN_message_t generateThrottleMessage(int16_t throttle, int16_t modeValue = 0);

    // Send neutral (zero throttle) immediately
    void sendNeutral(int16_t modeValue = 0);

    // CAN e-stop (see Firmware/Documentation/Curtis1229/Emergency_Stop_over_CAN.md).
    // RPDO1 User 1 is mapped on the Curtis (1313 programmer: 0x30D4 UserFaultEStopInput = 111)
    // into 119-User Fault Estop: non-zero performs a controlled powered stop at the E Stop Decel
    // rate and raises a User Fault Estop; returning to 0 releases it. While active, every RPDO1
    // frame this class builds carries a non-zero User 1. Safe to call every loop (change-detected).
    void setEStopActive(bool active);
    bool isEStopActive() const { return eStopActive; }

    // COB-ID override
    void setCobId(uint16_t customCobId);

    // Accessors
    uint8_t getNodeId() const { return myNodeId; }
    uint16_t getCobId() const { return cobId; }
    uint16_t getTPDO1CobId() const { return CURTIS_TPDO1_BASE_COBID + myNodeId; }
    uint16_t getEMCYCobId() const { return CURTIS_EMCY_COBID + myNodeId; }
    uint16_t getHeartbeatCobId() const { return CURTIS_HEARTBEAT_COBID + myNodeId; }

    // Set Node ID and update cobId
    void setNodeID(uint8_t nodeId);

    // Utility
    static uint16_t calculateCobId(uint8_t nodeId, uint16_t pdoBaseCobId);

    // ── TPDO receive ──
    // Called from CAN RX ISR — must be fast, no Serial, no blocking
    void processTPDO1(const uint8_t* data);
    const volatile CurtisTPDOData& getTPDOData() const { return tpdoData; }

private:
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& canBus;
    uint8_t myNodeId;
    uint16_t cobId;
    uint8_t user;
    bool eStopActive;
    volatile CurtisTPDOData tpdoData;

    void setPdoMsgUserData(uint8_t* dataBuffer, uint8_t userNum, int16_t data);
    uint16_t determinePdoBaseCobId(uint8_t userNum);
};

#endif // CURTIS1229_CONTROLLER_H
