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

class Curtis1229Controller
{
public:
    // Constructor — takes the FlexCAN bus reference and a node ID
    Curtis1229Controller(FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>& canBus, uint8_t nodeId = CURTIS_DEFAULT_NODE_ID);

    // Send a throttle command to this controller's node
    // modeValue: 0 = Speed Mode 1, non-zero = Speed Mode 2 (sent via User 3, bytes 4-5)
    void sendThrottleCommand(int16_t throttle, int16_t modeValue = 0);

    // Generate throttle message without sending (for debugging/testing)
    CAN_message_t generateThrottleMessage(int16_t throttle, int16_t modeValue = 0);

    // Send neutral (zero throttle) immediately
    void sendNeutral(int16_t modeValue = 0);

    // COB-ID override
    void setCobId(uint16_t customCobId);

    // Accessors
    uint8_t getNodeId() const { return myNodeId; }
    uint16_t getCobId() const { return cobId; }

    // Set Node ID and update cobId
    void setNodeID(uint8_t nodeId);

    // Utility
    static uint16_t calculateCobId(uint8_t nodeId, uint16_t pdoBaseCobId);

private:
    FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>& canBus;
    uint8_t myNodeId;
    uint16_t cobId;
    uint8_t user;

    void setPdoMsgUserData(uint8_t* dataBuffer, uint8_t userNum, int16_t data);
    uint16_t determinePdoBaseCobId(uint8_t userNum);
};

#endif // CURTIS1229_CONTROLLER_H
