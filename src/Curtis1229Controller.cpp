/*
  Curtis1229Controller.cpp — Low-level CAN PDO wrapper for a single Curtis 1229 controller.

  Ported from Virgin Galactic BT project. Adapted from message-generation-only
  to actually sending via FlexCAN.

  Original: Libraries/Curtis1229Controller/src/Curtis1229Controller.cpp
  Created by Eric Alsop, July 14, 2025.
  Ported to Best Tugs by Eric Alsop, March 20, 2026.
  Copyright 2025-2026 Best Tugs, LLC
*/
#include "Curtis1229Controller.h"

// Debugging Macro Enable Flags
#define SERIAL_DEBUG_LEVEL_1_ENABLED
// #define SERIAL_DEBUG_LEVEL_2_ENABLED
// #define SERIAL_DEBUG_LEVEL_3_ENABLED
#define SERIAL_DEBUG_LEVEL_ERROR_ENABLED
#include <DebugMacros.h>

// ─── Constructor ────────────────────────────────────────────────────────────

Curtis1229Controller::Curtis1229Controller(FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>& canBus, uint8_t nodeId)
    : canBus(canBus)
{
    if (nodeId < 1 || nodeId > 127)
    {
        ERRORPRINTLN("Error: Invalid Node ID. Must be between 1 and 127.");
        this->myNodeId = CURTIS_DEFAULT_NODE_ID;
    }
    else
    {
        this->myNodeId = nodeId;
    }

    // For now we use a fixed user value; cobId only calculated once.
    // This can be expanded later to change based on input.
    this->user = CURTIS_DEFAULT_USER;
    D1PRINTVAR(this->user);
    uint16_t pdoBaseCobId = determinePdoBaseCobId(this->user);
    D1PRINTVAR(nodeId);
    this->cobId = calculateCobId(myNodeId, pdoBaseCobId);
    if (cobId == 0)
    {
        ERRORPRINTLN("Error - Failed to calculate cobId");
    }
    D1PRINTVAR(cobId);
    D1PRINTLN("Curtis 1229 Controller Setup Complete");
}

// ─── Send throttle command over CAN ─────────────────────────────────────────

void Curtis1229Controller::sendThrottleCommand(int16_t throttle, int16_t modeValue)
{
    CAN_message_t msg = generateThrottleMessage(throttle, modeValue);
    canBus.write(msg);
}

// ─── Send neutral immediately ───────────────────────────────────────────────

void Curtis1229Controller::sendNeutral(int16_t modeValue)
{
    sendThrottleCommand(CURTIS_NEUTRAL, modeValue);
}

// ─── Generate throttle message (returns CAN_message_t, doesn't send) ────────
// modeValue: 0 = Speed Mode 1, non-zero = Speed Mode 2 (sent via User 3, bytes 4-5)

CAN_message_t Curtis1229Controller::generateThrottleMessage(int16_t throttle, int16_t modeValue)
{
    CAN_message_t msg;

    msg.id = cobId;
    msg.len = 8;

    // Initialize data buffer to zero
    for (int i = 0; i < 8; i++)
    {
        msg.buf[i] = 0;
    }

    // Set speed mode in User 3 (bytes 4-5): 0 = Mode 1, non-zero = Mode 2
    setPdoMsgUserData(msg.buf, 3, modeValue);

    // Set throttle in the configured user slot (bytes 2-3 for User 2)
    setPdoMsgUserData(msg.buf, user, throttle);

    // Debug output (throttled to once every 100 ms)
    static unsigned long lastPrintTime = 0;
    unsigned long now = millis();
    if (now - lastPrintTime > 100)
    {
        D1PRINT("CAN Msg to Curtis (Node ");
        D1PRINT(myNodeId);
        D1PRINT("): ID=0x");
        D1PRINT(msg.id, HEX);
        D1PRINT(", Throttle=");
        D1PRINT(throttle);
        D1PRINT(", Data=[");
        for (int i = 0; i < msg.len; i++)
        {
            if (i > 0) D1PRINT(" ");
            D1PRINT("0x");
            if (msg.buf[i] < 0x10) D1PRINT("0");
            D1PRINT(msg.buf[i], HEX);
        }
        D1PRINTLN("]");
        lastPrintTime = now;
    }

    return msg;
}

// ─── COB-ID override ────────────────────────────────────────────────────────

void Curtis1229Controller::setCobId(uint16_t customCobId)
{
    this->cobId = customCobId;
    D1PRINT("Curtis Node ");
    D1PRINT(myNodeId);
    D1PRINT(" -> Custom COB-ID set to: 0x");
    D1PRINTLN(cobId, HEX);
}

// ─── Set Node ID and recalculate cobId ──────────────────────────────────────

void Curtis1229Controller::setNodeID(uint8_t nodeId)
{
    if (nodeId < 1 || nodeId > 127)
    {
        ERRORPRINTLN("Error: Invalid Node ID. Must be between 1 and 127.");
        return;
    }
    this->myNodeId = nodeId;
    uint16_t pdoBaseCobId = determinePdoBaseCobId(this->user);
    this->cobId = calculateCobId(myNodeId, pdoBaseCobId);
    D1PRINT("Node ID set to: ");
    D1PRINT(myNodeId);
    D1PRINT(", COB-ID: 0x");
    D1PRINTLN(cobId, HEX);
}

// ─── Private helpers ────────────────────────────────────────────────────────

uint16_t Curtis1229Controller::determinePdoBaseCobId(uint8_t userNum)
{
    uint16_t pdoBaseCobId = 0;
    if (userNum > 0 && userNum <= 4)
    {
        pdoBaseCobId = CURTIS_RPDO1_BASE_COBID;
    }
    else if (userNum > 4 && userNum <= 8)
    {
        pdoBaseCobId = CURTIS_RPDO2_BASE_COBID;
    }
    else
    {
        ERRORPRINTLN("Invalid User number");
        D1PRINTVAR(userNum);
    }
    return pdoBaseCobId;
}

void Curtis1229Controller::setPdoMsgUserData(uint8_t* dataBuffer, uint8_t userNum, int16_t data)
{
    switch (userNum)
    {
    case 1:
    case 5:
        dataBuffer[0] = data;
        dataBuffer[1] = data >> 8;
        break;
    case 2:
    case 6:
        dataBuffer[2] = data;
        dataBuffer[3] = data >> 8;
        break;
    case 3:
    case 7:
        dataBuffer[4] = data;
        dataBuffer[5] = data >> 8;
        break;
    case 4:
    case 8:
        dataBuffer[6] = data;
        dataBuffer[7] = data >> 8;
        break;
    default:
        ERRORPRINTLN("Error - User number must be 1 through 8");
        break;
    }
}

uint16_t Curtis1229Controller::calculateCobId(uint8_t nodeId, uint16_t pdoBaseCobId)
{
    if (nodeId < 1 || nodeId > 127)
    {
        ERRORPRINTLN("Error: Invalid Node ID. Must be between 1 and 127.");
        return 0;
    }

    if (pdoBaseCobId != CURTIS_TPDO1_BASE_COBID &&
        pdoBaseCobId != CURTIS_TPDO2_BASE_COBID &&
        pdoBaseCobId != CURTIS_RPDO1_BASE_COBID &&
        pdoBaseCobId != CURTIS_RPDO2_BASE_COBID)
    {
        ERRORPRINTLN("Error: Invalid PDO Base COB-ID.");
        return 0;
    }

    return pdoBaseCobId + nodeId;
}
