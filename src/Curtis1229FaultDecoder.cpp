/*
  Curtis1229FaultDecoder.cpp — see Curtis1229FaultDecoder.h.

  Created by Eric Alsop, August 18, 2026.
  Copyright 2026 Best Tugs, LLC
*/
#include "Curtis1229FaultDecoder.h"
#include "Curtis1229ErrorCodes.h"

// Debugging Macro Enable Flags
#define SERIAL_DEBUG_LEVEL_1_ENABLED
#define SERIAL_DEBUG_LEVEL_ERROR_ENABLED
#include <DebugMacros.h>

Curtis1229FaultDecoder::Curtis1229FaultDecoder(uint8_t nodeId)
    : nodeId(nodeId)
    , emcyCobId((uint16_t)(CURTIS_EMCY_COBID + nodeId))
    , rxCategory(0)
    , rxRegister(0)
    , rxStatus{}
    , rxPending(false)
    , everReceived(false)
    , errorCode(0)
    , statusFlags(0)
{
}

void Curtis1229FaultDecoder::setNodeId(uint8_t newNodeId)
{
    nodeId = newNodeId;
    emcyCobId = (uint16_t)(CURTIS_EMCY_COBID + newNodeId);
}

void Curtis1229FaultDecoder::captureFromISR(const CAN_message_t& msg)
{
    // CANopen EMCY payload: bytes 0-1 = error category (little-endian), byte 2 = error
    // register, bytes 3-7 = manufacturer-specific status-register bitmasks.
    rxCategory = (uint16_t)(msg.buf[0] | (msg.buf[1] << 8));
    rxRegister = msg.buf[2];
    for (int i = 0; i < 5; i++)
    {
        rxStatus[i] = msg.buf[3 + i];
    }
    everReceived = true;
    rxPending = true;
}

void Curtis1229FaultDecoder::update()
{
    if (!rxPending)
    {
        return; // nothing new; keep whatever is latched
    }

    // Snapshot the ISR-written fields with interrupts briefly disabled.
    uint16_t category;
    uint8_t  reg;
    uint8_t  status[5];
    noInterrupts();
    category = rxCategory;
    reg = rxRegister;
    for (int i = 0; i < 5; i++) status[i] = rxStatus[i];
    rxPending = false;
    interrupts();

    // Log every EMCY raw so a serial capture shows exactly what the Curtis sent, set AND clear.
    // The wired receiver has had this since July; the wireless one did not, which is part of
    // why it took so long to establish that the EMCY frames were arriving on Romeo all along.
    {
        char emcyMsg[80];
        snprintf(emcyMsg, sizeof(emcyMsg),
                 "Curtis node %u EMCY: cat=0x%04X reg=0x%02X status=%02X %02X %02X %02X %02X",
                 (unsigned)nodeId, category, reg,
                 status[0], status[1], status[2], status[3], status[4]);
        ERRORPRINTLN(emcyMsg);
    }

    if (reg & 0x01) // Error Register bit 0: set while ANY fault is active (1229 manual pg 120)
    {
        errorCode = curtis1229DecodeFaultFromEMCY(category, status);
        statusFlags = (uint16_t)(status[0] | ((uint16_t)status[1] << 8));
    }
    else
    {
        // Bit 0 clear = no faults remain active. The 1229 can report a clear using the fault's
        // own category (0x1000/0x6200) rather than 0x0000, so keying the clear on the register
        // — not on category == 0x0000 — is the reliable test. The old exact-match condition is
        // why boot transients like HPD Sequencing latched on screen forever.
        errorCode = 0;
        statusFlags = 0;
    }
}

void Curtis1229FaultDecoder::clear()
{
    noInterrupts();
    rxPending = false; // drop any EMCY captured before the reset but not yet decoded
    interrupts();
    errorCode = 0;
    statusFlags = 0;
}
