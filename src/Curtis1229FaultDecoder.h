/*
  Curtis1229FaultDecoder.h — one Curtis 1229 node's EMCY fault state.

  Captures a node's CANopen Emergency (EMCY) frames, decodes them into Best Tugs fault codes
  via the shared Curtis1229ErrorCodes tables, and latches the result until the node reports
  the fault cleared.

  This exists because the fault half used to be implemented twice: once inside
  Curtis1229MotorController (differential-drive wireless tugs, two nodes) and once in the wired
  receiver's CurtisFaultMonitor (single node). The drive halves genuinely differ — two-wheel
  firmware-ramped mixing versus single-node raw passthrough — but the fault half never did, and
  keeping two copies cost real time: the "clear keys on Error Register bit 0, not category
  0x0000" fix had to be made twice on the same day (2026-07-28), and the NMT-reset-clears-latched
  -faults fix landed eleven days apart in the two copies.

  Deliberately does NOT touch the CAN peripheral. Registering a receive callback is the caller's
  job, because the two callers need different routing: Curtis1229MotorController multiplexes
  TPDO1, EMCY and heartbeat for two nodes through one callback, while CurtisFaultMonitor
  registers its own callback for a single node. The caller checks matches() and hands the frame
  to captureFromISR().

  Created by Eric Alsop, August 18, 2026.
  Copyright 2026 Best Tugs, LLC
*/
#ifndef Curtis1229FaultDecoder_H
#define Curtis1229FaultDecoder_H

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "Curtis1229Dictionary.h"

class Curtis1229FaultDecoder
{
public:
    explicit Curtis1229FaultDecoder(uint8_t nodeId = CURTIS_DEFAULT_NODE_ID);

    void     setNodeId(uint8_t nodeId);
    uint8_t  getNodeId() const { return nodeId; }
    uint16_t getEMCYCobId() const { return emcyCobId; }

    // True when this CAN ID is our node's EMCY frame. Check before captureFromISR().
    bool matches(uint16_t canId) const { return canId == emcyCobId; }

    // ISR context: stash the frame. Does no decoding and takes no locks.
    void captureFromISR(const CAN_message_t& msg);

    // Main loop: decode any pending capture into the latched fault state. Call once per loop.
    void update();

    // Forget the latched fault and any pending capture. Call after commanding an NMT Reset
    // Node: a rebooted Curtis has no faults and never re-broadcasts a clear for faults from
    // its previous life, so the latched code would otherwise stick forever. A fault that
    // genuinely survives the reset re-latches from its next EMCY within moments.
    void clear();

    bool     hasError() const       { return 0 != errorCode; }
    uint16_t getErrorCode() const   { return errorCode; }
    uint16_t getStatusFlags() const { return statusFlags; }

    // Sticky: true once ANY EMCY has been captured from this node, and never cleared by
    // update() consuming the capture. The no-comms watchdog needs "have we ever heard this
    // node", which is a different question from "is a fault pending decode".
    bool hasEverReceived() const { return everReceived; }

private:
    uint8_t  nodeId;
    uint16_t emcyCobId; // 0x080 + nodeId

    // Written in the RX ISR, consumed in update().
    volatile uint16_t rxCategory;
    volatile uint8_t  rxRegister;
    volatile uint8_t  rxStatus[5];
    volatile bool     rxPending;
    volatile bool     everReceived;

    // Latched, main-loop side.
    uint16_t errorCode;
    uint16_t statusFlags;
};

#endif // Curtis1229FaultDecoder_H
