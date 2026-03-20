/*
  IMotorController.h — Abstract interface for motor controller drivers.

  All receiver-side motor controllers (RoboteQ, Curtis 1229/1228, PWM, passthrough)
  must implement this interface. The WirelessReceiver library calls only these
  methods, so the state machine and comms layer never know which motor controller
  hardware is behind them.

  For motor controllers that support additional capabilities (drive modes,
  diagnostics), see the optional IDriveModeController and IDiagnosticSource
  interfaces below. Implement them alongside IMotorController using multiple
  inheritance — the receiver will detect them at construction time.

  Created by Eric Alsop, March 20, 2026.
  Copyright 2026 Best Tugs, LLC
*/
#ifndef IMotorController_H
#define IMotorController_H

#include <Arduino.h>

#define NEUTRAL  0  // Throttle neutral position
#define STRAIGHT 0  // Steering straight position

// ─── Core Motor Controller Interface ──────────────────────────────────────────
// Every motor controller driver must implement this.

class IMotorController
{
public:
    virtual ~IMotorController() = default;

    // Called once during setup
    virtual void init() = 0;

    // Called every loop iteration (for CAN heartbeats, ramp calculations, etc.)
    virtual void update() = 0;

    // Set throttle value. Range: -1000 to +1000 (0 = neutral)
    // For passthrough/direct-pot hardware, this is a no-op.
    virtual void setThrottle(int16_t value) = 0;

    // Set steering value. Range: -1000 to +1000 (0 = straight)
    // For passthrough/direct-pot hardware, this is a no-op.
    virtual void setSteering(int16_t value) = 0;

    // Emergency stop — immediately cut motor output by whatever means necessary.
    // Hardware-specific: could be a CANopen command, cutting a relay, or a PWM kill.
    virtual void eStop() = 0;

    // Safety stop — controlled deceleration to zero (e.g., quick reversal detected)
    virtual void safetyStop() = 0;

    // Release a previous stop condition and allow normal operation
    virtual void releaseStop() = 0;

    // Query current stop state
    virtual bool isEStopped() const = 0;
    virtual bool isSafetyStopped() const = 0;
};

// ─── Optional: Drive Mode Switching ───────────────────────────────────────────
// For motor controllers that support multiple drive modes (e.g., Curtis 1229
// speed/torque/creep modes). Only implement if the hardware supports it.

class IDriveModeController
{
public:
    virtual ~IDriveModeController() = default;

    virtual void setDriveMode(uint8_t mode) = 0;
    virtual uint8_t getDriveMode() const = 0;
    virtual uint8_t getDriveModeCount() const = 0;
};

// ─── Optional: Diagnostics / Error Reporting ──────────────────────────────────
// For motor controllers that can report errors and status over CAN or similar.
// The receiver will read these and pass them up to the controller for display.

class IDiagnosticSource
{
public:
    virtual ~IDiagnosticSource() = default;

    // Returns true if the motor controller is currently reporting an error
    virtual bool hasError() const = 0;

    // Returns a manufacturer-specific error code (0 = no error)
    virtual uint16_t getErrorCode() const = 0;

    // Returns a manufacturer-specific status/warning bitfield
    virtual uint16_t getStatusFlags() const = 0;
};

#endif // IMotorController_H
