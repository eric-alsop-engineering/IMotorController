/*
  Curtis1229MotorController.h — IMotorController implementation for Curtis 1229.

  Manages two Curtis 1229 motor controllers in a differential drive configuration
  (left wheel + right wheel). Implements IMotorController for the state machine,
  IDriveModeController for speed mode switching, and IDiagnosticSource for
  error/status reporting.

  Differential drive mixing:
    Left wheel  = throttle + steeringOffset
    Right wheel = throttle - steeringOffset

  Steering curve and max output tuning ported from VG CurtisMotorController.
  Acceleration ramping ported from RoboteQMotorController.

  Used by HD Romeo receivers.

  Created by Eric Alsop, March 20, 2026.
  Copyright 2026 Best Tugs, LLC
*/
#ifndef Curtis1229MotorController_H
#define Curtis1229MotorController_H

#include "IMotorController.h"
#include "Curtis1229Controller.h"
#include "Curtis1229Dictionary.h"
#include <FlexCAN_T4.h>

// ─── Acceleration/deceleration rates (units per millisecond) ────────────────
// Tuned for Curtis 1229 — slightly gentler than RoboteQ defaults
#define CURTIS1229_SAFETY_STOP_DECELERATION    1.5
#define CURTIS1229_EMERGENCY_STOP_DECELERATION 2.0
#define CURTIS1229_THROTTLE_ACCELERATION       0.5
#define CURTIS1229_STEERING_ACCELERATION       2.0

// ─── Steering tuning defaults ───────────────────────────────────────────────
#define CURTIS1229_DEFAULT_MAX_OUTPUT       1000    // Full range +-1000
#define CURTIS1229_DEFAULT_STEERING_CURVE   1.0f    // 1.0 = linear, >1.0 = more aggressive
#define CURTIS1229_DEFAULT_STEERING_SCALE   0.615f  // Steering input scaling (from VG tuning)

// ─── CAN send interval ─────────────────────────────────────────────────────
#define CURTIS1229_CAN_SEND_INTERVAL_MS     20  // Send PDO commands at 50 Hz

// ─── Heartbeat consumer timeout ────────────────────────────────────────────
// If no heartbeat received from a Curtis node within this period, flag it as lost.
// Curtis default heartbeat producer rate is 100ms (object 0x1017).
#define CURTIS_HEARTBEAT_TIMEOUT_MS  500

// ─── EMCY data structure ───────────────────────────────────────────────────
struct CurtisEMCYData
{
    uint16_t errorCategory;     // 0x0000=no fault, 0x1000/0x1001=generic, 0x6200=user
    uint8_t  errorRegister;     // Bit 0 = fault active
    uint8_t  statusBytes[5];    // Manufacturer-specific: status register bitmasks
    unsigned long timestamp;
    bool valid;
};

class Curtis1229MotorController : public IMotorController,
                                   public IDriveModeController,
                                   public IDiagnosticSource
{
public:
    // Constructor — takes a FlexCAN bus reference so the caller can pass their CAN instance
    Curtis1229MotorController(FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>& canBus);

    // ── IMotorController interface ──
    void init() override;
    void update() override;
    void setThrottle(int16_t value) override;
    void setSteering(int16_t value) override;
    void eStop() override;
    void safetyStop() override;
    void releaseStop() override;
    bool isEStopped() const override;
    bool isSafetyStopped() const override;

    // ── IDriveModeController interface ──
    void setDriveMode(uint8_t mode) override;
    uint8_t getDriveMode() const override;
    uint8_t getDriveModeCount() const override;

    // ── IDiagnosticSource interface ──
    bool hasError() const override;
    uint16_t getErrorCode() const override;
    uint16_t getStatusFlags() const override;

    // ── Tuning parameters (ported from VG CurtisMotorController) ──
    void setMaxOutput(int16_t maxOutput);
    void setSteeringCurve(float curve);
    void setSteeringScale(float scale);

    // ── Debug accessors ──
    int16_t getLeftWheelValue() const;
    int16_t getRightWheelValue() const;
    int16_t getAppliedThrottle() const;
    int16_t getAppliedSteering() const;

    // ── CAN receive (called from ISR — static because FlexCAN requires it) ──
    static void canRxCallback(const CAN_message_t& msg);

private:
    // Low-level Curtis controller instances (one per wheel)
    Curtis1229Controller leftWheel;
    Curtis1229Controller rightWheel;

    // Reference to the FlexCAN bus (needed for NMT commands)
    FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>& canBus;

    // Static instance pointer (FlexCAN ISR callback must be static)
    static Curtis1229MotorController* instance;

    // Stop states
    bool eStopActive;
    bool safetyStopActive;

    // Drive mode (speed mode 1 or 2)
    uint8_t currentDriveMode;

    // Diagnostic state (populated from TPDO/EMCY data in processReceivedCAN)
    uint16_t lastErrorCode;
    uint16_t lastStatusFlags;

    // Ramped values (what we actually send to the motors)
    int16_t appliedThrottle;
    int16_t appliedSteering;
    unsigned long lastThrottleUpdate;
    unsigned long lastSteeringUpdate;

    // Target values (what the caller requested)
    int16_t targetThrottle;
    int16_t targetSteering;

    // Last computed wheel outputs (for debug)
    int16_t lastLeftWheelValue;
    int16_t lastRightWheelValue;

    // Tuning parameters (from VG CurtisMotorController)
    int16_t maxOutput;
    float steeringCurve;
    float steeringScale;

    // CAN send timing
    unsigned long lastCanSendTime;

    // ── CAN receive state (volatile — written by ISR, read by main loop) ──
    volatile CurtisEMCYData emcyLeft;
    volatile CurtisEMCYData emcyRight;
    volatile unsigned long lastHeartbeatLeftTime;
    volatile unsigned long lastHeartbeatRightTime;
    volatile uint8_t curtisNMTStateLeft;
    volatile uint8_t curtisNMTStateRight;

    // Heartbeat-lost flags (set in main loop from volatile timestamps)
    bool heartbeatLostLeft;
    bool heartbeatLostRight;

    // NMT startup tracking
    bool nmtStartSent;

    // ── Internal methods ──

    // CAN receive setup and processing
    void setupCANReceive();
    void processReceivedCAN();

    // NMT commands
    void sendNMTCommand(uint8_t command, uint8_t nodeId);
    void startupNMT();

    // Send current applied values to both Curtis controllers via CAN
    void sendDriveCommands();

    // Steering curve and output constraining (from VG)
    int16_t applyCurve(int16_t value, float curve);
    int16_t constrainOutput(int16_t value);

    // Acceleration ramping (ported from RoboteQMotorController)
    int16_t adjustToTarget(int16_t value, int16_t targetValue, float rate, unsigned long& lastUpdate);
    bool isInDeadband(int16_t potValue, uint16_t threshold);
    bool throttleQuicklyReversed(int16_t currentThrottle);
    bool directionHasReversed(int16_t a, int16_t b);
};

#endif // Curtis1229MotorController_H
