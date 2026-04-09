/*
  Curtis1229MotorController.cpp — IMotorController implementation for Curtis 1229.

  Manages two Curtis 1229 controllers in differential drive configuration.
  Acceleration ramping ported from RoboteQMotorController.
  Steering curve and max output tuning ported from VG CurtisMotorController.

  Created by Eric Alsop, March 20, 2026.
  Copyright 2026 Best Tugs, LLC
*/
#include "Curtis1229MotorController.h"
#include "Curtis1229ErrorCodes.h"

// Debugging Macro Enable Flags
#define SERIAL_DEBUG_LEVEL_1_ENABLED
// #define SERIAL_DEBUG_LEVEL_2_ENABLED
// #define SERIAL_DEBUG_LEVEL_3_ENABLED
#define SERIAL_DEBUG_LEVEL_ERROR_ENABLED
#include <DebugMacros.h>

// ═══════════════════════════════════════════════════════════════════════════
// Constructor
// ═══════════════════════════════════════════════════════════════════════════

// Static instance pointer for ISR callback routing
Curtis1229MotorController* Curtis1229MotorController::instance = nullptr;

Curtis1229MotorController::Curtis1229MotorController(FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>& canBus)
    : leftWheel(canBus, CURTIS_LEFT_WHEEL_NODE_ID)
    , rightWheel(canBus, CURTIS_RIGHT_WHEEL_NODE_ID)
    , canBus(canBus)
    , eStopActive(false)
    , safetyStopActive(false)
    , currentDriveMode(CURTIS_SPEED_MODE_1)
    , lastErrorCode(0)
    , lastStatusFlags(0)
    , appliedThrottle(0)
    , appliedSteering(0)
    , lastThrottleUpdate(0)
    , lastSteeringUpdate(0)
    , targetThrottle(0)
    , targetSteering(0)
    , lastLeftWheelValue(0)
    , lastRightWheelValue(0)
    , maxOutput(CURTIS1229_DEFAULT_MAX_OUTPUT)
    , steeringCurve(CURTIS1229_DEFAULT_STEERING_CURVE)
    , steeringScale(CURTIS1229_DEFAULT_STEERING_SCALE)
    , lastCanSendTime(0)
    , emcyLeft{}
    , emcyRight{}
    , lastHeartbeatLeftTime(0)
    , lastHeartbeatRightTime(0)
    , curtisNMTStateLeft(0)
    , curtisNMTStateRight(0)
    , heartbeatLostLeft(false)
    , heartbeatLostRight(false)
    , nmtStartSent(false)
{
    instance = this;
}

// ═══════════════════════════════════════════════════════════════════════════
// IMotorController interface
// ═══════════════════════════════════════════════════════════════════════════

void Curtis1229MotorController::init()
{
    D1PRINTLN("Curtis1229MotorController::init()");
    D1PRINT("  Left wheel  node ID: ");
    D1PRINT(leftWheel.getNodeId());
    D1PRINT(", RPDO1 COB-ID: 0x");
    D1PRINT(leftWheel.getCobId(), HEX);
    D1PRINT(", TPDO1 COB-ID: 0x");
    D1PRINTLN(leftWheel.getTPDO1CobId(), HEX);
    D1PRINT("  Right wheel node ID: ");
    D1PRINT(rightWheel.getNodeId());
    D1PRINT(", RPDO1 COB-ID: 0x");
    D1PRINT(rightWheel.getCobId(), HEX);
    D1PRINT(", TPDO1 COB-ID: 0x");
    D1PRINTLN(rightWheel.getTPDO1CobId(), HEX);

    // Configure CAN receive filters for TPDO, EMCY, and heartbeat messages
    setupCANReceive();

    // Send NMT Start to ensure Curtis controllers enter Operational state.
    // Curtis defaults to "Operational on KSI" = On, so this is a safety net
    // in case that setting was changed or a boot timing issue occurs.
    startupNMT();

    // Send neutral to both wheels on init
    leftWheel.sendNeutral();
    rightWheel.sendNeutral();
}

void Curtis1229MotorController::update()
{
    // Send CAN commands at a fixed interval
    unsigned long now = millis();
    if (now - lastCanSendTime >= CURTIS1229_CAN_SEND_INTERVAL_MS)
    {
        lastCanSendTime = now;
        sendDriveCommands();
    }

    // Process received CAN data (copies volatile ISR data into diagnostic fields)
    processReceivedCAN();
}

void Curtis1229MotorController::setThrottle(int16_t throttle)
{
    if (eStopActive)
    {
        D1PRINTLN("E-Stop Active. Throttle locked to NEUTRAL.");
        appliedThrottle = NEUTRAL;
        targetThrottle = NEUTRAL;
        return;
    }

    targetThrottle = throttle;

    if (safetyStopActive || throttleQuicklyReversed(throttle))
    {
        D1PRINTVAR(safetyStopActive);
        appliedThrottle = adjustToTarget(appliedThrottle, NEUTRAL,
                                          CURTIS1229_SAFETY_STOP_DECELERATION,
                                          lastThrottleUpdate);
        uint16_t threshold = 300;
        if (safetyStopActive && isInDeadband(throttle, threshold))
        {
            D1PRINTLN("Releasing Safety Stop");
            D2PRINTVAR(threshold);
            D2PRINTVAR(throttle);
            safetyStopActive = false;
        }
    }
    else
    {
        appliedThrottle = adjustToTarget(appliedThrottle, throttle,
                                          CURTIS1229_THROTTLE_ACCELERATION,
                                          lastThrottleUpdate);
    }
    D2PERIODICPRINTVAR(50, appliedThrottle);
}

void Curtis1229MotorController::setSteering(int16_t steering)
{
    if (eStopActive)
    {
        D1PERIODICPRINTLN(50, "E-Stop Active. Steering locked to STRAIGHT.");
        appliedSteering = STRAIGHT;
        targetSteering = STRAIGHT;
        return;
    }

    targetSteering = steering;

    if (safetyStopActive)
    {
        D1PERIODICPRINTLN(50, "Safety Stop Active. Steering ramping to STRAIGHT.");
        appliedSteering = adjustToTarget(appliedSteering, STRAIGHT,
                                          CURTIS1229_SAFETY_STOP_DECELERATION,
                                          lastSteeringUpdate);
    }
    else
    {
        appliedSteering = adjustToTarget(appliedSteering, steering,
                                          CURTIS1229_STEERING_ACCELERATION,
                                          lastSteeringUpdate);
    }
    D2PERIODICPRINTVAR(50, appliedSteering);
}

void Curtis1229MotorController::eStop()
{
    eStopActive = true;
    appliedThrottle = NEUTRAL;
    appliedSteering = STRAIGHT;
    targetThrottle = NEUTRAL;
    targetSteering = STRAIGHT;

    // Immediately send neutral to both wheels
    int16_t modeValue = (currentDriveMode == CURTIS_SPEED_MODE_2) ? 1 : 0;
    leftWheel.sendThrottleCommand(CURTIS_NEUTRAL, modeValue);
    rightWheel.sendThrottleCommand(CURTIS_NEUTRAL, modeValue);

    D1PRINTLN("Curtis1229MotorController: E-STOP ACTIVATED");
}

void Curtis1229MotorController::safetyStop()
{
    safetyStopActive = true;
    D1PRINTLN("Curtis1229MotorController: Safety Stop Activated");
}

void Curtis1229MotorController::releaseStop()
{
    eStopActive = false;
    safetyStopActive = false;
    D1PRINTLN("Curtis1229MotorController: Stop Released");
}

bool Curtis1229MotorController::isEStopped() const
{
    return eStopActive;
}

bool Curtis1229MotorController::isSafetyStopped() const
{
    return safetyStopActive;
}

// ═══════════════════════════════════════════════════════════════════════════
// IDriveModeController interface
// ═══════════════════════════════════════════════════════════════════════════

void Curtis1229MotorController::setDriveMode(uint8_t mode)
{
    if (mode == CURTIS_SPEED_MODE_1 || mode == CURTIS_SPEED_MODE_2)
    {
        currentDriveMode = mode;
        D1PRINT("Curtis drive mode set to: ");
        D1PRINTLN(currentDriveMode);
    }
    else
    {
        ERRORPRINTLN("Invalid Curtis drive mode");
        D1PRINTVAR(mode);
    }
}

uint8_t Curtis1229MotorController::getDriveMode() const
{
    return currentDriveMode;
}

uint8_t Curtis1229MotorController::getDriveModeCount() const
{
    return 2; // Speed Mode 1 and Speed Mode 2
}

// ═══════════════════════════════════════════════════════════════════════════
// IDiagnosticSource interface
// ═══════════════════════════════════════════════════════════════════════════

bool Curtis1229MotorController::hasError() const
{
    return lastErrorCode != 0;
}

uint16_t Curtis1229MotorController::getErrorCode() const
{
    return lastErrorCode;
}

uint16_t Curtis1229MotorController::getStatusFlags() const
{
    return lastStatusFlags;
}

// ═══════════════════════════════════════════════════════════════════════════
// Tuning parameters
// ═══════════════════════════════════════════════════════════════════════════

void Curtis1229MotorController::setMaxOutput(int16_t maxOut)
{
    this->maxOutput = constrain(maxOut, 1, 1000);
}

void Curtis1229MotorController::setSteeringCurve(float curve)
{
    this->steeringCurve = constrain(curve, 0.1f, 5.0f);
}

void Curtis1229MotorController::setSteeringScale(float scale)
{
    this->steeringScale = constrain(scale, 0.0f, 1.0f);
}

// ═══════════════════════════════════════════════════════════════════════════
// Debug accessors
// ═══════════════════════════════════════════════════════════════════════════

int16_t Curtis1229MotorController::getLeftWheelValue() const
{
    return lastLeftWheelValue;
}

int16_t Curtis1229MotorController::getRightWheelValue() const
{
    return lastRightWheelValue;
}

int16_t Curtis1229MotorController::getAppliedThrottle() const
{
    return appliedThrottle;
}

int16_t Curtis1229MotorController::getAppliedSteering() const
{
    return appliedSteering;
}

// ═══════════════════════════════════════════════════════════════════════════
// Internal: Send drive commands to both wheels via CAN
// ═══════════════════════════════════════════════════════════════════════════

void Curtis1229MotorController::sendDriveCommands()
{
    // Scale steering input for more manageable curve application (from VG tuning)
    int16_t scaledSteering = (int16_t)(appliedSteering * steeringScale);

    // Apply steering curve for more natural feel
    int16_t steeringOffset = applyCurve(scaledSteering, steeringCurve);

    // Differential drive mixing:
    //   Left wheel  = throttle + steeringOffset
    //   Right wheel = throttle - steeringOffset
    // Use int32_t intermediates to avoid signed int16_t overflow (UB)
    int32_t leftMix  = (int32_t)appliedThrottle + steeringOffset;
    int32_t rightMix = (int32_t)appliedThrottle - steeringOffset;
    int16_t leftOutput  = (int16_t)constrain(leftMix, -32767, 32767);
    int16_t rightOutput = (int16_t)constrain(rightMix, -32767, 32767);

    // Constrain to valid motor range
    leftOutput  = constrainOutput(leftOutput);
    rightOutput = constrainOutput(rightOutput);

    // Store for debug
    lastLeftWheelValue  = leftOutput;
    lastRightWheelValue = rightOutput;

    // Mode value: 0 = Speed Mode 1, non-zero = Speed Mode 2
    int16_t modeValue = (currentDriveMode == CURTIS_SPEED_MODE_2) ? 1 : 0;

    // Send to both wheels
    leftWheel.sendThrottleCommand(leftOutput, modeValue);
    rightWheel.sendThrottleCommand(rightOutput, modeValue);
}

// ═══════════════════════════════════════════════════════════════════════════
// Internal: Steering curve (ported from VG CurtisMotorController)
// ═══════════════════════════════════════════════════════════════════════════

int16_t Curtis1229MotorController::applyCurve(int16_t value, float curve)
{
    if (value == 0) return 0;

    float normalized = (float)value / 1000.0f;  // Normalize to -1.0 to +1.0
    float curved = pow(abs(normalized), curve) * (normalized > 0 ? 1.0f : -1.0f);
    return (int16_t)(curved * 1000.0f);
}

int16_t Curtis1229MotorController::constrainOutput(int16_t value)
{
    return constrain(value, -maxOutput, maxOutput);
}

// ═══════════════════════════════════════════════════════════════════════════
// Internal: Acceleration ramping (ported from RoboteQMotorController)
// ═══════════════════════════════════════════════════════════════════════════

int16_t Curtis1229MotorController::adjustToTarget(int16_t value, int16_t targetValue,
                                                    float rate, unsigned long& lastUpdate)
{
    if (rate <= 0)
    {
        ERRORPRINTLN("rate is not greater than zero");
        return value;
    }

    const uint16_t maxStepSize = 125;
    unsigned long currentTime = millis();
    unsigned long timeSinceLastUpdate = currentTime - lastUpdate;

    lastUpdate = currentTime;

    if (value == targetValue)
    {
        return value;
    }

    // Decelerate faster when reversing direction
    float decelerationMultiplier = 1.0;
    if (directionHasReversed(value, targetValue))
    {
        float reversalIntensity = abs(targetValue) / 1000.0f;
        decelerationMultiplier += reversalIntensity;
        D1PRINT("Decelerating faster by ");
        D1PRINTVAR(decelerationMultiplier);
    }

    if (timeSinceLastUpdate > 250)
    {
        ERRORPRINTLN("It's been more than 250 ms since we updated steering/throttle");
        D1PRINTVAR(timeSinceLastUpdate);
    }

    float stepSize = timeSinceLastUpdate * rate * decelerationMultiplier;

    if (!safetyStopActive && !eStopActive)
    {
        if (stepSize > maxStepSize)
        {
            ERRORPRINTLN("Step size exceeds maxStepSize");
            D1PRINTVAR(stepSize);
            D1PRINTVAR(maxStepSize);
            stepSize = maxStepSize;
        }
    }

    D1PRINT("Accelerating by ");
    D1PRINTVAR(stepSize);
    if (value > targetValue)
    {
        value -= stepSize;
        if (value < targetValue)
        {
            value = targetValue;
        }
    }
    else if (value < targetValue)
    {
        value += stepSize;
        if (value > targetValue)
        {
            value = targetValue;
        }
    }
    return value;
}

bool Curtis1229MotorController::isInDeadband(int16_t potValue, uint16_t threshold)
{
    return (-threshold < potValue && threshold > potValue);
}

bool Curtis1229MotorController::throttleQuicklyReversed(int16_t currentThrottle)
{
    D2PRINTLN("Entering throttleQuicklyReversed");

    static int16_t prevThrottle1 = 0;
    static int16_t prevThrottle2 = 0;
    static unsigned long directionChangeTime = 0;
    unsigned long currentTime = millis();
    const int16_t threshold = 800;
    const unsigned long reversalTimeLimit = 150;

    if (isInDeadband(currentThrottle, 250))
    {
        D2PRINTLN("Throttle is in the deadband\nLeaving throttleQuicklyReversed");
        prevThrottle2 = prevThrottle1;
        prevThrottle1 = currentThrottle;
        return false;
    }

    int16_t rollingAvg = (prevThrottle1 + prevThrottle2) / 2;
    if (directionHasReversed(rollingAvg, currentThrottle))
    {
        D2PRINTVAR(rollingAvg);
        D2PRINTVAR(currentThrottle);
        D2PRINTLN("Direction changed!");
        directionChangeTime = currentTime;
    }
    unsigned long timeSinceDirectionChange = currentTime - directionChangeTime;
    if ((currentThrottle >= threshold || currentThrottle <= -threshold) &&
        (timeSinceDirectionChange <= reversalTimeLimit))
    {
        D2PRINTVAR(currentThrottle);
        D2PRINTVAR(threshold);
        D2PRINTVAR(timeSinceDirectionChange);
        D2PRINTVAR(reversalTimeLimit);
        D1PRINTLN("Safety Stop Triggered (quick reversal detected)");
        safetyStopActive = true;
        return true;
    }

    prevThrottle2 = prevThrottle1;
    prevThrottle1 = currentThrottle;
    D2PRINTLN("Leaving throttleQuicklyReversed");
    return false;
}

bool Curtis1229MotorController::directionHasReversed(int16_t a, int16_t b)
{
    return ((a < 0 && b > 0) || (a > 0 && b < 0));
}

// ═══════════════════════════════════════════════════════════════════════════
// CAN Receive Infrastructure
// ═══════════════════════════════════════════════════════════════════════════

void Curtis1229MotorController::setupCANReceive()
{
    D1PRINTLN("Setting up CAN receive filters...");

    // Enable mailbox-based reception with interrupts.
    // FlexCAN_T4 on Teensy 4.1 CAN3 supports up to 64 mailboxes.
    // The first few are typically used for TX; we configure higher ones for RX.
    canBus.setMaxMB(16);
    canBus.enableFIFO();
    canBus.enableFIFOInterrupt();

    // Set up a global callback — we route by COB-ID inside canRxCallback.
    canBus.onReceive(canRxCallback);

    D1PRINT("  TPDO1 left  filter: 0x");
    D1PRINTLN(leftWheel.getTPDO1CobId(), HEX);
    D1PRINT("  TPDO1 right filter: 0x");
    D1PRINTLN(rightWheel.getTPDO1CobId(), HEX);
    D1PRINT("  EMCY left   filter: 0x");
    D1PRINTLN(leftWheel.getEMCYCobId(), HEX);
    D1PRINT("  EMCY right  filter: 0x");
    D1PRINTLN(rightWheel.getEMCYCobId(), HEX);
    D1PRINT("  HB left     filter: 0x");
    D1PRINTLN(leftWheel.getHeartbeatCobId(), HEX);
    D1PRINT("  HB right    filter: 0x");
    D1PRINTLN(rightWheel.getHeartbeatCobId(), HEX);

    D1PRINTLN("CAN receive setup complete.");
}

void Curtis1229MotorController::canRxCallback(const CAN_message_t& msg)
{
    if (!instance) return;

    uint16_t id = msg.id;
    Curtis1229MotorController* self = instance;

    // ── TPDO1 (Curtis → Teensy) ──
    if (id == self->leftWheel.getTPDO1CobId())
    {
        self->leftWheel.processTPDO1(msg.buf);
        return;
    }
    if (id == self->rightWheel.getTPDO1CobId())
    {
        self->rightWheel.processTPDO1(msg.buf);
        return;
    }

    // ── Emergency messages ──
    if (id == self->leftWheel.getEMCYCobId())
    {
        self->emcyLeft.errorCategory = (uint16_t)(msg.buf[0] | (msg.buf[1] << 8));
        self->emcyLeft.errorRegister = msg.buf[2];
        for (int i = 0; i < 5; i++) self->emcyLeft.statusBytes[i] = msg.buf[3 + i];
        self->emcyLeft.timestamp = millis();
        self->emcyLeft.valid = true;
        return;
    }
    if (id == self->rightWheel.getEMCYCobId())
    {
        self->emcyRight.errorCategory = (uint16_t)(msg.buf[0] | (msg.buf[1] << 8));
        self->emcyRight.errorRegister = msg.buf[2];
        for (int i = 0; i < 5; i++) self->emcyRight.statusBytes[i] = msg.buf[3 + i];
        self->emcyRight.timestamp = millis();
        self->emcyRight.valid = true;
        return;
    }

    // ── Heartbeat messages ──
    if (id == self->leftWheel.getHeartbeatCobId())
    {
        self->curtisNMTStateLeft = msg.buf[0] & 0x7F;
        self->lastHeartbeatLeftTime = millis();
        return;
    }
    if (id == self->rightWheel.getHeartbeatCobId())
    {
        self->curtisNMTStateRight = msg.buf[0] & 0x7F;
        self->lastHeartbeatRightTime = millis();
        return;
    }
}

void Curtis1229MotorController::processReceivedCAN()
{
    unsigned long now = millis();

    // ── Check heartbeat status ──
    if (lastHeartbeatLeftTime > 0 && (now - lastHeartbeatLeftTime > CURTIS_HEARTBEAT_TIMEOUT_MS))
    {
        if (!heartbeatLostLeft)
        {
            heartbeatLostLeft = true;
            D1PRINTLN("WARNING: Left Curtis heartbeat lost!");
        }
    }
    else
    {
        heartbeatLostLeft = false;
    }

    if (lastHeartbeatRightTime > 0 && (now - lastHeartbeatRightTime > CURTIS_HEARTBEAT_TIMEOUT_MS))
    {
        if (!heartbeatLostRight)
        {
            heartbeatLostRight = true;
            D1PRINTLN("WARNING: Right Curtis heartbeat lost!");
        }
    }
    else
    {
        heartbeatLostRight = false;
    }

    // ── Detect NMT state changes (Curtis dropped out of Operational) ──
    if (lastHeartbeatLeftTime > 0 && curtisNMTStateLeft != HB_STATE_OPERATIONAL && curtisNMTStateLeft != HB_STATE_BOOTUP)
    {
        D1PRINT("WARNING: Left Curtis NMT state = 0x");
        D1PRINTLN(curtisNMTStateLeft, HEX);
    }
    if (lastHeartbeatRightTime > 0 && curtisNMTStateRight != HB_STATE_OPERATIONAL && curtisNMTStateRight != HB_STATE_BOOTUP)
    {
        D1PRINT("WARNING: Right Curtis NMT state = 0x");
        D1PRINTLN(curtisNMTStateRight, HEX);
    }

    // ── Process EMCY data into diagnostic fields ──
    // Decode individual Curtis fault codes from EMCY status bitmask bytes.
    // Left controller has priority; fall through to right if left has no fault.
    uint16_t errorCode = 0;
    uint16_t statusFlags = 0;

    if (emcyLeft.valid && (emcyLeft.errorRegister & 0x01))
    {
        uint8_t statusCopy[5];
        noInterrupts();
        for (int i = 0; i < 5; i++) statusCopy[i] = emcyLeft.statusBytes[i];
        uint16_t cat = emcyLeft.errorCategory;
        interrupts();

        errorCode = curtis1229DecodeFaultFromEMCY(cat, statusCopy);
        statusFlags = (uint16_t)(statusCopy[0] | ((uint16_t)statusCopy[1] << 8));
    }
    if (errorCode == 0 && emcyRight.valid && (emcyRight.errorRegister & 0x01))
    {
        uint8_t statusCopy[5];
        noInterrupts();
        for (int i = 0; i < 5; i++) statusCopy[i] = emcyRight.statusBytes[i];
        uint16_t cat = emcyRight.errorCategory;
        interrupts();

        errorCode = curtis1229DecodeFaultFromEMCY(cat, statusCopy);
        statusFlags = (uint16_t)(statusCopy[0] | ((uint16_t)statusCopy[1] << 8));
    }

    // Check for EMCY "all clear" (category 0x0000, error register 0x00)
    if (emcyLeft.valid && emcyLeft.errorCategory == EMCY_CATEGORY_NO_FAULT && emcyLeft.errorRegister == 0)
    {
        emcyLeft.valid = false;
    }
    if (emcyRight.valid && emcyRight.errorCategory == EMCY_CATEGORY_NO_FAULT && emcyRight.errorRegister == 0)
    {
        emcyRight.valid = false;
    }

    // Add heartbeat-lost as a status flag
    if (heartbeatLostLeft || heartbeatLostRight)
    {
        statusFlags |= 0x8000; // Bit 15 = heartbeat lost
    }

    lastErrorCode = errorCode;
    lastStatusFlags = statusFlags;

    // Debug print diagnostics periodically
    static unsigned long lastDiagPrint = 0;
    if (now - lastDiagPrint > 2000)
    {
        lastDiagPrint = now;
        if (lastErrorCode != 0)
        {
            D1PRINT("Curtis FAULT: code ");
            D1PRINT(lastErrorCode);
            D1PRINT(" (");
            D1PRINT(curtis1229FaultString(lastErrorCode));
            D1PRINTLN(")");
        }
        if (leftWheel.getTPDOData().valid)
        {
            D1PRINT("Curtis LEFT TPDO1: U1=");
            D1PRINT(leftWheel.getTPDOData().user1);
            D1PRINT(" U2=");
            D1PRINT(leftWheel.getTPDOData().user2);
            D1PRINT(" U3=");
            D1PRINT(leftWheel.getTPDOData().user3);
            D1PRINT(" U4=");
            D1PRINTLN(leftWheel.getTPDOData().user4);
        }
        if (rightWheel.getTPDOData().valid)
        {
            D1PRINT("Curtis RIGHT TPDO1: U1=");
            D1PRINT(rightWheel.getTPDOData().user1);
            D1PRINT(" U2=");
            D1PRINT(rightWheel.getTPDOData().user2);
            D1PRINT(" U3=");
            D1PRINT(rightWheel.getTPDOData().user3);
            D1PRINT(" U4=");
            D1PRINTLN(rightWheel.getTPDOData().user4);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// NMT Commands
// ═══════════════════════════════════════════════════════════════════════════

void Curtis1229MotorController::sendNMTCommand(uint8_t command, uint8_t nodeId)
{
    CAN_message_t msg;
    msg.id = CURTIS_NMT_COBID;  // 0x000
    msg.len = 2;
    msg.buf[0] = command;
    msg.buf[1] = nodeId;  // 0 = broadcast to all nodes
    canBus.write(msg);

    D1PRINT("NMT command 0x");
    D1PRINT(command, HEX);
    D1PRINT(" sent to node ");
    D1PRINTLN(nodeId);
}

void Curtis1229MotorController::startupNMT()
{
    // Send NMT Start Remote Node to both controllers (broadcast).
    // Curtis defaults to "Operational on KSI" = On, so this is a safety net.
    // Using broadcast (nodeId=0) covers both wheels in one message.
    sendNMTCommand(NMT_CMD_START_REMOTE_NODE, 0);
    nmtStartSent = true;
    D1PRINTLN("NMT Start Remote Node sent (broadcast).");
}
