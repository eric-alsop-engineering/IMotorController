/*
  Curtis1229MotorController.cpp — IMotorController implementation for Curtis 1229.

  Manages two Curtis 1229 controllers in differential drive configuration.
  Acceleration ramping ported from RoboteQMotorController.
  Steering curve and max output tuning ported from VG CurtisMotorController.

  Created by Eric Alsop, March 20, 2026.
  Copyright 2026 Best Tugs, LLC
*/
#include "Curtis1229MotorController.h"

// Debugging Macro Enable Flags
#define SERIAL_DEBUG_LEVEL_1_ENABLED
// #define SERIAL_DEBUG_LEVEL_2_ENABLED
// #define SERIAL_DEBUG_LEVEL_3_ENABLED
#define SERIAL_DEBUG_LEVEL_ERROR_ENABLED
#include <DebugMacros.h>

// ═══════════════════════════════════════════════════════════════════════════
// Constructor
// ═══════════════════════════════════════════════════════════════════════════

Curtis1229MotorController::Curtis1229MotorController(FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>& canBus)
    : leftWheel(canBus, CURTIS_LEFT_WHEEL_NODE_ID)
    , rightWheel(canBus, CURTIS_RIGHT_WHEEL_NODE_ID)
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
{
}

// ═══════════════════════════════════════════════════════════════════════════
// IMotorController interface
// ═══════════════════════════════════════════════════════════════════════════

void Curtis1229MotorController::init()
{
    D1PRINTLN("Curtis1229MotorController::init()");
    D1PRINT("  Left wheel  node ID: ");
    D1PRINT(leftWheel.getNodeId());
    D1PRINT(", COB-ID: 0x");
    D1PRINTLN(leftWheel.getCobId(), HEX);
    D1PRINT("  Right wheel node ID: ");
    D1PRINT(rightWheel.getNodeId());
    D1PRINT(", COB-ID: 0x");
    D1PRINTLN(rightWheel.getCobId(), HEX);

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

    // TODO: Read TPDO messages from Curtis controllers to populate diagnostics.
    //       This requires parsing incoming CAN frames with COB-IDs:
    //         Left  TPDO1: 0x180 + CURTIS_LEFT_WHEEL_NODE_ID  = 0x181
    //         Right TPDO1: 0x180 + CURTIS_RIGHT_WHEEL_NODE_ID = 0x182
    //       Parse User fields for actual velocity, error codes, status flags.
    //       Store results in lastErrorCode and lastStatusFlags.

    // TODO: Implement SDO error reads for more detailed fault information.
    //       Send SDO read requests to CURTIS_ERROR_CODE (0x603F00) and
    //       CURTIS_STATUS_WORD (0x604100) on each controller node.
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
// IDiagnosticSource interface (stubbed — TODO: implement TPDO/SDO reads)
// ═══════════════════════════════════════════════════════════════════════════

bool Curtis1229MotorController::hasError() const
{
    // TODO: Populate lastErrorCode from TPDO reads or SDO queries in update().
    //       For now, returns false until CAN receive parsing is implemented.
    return lastErrorCode != 0;
}

uint16_t Curtis1229MotorController::getErrorCode() const
{
    // TODO: Read CURTIS_ERROR_CODE (0x603F00) via SDO from both controllers.
    //       Return the most severe / first non-zero error code.
    return lastErrorCode;
}

uint16_t Curtis1229MotorController::getStatusFlags() const
{
    // TODO: Read CURTIS_STATUS_WORD (0x604100) via SDO from both controllers.
    //       Combine or return flags for the caller to interpret.
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
    int16_t leftOutput  = appliedThrottle + steeringOffset;
    int16_t rightOutput = appliedThrottle - steeringOffset;

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
