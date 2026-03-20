/*
  RoboteQMotorController.cpp — IMotorController implementation for RoboteQ controllers.

  Logic preserved from the original MotorController.cpp — ramping, safety stop,
  quick-reversal detection, and e-stop behavior are unchanged.

  Original: Firmware/Libraries/MotorController/src/MotorController.cpp
  Created by Eric Alsop, Feb 9, 2024.
  Refactored March 20, 2026.
  Copyright 2024-2026 Best Tugs, LLC
*/
#include "RoboteQMotorController.h"

// Debugging Macro Enable Flags
#define SERIAL_DEBUG_LEVEL_1_ENABLED
// #define SERIAL_DEBUG_LEVEL_2_ENABLED
// #define SERIAL_DEBUG_LEVEL_3_ENABLED
#define SERIAL_DEBUG_LEVEL_ERROR_ENABLED
#include <DebugMacros.h>

RoboteQMotorController::RoboteQMotorController()
    : canOpen()
    , eStopActive(false)
    , safetyStopActive(false)
    , appliedThrottle(0)
    , appliedSteering(0)
    , lastThrottleUpdate(0)
    , lastSteeringUpdate(0)
{
}

void RoboteQMotorController::init()
{
    canOpen.init();
}

void RoboteQMotorController::update()
{
    canOpen.update();
}

void RoboteQMotorController::eStop()
{
    eStopActive = true;
    writeThrottle(NEUTRAL);
    writeSteering(STRAIGHT);
}

void RoboteQMotorController::safetyStop()
{
    safetyStopActive = true;
}

void RoboteQMotorController::releaseStop()
{
    eStopActive = false;
    safetyStopActive = false;
}

bool RoboteQMotorController::isEStopped() const
{
    return eStopActive;
}

bool RoboteQMotorController::isSafetyStopped() const
{
    return safetyStopActive;
}

void RoboteQMotorController::setThrottle(int16_t throttle)
{
    if (eStopActive)
    {
        D1PRINTLN("E-Stop Activated. Setting Throttle to NEUTRAL");
        writeThrottle(NEUTRAL);
        return;
    }

    if (safetyStopActive || throttleQuicklyReversed(throttle))
    {
        D1PRINTVAR(safetyStopActive);
        appliedThrottle = adjustToTarget(appliedThrottle, NEUTRAL, ROBOTEQ_SAFETY_STOP_DECELERATION, lastThrottleUpdate);
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
        appliedThrottle = adjustToTarget(appliedThrottle, throttle, ROBOTEQ_THROTTLE_ACCELERATION, lastThrottleUpdate);
    }
    D2PERIODICPRINTVAR(50, appliedThrottle);
    writeThrottle(appliedThrottle);
}

void RoboteQMotorController::setSteering(int16_t steering)
{
    if (eStopActive)
    {
        D1PERIODICPRINTLN(50, "E-Stop Activated. Setting Steering to STRAIGHT");
        writeSteering(STRAIGHT);
        return;
    }

    if (safetyStopActive)
    {
        D1PERIODICPRINTLN(50, "Safety Stop Activated. Setting Steering to STRAIGHT");
        appliedSteering = adjustToTarget(appliedSteering, NEUTRAL, ROBOTEQ_SAFETY_STOP_DECELERATION, lastSteeringUpdate);
    }
    else
    {
        appliedSteering = adjustToTarget(appliedSteering, steering, ROBOTEQ_STEERING_ACCELERATION, lastSteeringUpdate);
    }
    D2PERIODICPRINTVAR(50, appliedSteering);
    writeSteering(appliedSteering);
}

void RoboteQMotorController::writeSteering(int16_t steering)
{
    canOpen.writeSDOCommand(STEERING_COMMAND, steering);
}

void RoboteQMotorController::writeThrottle(int16_t throttle)
{
    canOpen.writeSDOCommand(THROTTLE_COMMAND, throttle);
}

bool RoboteQMotorController::isInDeadband(int16_t potValue, uint16_t threshold)
{
    return (-threshold < potValue && threshold > potValue);
}

bool RoboteQMotorController::throttleQuicklyReversed(int16_t currentThrottle)
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
        D2PRINTLN("Throttle is in the deadband \nLeaving throttleQuicklyReversed");
        prevThrottle2 = prevThrottle1;
        prevThrottle1 = currentThrottle;
        return false;
    }

    int16_t rollingAvg = (prevThrottle1 + prevThrottle2) / 2;
    if (directionHasReversed(rollingAvg, currentThrottle))
    {
        D2PRINTVAR(rollingAvg);
        D2PRINTVAR(currentThrottle);
        D2PRINTLN("****************************Direction changed!");
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
        D1PRINTLN("******************Safety Stop Triggered********************");
        safetyStopActive = true;
        return true;
    }

    prevThrottle2 = prevThrottle1;
    prevThrottle1 = currentThrottle;
    D2PRINTLN("Leaving throttleQuicklyReversed");
    return false;
}

bool RoboteQMotorController::directionHasReversed(int16_t a, int16_t b)
{
    return ((a < 0 && b > 0) || (a > 0 && b < 0));
}

int16_t RoboteQMotorController::adjustToTarget(int16_t value, int16_t targetValue, float rate, unsigned long &lastUpdate)
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

    float decelerationMultiplier = 1.0;
    if (directionHasReversed(value, targetValue))
    {
        float reversalIntensity = abs(targetValue) / 1000.0;
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
            ERRORPRINTLN("We tried to set a step size beyond the maxStepSize");
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
