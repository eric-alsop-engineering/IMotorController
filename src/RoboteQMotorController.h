/*
  RoboteQMotorController.h — IMotorController implementation for RoboteQ controllers.

  Wraps the existing MyCANOpen library to communicate with RoboteQ motor controllers
  via CANopen SDO commands. Used by Romeo R5/R8 receivers.

  This is a refactored version of the original MotorController class. The ramping,
  safety stop, quick-reversal detection, and e-stop logic are preserved exactly
  as they were — only the interface has changed to implement IMotorController.

  Original: Firmware/Libraries/MotorController/src/MotorController.h
  Created by Eric Alsop, Feb 9, 2024.
  Refactored March 20, 2026.
  Copyright 2024-2026 Best Tugs, LLC
*/
#ifndef RoboteQMotorController_H
#define RoboteQMotorController_H

#include "IMotorController.h"
#include <MyCANOpen.h>

// Acceleration/deceleration rates (units per millisecond)
#define ROBOTEQ_SAFETY_STOP_DECELERATION   1.5
#define ROBOTEQ_EMERGENCY_STOP_DECELERATION 2.0
#define ROBOTEQ_THROTTLE_ACCELERATION      0.5
#define ROBOTEQ_STEERING_ACCELERATION      2.0

class RoboteQMotorController : public IMotorController
{
public:
    RoboteQMotorController();

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

private:
    MyCANOpen canOpen;

    bool eStopActive;
    bool safetyStopActive;

    int16_t appliedThrottle;
    int16_t appliedSteering;
    unsigned long lastThrottleUpdate;
    unsigned long lastSteeringUpdate;

    void writeThrottle(int16_t throttle);
    void writeSteering(int16_t steering);
    int16_t adjustToTarget(int16_t value, int16_t targetValue, float rate, unsigned long &lastUpdate);
    bool isInDeadband(int16_t potValue, uint16_t threshold);
    bool throttleQuicklyReversed(int16_t currentThrottle);
    bool directionHasReversed(int16_t a, int16_t b);
};

#endif // RoboteQMotorController_H
