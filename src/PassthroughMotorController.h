/*
  PassthroughMotorController.h — IMotorController for direct-wired potentiometer systems.

  Used when the throttle potentiometer voltage is wired directly to the motor
  controller (e.g., wired tug). The receiver has no software control of
  throttle/steering — those methods are no-ops. E-stop is handled by cutting
  a relay or triggering a hardware e-stop line.

  Created by Eric Alsop, March 20, 2026.
  Copyright 2026 Best Tugs, LLC
*/
#ifndef PassthroughMotorController_H
#define PassthroughMotorController_H

#include "IMotorController.h"

class PassthroughMotorController : public IMotorController
{
public:
    // eStopPin: digital output pin that cuts power or triggers hardware e-stop
    PassthroughMotorController(uint8_t eStopPin);

    void init() override;
    void update() override;

    // No-ops: hardware handles throttle/steering directly
    void setThrottle(int16_t value) override { (void)value; }
    void setSteering(int16_t value) override { (void)value; }

    void eStop() override;
    void safetyStop() override;
    void releaseStop() override;
    bool isEStopped() const override;
    bool isSafetyStopped() const override;

private:
    uint8_t eStopPin;
    bool eStopActive;
    bool safetyStopActive;
};

#endif // PassthroughMotorController_H
