/*
  Curtis1228MotorController.h — IMotorController implementation for Curtis 1228.

  The Curtis 1228 is simpler than the 1229 — likely fewer drive modes and
  less diagnostic capability. Will replace RoboteQ on R5/R8 receivers.

  Created by Eric Alsop, March 20, 2026.
  Copyright 2026 Best Tugs, LLC

  TODO: Determine communication protocol (CAN? Analog? Hybrid?)
  TODO: Define command set and any diagnostic capability
*/
#ifndef Curtis1228MotorController_H
#define Curtis1228MotorController_H

#include "IMotorController.h"

class Curtis1228MotorController : public IMotorController
{
public:
    Curtis1228MotorController();

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
    bool eStopActive;
    bool safetyStopActive;
    int16_t appliedThrottle;
    int16_t appliedSteering;

    // TODO: Add hardware-specific members (CAN handle, pin numbers, etc.)
};

#endif // Curtis1228MotorController_H
