#pragma once

#include "AccelStepperShift.h"
#include "begin_run.h"

class AccelShiftBeastie : public BeginRun {
    // show memory and time used by all AccelStepper

  public:
    AccelStepperShift *accel_stepper_shift;
    const int motor_ct;
    const int latch_pin;

    AccelShiftBeastie(int motor_ct, int latch_pin) : motor_ct(motor_ct), latch_pin(latch_pin) {}

    void begin() {
      accel_stepper_shift = new AccelStepperShift(MOTOR_CT, LATCH_PIN);

      Serial << F("SETUP AccelStepperShift: ") << endl;

      accel_stepper_shift->begin();

      for (int i = 0; i < accel_stepper_shift->motor_ct; i++) {
        AccelStepperNoted* m = accel_stepper_shift->motors[i];
        m->moveTo(300);
      }
    }

    boolean run() {
      accel_stepper_shift->run();
      return true;
    }
};
