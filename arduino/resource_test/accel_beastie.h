#pragma once

#include "begin_run.h"

class AccelBeastie : public BeginRun {
    // show memory and time used by all AccelStepper

  public:
    static void dummyStep() {}; // not actually used, see new AccelStepper

    AccelStepper* motors[MOTOR_CT];

    void begin() {
      Serial << F("SETUP AccelSteppers: ") << endl;

      for (int i = 0; i < MOTOR_CT; i++) {
        // no pin use
        motors[i] = new AccelStepper(&dummyStep, &dummyStep);
      }

      for (AccelStepper* m : motors) {
        m->setAcceleration(1000);
        m->setMaxSpeed(100);
        m->moveTo(300);
      }
    }

    void loop() {
      static boolean all_done = false;

      int done_ct = 0;
      int i = 0;
      for (AccelStepper* m : motors) {
        if ( ! m->run() ) {
          if (!all_done) Serial << F("  done ") << i << endl;
          done_ct ++;
        }
        i++;
      }
      if (done_ct >= MOTOR_CT) all_done = true;
    }
};
