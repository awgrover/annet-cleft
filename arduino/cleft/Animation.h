#pragma once

/*
  A manager/protocol,
  I plan on an animation framework, so this is an interim solution.
*/

#include "every.h"
#include "begin_run.h"
#include "AccelStepperShift.h"

class Animation : public BeginRun {
    // Protocol that Animations do.
    // FIXME: move to own file, or will change w/real animation framework
    // ALSO: manager for current_anmiation (class static)

  public:
    // keep track of your own state!
    // FIXME: document which are public states
    enum State { Restart, Starting, Running, Stopping, Idle, Off };

    // Which animation we are running
    static Animation* current_animation;
    static Animation* animations[];
    static const int animation_ct;

    AccelStepperShift* const all_motors; // ->motors[] of AccelStepper
    State state = Restart;

    Animation(AccelStepperShift* all_motors) : all_motors(all_motors) {}

    boolean is_running() {
     return ! ( state == Idle || state == Off);
    }
    
    void mirror_moveTo( int left_i, int amplitude) {
      // moves motor [left_i] and the mirror'd right side the same

      all_motors->motors[left_i]->moveTo( amplitude );
      all_motors->motors[ all_motors->motor_ct - left_i - 1 ]->moveTo( amplitude );
      Serial << F("Move ") << left_i << F( " / " ) << (all_motors->motor_ct - left_i - 1) << F(" TO ") << amplitude << endl;
    }

  virtual boolean commands(char command) {
    // handle a serial-input character as a command: return true
    // return false otherwise (someone else might handle it)
    return false;
  }
};
