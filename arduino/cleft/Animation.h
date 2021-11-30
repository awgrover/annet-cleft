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

    virtual boolean run() {
      // move us through the states
      // override if you have weirdness
      State last_state = state;
      boolean handled = false; // actuall, "is still running?"
      
      switch (state) {
        case Restart:
          restart();
          handled = true;
          break;

        case Starting:
          startup();
          handled = true;
          break;

        case Running:
          running();
          return true;
          break;

        case Stopping:
          stopping();
          handled = true;
          break;

        case Idle:
          break;

        case Off:
          // don't restart
          break;

          
      }
      if (last_state != state) Serial << F("  state ") << state << endl;
      return handled; // IFF working
    }

    // state functions, override as needed
    virtual void restart() = 0; // must provide
    virtual void startup() {}
    virtual void running() {}
    virtual void stopping() {}

    static int meters_to_steps(const float m) {
      return AccelStepperShift::STEPS_METER * m;
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

class AnimationNoop  : public Animation {
    // A dumy animation that gets to IDLE immediately
  public:

    AnimationNoop( AccelStepperShift* all_motors)
      : Animation(all_motors)
    {
    }

    void restart() {
      Serial << F("noop anim") << endl;
      state = Idle;
    }

    void begin() {
    }   

    void startup() {
    }

    void running() {
    }

    void stopping() {
    }
};

class AnimationHome  : public Animation {
    // return to home
  public:

    AnimationHome( AccelStepperShift* all_motors)
      : Animation(all_motors)
    {
    }

    void restart() {
      Serial << F("home anim") << endl;
      for (int i = 0; i < all_motors->motor_ct; i++) {
        // easiest to just set all motors
        all_motors->motors[i]->setMaxSpeed(AccelStepperShift::MAX_SPEED);
        all_motors->motors[i]->setAcceleration(AccelStepperShift::MAX_SPEED * 10);
        all_motors->motors[i]->moveTo( 0 );
      }

      state = Stopping;
    }

    void begin() {
    }

    void stopping() {
      // use the last motor as the indicator
      // only check if we moved
      if ( all_motors->motors[ all_motors->motor_ct - 1]->do_step ) {

        for (int i = 0; i < all_motors->motor_ct; i++) {
          if ( all_motors->motors[ i ]->distanceToGo() != 0 ) {
            return; // wait till home
          }
        }
        state = Idle;
      }
    }
};
