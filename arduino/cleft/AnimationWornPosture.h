#pragma once

/* Animation of being worn:
    shape of being worn, hold that till next animation
    later: subtle breathing...
*/

#include "Animation.h"

class AnimationWornPosture  : public Animation {
    // FIXME: this could be a catmull-rom target
  public:
    const float amplitude; // in steps
    const unsigned int time_to_position; // msec to reach position

    // "half" the segments: notionally 1..7 (8 dups 7) mirror for 9..15
    // precalced positions see worn-posture.py
    // magnitudes
    static const float positions[7]; // NB: see initialization at bottom

    AnimationWornPosture(AccelStepperShift* all_motors, float amplitude_meters, int time_to_position)
      : Animation(all_motors),
        amplitude(amplitude_meters * AccelStepperShift::STEPS_METER),
        time_to_position(time_to_position)
    {
    }

    void restart() {
      // just set targets
      Serial << F("WP start") << endl;

      for (unsigned int i = 0; i < array_size(positions); i++) {
        // annet liked it being not perfectly balanced
        // so, systematically reduce speed:
        int target_position = positions[i] * amplitude;
        // to go half the distance in half the time:
        // (keep in float for small values)
        float accel = (2.0 * target_position / 2) / ((time_to_position * time_to_position) / 2);
        float speed = accel * time_to_position / 2; // final speed
        if (speed < 1) speed = 1.0;
        Serial << F("## ") << i << F(" ") << int(speed) << endl;

        all_motors->motors[i]->setMaxSpeed(speed);
        all_motors->motors[i]->setAcceleration(accel);
        all_motors->motors[i]->moveTo(target_position);
        if (i == 7) {
          // 8=7
          all_motors->motors[8]->setMaxSpeed(speed);
          all_motors->motors[8]->setAcceleration(accel);
          all_motors->motors[8]->moveTo(target_position);
        }
        // mirrors 9..15
        all_motors->motors[all_motors->motor_ct - i]->setMaxSpeed(speed);
        all_motors->motors[all_motors->motor_ct - i]->setAcceleration(accel);
        all_motors->motors[all_motors->motor_ct - i]->moveTo(target_position);
      }

      state = Starting;
    }

    void begin() {
    }

    boolean run() {
      switch (state) {
        case Restart:
          restart();
          return true;
          break;

        case Starting:
          startup();
          return true;
          break;

        case Running:
          running();
          return true;
          break;

        case Stopping:
          stopping();
          return true;
          break;

        case Idle:
          break;

        case Off:
          // don't restart
          break;
      }
      return false; // IFF working
    }

    void startup() {
      // wait till we hit min/max, reverse (or note that we are done)
      // it would be nice if we just got signalled that the motor hit its target...
      // sadly, we have to check them all
      // (I suppose we could calculate exactly when we'd hit "done", and thus need only one test)
      for (unsigned int i = 0; i < 6 /*array_size(positions)*/; i++) {
        int target_position = 0; // positions[i] * amplitude;
        if ( all_motors->motors[i]->currentPosition() != target_position ) return;
        if ( all_motors->motors[all_motors->motor_ct - i]->currentPosition() != target_position ) return;
        if ( i == 7 && all_motors->motors[8]->currentPosition() != target_position ) return;
      }

      state = Running;
    }

    void running() {
      // stay running
      // FIXME: breathing goes here
    }

    void stopping() {
      // we do not go to idle! we hold till told otherwise
      Serial << F("WP idle") << millis() << endl;
    }
};
const float AnimationWornPosture::positions[7] = { -0.500, -0.000, 0.366, 0.500, 0.366, -0.000, -0.500}; // cos(-pi/2..pi/2) ct=7 mid=0.5 Sat Nov 27 14:54:14 2021
