#pragma once
// interactive animations

#include "Animation.h"
#include "every.h"

Timer bob(0);

class AnimationWornPosture  : public Animation {
    // FIXME: this could be a catmull-rom target
    /* Animation of being worn:
      shape of being worn, hold that till next animation
      later: subtle breathing...
    */

  public:
    const float amplitude; // in steps
    const unsigned int time_to_position; // msec to reach position
    const int breath_time; // inhale time
    const int breathing_amplitude;
    Timer *breath;
    int direction = -1; // we'll exhale first

    // "half" the segments: notionally 1..7 (8 dups 7) mirror for 9..15
    // precalced positions see worn-posture.py
    // magnitudes
    static const float positions[7]; // NB: see initialization at bottom

    AnimationWornPosture(AccelStepperShift* all_motors,
                         float amplitude_meters,
                         int time_to_position,
                         unsigned int breath_time, // how long an inhale takes, and how long to hold
                         int breathing_amplitude // cm
                        )

      : Animation(all_motors),
        amplitude(amplitude_meters * AccelStepperShift::STEPS_METER),
        time_to_position(time_to_position),
        breath_time(breath_time),
        breathing_amplitude( meters_to_steps( breathing_amplitude / 100.0 ) )
    {
      breath = new Timer(breath_time, false);
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

    void startup() {
      // wait till we get in position
      // re-used to do breating
      for (int i = 0; i < all_motors->motor_ct; i++) {
        if ( all_motors->motors[i]->distanceToGo() != 0 ) return;
      }
      breath->reset();

      state = Running;
    }

    void running() {
      // stay running

      // alternate up/down by 1 or 2 cm, taking 1/2 breath time (~1sec), pausing for 1sec

      // "hold" breath
      if ( ! (*breath)() ) return;

      // change to breathing movement
      for (int i = 0; i < all_motors->motor_ct; i++) {
        // the amount of breath is proportional to downwardness
        // i.e. the shoulders don't move, just chest/back
        int distance = direction * breathing_amplitude * (1 - positions[i] + 0.5);
        int accel = 2 * abs(distance) / 2 / ((breath_time * breath_time) / 4);
        if (accel < 10) accel = 10;
        int speed = 0.5 * accel * breath_time / 2;
        if (speed < 10) speed = 10;
        int target_position = positions[i] * amplitude + distance;
        all_motors->motors[i]->setMaxSpeed(speed);
        all_motors->motors[i]->setAcceleration(accel);
        all_motors->motors[i]->moveTo(target_position);
      }

      state = Starting; // cheat reuse
    }

    void stopping() {
      // we do not go to idle! we hold till told otherwise
    }
};
const float AnimationWornPosture::positions[7] = { -0.500, -0.000, 0.366, 0.500, 0.366, -0.000, -0.500}; // cos(-pi/2..pi/2) ct=7 mid=0.5 Sat Nov 27 14:54:14 2021

class AnimationJitter  : public Animation {
    /* act nervous, jittering around.
    */

  public:
    const float amplitude; // in steps
    int motor_i; // current motor

    AnimationJitter(AccelStepperShift* all_motors, float amplitude_meters)
      : Animation(all_motors),
        amplitude(amplitude_meters * AccelStepperShift::STEPS_METER),
        motor_i(0)
    {
    }

    void restart() {
      for (int i = 0; i < all_motors->motor_ct; i++) {
        all_motors->motors[i]->setMaxSpeed(AccelStepperShift::MAX_SPEED);
        all_motors->motors[i]->setAcceleration(AccelStepperShift::MAX_SPEED * 10);
      }

      state = Running;
    }

    void begin() {
    }

    int jitter(int motor_i) {
      // pick a new target for a motor
      int target = all_motors->motors[ motor_i ]->targetPosition();

      // ensure we get a different position
      while ( target == all_motors->motors[ motor_i ]->targetPosition() ) {
        target = meters_to_steps( random(-amplitude, amplitude + 1) / 100.0 );
      }
      return target;
    }

    void running() {
      // stay running
      // as a motor finishes, pick a new one
      if ( all_motors->motors[ motor_i ]->distanceToGo() == 0 ) {
        motor_i = random( all_motors->motor_ct );
        all_motors->motors[motor_i]->moveTo( jitter(motor_i) );
      }

    }

    void stopping() {
      // we do not go to idle! we hold till told otherwise
    }
};
