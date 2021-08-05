#pragma once

#include "begin_run.h"
#include "AccelStepperShift.h"
#include "AnimationWave1.h"

class Commands : public BeginRun {
  public:
    AccelStepperShift* all_motors;

    Commands(AccelStepperShift* all_motors)
      : all_motors(all_motors)
    {}
    void begin() {
      // Signal the processing sketch
      Serial << F("<S") << endl;
    }

    boolean run() {
      if (Serial.available() >  0) {
        char command = Serial.read();
        Serial << F("* ") << command << endl; // echo

        switch (command) {
          case '?': // are you there?
            Serial << F("<S") << endl; // yes
            break;

          case 'Q': // tell positions
            Serial << F("Homing") << endl;
            Animation::current_animation->state = Animation::Off;
            for (int i = 0; i < all_motors->motor_ct; i++) {
              Serial << F("<P ") << i << F(" ") << all_motors->motors[i]->currentPosition() << endl;
            }
            break;

          case '0': // stop animation, goto 0
            Serial << F("Homing") << endl;
            Animation::current_animation->state = Animation::Off;
            for (int i = 0; i < all_motors->motor_ct; i++) {
              all_motors->motors[i]->moveTo( 0 );
            }
            break;
          case 'u': // stop animation, goto up
            Serial << F("Upping") << endl;
            Animation::current_animation->state = Animation::Off;
            for (int i = 0; i < all_motors->motor_ct; i++) {
              all_motors->motors[i]->moveTo( 0.5f * AccelStepperShift::STEPS_METER );
            }
            break;
          default:
            if (command >= '1' && command <= '9') {
              // run an animation
              int animation_i = command - '1';
              Serial << F("restart animation ") << animation_i << endl;
              if (animation_i < Animation::animation_ct) {
                Animation::current_animation->state = Animation::Off;
                Animation::current_animation = Animation::animations[animation_i];
                Animation::current_animation->state = Animation::Restart;
              }
            }
            return false; // not handling a command is false
        }

        return true; // handling a command is a true
      }
      return false;
    }
};