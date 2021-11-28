#pragma once

#include "begin_run.h"
#include "AccelStepperShift.h"
#include "Animation.h"

class Commands : public BeginRun {
    // Respond to commands on the serial port, mostly for demo & debug

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

        // reserve upper-case for other systems

        switch (command) {
          case '?': // are you there?
            Serial << F("<S") << endl; // yes
            break;

          case 'q': // tell positions
            Serial << F("Homing") << endl;
            Animation::current_animation->state = Animation::Off;
            for (int i = 0; i < all_motors->motor_ct; i++) {
              Serial << F("<P ") << i << F(" ") << all_motors->motors[i]->currentPosition() << endl;
            }
            break;

          case 'z': // goto_limit, does not reset usable_motor
            Serial << F("Goto Limt") << endl;
            Animation::current_animation->state = Animation::Off;
            all_motors->goto_limit();
            break;

          case '0': // stop animation, goto 0
            Serial << F("Homing") << endl;
            Animation::current_animation->state = Animation::Off;
            for (int i = 0; i < all_motors->motor_ct; i++) {
              all_motors->motors[i]->moveTo( 0 );
            }
            break;

          case 'x': // stop animation
            Serial << F("STOP") << endl;
            allow_random = false;
            for (int i = 0; i < all_motors->motor_ct; i++) {
              all_motors->motors[i]->setAcceleration(2000);
              all_motors->motors[i]->move( 0 );
              all_motors->motors[i]->stop();
            }
            Animation::current_animation->state = Animation::Off;
            break;

          case 'u': // stop animation, goto up
            Serial << F("Upping") << endl;
            Animation::current_animation->state = Animation::Off;
            for (int i = 0; i < all_motors->motor_ct; i++) {
              all_motors->motors[i]->moveTo( 0.5f * AccelStepperShift::STEPS_METER );
            }
            break;

          case '+': // up 2 steps
            for (int i = 0; i < all_motors->motor_ct; i++) {
              all_motors->motors[i]->move(2);
            }
            break;

          case '-': // down 2 steps
            for (int i = 0; i < all_motors->motor_ct; i++) {
              all_motors->motors[i]->move( -2 );
            }
            break;


          default:
            // 1..9A..Z: run an animation
            // single character commands are easier, so one char for 1..36
            // NB: 1->[0] because 1=="1st" for humans.
            if ( (command >= '1' && command <= '9') || (command >= 'A' && command <= 'F') ) {
              allow_random = true;

              int animation_i = command - (command <= '9' ? '1' : 'A');
              Serial << F("restart animation ") << (animation_i + 1 ) << endl;
              if (animation_i < Animation::animation_ct && Animation::animations[animation_i] ) {
                Animation::current_animation->state = Animation::Off;
                Animation::current_animation = Animation::animations[animation_i];
                Animation::current_animation->state = Animation::Restart;
                Serial << F("  animation is ") << ((long) Animation::current_animation)
                       << F(" @ ") << Animation::current_animation->state
                       << endl;
                break;
              }
              else {
                Serial << F("No Such Animation") << endl;
              }
            }
            else if ( Animation::current_animation && Animation::current_animation->commands( command ) ) {
              Serial << F("Handled by ") << ((long) Animation::current_animation) << endl;
              break;
            }

            Serial << F("wat ") << command << F("currenta? ") << (!! Animation::current_animation ) << endl;
            return false; // not handling a command is false
        }

        return true; // handling a command is a true
      }
      return false;
    }
};
