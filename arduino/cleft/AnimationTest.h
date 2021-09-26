#pragma once

/*
  Several test animations
*/

#include "Animation.h"

class AnimationFast : public Animation {
    // run real fast
  public:
    // for the  smaller motor at 0.5A, 400 is about max (no microstep)
    static constexpr int speed = 1000; // steps per sec, about fastest I can run
    static constexpr int time_to_accel = 2; // secs

    AnimationFast(AccelStepperShift* all_motors)
      : Animation(all_motors)
    {
    }

    void restart() {
      Serial << F("AF start ") << ((long) this) << endl;

      // Want to get up to speed, run for 1 second, then decl
      int accel = speed / time_to_accel; // accel to speed in 2 second
      int distance_while_accelerating = accel / 2 * time_to_accel * time_to_accel;
      int total_distance = 2 * distance_while_accelerating + speed;

      // initial target is the max-amplitude of each segment
      for (int i = 0; i < all_motors->motor_ct; i++) {
        all_motors->motors[i]->setMaxSpeed(speed);
        all_motors->motors[i]->setAcceleration(accel);
        all_motors->motors[i]->move( total_distance );
      }
    }

    void begin() {
    }

    boolean run() {
      switch (state) {
        case Restart:
          restart();
          state = Running;
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
      // Nothing to do, just run
      Serial << F("FAIL: should do startup") << endl;
      delay(500);
    }

    void running() {

      // Nothing to do, just run
    }

    void stopping() {
      // we never get here
    }
};

class AnimationMotorTests : public Animation {
    // Takes various commands to run motors in various ways
    // Type '/' to get commands

  public:
    // for the  smaller motor at 0.5A, 400 is about max (no microstep)
    int time_to_accel = 2; // secs
    int distance = 200; // secs
    int speed = 200; // steps per sec

    AnimationMotorTests(AccelStepperShift* all_motors)
      : Animation(all_motors)
    {
    }

    void restart() {
      Serial << F("AF start ") << ((long) this) << endl;
      allow_random = false; // disable idles while testing

      // Want to get up to speed, run for 1 second, then decl
      int accel = speed / time_to_accel; // accel to speed in 2 second
      int distance_while_accelerating = accel / 2 * time_to_accel * time_to_accel;
      int total_distance = 2 * distance_while_accelerating + speed;

      // initial target is the max-amplitude of each segment
      for (int i = 0; i < all_motors->motor_ct; i++) {
        all_motors->motors[i]->setMaxSpeed(speed);
        all_motors->motors[i]->setAcceleration(accel);
      }
      Serial << F("Speed ") << speed
             << F(" Accel ") << accel
             << F(" to ") << distance
             << endl;
      state = Starting;
    }

    void begin() {
    }

    boolean run() {
      switch (state) {
        case Restart:
          restart();
          break;

        case Starting:
          startup();
          break;

        case Running:
          running();
          break;

        case Stopping:
          stopping();
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
      // Nothing to do, just run
      for (int i = 0; i < all_motors->motor_ct; i++) {
        all_motors->motors[i]->move( distance );
      }
      Serial << F("+") << endl;
      state = Running;
    }

    void running() {
      if (all_motors->all_done) {
        for (int i = 0; i < all_motors->motor_ct; i++) {
          all_motors->motors[i]->move( - distance );
        }
        Serial << F("-") << endl;
        state = Stopping;
      }
    }

    void stopping() {
      if (all_motors->all_done) {
        Serial << F("!") << endl;
        state = Idle;
        Serial << F("Speed ") << speed
               << F(" to ") << distance
               << endl;
      }
    }

    boolean commands(char command) {
      boolean handled = false;
      Serial << F("Try ") << command << endl;

      switch (command) {
        case '/': // help
          Serial
              << F("A  all motors forward/backward slow") << endl
              << F("B  all motors forward/backward medium") << endl
              << F("C  all motors forward/backward fast") << endl
              << F("/  help")
              << endl;
          handled = true;
          break;

        case 'A' :
          speed = 60;
          distance = speed * 8;
          restart();
          break;

        case 'B' :
          speed = 200;
          distance = 200 * 4;
          restart();
          break;

        case 'C' :
          speed = 1000;
          distance = 200 * 4;
          restart();
          break;

        default:
          Serial << F("???") << endl;
          break;
      }

      return handled;
    }
};

class AnimationIntegrationTests : public Animation {
    // Allows testing of motor & switch:
    // clockwise while limit is open, counter while closed
    // Type '/' to get commands

  public:
    // for the  smaller motor at 0.5A, 400 is about max (no microstep)
    int time_to_accel = 2; // secs
    int speed = (200 * 8) / 3; // steps per sec: steps/rev * micro/step / secs

    AnimationIntegrationTests(AccelStepperShift* all_motors)
      : Animation(all_motors)
    {
    }

    void restart() {
      Serial << F("AI start ") << ((long) this) << endl;
      allow_random = false; // disable idles while testing

      // Want to get up to speed, run for 1 second, then decl
      int accel = speed / time_to_accel; // accel to speed in 2 second
      int distance_while_accelerating = accel / 2 * time_to_accel * time_to_accel;
      int total_distance = 2 * distance_while_accelerating + speed;

      // initial target is the max-amplitude of each segment
      for (int i = 0; i < all_motors->motor_ct; i++) {
        all_motors->motors[i]->setMaxSpeed(speed);
        all_motors->motors[i]->setAcceleration(accel);
      }
      Serial << F("Speed ") << speed
             << F(" Accel ") << accel
             << endl;
      state = Starting;
    }

    void begin() {
    }

    boolean run() {
      switch (state) {
        case Restart:
          restart();
          break;

        case Starting: // clockwise
          startup();
          break;

        case Running: // limit-closed: counter clock
          running();
          break;

        case Stopping:
          stopping();
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
      // Nothing to do, just run
      Serial << F("+") << endl;
      state = Running;
    }

    void running() {
      // determine direction
      for (int i = 0; i < all_motors->motor_ct; i++) {
        // we don't call runspeed, so just keep moving the goalpost
        // (but not every loop)
        if ( abs(all_motors->motors[i]->targetPosition() - all_motors->motors[i]->currentPosition() ) < 50 ) {
          long direction =  all_motors->limit_switch(i) ? speed : -speed; // arbitrary, sometime in the future
          all_motors->motors[i]->move( direction );
        }
      }
      Serial << F("-") << endl;

      // never leaves this state
    }

    void stopping() {
      // we never get here
      Serial << F("!") << endl;
    }

    boolean commands(char command) {
      boolean handled = false;
      Serial << F("Try ") << command << endl;

      switch (command) {
        case '/': // help
          Serial
              << F("A  2x speed") << endl
              << F("B  / 2 speed") << endl
              << F("/  help")
              << endl;
          handled = true;
          break;

        case 'A' :
          speed = speed * 2;
          restart();
          break;

        case 'B' :
          speed = speed / 2;
          restart();
          break;

        
        default:
          Serial << F("???") << endl;
          break;
      }

      return handled;
    }
};
