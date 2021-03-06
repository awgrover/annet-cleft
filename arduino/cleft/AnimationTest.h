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

      // initial target is the max-amplitude of each segment
      for (int i = 0; i < all_motors->motor_ct; i++) {
        all_motors->motors[i]->setMaxSpeed(speed);
        all_motors->motors[i]->setAcceleration(accel);
        all_motors->motors[i]->setCurrentPosition( 0 ); // just assume, no limit...
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
      static Every print_limit(300);
      if ( print_limit() ) {
        Serial << F("          ") << F("          ") << F("          ") << F("          ");
        for (int i = 0; i < all_motors->motor_ct; i++) {
          if ( i != 0 && i % 4 == 0 ) Serial << F(" . ");
          Serial << all_motors->limit_switch(i) << F(" ");
        }
        Serial << F("    ");
        all_motors->dump_bit_vector( all_motors->limit_switch_bit_vector );
        Serial << endl;
      }
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
        state = Starting;
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

      // initial target is the max-amplitude of each segment
      for (int i = 0; i < all_motors->motor_ct; i++) {
        all_motors->motors[i]->setMaxSpeed(speed);
        all_motors->motors[i]->setAcceleration(accel);
        all_motors->motors[i]->setCurrentPosition( 0 ); // just assume, no limit...
      }
      Serial << F("Speed ") << speed
             << F(" Accel ") << accel
             << endl;
      state = Starting;
    }

    void begin() {
    }

    boolean run() {
      static Every print_limit(300);
      if ( print_limit() ) {
        /*
          Serial << F("          ") << F("          ") << F("          ") << F("          ");
          for (int i = 0; i < all_motors->motor_ct; i++) {
          if ( i != 0 && i % 4 == 0 ) Serial << F(" . ");
          Serial << all_motors->limit_switch(i) << F(" ");
          }
          Serial << F("    ");
          all_motors->dump_bit_vector( all_motors->limit_switch_bit_vector );
          Serial << endl;
        */
      }

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

        int direction =  all_motors->limit_switch(i) ? -1 : 1;
        boolean accel_direction = all_motors->limit_switch(i) ? 0 : 1;

        // we don't call runspeed, so just keep moving the goalpost
        // (but not every loop)
        if ( accel_direction != all_motors->motors[i]->direction()
             || ( abs(all_motors->motors[i]->targetPosition() - all_motors->motors[i]->currentPosition() ) ) < 50
           ) {
          all_motors->motors[i]->move( direction * speed );
          /*
            Serial << F("Update ") << i << F(" dir ") << accel_direction << F(" to ") << (direction*speed) << endl;
            Serial << F("    is ") << all_motors->motors[i]->currentPosition()
            << F(" dir ") <<  all_motors->motors[i]->direction()
            << F(" to ") << all_motors->motors[i]->targetPosition()
            << endl;
          */
        }
      }
      // Serial << F("-") << endl;

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

class AnimationSequenceTests : public Animation {
    // Runs motors in sequence, forward, then reverse
    // Allows choosing each motor: M[0-9a-f] or All
    // clockwise while limit is open, counter while closed
    // Type '/' to get commands

  public:
    // for the  smaller motor at 0.5A, 400 is about max (no microstep)
    int time_to_accel = 2; // secs
    int speed = (200 * 8) / 3; // steps per sec: steps/rev * micro/step / secs
    int which_motor = -1; // all
    boolean which_direction = 0; // forward
    int motor_i = 0; // current motor

    AnimationSequenceTests(AccelStepperShift* all_motors)
      : Animation(all_motors)
    {
    }

    void restart() {
      Serial << F("AS start ") << ((long) this) << endl;
      allow_random = false; // disable idles while testing

      // Want to get up to speed, run for 1 second, then decl
      int accel = speed / time_to_accel; // accel to speed in 2 second

      // initial target is the max-amplitude of each segment
      for (int i = 0; i < all_motors->motor_ct; i++) {
        all_motors->motors[i]->setMaxSpeed(speed);
        all_motors->motors[i]->setAcceleration(accel);
        all_motors->motors[i]->setCurrentPosition( 0 ); // just assume, no limit...
      }
      Serial << F("Speed ") << speed
             << F(" Accel ") << accel
             << endl;

      which_direction = 0; // forward
      motor_i = which_motor == -1 ? 0 : which_motor;

      state = Starting;
    }

    void begin() {
    }

    boolean run() {
      static Every print_limit(300);
      if ( print_limit() ) {
        /*
          Serial << F("          ") << F("          ") << F("          ") << F("          ");
          for (int i = 0; i < all_motors->motor_ct; i++) {
          if ( i != 0 && i % 4 == 0 ) Serial << F(" . ");
          Serial << all_motors->limit_switch(i) << F(" ");
          }
          Serial << F("    ");
          all_motors->dump_bit_vector( all_motors->limit_switch_bit_vector );
          Serial << endl;
        */
      }

      switch (state) {
        case Restart:
          restart();
          break;

        case Starting:
          startup(); // set current motor
          break;

        case Running: // wait for it to finish, next motor & dir
          running();
          break;

        case Stopping:
          // not used
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
      int direction = which_direction ? 1 : -1 ;
      all_motors->motors[motor_i]->move( direction * speed );
      Serial << F("Motor ") << motor_i << F(" dir ") << which_direction << F(" for ") << speed << endl;

      state = Running;
    }

    void running() {
      // wait for finish
      if (! all_motors->all_done) return;

      if (which_motor == -1) {
        // reverse when we finish last
        if (motor_i == MOTOR_CT - 1) which_direction = ! which_direction; // reverse

        motor_i = (motor_i + 1) % MOTOR_CT;
      }
      else {
        which_direction = ! which_direction; // reverse
      }

      state = Starting;
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
              << F("Mn  only motor n 0-9a-f") << endl
              << F("A  all motors") << endl
              << F("/  help")
              << endl;
          handled = true;
          break;

        case 'A' :
          which_motor = -1;
          motor_i = 0;
          Serial << F("Motor All") << endl;
          restart();
          handled = true;
          break;

        case 'M' :
          {
            Serial << F("A-O (0..15)? ");
            while (Serial.available() <= 0) { delay(50); }
            char c = Serial.read();
            Serial << endl;
            if (c >= 'A' && c <= 'O') {
              which_motor = c - 'A';
              motor_i = which_motor;
              handled = true;
            }
            else {
              Serial << F("Not A-O");
              break;
            }
            Serial << F("Motor ") << which_motor << endl;
            restart();
            break;
          }


        default:
          Serial << F("???") << endl;
          break;
      }

      return handled;
    }
};
