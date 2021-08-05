#pragma once /* full sin wave that moves from each tip

*/

#include "every.h"
#include "begin_run.h"
#include "AccelStepperShift.h"

class Animation : public BeginRun {
  public:
    enum State { Restart, Starting, Running, Stopping, Idle, Off };
    static Animation* current_animation;
    static Animation* animations[];
    static const int animation_ct;

    AccelStepperShift* const all_motors; // ->motors[] of AccelStepper
    State state = Restart;

    Animation(AccelStepperShift* all_motors) : all_motors(all_motors) {}
};

class AnimationWave1  : public Animation {
  public:
    const float amplitude; // in steps
    const float cycle_fraction; // amount of full cycle that fits in 1/2 motor_ct
    const float frequency; // in cycles/sec
    const int half_segments;
    const int phase_delay; // in millis
    const int total_cycles; // how many to do

    int i_for_phase = 0;
    Every start_next_phase;
    int cycles = 0; // total cycles

    AnimationWave1(AccelStepperShift* all_motors, float amplitude_meters, float cycle_fraction, float frequency, int total_cycles)
      : Animation(all_motors),
        amplitude(amplitude_meters * AccelStepperShift::STEPS_METER),
        cycle_fraction(cycle_fraction),
        frequency(frequency),
        half_segments( ceil(all_motors->motor_ct / 2.0) ), // works for odd number of segments
        phase_delay( ((1.0 / frequency) / 2 / half_segments) * 1000 ), // 1/2 a cycle for each segment (of 1/2 of the whole)
        total_cycles(total_cycles)
    {
    }

    void restart() {
      state = Starting;
      Serial << F("AW start") << endl;
      i_for_phase = 0;
      cycles = 0;

      // FIXME: each motor, set maxspeed and accel to achieve frequency for amplitude

      float cycle_length = 1.0 / frequency; // seconds per cycle
      int distance = 2 * amplitude; // how far we'll need to move
      // we'll accel half the distance, then decel half
      // 1/2 at^2 = distance
      float time = (cycle_length / 2);
      int accel = (2 * distance) / (time * time); // steps per sec
      int speed = accel * time; // final speed

      // FIXME: annet liked it being not perfectly balanced
      // I had set the right-half segments to some other max/accel (the default I think)
      // so, re-add some? or do another animation w/systemic assymetry
      for (int i = 0; i < all_motors->motor_ct; i++) {
        all_motors->motors[i]->setMaxSpeed(speed);
        all_motors->motors[i]->setAcceleration(accel);
      }

      Serial << F("  Amplitude steps ") << amplitude << endl
             << F("  Cycle length secs ") << cycle_length << endl
             << F("  Distance ") << distance << endl
             << F("  Accel time secs ") << time << endl
             << F("  Accel steps/sec ") << accel << endl
             << F("  Maxspeed steps/sec ") << speed << endl
             ;

      start_next_phase.reset(phase_delay, true); // we intend to start running immediately, so start phase stuff immed
      cycles = 0;
    }

    void begin() {
    }

    boolean run() {
      switch (state) {
        case Restart:
          restart();
          state = Starting;
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
      running(); // need to handle runninge segments

      if (start_next_phase()) {
        // at each phase, start the next motor
        all_motors->motors[i_for_phase]->moveTo( amplitude );
        all_motors->motors[all_motors->motor_ct - i_for_phase - 1]->moveTo( amplitude );
        // FIXME: start the other 1/2 of the segments, where 0->max, 1->max-1, etc
        Serial << F("AW start i phase ") << millis() << F(" ") << i_for_phase << F(" & ") << (all_motors->motor_ct - i_for_phase - 1) << endl;
        Serial << F("L ") << all_motors->motors[i_for_phase]->currentPosition()
               << F(" R ") << all_motors->motors[i_for_phase]->currentPosition()
               << endl;

        i_for_phase++;

        // till we've started everybody
        if (i_for_phase >= half_segments) {
          Serial << F("AW running") << millis() << endl;
          state = Running;
        }
      }
    }

    void running() {

      // wait till we hit min/max, reverse (or note that we are done)
      // it would be nice if we just got signalled that the motor hit its target...
      // sadly, we have to check them all
      // (I suppose we could calculate exactly when we'd hit "done", and thus need only one test)
      for (int i = 0; i < half_segments; i++) {

        // only consider changes if we moved this time
        if ( all_motors->motors[i]->do_step ) {
          if ( abs(all_motors->motors[i]->currentPosition()) == (int) amplitude ) {
            // hit max/min, so reverse
            // FIXME: assuming around 0, add a around-home to all_motors or something
            // fixme: this reverses when [0..7] hit max, but that means [8..14] are 1 behind!
            //  so, we'll get slightly out of step (by 1 on each reverse)
            //  could fix by testing [7..14] instead...
            all_motors->motors[i]->moveTo( - all_motors->motors[i]->currentPosition() );
            all_motors->motors[all_motors->motor_ct - i - 1]->moveTo( - all_motors->motors[i]->currentPosition() );
            Serial << F("AW chg dir ") << millis() << F(" ") << i << F(" ") << (all_motors->motors[i]->currentPosition() > 0 ? -1 : 1) << endl;

            // FIXME: and the other 1/2

            // only need to count cycles at motor[0]
            // FIXME: end of thing is when [0] hits cycle, then each time we hit max, aim for zero, till all end & zerod
            if (i == 0 ) {
              cycles ++;
              // "+1" because we go: 0 -> amp -> 0 -> -amp -> 0 -> amp. So initial +amp, then twice at +amp,-amp for a cycle
              // so, really we do 1 + 1/4 cycles
              if (cycles >= (total_cycles * 2) + 1) {
                state = Stopping;
                Serial << F("AW stopping") << millis() << endl;
                for (int ii = 0; ii < half_segments; ii++) {
                  all_motors->motors[ii]->moveTo( 0 ); // because we won't notice in Stopping for [0]
                  all_motors->motors[all_motors->motor_ct - ii - 1]->moveTo( 0 ); // because we won't notice in Stopping for [0]
                }
                // FIXME: and the other 1/2
                return;
              }
            }
          }
        }
      }
    }

    void stopping() {
      // FIXME: this is wrong. keep phase shifting, but "tail" goes to 0 till all phased out, like runnning

      // wait till all hit 0
      // we don't look for do_step, because when we get to 0, there will be no do_step
      // because we are pre-change...
      static Every say_pos(100);
      for (int i = 0; i < half_segments; i++) {
        if (say_pos()) Serial << F("stopping@ ") << i << F(" ") << all_motors->motors[ i ]->currentPosition() << endl;
        if ( abs(all_motors->motors[ i ]->currentPosition()) != 0 ) {
          return;
        }
      }

      // we get here if everybody has hit 0
      state = Idle;
      Serial << F("AW idle") << millis() << endl;
    }
};
