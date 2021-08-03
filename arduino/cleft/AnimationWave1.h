#pragma once /* full sin wave that moves from each tip

*/

#include "every.h"
#include "begin_run.h"
#include "AccelStepperShift.h"

class AnimationWave1  : public BeginRun {
  public:
    AccelStepperShift* const all_motors; // ->motors[] of AccelStepper
    const float amplitude; // in steps
    const float cycle_fraction; // amount of full cycle that fits in 1/2 motor_ct
    const float frequency; // in cycles/sec
    const int half_segments;
    const int phase_delay; // in millis
    const int total_cycles; // how many to do

    enum State { Starting, Running, Stopping, Idle };
    State state = Starting;
    int i_for_phase = 0;
    Every start_next_phase;
    int cycles = 0; // total cycles

    AnimationWave1(AccelStepperShift* all_motors, float amplitude_meters, float cycle_fraction, float frequency, int total_cycles)
      : all_motors(all_motors),
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

      // FIXME: each motor, set maxspeed and accel to achieve frequency for amplitude

      float cycle_length = 1.0 / frequency; // seconds per cycle
      int distance = 2 * amplitude; // how far we'll need to move
      // we'll accel half the distance, then decel half
      // 1/2 at^2 = distance
      float time = (cycle_length / 2);
      int accel = (2 * distance) / (time * time); // steps per sec
      int speed = accel * time; // final speed

      for (int i = 0; i < half_segments; i++) {
        all_motors->motors[i]->setMaxSpeed(speed);
        all_motors->motors[i]->setAcceleration(accel);
      }

      Serial << F("Cycle length secs ") << cycle_length << endl
             << F("Distance ") << distance << endl
             << F("Accel time secs ") << time << endl
             << F("accel steps/sec ") << accel << endl
             << F("maxspeed steps/sec ") << speed << endl
             ;

      start_next_phase.reset(phase_delay, true); // we intend to start running immediately, so start phase stuff immed
      cycles = 0;
    }

    void begin() {
      restart();
    }

    boolean run() {
      switch (state) {
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
      }
      return false; // IFF working
    }

    void startup() {
      if (start_next_phase()) {
        // at each phase, start the next motor
        all_motors->motors[i_for_phase]->moveTo( amplitude );
        // FIXME: start the other 1/2 of the segments, where 0->max, 1->max-1, etc
        i_for_phase++;

        // till we've started everybody
        if (i_for_phase >= half_segments) {
          Serial << F("AW running") << endl;
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
        if ( abs(all_motors->motors[i]->currentPosition()) == amplitude ) {
          // hit max/min, so reverse
          // FIXME: assuming around 0, add a around-home to all_motors or something
          all_motors->motors[i]->moveTo( - all_motors->motors[i]->currentPosition() );
          Serial << F("AW chg dir ") << i << (all_motors->motors[i]->currentPosition() > 0 ? -1 : 1) << endl;

          // FIXME: and the other 1/2

          // only need to count cycles at motor[0]
          if (i == 0) {
            cycles ++;
            if (cycles >= total_cycles) {
              state = Stopping;
              Serial << F("AW stopping") << endl;
              all_motors->motors[i]->moveTo( 0 ); // because we won't notice in Stopping for [0]
              // FIXME: and the other 1/2
            }
          }
        }
      }
    }

    void stopping() {
      // wait till all hit 0
      for (int i = 0; i < half_segments; i++) {
        if ( abs(all_motors->motors[ i ]->currentPosition()) != 0 ) {
          return;
        }
      }

      // we get here if everybody has hit 0
      state = Idle;
      Serial << F("AW idle") << endl;

    }
};
