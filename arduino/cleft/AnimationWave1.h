/* animations using the Animation protocol
 *  
 */

#include "Animation.h"

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

      float cycle_length = 1.0 / frequency; // seconds per cycle
      int distance = 2 * amplitude; // how far we'll need to move
      // we'll accel half the distance, then decel half
      // 1/2 at^2 = distance
      float time = (cycle_length / 2);
      int accel = (2 * distance) / (time * time); // steps per sec
      int speed = accel * time; // final speed

      for (int i = 0; i < all_motors->motor_ct; i++) {
        // annet liked it being not perfectly balanced
        // so, systematically reduce speed:
        int this_speed = speed - i * 0.01 * speed;
        Serial << F("## ") << i << F(" ") << this_speed << endl;
        all_motors->motors[i]->setMaxSpeed(this_speed);
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
      running(); // need to handle runninge segments

      if (start_next_phase()) {
        // at each phase, start the next motor
        // "left" motors 0..1/2ct
        all_motors->motors[i_for_phase]->moveTo( amplitude );
        // right motors ct..1/2ct (-1)
        all_motors->motors[all_motors->motor_ct - i_for_phase - 1]->moveTo( amplitude );
        Serial << F("AW start i phase ") << millis() << F(" ") << i_for_phase << F(" & ") << (all_motors->motor_ct - i_for_phase - 1)
               << F(" to ") << amplitude << F(" tp ") << all_motors->motors[i_for_phase]->targetPosition() << endl;
        Serial << F("AW L ") << all_motors->motors[i_for_phase]->currentPosition()
               << F(" R ") << all_motors->motors[i_for_phase]->currentPosition()
               << endl;

        i_for_phase++;

        // till we've started everybody
        if (i_for_phase >= half_segments) {
          Serial << F("AW running ") << millis() << endl;
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
            // fixme: this reverses when [0..7] hit max, but that means [8..14] are 1 behind!
            //  so, we'll get slightly out of step (by 1 on each reverse)
            //  could fix by testing [7..14] instead...
            all_motors->motors[i]->moveTo( - all_motors->motors[i]->currentPosition() );
            all_motors->motors[all_motors->motor_ct - i - 1]->moveTo( - all_motors->motors[i]->currentPosition() );
            Serial << F("AW chg dir ") << millis() << F(" ") << i << F(" ") << (all_motors->motors[i]->currentPosition() > 0 ? -1 : 1) << endl;

            // only need to count cycles at motor[0]
            if (i == 0 ) {
              cycles ++;
              // "+1" because we go: 0 -> amp -> 0 -> -amp -> 0 -> amp. So initial +amp, then twice at +amp,-amp for a cycle
              // so, really we do 1 + 1/4 cycles
              if (cycles >= (total_cycles * 2) + 1) {
                state = Stopping;
                Serial << F("AW stopping ") << millis() << endl;
                for (int ii = 0; ii < half_segments; ii++) {
                  all_motors->motors[ii]->moveTo( 0 ); // because we won't notice in Stopping for [0]
                  all_motors->motors[all_motors->motor_ct - ii - 1]->moveTo( 0 ); // because we won't notice in Stopping for [0]
                }
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
        //if (say_pos()) Serial << F("stopping@ ") << i << F(" ") << all_motors->motors[ i ]->currentPosition() << endl;
        if ( abs(all_motors->motors[ i ]->currentPosition()) != 0 ) {
          return;
        }
      }

      // we get here if everybody has hit 0
      state = Idle;
      Serial << F("AW idle") << millis() << endl;
    }
};

class AnimationWave2  : public Animation {
    // a "cute" hand wave
    // but this fails, it's actually a diminishing oscillation
  public:
    const float amplitude; // in steps
    const float frequency; // in cycles/sec
    const int segments;
    const int total_cycles; // how many to do

    int cycles = 0; // total cycles

    AnimationWave2(AccelStepperShift* all_motors, int segments, float amplitude_meters, float frequency, int total_cycles)
      : Animation(all_motors),
        amplitude(amplitude_meters * AccelStepperShift::STEPS_METER),
        frequency(frequency),
        segments( segments ), // works for odd number of segments
        total_cycles(total_cycles)
    {
    }

    void restart() {
      Serial << F("AW start ") << ((long) this) << endl;
      cycles = 0;

      float cycle_length = 1.0 / frequency; // seconds per cycle
      int distance = (amplitude / 2.0); // how far we'll need to move
      // we'll accel half the distance, then decel half
      // 1/2 at^2 = distance
      float time = (cycle_length / 2);
      int accel = (2 * distance) / (time * time); // steps per sec
      int speed = accel * time; // final speed

      // initial target is the max-amplitude of each segment
      for (int i = 0; i < all_motors->motor_ct; i++) {
        // easiest to just set all motors
        all_motors->motors[i]->setMaxSpeed(speed);
        all_motors->motors[i]->setAcceleration(accel);
        // but only "move" those of interest
        if (i < segments) mirror_moveTo( i, amplitude_for_segment(i, HIGH) );
      }

      Serial << F("  Segments ") << segments << endl
             << F("  Amplitude steps ") << amplitude << endl
             << F("  Cycle length secs ") << cycle_length << endl
             << F("  Distance ") << distance << endl
             << F("  Accel time secs ") << time << endl
             << F("  Accel steps/sec ") << accel << endl
             << F("  Maxspeed steps/sec ") << speed << endl
             ;
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

      // wait till we hit min/max, reverse (or note that we are done)
      // it would be nice if we just got signalled that the motor hit its target...
      // sadly, we have to check them all
      // (I suppose we could calculate exactly when we'd hit "done", and thus need only one test)
      for (int i = 0; i < segments; i++) {
        // Serial << F("MOTOR ") << i << endl;
        // only consider changes if we moved this time
        if ( all_motors->motors[i]->do_step ) {
          const boolean direction = all_motors->motors[i]->direction();
          const int min_max_amp = amplitude_for_segment(i, direction);
          const long pos = all_motors->motors[i]->currentPosition();
          const boolean hit_min_max = direction ? pos >= min_max_amp : pos <= min_max_amp;
          //Serial << F("  dir ") << direction << F(" pos ") << pos
          //       << F(" minmax ") << min_max_amp << F(" hit? ") << hit_min_max
          //       << endl;

          if ( hit_min_max ) {
            // so reverse
            mirror_moveTo(i, amplitude_for_segment(i, ! direction) );

            Serial << F("AW chg dir ") << millis() << F(" ") << i << F(" ") << (all_motors->motors[i]->currentPosition() > 0 ? -1 : 1) << endl;

            // only need to count cycles at motor[max]
            if (i == 0) {
              cycles ++;
              // "+1" because we go: 0 -> amp -> 0 -> -amp -> 0 -> amp.
              // so, really we do 1 + 4 cycles
              if (cycles >= (2 * total_cycles) + 1) {
                state = Stopping;
                Serial << F("AW stopping") << millis() << endl;
                for (int ii = 0; ii < segments; ii++) {
                  mirror_moveTo(ii, 0 );
                }
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
      for (int i = 0; i < segments; i++) {
        if (say_pos()) Serial << F("stopping@ ") << i << F(" ") << all_motors->motors[ i ]->currentPosition() << endl;
        if ( abs(all_motors->motors[ i ]->currentPosition()) != 0 ) {
          return;
        }
      }

      // we get here if everybody has hit 0
      state = Idle;
      Serial << F("AW idle") << millis() << endl;
    }

    int amplitude_for_segment(int motor_i, boolean top) {
      // full amplitude for [0], reducing to 10% for nth
      float fraction = (segments - motor_i) / (float) segments;
      int max_amp = amplitude * fraction;
      return top ? max_amp : (fraction * (amplitude / 2));
    }
};

class AnimationWaveCute  : public Animation {
    // a "cute" hand wave
    // Start by lifting the first
    // Then it oscillates around ~ + 5cm
    // The next couple of segments follow, but their "around" is progressively less

  public:
    const float amplitude; // in steps
    const float frequency; // in cycles/sec
    const int segments;
    const int total_cycles; // how many to do

    int cycles = 0; // total cycles

    AnimationWaveCute(AccelStepperShift* all_motors, int segments, float amplitude_meters, float frequency, int total_cycles)
      : Animation(all_motors),
        amplitude(amplitude_meters * AccelStepperShift::STEPS_METER),
        frequency(frequency),
        segments( segments ), // works for odd number of segments
        total_cycles(total_cycles)
    {
    }

    void restart() {
      Serial << F("AW start ") << ((long) this) << endl;
      cycles = 0;

      float cycle_length = 1.0 / frequency; // seconds per cycle
      int distance = (amplitude / 2.0); // how far we'll need to move
      // we'll accel half the distance, then decel half
      // 1/2 at^2 = distance
      float time = (cycle_length / 2);
      int accel = (2 * distance) / (time * time); // steps per sec
      int speed = accel * time; // final speed

      // initial target is the max-amplitude of each segment
      for (int i = 0; i < all_motors->motor_ct; i++) {
        // easiest to just set all motors
        all_motors->motors[i]->setMaxSpeed(speed);
        all_motors->motors[i]->setAcceleration(accel);
        // but only "move" those of interest
        if (i < segments) all_motors->motors[i]->moveTo( amplitude_for_segment(i, HIGH) );
      }

      Serial << F("  Segments ") << segments << endl
             << F("  Amplitude steps ") << amplitude << endl
             << F("  Cycle length secs ") << cycle_length << endl
             << F("  Distance ") << distance << endl
             << F("  Accel time secs ") << time << endl
             << F("  Accel steps/sec ") << accel << endl
             << F("  Maxspeed steps/sec ") << speed << endl
             ;
    }

    void begin() {
    }

    boolean run() {
      switch (state) {
        case Restart:
          restart();
          state = Running; // skip starting
          break;

        case Starting:
          // skipped
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

      // wait till we hit min/max on [0], reverse everybody (or note that we are done)
      // it would be nice if we just got signalled that the motor hit its target...
      int leader = 0;

      // only consider changes if we moved this time
      if ( all_motors->motors[leader]->do_step ) {
        const boolean direction = all_motors->motors[leader]->direction();
        const int min_max_amp = amplitude_for_segment(leader, direction);
        const long pos = all_motors->motors[leader]->currentPosition();
        const boolean hit_min_max = direction ? pos >= min_max_amp : pos <= min_max_amp;
        //Serial << F("  dir ") << direction << F(" pos ") << pos
        //       << F(" minmax ") << min_max_amp << F(" hit? ") << hit_min_max
        //       << endl;

        if ( hit_min_max ) {
          // so reverse (all participants)
          for (int i = 0; i < segments; i++) {

            all_motors->motors[i]->moveTo( amplitude_for_segment(i, ! direction) );

            Serial << F("AW chg dir ") << millis() << F(" ") << i << F(" ") << (all_motors->motors[leader]->currentPosition() > 0 ? -1 : 1) << endl;
          }

          // only need to count cycles at motor[max]
          cycles ++;
          // "+1" because we go: 0 -> amp -> 0 -> -amp -> 0 -> amp.
          // so, really we do 1 + 4 cycles
          if (cycles >= (2 * total_cycles) + 1) {
            state = Stopping;
            Serial << F("AW stopping") << millis() << endl;
            for (int ii = 0; ii < segments; ii++) {
              mirror_moveTo(ii, 0 );
            }
            return;
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
      for (int i = 0; i < segments; i++) {
        if (say_pos()) Serial << F("stopping@ ") << i << F(" ") << all_motors->motors[ i ]->currentPosition() << endl;
        if ( abs(all_motors->motors[ i ]->currentPosition()) != 0 ) {
          return;
        }
      }

      // we get here if everybody has hit 0
      state = Idle;
      Serial << F("AW idle") << millis() << endl;
    }

    int amplitude_for_segment(int motor_i, boolean top) {
      // we oscillate around amp/2
      // full amplitude for [0], reducing to 10% for nth
      float fraction = (segments - motor_i) / (float) segments;
      int max_amp = amplitude * fraction;
      return top ? max_amp : (fraction * (amplitude / 2));
    }
};
