#pragma once
// interactive animations

#include "Animation.h"
#include "every.h"

class AnimationWornPosture  : public Animation {
    // FIXME: this could be a catmull-rom target
    /* Animation of being worn:
      shape of being worn, hold that till next animation
      later: subtle breathing...
    */

  public:
    const float amplitude; // in steps
    const unsigned int time_to_position; // msec to reach position
    const int breathing_amplitude;
    enum BreathStates { BreathHold, BreathIn, BreathOut, BreathingTimesCount };
    BreathStates breath_state; // start on a hold
    unsigned int breathing_times[BreathingTimesCount];
    Timer *breath;
    Timer timeout = *(new Timer(10 * 1000)); // exit animation after this long

    // "half" the segments: notionally 1..7 (8 dups 7) mirror for 9..15
    // precalced positions see worn-posture.py
    // magnitudes
    static const float positions[7]; // NB: see initialization at bottom
    static const float MidPointBias; // see initialization at bottom
    
    AnimationWornPosture(AccelStepperShift* all_motors,
                         float amplitude_meters,
                         int time_to_position,
                         float breathing_amplitude, // m
                         unsigned int breath_in, // how long
                         unsigned int breath_out, // how long
                         unsigned int breath_pause // how log
                        )

      : Animation(all_motors),
        amplitude( meters_to_steps(amplitude_meters) ),
        time_to_position(time_to_position),
        breathing_amplitude( meters_to_steps( breathing_amplitude ) )
    {
      breath_state = BreathHold;
      breathing_times[BreathIn] = breath_in;
      breathing_times[BreathOut] = breath_in;
      breathing_times[BreathHold] = breath_in;
      breath = new Timer(breath_pause, false);
    }

    void restart() {
      // Move to initial "shape"

      Serial << F("Worn start to " ) << amplitude << F(" in ") << time_to_position << F("msec") << endl;

      timeout.reset();
      
      for (unsigned int i = 0; i < array_size(positions); i++) {
        // annet liked it being not perfectly balanced
        // so, systematically reduce speed:
        int target_position = positions[i] * amplitude;
        int delta = target_position - all_motors->motors[i]->currentPosition();
        // to go half the distance in half the time:
        // (keep in float for small values)
        float accel = (2.0 * abs(delta) / 2) / ((time_to_position / 1000 * time_to_position / 1000) / 2);
        if (accel < 10) accel = 10.0;
        float speed = accel * time_to_position / 1000 / 2; // final speed
        if (speed < 1) speed = 10.0;
        /*Serial << F("## ") << i << F("/") << (all_motors->motor_ct - i - 1) << (i == 6 ? "&7" : "")
               << F(" to ") << target_position
               << F(" delta ") << delta
               << F(" ") << int(speed) << F(" st/sec")
               << F(" ") << accel << F(" st/sec-sec")
               << endl;
        */
        all_motors->motors[i]->setMaxSpeed(speed);
        all_motors->motors[i]->setAcceleration(accel);
        all_motors->motors[i]->moveTo(target_position);
        if (i == 6) {
          // 7=6
          all_motors->motors[7]->setMaxSpeed(speed);
          all_motors->motors[7]->setAcceleration(accel);
          all_motors->motors[7]->moveTo(target_position);
        }
        // mirrors 9..15
        all_motors->motors[all_motors->motor_ct - i - 1]->setMaxSpeed(speed);
        all_motors->motors[all_motors->motor_ct - i - 1]->setAcceleration(accel);
        all_motors->motors[all_motors->motor_ct - i - 1]->moveTo(target_position);
      }

      breath_state = BreathHold;
      state = Starting; // move to initial position
    }

    void begin() {
    }

    void startup() {
      // moving, wait till we get to the position from startup OR running
      // then go to running() and decide what to do next
      // re-used to do breathing
      static Every debug(200);

      for (int i = 0; i < all_motors->motor_ct; i++) {
        // if (debug()) Serial << F("[") << i << F(" d ") << all_motors->motors[i]->distanceToGo() << endl;
        if ( all_motors->motors[i]->distanceToGo() != 0 ) return;
      }
      Serial << F("in position, next bstate ") << breath_state << endl;
      if (breath_state == BreathHold) {
        Serial << F("breath hold reset") << endl;
        breath->reset();
      }

      state = Running;
    }

    void running() {
      // breathing...
      // We are at target position

      if (timeout()) {
        Serial << F("TIMEOUT stopping") << endl;
        state = Stopping;
        return;
      }
      // pausing
      if ( breath_state == BreathHold ) {
        if (  (*breath)() ) {
          // if pause is over, go to BreathIn (else wait)
          Serial << F("Paused, breathin... ") << millis() << endl;
          breath_state = BreathIn;
        }
        return;
      }

      // change direction & time...
      int direction = 0; // which way
      unsigned int breath_time = breathing_times[breath_state];
      BreathStates next_breath_state;

      if ( breath_state == BreathIn ) {
        direction = 1;
        next_breath_state = BreathOut;
      }
      else if (breath_state == BreathOut ) {
        direction = -1;
        next_breath_state = BreathHold;
      }
      else {
        Serial << F("FAIL: Bad State ") << breath_state << endl;
        while (1) {
          delay(500);
        }
      }

      Serial << F("breath state ") << breath_state
             << F(" br-amp ") << breathing_amplitude
             << F(" dir ") << direction
             << F(" msecs: ") << breath_time
             <<  endl;

      for (int i = 0; i < all_motors->motor_ct; i++) {
        int pos_i = (i <= 6) ? i : ( i == 7 ? 6 : (all_motors->motor_ct - i - 1));
        // commented out limits breathing to the segments below 0, i.e. front/rear
        //float motion_unit = positions[pos_i] > 0 ? 0 : (abs(positions[pos_i]) + MidPointBias);
        float motion_unit = (abs(positions[pos_i]) + MidPointBias);
        // from initial position
        int distance = direction * breathing_amplitude * motion_unit;
        int target_position = positions[pos_i] * amplitude + distance;
        // from actual current position
        int delta = target_position - all_motors->motors[i]->currentPosition();
        int accel = 2 * abs(delta) / 2 / ((breath_time / 1000 * breath_time / 1000) / 4);
        if (accel < 10) accel = 10;
        int speed = 0.5 * accel * breath_time / 1000 / 2;
        if (speed < 10) speed = 10;

        /* Serial << F("## ") << i
               << F(" pos_i ") << pos_i
               //<< F(" <=6 ") << (i <= 6) << ((i <= 6) ? '<' : '>')
               << F(" mu ") << motion_unit
               << F(" d-initial ") << distance
               << F(" d-actual ") << delta
               << F(" target ") << target_position
               << F(" ") << accel << F(" st/sec-sec")
               << F(" ") << int(speed) << F(" st/sec")
               << endl;
        */

        all_motors->motors[i]->setMaxSpeed(speed);
        all_motors->motors[i]->setAcceleration(accel);
        all_motors->motors[i]->moveTo(target_position);
      }

      breath_state = next_breath_state; // in->out->hold

      state = Starting; // means "wait till finished this movement"
    }

    void stopping() {
      Serial << F("STOP") << endl;
      start_homing(2000);
      while (! at_target() ) {}
      state = Idle;
    }
};
// generated from: ./worn-posture-generated
// cos(-pi/2..pi/2) ct=7 mid=0.8 Fri Dec  3 09:25:45 2021
const float AnimationWornPosture::MidPointBias = 0.8;
const float AnimationWornPosture::positions[7] = {-0.800,-0.300, 0.066, 0.200, 0.066,-0.300,-0.800};

class AnimationJitter  : public Animation {
    /* act nervous, jittering around.
    */

  public:
    const float original_amplitude;
    float amplitude = 0; // in steps, decays
    int motor_i; // current motor
    static constexpr float Decay = 0.1; // how much to decay each time
    static constexpr int Moot =  meters_to_steps(0.005); // no point if amp < this
    const float decay; // abs amount to decay
    Every *decay_now; // when to do each decay

    AnimationJitter(AccelStepperShift* all_motors, float amplitude_meters, int duration)
      : Animation(all_motors),
        original_amplitude(meters_to_steps(amplitude_meters)),
        amplitude(original_amplitude),
        motor_i(0),
        decay( (amplitude - Moot) * Decay ) // will be only approximate
    {
      // +1 because the last step is probably moot
      decay_now = new Every( int(duration / (1 + 1 / Decay)) );
    }

    void restart() {
      amplitude = original_amplitude;

      Serial << F("jitter anim amplitude ") << amplitude 
      << F(" at ") << millis() 
      << F(" Moot ") << Moot
      << F(" decay ") << decay
      << endl;
      
      int speed = 200 * 8 * 2; // 2 revolutions/sec
      for (int i = 0; i < all_motors->motor_ct; i++) {
        all_motors->motors[i]->setMaxSpeed(speed);
        all_motors->motors[i]->setAcceleration(speed * 10);
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
        Serial << F("  try motor ") << motor_i << F(" to ") << target << endl;
        target = random(-amplitude, amplitude + 1);
      }
      Serial << F("  motor ") << motor_i << F(" to ") << target << endl;
      return target;
    }

    void running() {
      // eventually stops


      // as a motor finishes, pick a new one
      if ( all_motors->motors[ motor_i ]->distanceToGo() == 0 ) {

        if ( (*decay_now)() ) {
          amplitude -= decay;
          Serial << F("decay ") << decay << F(" to amp ") << amplitude << endl;

          // decrease current posture toward 0

          for (int i = 0; i < all_motors->motor_ct; i++) {
            int position = all_motors->motors[i]->currentPosition();
            int actual_decay = decay;
            if (position > decay ) {
              if (position < decay) actual_decay = position; // min
              Serial << F("dec posture ") << i << F(" by ") << (-actual_decay) << endl;
              all_motors->motors[i]->move( - actual_decay );
            }
            else if (position < decay ) {
              if (position > -decay) actual_decay = -position; // min
              Serial << F("dec posture ") << i << F(" by ") << (actual_decay) << endl;
              all_motors->motors[i]->move( actual_decay );
            }
          }

          if ( amplitude < Moot ) {
            Serial << F("DECAYED! ") << millis() << endl;
            state = Stopping;
          }
        }

        else {
          // jitter one
          motor_i = random( all_motors->motor_ct );
          all_motors->motors[motor_i]->moveTo( jitter(motor_i) );
        }
      }

    }

    void stopping() {
      // we do not go to idle! we hold till told otherwise
    }
};
