#pragma once
/*
  We want AccelStepper acceleration, etc., but we are using SPI not pins, and several steppers.

  So, subclass (class AccelStepperNoted) and note that a step is desired.

  And, have a helper (class AccelStepperShift) to collect all the stepper's desires,
  and send them out the SPI to shift registers.

  We are also reading from the shift-registers, as limit switches, so stop if we hit a limit.
*/

#include <SPI.h>

#include <AccelStepper.h> // Mike McCauley <mikem@airspayce.com>
#include <Streaming.h> // Mikal Hart <mikal@arduiniana.org>

#include "freememory.h"
#include "every.h"
#include "begin_run.h"

// true to print "done" everytime all motors finish
// probably too noisy when running a real pattern
#define DEBUGDONERUN 1

// the COUNT of motors to debug, stops after that
//  undef or -1 is don't,
//#define DEBUGBITVECTOR 1
// 1 to print the bit-vector every time
// 2 to print on "blink"
// 3 to print on shift-out
//#define DEBUGLOGBITVECTOR 3
// print position of each motor on each step if true
//#define DEBUGPOSPERSTEP 1
// stop run()'ing after this number of steps, 0 means don't stop
//#define DEBUGSTOPAFTER 2
// print calcalation for frame[i] = value & mask
///#define    DEBUGFRAME 1
// Delay between each shift
//#define DEBUGSTUPIDSLOW 500

#ifndef DEBUGDONERUN
#define DEBUGDONERUN 0
#endif
#ifndef DEBUGBITVECTOR
#define DEBUGBITVECTOR -1
#endif
#ifndef DEBUGLOGBITVECTOR
#define DEBUGLOGBITVECTOR 0
#endif

#ifndef DEBUGPOSPERSTEP
#define DEBUGPOSPERSTEP 0
#endif
#ifndef DEBUGSTOPAFTER
#define DEBUGSTOPAFTER 0
#endif
#ifndef DEBUGFRAME
#define DEBUGFRAME 0
#endif
#ifndef DEBUGSTUPIDSLOW
#define DEBUGSTUPIDSLOW 0
#endif

class AccelStepperNoted: public AccelStepper {
    // This class just notes that a step is requested.
    // A user of the instance should reset `do_step` once it is read.
    // Need to call the usual .runX(), and other .setX() as normal.
    // NB: the last step() will have .run() return false, so use do_step():
    //  amotor.run();
    //  if (amotor.do_step) { handle it, set do_step=false ...}

  public:
    inline boolean direction() {
      return _direction;  // expose, true == foward
    }

    const int motor_i; // my index

    boolean do_step; // true == do a step, reset after this is used
    boolean at_limit = false; // triggered by limit_switch detection during home-to-up-limit
    int at_limit_pos = 0; // where the limit switch was last triggered (we can decel past this).

    // this super's constructor causes no use of pins
    // "enable" is common to all motors at once.
    AccelStepperNoted(int i) : AccelStepper(&dumyStep, &dumyStep) , motor_i(i), do_step(false) {}
    static void dumyStep() {}

    void step(long astep) {
      // step is just current position
      // we just note that a step is requested
      (void)(astep); // Unused

      if (DEBUGPOSPERSTEP) Serial << F("<P ") << motor_i << F(" ") << currentPosition() << endl;
      do_step = true;
    }

    int time_to(int position, float acceleration ) { // OMG private var again
      // how many microseconds
      // just an estimate, not good for short distances
      int distance = abs(position - currentPosition());
      int ms = int( 1000.0 * distance / maxSpeed() );
      //Serial << F("time_to ") << position << F(" from ") << currentPosition() << F(" dist ") << distance << F(" ms ") << ms << endl;
      return ms ;

      // d= 1/2 a t^2
      // 2d = a t^2
      // 2d/a = t^2
      // sqrt(2d/a) = t
      // BUT, we want to to accel half-way, then decl, so the `distance` is 1/2, and the time is *2
      //int ms =  int( 2 * sqrt( distance / acceleration ) * 1000 ); // sec to msec
      //Serial << F("time_to ") << position << F(" from ") << currentPosition() << F(" dist ") << distance << F(" @ accel ") << acceleration << F(" ms ") << ms << endl;
      //return ms ;
    }
};

class AccelStepperShift : public BeginRun {
    // for many DFRobot TB6600
    //    input labeling seems confusing, all "+" to +,
    //      pul- steps on transition to high.
    //      dir- : high is forward, low is backwards (clock/counter-clock).
    // via a chain of shift-registers (2 bits per tb6600, common enable)
    //  74HC595 is serial-to-parallel: output on latch going high
    //  74HC165 is parallel-to-serial: input on latch going low (before read)
    //    nb: opposite sense of latch from the 595
    //    we are doing MSBFIRST, so 165's bits are reversed: A is [7]
    //  Wiring: Arduino   Shift-Register 74HC595  tb6600          Shift-register 74HC165
    //          3.3V      16 VCC 3V               dir+,pul+,en+   16 Vcc
    //          GND       8 GND                                   8 GND
    //          MOSI      14 SER (shiftin)
    //          MISO                                              9 Qh (shiftout) (7 is ~Qh)
    //          SCK       11 SRCLK                                2 clk (15 & 2 can be swapped)
    //          latch_pin 12 RCLK
    //          load-pin                                          1 SH/~LD (latch - read on low, high for shift)
    //                    13 ~OE pull low
    //                    10 ~SRCLR pull high
    //                    9 q7'/data-out -> SER
    //                    15 q0                   motor0 dir-
    //                    1 q1                    motor0 pul- (step)
    //                    q2/q3                   motor1, ...
    //                                                            11 A limit1
    //                                                            13 C limit2, 3 E limit 3, 5 G limit 4
    //                                                            11,12,13,14, 3,4,5,6 Do not let them float. pull low if not used
    //                                                            15 clk-inh, pull low
    //                                                            10 SER shiftin daisy chain to 9 Qh, pull low on last
    //          GND
    //          motor-enable                      en- (HIGH for engage, low for free), pull low
    //
    //  5MHz clock max at 2v, 25MHz at 5v, so... at 3v...
    //  20mA max per output (?)
    //
    // To test for acceleration
    //    Set the driver to a very small current so the motor will slip (maybe 0.5A).
    //    Test various accelerations till no slip
    //    Set the driver back to operating current
    //
    // Indicators
    //    SCK is too fast to see (only on for about 60micros!)
    //    mosi gets left on, so not useful
    //    data doesn't even seem to change!
    // using an AccelStepper per motor
    //
    // This class
    // * constructs AccelStepperNoted per motor
    // * has run() to run all motors
    // * converts the "noted" steps into bit-vectors for fast spi-shift out
    //    to drive the tb6600's
    // * reads the limit switches

    // driver chip: https://toshiba.semicon-storage.com/info/docget.jsp?did=14683&prodName=TB6600HG
    //"common anode"
    // pull up to logic level: PUL+, DIR+ and EN+
    // 8-15mA each signal
    // signals: pul-, dir-, en-
    // (swap those sets for "common cathode" and reverse signal polarities)
    // That's "1 pin" mode for AccelStepper
    // min 2.2micros for pulse width
    // Order:
    //  set direction & enable ~20micros
    //    also serves as >2.2micros between pulses
    //  set pulse ~20micros
    //  2.2micros
    //  reset pulse ~20micros
    //  total ~60microsec

  public:
    // per frame
    // invert if necessary
    static byte constexpr FWD_DIRBIT = 1 << 0;
    static byte constexpr REV_DIRBIT = 0 << 0;
    static byte constexpr STEPBIT = 1 << 1;
    static byte constexpr NOT_STEPBIT = 0 << 1;
    static boolean constexpr LATCHSTART = HIGH; // 1 if the latch pulse is LOW->HIGH->LOW
    static boolean constexpr LATCHIDLE = ! LATCHSTART;
    static boolean constexpr SH_LD_IDLE = HIGH; // allow shift while high, load on low
    static boolean constexpr SH_LD_LOAD = !SH_LD_IDLE; // allow shift while high, load on low
    static boolean constexpr ENABLE = HIGH;
    static boolean constexpr DISABLE = !ENABLE;

    // depends on how far the CRASH_LIMIT steps actually is
    static int constexpr CRASH_LIMIT = 200; // steps past limit switch, absolute limit
    static float constexpr STEPS_METER = 7.2 * 200; // fixme: measure
    static float constexpr MAX_MOTION = 0.1; // meters fixme: measure
    static int constexpr MAX_SPEED = 200; // 1/rev sec, ...

    // We are using 8 bit shift registers
    // And 2 bits per motor (a "frame"): dir & step
    // We shift out MSB 1st, which works for the output shift-register
    //  but is backwards for the input shift-register.
    // so our bit-vector[0] is the first shifted out (farthest shift-register)
    //  arduino
    //  -> ...
    //  -> motor[1]:motor[0]
    //  -> led-bar goes here "extra frame"
    //  -> unused_frame[1]:unused_frame[0]... if any
    //  end-of-chain
    static constexpr int bits_per_frame = 2; // dir-,pul-, ("en" is common)
    static constexpr int extra_frames = 8 / bits_per_frame; // the led-bar: 1 whole shift-register
    // cf unused_frames. use frame_i = unused_frames as index of 1st "extra frame"
    static constexpr int used_bits = 2; // dir-,pul- (unused per frame is possible) ("en" is common)
    static constexpr byte frame_mask = (1 << bits_per_frame) - 1; // just the bits of the frame, i.e. n bits
    static constexpr byte used_mask = (1 << used_bits) - 1; // just the bits of the frame that are used, i.e. step & dir

    // State
    // NB: this MUST be in the same order as the initialization list of the constructor
    // or values can get corrupted.
    // Turn on "more" compiler warnings in preferences.

    const int motor_ct;
    const int latch_pin;
    const int shift_load_pin; // for parallel-to-serial
    const int enable_pin; // common enable. AccelStepper has _enablePin but private, and enable/disable does too much
    const int fake_limit_pin;
    const int fake_limit_pin_x;
    // derived
    const int total_used_bits;
    const int byte_ct;
    const int unused_frames; // lsb frames that aren't used (because we need byte aligned)
    const int total_frames;

    AccelStepperNoted** motors; // an [motor_ct] of them

    // for blinking indicators, i.e. leaves led on for at least this time
    Timer recent_step = Timer(200, false);
    Timer recent_limit = Timer(200, false);

    // frame order is lsb:
    // unused-frames, led-bar, motor0 ... motorN
    // an [] of bytes for the shift-register bits, for per-motor-frame
    byte* dir_bit_vector = NULL; // dir & step=0 for each frame
    byte* step_bit_vector = NULL; // dir&step for each frame
    byte* limit_switch_bit_vector = NULL; // read via limit_switch(i)

    AccelStepperShift(const int motor_ct, const int latch_pin, const int shift_load_pin, const int enable_pin, const int fake_limit_pin, const int fake_limit_pin_x)
      : motor_ct(motor_ct)
      , latch_pin(latch_pin)
      , shift_load_pin(shift_load_pin)
      , enable_pin(enable_pin)
      , fake_limit_pin(fake_limit_pin)
      , fake_limit_pin_x(fake_limit_pin_x)
        // pre-calculating a bunch of stuff
      , total_used_bits( (motor_ct + extra_frames) * bits_per_frame)
        // we have to fill out the bits to make a whole frame, i.e. ceiling()
      , byte_ct( ceil( total_used_bits / (sizeof(byte) * 8.0) ) ) // i.e. need a whole byte for a fraction
        // and fill out to byte align
      , unused_frames( byte_ct - (total_used_bits / (sizeof(byte) * 8)) )
      , total_frames( byte_ct * ( (sizeof(byte) * 8 / bits_per_frame) ) )
    {
      // not constructing `new` members till .begin()
    }

    void begin() {
      // our 2 "latches"
      pinMode(latch_pin, OUTPUT);
      digitalWrite(latch_pin, LATCHIDLE);

      pinMode(shift_load_pin, OUTPUT);
      digitalWrite(shift_load_pin, SH_LD_IDLE);

      // common enable
      pinMode(enable_pin, OUTPUT);

      // development jumper
      pinMode(fake_limit_pin, INPUT_PULLUP);
      pinMode(fake_limit_pin_x, OUTPUT);

      enable();

      // construct now, so we can control when memory is allocated
      Serial << F("BEGIN AccelStepperShift ") << motor_ct
             << F(" bit_vector bytes ") << byte_ct << F(" motor ct ") << motor_ct
             << F(" extra frames ") << extra_frames
             << F(" unused frames ") << unused_frames
             << F(" used bits ") << total_used_bits
             << F(" total frames ") << total_frames
             << endl;
      // I was getting corrupted data because initialization-list must be in same order as instance-vars.
      if ( unused_frames != (byte_ct - (total_used_bits  / ((int)sizeof(byte) * 8) ))) {
        Serial << F("Bad unused_frames again! free ") << freeMemory() << endl;
        while (1) delay(20);
      }
      if (DEBUGBITVECTOR > 0) {
        Serial
            << F("(sizeof(byte) * 8.0) ") << (sizeof(byte) * 8.0) << endl
            << F("byte_ct - (total_used_bits  / (sizeof(byte) * 8) ") << (byte_ct - (total_used_bits  / (sizeof(byte) * 8) )) << endl
            ;
        Serial << F("Free ") << freeMemory() << endl;
        //while (1) delay(20);
      }
      Serial << F("DEBUGBITVECTOR ") << DEBUGBITVECTOR
             << F(" DEBUGLOGBITVECTOR ")  << DEBUGLOGBITVECTOR
             << F(" DEBUGPOSPERSTEP ")  << DEBUGPOSPERSTEP
             << F(" DEBUGSTOPAFTER ")  << DEBUGSTOPAFTER
             << F(" DEBUGFRAME ")  << DEBUGFRAME
             << F(" DEBUGSTUPIDSLOW ")  << DEBUGSTUPIDSLOW
             << endl;

      motors = new AccelStepperNoted*[motor_ct];
      for (int i = 0; i < motor_ct; i++) {
        motors[i] = new AccelStepperNoted(i);
      }

      // allocate the bit-vector
      // better be 8 bits/byte! Yes, there are cpu's with other ideas.

      step_bit_vector = new byte[byte_ct];
      memset(step_bit_vector, 0, sizeof(byte)*motor_ct);
      dir_bit_vector = new byte[byte_ct];
      memset(dir_bit_vector, 0, sizeof(byte)*motor_ct);
      limit_switch_bit_vector = new byte[byte_ct];
      memset(limit_switch_bit_vector, 0, sizeof(byte)*motor_ct);

      // default speed/accel
      for (int i = 0; i < motor_ct; i++) {
        // to 200 steps/sec in 0.1 sec
        // targetspeed = 1/2 * A * 0.1
        // (targetspeed*2)/0.1
        constexpr float time_to_max = 0.25;
        constexpr int acceleration = (MAX_SPEED * 2) / time_to_max;
        motors[i]->setAcceleration(acceleration);
        motors[i]->setMaxSpeed(MAX_SPEED); // might be limited by maximum loop speed
      }
    }

    boolean run() {
      if ( run_all() ) {
        set_led_bar();
        shift_out();
        return true;
      }
      else if ( recent_step() || recent_limit() ) {
        set_led_bar(true);
        shift_out();
      }
      return false;
    }

    void finish_loop() {
      // reset do_step's
      for (int i = 0; i < motor_ct; i++) {
        motors[i]->do_step = false;
      }
    }

    void set_led_bar(boolean clear = false) {
      // use led-bar for various indications
      // true clears some bits

      static Every::Toggle spi_heartbeat(200); // "blink" one of the leds NB: class level!

      // shift register bits, we numbered the led from 9...0 in the comment
      static constexpr int heartbeat_bit = 7;
      static constexpr int recent_step_bit = 1;
      static constexpr int recent_limit_bit = 2;
      bool did_heartbeat = false;

      byte led_bits = 0;

      // SPI heartbeat
      did_heartbeat = spi_heartbeat(); // have to call () to cause .state to change
      led_bits |= (clear ? 0 : spi_heartbeat.state ) << heartbeat_bit;

      // Recent step or limit?

      if ( recent_step.running ) {
        led_bits |= (clear ? 0 : 1) << recent_step_bit;
      }

      if ( recent_limit.running ) {
        led_bits |= (clear ? 0 : 1) << recent_limit_bit;
      }

      // last frame: repeat because we send both bit-vectors
      dir_bit_vector[ 0 ] = led_bits;
      step_bit_vector[ 0 ] = led_bits;

      if ( did_heartbeat ) {
        if (DEBUGLOGBITVECTOR == 2) {
          Serial << F("OUT: ") << spi_heartbeat.state << F(" ") << _BIN(led_bits) << endl;
          dump_bit_vectors();
          //while (1) {};
        }
      }
    }

    inline void stop_at_limit(int motor_i) {
      // limit switches are optional (when developing),
      // this adds about 50micros to the loop

      // only while moving up
      // (we can always go past a limit on the way down)
      const boolean hit_limit = motors[motor_i]->direction() && limit_switch(motor_i);

      if (hit_limit) {
        
        recent_limit.reset(); // indicator duration

        if ( ! motors[motor_i]->at_limit) { // and not already at limit

          // flag that we are atlimit or above
          // remember where we saw the switch
          motors[motor_i]->at_limit = true;
          motors[motor_i]->at_limit_pos = motors[motor_i]->currentPosition();
        }
        // and keep stopping
        motors[motor_i]->stop(); // at current accel
      }
    }

    boolean run_all() {
      // run all
      // capture bit vector
      // return true if ANY ran (have to shift all bits)
      // We should be overwriting the whole bit-vector buffer(s), all bits

      static boolean all_done = false; // for debug messages FIXME
      static int step_count = 0; // for debug, see DEBUGSTOPAFTER

      static Every say_pos(100);
      boolean say = say_pos();
      // if (say) Serial << F("  running @ ") << millis() << F(" ") << endl;

      if (DEBUGSTOPAFTER && step_count >= DEBUGSTOPAFTER) return false;

      //Serial << F("run all, already all_done? ") << all_done << endl;

      boolean done = true;
      for (int i = 0; i < motor_ct; i++) {
        stop_at_limit(i);

        // ->run is about 4000 micros for 15 motors @ 8MHz clock
        // got about 1800 micros @ 48mhz (so about 550 steps/sec max)
        // NB. AccelStepper thinks the protocol is:
        //    step(), if that was last step then ->run() will return false
        // Which means we will not see run()==true for the last step. thus do_step() instead:
        if ( motors[i]->run() || motors[i]->do_step ) {
          done = false;
          if (say && !DEBUGPOSPERSTEP) Serial << F("<P ") << i << F(" ") << motors[i]->currentPosition() << F(" ") << millis() << endl;
        }
        else {
          // if (!all_done) Serial << F("  done ") << i << F(" to ") << motors[i]->currentPosition() << endl; // message as each motor finishes
        }

        //Serial << F("  motors at i done? ") << i << F(" ") << done << endl;

        if (false && DEBUGBITVECTOR > 0 && DEBUGBITVECTOR <= i && !all_done) {
          Serial
              << F("motor[") << i << F("] do_step? ") << motors[i]->do_step
              << endl;
        }

        // add the step bit to our vector (might be 0 : no movement)
        collect_bit(i);

      }

      if (!done) step_count++;

      if (DEBUGDONERUN && (!all_done && done)) Serial << F("All done @ ") << millis() << F(" steps ") << step_count << endl;

      all_done = done;
      //if (!all_done) Serial << F("Done ? ") << all_done << endl;

      if (! done) recent_step.reset(); // restart indicator

      return ! done;
    }

    void enable() {
      // we have a common enable pin
      digitalWrite(enable_pin, ENABLE);
    }

    void disable() {
      digitalWrite(enable_pin, DISABLE);
    }

    void collect_bit(int i) { // motor_i
      // collect the bits for "dir" and "step" bit_vectors
      // about 100micros for all 15 at 8MHz 32u4

      int frame_i = i;

      if (motors[i]->do_step) {

        // the do_step flag is reset at finish_loop(), so we only handle a step once (till next step)

        if (DEBUGBITVECTOR > 0 && DEBUGBITVECTOR >= i) {
          Serial << F("BV motor[") << i << F("] ")
                 << F(" extra ") << extra_frames << F(" unused ") << unused_frames << F(" = ") << (extra_frames + unused_frames)
                 << F("| frame[") << frame_i << F("] ")
                 << F(" direction ") << motors[i]->direction()
                 << endl;
          Serial << F("  mask 0b") << _BIN(frame_mask) << endl;
          //while (1) delay(20);
        }

        // ensure we don't go to far after limit
        const boolean hit_limit_going_up = motors[i]->direction() && motors[i]->at_limit;

        // This is instant stop.
        // Forces the step bit to never go high
        const boolean allow_step = ! (
                                     hit_limit_going_up
                                     && (motors[i]->currentPosition() - motors[i]->at_limit_pos) >= CRASH_LIMIT
                                   );
        if (! allow_step) Serial << F("CRASH motor ") << i << endl;

        // want the dir-bit, and step=0
        const byte dir_bit = ((motors[i]->direction() ? FWD_DIRBIT : REV_DIRBIT) | NOT_STEPBIT);

        // want the dir-bit AND step (unless limit-switch while going up)
        const byte step_bit = dir_bit | (allow_step ? STEPBIT : NOT_STEPBIT);

        if (DEBUGBITVECTOR > 0 && DEBUGBITVECTOR >= i) {
          Serial << "  new dir_bits " << _BIN(dir_bit) << endl;
          Serial << "  new step_bits " << _BIN(step_bit) << endl;
          //while (1) delay(20);
        }

        int byte_i = set_frame( dir_bit_vector, frame_i, frame_mask, dir_bit );
        set_frame( step_bit_vector, frame_i, frame_mask, step_bit );

        if (DEBUGBITVECTOR > 0 && DEBUGBITVECTOR >= i) {
          Serial << F("  dir byte[") << byte_i << F("]= ") << _BIN(dir_bit_vector[ byte_i ]) << endl;
          Serial << F("  step byte[") << byte_i << F("]= ") << _BIN(step_bit_vector[ byte_i ]) << endl;
        }

        // dump, then stop if debugging this motor
        if (DEBUGLOGBITVECTOR == 1 || (DEBUGBITVECTOR > 0 && DEBUGBITVECTOR == i)) {
          for (int bi = byte_ct - 1; bi >= 0; bi--) {
            Serial << F("      ") << F(" ") << (bi > 10 ? "" : " ") << bi << F(" ");
          }
          Serial << endl;

          dump_bit_vectors();
          if (DEBUGBITVECTOR > 0 && DEBUGBITVECTOR == i) while (1) delay(20);
        }
      }
      else {
        // no step(), so mark/clear as such
        // (dir_bit won't matter)
        set_frame( step_bit_vector, frame_i, frame_mask, NOT_STEPBIT );
      }
    }

    void dump_bit_vectors() {
      // just for debug print
      byte* twobvs[2] = { dir_bit_vector, step_bit_vector };
      for (byte* bv : twobvs) {
        dump_bit_vector(bv);
      }
      Serial << endl;
    }

    void dump_bit_vector(byte *bit_vector) {
      // NB; [8] is shifted out last, so is shift-register nearest ard
      // We reorder here so it reads right-to-left
      for (int bi = 0; bi < byte_ct; bi++) {
        for (int bit_i = 0; bit_i < 8; bit_i++) {
          if ( ! (bit_i % bits_per_frame) && bit_i != 0) Serial << ".";
          Serial << ( ( bit_vector[bi] & (1 << (7 - bit_i)) ) ? '1' : '0' );
        }
        Serial << " ";
      }
      Serial << endl;
    }

    void shift_out() {
      // Send all bits,
      // each is about 60micros at 4MHz spi
      // at least 2.2micros pulse durations, and each takes 60, so ok

      byte dir_copy[byte_ct];
      unsigned long start = millis();
      if (DEBUGLOGBITVECTOR == 3) Serial << F("shift out ") << DEBUGSTUPIDSLOW << F(" | ") << (DEBUGSTUPIDSLOW ? "T" : "F") << endl;

      // SPI.transfer(byte []) overwrites the byte[] buffer,
      // and we need to preserve it
      // SO make a copy of bit_vectors
      memcpy( dir_copy, dir_bit_vector, byte_ct * sizeof(byte));
      // out is the step_bit_vector, but we capture the input
      memcpy( limit_switch_bit_vector, step_bit_vector, byte_ct * sizeof(byte));

      // read input pins, allow shift-in (5ns? 1000ns? for low to read)
      digitalWrite(shift_load_pin, LOW); delayMicroseconds(1); digitalWrite(shift_load_pin, HIGH); delayMicroseconds(1);

      // use beginTransaction() to be friendly to other spi users (& disable interrupts!)
      // MSBFIRST affects read and write
      SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

      // Each transfer reads the input shift-register too, but we capture on the 2nd one

      if (DEBUGLOGBITVECTOR == 3) dump_bit_vector(dir_copy);
      SPI.transfer(dir_copy, byte_ct);
      // latch signal needs to be 100ns long, and digitalWrite takes 5micros! so ok.
      digitalWrite(latch_pin, LATCHSTART); digitalWrite(latch_pin, LATCHIDLE);
      if (DEBUGSTUPIDSLOW) delay(DEBUGSTUPIDSLOW);

      // Out (limit_switch_bit_vector) is a copy of step_bit_vector
      // Here's where we capture the limit-switches, back into limit_switch_bit_vector.
      if (DEBUGLOGBITVECTOR == 3) dump_bit_vector(limit_switch_bit_vector);
      SPI.transfer(limit_switch_bit_vector, byte_ct);
      digitalWrite(latch_pin, LATCHSTART); digitalWrite(latch_pin, LATCHIDLE);
      if (DEBUGLOGBITVECTOR == 3) {
        Serial << F("Limit:") << endl;
        dump_bit_vector(limit_switch_bit_vector);
      }
      if (DEBUGSTUPIDSLOW) delay(DEBUGSTUPIDSLOW);

      // again, because the stop bit is always 0 in this bit-vector, i.e. finish the step pulse.
      memcpy( dir_copy, dir_bit_vector, byte_ct * sizeof(byte));
      if (DEBUGLOGBITVECTOR == 3) dump_bit_vector(dir_copy);
      SPI.transfer(dir_copy, byte_ct); // this is "step pulse off"

      SPI.endTransaction(); // "as soon as possible"
      // last latch
      digitalWrite(latch_pin, LATCHSTART); digitalWrite(latch_pin, LATCHIDLE);

      if (DEBUGSTUPIDSLOW) {
        Serial << F("--- ") << (millis() - start) << endl;
        delay(DEBUGSTUPIDSLOW);
      }
    }

    inline int set_frame( byte * bit_vector, int frame_i, byte mask, byte value ) {
      // In the bit_vector
      // at the [frame_i] (lsb)
      // update the bits to `value`, masked by `mask`
      // returns the byte_i
      // byte[0] will be the furthest shift register [n]
      // and we are doing msbfirst, so bit[0] is the nearest bit

      // If we were 0=[0]:
      const int byte_i = (frame_i * bits_per_frame ) / (sizeof(byte) * 8);
      // But, we need 0=[n] byte, low-nybble
      const int r_byte_i = (byte_ct - 1) - byte_i;
      const int frames_per_byte = (sizeof(byte) * 8) / bits_per_frame; // better be multiple!
      const int offset = (frame_i % frames_per_byte ) * bits_per_frame; // frame0=0, frame1=2, frame2=3, etc
      if (DEBUGFRAME > 0) {
        Serial << F("    set frame ") << frame_i << F(" = 0b") << _BIN(value) << F(" mask ") << _BIN(mask)
               << F(" byte[") << r_byte_i << F("]")
               << F(" frameoffset ") << (frame_i % frames_per_byte ) << F(" <<offset ") << offset
               << F(" == mask 0b") << _BIN(mask << offset) << F(" = ") << _BIN(value << offset)
               << endl;
      }
      bit_vector[ r_byte_i ] = ( bit_vector[ r_byte_i ] & ~(mask << offset) ) | (value << offset);
      return r_byte_i;
    }

    boolean limit_switch(int motor_i) {
      // get value of a limit switch
      // which was read by the spi-transfer
      // spi.transfer is MSBFIRST, and reads into byte[0] first
      // so byte[0].bit[0] is actual shift-register[0].bit[7]
      // and we are using the "first" bit of the shift-register as the first switch
      const int byte_i = (motor_i * bits_per_frame ) / (sizeof(byte) * 8);
      const int frames_per_byte = (sizeof(byte) * 8) / bits_per_frame; // better be multiple!
      const int offset = (motor_i % frames_per_byte ) * bits_per_frame; // frame0=0, frame1=2, frame2=3, etc
      const int r_offset = (sizeof(byte) * 8 - 1) - offset; // frame0=4, 3, 2 ..

      // open switch == pullup, so 0 is "limit hit"
      return ! ( limit_switch_bit_vector[ byte_i ] & (1 << r_offset) );
    }

    void goto_limit() {
      // move all motors to the limit switch
      // skip this if no limit switches
      boolean doing_fake_limit = false; // fake all limit switch if jumper on FAKE_LIMIT_PIN

      Serial << F("Goto LIMITUP") << endl;

      constexpr float our_acceleration = 400.0f;
      constexpr int distance_drop = - (CRASH_LIMIT + CRASH_LIMIT / 2);
      static_assert(distance_drop < 2 * MAX_MOTION * STEPS_METER, "Drop before uplimit was > MAX_MOTION");
      Timer too_long(1);

      heartbeat( NEO_STATE_UPLIMIT );

      // move well below the limit switch
      for (int i = 0; i < motor_ct; i++) {
        motors[i]->setMaxSpeed( 100 ); // slow
        motors[i]->setAcceleration( our_acceleration );
        motors[i]->move( distance_drop );
      }
      too_long.reset( int( motors[0]->time_to( distance_drop, our_acceleration) * 1.5) );
      while (run() && ! too_long()) {
        heartbeat( NEO_STATE_UPLIMIT );
        finish_loop(); // resets do_step() so we can detect all-done
      }
      if (too_long.after()) {
        Serial << F("FAULT: too long to run ") << distance_drop << endl;
        fault();
      }
      Serial << F("Drop below limit: done") << endl;

      if ( fake_limit_pin && ! digitalRead(fake_limit_pin) ) {
        doing_fake_limit = true;
        Serial << F("FIXME NOT LIMITING") << endl;
      }

      // move up till limit switch
      constexpr int distance_to_limit = 2 * MAX_MOTION * STEPS_METER;
      for (int i = 0; i < motor_ct; i++) {
        motors[i]->move( distance_to_limit ); // 24 revs should be about 2 meters
      }
      too_long.reset( int( motors[0]->time_to( distance_to_limit, our_acceleration) * 1.5) ); // 2.5 secs / rev
      Timer fake_limit( int( motors[0]->time_to( distance_to_limit, our_acceleration) ) );
      while (run() && ! too_long()) {
        heartbeat( NEO_STATE_UPLIMIT );
        finish_loop(); // resets do_step() so we can detect all-done

        if (doing_fake_limit && fake_limit() ) break;
      }
      if (too_long.after()) {
        Serial << F("FAULT: too long to run ") << distance_to_limit << endl;
        fault();
      }
      Serial << F("All limits hit: done") << endl;

      boolean hit_limit = true;
      for (int i = 0; i < motor_ct; i++) {
        hit_limit &= doing_fake_limit || motors[i]->at_limit;
      }
      if (!hit_limit) {
        Serial << F("FAULT: didn't hit limit ") << distance_to_limit << endl;
        fault();
      }

      // all motors at LIMITUP
      // reset to 0 and cleanup
      constexpr int distance_to_zero = (10 + MAX_MOTION * STEPS_METER);
      for (int i = 0; i < motor_ct; i++) {
        long stopped_at = motors[i]->currentPosition(); // we may have gone past a bit
        motors[i]->setCurrentPosition( distance_to_zero + stopped_at - motors[i]->at_limit_pos );
        motors[i]->at_limit_pos = distance_to_zero; // nobody is relying on this going to 0
        motors[i]->moveTo( 0 ); // limit swith is ~ MAX_MOTION * STEPS_METER
      }
      too_long.reset(  int( motors[0]->time_to( 0, our_acceleration) * 1.5)  );
      while (run() && ! too_long()) {
        heartbeat( NEO_STATE_UPLIMIT );
        finish_loop(); // resets do_step() so we can detect all-done
      }
      if (too_long.after()) {
        Serial << F("FAULT: too long to runto ") << 0 << endl;
        while (1);
      }
      Serial << F("Move to HOME: done") << endl;
    }

    void fault() {
      disable();
      builtin_neo.setPixelColor(0, NEO_STATE_FAULT );
      builtin_neo.show();
      while (1); // and lock up
    }
};

#undef DEBUGBITVECTOR
