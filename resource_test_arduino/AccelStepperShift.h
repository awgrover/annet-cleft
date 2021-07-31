#pragma once

#include <SPI.h>

#include <AccelStepper.h> // Mike McCauley <mikem@airspayce.com>
#include <Streaming.h>

#include "beasties.h"
#include "freememory.h"
#include "every.h"

// the COUNT of motors to debug.
//  undef or -1 is don't,
//#define DEBUGBITVECTOR 1
// 1 to print the bit-vector every time
// 2 to print on "blink"
//#define DEBUGLOGBITVECTOR 2

#ifndef DEBUGBITVECTOR
#define DEBUGBITVECTOR -1
#endif
#ifndef DEBUGLOGBITVECTOR
#define DEBUGLOGBITVECTOR 0
#endif

class AccelStepperNoted: public AccelStepper {
    // This class just notes that a step is requested.
    // A user of the instance should reset `do_step` once it is read
    // Need to call the usual .runX(), and other .setX() as normal

  public:
    inline boolean direction() {
      return _direction;  // expose, true == foward
    }
    boolean do_step; // true == do a step, reset after this is used

    AccelStepperNoted() : AccelStepper(&dumyStep, &dumyStep) , do_step(false) {}
    static void dumyStep() {}

    void step(long astep) {
      // step is just current position, not relevant for 1-pin mode
      // we just note that a step is requested

      (void)(astep); // Unused

      do_step = true;
    }
} ;

class AccelStepperShift {
    // for many DFRobot TB6600
    // via a chain of shift-registers (2 bits per tb6600, 2 bits unused, common enable)
    //  74HC595
    //  Wiring: Arduino   Shift-Register
    //                    VCC 3V
    //                    GND
    //          MOSI      14 SER
    //          SCK       11 SRCLK
    //          latch_pin 12 RCLK
    //                    13 ~OE pull low
    //                    10 ~SRCLR pull high
    //  5MHz clock max at 2v, 25MHz at 5v, so... at 3v...
    //  20mA max per output (?)
    // using an AccelStepper per motor
    //
    // This class
    // * constructs AccelStepperNoted per motor
    // * has run() to run all motors
    // * converts the "noted" steps into bit-vectors for fast spi-shift out
    //    to drive the tb6600's

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
    static byte constexpr FWD_DIRBIT = 0b10;
    static byte constexpr REV_DIRBIT = 0b00;
    static byte constexpr STEPBIT = 0b01;
    static boolean constexpr LATCHSTART = 1; // 1 if the latch pulse is LOW->HIGH->LOW
    static boolean constexpr LATCHIDLE = ! LATCHSTART;

    // We are using 8 bit shift registers
    // And 4 bits per motor (a frame)
    // We shift out LSB 1st,
    // so our bit-vector[0] is the first shifted out (farthest shift-register. lsb)
    // and shift-register[i].bit[7] is our frame-bit[0] ?
    // so,
    //  arduino
    //  -> ...
    //  -> motor[1]:motor[0]
    //  -> extra_frame[1]:extra_frame[0]
    //    led-bar goes here
    //  -> unused_frame[1]:unused_frame[0]... if any
    //  end-of-chain
    static constexpr int bits_per_frame = 4; // dir-,pul-, 2 unused ("en" is common)
    static constexpr int extra_frames = 8 / bits_per_frame; // the led-bar: 8 leds
    // cf unused_frames. use frame_i = unused_frames as index of 1st "extra frame"
    static constexpr int used_bits = 2; // dir-,pul-, 2 unused ("en" is common)
    static constexpr byte frame_mask = (1 << bits_per_frame) - 1; // just the bits of the frame, i.e. n bits
    static constexpr byte used_mask = (1 << used_bits) - 1; // just the bits of the frame that are used, i.e. step & dir

    const int motor_ct;
    const int latch_pin;

    // derived
    // NB: this MUST be in the same order as the initialization list of the constructor
    const int total_bits;
    const int byte_ct;
    const int unused_frames; // lsb frames that aren't used (because we need byte aligned)

    AccelStepperNoted** motors; // an [] of them

    // frame order is lsb:
    // unused-frames, led-bar, motor0 ... motorN
    // an [] of bytes for the shift-register bits, for per-motor-frame
    byte* dir_bit_vector = NULL; // dir & step=0 for each frame
    byte* step_bit_vector = NULL; // dir&step for each frame

    AccelStepperShift(const int motor_ct, const int latch_pin)
      : motor_ct(motor_ct)
      , latch_pin(latch_pin)
      , total_bits( (motor_ct + extra_frames) * bits_per_frame)
        // we have to fill out the bits to make a whole frame, i.e. ceiling()
      , byte_ct( ceil( total_bits / (sizeof(byte) * 8.0) ) )
        // and fill out to byte align
      , unused_frames( byte_ct - (total_bits / (sizeof(byte) * 8)) )
    {
      // not constructing `new` members till .begin()
    }

    void begin() {
      pinMode(latch_pin, OUTPUT);
      digitalWrite(latch_pin, LATCHIDLE);

      // construct now, so we can control when memory is allocated
      Serial << F("SETUP AccelStepperShift ") << motor_ct
             << F(" bit_vector bytes ") << byte_ct << F(" bits ") << total_bits << F(" unused frames ") << unused_frames
             << endl;
      if ( unused_frames != (byte_ct - (total_bits  / ((int)sizeof(byte) * 8) ))) {
        Serial << F("Bad unused_frames again! free ") << freeMemory() << endl;
        while (1) delay(20);
      }
      if (DEBUGBITVECTOR > 0) {
        Serial
            << F("(sizeof(byte) * 8.0) ") << (sizeof(byte) * 8.0) << endl
            << F("byte_ct - (total_bits  / (sizeof(byte) * 8) ") << (byte_ct - (total_bits  / (sizeof(byte) * 8) )) << endl
            ;
        Serial << F("Free ") << freeMemory() << endl;
        //while (1) delay(20);
      }

      motors = new AccelStepperNoted*[motor_ct];
      for (int i = 0; i < motor_ct; i++) {
        motors[i] = new AccelStepperNoted;
      }

      // allocate the bit-vector
      // better be 8 bits/byte!

      step_bit_vector = new byte[byte_ct];
      memset(step_bit_vector, 0, sizeof(byte)*motor_ct);
      dir_bit_vector = new byte[byte_ct];
      memset(dir_bit_vector, 0, sizeof(byte)*motor_ct);

      // construct now, so we can control when memory is allocated
      Serial << F("BEGIN AccelStepperShift: ") << endl;

      for (int i = 0; i < motor_ct; i++) {
        // to 200 steps/sec in 0.1 sec
        // targetspeed = 1/2 * A * 0.1
        // (targetspeed*2)/0.1
        constexpr int max_speed = 200;
        constexpr float time_to_max = 0.1;
        constexpr int acceleration = (max_speed * 2)/time_to_max;
        motors[i]->setAcceleration(acceleration);
        motors[i]->setMaxSpeed(max_speed); // might be limited by maximum loop speed
      }

      // setup SPI. not nice. should be global in main setup()
      SPI.begin();
    }

    boolean run() {

      if ( run_all() ) {
        set_led_bar();
        shift_out();
        return true;
      }
      return false;
    }

    void set_led_bar() {
      static Every::Toggle shift_blink(100);

      // use led-bar to indicate that we are shifting, i.e. running
      byte dir_bit = 0;
      byte step_bit = 0;
      // last bit always on: "running"
      dir_bit |= 1;
      step_bit |= 1;
      // second bit 2/3 on: "dir/step alternating"
      dir_bit |= 1 << 1;
      step_bit |= 0 << 1;
      // third bit 1/3 on: "dir/step alternating"
      dir_bit |= 0 << 2;
      step_bit |= 1 << 2;
      // 4th bit blinking
      boolean do_blink = shift_blink(); // cache
      if (do_blink) {
        dir_bit |= shift_blink.state << 3;
        step_bit |= shift_blink.state << 3;
      }
      set_frame( dir_bit_vector, unused_frames + 0, frame_mask, dir_bit );
      set_frame( step_bit_vector, unused_frames + 0, frame_mask, step_bit );

      if (do_blink && DEBUGLOGBITVECTOR == 2) {
        Serial << F("OUT: ") << endl;
        dump_bit_vectors();
        //while (1) {};
      }
    }

    boolean run_all() {
      // run all
      static boolean all_done = false;

      boolean done = true;
      for (int i = 0; i < motor_ct; i++) {
        // ->run is about 4000 micros for 15 motors @ 8MHz clock
        if ( motors[i]->run() ) {
          done = false;
        }
        else {
          // if (!all_done) Serial << F("  done ") << i << endl; // message as each motor finishes
        }

        if (false && DEBUGBITVECTOR > 0 && DEBUGBITVECTOR <= i && !all_done) {
          Serial
              << F("motor[") << i << F("] do_step? ") << motors[i]->do_step
              << endl;
        }
        // collect the bits
        // about 100micros for all 15 at 8MHz 32u4
        if (motors[i]->do_step) {
          motors[i]->do_step = false; // reset once read

          int frame_i = extra_frames + unused_frames ;
          frame_i += i;
          const byte mask = frame_mask; // just our bits
          if (DEBUGBITVECTOR > 0 && DEBUGBITVECTOR >= i) {
            Serial << F("BV motor[") << i << F("] ")
                   << F(" extra ") << extra_frames << F(" unused ") << unused_frames << F(" = ") << (extra_frames + unused_frames)
                   << F("| frame[") << frame_i << F("] ")
                   << F(" direction ") << motors[i]->direction()
                   << endl;
            Serial << F("  mask 0b") << _BIN(mask) << endl;
            //while (1) delay(20);
          }
          // want the dir-bit, and !step
          const byte dir_bit = ((motors[i]->direction() ? FWD_DIRBIT : REV_DIRBIT) | (~STEPBIT & used_mask));
          // want the dir-bit AND step
          const byte step_bit = ((motors[i]->direction() ? FWD_DIRBIT : REV_DIRBIT) | STEPBIT);
          if (DEBUGBITVECTOR > 0 && DEBUGBITVECTOR >= i) {
            Serial << "  new dir_bits " << _BIN(dir_bit) << endl;
            Serial << "  new step_bits " << _BIN(step_bit) << endl;
            //while (1) delay(20);
          }

          int byte_i = set_frame( dir_bit_vector, frame_i, mask, dir_bit );
          set_frame( step_bit_vector, frame_i, mask, step_bit );

          if (DEBUGBITVECTOR > 0 && DEBUGBITVECTOR >= i) {
            Serial << F("  dir byte[") << byte_i << F("]= ") << _BIN(dir_bit_vector[ byte_i ]) << endl;
            Serial << F("  step byte[") << byte_i << F("]= ") << _BIN(step_bit_vector[ byte_i ]) << endl;
          }

          // stop if debugging this motor
          if (DEBUGLOGBITVECTOR == 1 || (DEBUGBITVECTOR > 0 && DEBUGBITVECTOR == i)) {
            for (int bi = byte_ct - 1; bi >= 0; bi--) {
              Serial << F("      ") << F(" ") << (bi > 10 ? "" : " ") << bi << F(" ");
            }
            Serial << endl;

            dump_bit_vectors();
            if (DEBUGBITVECTOR > 0 && DEBUGBITVECTOR == i) while (1) delay(20);

          }
        }

      }
      if (!all_done && done) Serial << F("All done @ ") << millis() << endl;
      all_done = done;
      return ! all_done;
    }

    void dump_bit_vectors() {
      byte* twobvs[2] = { dir_bit_vector, step_bit_vector };
      for (byte* bv : twobvs) {
        for (int bi = byte_ct - 1; bi >= 0; bi--) {
          for (int bit_i = 0; bit_i < 8; bit_i++) {
            if ( ! (bit_i % bits_per_frame) && bit_i != 0) Serial << ".";
            Serial << ( ( bv[bi] & (1 << (7 - bit_i)) ) ? '1' : '0' );
          }
          Serial << " ";
        }
        Serial << endl;
      }
      Serial << endl;
    }

    void shift_out() {
      // Send bits, nb, dir_bit_vector & step_bit_vector get overwritten
      // each is about 60micros at 4MHz spi
      // at least 2.2micros pulse durations, and each takes 60, so ok
      // SPI.transfer(byte []) overwrites the byte[] buffer,
      // and we want to reuse it here.
      // SO make a copy of dir_bit_vector

      byte dir_copy[byte_ct];
      memcpy( dir_copy, dir_bit_vector, byte_ct * sizeof(byte));

      // use beginTransaction() to be friendly to other spi users (& disable interrupts!)
      SPI.beginTransaction(SPISettings(6000000, LSBFIRST, SPI_MODE0));
       
      SPI.transfer(dir_bit_vector, motor_ct);
      // latch signal needs to be 100ns long, and digitalWrite takes 5micros! so ok.
      digitalWrite(latch_pin, LATCHSTART); digitalWrite(latch_pin, LATCHIDLE);
      SPI.transfer(step_bit_vector, motor_ct);
      digitalWrite(latch_pin, LATCHSTART); digitalWrite(latch_pin, LATCHIDLE);
      SPI.transfer(dir_bit_vector, motor_ct); // this is "step pulse off"
      
      SPI.endTransaction(); // "as soon as possible"
      // last latch
      digitalWrite(latch_pin, LATCHSTART); digitalWrite(latch_pin, LATCHIDLE);
    }

    inline int set_frame( byte * bit_vector, int frame_i, byte mask, byte value ) {
      // In the bit_vector
      // at the [frame_i] (lsb)
      // set the bits in `value`, masked by `mask`
      // returns the byte_i
      const int byte_i = (frame_i * bits_per_frame ) / (sizeof(byte) * 8);
      const int offset = (frame_i * bits_per_frame ) % 8;
      if (DEBUGBITVECTOR > 0) {
        Serial << F("    set [") << byte_i << F("] offset ") << offset << F(" mask 0b") << _BIN(mask << offset) << F(" = ") << _BIN(value << offset) << endl;
      }
      bit_vector[ byte_i ] = ( bit_vector[ byte_i ] & ~(mask << offset) ) | (value << offset);
      return byte_i;
    }
};

#undef DEBUGBITVECTOR
