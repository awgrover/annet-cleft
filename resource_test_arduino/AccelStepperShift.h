#pragma once

#include <SPI.h>

#include <AccelStepper.h> // Mike McCauley <mikem@airspayce.com>
#include <Streaming.h>

#include "beasties.h"
#include "freememory.h"

// the COUNT of motors to debug. undef or -1 is don't
// #define DEBUGBITVECTOR 14

#ifndef DEBUGBITVECTOR
#define DEBUGBITVECTOR -1
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

class AccelStepperShift : public Beastie {
    // for many DFRobot TB6600
    // via a chain of shift-registers (2 bits per tb6600, common enable)
    // using an AccelStepper per
    // This class
    // * constructs AccelStepperNoted per motor
    // * has several of the runX() methods to run all motors
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
    //  total ~60micros-sec

  public:
    // per frame
    static byte constexpr FWD_DIRBIT = 0b10;
    static byte constexpr REV_DIRBIT = 0b00;
    static byte constexpr STEPBIT = 0b01;

    static constexpr int bits_per_frame = 4; // dir-,pul-, 2 unused ("en" is common)
    static constexpr int extra_frame = 8 / bits_per_frame; // the led-bar: 8 leds
    static constexpr int used_bits = 2; // dir-,pul-, 2 unused ("en" is common)
    static constexpr byte frame_mask = (1 << bits_per_frame) - 1; // just the bits of the frame, i.e. n bits
    static constexpr byte used_mask = (1 << used_bits) - 1; // just the bits of the frame that are used, i.e. step & dir

    const int motor_ct;
    const int total_bits;
    // we have to fill out the bits to make a whole frame, i.e. ceiling()
    const int unused_frames;
    const int byte_ct;

    AccelStepperNoted** motors; // an [] of them

    // frame order is lsb:
    // unused-frames, led-bar, motor0 ... motorN
    // an [] of bytes for the shift-register bits, for per-motor-frame
    byte* dir_bit_vector; // dir & step=0 for each frame
    byte* step_bit_vector; // dir&step for each frame

    AccelStepperShift(int motor_ct)
      : dir_bit_vector(NULL) // I saw `unused_frames` getting corrupted if I didn't do this
      , step_bit_vector(NULL)
      , motor_ct(motor_ct)
      , total_bits( (motor_ct + extra_frame) * bits_per_frame)
      , byte_ct( ceil( total_bits / (sizeof(byte) * 8.0) ) )
      , unused_frames( byte_ct - (total_bits / (sizeof(byte) * 8)) )
    {
      // not constructing members till .begin()
    }

    void setup() {
      // construct now, so we can control when memory is allocated
      if (DEBUGBITVECTOR >= 0) {
        Serial << F("SETUP AccelStepperShift ") << motor_ct
               << F(" bit_vector bytes ") << byte_ct << F(" bits ") << total_bits << F(" unused frames ") << unused_frames
               << endl;
        Serial
            << F("(sizeof(byte) * 8.0) ") << (sizeof(byte) * 8.0) << endl
            << F("byte_ct - (total_bits  / (sizeof(byte) * 8) ") << (byte_ct - (total_bits  / (sizeof(byte) * 8) )) << endl
            ;
        Serial << F("Free ") << freeMemory() << endl;
        //while(1) delay(20);
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
    }

    void begin() {
      // construct now, so we can control when memory is allocated
      Serial << F("BEGIN AccelStepperShift: ") << endl;

      for (int i = 0; i < motor_ct; i++) {
        motors[i]->setAcceleration(1000);
        motors[i]->setMaxSpeed(200); // really, 1/~60micros
        motors[i]->moveTo(300);
      }

      // setup SPI
      SPI.begin();
      SPI.setClockDivider(SPI_CLOCK_DIV8); // change to <10MHz
      SPI.setBitOrder(LSBFIRST);
      SPI.setDataMode(SPI_MODE0);
    }

    void loop() {
      run();
    }

    void run() {
      // run all
      static boolean all_done = false;

      int done_ct = 0;
      for (int i = 0; i < motor_ct; i++) {
        // ->run is about 4000 micros for 15 motors @ 8MHz clock
        if ( ! motors[i]->run() ) {
          if (!all_done) Serial << F("  done ") << i << endl;
          done_ct ++;
        }

        if (false && DEBUGBITVECTOR >= 0 && DEBUGBITVECTOR <= i && !all_done) {
          Serial
              << F("motor[") << i << F("] do_step? ") << motors[i]->do_step
              << endl;
        }
        // collect the bits
        // about 100micros for all 15 at 8MHz 32u4
        if (motors[i]->do_step) {
          motors[i]->do_step = false; // reset once read

          int frame_i = extra_frame + unused_frames ;
          frame_i += i;
          const byte mask = frame_mask; // just our bits
          if (DEBUGBITVECTOR >= 0 && DEBUGBITVECTOR >= i) {
            Serial << F("BV motor[") << i << F("] ")
                   << F(" extra ") << extra_frame << F(" unused ") << unused_frames << F(" = ") << (extra_frame + unused_frames)
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
          if (DEBUGBITVECTOR >= 0 && DEBUGBITVECTOR >= i) {
            Serial << "  new dir_bits " << _BIN(dir_bit) << endl;
            Serial << "  new step_bits " << _BIN(step_bit) << endl;
            //while (1) delay(20);
          }

          int byte_i = set_frame( dir_bit_vector, frame_i, mask, dir_bit );
          set_frame( step_bit_vector, frame_i, mask, step_bit );

          if (DEBUGBITVECTOR >= 0 && DEBUGBITVECTOR >= i) {
            Serial << F("  dir byte[") << byte_i << F("]= ") << _BIN(dir_bit_vector[ byte_i ]) << endl;
            Serial << F("  step byte[") << byte_i << F("]= ") << _BIN(step_bit_vector[ byte_i ]) << endl;
          }

          // stop if debugging this motor
          if (DEBUGBITVECTOR >= 0 && DEBUGBITVECTOR == i) {
            for (int bi = byte_ct - 1; bi >= 0; bi--) {
              Serial << F("      ") << F(" ") << (bi > 10 ? "" : " ") << bi << F(" ");
            }
            Serial << endl;

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

            while (1) delay(20);
          }
        }

      }
      if (done_ct >= motor_ct) all_done = true;

      // Send bits
      // each is about 60micros at 4MHz spi
      SPI.transfer(dir_bit_vector, motor_ct);
      SPI.transfer(step_bit_vector, motor_ct);
      SPI.transfer(dir_bit_vector, motor_ct); // this is "step pulse off"
    }

    inline int set_frame( byte * bit_vector, int frame_i, byte mask, byte value ) {
      const int byte_i = (frame_i * bits_per_frame ) / (sizeof(byte) * 8);
      const int offset = (frame_i * bits_per_frame ) % 8;
      if (DEBUGBITVECTOR >= 0) {
        Serial << F("    set [") << byte_i << F("] offset ") << offset << F(" mask 0b") << _BIN(mask << offset) << F(" = ") << _BIN(value << offset) << endl;
      }
      bit_vector[ byte_i ] = ( bit_vector[ byte_i ] & ~(mask << offset) ) | (value << offset);
      return byte_i;
    }
};

#undef DEBUGBITVECTOR
