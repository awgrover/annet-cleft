#pragma once

#include "array_size.h"
#include "every.h"

// true to print out limit/unlimit messages
#define DEBUGLIMIT 1

#ifndef DEBUGLIMIT
#define DEBUGLIMIT 0
#endif

class LimitSwitch : public BeginRun {
    // read & track all the limit switches
    // respond "immediately" to "on"
  public:
    static const int pins[15];
    const int count;
    boolean* status = NULL; // [] of limit switch status true=triggered
    Timer ** debouncer = NULL; // [] for debouncing
    static constexpr int debounce_time = 10; // maybe less...
    Every check = Every(debounce_time/2); // only check every so often

    LimitSwitch(int ct) : count(ct) {}

    void begin() {
      if (count > (int) array_size(pins)) {
        Serial << F("FAIL: limit of 15 pins for limit switches") << endl;
        while (1) ;
      }

      for (int i = 0; i < count; i++) {
        pinMode(pins[i], INPUT_PULLUP);
      }

      Serial << F("BEGIN LimitSwitch ") << count << F(" pins ");
      for (int i = 0; i < count; i++) {
        Serial << pins[i] << F(" ");
      }
      Serial << endl;

      status = new boolean[count];
      debouncer = new Timer*[count];
      for (int i = 0; i < count; i++) {
          status[i] = false;
          debouncer[i] = new Timer(debounce_time);
      }
    }

    boolean run() {
      if (check()) {
        // takes about 175micros for 15 at 8MHz on 32u4
        for (int i = 0; i < count; i++) {
          int value = ! digitalRead( pins[i] ); // HIGH=open. sooo slow. 10micros
          if (value) {
            if ( ! status[i] ) {
              debouncer[i]->reset(); // debounce set
              if (DEBUGLIMIT) Serial << F("LIMIT ") << i << endl;
            }
            status[i] = true; // "instantly" true            
          } else if ( debouncer[i]->after() ) {
            if (DEBUGLIMIT && status[i]) Serial << F("UNLIMIT ") << i << endl;
            status[i] = false;
          }
        }
      }
      return true; // doesn't really mean anything
    }
};

const int LimitSwitch::pins[15] = {   // should be MOTOR_CT
  // All should have interrupt
  // reserve: 5=vhi, 2/3=i2c, 13=builtinled
  // 4 has no interrupt

  0, 1, 6, 7, 8,
  9, 10, 11, 12, A0,
  A1, A2, A3, A4, A5
  // #4 unused
};
