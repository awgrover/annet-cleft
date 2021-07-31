#pragma once

class LimitSwitchBeastie : public Beastie {
    // show memory and time used by all limit switches

  public:
    static int pins[15];
    

    void setup() {
      Serial << F("SETUP LimitSwitch pin-ct= ") << array_size(pins) << endl;

      for (int p : pins) {
        pinMode(p, INPUT_PULLUP);
      }
    }

    void begin() {
      

    }

    void loop() {
      for (int p : pins) {
        volatile int v = digitalRead(p);
      }
    }
};
int LimitSwitchBeastie::pins[15] = {   // should be MOTOR_CT
      // reserve: 5=vhi, 2/3=i2c, 13=builtinled
      // 4 has no interrupt

      0, 1, 6, 7, 8, 
      9, 10,11, 12, A0, 
      A1,A2, A3, A4, A5
      // #4 unused
    };
