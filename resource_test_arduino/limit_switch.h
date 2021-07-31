#pragma once

class LimitSwitch : public Beastie {
    // show memory and time used by all limit switches

  public:
    // reserve: 5=vhi, 2/3=i2c, 13=builtinled
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
int LimitSwitch::pins[15] = {   // should be MOTOR_CT
      0, 1, 7, 9, 10,
      11, 12, 6, A0, A1,
      A2, A3, A4, A5, 4
      // 8 unused
    };
