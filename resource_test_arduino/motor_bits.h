#pragma once

class MotorBits : public Beastie {
    // show memory and time used by all motor bit-vector 

  public:
    static constexpr int extra_frame = 1; // the led-bar
    static constexpr int bits_per_frame = 4; // dir+,dir-,pul+,pul-
    static constexpr int byte_ct = ( (MOTOR_CT + extra_frame) * bits_per_frame ) / (sizeof(byte)*8);

    byte bit_vector[ byte_ct ];

    void setup() {

      Serial << F("SETUP MotorBits bytes_ct ") << byte_ct << endl;
    }

    void begin() {

    }

    void loop() {

    }
};
