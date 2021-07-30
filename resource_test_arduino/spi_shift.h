#pragma once

#include <SPI.h>

class SPI_Shift : public Beastie {
    // show memory and time used by spi bit shift

  public:
    MotorBits motor_bits;

    void setup() {
      motor_bits.setup();
      Serial << F("SETUP SPIShift:") << endl;
    }

    void begin() {
      SPI.begin();
      SPI.setClockDivider(SPI_CLOCK_DIV8); // change to <10MHz
      SPI.setBitOrder(LSBFIRST);
      SPI.setDataMode(SPI_MODE0);

    }

    void loop() {
      SPI.transfer(motor_bits.bit_vector, MotorBits::byte_ct);
    }
};
