#pragma once

#include <SPI.h>
#include "begin_run.h"

class SPI_Shift : public BeginRun {
    // show memory and time used by spi bit shift

  public:
    MotorBits motor_bits;

    void begin() {
      motor_bits.begin();
      Serial << F("SETUP SPIShift:") << endl;

      SPI.begin();
      SPI.setClockDivider(SPI_CLOCK_DIV8); // change to <10MHz
      SPI.setBitOrder(LSBFIRST);
      SPI.setDataMode(SPI_MODE0);

    }

    boolean run() {
      SPI.transfer(motor_bits.bit_vector, MotorBits::byte_ct);
      return true;
    }
};
