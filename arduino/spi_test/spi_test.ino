/*
  1. outputs a pile of bits, which can control the stepper-driver, but only on byte 0
  2. reads "miso", assuming its a shiftin-register, but only byte 0
*/
#include <SPI.h>
#include <array_size.h>
#include <Streaming.h>

// set to true to go slow (500ms between transfer), and printout values
#define SLOW 1

constexpr int latch_pin = 12;
constexpr int LATCHSTART = HIGH;
constexpr int LATCHIDLE = ! LATCHSTART;
constexpr int SLAVESELECT = 10; // samd21's don't need this
byte bit_vector[2 * 4 * 15];

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(latch_pin, OUTPUT);
  digitalWrite(latch_pin, LATCHIDLE);
  //pinMode(SLAVESELECT, OUTPUT);
  //digitalWrite(SLAVESELECT, LOW); //release chip
  SPI.begin();
  Serial.begin(115200); while (!Serial);
  Serial << endl << F("Begin") << endl;

  for (int i = 0; i < array_size(bit_vector); i++) {
    bit_vector[i] = 0xff;
  }
}

void loop() {
  static boolean alt = false;

  // cw 0b01, 0b11, 0b01
  // ccw 0b00, 0b10, 0b00
  static const byte pattern[] = { 0b00, 0b10, 0b00 }; // step|dir bits

  //Serial << F("msb ") << _HEX(bit_vector[ 0 ]) << F(" next ") << alt << endl;
  digitalWrite(LED_BUILTIN, HIGH);
  for (byte p : pattern) {
    bit_vector[0] = p;
    bit_vector[ array_size(bit_vector) - 1] = p;
    if (SLOW) Serial << F("lsb ") << _BIN(bit_vector[ 0 ]) 
    << F(" msb[") << (array_size(bit_vector) - 1) << F("] ") << _BIN(bit_vector[ array_size(bit_vector) - 1]) << endl;
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    // sends 0...[n-1] [n], no matter what MSB/LSB first is!
    // so, nearest shift-register is [n]
    // Also, reads-in to the bit_vector
    SPI.transfer(bit_vector, array_size(bit_vector));
    // so, restore the vector
    for (int i = 0; i < array_size(bit_vector); i++) {
      bit_vector[i] = 0xff;
    }

    // so, nearest shift-register is [0]:
    //for (int i = array_size(bit_vector) - 1; i >= 0; i--) {
    //  SPI.transfer( bit_vector[i] ); // this does not destroy bit_vector
    //}
    SPI.endTransaction();
    digitalWrite(latch_pin, LATCHSTART); digitalWrite(latch_pin, LATCHIDLE);
    if (SLOW) delay(500);// comment to test speed, uncomment to see blinkin
  }
  digitalWrite(LED_BUILTIN, LOW);
  if (SLOW) Serial << F("---") << endl;
  //bit_vector is whacked now
  for (int i = 0; i < array_size(bit_vector); i++) {
    bit_vector[i] = alt ? 0xFF : 0x00;
  }
  alt = ! alt;
}
