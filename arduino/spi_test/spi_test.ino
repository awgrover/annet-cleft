#include <SPI.h>
#include <array_size.h>
#include <Streaming.h>

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
    //Serial << F("lsb ") << _BIN(bit_vector[ 0 ]) << endl;
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    // sends 0...[n-1] [n], no matter what MSB/LSB first is!
    // so, nearest shift-register is [n]
    //SPI.transfer(bit_vector, array_size(bit_vector));
    // so, nearest shift-register is [0]:
    for (int i = array_size(bit_vector) - 1; i >= 0; i--) {
      SPI.transfer( bit_vector[i] );
    }
    SPI.endTransaction();
    digitalWrite(latch_pin, LATCHSTART); digitalWrite(latch_pin, LATCHIDLE);
    //delay(500);
  }
  digitalWrite(LED_BUILTIN, LOW);
  //Serial << F("---") << endl;
  //bit_vector is whacked now
  for (int i = 0; i < array_size(bit_vector); i++) {
    bit_vector[i] = alt ? 0xFF : 0x00;
  }
  alt = ! alt;
}
