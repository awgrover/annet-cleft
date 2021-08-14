/*
  1. outputs a pile of bits, which can control the stepper-driver, but only on byte 0
    the other bytes alternate off/on
  2. reads "miso", assuming its a shiftin-register, but only byte 0
    copies the first 2 bits to output byte[n]'s first 2 bytes
*/
#include <SPI.h>
#include <array_size.h>
#include <Streaming.h>

// set to true to go slow (500ms between transfer), and printout values
#define SLOW 1

constexpr int latch_pin = 12;
constexpr int in_latch_pin = 11; // for the shift-in, is opposite sense
constexpr int LATCHSTART = HIGH;
constexpr int LATCHIDLE = ! LATCHSTART;
constexpr int SLAVESELECT = 10; // samd21's don't need this
// i'm using an LED bar on the 2nd out-shift-register, which is bit_vector[-2]
constexpr int COPY_0IN_TO_OUT_BYTE = 2; // copy the 0th in byte, to the xTh outbyte (nb msb/lsb reversed)
constexpr int BITS_PER_MOTOR = 2;
byte bit_vector[(int) ceil(15.0 * BITS_PER_MOTOR / 8)];
byte input_bit_vector[ array_size(bit_vector) ];
constexpr int MAX_BYTE_I = array_size(bit_vector) - 1;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(latch_pin, OUTPUT);
  digitalWrite(latch_pin, LATCHIDLE);
  pinMode(in_latch_pin, OUTPUT);
  digitalWrite(in_latch_pin, HIGH);

  pinMode(SLAVESELECT, OUTPUT);
  digitalWrite(SLAVESELECT, LOW); //release chip
  Serial.begin(115200); while (!Serial);
  Serial << endl << F("Begin motors ") << (8 * array_size(bit_vector) / BITS_PER_MOTOR)
         << F(", bytes ") << array_size(bit_vector)
         << F(", bits per ") << BITS_PER_MOTOR
         << endl;

  for (int i = 0; i < array_size(bit_vector); i++) {
    bit_vector[i] = 0xff;
  }
}

void loop() {
  // pick one
  read_write();
  //shift_read();
  //spi_read();
}

void shift_read() {
  static boolean first_time = true;
  if (first_time) {
    // aka setup()
    pinMode(MISO, INPUT);
    pinMode(SCK, OUTPUT);
    pinMode(in_latch_pin, OUTPUT);
    digitalWrite(in_latch_pin, HIGH);
    first_time = false;
  }

  digitalWrite(LED_BUILTIN, HIGH);

  for (int i = 0; i < array_size(bit_vector); i++) {
    input_bit_vector[i] = 0x00;
  }

  // read input pins, allow shift-in (5ns? 1000ns? for low to read)
  digitalWrite(in_latch_pin, LOW); delay(1); digitalWrite(in_latch_pin, HIGH); delay(1);

  digitalWrite(SCK, LOW);
  input_bit_vector[0] = shiftIn(MISO, SCK, MSBFIRST);
  input_bit_vector[1] = shiftIn(MISO, SCK, MSBFIRST);

  dump_bit_vector(input_bit_vector);

  digitalWrite(LED_BUILTIN, LOW);
  delay(SLOW ? 500 : 10);

}

void spi_read() {
  static boolean first_time = true;
  if (first_time) {
    SPI.begin();
    first_time = false;
  }

  digitalWrite(LED_BUILTIN, HIGH);

  for (int i = 0; i < array_size(bit_vector); i++) {
    input_bit_vector[i] = 0x00;
  }

  // note no (shift-in) latch_pin, because we are only concerned with reading

  // read input pins, allow shift-in (5ns? 1000ns? for low to read)
  digitalWrite(in_latch_pin, LOW); delay(1); digitalWrite(in_latch_pin, HIGH); delay(1);

  SPI.beginTransaction(SPISettings(SLOW ? 1000 : 1000000, MSBFIRST, SPI_MODE0));
  // shifts the bit-vector out (don't care), and shifts-in into same bit-vector
  SPI.transfer(input_bit_vector, array_size(input_bit_vector));
  SPI.endTransaction();

  //Serial << F("[0]  ") << _BIN(input_bit_vector[ 0]) << endl;
  //Serial << F(" [") << MAX_BYTE_I << F("] ") << _BIN(input_bit_vector[ MAX_BYTE_I ]) << endl;
  dump_bit_vector(input_bit_vector);

  digitalWrite(LED_BUILTIN, LOW);
  delay(SLOW ? 500 : 10);
}

void read_write() {
  static boolean first_time = true;
  if (first_time) {
    SPI.begin();
    first_time = false;
  }

  static boolean alt = false; // for blink in unused bits

  // cw 0b01, 0b11, 0b01
  // ccw 0b00, 0b10, 0b00
  static const byte pattern[] = { 0b00, 0b10, 0b00 }; // step|dir bits

  //Serial << F("msb ") << _HEX(bit_vector[ 0 ]) << F(" next ") << alt << endl;
  digitalWrite(LED_BUILTIN, HIGH);

  // cause shiftin to read inputs, allow shift-out
  digitalWrite(in_latch_pin, LOW); delay(1); digitalWrite(in_latch_pin, HIGH); delay(1);

  boolean first_transfer = true; // of the three steps for motorcontrol
  for (byte p : pattern) {
    bit_vector[0] = p;
    // bit_vector[ MAX_BYTE_I ] = p; // while debugging byte order
    if (SLOW) Serial << F("lsb ") << _BIN(bit_vector[ 0 ])
                       << F(" copy[") << COPY_0IN_TO_OUT_BYTE << F("] ") << _BIN(bit_vector[ COPY_0IN_TO_OUT_BYTE]) << endl;

    SPI.beginTransaction(SPISettings(SLOW ? 1000 : 1000000, MSBFIRST, SPI_MODE0));
    // sends 0...[n-1] [n], no matter what MSB/LSB first is!
    // so, nearest shift-register is [n]
    // Also, reads-in to the bit_vector
    SPI.transfer(bit_vector, array_size(bit_vector));

    // And, capture the input
    // for the 74HC165, the inputs are read when latch is low (above)
    if (first_transfer) {
      memcpy( input_bit_vector, bit_vector, array_size(bit_vector));
      if (SLOW) {
        Serial << F("IN  ");
        dump_bit_vector(input_bit_vector);
      }

      if (SLOW) Serial << F(" copy [0] to [") << COPY_0IN_TO_OUT_BYTE << F("] ") <<  endl;
    }

    // so, restore the vector
    for (int i = 0; i < array_size(bit_vector); i++) {
      bit_vector[i] = alt ? 0xFF : 0x00;
    }

    // we are copying the in[0] to out[xTh] (to run an led bar)
    bit_vector[COPY_0IN_TO_OUT_BYTE] = input_bit_vector[0];
    if (SLOW) {
      Serial << F("Out ");
      dump_bit_vector(bit_vector);
    }

    // so, nearest shift-register is [0]:
    //for (int i = MAX_BYTE_I; i >= 0; i--) {
    //  SPI.transfer( bit_vector[i] ); // this does not destroy bit_vector
    //}
    SPI.endTransaction();
    digitalWrite(latch_pin, LATCHSTART); digitalWrite(latch_pin, LATCHIDLE);
    if (SLOW) delay(500);// comment to test speed, uncomment to see blinkin
    first_transfer = false;
  }

  digitalWrite(LED_BUILTIN, LOW);
  if (SLOW) {
    Serial << F("---") << endl;
  }
  //bit_vector is whacked now
  for (int i = 0; i < array_size(bit_vector); i++) {
    bit_vector[i] = alt ? 0xFF : 0x00;
  }
  alt = ! alt;
  delay(10); // way more than shift-in to read
}

void dump_bit_vector(byte *bytes) {
  // NB; [8] is shifted out last, so is shift-register nearest ard
  // We reorder here so it reads right-to-left
  for (int bi = 0; bi < array_size(bit_vector); bi++) {
    for (int bit_i = 0; bit_i < 8; bit_i++) {
      if ( ! (bit_i % BITS_PER_MOTOR) && bit_i != 0) Serial << ".";
      Serial << ( ( bytes[bi] & (1 << (7 - bit_i)) ) ? '1' : '0' );
    }
    Serial << "  ";
  }
  Serial << endl;
}
