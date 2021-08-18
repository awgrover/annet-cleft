/*
  Has several test modes in loop().
  For reading, writing, and read+writing shift-registers.
  Assumes:
  A chain of out shift-registers (serial-to-parallel) on spi MOSI,
  A chain of corresponding in shift-registers (parallel-to-serial) on the same spi MISO.

  Testing:
    adjust the MOTOR_CT to get the REGISTER_CT you want

    Probably do in this order:
    set SLOW=1 & SLOWLATCH=1
      more output, blinks led-bar, and long delay between shift in/out
    shift-read (note off-by-one bit)
      pull various in-bits to high/low
      most likely to succeed: shows in-wiring is ok
    spi-read
      same, shows spi works at much higher speed
    shift-write
      put leds on various out-bits
      most likely to succeed: shows out-wiring is ok
    spi-write
      same, shows spi works at much higher speed
    spi-read-write
      test with leds & pulling inputs low.
      then add stepper
    set SLOWLATCH=0
    spi-read-write
      shows higher speed is ok for in-latch
    set SLOW=0 & hookup a stepper to various out-shift-registers
    spi-read-write
      shows it running steppers at more realistic speed

*/
#include <SPI.h>
#include <array_size.h>
#include <Streaming.h>

// set to true to go slow (500ms between transfer), and printout values: easily seen blink
#define SLOW 1
// set to true for large delay between latch/etc
#define SLOWLATCH 1

constexpr int MOTOR_CT = 2 * 8 / 2; // nb: see REGISTER_CT, i.e. to get bytes divide by bits-per
constexpr int BITS_PER_MOTOR = 2;
constexpr int REGISTER_CT = (int) ceil((float)MOTOR_CT * BITS_PER_MOTOR / 8);
constexpr int latch_pin = 12;
constexpr int in_latch_pin = 11; // for the shift-in, is opposite sense
constexpr int LATCHSTART = HIGH;
constexpr int LATCHIDLE = ! LATCHSTART;
constexpr int SLAVESELECT = 10; // samd21's don't need this
// i'm using an LED bar on the 2nd out-shift-register, which is bit_vector[-2]
constexpr int COPY_0IN_TO_OUT_BYTE = 1; // copy the 0th in byte, to the xTh outbyte (nb msb/lsb reversed)
static_assert(COPY_0IN_TO_OUT_BYTE < REGISTER_CT, "Can't copy to a byte that doesn't exist"); // xTh byte is in the bit_vector, so...
byte bit_vector[ REGISTER_CT ];
byte input_bit_vector[ array_size(bit_vector) ];
constexpr int MAX_BYTE_I = array_size(bit_vector) - 1;

void dump_bit_vector(byte *bytes, int byte_ct = 0);// arduino does not like default args in definition

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

  //shift_read();
  //spi_read();
  spi_read_write();
}

void shift_read() {
  // NB: this is 1 bit off! in-pin0 is bit 1, we lose in-pin7
  // do REGISTER_CT + 1 bytes worth of shift-in,
  // which should show if you have the serial-in-daisy-chain pulled-low.
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
  digitalWrite(in_latch_pin, LOW);
  if (SLOWLATCH) delay(1);
  digitalWrite(in_latch_pin, HIGH);
  if (SLOWLATCH) delay(1);

  digitalWrite(SCK, LOW);
  for (int i = 0; i < array_size(bit_vector); i++) {
    input_bit_vector[i] = shiftIn(MISO, SCK, MSBFIRST);
  }
  byte extra[1];
  extra[0] = shiftIn(MISO, SCK, MSBFIRST);

  dump_bit_vector(input_bit_vector);
  Serial << F("  xtra ");
  dump_bit_vector(extra, 1);

  digitalWrite(LED_BUILTIN, LOW);
  delay(SLOW ? 500 : 10);

}

void spi_read() {
  // do REGISTER_CT + 1 bytes worth of shift-in,
  // which should show if you have the serial-in-daisy-chain pulled-low.

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
  digitalWrite(in_latch_pin, LOW);
  if (SLOWLATCH) delay(1);
  digitalWrite(in_latch_pin, HIGH);
  if (SLOWLATCH) delay(1);


  byte extra[1] = {0}; // for the SER (daisy-chain in)
  SPI.beginTransaction(SPISettings(SLOW ? 1000 : 1000000, MSBFIRST, SPI_MODE0));
  // shifts the bit-vector out (don't care), and shifts-in into same bit-vector
  SPI.transfer(input_bit_vector, array_size(input_bit_vector));
  SPI.transfer(extra, array_size(extra));
  SPI.endTransaction();

  //Serial << F("[0]  ") << _BIN(input_bit_vector[ 0]) << endl;
  //Serial << F(" [") << MAX_BYTE_I << F("] ") << _BIN(input_bit_vector[ MAX_BYTE_I ]) << endl;
  dump_bit_vector(input_bit_vector);
  Serial << F("  xtra ");
  dump_bit_vector(extra, 1);

  digitalWrite(LED_BUILTIN, LOW);
  delay(SLOW ? 500 : 10);
}

void spi_read_write() {
  // sets up a stepper-motor sequence to step 1 step forward per loop.
  // reads all in-shift-registers, (and writes the 1st stage of stepping): dir before step
  // copies the in-byte[0] to output byte[COPY_0IN_TO_OUT_BYTE], i.e. led-bar
  // writes out (the 2nd stage of stepping): actual move & led-bar
  // (does the 3rd stage of stepping): pulse off

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
  digitalWrite(in_latch_pin, LOW);
  if (SLOWLATCH) delay(1);
  digitalWrite(in_latch_pin, HIGH);
  if (SLOWLATCH) delay(1);

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
  if (!SLOW) delay(10); // time for usb interrupt and stepper speed
}

void dump_bit_vector(byte *bytes, int byte_ct) {
  // NB; [8] is shifted out last, so is shift-register nearest ard
  // We reorder here so it reads right-to-left
  if (byte_ct == 0) byte_ct = array_size(bit_vector); // optional count

  for (int bi = 0; bi < byte_ct; bi++) {
    for (int bit_i = 0; bit_i < 8; bit_i++) {
      if ( ! (bit_i % BITS_PER_MOTOR) && bit_i != 0) Serial << ".";
      Serial << ( ( bytes[bi] & (1 << (7 - bit_i)) ) ? '1' : '0' );
    }
    Serial << "  ";
  }
  Serial << endl;
}
