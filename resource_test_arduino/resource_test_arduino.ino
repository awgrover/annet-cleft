/*
   Test various pieces for their speed and ram
   Only has partial behavior of actual pieces/objects/classes.

   Designed for
   Metro M0 Express (samd21)
    32k RAM 48MHz
    32bit ints (doesn't force 32bit aligned data)
    (which is same as itsybitsy)
    Metro has 25 pins, itsybitys has 23

    Supposedly, constexpr *somestr = "BOB"; uses flash, not ram
*/

#include <AccelStepper.h> // Mike McCauley <mikem@airspayce.com>
#include <Streaming.h> // Mikal Hart <mikal@arduiniana.org>

#include "array_size.h"
#include "freememory.h"
#include "every.h"
#include "ExponentialSmooth.h"
#include "AccelStepperShift.h"
Print &operator <<(Print &obj, const __FlashStringHelper* arg) {
  obj.print(arg);
  return obj;
}

constexpr int MOTOR_CT = 15;
constexpr int LATCH_PIN = LED_BUILTIN;

// NB: a #include list is auto-generated of extant .h files
//  and it is alphabetical, so this order is not relevant
//  and, in fact, is redundant for arduino-ide
#include "begin_run.h"
#include "accel_beastie.h"
#include "motor_bits.h"
#include "spi_shift.h"
#include "limit_switch_beastie.h"
#include "accel_shift_beastie.h"

Every say_status(300);

BeginRun *beast[] = {
  // individually test by commenting out all but one
  //new NoopBeastie() //, // for null loop timing
  //new AccelBeastie(),
  //new MotorBits(),
  new SPI_Shift(),
  //new LimitSwitchBeastie(),
  //new AccelShiftBeastie(MOTOR_CT, LATCH_PIN),
};

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(2000); // allow uploading when cpu gets screwed
  digitalWrite(LED_BUILTIN, LOW);

  const int base_memory = freeMemory();
  int last_free = base_memory;

  Serial.begin(115200); while (!Serial) {}
  Serial << endl;
  Serial << F("Base ") << base_memory << endl;
  Serial << F("After Serial.begin ") << freeMemory() << endl;
  Serial << F("Clock ") << ((F_CPU / 1000.0) / 1000.0) << F(" MHz") << endl;

  int i = 0;

  i = 0;
  for (BeginRun* b : beast) {
    b->begin();
    int freem = freeMemory();
    Serial << F("BEGIN [") << i << F("] ") << freeMemory()
           << F(" used ") << (last_free - freem) << endl;
    i++;
    last_free = freeMemory();
  }

  Serial << F("Before LOOP @ ") << millis() << F(" Free: ") << freeMemory() << endl;
}

void loop() {
  unsigned long last_micros = micros();
  static ExponentialSmooth<unsigned long> elapsed(5);

  for (BeginRun* b : beast) {
    b->run();
  }

  unsigned long now = micros();
  elapsed.average( now - last_micros  );
  last_micros = now;

  if ( say_status() ) {
    Serial << F("Loop ") << elapsed.value() << " free " << freeMemory() << endl;
  }

}
