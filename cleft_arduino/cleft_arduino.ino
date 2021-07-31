/*
  Interactive animation system for Cleft (Annett Couwenberg)
    cf. http://www.annetcouwenberg.com/spirit
  1st installation at https://cadvc.umbc.edu/ 2021

  The sculpture has 15 segments, roughly pie-pieces.
  Each piece has a motor attached to its outer edge,
  the inner edge being at a fixed height, and fixed to adjacent segments.
  So, it can tilt.

  Based on a sensor (initially a 8x8 ir camera),
  the segments should perform various pleasing motions.

  As of July 2021, the design is:
    one arduino Zero equivalent (48MHz)
    15 motors, with TB6600 controllers, and limit switches
    a chain of shift registers driving the controllers (SPI)
    3 IR cameras (I2C)
    Several LED indicators
    AccelStepper to run the motors (but capturing as bit-vector for SPI)
    An animation framework based on key-frames and transition-functions.
    An interaction module that interprets the cameras and decides on animation sequences.
    Lots of wiring & connectors

*/

#include <AccelStepper.h> // Mike McCauley <mikem@airspayce.com>
#include <Streaming.h> // Mikal Hart <mikal@arduiniana.org>
// Not in my version of streaming, for F("") strings:
Print &operator <<(Print &obj, const __FlashStringHelper* arg) {
  obj.print(arg);
  return obj;
}

// NB: a #include list is auto-generated of extant .h files
//  and it is alphabetical, so this order is not relevant
//  and, in fact, is redundant for arduino-ide
#include "freememory.h"
#include "AccelStepperShift.h"

constexpr int MOTOR_CT = 15;
constexpr int LATCH_PIN = LED_BUILTIN;

AccelStepperShift* motor_system; // holds AccelSteppers, bit-vector, and shift-out code

void setup() {
  // Integrated USB chips have trouble uploading sometimes
  // (especially if you corrupt memory, or go into a tight loop)
  // So, give some time at startup to notice re-upload
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(2000); // allow uploading when cpu gets screwed
  digitalWrite(LED_BUILTIN, LOW);

  // We want to note memory
  const int base_memory = freeMemory();
  int last_free = base_memory;

  Serial.begin(115200); while (!Serial) {}
  Serial << endl;
  Serial << F("Base ") << base_memory << endl;
  Serial << F("After Serial.begin ") << freeMemory() << endl;
  Serial << F("Clock ") << ((F_CPU / 1000.0) / 1000.0) << F(" MHz") << endl;

  // Each main object

  motor_system = new AccelStepperShift(MOTOR_CT, LATCH_PIN);
  Serial << F("AccelStepperShift") << endl;
  Serial << F("  new")
         << F(" free ") << freeMemory()
         << F(" used ") << (last_free - freeMemory()) << endl;
  last_free = freeMemory();
  motor_system->begin();
  Serial << F("  begin")
         << F(" free ") << freeMemory()
         << F(" used ") << (last_free - freeMemory()) << endl;
  last_free = freeMemory();
}

void loop() {
  delay(20);
}

/*
  // prefix
  #include "awg_combinators/serial.h"

  constexpr int SEGMENT_CT = 7;
  using Heights = float[SEGMENT_CT];

  // Our bits
  #include "AccelStepperMotorShield.h"
  #include "ExponentialSmooth.h"
  #include "every.h"
  #include "freememory.h"
  #include "array_animation.h"


  void setup() {
  Serial.begin(115200); while (!Serial);
  Serial << F("Startup ") << millis() << F(" free ") << freeMemory() << endl;


  all_motors.begin();
  Serial << F("Animation ") << millis() << F(" free ") << freeMemory() << endl;
  }


  void loop() {
  static unsigned long last_time = millis();
  Heights delta;
  static ArrayAnimation anim; // = ArrayAnimation.goto_zero_plane();

  // note framerate
  static Every say_looprate(500);
  static ExponentialSmooth<unsigned long> framerate(1);
  static unsigned long last_loop_time = millis();
  static Every animation_update(1000 / 10); // 10 frames/sec maximum

  // update motors every frame
  if ( all_motors.run() || animation_update() ) { // i.e. done-running or time for next frame
    Serial << F("Next step") << endl;

    anim.update( millis() - last_time, delta);
    last_time = millis();
    Serial << F("delta "); pheights( delta ); Serial << endl;

    anim.add( delta );
    Heights motor_values;
    //anim.scaled( StepsPerMeter, motor_values);
  }

  // info
  framerate.average( millis() - last_loop_time );
  last_loop_time = millis();
  if ( say_looprate() ) {
    float fr = 1000.0 / framerate.value();
    Serial << F("looprate ") << fr << F(" free ") << freeMemory() << endl;
  }

  }

  void pheights( Heights delta ) {
  for (int i = 0; i < SEGMENT_CT; i++) {
    Serial << delta[i] << F(" ");
  }
  }
*/
