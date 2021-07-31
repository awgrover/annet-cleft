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
#include "every.h"
#include "ExponentialSmooth.h"
#include "AccelStepperShift.h"
#include "LimitSwitch.h"

constexpr int MOTOR_CT = 15;
constexpr int LATCH_PIN = LED_BUILTIN;

AccelStepperShift* stepper_shift = new AccelStepperShift(MOTOR_CT, LATCH_PIN);
LimitSwitch* limit_switches = new LimitSwitch(MOTOR_CT);

BeginRun* systems[] = {
  stepper_shift,
  limit_switches
};

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
  Serial << F("Start free ") << base_memory << endl;
  Serial << F("After Serial.begin ") << freeMemory() << endl;
  Serial << F("Clock ") << ((F_CPU / 1000.0) / 1000.0) << F(" MHz") << endl;

  // Each main object
  // We `new` them so we can more easily see memory usage in console
  last_free = freeMemory();
  for (BeginRun* a_system : systems) {
    a_system->begin();
    Serial << F(" free ") << freeMemory()
           << F(" used ") << (last_free - freeMemory()) << endl;
    last_free = freeMemory();
  }

  // tie some things together
  stepper_shift->limit_switch = limit_switches->status;

  stepper_shift->goto_limit();
  
  // and
  Serial << F("End setup() @ ") << millis() << F(" Free: ") << freeMemory() << endl;
}

void loop() {
  // track loop time (in microseconds)
  static Every say_status(1000);
  static ExponentialSmooth<unsigned long> elapsed(5);
  static unsigned long last_elapsed = 0;
  const unsigned long last_micros = micros(); // top of loop

  // Run each object/module/system
  for (BeginRun* a_system : systems) {
    a_system->run();
  }

  // keep track of loop time, and output status every so often
  // but only if it changes (less noisy)
  const unsigned long now = micros();
  const unsigned long elapsed_micros = now - last_micros;
  if (elapsed_micros < 200ul) delayMicroseconds(200); // ensure usb can notice upload
  elapsed.average( elapsed_micros  );

  if ( say_status() && last_elapsed != elapsed.value() ) {
    Serial << F("Loop ") << elapsed.value() << " free " << freeMemory() << endl;
    last_elapsed = elapsed.value();
  }
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
