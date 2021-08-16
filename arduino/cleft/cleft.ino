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

  There is a provision for a visualization program written in processing.
  We send commands starting with "<".
  We respond to a few commands.

  As of July 2021, the design is:
    one arduino Zero equivalent (48MHz): adafruit Metro M0 Express
    15 motors, with TB6600 controllers, and limit switches
    a chain of shift registers driving the controllers (SPI)
    a chain of (in) shift-registers to read the limit switches (SPI at same time)
    3 IR cameras (I2C)
    Several LED indicators
    AccelStepper to run the motors (but capturing as bit-vector for SPI)
    An animation framework based on key-frames and transition-functions.
    An interaction module that interprets the cameras and decides on animation sequences.
    Lots of wiring & connectors


    Wiring
      shift registers (in/out), enable, : see AccelStepperShift.h
      LED-Bar, end of (out) shift-register chain
        bit   assignment                          Desc
        9     Vcc "power"                         Should be on == power
        8     common-enable                       off on power-up, on at setup() and stays on
        7     spi-heartbeat (shiftreg[H]-7)       blink every 200ms, indicates spi.transfer loop being called
        6     SCK                                 looks always on after spi.begin (actually blinks off, but imperceptible)
        5     latch-out                           very fast flash on each spi-transfer
        4     latch-in                            looks always on after spi.begin (actually blinks off, but imperceptible)
        3     recent step (shiftreg[B]-1)         on for 200ms if any motor step'd
        2     recent limit (shiftreg[C]-2)        on for 200ms if any limit switch was on
        1     spi-heartbeat (shiftreg[H']-9)      complementary blink every 200ms, indicates spi.transfer loop being called
        0     Vcc "power"                         Should be on == power

      Built-in Neopixel
        Green         power-on (def)
        Yellow        setup()
        Blink Orange  doing uplimit()
        Blink Green   loop'ing() : heartbeat
        Red           fault (past limit, or hit limit during animation)

      Neopixel 6 (center w/pentagon 5)
        spi-heartbeat -- center
        clockwise from top
        sensor
        animation state
*/

#include <Adafruit_NeoPixel.h> // Adafruit
#include <AccelStepper.h> // Mike McCauley <mikem@airspayce.com>
#include <Streaming.h> // Mikal Hart <mikal@arduiniana.org>
// Not in my version of streaming, for F("") strings:
Print &operator <<(Print &obj, const __FlashStringHelper* arg) {
  obj.print(arg);
  return obj;
}

// up here so available to the #includes, which is ugly
Adafruit_NeoPixel builtin_neo(1,  40, NEO_GRB + NEO_KHZ800);
const int bob=1;
const uint32_t NEO_STATE_SETUP = Adafruit_NeoPixel::Color(255, 255, 0); // Yellow
const uint32_t NEO_STATE_FAULT = Adafruit_NeoPixel::Color(255, 0, 0); // Red
const uint32_t NEO_OFF = Adafruit_NeoPixel::Color(0, 0, 0); // off
const uint32_t NEO_STATE_LOOPING = Adafruit_NeoPixel::Color(0, 255, 0); // Green (blink)
const uint32_t NEO_STATE_UPLIMIT = Adafruit_NeoPixel::Color(255, 165, 0); // Orange (blink)


// NB: a #include list is auto-generated of extant .h files
//  and it is alphabetical, so this order is not relevant
//  and, in fact, is redundant for arduino-ide
#include "freememory.h"
#include "every.h"
#include "array_size.h"
#include "ExponentialSmooth.h"
#include "AccelStepperShift.h"
// #include "ArrayAnimation.h"
#include "AnimationWave1.h"
#include "Commands.h"

// if true: no initial move-to-uplimit, instead +|- move up 2, down 2
#define DEBUGMOVETEST 1
#ifndef DEBUGMOVETEST
#define DEBUGMOVETEST 0
#endif

constexpr int MOTOR_CT = 15;
constexpr int LATCH_PIN = 12;
constexpr int SH_LD_PIN = 11; // allow shift while high, load on low
constexpr int MOTOR_ENABLE_PIN = 10; // common stepper-driver enable



// SYSTEMS
// We run them via systems[] (below)
// We need to explicitly refer to a few, so need explicit name
// When testing/developing, you can set these to NULL to skip using them
AccelStepperShift* stepper_shift = new AccelStepperShift(MOTOR_CT, LATCH_PIN, SH_LD_PIN, MOTOR_ENABLE_PIN);
//ArrayAnimation* animation = new ArrayAnimation(MOTOR_CT);
AnimationWave1* animation = new AnimationWave1( // moving sine wave
  stepper_shift,
  0.15, // amplitude meters
  0.5, // wavelength fraction that fits in 1/2 of cleft
  0.25, // frequency
  1 // cycles
);

Animation* Animation::current_animation = animation;

// this list is only for serial commands (see Commands)
Animation* Animation::animations[] = {
  animation,
  new AnimationWave2( // the cute wave "@"
    stepper_shift,
    5, // segments
    0.25, // max amplitude
    1, // frequency
    1 // cycles
  ),
  NULL, NULL, NULL, NULL, NULL, NULL,
  new AnimationFast(stepper_shift), // 9
};
const int Animation::animation_ct = array_size(animations);

// We automatically call .begin() in setup, and .run() in loop, for each thing in systems[]
// When testing/developing, you can set these to NULL to skip using them
BeginRun* systems[] = {
  // all null: 9micros @ 8MHz 32u4
  stepper_shift,  // idle: 350micros @ 8MHz 32u4 1164 bytes used
  // runall: 300micros @48Mhz samd21 1620 used
  // idle: 78micros @48Mhz
  //// NB Animation::current_animation, NEVER goes in here. sad
  new Commands(stepper_shift),
};

void setup() {
  // We want to note memory
  const int base_memory = freeMemory();
  int last_free = base_memory;

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  builtin_neo.begin();
  // weirdly, the first set doesn't show
  builtin_neo.setBrightness(60); // too bright!
  builtin_neo.setPixelColor(0, 0, 0, 255) ; // NEO_STATE_SETUP );
  builtin_neo.show();
  builtin_neo.setPixelColor(0, NEO_STATE_SETUP );
  builtin_neo.show();

  // Integrated USB chips have trouble uploading sometimes
  // (especially if you corrupt memory, or go into a tight loop)
  // So, give some time at startup to notice re-upload
  delay(2000); // allow uploading when cpu gets screwed
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(115200); while (!Serial) {}
  Serial << endl;
  // This prefix should be first, for memory/millis
  Serial << F("Start free ") << base_memory << endl;
  Serial << F("After Serial.begin ") << freeMemory() << endl;
  Serial << F("Clock ") << ((F_CPU / 1000.0) / 1000.0) << F(" MHz") << endl;

  // allow other spi users
  SPI.begin();

  // Each main object
  last_free = freeMemory();
  for (BeginRun* a_system : systems) {
    if (! a_system) continue; // skip NULLs

    a_system->begin();
    Serial << F(" free ") << freeMemory()
           << F(" used ") << (last_free - freeMemory()) << endl;
    last_free = freeMemory();
  }

  // other startup behavior

  if (stepper_shift) stepper_shift->goto_limit();

  if (DEBUGMOVETEST) {
    Animation::current_animation->state = Animation::Off;
  }

  // and
  Serial << F("End setup() @ ") << millis() << F(" Free: ") << freeMemory() << endl;
}

void loop() {
  // track loop time (in microseconds)
  static Every say_status(1000);
  static ExponentialSmooth<unsigned long> elapsed(5);
  static unsigned long last_elapsed = 0;
  const unsigned long last_micros = micros(); // top of loop

  heartbeat(NEO_STATE_LOOPING);
  
  // Run each object/module/system
  for (BeginRun* a_system : systems) {
    if (! a_system) continue; // skip NULLs
    a_system->run();
  }
  // bah, I change animations, but I would have to update systems[n]
  if (Animation::current_animation) Animation::current_animation->run();

  // let a system clean up from the state of one loop
  // i.e. systems might check each other for a per-loop state
  // e.g. AccelStepperNote sets do_step if it wants a step this loop
  for (BeginRun* a_system : systems) {
    if (! a_system) continue; // skip NULLs
    a_system->finish_loop();
  }

  // keep track of loop time, and output status every so often
  // but only if it changes (less noisy)
  const unsigned long now = micros();
  const unsigned long elapsed_micros = now - last_micros;
  if (elapsed_micros < 200ul) delayMicroseconds(200); // ensure usb can notice upload
  elapsed.average( elapsed_micros  );

  if ( say_status() && last_elapsed != elapsed.value() ) {
    Serial << F("Loop ") << elapsed.value() << " free " << freeMemory()
           << F(" animation ") << ( (long) Animation::current_animation)
           << endl;
    last_elapsed = elapsed.value();
  }
}

void heartbeat(const uint32_t color) {
  static Every::Toggle heartbeat(200);
  if ( heartbeat() ) {
    builtin_neo.setPixelColor(0, heartbeat.state ? NEO_OFF : color );
    builtin_neo.show();
  }
}
  
/*

  void loop() {

  Heights delta;
  static ArrayAnimation anim; // = ArrayAnimation.goto_zero_plane();

  // note framerate
  static Every animation_update(1000 / 10); // 10 frames/sec maximum

  // update motors every frame
  if ( animation_update() ) { // i.e. done-running or time for next frame
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
