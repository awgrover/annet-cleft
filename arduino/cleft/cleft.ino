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

  Review the #define's, which change some debug & performance modes. I
  use this code in various modes (especially with the visualization) to test and develop.
  NB: AccelStepperShift.h has a relevant #define.

  In Arduino-IDE, set the Preferences:Compiler Warnings to at lease "more". Instance variable
  initiatlization order is important. We should be warning free.

  As of Nov 2021 the design is:
    one arduino Zero equivalent (48MHz): adafruit Metro M0 Express
      15 motors, with TB6600 controllers, and limit switches
      a chain of shift registers driving the controllers (SPI)
      a chain of (in) shift-registers to read the limit switches (SPI at same time)
      Several LED indicators
      AccelStepper to run the motors (but capturing as bit-vector for SPI)
    one Pi Zero
      1 IR cameras (I2C)
      python code to analyze IR camera and decide on animation
    Lots of wiring & connectors
    Not done yet:
    An animation framework based on key-frames and transition-functions.

    Wiring
      Arduino USB to Pi USB (using serial mode, and powering arduino)
      Pi : i2c pins -> i2c booster -> 3' wire -> i2c Mux -> IR Camera
      shift registers (in/out), latch-out, latch-in, stepper-enable, : see AccelStepperShift.h
      LED-Bar, end of (out) shift-register chain
        bit   assignment                          Desc
        9     Vcc "power"                         Should be on == power
        8     common-enable                       off on power-up, on at setup() and stays on
        7     spi-heartbeat (shiftreg[H]-7)       blink every 200ms, indicates spi.transfer loop being called
        6     SCK                                 looks always on during spi (actually blinks off, but imperceptible)
        5     latch-out                           very fast flash on each spi-transfer
        4     latch-in                            looks always on after spi.begin (actually blinks off, but imperceptible)
        3     recent step (shiftreg[B]-1)         on for 200ms if any motor step'd
        2     recent limit (shiftreg[C]-2)        on for 200ms if any limit switch was on
        1     spi-heartbeat (shiftreg[H']-9)      QH' (serial out) = H, slightly out of phase, blink every 200ms, indicates spi.transfer loop being called
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

  Several of the .h files are from my personal library of useful things:
    https://github.com/awgrover/misc_arduino_tools
  and especially a non-blocking periodic & timer:
    https://github.com/awgrover/misc_arduino_tools/blob/master/every/src/every.h
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

// print position of each motor on each step if true
// If you see no steps, maybe you need the debug-jumper to fake limit switches?
// 0 = no step output
// 1 = step output on 100msec
// 2 = step output every step
// > 100 means print every n-msec
int DEBUGPOSPERSTEP = 0; // want changeable so processing can ask for steps

constexpr int MOTOR_CT = 15; // during startup, will auto detect and ignore motors whose limit is not triggered
constexpr boolean NO_EXTRA_FRAMES = false; // "true" turns off the led-indicator frame (extra shift-register).

Adafruit_NeoPixel builtin_neo(1,  40, NEO_GRB + NEO_KHZ800);
// indications for builtin neo
const uint32_t NEO_STATE_SETUP = Adafruit_NeoPixel::Color(255, 255, 0); // Yellow
const uint32_t NEO_STATE_FAULT = Adafruit_NeoPixel::Color(255, 0, 0); // Red
const uint32_t NEO_OFF = Adafruit_NeoPixel::Color(0, 0, 0); // off
const uint32_t NEO_STATE_LOOPING = Adafruit_NeoPixel::Color(0, 255, 0); // Green (blink)
const uint32_t NEO_STATE_UPLIMIT = Adafruit_NeoPixel::Color(255, 165, 0); // Orange (blink)

static boolean allow_random = true; // enable idle random animations

// NB: a #include list is magically done by arduino-ide of extant .h files
//  and it is alphabetical, so this order is not relevant
//  and, in fact, is redundant for arduino-ide
#include "freememory.h"
#include "every.h"
#include "array_size.h"
#include "ExponentialSmooth.h"
#include "AccelStepperShift.h"
#include "begin_run.h"
// #include "ArrayAnimation.h"
#include "AnimationWave1.h"
#include "AnimationTest.h"
#include "AnimationInteractive.h"

#include "Commands.h"

// if true: no initial move-to-uplimit, instead +|- move up 2, down 2
#define DEBUGMOVETEST 1
#ifndef DEBUGMOVETEST
#define DEBUGMOVETEST 0
#endif

constexpr int LATCH_PIN = 12;
constexpr int SH_LD_PIN = 11; // allow shift while high, load on low
constexpr int MOTOR_ENABLE_PIN = 10; // common stepper-driver enable
constexpr int FAKE_LIMIT_PIN = A5; // pull-up, so a jumper to A4 is "closed"
constexpr int FAKE_LIMIT_PIN_X = A4; // set to LOW, so a jumper is the "switch"

const int DISABLE_LIST[] = { -1}; /// { 5,6,9,11,0, -1 }; // 0..14, permanently not-use motors, end with -1

constexpr unsigned long ATTRACTOR_IDLE = 30UL * 1000UL; // pick an animation if we have been idle

// SYSTEMS
// Things that setup, and run during loop. See begin_run.h.
// We run them via systems[] (below)
// We need to explicitly refer to a few, so need explicit names.
// When testing/developing, you can set these to NULL to skip using them
AccelStepperShift* stepper_shift = new AccelStepperShift(
  MOTOR_CT,
  LATCH_PIN,
  SH_LD_PIN,
  MOTOR_ENABLE_PIN,
  DISABLE_LIST,
  FAKE_LIMIT_PIN,
  FAKE_LIMIT_PIN_X);
//ArrayAnimation* animation = new ArrayAnimation(MOTOR_CT); // later

AnimationWaveCute cute_wave_left( // the cute wave. sidedness is set in setup
  stepper_shift,
  5, // segments
  (AccelStepperShift::MAX_MOTION  / 2 ), // max amplitude
  0.2, // frequency
  1 // cycles
);

AnimationWaveCute cute_wave_right = cute_wave_left; // copy instance vars. sidedness is set in setup

// this list is only for serial commands (see Commands), to pick an animation.
Animation* Animation::animations[] = {
  new AnimationWave1( // '1' moving sine wave '!'
    stepper_shift,
    AccelStepperShift::MAX_MOTION / 2, // amplitude meters
    0.8, // wavelength fraction that fits in 1/2 of cleft
    0.1, // frequency
    1 // cycles
  ),
  new AnimationWave2( // the failed cute wave "2"
    stepper_shift,
    5, // segments
    AccelStepperShift::MAX_MOTION / 2, // max amplitude
    0.08, // frequency
    1 // cycles
  ),

  &cute_wave_left, // '3'

  NULL, NULL,
  new AnimationSequenceTests(stepper_shift), // '6' ([5])
  new AnimationIntegrationTests(stepper_shift), // '7'
  new AnimationMotorTests(stepper_shift), // '8'
  new AnimationFast(stepper_shift), // '9'

  // 'a'==[9] .. 'u' == [29]. intended for interactive animations decided by Pi
  // MUST CORRESPOND:
  // see lib/cleft.py choose_animation() and the constants above it
  new AnimationNoop(stepper_shift), // immediately goes to IDLE, 'a'
  new AnimationHome(stepper_shift), // aka RELAX 'b'
  NULL, // REAR_UP 'c'
  new AnimationJitter(stepper_shift, 0.02/*cm*/, 5000 /*msec*/), // JITTER 'd'
  new AnimationWornPosture( // WORN_POSTURE 'e'
    stepper_shift,
    0.10 /*m*/, 3000 /*msec*/, // initial posture
    0.02 /*m*/, 2000, 1000, 1000 /*msec*/ // breath, in,out,pause
  ),
  &cute_wave_left, // LEFT_WAVE 'f'
  &cute_wave_right, // RIGHT_WAVE 'g'
};
const int Animation::animation_ct = array_size(animations);
// 1st animation
Animation* Animation::current_animation = Animation::animations[2]; // cute first

// SYSTEMS
// We automatically call .begin() in setup, and .run() in loop, for each thing in systems[]
// When testing/developing, you can set these to NULL to skip using them
BeginRun* systems[] = {
  // all null: 9micros @ 8MHz 32u4
  stepper_shift,  // idle: 350micros @ 8MHz 32u4 1164 bytes used
  // runall: 300micros @48Mhz samd21 1620 used
  // idle: 100micros @48Mhz
  //// NB Animation::current_animation, NEVER goes in here. sad
  new Commands(stepper_shift),
};

void setup() {
  // We want to note memory
  const int base_memory = freeMemory();
  int last_free = base_memory;

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  serial_begin(115200); // cf. integrated usb vs not-integrated
  id();

  // Integrated USB chips have trouble uploading sometimes
  // (especially if you corrupt memory, or go into a tight loop)
  // So, give some time at startup to notice re-upload
  if (Serial) delay(2000); // allow uploading when cpu gets screwed

  digitalWrite(LED_BUILTIN, LOW);

  Serial << endl;
  Serial << F(__FILE__) << F(" ") << F(__DATE__) << F(" ") << F(__TIME__) << endl;
  // This prefix should be first, for memory/millis
  Serial << F("Start free ") << base_memory << endl;
  Serial << F("After Serial.begin ") << freeMemory() << endl;
  Serial << F("Clock ") << ((F_CPU / 1000.0) / 1000.0) << F(" MHz") << endl;

  builtin_neo.begin();
  // weirdly, the first set doesn't show
  builtin_neo.setBrightness(5); // too bright!
  builtin_neo.setPixelColor(0, 0, 0, 255) ;
  builtin_neo.show();
  builtin_neo.setPixelColor(0, NEO_STATE_SETUP );
  builtin_neo.show();

  // allow other spi users
  SPI.begin();

  cute_wave_right.right();
  cute_wave_left.left();

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
  if (! DEBUGPOSPERSTEP) Serial << F("NO STEP OUTPUT 'DEBUGPOSPERSTEP'") << endl;

  // FIXME? shouldn't I have currentanimation.begin() here?

  // Self callibrate to zero
  if (stepper_shift) stepper_shift->goto_limit(); // null-op if jumper is ignore-limits

  if (DEBUGMOVETEST) {
    Animation::current_animation->state = Animation::Off;
  }

  // and
  Serial << F("End setup() @ ") << millis() << F(" Free: ") << freeMemory() << endl;
}

void id() {
  Serial << F(__FILE__) << F(" ") << F(__DATE__) << F(" ") << F(__TIME__) << endl;
}

void serial_begin(unsigned long baud) {
  // Non-integrated USB (e.g. Uno) doesn't need a test of: `while (!Serial)`.
  // Integrated USB does (e.g. samd21 etc),
  //  * but `Serial` only goes true if usb is actually hooked up
  //  * and if the arduino gets hung (e.g. memory corruption), it may be very
  //    difficult to get it to respond to a new upload.
  //    SO: you should add a delay(2000) in your setup, early, so you can
  //    guarantee time to upload.
  Every fastblink(100);
  Timer serialtimeout(1500); // should be plenty of time

  Serial.begin(baud);
  // timeout if Serial never goes true (integrated USB & not connected).
  while (! ( Serial || serialtimeout() )) {
    if (fastblink()) digitalWrite(LED_BUILTIN, ! digitalRead(LED_BUILTIN));
  }
  if (Serial) {
    Serial << F("Serial ") << baud << endl;
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else {
    digitalWrite(LED_BUILTIN, LOW); // not that informative, heartbeat will quickly change this.
  }
}

void loop() {
  // track loop time (in microseconds)
  static Every say_status(1000);
  static ExponentialSmooth<unsigned long> elapsed(5);
  static Every start_idle(ATTRACTOR_IDLE);
  static unsigned long last_elapsed = 0;
  const unsigned long last_micros = micros(); // top of loop

  static Every debug(1000);
  if (debug()) {
    Serial << F("allow rand? ") << allow_random
           << F(" is running? ") << Animation::current_animation->is_running()
           << F(" state " ) << Animation::current_animation->state
           << endl;
  }

  heartbeat(NEO_STATE_LOOPING);

  // Run each object/module/system
  for (BeginRun* a_system : systems) {
    if ( a_system ) { // skip NULLs
      a_system->run();
    }
  }
  // bah, this is a BeginRun system, but I change animations, and so I would have to update systems[n]
  // FIXME
  if (Animation::current_animation) Animation::current_animation->run();

  // idle animation
  // wait some duration after !is_running (Timer)
  if ( Animation::current_animation->is_running() ) {
    start_idle.reset();
  }
  else if (allow_random && start_idle()) {
    const static int allowed[] = { 0, 2 }; // big wave & cute wave
    int which = allowed[ random( array_size( allowed ) ) ];

    Animation::current_animation = Animation::animations[which];
    Animation::current_animation->state = Animation::Restart;
    Serial << F("  RAND animation is ") << which << F(" ") << ((long) Animation::current_animation)
           << F(" @ ") << Animation::current_animation->state
           << endl;
  }

  // let a system clean up from the state of one loop
  // i.e. systems might check each other for a per-loop state
  // e.g. AccelStepperNote sets do_step if it wants a step this loop
  for (BeginRun* a_system : systems) {
    if ( a_system ) { // skip NULLs
      a_system->finish_loop();
    }
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
  // heartbeat for a particular color (which is the state)
  // NB: a global function, used by other classes!
  static Every::Toggle heartbeat(200);
  if ( heartbeat() ) {
    builtin_neo.setPixelColor(0, heartbeat.state ? NEO_OFF : color );
    builtin_neo.show();
  }
}

/*

  Left this here to remind me about the animation framework that isn't done yet.

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
