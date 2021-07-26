/*

Motor shield has 5 i2c address pins: base 0x60 + pin setting.

*/

// prefix
#include "awg_combinators/serial.h"

// Our bits
#include "AccelStepperMotorShield.h"
#include "ExponentialSmooth.h"
#include "every.h"
#include "freememory.h"

constexpr int SEGMENT_CT=7;
using Heights = float[SEGMENT_CT];

#include "all_motors.h"
AllMotors all_motors;

void setup() {
  Serial.begin(115200); while(!Serial);
  Serial << F("Startup ") << millis() << F(" free ") << freeMemory() << endl;


  all_motors.begin();
  Serial << F("Animation ") << millis() << F(" free ") << freeMemory() << endl;
}


void loop() {
  static unsigned long last_time = millis();
  Heights delta;

  // note framerate
  static Every say_looprate(500);
  static ExponentialSmooth<unsigned long> framerate(1);
  static unsigned long last_loop_time=millis();
  static Every animation_update(1000 / 10); // 10 frames/sec maximum

  Serial << F("run/update") << endl;
  // if ( all_motors.run() || animation_update() ) { // i.e. done-running or time for next frame
  {
    // update motors every frame
    }
  Serial << F("did run/update") << endl;

  framerate.average( millis() - last_loop_time );
  last_loop_time = millis();
  if ( say_looprate() ) {
    float fr = 1000.0/framerate.value();
    Serial << F("framerate ") << fr << F(" free ") << freeMemory() << endl;
  }

    Serial << F(" free ") << freeMemory() << endl;
  while(1);
}
