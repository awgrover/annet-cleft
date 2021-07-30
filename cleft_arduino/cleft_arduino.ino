/*

Motor shield has 5 i2c address pins: base 0x60 + pin setting.

*/

// prefix
#include "awg_combinators/serial.h"

constexpr int SEGMENT_CT=7;
using Heights = float[SEGMENT_CT];

// Our bits
#include "AccelStepperMotorShield.h"
#include "ExponentialSmooth.h"
#include "every.h"
#include "freememory.h"
#include "array_animation.h"

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
  static ArrayAnimation anim; // = ArrayAnimation.goto_zero_plane();

  // note framerate
  static Every say_looprate(500);
  static ExponentialSmooth<unsigned long> framerate(1);
  static unsigned long last_loop_time=millis();
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
    float fr = 1000.0/framerate.value();
    Serial << F("looprate ") << fr << F(" free ") << freeMemory() << endl;
  }

}

void pheights( Heights delta ) {
  for (int i=0; i<SEGMENT_CT; i++) {
    Serial << delta[i] << F(" ");
  }
}

