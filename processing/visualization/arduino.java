/*ard: 
 arduino.java, strip up to // Arduino, chomp final }, unindent
 blockcom+*x*+endblockcom strips comment
 final -> const
 */
import processing.core.PApplet;

class Arduino implements ArduinoMixin {
  // processing prefix
  final PApplet p;
  final PApplet Serial;
  Arduino() { 
    this.p=GlobalProcessing.P; 
    this.Serial = GlobalProcessing.P;
  }
  public long millis() {
    return p.millis();
  }

  // Arduino
  /**#include "array_animation.h"**/


  void setup() {
    Serial.print("Arduino Startup ");
    Serial.println(millis());

    last_time = millis();
    Serial.print("Animate ");
  }

  static ArrayAnimation anim = ArrayAnimation.goto_zero_plane();
  static /**unsigned**/ long last_time;
  void loop() {

    /**Positions**/
    float delta[] = new float[ArrayAnimation.SEGMENT_CT];

    // we hope our frame-length is close to actual
    if (all_motors.done()) {
      // if minimum frame rate
      anim.update( millis() - last_time, delta);
      last_time = millis();
      
      anim.add( delta );
      float motor_values[] = new float[ArrayAnimation.SEGMENT_CT];
      anim.scaled( StepsPerMeter, motor_values);
    }
  }
}
