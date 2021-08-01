// supposed to be c++ poly-linquistic for arduino
import processing.core.PApplet;


class ArrayAnimation implements  ArduinoMixin {
  /**prefix */
  public long millis() {
    return p.millis();
  }
  
  /**public:**/
  static final int SEGMENT_CT = 15;
  float shape[] = new float[SEGMENT_CT]; // actually "height"
  final PApplet p;
  final PApplet Serial;

  ArrayAnimation() {
    this.p=GlobalProcessing.P; 
    this.Serial = GlobalProcessing.P;

    for (float x : shape) x = 0.0f;
  }

  // Patterns
  static ArrayAnimation /**&**/ goto_zero_plane() {
    // default init
    ArrayAnimation /**&**/ a = new ArrayAnimation();
    return a;
  }
};
