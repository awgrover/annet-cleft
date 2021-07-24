import processing.core.PShape;
import processing.core.PApplet;

class Cleft {
  PShape segment;

  PApplet p;

  static final int SEGMENT_CT=15;
  // Height of the outer edge of each segment
  // 0 = "center", so, we can go +/-
  float height[] = new float[SEGMENT_CT];

  // segment info:
  // sizes like "1.0" means this many render "pixels"
  static final float scale_up = 100f;
  static final float depth = 6.0f  * scale_up;

  Cleft(PApplet processing) { 
    this.p=processing; 
    make_segment();

    for (float h : height ) {
      h=0.0f;
    }
  }
  void render() {
    p.pushMatrix();
    // we start with inner-edge on x axis, in the xy plane, centered at x=0
    float rotation = (float) Math.atan( height[0] / depth );
    // calculate our rotateX from height
    p.rotateX( rotation ); // so the inner-edge is still at z=0, because it was on xy
    p.shape(segment);
    p.popMatrix();
  }

  void make_segment() {
    segment = p.createShape();
    segment.beginShape();

    // I think we draw "faces"?
    // seems to like counterclockwise for each? (if math xy orientation)

    segment.fill(255);
    segment.stroke(128);

    // segment.scale(10);
    // xy plane quad shape
    final float inner_width = 1.0f * scale_up;
    final float outer_width = 2.5f  * scale_up;

    segment.vertex( (inner_width/2), 0, 0);
    segment.vertex(-(inner_width/2), 0, 0);
    segment.vertex(-(outer_width/2), -depth, 0);
    segment.vertex( (outer_width/2), -depth, 0);
    // a translate here changes the coord for a glob trans/rot
    //segment.translate(50,100);
    // movement has to be converted to rotateX
    /*
   segment.vertex( -0.5, 0, 0);
     
     segment.vertex( -0.5, 0, 1);
     segment.vertex( 0.5, 0, 1);
     segment.vertex( 0.75, 4, 1);
     segment.vertex( -0.75, 4, 1);
     segment.vertex( -0.5, 1, 0);
     */
    /*
  segment.vertex(-100, -100, 0);
     segment.vertex( 100, -100, 0);
     segment.vertex( 100, 100, 0);
     segment.vertex(-100, 100, 0);
     segment.vertex(-100, -100, 0);
     
     segment.vertex(-100, -100, 20);
     segment.vertex( 100, -100, 20);
     segment.vertex( 100, 100, 20);
     segment.vertex(-100, 100, 20);
     segment.vertex(-100, -100, 20);
     */
    segment.endShape(p.CLOSE);
  }
}
