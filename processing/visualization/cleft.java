import processing.core.PShape;
import processing.core.PApplet;

class Cleft {
  PShape segment;
  boolean first_time = true;

  final PApplet p;

  static final int SEGMENT_CT=15;
  static final int in_circle = 15;
  static final float STEPS_METER = 7.2f;// * 200; // fixme: get from arduino

  // Height of the outer edge of each segment
  // 0 = "center", so, we can go +/-
  float height[] = new float[SEGMENT_CT];

  // segment info:
  // use meters, then scale by pixels.
  // sizes like "1.0" means this many render "pixels"
  static final float scale_up = 600f;
  static final float inner_width = 0.1f * scale_up;
  static final float outer_width = 0.3f  * scale_up;
  static final float depth = 1.0f  * scale_up;
  static final float spacing = 0.05f * scale_up; // gap between
  static final float cleft_radius = 0.5f * scale_up;

  Cleft() { 

    this.p=GlobalProcessing.P; 
    make_segment();
    reset();
    p.println("I'm " + this.getClass().getName());
    p.println(" ... " + this.getClass().getPackage());
  }

  void reset() {
    first_time = true;
    for (int i=0; i<height.length; i++) height[i] = 0.0f;
  }

  void render() {
    // 11 around a half circle
    // 2 off each end straight out

    
    // around circle
    // with a gap of 3
    final float interval = (2*p.PI)/(in_circle - 1 + 3)  ; // gap 3
    final int per_tip = (SEGMENT_CT - in_circle) / 2;
    for (int segment_i=per_tip; segment_i<in_circle+per_tip; segment_i++) {
      p.pushMatrix();
      p.rotateZ((segment_i-per_tip) * interval - p.PI/2 - 2.75f * interval);
      p.translate( 0, - cleft_radius  );
      set_to_height( segment_i );
      p.shape(segment);
      if (first_time) p.println("draw " + segment_i);
      p.popMatrix();
    }


    first_time = false;
  }

  void set_to_height(int segment_i) {
    // we start with inner-edge on x axis, in the xy plane, centered at x=0
    // calculate our rotateX from height
    float rotation = (float) Math.atan( height[segment_i] / depth );
    p.rotateX( rotation ); // so the inner-edge is still at z=0, because it was on xy
  }

  void goTo(int segment_i, float h) {
    if (segment_i >= 0 && segment_i < SEGMENT_CT) {
      height[segment_i] = h * scale_up;
    } else {
      p.println("bad segment_i " + segment_i);
    }
  }

  void move(int segment_i, float distance) {
    if (segment_i >= 0 && segment_i < SEGMENT_CT) {
      height[segment_i] += distance * scale_up;
    } else {
      p.println("bad segment_i " + segment_i);
    }
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
