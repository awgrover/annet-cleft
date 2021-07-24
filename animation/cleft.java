import processing.core.PShape;
import processing.core.PApplet;

class Cleft {
  PShape segment;
  boolean first_time = true;

  PApplet p;

  static final int SEGMENT_CT=15;
  static final int in_circle = 11; // leaving rest for tips
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

  Cleft(PApplet processing) { 
    this.p=processing; 
    make_segment();
    reset();
  }

  void reset() {
    first_time = true;
    for (float h : height ) {
      h=0.0f;
    }
  }

  void render() {
    // 11 around a half circle
    // 2 off each end straight out

    // tips: straight out
    // [0,1, 13,14]
    for (int tip_edge=0; tip_edge<=1; tip_edge++) {
      // going clockwise, "left"/first tip is 0
      // i.e. [0,1] is mirror of [14,13]
      for (int i=0; i<=1; i++) {
        int segment_i = tip_edge == 0 ? 1-i : 13 + i;
        float tip_direction = tip_edge == 0 ? -1 : 1;
        p.pushMatrix();
        p.rotateZ(tip_direction * p.PI/2.0f); // inner-edge on y
        p.translate( tip_direction * (i * (outer_width + spacing) + outer_width + spacing ), - cleft_radius  );
        set_to_height( segment_i );
        p.shape(segment);
        if (first_time) p.println("draw " + segment_i);
        p.popMatrix();
      }
    }

    // around circle
    final float interval = p.PI/(in_circle - 1)  ; // used 4 in tips
    final int per_tip = (SEGMENT_CT - in_circle) / 2;
    for (int segment_i=per_tip;segment_i<in_circle+per_tip; segment_i++) {
      p.pushMatrix();
      p.rotateZ((segment_i-per_tip) * interval - p.PI/2);
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
  void move(int segment_i, float distance) {
    if (segment_i >= 0 && segment_i < SEGMENT_CT) {
      height[segment_i] += distance;
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
