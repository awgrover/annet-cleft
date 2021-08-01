import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class animation extends PApplet {

static boolean mouse_down = false;
static PVector mouse_start = new PVector();
static PVector last_mouse = new PVector();
static PVector rotate = new PVector();
static PVector last_rotate = new PVector();

static PShape segment;

public void setup() {
  
  fill(255);

  make_segment();
}

public void draw() {
  background(90);
  fill(255);

  // x,y,z indicators: green, red, gray=z
  pushMatrix();
  fill(0, 255, 0);
  translate(300, 0, 0);
  box(20);
  popMatrix();
  pushMatrix();
  fill(255, 0, 0);
  translate(0, 300, 0);
  box(20);
  popMatrix();
  pushMatrix();
  fill(90, 90, 90);
  translate(0, 0, 300);
  box(20);
  popMatrix();



  //translate(width/2, height/2, 0);

  if (mouse_down) {
    // only on change
    if (last_mouse.x != mouseX && last_mouse.y != mouseY) {
      last_mouse.set(mouseX, mouseY);
      rotate.x = last_rotate.x + (mouseX - mouse_start.x) / width;
      rotate.y = last_rotate.y + (mouse_start.y - mouseY) / height;
      // print("mouse "); 
      // print(last_mouse); 
      // print(" * ");
      // println(rotate);
    }
  }

  // default" camera will have y pointing at us, z up, x to right
  float orbitRadius= width;
  float ypos= sin(TWO_PI * rotate.x + PI/2)*orbitRadius;
  ; // mouseY/3;
  //float xpos= cos(radians(rotation))*orbitRadius;
  float xpos= cos(TWO_PI * rotate.x + PI/2)*orbitRadius;
  float zpos= cos(TWO_PI * rotate.y + PI/2)*orbitRadius;
  ; // sin(TWO_PI * rotate.x)*orbitRadius;
  camera(xpos, ypos, zpos, 0, 0, 0, 0, 0, -1);

  // box(400);
  // draw and orient each segment
  fill(255);

  // order doesn't matter
  translate(50,100);
  rotateX( PI/8); // so the inner-edge is still at z=0, because it was on xy
  shape(segment);
}

public void mousePressed() { 
  mouse_down = true; 
  mouse_start.set(mouseX, mouseY);
  last_mouse.set(-1, -1);
  print("Startat "); 
  println(mouse_start);
}
public void mouseReleased() { 
  mouse_down = false;
  last_rotate.set( rotate );
}
public void mouseClicked(MouseEvent evt) {
  if (evt.getCount() == 2) {
    // double-click
    rotate.set(0.0f, 0.0f);
    last_rotate.set(rotate);
  }
}

public void make_segment() {
  segment = createShape();
  segment.beginShape();

  // I think we draw "faces"?
  // seems to like counterclockwise for each? (if math xy orientation)

  segment.fill(255);
  segment.stroke(128);


  // segment.scale(10);
  // xy plane quad shape
  float scale_up = 100;
  float inner_width = 1.0f * scale_up;
  float outer_width = 2.5f  * scale_up;
  float depth = 6.0f  * scale_up;
  float height = 2.0f;
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
  segment.endShape(CLOSE);
}
  public void settings() {  size(1200, 924, P3D); }
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "animation" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
