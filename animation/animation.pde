/*
Visualizing the animations
 1. framework for drawing a "cleft" visualization
 2. framework for "c++" like code of doing animations (minimal port work)
 */

static boolean mouse_down = false;
static PVector mouse_start = new PVector();
static PVector last_mouse = new PVector();
static PVector rotate = new PVector();
static PVector last_rotate = new PVector();

static Cleft cleft;
static int segment_i; // selected segment to operate on

void setup() {
  size(1200, 924, P3D);
  fill(255);

  cleft = new Cleft(this);

  println("1..9,a..f select segment");
  println("up/down move segment");
  println("del resets heights");
}

void draw() {
  background(90);
  
  fill(0,0,255);
  stroke(0,90,255);
  sphere(20);

  stroke(0);
  
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

  // set camera to current pan/tilt (from above)
  // "default" camera will have y pointing at us, z up, x to right
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


  cleft.render();
}

void mousePressed() { 
  mouse_down = true; 
  mouse_start.set(mouseX, mouseY);
  last_mouse.set(-1, -1);
  print("Startat "); 
  println(mouse_start);
}
void mouseReleased() { 
  mouse_down = false;
  last_rotate.set( rotate );
}
public void mouseClicked(MouseEvent evt) {
  if (evt.getCount() == 2) {
    // double-click
    rotate.set(0.0, 0.0);
    last_rotate.set(rotate);
  }
}

void keyPressed() {
  if (key != CODED) {
    // ascii's 1..9,a..e -> 0..14 
    if (key >= '0' && key <= '9') {
      segment_i = int(key) - int('1');
      println("select " + segment_i);
    } else if (key >= 'a' && key <= 'f') {
      segment_i = int(key) - int('a') + 10 - 1; // a is 10th which is [9]
      println("select " + segment_i);
    } else if (int(key) == 127) {
      cleft.reset();
    } else if (int(key) == 43) {
      cleft.move(segment_i, -10);
    } else if (int(key) == 45) {
      cleft.move(segment_i, 10);
    } else {
      println("what " + int(key) + " " + keyCode);
    }
  } else {
    // special keys, like arrows
    if (keyCode == 38) {
      cleft.move( segment_i, -1);
    } else if (keyCode == 40) {
      // downarrow
      cleft.move( segment_i, +1);
    } else {
      println("whatx " + int(key) + " " + keyCode);
    }
  }
}
