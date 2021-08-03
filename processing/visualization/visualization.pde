/*
Visualizing the animations
 1. framework for drawing a "cleft" visualization
 2. framework for "c++" like code of doing animations (minimal port work)
 */

boolean mouse_down = false;
PVector mouse_start = new PVector();
PVector last_mouse = new PVector();
PVector last_rotate = new PVector(0, -PI/8, 0); // start tilted
PVector rotate = new PVector();

Cleft cleft;
Serial arduino_port;

Every check_arduino_port = new Every(500);

static int segment_i; // selected segment to operate on

void setup() {
  size(1200, 924, P3D);

  GlobalProcessing.P = this;

  println("Java " + System.getProperty("java.version"));
  println("I'm " + this.getClass().getName());
  println(" ... " + this.getClass().getPackage());
  fill(255);

  rotate.set( last_rotate );

  cleft = new Cleft();
  for (int i=0; i<cleft.height.length; i++) cleft.move(i, random(-1.0, 1.0));
  println(cleft.height);

  println("1..9,a..f select segment");
  println("up/down move segment");
  println("del resets heights");
  println("? asks arduino for hello");
  println("R closes/reopens arduino port");

  connect_to_arduino();
}

void draw() {
  if (check_arduino_port.now()) {
    if (arduino_port == null) {
      connect_to_arduino();
    // sadly, no way to easily test if a serial-port disappeared
    // you could search Serial.list() for the port your opened
    // (an entry is a string, so save it, check)
    }
  }

  background(90);

  fill(0, 0, 255);
  stroke(0, 90, 255);
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
  final float orbitRadius= width + Cleft.depth;
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

  // print("CH ");println(cleft.height);
  cleft.render();
}

void connect_to_arduino() {
  if (arduino_port == null) {
    arduino_port = connectUSBSerial(115200);
    if (arduino_port != null) {
      arduino_port.bufferUntil(10); // lf
      delay(500);
      arduino_port.write("?"); // evoke helo
    }
  }
}

void serialEvent(Serial port) {
  String command = port.readString();
  if (! command.startsWith("*") ) print("* " + command);
}

void mousePressed() { 
  mouse_down = true; 
  mouse_start.set(mouseX, mouseY);
  last_mouse.set(-1, -1);
  //print("Startat "); 
  //println(mouse_start);
}
void mouseReleased() { 
  mouse_down = false;
  last_rotate.set( rotate );
}
public void mouseClicked(MouseEvent evt) {
  if (evt.getCount() == 2) {
    // double-click
    rotate.set(0.0, -PI/8, 0.0);
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
    } else if (key == '?') {
      if (arduino_port != null) { 
        arduino_port.write('?');
      }
      } else if (key == 'R') {
      if (arduino_port != null) { 
        arduino_port.stop();
        arduino_port = null;
        connect_to_arduino();
      }
    } else if (int(key) == 127) {
      cleft.reset();
    } else if (int(key) == 43) {
      cleft.move(segment_i, -0.1);
    } else if (int(key) == 45) {
      cleft.move(segment_i, 0.1);
    } else {
      println("what " + int(key) + " " + keyCode);
    }
  } else {
    // special keys, like arrows
    if (keyCode == 38) {
      cleft.move( segment_i, -0.01);
    } else if (keyCode == 40) {
      // downarrow
      cleft.move( segment_i, +0.01);
    } else {
      println("whatx " + int(key) + " " + keyCode);
    }
  }
}
