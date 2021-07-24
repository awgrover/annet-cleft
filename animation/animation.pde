static boolean mouse_down = false;
static PVector mouse_start = new PVector();
static PVector last_mouse = new PVector();
static PVector rotate = new PVector();
static PVector last_rotate = new PVector();

static Cleft cleft;

void setup() {
  size(1200, 924, P3D);
  fill(255);
  
  cleft = new Cleft(this);

}

void draw() {
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
