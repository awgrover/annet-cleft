/*
Visualizing the ir-camera
 1. use the Adafruit_AMG88xx examples/pixels_test
 
 To connect to the remote-pi (which is connected to the arduino):
 * remote-pi should be trying wifi for awgdart
 * laptop should be hotspot
 * check with ssh cleftpi.local
 * make pipe:
 mkfifo -m 0600 $THISDIR/remote-serial
 ssh cleftpi.local bin/cat-arduino > $THISDIR/remote-serial
 */

import java.util.StringTokenizer;

// Drawing areas
final int tv_width = 900;
final int tv_height = tv_width;
final int histo_x0 = tv_width + 1;
final int histo_width = 400;
final int histo_height = 200;
final int total_width = tv_width + Math.max(0, histo_width);
final int total_height =  Math.max(tv_width, histo_height);

float min_temp = 0; // celsius
float max_temp = 80;
final float RoomTemp = 22.5; // 21=70f
float firsthigh_bin_temp = 0.0; //celsius from arduino
int firsthigh_bin_i = 0;

// reading arduino data state:
boolean read_next_row = false;
int read_row_i = 0;
boolean new_data = false; // is temperatures[][] ready? reset on consume
float temperatures[][] = null;
int temp_cols = 0;
int temp_rows = 0;
boolean pixel_is_color = false; // false==temperature values

int histo[] = null;
boolean new_histo = false;

boolean mouse_down = false;
PVector mouse_start = new PVector();
PVector last_mouse = new PVector();


Serial arduino_port;
boolean arduino_tracking = false;
boolean arduino_reopen = true; // false to stop trying

// so I can map them to 1..9
final char[] shift_numbers = {'!', '@', '#', '$', '%', '^', '&', '*', '(', ')'};

Every check_arduino_port = new Every(500);
Every say_range = new Every(1000);

static int segment_i; // selected segment to operate on

void setup() {
  size(1200, 900);
  background(0);

  GlobalProcessing.P = this;

  println("Java " + System.getProperty("java.version"));
  println("I'm " + this.getClass().getName());
  println(" ... " + this.getClass().getPackage());
  fill(255);


  // black->red->orange->yellow 

  println("1..9,a..f select segment");
  println("up/down move segment");
  println("del resets heights");
  println("? starts tracking arduino");
  println("r closes/reopens arduino port");
  println("x closes arduino port");
  println("q synchronizes position with arduino");
  println("h tells arduino to home");
  println("u tells arduino all up to 0.5 meters");
  println("shift 1..9 (re)starts animation 1..9");

  connect_to_arduino();
}

void draw() {
  if (check_arduino_port.now()) {
    if (arduino_port == null && arduino_reopen) {
      connect_to_arduino();
      // sadly, no way to easily test if a serial-port disappeared
      // you could search Serial.list() for the port your opened
      // (an entry is a string, so save it, check)
    }
  }

  if ( new_data ) {
    draw_frame();
    draw_pixels();
    new_data = false;
  }

  if ( new_histo ) {
    draw_histo(); 
    new_histo = false;
  }
  /*background(90);
   
   fill(0, 0, 255);
   stroke(0, 90, 255);
   sphere(20);
   
   stroke(0);
   */


  if (mouse_down) {
    // only on change
    if (last_mouse.x != mouseX && last_mouse.y != mouseY) {
      last_mouse.set(mouseX, mouseY);
    }
  }



  // box(400);
  // draw and orient each segment
  fill(255);
}

void draw_histo() {
  int histo_max = 0;
  for (int i = 0; i < histo.length; i++) {
    if (histo_max < histo[i]) histo_max = histo[i];
  }

  float pixel_per_height = histo_height / histo_max;
  int bar_width = histo_width / histo.length; // pix per bar

  fill(255);

  for (int i = 0; i < histo.length; i++) {
    int bar_height = Math.round(pixel_per_height * histo[i]) ;

    // high temp island
    if (i == firsthigh_bin_i) {
      //println("DIV " + i + " == " + firsthigh_bin_i);
      fill(90, 255, 90); // blue
      rect(
        histo_x0 + i * bar_width, histo_height+20, // -bar_height,
        bar_width, -20
        );
    }
    if (firsthigh_bin_i > 0 && i > firsthigh_bin_i) {
      fill(255, 90, 90); // red
    } else {
      fill(255);
    }

    rect(
      histo_x0 + i * bar_width, histo_height, 
      bar_width, -bar_height 
      );
    //println("histo " + i + "=" + histo[i]);
    //println("   x " + (histo_x0 + i * bar_width));
  }
  //println("histo width " + histo_width + " histo.length " + histo.length);
  //println("Histo max " + histo_max + " bar width " + bar_width + " pix/height " + pixel_per_height);
}
void draw_frame() {
  // Draw an frames
  background(0, 0, 0);

  stroke(255, 255, 255);
  for (int y = 0; y <= tv_height; y += tv_height / temp_rows) {
    line(0, y, tv_width, y);
    //println("y " + y);
    for (int x = 0; x <= tv_width; x += tv_width / temp_cols) {
      line(x, 0, x, tv_height);
    }
  }
}

void connect_to_arduino() {
  if (arduino_port == null) {
    arduino_port = connectUSBSerial(115200);
    if (arduino_port != null) {
      arduino_port.buffer(1024);
      arduino_port.bufferUntil(10); // lf
      delay(500);
      arduino_port.write("?"); // evoke helo
      arduino_port.write("q"); // synch
    }
  }
}

void serialEvent(Serial port) {
  // event driven: each line that comes in,
  // so we have to maintain state. yech.

  try {
    String command = port.readString();

    if (command.startsWith("xy[")) { // xy[cols, rows ]
      setup_pixel_data(command);
    } else if (command.startsWith("histo[")) { // histo[bins,ct,ct,...]
      read_histo(command);
    } else if (command.startsWith("firsthigh[")) { // firsthigh[bin_i,temp_v]
      read_firsthigh(command);
    }// data looks like [f, f,... \n ...\n ] for 8 lines of 8 data
    else if (
      (command.startsWith("[") || command.startsWith("C[")) 
      && ! new_data 
      && temperatures != null
      ) {
      //print("* " + command);

      // color (hex) or temperature
      pixel_is_color = command.startsWith("C[");

      read_row_i = 0;
      if ( one_row(read_row_i, command) ) {
        read_next_row = true;
      } else {
        read_next_row = false;
      }
    } else if (read_next_row) {
      read_row_i += 1;
      if (command.startsWith("]")) {
        if (read_row_i == temp_rows) { // ] is on own line
          //println("* "+command + "<end data at "+(read_row_i+1)+">");
          new_data = true;
        } else {
          println("Wrong number of data rows ending at line "+read_row_i);
          println("* "+command);
        }
        read_next_row = false;
      } else if ( read_row_i >= temp_rows) {
        println("Too many data rows at line "+(read_row_i+1));
        println("* "+command);
      } else if ( one_row(read_row_i, command) ) {
        read_next_row = true;
      } else {
        // bad row
        read_next_row = false;
      }
    } else if (command.startsWith("<P ")) {
      // <P idx pos
      StringTokenizer tokens = new StringTokenizer(command);
      tokens.nextToken(); // discard "<P "

      // update our position
      String t;
      t= tokens.nextToken();
      //println("  token '" + t + "'");
      try {
        int i = Integer.parseInt( t );
        t= tokens.nextToken();
        //println("  token '" + t + "'");
        int steps = Integer.parseInt( t );
        float position = - steps;
      } 
      catch (NumberFormatException  e) {
        println("FAIL bad int from arduino " + t);
      }
      catch (Exception  e) {
        println("FAIL bad something from arduino " + e.toString());
      }
    } else {
      // print(":" + command);
    }
  }
  catch (Exception  e) {
    println("FAIL bad during serial-event " + e.toString());
    e.printStackTrace();
  }
}

void setup_pixel_data(String line) {
  String t = null;
  try {
    // from the xy[cols, rows ] line
    StringTokenizer tokens = new StringTokenizer(line, "[, \t\n\r\f");
    print("setup pixel data :"+line);
    t = tokens.nextToken(); // reads "xy"
    t = tokens.nextToken();
    println("  cols '"+t+"'");
    int cols = Integer.parseInt( t );
    t = tokens.nextToken();
    println("  rows '"+t+"'");
    int rows = Integer.parseInt( t );

    if (temperatures == null || temp_cols != cols || temp_rows != rows) {
      // old temperatures[][] gets gc'd
      temp_cols = cols;
      temp_rows = rows;
      temperatures = new float[cols][rows];
    }
  } 
  catch (NumberFormatException  e) {
    println("FAIL bad float from arduino '" + t + "'");
  }
  catch (Exception  e) {
    println("FAIL bad something from arduino " + e.toString());
  }
}

void read_histo(String line) {
  // histo[binct, ct,...]
  String t = null;
  try {
    // from the xy[cols, rows ] line
    StringTokenizer tokens = new StringTokenizer(line, "[, \t\n\r\f");
    print("histo :"+line);
    t = tokens.nextToken(); // reads "histo["
    t = tokens.nextToken(); // binct
    //println("  binct '"+t+"'");
    int binct = Integer.parseInt( t );
    if (histo == null || histo.length != binct) {
      histo = new int[binct];
    }

    // cts...
    for (int i=0; i<binct; i++) {
      t = tokens.nextToken();
      //println("  ct "+i+" '"+t+"'");
      int ct = Integer.parseInt( t );

      histo[i] = ct;
    }
    new_histo = true;
  } 
  catch (NumberFormatException  e) {
    println("FAIL bad int from arduino '" + t + "'");
  }
  catch (Exception  e) {
    println("FAIL bad something from arduino " + e.toString());
  }
}

void read_firsthigh(String line) {
  // firsthigh[bin_i, bin_low_temp]
  // which bin starts the high-temp island
  String t = null;
  try {
    // from the xy[cols, rows ] line
    StringTokenizer tokens = new StringTokenizer(line, "[], \t\n\r\f");
    print("firsthigh :"+line);
    t = tokens.nextToken(); // reads "firsthigh["
    t = tokens.nextToken(); // bin_i
    //println("  bin_i '"+t+"'");
    firsthigh_bin_i = Integer.parseInt( t );

    // temp
    t = tokens.nextToken();
    //println("  1st high '"+t+"'");
    firsthigh_bin_temp = Float.parseFloat( t );
  } 
  catch (NumberFormatException  e) {
    println("FAIL bad int/float from arduino '" + t + "'");
  }
  catch (Exception  e) {
    println("FAIL bad something from arduino " + e.toString());
  }
}
boolean one_row(int row_i, String line) {
  // the ir-camera is bottom-right 0,0
  // so "reflect"
  StringTokenizer tokens = new StringTokenizer(line, "[, \t\n\r\f");
  //println("row "+row_i+" ...");
  if (line.startsWith("C") ) tokens.nextToken(); // the C in C[

  for (int col_i=0; col_i < temp_cols; col_i++) {
    String t = tokens.nextToken();
    // println("  col "+col_i+" = "+t);
    try {
      float temp = Float.parseFloat( t );
      temperatures[temp_cols - 1 - col_i ][temp_rows - 1 - row_i ] = temp;
    } 
    catch (NumberFormatException  e) {
      println("FAIL bad float from arduino '" + t + "'");
      return false;
    }
    catch (Exception  e) {
      println("FAIL bad something from arduino " + e.toString());
      return false;
    }
  }
  //println("consumed row "+ row_i);
  return true;
}

void draw_pixels() {  
  float min_t =max_temp, max_t =min_temp;

  for (int y=0; y<temp_rows; y++) {
    for (int x=0; x<temp_cols; x++) {
      min_t = min(min_t, temperatures[x][y] );
      max_t = max(max_t, temperatures[x][y] );
    }
  }
  /*if (max_t - min_t < 4) {
   // at least 16..30
   min_t = min(min_t, 16.0);
   max_t = max(max_t, 30.0);
   } */
  if (say_range.now()) println("range " + min_t + " " + max_t + " color? " +pixel_is_color);

  int box_width = tv_width / temp_cols;
  int box_height = tv_height / temp_rows;
  for (int y=0; y<temp_rows; y++) {
    int y_top_left = box_height * y;
    for (int x=0; x<temp_cols; x++) {
      int x_top_left = box_width * x;

      if (pixel_is_color) {
        // using adafruit gfx colors:
        // 16 bit: 5bits red, 6bits green, 5bits blue
        // need to scale back up to 8bit
        int gfx_color = (int) temperatures[x][y];
        int red = (int) Math.pow(2, (8-5)) * ( (gfx_color >> 5+6) & 0x1f );
        int green = (int) Math.pow(2, (8-6)) * ( (gfx_color >> 6) & 0x3f);
        int blue = (int) Math.pow(2, (8-5)) * ( (gfx_color >> 0) & 0x1f);
        //if (y==12 && x == 12) println("C "+gfx_color+" -> "+red," ",blue," ",green);

        fill(red, green, blue);
        stroke(red, green, blue );
      } else {
        // non-interpolated, so my scheme
        float temp = temperatures[x][y];
        int red, green, blue;
        if (firsthigh_bin_i > 0 && temp >= firsthigh_bin_temp) {
          red = 255;
          green = int(map( temp, firsthigh_bin_temp, max_t, 0, 255));
          blue = green;
        } else if (firsthigh_bin_i > 0 && temp >= RoomTemp) {
          red = 255;
          blue = red;
          green = int(map( temp, RoomTemp, firsthigh_bin_temp, 0, 128));
        } else {
          blue = int(map( temp, min_t, firsthigh_bin_temp, 255, 0));
          green = 0;
          red = green;
        }
        fill(red, green, blue);
        stroke(red, green, blue);

        /*int brightness = Math.round( map(temp, min_t, max_t, 0.0, 255.0) );
         
         fill(brightness, brightness, brightness);
         stroke(brightness, brightness, brightness);
         */
      }

      int x_pos = x_top_left + 1;
      int y_pos = y_top_left + 1;
      rect(x_pos, y_pos, box_width-2, box_height-2);
      // print(x_pos+" "+y_pos+" "+brightness + " ");
    }
    //println();
  }
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
}
public void mouseClicked(MouseEvent evt) {
  if (evt.getCount() == 2) {
    // double-click
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
        arduino_tracking = true;
      } else {
        println("Not connected to arduino");
      }
    } else if (key == '-') {
      arduino_tracking = false;
    } else if (key == 'h') { 
      arduino_port.write("q0");
    } else if (key == 'u') { 
      arduino_port.write("qu");
    } else if (key == 'q') { 
      arduino_port.write("q");
    } else if (key == 'r') {
      if (arduino_port != null) { 
        arduino_port.stop();
        arduino_port = null;
      }
      arduino_reopen = true;
      connect_to_arduino();
    } else if (key == 'x') {
      if (arduino_port != null) { 
        arduino_port.stop();
        println("Disconnected");

        arduino_port = null;
        arduino_reopen = false;
      }
    } else if (int(key) == 127) { // DEL
    } else if (int(key) == 43) { // down
    } else if (int(key) == 45) { // up
    } else if (key >= 'A' && key <= 'Z') {
      println("Ard Command " + key);
      if (arduino_port != null) arduino_port.write(char(int(key)));
      return;
    } else {
      // detect the shift-numbers, convert to 1..n
      for (int i=0; i<shift_numbers.length; i++) {
        if (key == shift_numbers[i]) {
          println("Send animation " + char(int('1') + i));
          if (arduino_port != null) arduino_port.write(char(int('1') + i));
          return;
        }
      }

      println("what " + int(key) + " " + keyCode);
    }
  } else {
    // special keys, like arrows
    if (keyCode == 38) {
    } else if (keyCode == 40) {
      // downarrow
    } else {
      println("whatx " + int(key) + " " + keyCode);
    }
  }
}
