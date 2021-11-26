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
int histo_height = 0;
final int total_width = tv_width + Math.max(0, histo_width);
final int total_height =  Math.max(tv_width, histo_height);

float min_temp = 0; // celsius
float max_temp = 80;
final float RoomTemp = 22.5; // 21=70f
float firsthigh_bin_temp = 0.0; //celsius from arduino
int firsthigh_bin_i = 0;
//ExponentialSmooth background_temp_i = new ExponentialSmooth(100.0);
int background_temp_i = 0;
float background_temp = 0.0;

class TempLocation {
  // to stucture some data together
  public float temp;
  public int i; // bin i
  public int x=0, y=0;
  TempLocation() { 
    this(0.0, -1, -1, -1);
  }
  TempLocation(float init_temp, int i, int x, int y) { 
    temp=init_temp;
    this.i=i;
    this.x=x;
    this.y=y;
  }
  public void read_data(String line, String why) {
    this.temp = 0;
    this.i = -1;
    this.x = -1;
    this.y = -1;
    //println(why + " " + line);
    StringTokenizer tokens = new StringTokenizer(line, "[, \t\n\r\f");
    tokens.nextToken(); // discard "blah["
    try {
      this.i = read_a_int(tokens, why+" .i");
      this.temp = read_a_float(tokens, why+" .temp");
      this.x = read_a_int(tokens, why+" .x");
      this.y = read_a_int(tokens, why+" .y");
    }
    catch (NumberFormatException  e) {
      // message already printed by read_a_x()
      this.temp = 0; // the flag
    }
  }
  String dump() {
    return String.format("[%d]=%.2f @(%d,%d)", this.i, this.temp, this.x, this.y);
  }
};

TempLocation low = new TempLocation();
TempLocation high = new TempLocation();

// reading arduino data state:
boolean read_next_row = false;
int read_row_i = 0;
boolean new_data = false; // is temperatures[][] ready? reset on consume
float temperatures[][] = null;
int temp_cols = 0;
int temp_rows = 0;
boolean pixel_is_color = false; // false==temperature values

int histo[] = null;
float histotemps[] = null;
boolean new_histo = false;

boolean mouse_down = false;
PVector mouse_start = new PVector();
PVector last_mouse = new PVector();


INonblockingReadLine arduino_port;
boolean arduino_tracking = false;
boolean arduino_reopen = true; // false to stop trying

// so I can map them to 1..9
final char[] shift_numbers = {'!', '@', '#', '$', '%', '^', '&', '*', '(', ')'};

Every check_arduino_port = new Every(500);
Every say_range = new Every(1000);

void setup() {
  size(1200, 900);
  background(0);
  histo_height = height;

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

  if (arduino_port != null) {
    String remote_line = arduino_port.readLine();
    if (remote_line != null) consume_ircam_data(remote_line);
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

  if (mouse_down) {
    // only on change
    if (last_mouse.x != mouseX && last_mouse.y != mouseY) {
      last_mouse.set(mouseX, mouseY);
    }
  }
}

void hot_pixels(TempLocation maximum) {
  // green circle == maximum-temp location
  int box_width = tv_width / temp_cols;
  int box_height = tv_height / temp_rows;
  int y_center = box_height * maximum.y + box_height/2;
  int x_center = box_width * maximum.x + box_width/2;

  stroke(0, 255, 255);
  fill(0, 255, 0);
  ellipse(x_center, y_center, box_width/2, box_height/2);

  int y_bottom = box_height * maximum.y + box_height;
  int y_top = box_height * maximum.y;
  int x_left = box_width * maximum.x;
  stroke(255, 255, 255);
  //strokeWeight(2);
  fill(0, 190, 190);
  textSize(box_height/4);
  String to_draw = "" + maximum.temp;
  int text_width = int(textWidth(to_draw));
  int text_height = int(textAscent() + textDescent());
  text("" + farenheight(maximum.temp), x_left + 2, y_top + text_height);
}


void draw_histo() {
  // draw horiz histo

  fill(0, 0, 0);
  rect( histo_x0, 0, histo_x0+histo_width, histo_height);

  int histo_max = 0;
  int histo_min = 1000;
  int max_temp_i = 0;
  for (int i = 0; i < histo.length; i++) {
    if (histo_max < histo[i]) histo_max = histo[i];
    if (histo_min > histo[i]) histo_min = histo[i];
    if (histo[i] > 0) max_temp_i = i;
  }

  float pixel_per_height = histo_width / (64/2); // only 64 pixels, so max ct==64
  int bar_width = histo_height / histo.length; // pix per bar

  fill(255);
  /*
  if (max_temp_i >= 2 ) {
   if (background_temp_i.to_int() == 0) {
   // initialize to the max to start
   // hopefully this is background, 
   background_temp_i.reset( max_temp_i );
   } else if ( background_temp_i.to_int() > max_temp_i ) {
   // quickly backoff if the max gets colder == background
   background_temp_i.reset( max_temp_i );
   } else {
   // track the max, assuming it is background
   // we want to stay below firsthigh_bin_i
   int to_heat_island = firsthigh_bin_i - background_temp_i.to_int();
   if ( to_heat_island >= 0 && to_heat_island <= 3 ) {
   background_temp_i.reset( firsthigh_bin_i - 2 );
   } else {
   // track the max
   background_temp_i.average( max_temp_i );
   }
   }
   }
   */

  for (int i = 0; i < histo.length; i++) {
    int bar_height = Math.round(pixel_per_height * histo[i]) ;
    float temp = histotemps== null ? 0.0 : histotemps[i];

    if ( i == background_temp_i) {
      fill(0, 0, 190); // blue
      rect(
        histo_x0, i * bar_width, 
        //histo_x0 + i * bar_width, histo_height+20, // -bar_height,
        histo_width * 0.7, bar_width
        //bar_width, -20
        );

      if (histotemps != null) {
        // text of first temp
        fill(255, 0, 255);
        textSize(bar_width * 5);

        text(""+farenheight(temp), histo_x0, i * bar_width + bar_width);
      }
    }

    if (i == firsthigh_bin_i) {
      // beginning of high-temp island
      //println("DIV " + i + " == " + firsthigh_bin_i);
      fill(90, 255, 90); // green
      rect(
        histo_x0, i * bar_width, 
        //histo_x0 + i * bar_width, histo_height+20, // -bar_height,
        histo_width * 0.7, bar_width
        //bar_width, -20
        );

      if (histotemps != null) {
        // text of first temp
        fill(255, 0, 255);
        textSize(bar_width * 5);

        text(""+farenheight(histotemps[firsthigh_bin_i]), histo_x0, i * bar_width + bar_width);
      }
    }


    if (firsthigh_bin_i > 0 && i >= firsthigh_bin_i) {
      // high temps
      if (histotemps != null && max_temp_i == i) {
        // text of first temp
        fill(0, 255, 255);
        textSize(bar_width * 5);
        text(""+(histotemps[i] * 9/5 + 32), histo_x0, i * bar_width + bar_width);
      }
      fill(255, 90, 90); // red
    } else {
      // low temps
      fill(255);
    }

    rect(
      histo_x0, i * bar_width, // histo_x0 + i * bar_width, histo_height, 
      bar_height, bar_width // bar_width, -bar_height 
      );
    //println("histo " + i + "=" + histo[i]);
    //println("   x " + (histo_x0 + i * bar_width));
  }
  //println("histo width " + histo_width + " histo.length " + histo.length);
  //println("Histo max " + histo_max + " bar width " + bar_width + " pix/height " + pixel_per_height);
}

float farenheight(float c) { 
  return c * 9/5 + 32;
}

void draw_frame() {
  // Draw an frames
  fill(0, 0, 0);
  rect(0, 0, tv_width, tv_height);

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
      delay(500);
      arduino_port.write("?"); // evoke helo
      arduino_port.write("q"); // synch
    }
  }
}

void x_serialEvent(Serial port) { // disabled
  // event driven: each line that comes in,
  // so we have to maintain state. yech.
  consume_ircam_data(port.readString());
}

void consume_ircam_data(String command) {
  // arduino_port is a INonblockingReadLine
  // from Serial (arduino) or a process (via ssh)
  try {
    if (command.startsWith("Traceback")) {
      println("Error from serial|port!");
      while (true) {
        String line = arduino_port.readLine();
        if (line != null) print(line);
      }
    } else if (command.startsWith("xy[")) { // xy[cols, rows ]
      setup_pixel_data(command);
    } else if (command.startsWith("histo[")) { // histo[bins,ct,ct,...]
      read_histo(command);
    } else if (command.startsWith("histotemps[")) { // histotemp[bins,t,t,...]
      read_histotemps(command);
    } else if (command.startsWith("firsthigh[")) { // firsthigh[bin_i,temp_v]
      read_firsthigh(command);
    } else if (command.startsWith("background[")) { // background[bin_i,temp_v]
      read_background(command);
    } else if (command.startsWith("mintemp[")) { // mintemp[bin_i,temp_v,x,y]
      low.read_data(command, "mintemp[");
      println("mintemp: " + low.dump());
    } else if (command.startsWith("maxtemp[")) { // mintemp[bin_i,temp_v,x,y]
      high.read_data(command, "maxtemp[");
      println("maxtemp: " + high.dump());
    } else if (command.startsWith("endstats[]")) { // end of stats
      // .clear seems to hang us sigh.
      // if (arduino_port != null) arduino_port.clear();
      println("QUE " + arduino_port.size());
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
      //print(read_row_i);
      if (command.startsWith("]")) {
        print(".");
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
    //println("  rows '"+t+"'");
    int rows = Integer.parseInt( t );

    if (temperatures == null || temp_cols != cols || temp_rows != rows) {
      // old temperatures[][] gets gc'd
      temp_cols = cols;
      temp_rows = rows;
      temperatures = new float[cols][rows];
    }
  } 
  catch (NumberFormatException  e) {
    println("FAIL bad float from arduino '" + t + "'" + " in '" + line + "'");
  }
  catch (Exception  e) {
    println("FAIL bad something from arduino " + e.toString());
  }
}

float read_a_float(StringTokenizer tokens, String why) {
  String t = null;  
  try {
    t = tokens.nextToken();
    return Float.valueOf( t );
  }
  catch (NumberFormatException  e) {
    println("FAIL bad float from serial|port during '"+why+"': '" + t + "'");
    throw e;
  }
  catch (Exception  e) {
    println("FAIL bad float from serial|port during '"+why+"': '" + t + "' "+ e.toString() );
  }
  return 0;
}
int read_a_int(StringTokenizer tokens, String why) {
  String t = null;
  try {
    t = tokens.nextToken();
    return Integer.valueOf( t );
  }
  catch (NumberFormatException  e) {
    println("FAIL bad int from serial|port during '"+why+"': '" + t + "'");
    throw e;
  }
  catch (Exception  e) {
    println("FAIL bad int from serial|port during '"+why+"': '" + t + "' "+ e.toString() );
  }
  return 0;
}
void read_histo(String line) {
  // histo[binct, ct,...]
  // cf read_histotemps()
  String t = null;
  try {
    // from the xy[cols, rows ] line
    StringTokenizer tokens = new StringTokenizer(line, "[, \t\n\r\f");
    println("histo :");
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
      int ct = Integer.valueOf( t );

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

void read_histotemps(String line) {
  // histotemps[binct, t,...]
  // cf read_histo. this is copypasta. sigh.
  String t = null;
  try {
    // from the xy[cols, rows ] line
    StringTokenizer tokens = new StringTokenizer(line, "[, \t\n\r\f");
    println("histotemp :");
    t = tokens.nextToken(); // reads "histo["
    t = tokens.nextToken(); // binct
    //println("  binct '"+t+"'");
    int binct = Integer.parseInt( t );
    if (histotemps == null || histotemps.length != binct) {
      histotemps = new float[binct];
    }

    // cts...
    for (int i=0; i<binct; i++) {
      t = tokens.nextToken();
      //println("  ct "+i+" '"+t+"'");
      float v = Float.valueOf( t );

      histotemps[i] = v;
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
    println("firsthigh :"+line);
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

void read_background(String line) {
  // background[bin_i, bin_temp]
  // which bin starts the high-temp island
  String t = null;
  try {
    // from the xy[cols, rows ] line
    StringTokenizer tokens = new StringTokenizer(line, "[], \t\n\r\f");
    println("background :"+line);
    t = tokens.nextToken(); // reads "background["
    t = tokens.nextToken(); // bin_i
    //println("  bin_i '"+t+"'");
    background_temp_i = Integer.parseInt( t );

    // temp
    t = tokens.nextToken();
    //println("  1st high '"+t+"'");
    background_temp = Float.parseFloat( t );
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
      temperatures[col_i ][row_i ] = temp;
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
  /*float min_t =max_temp;
   TempLocation maximum = new TempLocation(min_temp);
   
   for (int y=0; y<temp_rows; y++) {
   for (int x=0; x<temp_cols; x++) {
   min_t = min(min_t, temperatures[x][y] );
   if (maximum.temp < temperatures[x][y]) {
   maximum.temp = temperatures[x][y];
   maximum.x = x;
   maximum.y = y;
   }
   }
   }*/
  /*if (max_t - min_t < 4) {
   // at least 16..30
   min_t = min(min_t, 16.0);
   max_t = max(max_t, 30.0);
   } */
  if (say_range.now()) { 
    println("range " + low.temp + " " + high.temp + " color? " +pixel_is_color);
    if (firsthigh_bin_i == 0) println("No firsthigh");
  }

  int box_width = tv_width / temp_cols;
  int box_height = tv_height / temp_rows;
  for (int y=0; y<temp_rows; y++) {
    int y_top_left = box_height * y;
    for (int x=0; x<temp_cols; x++) {
      int x_top_left = box_width * x;
      boolean mark_background = false;

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
          // reds == higher than gap
          red = 255;
          green = int(map( temp, firsthigh_bin_temp, high.temp, 0, 255));
          blue = green;
        } else if (firsthigh_bin_i > 0 && temp >= RoomTemp) {
          // purples > arbitrary RoomTemp
          red = 255;
          blue = red;
          green = int(map( temp, RoomTemp, firsthigh_bin_temp, 0, 128));
        } else if (firsthigh_bin_i > 0) {
          // blues < gap
          blue = int(map( temp, low.temp, firsthigh_bin_temp, 255, 0));
          green = 0;
          red = green;
        } else {
          // light purple == no gap yet
          green = 0;
          red = 60;
          blue = red;
        }
        fill(red, green, blue);
        stroke(red, green, blue);

        //println("bkgnd i " + background_temp_i.to_int());
        mark_background = histotemps != null && ( temp <= background_temp );

        /*int brightness = Math.round( map(temp, low.temp, max_t, 0.0, 255.0) );
         
         fill(brightness, brightness, brightness);
         stroke(brightness, brightness, brightness);
         */
      }

      // fill a pixel block
      int x_pos = x_top_left + 1;
      int y_pos = y_top_left + 1;
      rect(x_pos, y_pos, box_width-2, box_height-2);
      // print(x_pos+" "+y_pos+" "+brightness + " ");

      if (mark_background) {
        background_pixel(
          x_top_left, y_top_left, 
          box_width, box_height 
          );
      }
    }
    //println();
  }

  hot_pixels(high);
}

void background_pixel(int x_top_left, int y_top_left, 
  int box_width, int box_height ) 
{
  // square box <= background temp
  stroke(180, 180, 180);
  int strokew = 4;
  strokeWeight(strokew);
  noFill();
  //rect(x_top_left+3, y_top_left+3, 
  //box_width-strokew-2, box_height-strokew-2);
  int inset = int(box_width*0.6);
  rect(x_top_left+inset, y_top_left+inset, 
    box_width-inset*2, box_height-inset*2);
  strokeWeight(1);
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
    } else if (key >= 'a' && key <= 'f') {
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
        arduino_port.close();
        arduino_port = null;
      }
      arduino_reopen = true;
      connect_to_arduino();
    } else if (key == 'x') {
      if (arduino_port != null) { 
        arduino_port.close();
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
