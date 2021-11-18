/*
  modified to work with my processing/ir_camera visualizer.

  Tried the adafruit i2c extender (https://www.adafruit.com/product/4756 (LTC4311 based):
    did not work: M0 -> extender -> 11' pots wire -> ir cam
    did work: M0 -> extender -> 11' pots wire -> muz -> 3" stemma/qwic -> ir cam
*/
/***************************************************************************
  This is a library for the AMG88xx GridEYE 8x8 IR camera

  This sketch makes an inetrpolated pixel thermal camera with the
  GridEYE sensor and a 2.4" tft featherwing:
	 https://www.adafruit.com/product/3315

  Designed specifically to work with the Adafruit AMG8833 Featherwing
          https://www.adafruit.com/product/3622

  These sensors use I2C to communicate. The device's I2C address is 0x69

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Dean Miller, James DeVito & ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/


#include <Wire.h>
#include <Adafruit_AMG88xx.h>

template <class I, class O>
O map(I x, I in_min, I in_max, O out_min, O out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


#include <Every.h>
#include <ExponentialSmooth.h>
#include "freememory.h"
#include <Streaming.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_I2C_Mux
QWIICMUX myMux;

#include "CollectStats.h"

//Comment this out to remove the text overlay
//#define SHOW_TEMP_TEXT

//low range of the sensor (this will be blue on the screen)
#define MINTEMP 20

//high range of the sensor (this will be red on the screen)
#define MAXTEMP 28

//the colors we will be using
const uint16_t camColors[] = {0x480F,
                              0x400F, 0x400F, 0x400F, 0x4010, 0x3810, 0x3810, 0x3810, 0x3810, 0x3010, 0x3010,
                              0x3010, 0x2810, 0x2810, 0x2810, 0x2810, 0x2010, 0x2010, 0x2010, 0x1810, 0x1810,
                              0x1811, 0x1811, 0x1011, 0x1011, 0x1011, 0x0811, 0x0811, 0x0811, 0x0011, 0x0011,
                              0x0011, 0x0011, 0x0011, 0x0031, 0x0031, 0x0051, 0x0072, 0x0072, 0x0092, 0x00B2,
                              0x00B2, 0x00D2, 0x00F2, 0x00F2, 0x0112, 0x0132, 0x0152, 0x0152, 0x0172, 0x0192,
                              0x0192, 0x01B2, 0x01D2, 0x01F3, 0x01F3, 0x0213, 0x0233, 0x0253, 0x0253, 0x0273,
                              0x0293, 0x02B3, 0x02D3, 0x02D3, 0x02F3, 0x0313, 0x0333, 0x0333, 0x0353, 0x0373,
                              0x0394, 0x03B4, 0x03D4, 0x03D4, 0x03F4, 0x0414, 0x0434, 0x0454, 0x0474, 0x0474,
                              0x0494, 0x04B4, 0x04D4, 0x04F4, 0x0514, 0x0534, 0x0534, 0x0554, 0x0554, 0x0574,
                              0x0574, 0x0573, 0x0573, 0x0573, 0x0572, 0x0572, 0x0572, 0x0571, 0x0591, 0x0591,
                              0x0590, 0x0590, 0x058F, 0x058F, 0x058F, 0x058E, 0x05AE, 0x05AE, 0x05AD, 0x05AD,
                              0x05AD, 0x05AC, 0x05AC, 0x05AB, 0x05CB, 0x05CB, 0x05CA, 0x05CA, 0x05CA, 0x05C9,
                              0x05C9, 0x05C8, 0x05E8, 0x05E8, 0x05E7, 0x05E7, 0x05E6, 0x05E6, 0x05E6, 0x05E5,
                              0x05E5, 0x0604, 0x0604, 0x0604, 0x0603, 0x0603, 0x0602, 0x0602, 0x0601, 0x0621,
                              0x0621, 0x0620, 0x0620, 0x0620, 0x0620, 0x0E20, 0x0E20, 0x0E40, 0x1640, 0x1640,
                              0x1E40, 0x1E40, 0x2640, 0x2640, 0x2E40, 0x2E60, 0x3660, 0x3660, 0x3E60, 0x3E60,
                              0x3E60, 0x4660, 0x4660, 0x4E60, 0x4E80, 0x5680, 0x5680, 0x5E80, 0x5E80, 0x6680,
                              0x6680, 0x6E80, 0x6EA0, 0x76A0, 0x76A0, 0x7EA0, 0x7EA0, 0x86A0, 0x86A0, 0x8EA0,
                              0x8EC0, 0x96C0, 0x96C0, 0x9EC0, 0x9EC0, 0xA6C0, 0xAEC0, 0xAEC0, 0xB6E0, 0xB6E0,
                              0xBEE0, 0xBEE0, 0xC6E0, 0xC6E0, 0xCEE0, 0xCEE0, 0xD6E0, 0xD700, 0xDF00, 0xDEE0,
                              0xDEC0, 0xDEA0, 0xDE80, 0xDE80, 0xE660, 0xE640, 0xE620, 0xE600, 0xE5E0, 0xE5C0,
                              0xE5A0, 0xE580, 0xE560, 0xE540, 0xE520, 0xE500, 0xE4E0, 0xE4C0, 0xE4A0, 0xE480,
                              0xE460, 0xEC40, 0xEC20, 0xEC00, 0xEBE0, 0xEBC0, 0xEBA0, 0xEB80, 0xEB60, 0xEB40,
                              0xEB20, 0xEB00, 0xEAE0, 0xEAC0, 0xEAA0, 0xEA80, 0xEA60, 0xEA40, 0xF220, 0xF200,
                              0xF1E0, 0xF1C0, 0xF1A0, 0xF180, 0xF160, 0xF140, 0xF100, 0xF0E0, 0xF0C0, 0xF0A0,
                              0xF080, 0xF060, 0xF040, 0xF020, 0xF800,
                             };

Adafruit_AMG88xx amg;
unsigned long delayTime;

#define AMG_COLS 8
#define AMG_ROWS 8
float pixels[AMG_COLS * AMG_ROWS];

#define INTERPOLATED_COLS 24
#define INTERPOLATED_ROWS 24


float get_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
void set_point(float *p, uint8_t rows, uint8_t cols, int8_t x, int8_t y, float f);
void get_adjacents_1d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
void get_adjacents_2d(float *src, float *dest, uint8_t rows, uint8_t cols, int8_t x, int8_t y);
float cubicInterpolate(float p[], float x);
float bicubicInterpolate(float p[], float x, float y);
void interpolate_image(float *src, uint8_t src_rows, uint8_t src_cols,
                       float *dest, uint8_t dest_rows, uint8_t dest_cols);

boolean draw_raw_data = false; // start in interpolate mode
int JumperGnd = A5;
int JumperRead = A4;

#define MUX -1 // which port or -1 for don't use

void setup() {
  while (!Serial) delay(100);
  Serial.begin(115200);
  //Serial << F("Free " ) << freeMemory() << endl;
  
  Serial.println("\n\nAMG88xx Interpolated Thermal Camera...");

  //Wire.setClock(100000);

  if (MUX != -1) {
    Wire.begin();
    if (myMux.begin()) {

      Serial.println("Mux detected");
      myMux.setPort(MUX); //Connect master to port labeled '1' on the mux
      Serial << F("  Using port 1 on mux") << endl;
      byte currentPortNumber = myMux.getPort();
      Serial.print("  CurrentPort: ");
      Serial.println(currentPortNumber);

    }
    else {
      Serial << F("No MUX") << endl;
      while (1) delay(100);
    }
  }
  
  // default settings
  if (!amg.begin()) {
    Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
    while (1) {
      delay(1);
    }
  }


  // 2 pins next to each other for jumper detect
  pinMode( JumperGnd, OUTPUT);
  digitalWrite( JumperGnd, LOW);
  pinMode( JumperRead, INPUT_PULLUP);

  Serial.println("-- Thermal Camera Test --");
  delay(10);
  Serial.println("go");
}

void loop() {
  static Every say_size(3 * 1000, true); // fire immediately

  static Every ir_framerate(100);
  static ExponentialSmooth<unsigned long> cam_rate = ExponentialSmooth<unsigned long>(5);
  static unsigned long last_loop = millis();
  static Every say_rate(1000, true);
  unsigned long start = millis();
  static float raw_pixels[AMG_COLS * AMG_ROWS]; // only static to save reallocating

  check_for_command();

  if (ir_framerate()) {
    //Serial << F("Free " ) << freeMemory() << endl;
    //Serial << F("Read...");
    // time it
    last_loop = millis();

    //read all the pixels
    constexpr int exp_smooth = 10;
    
    amg.readPixels(raw_pixels);
    for(int i=0; i< AMG_COLS * AMG_ROWS; i++) {
      pixels[i] = (exp_smooth-1)*(pixels[i]/exp_smooth) + raw_pixels[i]/exp_smooth;
    }
  
    //Serial << F("...read") << endl;

    if (draw_raw_data) {
      //Serial << F("raw...") << endl;
      if (say_size()) {
        Serial << "xy[" << AMG_COLS << ", " << AMG_ROWS << " ]" << endl;
      }
      //Serial << F("draw...") << endl;
      draw_pixels(pixels, AMG_COLS, AMG_ROWS);
    }
    else {
      //Serial << F("interp...") << endl;
      if (say_size()) {
        Serial << "xy[" << INTERPOLATED_COLS << ", " << INTERPOLATED_ROWS << " ]" << endl;
      }

      static float dest_2d[INTERPOLATED_ROWS * INTERPOLATED_COLS];

      int32_t t = millis();
      interpolate_image(pixels, AMG_ROWS, AMG_COLS, dest_2d, INTERPOLATED_ROWS, INTERPOLATED_COLS);
      Serial.print("Interpolation took "); Serial.print(millis() - t); Serial.println(" ms");

      draw_interpolation_pixels(dest_2d, INTERPOLATED_ROWS, INTERPOLATED_COLS);
    }

  }
  if (say_rate()) Serial << F("loop ") << (millis() - start) << endl;
}

void check_for_command() {
  static Every jumper_check(500, true);
  if (jumper_check()) {
    draw_raw_data = ! digitalRead(JumperRead);
  }

  if (Serial.available() > 0) {
    Serial << F("ser read...");
    char cmd = Serial.read();
    Serial << F("...read") << endl;
  }
}

void draw_pixels(float * p, uint8_t rows, uint8_t cols) {
  // raw data as ints (not color)
  static CollectStats collect_stats;
  static Every say_stats(1000);
  static Every slow(100);
  //Serial << F("Free " ) << freeMemory() << endl;
  //Serial << F("in draw..") << endl;
  
  if (slow()) {
    float last_min = collect_stats.min_v;
    float last_max = collect_stats.max_v;
    //Serial << F("slow...") << endl;
    
    collect_stats.reset();

    Serial.print("[");
    for (int y = 0; y < rows; y++) {
      for (int x = cols - 1; x >= 0; x--) { // 0..cols will give mirrored left/right effect
        float temp = pixels[ x  + y * rows ] ;
        collect_stats.value( temp );
        //if (temp <= last_max - 1) temp = 0;
        Serial << temp;
        Serial.print(", ");
      }
      Serial.println();

    }
    Serial << "]" << endl;
  }

  if (say_stats()) {
    Serial << F("minmax[") << collect_stats.min_v << F(",") << collect_stats.max_v << F("]") << endl;
    collect_stats.print_histo();
  }
}

void draw_interpolation_pixels(float * p, uint8_t rows, uint8_t cols) {
  int colorTemp;

  Serial.print("C[");
  for (int y = 0; y < rows; y++) {
    for (int x = 0; x < cols; x++) {
      float val = get_point(p, rows, cols, x, y);
      if (val >= MAXTEMP) colorTemp = MAXTEMP;
      else if (val <= MINTEMP) colorTemp = MINTEMP;
      else colorTemp = val;

      uint8_t colorIndex = map(colorTemp, MINTEMP, MAXTEMP, 0, 255);
      colorIndex = constrain(colorIndex, 0, 255);
      //draw the pixels!
      Serial.print(camColors[colorIndex]);
      Serial.print(", ");

    }
    Serial.println();

  }
  Serial << "]" << endl;
}
