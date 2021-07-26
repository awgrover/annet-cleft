/***************************************************************************
  TRON Tech

  Designed for the UNO.

  Uses the AMG88xx GridEYE 8x8 IR camera:

  Designed specifically to work with the Adafruit AMG88 breakout
  ----> http://www.adafruit.com/products/3538
  Library "Adafruit AMG88xx Library" , dependencies not required

  These sensors use I2C to communicate. The device's I2C address is 0x69

  Written by Richard Jenkins edited by Jacob Lawler
  Copyright 2021, Richard Jenkins, Jacob Lawler
  This code is not opensource.

  IR Camera settup bassed on Writting of Dean Miller for Adafruit Industries.

 ***************************************************************************/

// Setuo for IR Camera
#include <Adafruit_AMG88xx.h> // "Adafruit AMG88xx Library", dependencies not required

// comment out to do "random" motion w/o needing the camera:
// #define USE_AMG88
// comment out to use "native"/bit-bang AccelStepper pins
#define USE_MOTORSHIELD

Adafruit_AMG88xx amg;

float oldPixels[64]; // used to compare
int dif[64];   //difference of old and new pixels

//Setup for motors
#include <AccelStepper.h> //Stepper motor Library
#ifdef USE_MOTORSHIELD
// this is using the motor shield(s)
#include <AccelStepperMotorShield.h>
Adafruit_MotorShield MotorShield1 = Adafruit_MotorShield(0x61); // shield #1
  // Make it just like a AccelStepper
AccelStepperMotorShield stepper1(
                                  MotorShield1,
                                  1, // stepper block #2
                                  SINGLE // optional/default, the stepperBlock.step-style
                                );
AccelStepperMotorShield stepper2(
                                  MotorShield1,
                                  2, // stepper block #2
                                  SINGLE // optional/default, the stepperBlock.step-style
                                );
#else
// this is "direct" bit-banging, i.e. native AccelStepper
AccelStepper stepper1(1, 4, 2);
AccelStepper stepper2(1 , 11, 7);
#endif

void setup() {
  // Setup for IR Camera
  Serial.begin(9600);

#ifdef USE_AMG88
  Serial.println(F("AMG88xx pixels"));
  bool status;
  // default settings
  status = amg.begin();
  if (!status) {
    Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
    while (1);
  }

  Serial.println("-- Pixels Test --");

  Serial.println();

  delay(100); // let sensor boot up
#else
  Serial.println("Random 'zone' instead of camera\n");
#endif

  //Setup for motors
#ifdef USE_MOTORSHIELD
  stepper1.begin();
#endif
  stepper1.setMaxSpeed(2500); //Steps per seconds
  stepper1.setAcceleration(1500); //Steps/sec^2

  stepper2.setMaxSpeed(2500); //Steps per seconds
  stepper2.setAcceleration(1500); //Steps/sec^2

  Serial.println("stepper1");
  stepper1.move(50);
  stepper1.move(-50);
  Serial.println("stepper2");
  stepper2.move(50);
  stepper2.move(-50);
}


void loop() {
#ifdef USE_AMG88
  // use ir-camera:
  diff_image( oldPixels, dif);
#else
  // Don't require ir-camera, do some random motion:
  int fake_temp = 16 * random(3); // 0..2
  dif[0] = dif[16] = dif[48] = 0;
  dif[0] = 2; // has to be > 1
#endif

  /************************************
     Motor output based on IR Camera reading
   ************************************/
  // if temp increases: in top third [else], mid-third >=[16], bottom-third >=[48]
  // run the pattern for that third
  int count = 0;
  for (int i = 0; i <= 63; ++i) {
    count++;
    if (dif[i] > 1) { //Doesnt acount for drop in temp because only works with positive difference values.
      if (count >= 48) {
        // if with 3 terms
        Serial.print(millis()); Serial.print(" ");
        Serial.println("Case 1");
        stepper1.move(6500);
        stepper1.runToPosition();
        delay(10);
        stepper1.move(-6500);
        stepper1.runToPosition();
        delay(10);

        stepper2.move(6500);
        stepper2.runToPosition();
        delay(10);
        stepper2.move(-6500);
        stepper2.runToPosition();
        delay(10);
        Serial.print(millis()); Serial.print(" done\n");
        break;

      }
      else if (count >= 16) {
        Serial.print(millis()); Serial.print(" ");
        Serial.println("Case 2");
        stepper1.move(4000);
        stepper1.runToPosition();
        delay(10);
        stepper1.move(-4000);
        stepper1.runToPosition();
        delay(10);
        stepper2.move(4000);
        stepper2.runToPosition();
        delay(10);
        stepper2.move(-4000);
        stepper2.runToPosition();
        delay(10);
        Serial.print(millis()); Serial.print(" done\n");
        break;
      }

      else {
        Serial.print(millis()); Serial.print(" ");
        Serial.println("Case 3");
        stepper1.move(1500);
        stepper1.runToPosition();
        delay(10);
        stepper1.move(-1500);
        stepper1.runToPosition();
        delay(10);
        stepper2.move(1500);
        stepper2.runToPosition();
        delay(10);
        stepper2.move(-1500);
        stepper2.runToPosition();
        delay(10);
        Serial.print(millis()); Serial.print(" done\n");
        break;
      }


      Serial.println("  Movement detected");
      delay(20);
      break;
    }
    else {
      //Serial.print(count);
      Serial.println("  No Movement");
      delay(20);
    }
  }
}

void diff_image( float oldPixels[], int dif[]) {
  float pixels[64];

  /************************************
      IR Camera Image comparing
   ************************************/
  // Read values of pixels array from camera
  amg.readPixels(pixels); //Function to read variable

  //Serial.println(pixels[0]);
  //Go through each value and subtract old image by new image to find the difference
  for (int i = 0; i < 64; ++i) {
    dif[i] = pixels[i] - oldPixels[i]; // 0 is no change, positive is raised temperature of data point, and negative is drop in temperature.
    oldPixels[i] = oldPixels[i] + dif[i];
  }

  //Print difference array value for diagnosing
  Serial.print("[");
  for (int i = 1; i <= AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    Serial.print(dif[i - 1]);
    Serial.print(", ");
    if ( i % 8 == 0 ) Serial.println();
  }
  Serial.println("]");
  Serial.println();
  //delay(1000);   //Delay 1 second readings
}
