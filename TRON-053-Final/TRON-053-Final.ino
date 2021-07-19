
/***************************************************************************
  TRON Tech
  
  Uses the AMG88xx GridEYE 8x8 IR camera

  Designed specifically to work with the Adafruit AMG88 breakout
  ----> http://www.adafruit.com/products/3538

  These sensors use I2C to communicate. The device's I2C address is 0x69

  Written by Richard Jenkins edited by Jacob Lawler
  Copyright 2021, Richard Jenkins edited by Jacob Lawler
  This code is not opensource.
  
  IR Camera settup bassed on Writting of Dean Miller for Adafruit Industries.
  
 ***************************************************************************/

// Setuo for IR Camera
#include <Wire.h>
#include <Adafruit_AMG88xx.h>

Adafruit_AMG88xx amg;

float pixels[64];
float oldPixels[64]; // used to compare
int dif[64];   //difference of old and new pixels

//Setup for motors
#include<AccelStepper.h> //Stepper motor Library
AccelStepper stepper1(1, 4, 2);
AccelStepper stepper2(1 ,11, 7);


void setup() 
{
    // Setup for IR Camera
    Serial.begin(9600);
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

    //Setup for motors
    stepper1.setMaxSpeed(2500); //Steps per seconds
    stepper1.setAcceleration(1500); //Steps/sec^2

    stepper2.setMaxSpeed(2500); //Steps per seconds
    stepper2.setAcceleration(1500); //Steps/sec^2

}


void loop() 
{   
    /************************************
     *  IR Camera Image comparing  
     ************************************/
    // Read values of pixels array from camera
    amg.readPixels(pixels); //Function to read variable
    //int i; // Counter
    

//Serial.println(pixels[0]);
//Go through each value and subtract old image by new image to find the difference
    for (int i=0; i < 64; ++i)
    {
      dif[i] = pixels[i] - oldPixels[i]; // 0 is no change, positive is raised temperature of data point, and negative is drop in temperature.
      oldPixels[i] = oldPixels[i]+dif[i];
    }
    //dif[0] = pixels[0] - oldPixels[0]; 
    //oldPixels[0] = oldPixels[0]+dif[0];
    
    //Print difference array value for diagnosing
    Serial.print("[");
    for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
      Serial.print(dif[i-1]);
      Serial.print(", ");
      if( i%8 == 0 ) Serial.println();
    }
    Serial.println("]");
    Serial.println();
    //delay(1000);   //Delay 1 second readings
    
    /************************************
     * Motor output based on IR Camera reading
     ************************************/
    int i;
    int count = 0;
    for (i=0; i <= 63; ++i)
      {
        dif[i];
        count++;
        if (dif[i] > 1) //Doesnt acount for drop in temp because only works with positive difference values.
        {
          if(count>=48)
          {
            // if with 3 terms
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
             
          }
          else if(count>=16)
          {
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
          }

          else
          {
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
            Serial.println("Case 3");
            break;
          }
          
            
            Serial.println("  Movement detected");
            delay(20);
            break;
            }
        else
        {
          //Serial.print(count);
          Serial.println("  No Movement");
          delay(20);
        }
      }  
    }
    
