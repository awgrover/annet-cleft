//**************************************************************//
//  Name    : shiftOutCode, Hello World
//  Author  : Carlyn Maw,Tom Igoe, David A. Mellis
//  Date    : 25 Oct, 2006
//  Modified: 23 Mar 2010
//  Version : 2.0
//  Notes   : Code for using a 74HC595 Shift Register           //
//          : to count from 0 to 255
//****************************************************************
#include <Streaming.h>

//Pin connected to ST_CP of 74HC595
int latchPin = 12;
//Pin connected to SH_CP of 74HC595
int clockPin = SCK; // sck
////Pin connected to DS of 74HC595
int dataPin = MOSI; // mosi
void setup() {
  Serial.begin(115200); while (!Serial);
  Serial << endl << F("Begin") << endl;

  //set pins to output so you can control the shift register
  pinMode(latchPin, OUTPUT);
  digitalWrite(latchPin, LOW);

  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  digitalWrite(dataPin, LOW);
}
void loop() {
  // lsb is dir
  // feel/sound is odd: steps on high, settles/locks on low
  // 0b01, 0b11, 0b01 steps on [1],[2] ccw
  // 0b00, 0b10, 0b00 steps on [1],[2] cw
  static const byte pattern[] = { 0b01, 0b11, 0b01 }; // step|dir bits AA = 10 55 = 01
  for (byte p : pattern) {

    Serial << _HEX(p) << F(" ") << _BIN(p & 0b11) << endl;

    // shift out the bits:
    for (int x = 0; x < 8; x++) {
      shiftOut(dataPin, clockPin, MSBFIRST, p);
    }
    // toggle latch
    digitalWrite(latchPin, HIGH); delay(1); digitalWrite(latchPin, LOW);
    // pause before next value:
    //delay(500);
  }
  Serial << F("---") << endl;
  
}
