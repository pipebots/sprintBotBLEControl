// Copyright 2019,2020,2025 University of Leeds.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
// the Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
// FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
// IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

// RGB LED Functions for inbuild LED on the Arduino Nano 33 Sense
#include <Arduino.h>

//pins for build in led on nano 33 sense
#define R_LED_PIN         22
#define G_LED_PIN         23
#define B_LED_PIN         24

void setupRGBLED(){
  pinMode(R_LED_PIN, OUTPUT);
  pinMode(G_LED_PIN, OUTPUT);
  pinMode(B_LED_PIN, OUTPUT);
}
void greenLED(){
    analogWrite(R_LED_PIN,UINT8_MAX);
    analogWrite(G_LED_PIN,0);
    analogWrite(B_LED_PIN,UINT8_MAX);
}
void blueLED(){
    analogWrite(R_LED_PIN,UINT8_MAX);
    analogWrite(G_LED_PIN,UINT8_MAX);
    analogWrite(B_LED_PIN,0);
}
void redLED(){
    analogWrite(R_LED_PIN,0);
    analogWrite(G_LED_PIN,UINT8_MAX);
    analogWrite(B_LED_PIN,UINT8_MAX);
}
void yellowLED(){
    analogWrite(R_LED_PIN,0);
    analogWrite(G_LED_PIN,0);
    analogWrite(B_LED_PIN,UINT8_MAX);
}
void magentaLED(){
    analogWrite(R_LED_PIN,0);
    analogWrite(G_LED_PIN,UINT8_MAX);
    analogWrite(B_LED_PIN,0);
}
void cyanLED(){
    analogWrite(R_LED_PIN,UINT8_MAX);
    analogWrite(G_LED_PIN,0);
    analogWrite(B_LED_PIN,0);
}
void offLED(){
    analogWrite(R_LED_PIN,UINT8_MAX);
    analogWrite(G_LED_PIN,UINT8_MAX);
    analogWrite(B_LED_PIN,UINT8_MAX);
}
void whiteLED(){
    analogWrite(R_LED_PIN,0);
    analogWrite(G_LED_PIN,0);
    analogWrite(B_LED_PIN,0);
}
void rgbLED(int r,int g,int b){
    analogWrite(R_LED_PIN,r);
    analogWrite(G_LED_PIN,g);
    analogWrite(B_LED_PIN,b);
}
