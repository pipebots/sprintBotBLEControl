/*Robot code for NANO 33 BLE Sense to pair with Robot controller App
* Nick Fry 2019
* Change robotName[] below for each board
* Temp code currently commented out as may be useful for different job later
* This version is to use the Joystick app so allows for speed control
*/

#include <Arduino.h>
#include <ArduinoBLE.h>
#include <rgbLED.h>

void readButtons();
void readJoystick();
void driveMotor(int pwmPin, int dirPin, int spd);
void joyDiffDrive(int nJoyX, int nJoyY);

// Motor driver inputs
#define ML_DIR            8
#define ML_PWM            9
#define MR_DIR            10
#define MR_PWM            11


char robotName[] = "BigBallBot"; //Device Name - will appear as BLE descripton when connecting

BLEService inputService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE Service for all inputs from app
BLEService outputService("1809"); //BLE Temperature service

// BLE LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic buttonCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEByteCharacteristic joystickCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEFloatCharacteristic tempChar("2A1C", BLERead | BLEIndicate); //
BLEDescriptor tempCharDescript("2901", "Temperature");

const int ledPin = LED_BUILTIN; // pin to use for the LED
byte tempArray[] = {};
float temp = 0.0;
float humidity = 0.0;
bool gesture = 0;
long previousMillis = 0;  // last time the temperature was checked, in ms
int xybyte = 0;
int ynib = 0;
int xnib = 0;

union {
    float tempfval;
    byte tempbval[4];
} floatAsBytes;


void setup() {
  Serial.begin(9600);
//  while (!Serial); //This stops the program running until the serial monitor is opened!

  // set LED pin to output mode
  pinMode(ledPin, OUTPUT);

  // Set motor driver pins to ouput mode
  pinMode(ML_DIR, OUTPUT);
  pinMode(ML_PWM, OUTPUT);
  pinMode(MR_DIR, OUTPUT);
  pinMode(MR_PWM, OUTPUT);


  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  // set advertised local name and service UUID:
  BLE.setLocalName(robotName);
  BLE.setAdvertisedService(inputService);
  BLE.setAdvertisedService(outputService);

  // add the characteristic to the service
  inputService.addCharacteristic(buttonCharacteristic);
  inputService.addCharacteristic(joystickCharacteristic);
  outputService.addCharacteristic(tempChar);
  tempChar.addDescriptor(tempCharDescript);

  // add service
  BLE.addService(inputService);
  BLE.addService(outputService);

  // set the initial value for the characeristic:
  buttonCharacteristic.writeValue(0);
  joystickCharacteristic.writeValue(0);
  tempChar.writeValue(0);

  // start advertising
  BLE.advertise();

  Serial.println("BLE LED Peripheral");
}



void loop() {
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    // while the central is still connected to peripheral:
    while (central.connected()) {

    /*  long currentMillis = millis();
      // if 500ms have passed, check the temperaturemeasurement:
      if (currentMillis - previousMillis >= 500) {
        previousMillis = currentMillis;
        updateTemp();
      }
      */
      readButtons();
      readJoystick();
    }


    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
}

void readButtons(){
  //most of these buttons no longer exist in app
  // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      if (buttonCharacteristic.written()) {
        Serial.println(buttonCharacteristic.value());
        switch (buttonCharacteristic.value()) {
          case 0:
            Serial.println("LED off");
            digitalWrite(ledPin, LOW);          // will turn the LED off
            //offLED();
            break;
          case 1:
            Serial.println("LED on");
            digitalWrite(ledPin, HIGH);          // will turn the LED off
            break;
          case 2:
            Serial.println("FWD");
            //greenLED();
            driveMotor(ML_PWM, ML_DIR, 255);
            driveMotor(MR_PWM, MR_DIR, 255);
            break;
          case 3:
            Serial.println("Back");
            //blueLED();
            driveMotor(ML_PWM, ML_DIR, -255);
            driveMotor(MR_PWM, MR_DIR, -255);
            break;
          case 4:
            Serial.println("Left");
            //cyanLED();
            driveMotor(ML_PWM, ML_DIR, -255);
            driveMotor(MR_PWM, MR_DIR, 255);
            break;
          case 5:
            Serial.println("Right");
            //magentaLED();
            driveMotor(ML_PWM, ML_DIR, 255);
            driveMotor(MR_PWM, MR_DIR, -255);
            break;
          case 6:
            Serial.println("Stop");
              //redLED();
            driveMotor(ML_PWM, ML_DIR, 0);
            driveMotor(MR_PWM, MR_DIR, 0);
            break;
          case 7:
            Serial.println("Fwd Left");
            //rgbLED(100,255,100);
            driveMotor(ML_PWM, ML_DIR, 0);
            driveMotor(MR_PWM, MR_DIR, 255);
            break;
          case 8:
            Serial.println("Fwd Right");
            //rgbLED(255,100,100);
            driveMotor(ML_PWM, ML_DIR, 255);
            driveMotor(MR_PWM, MR_DIR, 0);
            break;
          case 9:
            Serial.println("Back Left");
            //rgbLED(255,50,100);
            driveMotor(ML_PWM, ML_DIR, 0);
            driveMotor(MR_PWM, MR_DIR, -255);
            break;
          case 10:
            Serial.println("Back Right");
            //rgbLED(50,100,100);
            driveMotor(ML_PWM, ML_DIR, -255);
            driveMotor(MR_PWM, MR_DIR, 0);
            break;
          case 11:
            Serial.println("Gesture Control");
            //yellowLED();
            gesture = !gesture; //flip bool
            Serial.println("Gesture");
            break;
         default:
             Serial.println("Error - no cases match");
            // whiteLED();
             break;
        }
       }
}

void readJoystick(){
    int xchange, ychange;

    if (joystickCharacteristic.written()) {
        //split byte back to two nibbles
        //right X=15, left X=0, up Y=0, down y=15
       //Serial.println(joystickCharacteristic.value());
       xybyte = joystickCharacteristic.value() & 0xFF;
       ynib = xybyte & 0xF;
       xnib = xybyte >> 4;
      Serial.print("x: ");
      Serial.print(xnib);
      Serial.print(" y: ");
      Serial.println(ynib);
      xchange = map(xnib, 0, 15, -127, 127);
      ychange = map(ynib, 0, 15, 127, -127);
      Serial.print("xchange: ");
      Serial.print(xchange);
      Serial.print(" ychange: ");
      Serial.println(ychange);
      joyDiffDrive(xchange, ychange);
      }
}

void joyDiffDrive(int nJoyX, int nJoyY){
  // Based upon:
  // Differential Steering Joystick Algorithm
  // ========================================
  //   by Calvin Hass
  //   https://www.impulseadventure.com/elec/robot-differential-steering.html
  //
  // Converts a single dual-axis joystick into a differential
  // drive motor control, with support for both drive, turn
  // and pivot operations.
  //

  // INPUTS
  //int     nJoyX;              // Joystick X input                     (-128..+127)
  //int     nJoyY;              // Joystick Y input                     (-128..+127)

  // OUTPUTS
  int     nMotMixL;           // Motor (left)  mixed output           (-128..+127)
  int     nMotMixR;           // Motor (right) mixed output           (-128..+127)

  // CONFIG
  // - fPivYLimt  : The threshold at which the pivot action starts
  //                This threshold is measured in units on the Y-axis
  //                away from the X-axis (Y=0). A greater value will assign
  //                more of the joystick's range to pivot actions.
  //                Allowable range: (0..+127)
  float fPivYLimit = 32.0;

  // TEMP VARIABLES
  float   nMotPremixL;    // Motor (left)  premixed output        (-128..+127)
  float   nMotPremixR;    // Motor (right) premixed output        (-128..+127)
  int     nPivSpeed;      // Pivot Speed                          (-128..+127)
  float   fPivScale;      // Balance scale b/w drive and pivot    (   0..1   )


  // Calculate Drive Turn output due to Joystick X input
  if (nJoyY >= 0) {
    // Forward
    nMotPremixL = (nJoyX>=0)? 127.0 : (127.0 + nJoyX);
    nMotPremixR = (nJoyX>=0)? (127.0 - nJoyX) : 127.0;
  } else {
    // Reverse
    nMotPremixL = (nJoyX>=0)? (127.0 - nJoyX) : 127.0;
    nMotPremixR = (nJoyX>=0)? 127.0 : (127.0 + nJoyX);
  }

  // Scale Drive output due to Joystick Y input (throttle)
  nMotPremixL = nMotPremixL * nJoyY/128.0;
  nMotPremixR = nMotPremixR * nJoyY/128.0;

  // Now calculate pivot amount
  // - Strength of pivot (nPivSpeed) based on Joystick X input
  // - Blending of pivot vs drive (fPivScale) based on Joystick Y input
  nPivSpeed = nJoyX;
  fPivScale = (abs(nJoyY)>fPivYLimit)? 0.0 : (1.0 - abs(nJoyY)/fPivYLimit);

  // Calculate final mix of Drive and Pivot
  nMotMixL = (1.0-fPivScale)*nMotPremixL + fPivScale*( nPivSpeed);
  nMotMixR = (1.0-fPivScale)*nMotPremixR + fPivScale*(-nPivSpeed);

  // Convert to Motor PWM range
  driveMotor(ML_PWM, ML_DIR, nMotMixL*2); //*2 to convert from -127..+127 to -254..+254
  driveMotor(MR_PWM, MR_DIR, nMotMixR*2);
}

void driveMotor(int pwmPin, int dirPin, int spd){ //input speed -255 to +255, 0 is stop
  // Make sure the speed is within the limit.
  if (spd > 255) {
    spd = 255;
  } else if (spd < -255) {
    spd = -255;
  }

  if (spd >= 0){
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, spd);
  }
  else{
    digitalWrite(dirPin, HIGH);
    analogWrite(pwmPin, -spd);
  }
}
