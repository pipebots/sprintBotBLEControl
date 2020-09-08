/*Robot code for NANO 33 BLE Sense to pair with Robot controller App
* Nick Fry 2019
* Change robotName[] below for each board
* Temp code currently commented out as may be useful for different job later
* This version is to use the Joystick app so allows for speed control
*
* (Encoder 2 is rght hand side)
*/

#include <Arduino.h>
#include <ArduinoBLE.h>
#include <rgbLED.h>
#include <PID_v1.h>
#include <ArduinoJson.h>


void readButtons();
void readJoystick();
void driveMotor(int pwmPin, int dirPin, int spd);
void joyDiffDrive(int nJoyX, int nJoyY);
void calcPID();
void doEncoder1();
void doEncoder2();
void calcSpeed();
void sendJSON();
void listenJSON();
void rampMotor1();
void rampMotor2();
void limitSpeed();

// Motor driver inputs
#define ML_DIR 8
#define ML_PWM 9
#define MR_DIR 10
#define MR_PWM 11

//encoder pins
#define ENC_1_A 2
#define ENC_1_B 3
#define ENC_2_A 4
#define ENC_2_B 5

volatile int encoder1Ticks, encoder2Ticks, wheel1Pos, wheel2Pos, wheel1Revs, wheel2Revs = 0;

char robotName[] = "BigBallBot"; //Device Name - will appear as BLE descripton when connecting

BLEService inputService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE Service for all inputs from app
BLEService outputService("1809"); //BLE Temperature service

// BLE LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic buttonCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEByteCharacteristic joystickXCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEByteCharacteristic joystickYCharacteristic("19B10234-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEFloatCharacteristic speedCharacteristic("19B10234-E8F2-537E-4F6C-D104768A1333", BLERead | BLEWrite);
BLEFloatCharacteristic tempChar("2A1C", BLERead | BLEIndicate); //
BLEDescriptor tempCharDescript("2901", "Temperature");

const int ledPin = LED_BUILTIN; // pin to use for the LED
byte tempArray[] = {};
float temp = 0.0;
float humidity = 0.0;
bool gesture = 0;
long previousMillis,  prevMillis1, prevMillis2 = 0;
int encoder1Prev, encoder2Prev = 0;

//params for ramping motor speed
bool ramp_flag_1, ramp_flag_2 = false;
int pid_ramp_1, pid_ramp_2 = 0;
int ramp_inc = 5;
int ramp_delay = 20;
float speedLimit = 0.6;

//setup PID controllers
double PID_SET_1, PID_SET_2, PID_IN_1, PID_IN_2, PID_OUT_1, PID_OUT_2 = 0;
/*More agressive tuning */
float kp = 0.3994062511991154;
float ki = 9.477300047940169;
float kd = 0.00841618143827802;

/*Slower accel
float kp = 0.12751539947693455;
float ki = 3.025745585973288;
float kd = 0.0026869703089283047; */


/* small overshoot
float kp = 0.5735857456094395;
float ki = 13.610313303915005;
float kd = 0.012086445044277552;*/

PID motorPID1(&PID_IN_1, &PID_OUT_1, &PID_SET_1, kp, ki, kd, DIRECT);
PID motorPID2(&PID_IN_2, &PID_OUT_2, &PID_SET_2, kp, ki, kd, DIRECT);

void setup() {
  Serial.begin(9600);
//  while (!Serial); //This stops the program running until the serial monitor is opened!

  //set PID ranges to -255 to 255
  motorPID1.SetOutputLimits(-255,255);
  motorPID2.SetOutputLimits(-255,255);
  motorPID1.SetSampleTime(20); //defualt is 200ms
  motorPID2.SetSampleTime(20);
  //turn on PIDs
  motorPID1.SetMode(AUTOMATIC);
  motorPID2.SetMode(AUTOMATIC);

  // set LED pin to output mode
  pinMode(ledPin, OUTPUT);
  setupRGBLED();
  // Set motor driver pins to ouput mode
  pinMode(ML_DIR, OUTPUT);
  pinMode(ML_PWM, OUTPUT);
  pinMode(MR_DIR, OUTPUT);
  pinMode(MR_PWM, OUTPUT);
  //setup encoder pins
  pinMode(ENC_1_A, INPUT);
  pinMode(ENC_1_B, INPUT);
  pinMode(ENC_2_A, INPUT);
  pinMode(ENC_2_B, INPUT);
  digitalWrite(ENC_1_A, HIGH);  //turn on pullup resistor
  digitalWrite(ENC_1_B, HIGH);  //turn on pullup resistor
  digitalWrite(ENC_2_A, HIGH);  //turn on pullup resistor
  digitalWrite(ENC_2_B, HIGH);  //turn on pullup resistor

  // encoder 1 channel on interrupt 0 (Arduino's pin 2)
  attachInterrupt(digitalPinToInterrupt(2), doEncoder1, RISING);
  // encoder 2
  attachInterrupt(digitalPinToInterrupt(4), doEncoder2, RISING);
  //(Due to gear ratio we dont need to track every pulse so just using interrupt on one channel Rising.)

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
  inputService.addCharacteristic(joystickXCharacteristic);
  inputService.addCharacteristic(joystickYCharacteristic);
  inputService.addCharacteristic(speedCharacteristic);
  outputService.addCharacteristic(tempChar);
  tempChar.addDescriptor(tempCharDescript);

  // add service
  BLE.addService(inputService);
  BLE.addService(outputService);

  // set the initial value for the characeristic:
  buttonCharacteristic.writeValue(0);
  joystickXCharacteristic.writeValue(127);
  joystickYCharacteristic.writeValue(127);
  speedCharacteristic.writeValue(0.6); //make match app slider default value
  tempChar.writeValue(0);

  // start advertising
  BLE.advertise();

  Serial.println("BLE LED Peripheral");

  Serial.println(ramp_inc);
  Serial.println(ramp_delay);
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

    long currentMillis = millis();
      // if 10ms have passed, check the speed
      if (currentMillis - previousMillis >= 20) {
        previousMillis = currentMillis;
        calcSpeed();
      }
      currentMillis = millis();
      if (currentMillis - prevMillis2 >= ramp_delay){
        rampMotor1();
        rampMotor2();
        prevMillis2 = currentMillis;
      }
      readButtons();
      readJoystick();
      calcPID();
      //sendJSON();
      listenJSON();
      limitSpeed();
      Serial.print(currentMillis);
      Serial.print(", Set 1:");
      Serial.print(PID_SET_1);
      Serial.print(", In 1:");
      Serial.print(PID_IN_1);
      Serial.print(" Out 1: ");
      Serial.println(PID_OUT_1);

/*
      Serial.print("Encoder count 1: ");
      Serial.print(wheel1Pos);
      Serial.print(" Wheel Rev 1: ");
      Serial.println(wheel1Revs);

      Serial.print("Encoder count 2: ");
      Serial.print(wheel2Pos);
      Serial.print(" Wheel Rev 2: ");
      Serial.println(wheel2Revs);
  */
    }


    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
    driveMotor(ML_PWM, ML_DIR, 0); //stop robot if bluetooth disconnects
    driveMotor(MR_PWM, MR_DIR, 0);
    ramp_flag_1 = false;
    ramp_flag_2 = false;
  }
}

void readButtons(){
  //most of these buttons no longer exist in app
  // if the remote device wrote to the characteristic,
      // use the value to control the LED:
      if (buttonCharacteristic.written()) {
        //Serial.println(buttonCharacteristic.value());
        switch (buttonCharacteristic.value()) {
          case 0:
            //Serial.println("LED off");
            digitalWrite(ledPin, LOW);          // will turn the LED off
            //offLED();
            break;
          case 1:
            //Serial.println("LED on");
            digitalWrite(ledPin, HIGH);          // will turn the LED off
            break;
          case 2:
          //  Serial.println("FWD");
            //greenLED();
            //PID_SET_1 = 255;
            //PID_SET_2 = 255;
            pid_ramp_1 = 255*speedLimit;
            pid_ramp_2 = 255*speedLimit;
            ramp_flag_1 = true;
            ramp_flag_2 = true;

            break;
          case 3:
            //Serial.println("Back");
            //blueLED();
            pid_ramp_1 = -255*speedLimit;
            pid_ramp_2 = -255*speedLimit;
            ramp_flag_1 = true;
            ramp_flag_2 = true;
            break;
          case 4:
            //Serial.println("Left");
            //cyanLED();
            pid_ramp_1 = 255*speedLimit;
            pid_ramp_2 = -255*speedLimit;
            ramp_flag_1 = true;
            ramp_flag_2 = true;
            break;
          case 5:
            //Serial.println("Right");
            //magentaLED();
            pid_ramp_1 = -255*speedLimit;
            pid_ramp_2 = 255*speedLimit;
            ramp_flag_1 = true;
            ramp_flag_2 = true;
            break;
          case 6:
            //Serial.println("Stop");
              //redLED();
              pid_ramp_1 = 0;
              pid_ramp_2 = 0;
              ramp_flag_1 = true;
              ramp_flag_2 = true;
            break;
          case 7:
            //Serial.println("Fwd Left");
            //rgbLED(100,255,100);
            pid_ramp_1 = 255*speedLimit;
            pid_ramp_2 = 0;
            ramp_flag_1 = true;
            ramp_flag_2 = true;
            break;
          case 8:
            //Serial.println("Fwd Right");
            //rgbLED(255,100,100);
            pid_ramp_1 = 0;
            pid_ramp_2 = 255*speedLimit;
            ramp_flag_1 = true;
            ramp_flag_2 = true;
            break;
          case 9:
            //Serial.println("Back Left");
            //rgbLED(255,50,100);
            pid_ramp_1 = -255*speedLimit;
            pid_ramp_2 = 0;
            ramp_flag_1 = true;
            ramp_flag_2 = true;
            break;
          case 10:
            //Serial.println("Back Right");
            //rgbLED(50,100,100);
            pid_ramp_1 = 0;
            pid_ramp_2 = -255*speedLimit;
            ramp_flag_1 = true;
            ramp_flag_2 = true;
            break;
          case 11:
            //Serial.println("Gesture Control");
            //yellowLED();
            gesture = !gesture; //flip bool
            //Serial.println("Gesture");
            break;
          case 12:
            //Serial.println("E-STOP");
            ramp_flag_1 = false;
            ramp_flag_2 = false;
            BLE.disconnect(); //this also turns off motors
            //WARNING - Robot needs hard resetting to recover from e-stop
            break;
         default:
             Serial.println("Error - no cases match");
            // whiteLED();
             break;
        }
       }
}

void readJoystick(){
    uint8_t xjoy, yjoy;

    if (joystickXCharacteristic.written() || joystickYCharacteristic.written()) {
      xjoy = joystickXCharacteristic.value();
      yjoy = joystickYCharacteristic.value();
      int8_t xmap = map(xjoy, 0, 254, -128, 127); //convert to range joyDiffDrive is expecting
      int8_t ymap = map(yjoy, 0, 254, 127, -128);
      joyDiffDrive(xmap*speedLimit,ymap*speedLimit);

/*    // debug
      Serial.print("x: ");
      Serial.print(xjoy);
      Serial.print(" y: ");
      Serial.print(yjoy);
      Serial.print(" xmap: ");
      Serial.println(xmap);
      Serial.print(" ymap: ");
      Serial.println(ymap);
*/
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
  PID_SET_1 =  nMotMixL*2; //*2 to convert from -127..+127 to -254..+254
  PID_SET_2 =  nMotMixR*2; //change PID setpoint
}

void driveMotor(int pwmPin, int dirPin, int spd){ //input speed -255 to +255, 0 is stop
  // Make sure the speed is within the limit.
  if (spd > 255) {
    spd = 255;
  } else if (spd < -255) {
    spd = -255;
  }

  if (spd > -35 && spd < 35){//add deadzone to turn off motors if PID is close to 0.
    spd = 0;
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

void calcPID(){
          //PID_SET_1 = setpoint set in read joystick or read buttons
          //PID_IN_1 = speed for encoder counts
          motorPID1.Compute(); //uses PID_SET_1 and PID_IN_1 to give PID_OUT_1 in -255 to 255
          motorPID2.Compute();
          driveMotor(ML_PWM, ML_DIR, PID_OUT_1);
          driveMotor(MR_PWM, MR_DIR, PID_OUT_2);
      /*  Serial.print(PID_IN_1);
          Serial.print(" In 1 ");
          Serial.print(PID_SET_1);
          Serial.print(" Set 1 ");
          Serial.print(PID_OUT_1);
          Serial.println(" Out 1"); */
}

void doEncoder1(){
  if(digitalRead(ENC_1_A)==digitalRead(ENC_1_B)){
    wheel1Pos++;
    encoder1Ticks++;
    if (wheel1Pos >1250){
      wheel1Pos = 0;
      wheel1Revs++;
    }
  }
  else{
    wheel1Pos--;
    encoder1Ticks--;
    if (wheel1Pos <-1250){
      wheel1Pos = 0;
      wheel1Revs--;
    }
  }
}
void doEncoder2(){
  if(digitalRead(ENC_2_A)==digitalRead(ENC_2_B)){
    wheel2Pos--; //opposite to Enc1 due to way motor is mounted
    encoder2Ticks--;
    if (wheel2Pos <-1250){
      wheel2Pos = 0;
      wheel2Revs--;
    }
  }
  else{
    wheel2Pos++;
    encoder2Ticks++;
    if (wheel2Pos >1250){
      wheel2Pos = 0;
      wheel2Revs++;
    }
  }
}
void calcSpeed(){
  //Get elapsed time
  long currentMillis = millis();
  long elapsedTime = currentMillis - prevMillis1;
  prevMillis1 = currentMillis;

  //Get encoder counts
  int encoder1Change = encoder1Ticks - encoder1Prev;
  int encoder2Change = encoder2Ticks - encoder2Prev;
  encoder1Prev = encoder1Ticks;
  encoder2Prev = encoder2Ticks;

  //Using top speed to be 1000 counts per second i.e range of -1 to 1
  double speed1 = (double)encoder1Change/(double)elapsedTime;
  double speed2 = (double)encoder2Change/(double)elapsedTime;
  int pidIn1 = speed1 * 255; //convert to range -255 - 255
  int pidIn2 = speed2 * 255;
  PID_IN_1 = constrain(pidIn1,-255,255);
  PID_IN_2 = constrain(pidIn2,-255,255);
}

void sendJSON(){
//Using JSON Doc format
const size_t capacity = JSON_OBJECT_SIZE(2);
StaticJsonDocument<capacity> doc;

/*doc["p"] = kp;
doc["i"] = ki;
doc["d"] = kd;
*/
doc["inc"] = ramp_inc;
doc["delay"] = ramp_delay;

serializeJson(doc, Serial);
Serial.println("");

}

void listenJSON(){
  // Used to change PID parameters via serial
  // Send in this formtal: {"p":3,"i":0,"d":0}
  if(Serial.available()){
    StaticJsonDocument<100> readJson;
    deserializeJson(readJson,Serial);
  /*  kp = readJson["p"];
    ki = readJson["i"];
    kd = readJson["d"];

    motorPID1.SetTunings(kp, ki, kd);
    */
   ramp_inc = readJson["inc"];
   ramp_delay = readJson["delay"];
    //sendJSON();
  }
}

void rampMotor1(){ //use timer in loop to call
  if(ramp_flag_1 == true){
    if(pid_ramp_1 == PID_SET_1){
      ramp_flag_1 = false;
    }
    else if(pid_ramp_1 > PID_SET_1){ //if desired setpoint is higher than current one
      PID_SET_1 = PID_SET_1 + ramp_inc; //increment value, change ratio of the and delay before calling function to change ramp
      if(PID_SET_1 >= pid_ramp_1){ //ramped up to desired setpoint so stop
        ramp_flag_1 = false;
      }
    }
    else if (pid_ramp_1 < PID_SET_1){
      PID_SET_1 = PID_SET_1 - ramp_inc;
      if(PID_SET_1 <= pid_ramp_1){ //ramped down to desired setpoint so stop
        ramp_flag_1 = false;
      }
    }
  }
}
void rampMotor2(){ //use timer in loop to call
  if(ramp_flag_2 == true){
    if(pid_ramp_2 == PID_SET_2){
      ramp_flag_2 = false;
    }
    else if(pid_ramp_2 > PID_SET_2){ //if desired setpoint is higher than current one
      PID_SET_2 = PID_SET_2 + ramp_inc; //increment value, change ratio of the and delay before calling function to change ramp
      if(PID_SET_2 >= pid_ramp_2){ //ramped up to desired setpoint so stop
        ramp_flag_2 = false;
      }
    }
    else if (pid_ramp_2 < PID_SET_2){
      PID_SET_2 = PID_SET_2 - ramp_inc;
      if(PID_SET_2 <= pid_ramp_2){ //ramped down to desired setpoint so stop
        ramp_flag_2 = false;
      }
    }
  }
}
void limitSpeed(){
  if (speedCharacteristic.written()) {
    speedLimit = speedCharacteristic.value();
  }
}
