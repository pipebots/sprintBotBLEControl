//Robot code for NANO 33 BLE Sense to pair with Robot controller App
//for new board - not Sense version
//Nick Fry 2019
//Change robotName[] below for each board

#include <ArduinoBLE.h>
#include <rgbLED.h>

void readButtons();

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
long previousMillis = 0;  // last time the temperature was checked, in ms
bool gesture = false; //in gesture control mode?
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
//       // updateTemp();
      }
      */
      readButtons();
      //readJoystick();
      //gestureControl();

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
            offLED();
            break;
          case 1:
            Serial.println("LED on");
            digitalWrite(ledPin, HIGH);          // will turn the LED off
            break;
          case 2:
            Serial.println("FWD");
            greenLED();
            digitalWrite(ML_DIR, HIGH);
            digitalWrite(ML_PWM, HIGH);
            digitalWrite(MR_DIR, HIGH);
            digitalWrite(MR_PWM, HIGH);
            break;
          case 3:
            Serial.println("Back");
            blueLED();
            digitalWrite(ML_DIR, LOW);
            digitalWrite(ML_PWM, HIGH);
            digitalWrite(MR_DIR, LOW);
            digitalWrite(MR_PWM, HIGH);
            break;
          case 4:
            Serial.println("Left");
            cyanLED();
            digitalWrite(ML_DIR, LOW);
            digitalWrite(ML_PWM, HIGH);
            digitalWrite(MR_DIR, HIGH);
            digitalWrite(MR_PWM, HIGH);
            break;
          case 5:
            Serial.println("Right");
            magentaLED();
            digitalWrite(ML_DIR, HIGH);
            digitalWrite(ML_PWM, HIGH);
            digitalWrite(MR_DIR, LOW);
            digitalWrite(MR_PWM, HIGH);
            break;
          case 6:
            Serial.println("Stop");
            redLED();
            digitalWrite(ML_DIR, LOW);
            digitalWrite(ML_PWM, LOW);
            digitalWrite(MR_DIR, LOW);
            digitalWrite(MR_PWM, LOW);
            break;
          case 7:
            Serial.println("Fwd Left");
            rgbLED(100,255,100);
            digitalWrite(ML_DIR, HIGH);
            digitalWrite(ML_PWM, HIGH);
            digitalWrite(MR_DIR, HIGH);
            digitalWrite(MR_PWM, LOW);
            break;
          case 8:
            Serial.println("Fwd Right");
            rgbLED(255,100,100);
            digitalWrite(ML_DIR, HIGH);
            digitalWrite(ML_PWM, LOW);
            digitalWrite(MR_DIR, HIGH);
            digitalWrite(MR_PWM, HIGH);
            break;
          case 9:
            Serial.println("Back Left");
            rgbLED(255,50,100);
            digitalWrite(ML_DIR, LOW);
            digitalWrite(ML_PWM, LOW);
            digitalWrite(MR_DIR, LOW);
            digitalWrite(MR_PWM, HIGH);
            break;
          case 10:
            Serial.println("Back Right");
            rgbLED(50,100,100);
            digitalWrite(ML_DIR, LOW);
            digitalWrite(ML_PWM, HIGH);
            digitalWrite(MR_DIR, LOW);
            digitalWrite(MR_PWM, LOW);
            break;
          case 11:
            Serial.println("Gesture Control");
            yellowLED();
            gesture = !gesture; //flip bool
            Serial.println("Gesture");
            break;
           default:
             Serial.println("Error - no cases match");
             whiteLED();
             break;
        }
       }
}
/*void readJoystick(){
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
      xchange = map(xnib, 0, 7, -255, 255);
      ychange = map(ynib, 0, 7, -255, 255);
      Serial.print("xchange: ");
      Serial.print(xchange);
      Serial.print("ychange: ");
      Serial.println(ychange);
      }
}
*/
