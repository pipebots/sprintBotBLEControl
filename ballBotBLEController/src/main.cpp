//Robot code for NANO 33 BLE Sense to pair with Robot controller App

#include <Arduino.h>
#include <ArduinoBLE.h>
#include "CytronMotorDriver.h"
#include <rgbLED.h>


void readButtons();

char robotName[] = "BigBallBot"; //Device Name - will appear as BLE descripton when connecting

BLEService inputService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE Service for all inputs from app

// BLE LED Switch Characteristic - custom 128-bit UUID, read and writable by central
BLEByteCharacteristic buttonCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEByteCharacteristic joystickCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

const int ledPin = LED_BUILTIN; // pin to use for the LED
int speed = 255;

// Configure the motor driver.
CytronMD motorL(PWM_DIR, 9, 8);  // PWM = Pin 9, DIR = Pin 8.
CytronMD motorR(PWM_DIR, 11, 10);  


void setup() {
  Serial.begin(9600);
//  while (!Serial); //This stops the program running until the serial monitor is opened!


  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  BLE.setLocalName(robotName);
  BLE.setAdvertisedService(inputService);

  // add the characteristic to the service
  inputService.addCharacteristic(buttonCharacteristic);
  inputService.addCharacteristic(joystickCharacteristic);

  // add service
  BLE.addService(inputService);

  // set the initial value for the characeristic:
  buttonCharacteristic.writeValue(0);
  joystickCharacteristic.writeValue(0);

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

    readButtons();

    }


    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
}


void readButtons(){
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
            motorL.setSpeed(speed);
            motorR.setSpeed(speed);
            greenLED();
            break;
          case 3:
            Serial.println("Back");
            motorL.setSpeed(-speed);
            motorR.setSpeed(-speed);
            blueLED();
            break;
          case 4:
            Serial.println("Left");
            motorL.setSpeed(-speed);
            motorR.setSpeed(speed);
            cyanLED();
            break;
          case 5:
            Serial.println("Right");
            motorL.setSpeed(speed);
            motorR.setSpeed(-speed);
            magentaLED();
            break;
          case 6:
            Serial.println("Stop");
            motorL.setSpeed(0);
            motorR.setSpeed(0);
            redLED();
            break;
          case 7:
            Serial.println("Fwd Left");
            motorL.setSpeed(speed/2);
            motorR.setSpeed(speed);
            rgbLED(100,255,100);
            break;
          case 8:
            Serial.println("Fwd Right");
            motorL.setSpeed(speed);
            motorR.setSpeed(speed/2);
            rgbLED(255,100,100);
            break;
          case 9:
            Serial.println("Back Left");
            motorL.setSpeed(speed);
            motorR.setSpeed(-speed/2);
            rgbLED(255,50,100);
            break;
          case 10:
            Serial.println("Back Right");
            motorL.setSpeed(-speed/2);
            motorR.setSpeed(speed);
            rgbLED(50,100,100);
            break;

           default:
             Serial.println("Error - no cases match");
             whiteLED();
             break;
        }
       }
}
