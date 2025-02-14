# Theme 3 Sprint 2 Robot Documentation

The purpose of this document is to record key information about the design of the robot, any lessons from the sprint and to help other themes use the platform for further testing once the sprint is complete.

## Sprint 2

Sprint 2 was a 4 week long (26th Feb - 24th March 2020) hardware focused sprint, starting with a 2 day meeting and ending with a single day integration push, both at ICAIR.
Due to the Covid-19 pandemic the end of the sprint was postponed until access to campuses was partially restored (October 2020).
The briefing document is [here](https://drive.google.com/drive/folders/1twYxoXcRTZItLzTB_jrvbxy3_hq5Eg7F)

The core aspects of the robot are the Electronics, Software and Hardware. The requirements for these interact with all the other contributing themes and were decided at the initial meeting at the start of the sprint.

## Nomenclature

The robot developed in Sprint 2 of the Pipebots project is referred to as _SprintBot_ or colloquially _BallBot_
>Note: Some members call it _Sprintbot-2_ which can be confusing as this was the first robot developed in a sprint, but was undertaken in the second sprint event of the project. Sprint2Bot maybe more accurate.

## Electronics

The electronics are separated into two distinct sections to allow for parallel development of the mobile platform independently from the requirements of other themes electronic and sensor packages. The essential items remain on the robot at all times and are the minimum parts required for it to operate under manual control.
An Arduino Nano is used to control this part. By popular consensus from the other themes a Raspberry Pi 4 is used as the higher level controller as it is capable for running more complex algorithms. A 'shield' board was developed which plugs onto this and is used to connect all of the sensors to the board. The two boards communicate via a USB serial connection when required.

## T3 Essentials

Component List:

- [Arduino Nano 33 Sense BLE](https://store.arduino.cc/arduino-nano-33-ble-sense).  This board has Bluetooth Low Energy (BLE) and a number of useful sensors onboard including;
  - 9 axis imu
  - humidity, and temperature sensor
  - barometric sensor
  - microphone
  - gesture, proximity, light color and light intensity sensor
- 2 x [Cytron MD13S Motor Drivers](https://www.cytron.io/p-13amp-6v-30v-dc-motor-driver)
- 2 x [Pololu Encoders](https://www.pololu.com/product/3499)
- 2 x [65mm Neopixel LED rings](https://shop.pimoroni.com/products/neopixel-ring-24-x-5050-rgbw-leds-w-integrated-drivers?variant=17450439431)
- 2S 8A Battery and [protection circuit](https://www.amazon.co.uk/TECNOIOT-Lithium-Battery-Charger-Protection/dp/B07T3Y8BLD/ref=pd_sbs_107_3/261-3493005-3721666?_encoding=UTF8&pd_rd_i=B07T3Y8BLD&pd_rd_r=0dc0d0fd-9ead-485c-a5e4-70e66ca22b89&pd_rd_w=v4RQp&pd_rd_wg=JtENh&pf_rd_p=96cae456-8d7a-4bc1-91c7-9b20b4dfd7c9&pf_rd_r=S5YJ5PVFCHBJ3NF28B34&psc=1&refRID=S5YJ5PVFCHBJ3NF28B34)
- [Pololu 5V, 9A Step-Down Voltage Regulator D24V90F5](https://www.active-robots.com/pololu-5v-9a-step-down-voltage-regulator-d24v90f5.html)

*Figure of circuit diagram here*
*Figure of board here*

## Other Theme Payloads

Given the original time scale only minimal integration was going to be possible. With the extra time, a board was created that allows all of the sensors to be easily plugged into the Pi at the same time.

### T2 Sensors

This theme has two distinct payloads to test ultrasound and acoustic sensors.

1. Ultrasound
  - Red Pitaya
  - 4 x Ultrasound Transducers in 12cm square
2. Acoustic
  - Raspberry Pi
  - 2 x Microphones
  - Amplifier

### T4 Autonomous Control

This theme wanted to mount 5 x VL53L0X Time-of-Flight Distance Sensors and ran their code on the Raspberry Pi. In the original sprint, the plan seemed to have been to output directly to the motor drivers using PWM and Direction pins. Due to the extra development that T3 was able to do, including adding PID control and other aspects of the firmware, T4 were able to send commands to to the arduino using serial communication to trigger the same code that is used when pressing buttons on the app.

### T5 Localisation

This theme used a camera connected to the Raspberry Pi to record data. They also needed the encoder counts to be recorded, so a way of communicating this to the Pi and recording it was developed by T3.
A 'BrightPi' LED ring was used to provide light but it is not very bright so may need upgrading in future.
The algorithmic contribution of this theme was run offline once the data had been collected.

### T6 Communication

This theme was due to include a LoPy 4 board for testing communications, but due to a number of reasons the final test was swapped to be above ground and therefore not a useful environment for gathering data for T6.

### T7 and T8

These did not require payloads and were playing a more enabling than technical role in this sprint.

## Software

T3 is not responsible for the high level control of the robot. Who is responsible for the low level control was not clear at the start of the sprint but as T3 had existing work that could be partly repurposed and had more direct access to the components we took it upon ourselves to create a method for manual control. This was later developed into firmware which can receive commands from a higher level program and then move the robot.

### Robot Firmware

This is the [code](https://github.com/pipebots/sprintBotBLEControl) which runs on the Arduino.
There are two main versions, the *master* branch is the standard code that allows remote Bluetooth (BLE) control.
The *T4_simple_coms* branch uses serial communication as the input rather than bluetooth but is otherwise very similar.
The functions are listed below:

#### readButtons();

This takes the number which has been transmitted via bluetooth or serial and switches the case.
In the movement cases the ramp flag is set high and the required setpoint for the PID controller for the motors is set, with the current speed limit modifying it.
When the ramp flag is high, in each loop of the code the value of the setpoint is adjusted by one step until it matches the desired setpoint. This helps the robot to accelerate smoothly reducing rocking of the body.

The available cases are:

0. LEDs Off
1. LEDs On
2. Forward
3. Backwards
4. Left (turn on the spot one motor forward, the other reverse)
5. Right (turn on the spot one motor forward, the other reverse)
6. Stop
7. Forward Left (Wider turn, one motor off, the other forwards)
8. Forward Right (Wider turn, one motor off, the other forwards)
9. Back Left (Wider turn, one motor off, the other backwards)
10. Back Right (Wider turn, one motor off, the other backwards)
11. (not currently in use: Autonomous control)
12. Emergency Stop (Disconnects Bluetooth and stops robot, requires hardware reset on robot so use carefully!)
13. (not currently in use: Manual control)

#### readJoystick();

This reads the BLE characteristic that relaes to the X & Y coordinates of the joystick in the app, maps the values the the correct range for joyDiffDrive() and modifies with the speed limit.

#### joyDiffDrive(int nJoyX, int nJoyY);

Converts a single dual-axis joystick into a differential drive motor control, with support for both drive, turn and pivot operations. Code by [Calvin Hass](https://www.impulseadventure.com/elec/robot-differential-steering.html).
The control via the joystick is not recommended as it needs some tuning. It seems that a large number of responses are generated when moving the stick so if it is moved quickly a queue of them can form and the response of the robot lags as it goes through them all. The sensitivity to motion is one parameter that was tuned to reduce this, but there is a trade-off with responsiveness to user input. The other possible fix would be to use lossy communication where just the latest command is retained, so that the robot only responds to the position of the control stick at the time it is executing the command. Time limitations and the limited advantage over the button control meant this has not been explored further.

#### calcPID();

Using the current speed and desired setpoint this calculates the required output for the motors, calls driveMotor.

#### driveMotor(int pwmPin, int dirPin, int spd);

Outputs PWM and direction to the motor drivers. Input speed value range is -255 to +255, 0 is stop.
The function checks the speed is within the limits and converts it to a PWM and direction command.
There is also a dead-zone which turns the motors off if the required speed is close to 0 as this causes the motors to whine and heat up without being enough to start the robot moving.

#### doEncoder1(); & doEncoder2();

These are triggered using interrupts so a pulse is not missed. These count the encoder counts and adjust the wheel position and total full revolution counters.

Pololu 3499 Encoder has 20 counts per rev when counting both edges of both channels. We are just counting one edge of one channel as the high gear ratio gives us plenty of counts this way, so 5 counts per revolution. The gear ratio of the motor is 1:62.5.
5 x 62.5 = 312.5 counts per revolution of gearbox output shaft.
Spur gear to ring gear ratio is: Spur gear PCD=15mm, PCD internal ring =120mm. 120/15=8.
312.5 x 8 = 2500 counts per revolution of the robots wheel.
> Note: A 1:31.25 gearbox motor was also used tests, in this case there are 1250 counts per revolution of the wheel

#### calcSpeed();

Using the change in encoder counts and the elapsed time since last call this works out the speed of each motor, for use in the PID loop.

#### sendJSON(); & listenJSON();

These send and receive data formatted as a JSON document over te serial port.
Mainly used for debugging and recording sensor data for T5 and others to use offline.
The parameters are:

- wheel1Pos & wheel2Pos - Current encoder count (0-2500)
- wheel1Revs & wheel2Revs - Number of full turns for each wheel. Decreases if the robot drives backwards.
- pressure_kPa - Pressure sensor
- temp_oC - read temp sensor in degrees celcius
- humidity_percent - Humidity in %

In testing with T4 it was discovered that this method caused problems when communicating with the Raspberry Pi so a simpler format was used to decrease the amount of data being transferred. These are called **readCom() & sendCOM()** in the T4_simple_coms branch.

#### rampMotor1(); &  rampMotor2();

Adjusts the setpoint by one step and checks if the required setpoint has been reached.

#### limitSpeed();

Reads BLE characteristic and sets variable that is used to adjust all motions.

#### loadingChase(int speed, uint32_t color, int loops, Adafruit_NeoPixel strip);

Called on startup, controls the 'loading sequence' on the LED rings

#### loadingChaseDoubleRing(int speed, uint32_t color, int loops, Adafruit_NeoPixel strip, Adafruit_NeoPixel strip2);

Leftover from when the rings were independently controlled. When trying to control two NeoPixel rings it caused issues with the rest of the code. It is hypothesised that this is because the library for these (surprisingly complex to run) LED rings uses timers and interrupts which may be interfering with the ones being used in the rest of the code.
The two rings are now connected to the same output pin of the arduino but when trying to remove this function from the code it would cause it to stop working. There seems to be no logical reason for this as the function is never called, but it is left in place as it seemed to fix the problem!

#### ringColour(char colour, Adafruit_NeoPixel strip1, Adafruit_NeoPixel strip2);

Sets the colour for the full LED ring. Used on startup and connection but doNeoRings used after.

####  doNeoRings();

A single led pixel changes colour to show which case in **readButtons()** has been activated. This pixel spins around the ring at the same speed as the wheel turns by using the encoder counts to change position.
The rest of the ring can be set to any colour, currently the Headlights slider in the app sets this to off or full white.

#### readIMU();

Reads the accelerometer and gyroscope values then uses the MadgwickAHRS library to filter these values and get roll, pitch and yaw.

## Android App

[This app](https://github.com/nfry321/PipebotsApp) was first created before the sprint and has been extended and edited to fit the features of this robot.
It was created using [Evothings](http://evothings.com/) which is designed to be a quick way of making an app by writing HTML and Javascript in the same way as a website is created. This allows development via a viewer app which essentially is a web browser with some additional packages. The advantage of this is that every time the code it edited it can instantly update on the phone allowing very quick development. The code can then be complied into a native android app and installed directly. It can also be compiled for iPhones but distributing apps without going through the extensive process of getting it onto the app store is difficult. It is therefore simpler to download the viewer app and direct it to a website hosting the code. Instructions have been written for installing the app if needed.
> Note: Since initially developing this app `Evothings` studio has been retired so this should not be used for developing future items.
> Note: 11 Feb 2025 - The repo containing the app is no longer available.

*TO DO: add screenshots here*

## Hardware

### To Do

- Add images of first version
- Add technical details such as size
- bulletpoint features
- costs/bom

## Development

During the initial meeting a conceptual design was agreed upon. The robot was to be ball shaped to allow it to fit into pipes and go around corners while maximising the internal space available for payloads.

![Initial Concept](https://lh3.googleusercontent.com/E7FioxXoL-sD-zDWrzA2RGpfFI7eF6nUFwQqx7UFxy_vZ_R5mDplhw4VAzh1OrHBHdupwxcmBbxF3go631b102Iw8U5Eb_4meXYR49-zBM7Rpwbfli8-9HHBrAoKDmLpVV-MdVv-RIjK5_oDu73OYlal7EFl6ujYleHDW7PzoO9_DiL66SersYBGapeCW-zjARIoLfbAff85Z5uSwGCnJ6UfuGmzIgYy5cr2w-85sbtnEkRcoOqBmJ2PNo91SmD7UVmtRZCSa1e1rhqks-aH1PXL2_OYJlbifZepETJRkPieoGwq9qrOeQK899fbBwwYsVApV-OtX-GWENCOV0HSty17_B4rmZEjMODti0sJXOuepSgcVokiA7k3IAFGY6MdSWqVNeOgyv5riF4kSS4XssJ0UhxEyQATmiHHvBzcDITr3sixGHQLD-9bXG4Do2lByJ8U9-gooKx292SVkNmjKZZQAER7MCIWl7CbByryLzN-XIRIkPgXkFhT7RIsjQAf4996Ha25CRKdyi0yA6r8Edinmpq1aeVNKFthkhn5x-OHUbaQOrzhVEQjoyJHERmY5JnDAb-ciIH6_cDwhPEhnbCqO4OJKHAlBHevrB4U9uHOTMYQMACrEDAvn2SGZlCcncxcuuIp543qxzjRJNtHTx3yFrz1QTdWdGeDkoc1Hi-_-tomOfG0MT0N5orx=w712-h607-no?authuser=0 =400x350)
*The initial Concept*

![Final Render](https://leeds365.sharepoint.com/sites/TEAM-EPSRCPipebots/Shared%20Documents/Sprint%202/Photos%20and%20Videos/FancyRender.jpg =400x400)
*Final Render*

![enter image description here](https://leeds365.sharepoint.com/sites/TEAM-EPSRCPipebots/Shared%20Documents/Sprint%202/Photos%20and%20Videos/IMG_20200826_094830%20b.jpg =400x400)
*Final Robot*

Designing, manufacturing, procuring and assembling the components required for a mobile robot designed to go into a real outdoor environment is a very difficult task in the time constrains of a sprint.
To achieve it we first decided upon the components that must be bought externally (motors, motor drivers gears etc.) before finalising our mechanical design while these were on order. Procuring specialist components in a short time-frame is a challenge that shoud not be overlooked in future sprints and it must be accepted that the costs will rise due delivery costs and the potential for design changes to make parts obsolete.
For the most part this was avoided, but it was decided to 3D print the gears rather than use the machined ones that were ordered as this gave a much greater flexibility in design and considerably reduced the weight.
This allowed the large ring gears to have a larger hole in the centre, giving more space for other components and the outer surface was keyed to allow the tyre to grip when over-moulded. The bevel drive gears now have much larger teeth giving greater robustness and shaft hole is easily varied and D shaped as required.

3D printing was used to manufacture the whole mechanical structure of the robot as it has the shortest lead time of the methods available to us, and allows great geometric freedom. Fused Filament Fabrication was used with an enhanced PLA material (Polymaker Polymax) as this is capable of producing parts that are functional and tough enough for testing use when producing parts of this size.

The robot was designed such that parts are side-invariant, i.e. they can be used on either side of the robot. This was decided to reduce complexity and repetitive design and means that a smaller number of spare parts need to be on hand. The robot only uses 4 major parts per side, allowing simple assembly.

## Usage Instructions

### To Do:

- Explain drive system inc key bits to tighten and where it may fail (i.e grub screws on bevel gears)
- Show access points and basic assembly
- Show battery connecting and charging
- refer to electronics section for connecting payloads

## ICAIR Testing Report

On 12th & 14th October 2020 .....we did.... *to do*

### Successes (Yays, Woops)

- It worked!
- Simple to use and drive
- Others able to use and drive by simply downloading the app (more covid secure than a controller ;) )
- Attention to disassembly and ease of access to the drive system really helpful
Parts could be replaced in the field with basic tools due to simple access and the use of detachable sockets for major electronic components.
- Connecting different payloads simple due to plug and socket board.
- Everything that needed to fit inside/on it (but not easily see below)
-

### Issues (Nays, Poops)

All in all the test was very successful and most of the issues were due to limitations of the shape or size of the robot or due to a lack of time due to te sprint format. The only two hardware failings which required fixes in the field when testing at ICAIR were a motor burning out (see Tyre Flexibility) and a gear slipping on a shaft (see Drive System Reliability). The components were simple to swap out and replace with spares.

#### Drive System Reliability

Due to the limited manufacturing time, shafts could not be machined properly with flats or keyways to grip the drive gears. The gears are also 3D printed which is usually strong enough but may wear quickly. A flat has been filed onto the shafts and a grub screw is used to keep the gears in place. This works but is not the ideal solution. In testing one of the bevel gears span on the shaft and it could be seen that it had cracked slightly around the grub screw causing it to come loose. This was replaced by a new one (that used tougher material so this problem should not repeat). A future upgrade would be to machine the proper shaped shafts and gears from metal, or at least make the drive shaft more D shaped with a corresponding gear printed to fit.

#### Possible to tip robot

If the robot is driven up the side of the pipe, especially when going round a bend or entering a pipe up a step, it is possible for the robot to tip onto its side and not be able to recover itself. This is a limitation of a design that is also designed to be a ball shape and be able to turn around inside a pipe. If the tyres matched the size of the body more closely it is possible that the robot would no be able to rest on its side to the same extent but the ground clearance would then be a problem. In a smaller pipe this would not happen when driving forwards as the side would touch the pipe before reaching a point of instability. However the robot can turn in the pipe so it is still possible to drive it at an angle up the wall of the pipe.

*picture of the 250mm pipe here*

#### Tyre Flexibility

The robot was outfitted with larger tyres than the original design giving a 360 degree bumper and helping it climb up obstacles. Due to the limitation covid placed upon the manufacturing the 3D printed hub size was not increased and softer silicone was used to case the tyres. This combination means that if the robot tips onto its side it is possible for the tyre to bend underneath the robot enough that it jams against the body and stops the wheel spinning. In an in pipe test this could not be seen and commands were still sent to the motor causing it to overheat and become damaged.
It is expected that by simply increasing the diameter of the hub and reducing the thickness of the tyre this will no longer be possible.

*Still from video showing this here*

### Not much space inside due to the wires.

This was expected and is the reason T3 insisted on the largest robot possible for the sprint. The ball shape and designing the battery and drive systems to be in the sides of the robot helped with this issue. The Pi and Arduino can fit together with the lid closed, the Red Pitaya would not as well, but the Pi was not needed at the same time. It is possible that making some of the wires longer would help as the position of the boards would be less constrained.

## Sprint Lessons Learnt

To Do.
