# Theme 3 Sprint 2 Robot Documentation

The purpose of this document is to record key information about the design of the robot, any lessons from the sprint and to help other themes use the platform for further testing once the sprint is complete. 

## Sprint 2
Sprint 2 was a 4 week long (26th Feb - 24th March 2020) hardware focused sprint, starting with a 2 day meeting and ending with a single day integration push, both at ICAIR. 
Due to the Covid-19 pandemic the end of the sprint was postphoned until access to campuses was partically restored (October 2020).
The briefing document is [here](https://drive.google.com/drive/folders/1twYxoXcRTZItLzTB_jrvbxy3_hq5Eg7F)

The core aspects of the robot are the Electronics, Software and Hardware. The requirements for these interact with all the other contributing themes.

## Nomenclature
The robot developed in Sprint 2 of the Pipebots project is refered to as _SprintBot_ or colloquially _BallBot_ 
>Note: Some members call it _Sprintbot-2_ which can be confusing as this was the first robot developed in a sprint, but was undertaken in the second sprint event of the project. Sprint2Bot maybe more accurate.

# Electronics
The electronics are separated into two distinct sections to allow for parallel development of the mobile platform independently from the requirements of other themes electronic and sensor packages. The essential items remain on the robot at all times and are the minimum parts required for it to operate under manual control.
An Arduino Nano is used to control this part. By popular consensus from the other themes a Raspberry Pi 4 is used as the higher level controller as it is capable for running more complex algorythms. A 'shield' board was developed which plugs onto this and is used to connect all of the sensors to the board. The two boards communicate via a USB serial connection when required.

## T3 Essentials

Component List:
- [Arduino Nano 33 Sense BLE](https://store.arduino.cc/arduino-nano-33-ble-sense). 
		This board has Bluetooth Low Energy (BLE) and a number of useful sensors onboard including;
	-   9 axis imu
	-   humidity, and temperature sensor
	-   barometric sensor
	-   microphone
	-   gesture, proximity, light color and light intensity sensor 
- 2 x [Cytron MD13S Motor Drivers](https://www.cytron.io/p-13amp-6v-30v-dc-motor-driver)
- 2 x [Pololu Encoders](https://www.pololu.com/product/3499)
- 2 x [65mm Neopixel LED rings](https://shop.pimoroni.com/products/neopixel-ring-24-x-5050-rgbw-leds-w-integrated-drivers?variant=17450439431)
- 2S 8A Battery and [protection circuit](https://www.amazon.co.uk/TECNOIOT-Lithium-Battery-Charger-Protection/dp/B07T3Y8BLD/ref=pd_sbs_107_3/261-3493005-3721666?_encoding=UTF8&pd_rd_i=B07T3Y8BLD&pd_rd_r=0dc0d0fd-9ead-485c-a5e4-70e66ca22b89&pd_rd_w=v4RQp&pd_rd_wg=JtENh&pf_rd_p=96cae456-8d7a-4bc1-91c7-9b20b4dfd7c9&pf_rd_r=S5YJ5PVFCHBJ3NF28B34&psc=1&refRID=S5YJ5PVFCHBJ3NF28B34)
- [Pololu 5V, 9A Step-Down Voltage Regulator D24V90F5](https://www.active-robots.com/pololu-5v-9a-step-down-voltage-regulator-d24v90f5.html)

*Figure of circuit diagram here*
*Figure of board here*

## Other Theme Payloads
Given the origional time scale only minimal integration was going to be possible. With the extra time, a board was created that allows all of the sensors to be easily plugged into the Pi at the same time. 

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
This theme wanted to mount 5 x VL53L0X Time-of-Flight Distance Sensors and ran their code on the Raspberry Pi. In the origional sprint the plan seemed to have been to output directly to the motor drivers using PWM and Direction pins. Due to the extra development that T3 was able to do, including adding PID control and other aspects of the firmware, T4 were able to send commands to to the arduino using serial communication to trigger the same code that is used when pressing buttons on the app. 

### T5 Localisation
This theme used a camera connected to the Raspberry Pi to record data. They also needed the encoder counts to be recorded, so a way of communicating this to the Pi and recording it was developed by T3. 
A 'BrightPi' LED ring was used to provide light but it is not very bright so may need upgrading in future.
The algrythmic contribution of this theme was run ofline once the data had been collected. 

### T6 Communication
This theme was due to include a LoPy 4 board for testing communications, but due to a number of reasons the final test was swapped to be above ground and therefore not a useful environment for gathering data for T6. 

### T7 and T8
These did not require payloads and were playing a more enabling than technical role in this sprint. 

# Software
T3 is not responsible for the high level control of the robot. Who is responsible for the low level control was not clear at the start of the sprint but as T3 had exsisting work that could be partly repurposed and had more direct access to the components we took it upon ourselves to create a method for manual control. This was later developed into firmware which can recieve commands from a higher level program and then move the robot.  

## Robot Firmware
This is the [code](https://github.com/pipebots/sprintBotBLEControl) which runs on the Arduino. 
There are two main versions, the *master* branch is the standard code that allows remote bluetooth control.
The *T4_simple_coms* branch uses serial communication as the input rather than bluetooth but is otherwise very similar. 
The functions are listed below:
### readButtons();
This takes the number which has been transmitted via bluetooth or serial and switches the case

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

void loadingChase(int speed, uint32_t color, int loops, Adafruit_NeoPixel strip);

void loadingChaseDoubleRing(int speed, uint32_t color, int loops, Adafruit_NeoPixel strip, Adafruit_NeoPixel strip2);

void ringColour(char colour, Adafruit_NeoPixel strip1, Adafruit_NeoPixel strip2);

void doNeoRings();

void readIMU();

## Android App




# Hardware
>This part is not currently intended to be a complete reference but maybe added to in the future


### To Do
- Add images of first version 
- Add technical details such as size
- Explain drive system inc key bits to tighten and where it may fail (i.e grub screws on bevel gears)
- Show access points and basic assembly
- bulletpoint features
- costs/bom

## Development
During the initial meeting a conceptual design was agreed upon. The robot was to be ball shaped to allow it to fit into pipes and go around corners while maximising the internal space avaliable for payloads. 

![Initial Concept](https://lh3.googleusercontent.com/E7FioxXoL-sD-zDWrzA2RGpfFI7eF6nUFwQqx7UFxy_vZ_R5mDplhw4VAzh1OrHBHdupwxcmBbxF3go631b102Iw8U5Eb_4meXYR49-zBM7Rpwbfli8-9HHBrAoKDmLpVV-MdVv-RIjK5_oDu73OYlal7EFl6ujYleHDW7PzoO9_DiL66SersYBGapeCW-zjARIoLfbAff85Z5uSwGCnJ6UfuGmzIgYy5cr2w-85sbtnEkRcoOqBmJ2PNo91SmD7UVmtRZCSa1e1rhqks-aH1PXL2_OYJlbifZepETJRkPieoGwq9qrOeQK899fbBwwYsVApV-OtX-GWENCOV0HSty17_B4rmZEjMODti0sJXOuepSgcVokiA7k3IAFGY6MdSWqVNeOgyv5riF4kSS4XssJ0UhxEyQATmiHHvBzcDITr3sixGHQLD-9bXG4Do2lByJ8U9-gooKx292SVkNmjKZZQAER7MCIWl7CbByryLzN-XIRIkPgXkFhT7RIsjQAf4996Ha25CRKdyi0yA6r8Edinmpq1aeVNKFthkhn5x-OHUbaQOrzhVEQjoyJHERmY5JnDAb-ciIH6_cDwhPEhnbCqO4OJKHAlBHevrB4U9uHOTMYQMACrEDAvn2SGZlCcncxcuuIp543qxzjRJNtHTx3yFrz1QTdWdGeDkoc1Hi-_-tomOfG0MT0N5orx=w712-h607-no?authuser=0 =400x350)
*The initial Concept*

![Final Render](https://leeds365.sharepoint.com/sites/TEAM-EPSRCPipebots/Shared%20Documents/Sprint%202/Photos%20and%20Videos/FancyRender.jpg =400x400)
*Final Render*

![enter image description here](https://leeds365.sharepoint.com/sites/TEAM-EPSRCPipebots/Shared%20Documents/Sprint%202/Photos%20and%20Videos/IMG_20200826_094830%20b.jpg =400x400)
*Final Robot*

Designing, manufacturing, procuring and assembling the components required for a mobile robot designed to go into a real outdoor environment is a very difficult task in the time constrains of a sprint. 
To achieve it we first decided upon the components that must be bought externally (motors, motor drivers gears etc.) before finialising our mechanical design while these were on order. Procuring specialist components in a short timeframe is a challenge that shoud not be overlooked in future sprints and it must be accepted that the costs will rise due delivery costs and the potential for design changes to make parts obsolete. 
For the most part this was avoided, but it was decided to 3D print the gears rather than use the machined ones that were ordered as this gave a much greater flexibility in design and considerablly reduced the weight. 
This allowed the large ring gears to have a larger hole in the centre, giving more space for other components and the outersurface was keyed to allow the tyre to grip when overmoulded. The bevel drive gears now have much larger teeth giving greater robustness and shaft hole is easily varied and D shaped as required. 

3D printing was used to manufacture the whole mechanical structure of the robot as it has the shortest leadtime of the methods avaliable to us, and allows great geometric freedom. Fused Filament Fabrication was used with an enhanced PLA material (Polymaker Polymax) as this is capable of producing parts that are functional and tough enough for testing use when producing parts of this size. 

The robot was designed such that parts are side-invariant, i.e. they can be used on either side of the robot. This was decided to reduce complexity and repetitive design and means that a smaller number of spare parts need to be on hand. The robot only uses 4 major parts per side, allowing simple assembly. 

<!--stackedit_data:
eyJoaXN0b3J5IjpbLTIwNDc3NzI4NzEsNTEzOTUxMjIxLDE5MD
AwODE4MDAsNTkyMjg2Mzg4LDEzNDMwNjU1NzMsLTE5MDEyMzkz
MDgsMTc4MzExNTQyLC03Mzg1NDAwMzIsOTg0OTIyNjkzLDQ2OD
AxMjkwOSwtNTA4Mzk3NjUzLC0xMTUwNTUzNTc1LC0yODA3ODQy
MCwxNDQyMTAyNTYsLTEyNTA0MDk0MTEsLTIxMjkxMjU3MDQsMT
E5OTYzMDUzNSwxMzYwNjA2NF19
-->