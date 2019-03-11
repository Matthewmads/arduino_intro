# Intro to Arduino
This repository contains notes, documentation, and code snippets for the MCT Intro to Arduino Class.

## Getting Started
The Arduino IDE is already configured on MakerspaceCT's computers, but when you go to set up the Development Environment at home, here is where to find the software:
* Download the latest version (1.8.8 as of this writing) of the Arduino IDE: https://www.arduino.cc/en/Main/Software

## Notes / Useful Links
* [Arduino in a Nutshell](https://www.teachmeteamwork.com/files/arduino-in-a-nutshell-1.8.pdf) - Jan Borchers' 20-page crash course in Arduino. Great, simple explanations of Arduino features.
* [Arduino Website](https://arduino.cc) - Official Arduino project website - lots of great learning tutorials here.
* [Smraza S32 Starter Kit](https://www.amazon.com/Smraza-Starter-Ultrasonic-Distance-Raspberry/dp/B01MATM4XF/ref=sr_1_1?ie=UTF8&qid=1547591276&sr=8-1&keywords=s32+smraza) - Parts kit used in class (does not include Arduino Uno board or USB cable).
* [Smraza S32 Tutorial Download](https://mega.nz/#F!9tYjhBLR!LerlzHYou2gNKJvzXs_8aA) - Link to download the source tutorials for the kit used in class.
* [Uno R3-compatible Board and Cable](https://www.amazon.com/Elegoo-EL-CB-001-ATmega328P-ATMEGA16U2-Arduino/dp/B01EWOE0UU/ref=sr_1_1?keywords=arduino+uno&qid=1552332604&s=gateway&sr=8-1) - sample Arduino-compatible board. This isn't linked because it's any better than any other board. You can also search Amazon, [Bang good](https://www.banggood.com/UNO-R3-ATmega328P-Development-Board-For-Arduino-No-Cable-p-964163.html?rmmds=search&cur_warehouse=CN), or any number of other sites for "Arduino Uno R3".

## Code snippets
These are some of the code snippets used in class.

### Blink
```
// Pin 13 has an LED connected on most Arduino boards.
int led = 13;

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);     
}

// the loop routine repeats forever:
void loop() {
  digitalWrite(led, HIGH);      // turn the LED on (HIGH is the voltage level)
  delay(1000);                  // wait for a second
  digitalWrite(led, LOW);       // turn the LED off by making the voltage LOW
  delay(1000);                  // wait for a second
}
```

### Potentiometer (Analog Input)
```
int sensorPin = A0;    // select the input pin for the potentiometer
int ledPin = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor

void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);
  Serial.println(sensorValue);
  // turn the ledPin on
  digitalWrite(ledPin, HIGH);
  // stop the program for <sensorValue> milliseconds:
  delay(sensorValue);
  // turn the ledPin off:
  digitalWrite(ledPin, LOW);
  // stop the program for for <sensorValue> milliseconds:
  delay(sensorValue);
}

```
## Part 2
These code samples were used in Part 2 of MakerspaceCT's Intro to Arduino. Feel free to give them a try on your own!

### Stepper
```
#include <Stepper.h>
const int stepsPerRevolution = 200;  // change this to fit the number of steps per revolution
// for your motor
// initialize the stepper library on pins 2 through 5:
Stepper myStepper(stepsPerRevolution, 2, 3, 4, 5);
int stepCount = 0;         // number of steps the motor has taken
void setup() {
  // initialize the serial port:
  Serial.begin(9600);
}
void loop() {
  // step one step:
  myStepper.step(1);
  Serial.print("steps:");
  Serial.println(stepCount);
  stepCount++;
  delay(500);
}
```

### IIC Scanner
```
// --------------------------------------
// i2c_scanner
//
// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not know.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    http://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
// Version 6, November 27, 2015.
//    Added waiting for the Leonardo serial communication.
//
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//

#include <Wire.h>


void setup()
{
  Wire.begin();

  Serial.begin(9600);
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner");
}


void loop()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}
```

### LCD1602
```
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3F,16,2);

void setup()                                                                                     
{
  lcd.init();
  lcd.backlight();
  lcd.print("Welcome to");
  lcd.setCursor(0,1);   //Display position
  lcd.print("MakerspaceCT");
}
void loop()
{
  // Turn off the display:
  lcd.noDisplay();
  delay(500);
  // Turn on the display:
  lcd.display();
  delay(500);
 }

```
### LCD1602 with Potentiometer
```
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27,16,2);

void setup()                                                                                     
{
  lcd.init();
  lcd.backlight();
  lcd.print("Welcome to");
  lcd.setCursor(0,1);   //Display position
  lcd.print("MakerspaceCT");
  delay(1000);
}
void loop()
{
  int sensorValue = analogRead(A0);
  // Move the cursor back to the beginning to replace existing text
  lcd.setCursor(0,0);
  lcd.print("Sensor Value:");
  // Move the cursor to the second line
  lcd.setCursor(0,1);
  lcd.print(String(sensorValue) + "           ");
  delay(500);
 }

```


### LCD1602 with  DHT Sensor
```
#include <LiquidCrystal_I2C.h>
#include <dht11.h>    

LiquidCrystal_I2C lcd(0x3F,16,2); // set the LCD address to 0x3F for a 16 chars and 2 line display  

dht11 DHT;                          //Note:DHT on behalf of the temperature and humidity sensor
const int dht11_data = 6;     
int temp=0;
int hum=0;


void setup()                                                                                     
{
  lcd.init();     // initialize the lcd
  lcd.backlight();
  lcd.print("Welcome to");
  lcd.setCursor(0,1);
  lcd.print("MakerspaceCT");
  delay(2000);
  lcd.clear();
}
void loop()
{
  DHT.read(dht11_data);
  temp=DHT.temperature;
  hum=DHT.humidity;
  lcd.clear();                   //clear display
  lcd.print("Hum=%");            //display "Hum=%"
  lcd.print(hum);
  lcd.setCursor(10,0) ;
  lcd.print("MCT");           //display "MCT"
  lcd.setCursor(0,1) ;           //Display position
  lcd.print("Tem=");            //display"Temp="
  lcd.print(temp);
  lcd.write(0xDF);              //Display custom characters '°'
  lcd.print("C");
  delay(500);                   //After 500ms ,the screen will be refreshed
}

```

If you have any questions after the class, please feel free to email education at makerspacect.com.
