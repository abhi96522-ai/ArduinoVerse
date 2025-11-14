import { CircuitBoard, Cpu, GraduationCap, Lightbulb, Projector, Bot, Thermometer, Gauge, Waves, Move3d, HardDrive, Book, GitBranch, Share2, Binary, Power, Monitor, SlidersHorizontal, Sun, Volume2, RadioReceiver, Magnet, Fingerprint, Fuel, Cable, Globe, GitCommitVertical, Pilcrow, Hash, Sigma, GitCompareArrows, Equal, Pointer, Repeat, Mouse, Joystick, Keyboard, Text, Rows, IterationCw, CaseLower, CaseUpper, Replace, Code, ArrowRightLeft, Radio, BetweenHorizontalStart, BetweenHorizontalEnd, Baseline, FileJson, SquareCode, Braces, FunctionSquare, Type, FileTerminal, Minus, Plus } from 'lucide-react';
import type { LucideIcon } from 'lucide-react';

export type LibraryItem = {
  title: string;
  description: string;
  slug: string;
  icon: LucideIcon;
  code: string;
}

export const CATEGORIES = [
  {
    title: 'Sensor Code Library',
    description: 'Explore code for a wide range of sensors.',
    href: '/sensors',
    icon: Thermometer,
  },
  {
    title: 'Output Device Library',
    description: 'Find snippets for LEDs, LCDs, motors and more.',
    href: '/outputs',
    icon: Lightbulb,
  },
  {
    title: 'Learning Path',
    description: 'Follow our tutorials to master Arduino programming.',
    href: '/learning',
    icon: GraduationCap,
  },
  {
    title: 'Arduino Reference',
    description: 'Quick reference for core concepts and functions.',
    href: '/reference',
    icon: GitBranch,
  },
];

export const SENSORS: LibraryItem[] = [
  {
    title: 'Temperature Sensor (DHT11)',
    description: 'Code for reading temperature and humidity from the popular DHT11 sensor.',
    slug: 'dht11',
    icon: Thermometer,
    code: `// Requires the DHT library by Adafruit
#include "DHT.h"

#define DHTPIN 2
#define DHTTYPE DHT11

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  Serial.println("DHT11 test!");
  dht.begin();
}

void loop() {
  delay(2000);
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.print("%\\t");
  Serial.print("Temperature: ");
  Serial.print(t);
  Serial.println(" *C");
}`
  },
  {
    title: 'Ultrasonic Sensor (HC-SR04)',
    description: 'Measure distance with the HC-SR04 ultrasonic sensor.',
    slug: 'hc-sr04',
    icon: Waves,
    code: `const int trigPin = 9;
const int echoPin = 10;

// Variables to store duration and distance
long duration; 
int distanceCm; 

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT); // Trig pin as output
  pinMode(echoPin, INPUT);  // Echo pin as input
  Serial.println("HC-SR04 Distance Sensor Ready.");
}

void loop() {
  // 1. Clear the trigPin by setting it LOW for a moment
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // 2. Send a 10-microsecond pulse to start the measurement
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // 3. Read the echo pin: returns the duration of the sound travel time
  duration = pulseIn(echoPin, HIGH);
  
  // 4. Calculate the distance (speed of sound: 343 m/s or 29 µs/cm)
  // Distance = (Time * Speed of Sound) / 2 (because it's a round trip)
  distanceCm = duration * 0.0343 / 2;
  
  Serial.print("Distance: ");
  Serial.print(distanceCm);
  Serial.println(" cm");
  
  delay(500);
}`
  },
  {
    title: 'Potentiometer',
    description: 'Read analog values from a potentiometer.',
    slug: 'potentiometer',
    icon: Gauge,
    code: `int potPin = A0;
int potValue = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  potValue = analogRead(potPin);
  Serial.println(potValue);
  delay(500);
}`
  },
  {
    title: 'PIR Motion Sensor (HC-SR501)',
    description: 'Detect movement with a Passive Infrared (PIR) sensor.',
    slug: 'pir-sensor',
    icon: Move3d,
    code: `const int pirPin = 2; // Connect PIR OUT to Digital Pin 2

void setup() {
  Serial.begin(9600);
  pinMode(pirPin, INPUT); // Set the pin as an input
  Serial.println("PIR Sensor Ready.");
}

void loop() {
  // Read the state of the pin
  int motionState = digitalRead(pirPin); 

  if (motionState == HIGH) {
    // Pin is HIGH when motion is detected
    Serial.println(">>> Motion Detected! <<<");
  } else {
    // Pin is LOW when no motion
    Serial.println("No Motion.");
  }
  
  // A small delay to keep the serial monitor readable
  delay(500); 
}`
  },
  {
    title: 'Accelerometer/Gyroscope (MPU6050)',
    description: 'Measure acceleration and rotation with the MPU6050 sensor using I2C.',
    slug: 'mpu6050',
    icon: Move3d,
    code: `// MPU6050 Accelerometer/Gyroscope Code (I2C)

// 1. Include required libraries
#include <Wire.h> // I2C communication library
#include <Adafruit_MPU6050.h> // MPU6050 library
#include <Adafruit_Sensor.h>  // Base sensor library

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200); // Use a faster baud rate for I2C data
  Wire.begin(); // Initialize I2C bus

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip! Check wiring.");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 Found and Ready!");
}

void loop() {
  // Create a structure to hold the sensor data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the data */
  Serial.print("Accel X: ");
  Serial.print(a.acceleration.x);
  Serial.print(" Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(" Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Gyro X: ");
  Serial.print(g.gyro.x);
  Serial.print(" Y: ");
  Serial.print(g.gyro.y);
  Serial.print(" Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");
  
  Serial.println("---");
  delay(100);
}`
  },
  {
    title: 'GPS Module (NEO-6M)',
    description: 'Read raw NMEA data from a GPS module using SoftwareSerial.',
    slug: 'gps-neo-6m',
    icon: Globe,
    code: `// GPS Module (NEO-6M) Code using SoftwareSerial

#include <SoftwareSerial.h>

// Define the pins for the software serial port (RX/TX)
// GPS TX goes to Arduino RX (pin 10)
// GPS RX goes to Arduino TX (pin 11)
SoftwareSerial gpsSerial(10, 11); 

void setup() {
  Serial.begin(9600); // For debugging output to PC
  gpsSerial.begin(9600); // For communication with the GPS module
  Serial.println("GPS Module Ready. Waiting for data...");
}

void loop() {
  // Read and relay raw GPS data (NMEA sentences) to the PC
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    Serial.write(c);
  }
}

// NOTE: For practical use, you'd integrate a library like 'TinyGPSPlus' 
// to parse this raw data into readable latitude/longitude.`
  },
  {
    title: 'Tilt Sensor / Reed Switch',
    description: 'Use a simple digital input sensor like a tilt switch or magnetic reed switch.',
    slug: 'tilt-reed-switch',
    icon: GitCommitVertical,
    code: `// Tilt Sensor or Reed Switch Code (Digital Input)

const int sensorPin = 2; // Connect to Digital Pin 2 (with a pull-up or pull-down resistor if needed)

void setup() {
  Serial.begin(9600);
  // Using INPUT_PULLUP is common to simplify wiring for simple switches
  // It uses the Arduino's internal pull-up resistor. The pin will be LOW when triggered.
  pinMode(sensorPin, INPUT_PULLUP); 
  Serial.println("Tilt/Reed Switch Sensor Ready.");
}

void loop() {
  int sensorState = digitalRead(sensorPin); 

  if (sensorState == LOW) {
    // LOW means the switch is closed (for INPUT_PULLUP mode)
    Serial.println("Sensor Triggered (Tilt/Magnet Detected)!");
  } else {
    // HIGH means the switch is open
    Serial.println("Sensor Open.");
  }
  
  delay(200); 
}`
  }
];

export const OUTPUTS: LibraryItem[] = [
    {
    title: 'Blink an LED',
    description: 'The "Hello, World!" of microcontrollers. Make an LED blink.',
    slug: 'blink-led',
    icon: Lightbulb,
    code: `int ledPin = 13;

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  digitalWrite(ledPin, HIGH);
  delay(1000);
  digitalWrite(ledPin, LOW);
  delay(1000);
}`
  },
  {
    title: 'Servo Motor',
    description: 'Control the position of a standard servo motor.',
    slug: 'servo-motor',
    icon: Bot,
    code: `// Requires the Servo library
#include <Servo.h>

Servo myServo;

void setup() {
  myServo.attach(9);
}

void loop() {
  for (int pos = 0; pos <= 180; pos += 1) {
    myServo.write(pos);
    delay(15);
  }
  for (int pos = 180; pos >= 0; pos -= 1) {
    myServo.write(pos);
    delay(15);
  }
}`
  },
  {
    title: 'LCD 16x2 Display',
    description: 'Display text on a 16x2 character Liquid Crystal Display.',
    slug: 'lcd-display',
    icon: Projector,
    code: `// Requires the LiquidCrystal library
#include <LiquidCrystal.h>

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  lcd.begin(16, 2);
  lcd.print("hello, world!");
}

void loop() {
  lcd.setCursor(0, 1);
  lcd.print(millis() / 1000);
  delay(500);
}`
  },
  {
    title: 'RGB LED Control',
    description: 'Create any color with a common cathode RGB LED.',
    slug: 'rgb-led',
    icon: Lightbulb,
    code: `int redPin = 9;
int greenPin = 10;
int bluePin = 11;

void setup() {
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
}

void loop() {
  setColor(255, 0, 0); // Red
  delay(1000);
  setColor(0, 255, 0); // Green
  delay(1000);
  setColor(0, 0, 255); // Blue
  delay(1000);
}

void setColor(int red, int green, int blue) {
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
}`
  },
  {
    title: 'PIR Motion Sensor (HC-SR501)',
    description: 'Detect movement with a Passive Infrared (PIR) sensor.',
    slug: 'pir-sensor',
    icon: Move3d,
    code: `const int pirPin = 2; // Connect PIR OUT to Digital Pin 2

void setup() {
  Serial.begin(9600);
  pinMode(pirPin, INPUT); // Set the pin as an input
  Serial.println("PIR Sensor Ready.");
}

void loop() {
  // Read the state of the pin
  int motionState = digitalRead(pirPin); 

  if (motionState == HIGH) {
    // Pin is HIGH when motion is detected
    Serial.println(">>> Motion Detected! <<<");
  } else {
    // Pin is LOW when no motion
    Serial.println("No Motion.");
  }
  
  // A small delay to keep the serial monitor readable
  delay(500); 
}`
  },
]

export const LEARNING: LibraryItem[] = [
  {
    title: 'Analog Read Serial',
    description: 'Read an analog input pin and print the value to the Serial Monitor.',
    slug: 'analog-read-serial',
    icon: FileTerminal,
    code: `/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
*/

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  // print out the value you read:
  Serial.println(sensorValue);
  delay(1);        // delay in between reads for stability
}`
  },
  {
    title: 'Bare Minimum',
    description: 'The basic sketch structure needed to start any Arduino program.',
    slug: 'bare-minimum',
    icon: SquareCode,
    code: `/*
  BareMinimum: The basic structure of an Arduino sketch.
  This is the minimum code needed to get a program to run.
*/

// the setup function runs once when you press reset or power the board
void setup() {
  // put your setup code here, to run once:
}

// the loop function runs over and over again forever
void loop() {
  // put your main code here, to run repeatedly:
}`
  },
  {
    title: 'Blink',
    description: 'The "Hello, World!" of microcontrollers. Make an LED blink.',
    slug: 'blink',
    icon: Lightbulb,
    code: `/*
  Blink
  Turns an LED on for one second, then off for one second, repeatedly.
*/

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}`
  },
  {
    title: 'Digital Read Serial',
    description: 'Read a digital input pin and print the state to the Serial Monitor.',
    slug: 'digital-read-serial',
    icon: FileTerminal,
    code: `/*
  DigitalReadSerial
  Reads a digital input on pin 2, prints the result to the Serial Monitor.
*/

// digital pin 2 has a pushbutton attached to it. Give it a name:
int pushButton = 2;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // make the pushbutton's pin an input:
  pinMode(pushButton, INPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input pin:
  int buttonState = digitalRead(pushButton);
  // print out the state of the button:
  Serial.println(buttonState);
  delay(1);        // delay in between reads for stability
}`
  },
  {
    title: 'Fading an LED',
    description: 'Demonstrates the use of analogWrite() to fade an LED.',
    slug: 'fading-led',
    icon: Lightbulb,
    code: `/*
  Fading
  This example shows how to fade an LED on pin 9 using the analogWrite() function.
*/

int led = 9;           // the PWM pin the LED is attached to
int brightness = 0;    // how bright the LED is
int fadeAmount = 5;    // how many points to fade the LED by

// the setup routine runs once when you press reset:
void setup() {
  // declare pin 9 to be an output:
  pinMode(led, OUTPUT);
}

// the loop routine runs over and over again forever:
void loop() {
  // set the brightness of pin 9:
  analogWrite(led, brightness);

  // change the brightness for next time through the loop:
  brightness = brightness + fadeAmount;

  // reverse the direction of the fading at the ends of the fade:
  if (brightness <= 0 || brightness >= 255) {
    fadeAmount = -fadeAmount;
  }
  // wait for 30 milliseconds to see the dimming effect
  delay(30);
}`
  },
  {
    title: 'Read Analog Voltage',
    description: 'Reads an analog input and converts the 0-1023 value to a voltage (0-5V).',
    slug: 'read-analog-voltage',
    icon: Gauge,
    code: `/*
  ReadAnalogVoltage
  Reads an analog input on pin A0, converts it to voltage, and prints the result to the Serial Monitor.
*/

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
  float voltage = sensorValue * (5.0 / 1023.0);
  // print out the value you read:
  Serial.println(voltage);
  delay(1);
}`
  },
  {
    title: 'Blink Without Delay',
    description: 'Blink an LED without using the delay() function, allowing other code to run.',
    slug: 'blink-without-delay',
    icon: IterationCw,
    code: `const int ledPin =  LED_BUILTIN;
int ledState = LOW;
unsigned long previousMillis = 0;
const long interval = 1000;

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    digitalWrite(ledPin, ledState);
  }
}`
  },
  {
    title: 'Button Debounce',
    description: 'Read a button press while filtering out noise (bouncing).',
    slug: 'button-debounce',
    icon: Mouse,
    code: `const int buttonPin = 2;
const int ledPin = 13;

int ledState = HIGH;
int buttonState;
int lastButtonState = LOW;

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, ledState);
}

void loop() {
  int reading = digitalRead(buttonPin);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == HIGH) {
        ledState = !ledState;
      }
    }
  }
  digitalWrite(ledPin, ledState);
  lastButtonState = reading;
}`
  },
  {
    title: 'State Change Detection',
    description: 'Detect when a button is pressed or released (edge detection).',
    slug: 'state-change-detection',
    icon: GitCompareArrows,
    code: `const int buttonPin = 2;
const int ledPin = 13;

int buttonPushCounter = 0;
int buttonState = 0;
int lastButtonState = 0;

void setup() {
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  buttonState = digitalRead(buttonPin);
  if (buttonState != lastButtonState) {
    if (buttonState == HIGH) {
      buttonPushCounter++;
      Serial.println("on");
      Serial.print("number of button pushes:  ");
      Serial.println(buttonPushCounter);
    } else {
      Serial.println("off");
    }
    delay(50);
  }
  lastButtonState = buttonState;

  if (buttonPushCounter % 4 == 0) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
}`
  },
  {
    title: 'Play a Melody',
    description: 'Play a melody with a piezo speaker using the tone() function.',
    slug: 'play-melody',
    icon: Volume2,
    code: `/*
  Melody
  Plays a melody with a piezo speaker.
*/
#include "pitches.h"

// notes in the melody:
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

void setup() {
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 8; thisNote++) {
    // to calculate the note duration, take one second divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(8, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(8);
  }
}

void loop() {
  // no need to repeat the melody.
}`
  },
  {
    title: 'Analog Input Smoothing',
    description: 'Smooth noisy analog sensor readings by averaging multiple samples.',
    slug: 'analog-smoothing',
    icon: Waves,
    code: `// Define the number of samples to keep track of. The higher the number,
// the more the readings will be smoothed, but the slower the output will
// respond to the input.
const int numReadings = 10;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

int inputPin = A0;

void setup() {
  Serial.begin(9600);
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
}

void loop() {
  total = total - readings[readIndex];
  readings[readIndex] = analogRead(inputPin);
  total = total + readings[readIndex];
  readIndex = readIndex + 1;

  if (readIndex >= numReadings) {
    readIndex = 0;
  }

  average = total / numReadings;
  Serial.println(average);
  delay(1);
}`
  },
  {
    title: 'Serial Call and Response',
    description: 'An example of handshaking between the Arduino and a computer.',
    slug: 'serial-call-response',
    icon: ArrowRightLeft,
    code: `const int ledPin = LED_BUILTIN;
int serialData;

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    serialData = Serial.read();
    Serial.print(serialData);
    Serial.print("\t");

    if (serialData == '1') {
      digitalWrite(ledPin, HIGH);
      Serial.println("LED ON");
    } else if (serialData == '0') {
      digitalWrite(ledPin, LOW);
      Serial.println("LED OFF");
    } else {
      Serial.println("Invalid command");
    }
  }
}`
  },
  {
    title: 'Using Arrays',
    description: 'A demonstration of how to use an array to store and access data.',
    slug: 'using-arrays',
    icon: Rows,
    code: `const int ledCount = 10;
int ledPins[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

void setup() {
  for (int thisLed = 0; thisLed < ledCount; thisLed++) {
    pinMode(ledPins[thisLed], OUTPUT);
  }
}

void loop() {
  for (int thisLed = 0; thisLed < ledCount; thisLed++) {
    digitalWrite(ledPins[thisLed], HIGH);
    delay(50);
    digitalWrite(ledPins[thisLed], LOW);
  }
}`
  },
  {
    title: 'For Loop (Knight Rider)',
    description: 'Use a for loop to create a "Knight Rider" style LED chaser.',
    slug: 'for-loop-knight-rider',
    icon: IterationCw,
    code: `int pins[] = {2, 3, 4, 5, 6, 7};
int numPins = 6;

void setup() {
  for (int i=0; i<numPins; i++) {
    pinMode(pins[i], OUTPUT);
  }
}

void loop() {
  for (int i=0; i<numPins; i++) {
    digitalWrite(pins[i], HIGH);
    delay(50);
    digitalWrite(pins[i], LOW);
  }
  for (int i=numPins-1; i>=0; i--) {
    digitalWrite(pins[i], HIGH);
    delay(50);
    digitalWrite(pins[i], LOW);
  }
}`
  },
  {
    title: 'Switch Case Statement',
    description: 'Use a switch-case statement to control program flow based on sensor input.',
    slug: 'switch-case',
    icon: GitBranch,
    code: `void setup() {
  Serial.begin(9600);
}

void loop() {
  int sensorReading = analogRead(A0);
  
  switch (sensorReading / 100) {
    case 0:
      Serial.println("Very dark");
      break;
    case 1:
    case 2:
      Serial.println("Dark");
      break;
    case 3:
    case 4:
      Serial.println("Medium");
      break;
    case 5:
    case 6:
      Serial.println("Bright");
      break;
    default:
      Serial.println("Very bright");
      break;
  }
  delay(500);
}`
  },
  {
    title: 'ADXL3xx Accelerometer',
    description: 'Read data from an ADXL3xx series accelerometer.',
    slug: 'adxl3xx-accelerometer',
    icon: Move3d,
    code: `const int xPin = A0;
const int yPin = A1;
const int zPin = A2;

void setup() {
  Serial.begin(9600);
}

void loop() {
  int x = analogRead(xPin);
  int y = analogRead(yPin);
  int z = analogRead(zPin);
  
  Serial.print("X: ");
  Serial.print(x);
  Serial.print("\\tY: ");
  Serial.print(y);
  Serial.print("\\tZ: ");
  Serial.println(z);
  
  delay(100);
}`
  },
  {
    title: 'Ping Ultrasonic Sensor',
    description: 'Measure distance with a Parallax PING)))™ ultrasonic sensor.',
    slug: 'ping-sensor',
    icon: Waves,
    code: `const int pingPin = 7;

void setup() {
  Serial.begin(9600);
}

void loop() {
  long duration, inches, cm;
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  inches = duration / 74 / 2;
  cm = duration / 29 / 2;

  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();

  delay(100);
}`
  },
  {
    title: 'String Manipulation',
    description: 'A collection of examples for using the String object.',
    slug: 'string-manipulation',
    icon: Text,
    code: `void setup() {
  Serial.begin(9600);
  
  String str1 = "Hello";
  String str2 = " World";
  String str3 = str1 + str2; // Concatenation
  Serial.println(str3);
  
  str3.toUpperCase();
  Serial.println(str3);
  
  if (str3.startsWith("HELLO")) {
    Serial.println("String starts with HELLO");
  }
  
  int index = str3.indexOf('W');
  Serial.print("Index of 'W': ");
  Serial.println(index);
  
  String sub = str3.substring(6);
  Serial.println(sub);
}

void loop() {}`
  },
  {
    title: 'Keyboard and Mouse Control',
    description: 'Control your computer\'s mouse and keyboard with an Arduino Leonardo, Micro, or Due.',
    slug: 'keyboard-mouse-control',
    icon: Keyboard,
    code: `/*
  KeyboardAndMouseControl
  Controls the mouse and keyboard from a Leonardo, Micro or Due.
*/
#include "Keyboard.h"
#include "Mouse.h"

void setup() {
  // initialize the controls
  Keyboard.begin();
  Mouse.begin();
}

void loop() {
  // move the mouse in a square
  Mouse.move(50, 0, 0);
  delay(500);
  Mouse.move(0, 50, 0);
  delay(500);
  Mouse.move(-50, 0, 0);
  delay(500);
  Mouse.move(0, -50, 0);
  delay(500);

  // type a message
  Keyboard.print("Hello, world!");
  delay(1000);
}`
  }
];
export const REFERENCE: LibraryItem[] = [
    {
    title: 'Arduino Reference: Variables, Structures & Functions',
    description: 'A reference for core programming concepts like variables, data structures, and functions.',
    slug: 'arduino-reference',
    icon: GitBranch,
    code: `// --- Variables ---
// Variables are used to store data.
int integerVar = 123;          // Stores whole numbers
float floatVar = 3.14;         // Stores numbers with decimal points
char charVar = 'A';            // Stores a single character
String stringVar = "Hello!";   // Stores a sequence of characters (text)
bool boolVar = true;           // Stores true or false

// --- Structures ---
// Structures (structs) group related variables into one place.
struct SensorReading {
  int id;
  float temperature;
  float humidity;
  bool isValid;
};

// Create an instance of the structure
SensorReading myReading;

// --- Functions ---
// Functions are reusable blocks of code that perform a specific task.

// This function takes two integers as input and returns their sum.
int add(int a, int b) {
  return a + b;
}

// This function does not return a value (void). It prints sensor data.
void printReading(SensorReading reading) {
  Serial.print("Reading ID: ");
  Serial.println(reading.id);
  if (reading.isValid) {
    Serial.print("  Temp: ");
    Serial.print(reading.temperature);
    Serial.println(" C");
    Serial.print("  Humidity: ");
    Serial.print(reading.humidity);
    Serial.println("%");
  } else {
    Serial.println("  (Invalid Reading)");
  }
}

void setup() {
  Serial.begin(9600);
  
  // Using the function
  int result = add(5, 10);
  Serial.print("5 + 10 = ");
  Serial.println(result);
  
  Serial.println("---");

  // Using the structure
  myReading.id = 1;
  myReading.temperature = 25.5;
  myReading.humidity = 45.8;
  myReading.isValid = true;
  
  // Using the function to print the struct data
  printReading(myReading);
}

void loop() {
  // The main loop is empty as this is a reference example.
}`
  }
];
