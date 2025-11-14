import { CircuitBoard, Cpu, GraduationCap, Lightbulb, Projector, Bot, Thermometer, Gauge, Waves, Move3d, HardDrive, Book, GitBranch, Share2, Binary, Power, Monitor, SlidersHorizontal, Sun, Volume2, RadioReceiver, Magnet, Fingerprint, Fuel, Cable, Globe, GitCommitVertical, Pilcrow, Hash, Sigma, GitCompareArrows, Equal, Pointer, Repeat, Mouse, Joystick, Keyboard, Text, Rows, IterationCw, CaseLower, CaseUpper, Replace, Code, ArrowRightLeft, Radio, BetweenHorizontalStart, BetweenHorizontalEnd, Baseline, FileJson, SquareCode, Braces, FunctionSquare, Type, FileTerminal, Minus, Plus } from 'lucide-react';
import type { LucideIcon } from 'lucide-react';

export type LibraryItem = {
  title: string;
  description: string;
  slug: string;
  icon: LucideIcon;
  code: string;
  category?: string;
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
    category: 'Basics',
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
    category: 'Basics',
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
    category: 'Basics',
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
    category: 'Basics',
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
    category: 'Basics',
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
    category: 'Basics',
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
    category: 'Digital',
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
    title: 'Button',
    description: 'How to Wire and Program a Button.',
    slug: 'button',
    icon: Mouse,
    category: 'Digital',
    code: `const int buttonPin = 2;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status

void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
}

void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(ledPin, HIGH);
  } else {
    // turn LED off:
    digitalWrite(ledPin, LOW);
  }
}`
  },
  {
    title: 'Button Debounce',
    description: 'Read a button press while filtering out noise (bouncing).',
    slug: 'button-debounce',
    icon: Mouse,
    category: 'Digital',
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
    title: 'Input Pull-up Serial',
    description: 'Demonstrates the use of INPUT_PULLUP with pinMode().',
    slug: 'input-pullup-serial',
    icon: Power,
    category: 'Digital',
    code: `const int ledPin = 13;
const int inputPin = 2;

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  pinMode(inputPin, INPUT_PULLUP);
}

void loop() {
  int sensorVal = digitalRead(inputPin);
  Serial.println(sensorVal);

  if (sensorVal == HIGH) {
    digitalWrite(ledPin, LOW);
  } else {
    digitalWrite(ledPin, HIGH);
  }
}`
  },
  {
    title: 'State Change Detection',
    description: 'Detect when a button is pressed or released (edge detection).',
    slug: 'state-change-detection',
    icon: GitCompareArrows,
    category: 'Digital',
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
    title: 'Simple Keyboard Tone',
    description: 'Simple keyboard using the tone() function.',
    slug: 'simple-keyboard-tone',
    icon: Keyboard,
    category: 'Digital',
    code: `#include "pitches.h"

const int buttonCount = 4;
const int buttonPins[buttonCount] = {2, 3, 4, 5};
const int tones[buttonCount] = {NOTE_C4, NOTE_D4, NOTE_E4, NOTE_F4};
const int speakerPin = 8;

void setup() {
  for (int i = 0; i < buttonCount; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }
}

void loop() {
  for (int i = 0; i < buttonCount; i++) {
    if (digitalRead(buttonPins[i]) == LOW) {
      tone(speakerPin, tones[i]);
    }
  }
  
  // A brief moment of silence to separate notes
  if (digitalRead(buttonPins[0]) == HIGH && digitalRead(buttonPins[1]) == HIGH && 
      digitalRead(buttonPins[2]) == HIGH && digitalRead(buttonPins[3]) == HIGH) {
    noTone(speakerPin);
  }
}`
  },
  {
    title: 'Play a Melody',
    description: 'Play a melody with a piezo speaker using the tone() function.',
    slug: 'play-melody',
    icon: Volume2,
    category: 'Digital',
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
    title: 'Tone on Multiple Speakers',
    description: 'Play tones on multiple speakers.',
    slug: 'tone-multiple-speakers',
    icon: Volume2,
    category: 'Digital',
    code: `const int speakerPin1 = 8;
const int speakerPin2 = 9;

void setup() {
}

void loop() {
  tone(speakerPin1, 262, 500); // C4 on speaker 1
  delay(500);
  noTone(speakerPin1);

  tone(speakerPin2, 440, 500); // A4 on speaker 2
  delay(500);
  noTone(speakerPin2);

  tone(speakerPin1, 523, 250); // C5 on speaker 1
  tone(speakerPin2, 392, 250); // G4 on speaker 2
  delay(250);
  noTone(speakerPin1);
  noTone(speakerPin2);

  delay(1000);
}`
  },
    {
    title: 'Pitch Follower',
    description: 'Use the tone() function to follow an analog input.',
    slug: 'pitch-follower',
    icon: Volume2,
    category: 'Digital',
    code: `const int sensorPin = A0;
const int speakerPin = 8;

void setup() {
  Serial.begin(9600);
}

void loop() {
  int sensorValue = analogRead(sensorPin);
  Serial.println(sensorValue);
  
  // Map the analog value (0-1023) to a frequency range (100-1000 Hz)
  int frequency = map(sensorValue, 0, 1023, 100, 1000);
  
  // Play the tone
  tone(speakerPin, frequency, 20);
  
  delay(10);
}`
  },
  {
    title: 'Analog In, Out Serial',
    description: 'Read an analog input, map its value, and use it to control an LED.',
    slug: 'analog-in-out-serial',
    icon: ArrowRightLeft,
    category: 'Analog',
    code: `const int analogInPin = A0;
const int analogOutPin = 9;

int sensorValue = 0;
int outputValue = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  sensorValue = analogRead(analogInPin);
  outputValue = map(sensorValue, 0, 1023, 0, 255);
  analogWrite(analogOutPin, outputValue);

  Serial.print("sensor = ");
  Serial.print(sensorValue);
  Serial.print("\t output = ");
  Serial.println(outputValue);

  delay(2);
}`
  },
  {
    title: 'Analog Input',
    description: 'Read a potentiometer and use its value to control an LED.',
    slug: 'analog-input',
    icon: Gauge,
    category: 'Analog',
    code: `const int sensorPin = A0;
const int ledPin = 9;
int sensorValue = 0;

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  sensorValue = analogRead(sensorPin);
  digitalWrite(ledPin, HIGH);
  delay(sensorValue);
  digitalWrite(ledPin, LOW);
  delay(sensorValue);
}`
  },
  {
    title: 'Analog Write Mega',
    description: 'Fade 12 LEDs on and off, one by one, on an Arduino Mega.',
    slug: 'analog-write-mega',
    icon: Lightbulb,
    category: 'Analog',
    code: `void setup() {
  for (int thisPin = 2; thisPin < 14; thisPin++) {
    pinMode(thisPin, OUTPUT);
  }
}

void loop() {
  for (int thisPin = 2; thisPin < 14; thisPin++) {
    for (int brightness = 0; brightness < 255; brightness++) {
      analogWrite(thisPin, brightness);
      delay(2);
    }
    for (int brightness = 255; brightness >= 0; brightness--) {
      analogWrite(thisPin, brightness);
      delay(2);
    }
    delay(100);
  }
}`
  },
  {
    title: 'Calibrate Sensor',
    description: 'Define a maximum and minimum for sensor readings during a calibration period.',
    slug: 'calibrate-sensor',
    icon: SlidersHorizontal,
    category: 'Analog',
    code: `const int sensorPin = A0;
const int ledPin = 9;
int sensorMin = 1023;
int sensorMax = 0;

void setup() {
  pinMode(ledPin, OUTPUT);
  while (millis() < 5000) {
    int sensorVal = analogRead(sensorPin);
    if (sensorVal > sensorMax) {
      sensorMax = sensorVal;
    }
    if (sensorVal < sensorMin) {
      sensorMin = sensorVal;
    }
  }
  digitalWrite(ledPin, HIGH);
}

void loop() {
  int sensorVal = analogRead(sensorPin);
  int range = map(sensorVal, sensorMin, sensorMax, 0, 255);
  range = constrain(range, 0, 255);
  analogWrite(ledPin, range);
}`
  },
  {
    title: 'Fading',
    description: 'Demonstrates the use of analogWrite() to fade an LED.',
    slug: 'fading',
    icon: Lightbulb,
    category: 'Analog',
    code: `int ledPin = 9;

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  for (int fadeValue = 0 ; fadeValue <= 255; fadeValue += 5) {
    analogWrite(ledPin, fadeValue);
    delay(30);
  }
  for (int fadeValue = 255 ; fadeValue >= 0; fadeValue -= 5) {
    analogWrite(ledPin, fadeValue);
    delay(30);
  }
}`
  },
  {
    title: 'Analog Smoothing',
    description: 'Smooth noisy analog sensor readings by averaging multiple samples.',
    slug: 'analog-smoothing',
    icon: Waves,
    category: 'Analog',
    code: `const int numReadings = 10;

int readings[numReadings];
int readIndex = 0;
int total = 0;
int average = 0;

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
    title: 'ASCII Table',
    description: 'Demonstrates Arduino\'s understanding of ASCII characters.',
    slug: 'ascii-table',
    icon: Text,
    category: 'Communication',
    code: `void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  Serial.println("ASCII Table ~ Character Map");
}

int thisByte = 33;

void loop() {
  Serial.write(thisByte);
  Serial.print(", dec: ");
  Serial.print(thisByte);
  Serial.print(", hex: ");
  Serial.print(thisByte, HEX);
  Serial.print(", oct: ");
  Serial.print(thisByte, OCT);
  Serial.print(", bin: ");
  Serial.println(thisByte, BIN);

  if (thisByte == 126) {
    while (true) {
      continue;
    }
  }
  thisByte++;
}`
  },
    {
    title: 'LED Dimmer',
    description: 'Control an LED with a byte of data from the serial port.',
    slug: 'led-dimmer',
    icon: Sun,
    category: 'Communication',
    code: `const int ledPin = 9;
int brightness = 0;

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    brightness = Serial.read();
    analogWrite(ledPin, brightness);
  }
}`
  },
  {
    title: 'Graph with Processing',
    description: 'Send data to the computer and graph it with Processing.',
    slug: 'graph-with-processing',
    icon: Monitor,
    category: 'Communication',
    code: `void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println(analogRead(A0));
  delay(10);
}`
  },
    {
    title: 'MIDI Player',
    description: 'Play MIDI notes on a MIDI-enabled device.',
    slug: 'midi-player',
    icon: Volume2,
    category: 'Communication',
    code: `void setup() {
  Serial.begin(31250); // Standard MIDI baud rate
}

void loop() {
  // Note on, channel 1, middle C (60), velocity 127
  noteOn(0x90, 60, 127);
  delay(1000);
  // Note off, channel 1, middle C, velocity 0
  noteOn(0x80, 60, 0);
  delay(1000);
}

void noteOn(int cmd, int pitch, int velocity) {
  Serial.write(cmd);
  Serial.write(pitch);
  Serial.write(velocity);
}`
  },
  {
    title: 'Multiple Serial Ports (Mega)',
    description: 'Use multiple serial ports on an Arduino Mega.',
    slug: 'multi-serial-mega',
    icon: Share2,
    category: 'Communication',
    code: `void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial.println("Hello, Computer!");
  Serial1.println("Hello, other device!");
}

void loop() {
  if (Serial.available()) {
    int inByte = Serial.read();
    Serial1.write(inByte);
  }
  if (Serial1.available()) {
    int inByte = Serial1.read();
    Serial.write(inByte);
  }
}`
  },
    {
    title: 'Physical Pixel',
    description: 'Turn an LED on and off based on serial input.',
    slug: 'physical-pixel',
    icon: Lightbulb,
    category: 'Communication',
    code: `const int ledPin = 13;
int inByte;

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    inByte = Serial.read();
    if (inByte == 'H') {
      digitalWrite(ledPin, HIGH);
    }
    if (inByte == 'L') {
      digitalWrite(ledPin, LOW);
    }
  }
}`
  },
  {
    title: 'Read ASCII String',
    description: 'Parse a comma-separated string of integers to fade an LED.',
    slug: 'read-ascii-string',
    icon: Text,
    category: 'Communication',
    code: `String inString = "";

void setup() {
  Serial.begin(9600);
}

void loop() {
  while (Serial.available() > 0) {
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      inString += (char)inChar;
    }
    if (inChar == '\\n') {
      int value = inString.toInt();
      analogWrite(9, value);
      Serial.print("Value: ");
      Serial.println(value);
      inString = "";
    }
  }
}`
  },
  {
    title: 'Serial Call and Response',
    description: 'An example of handshaking between the Arduino and a computer.',
    slug: 'serial-call-response',
    icon: ArrowRightLeft,
    category: 'Communication',
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
    title: 'Serial Call ASCII',
    description: 'Handshaking with ASCII-encoded output.',
    slug: 'serial-call-ascii',
    icon: ArrowRightLeft,
    category: 'Communication',
    code: `const int analogPin = A0;
int inByte = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    inByte = Serial.read();
    if (inByte == 'r') {
      int sensorValue = analogRead(analogPin);
      Serial.println(sensorValue);
    }
  }
}`
  },
  {
    title: 'SerialEvent',
    description: 'Demonstrates use of the SerialEvent() function.',
    slug: 'serial-event',
    icon: RadioReceiver,
    category: 'Communication',
    code: `String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);
  inputString.reserve(200);
}

void loop() {
  if (stringComplete) {
    Serial.println(inputString);
    inputString = "";
    stringComplete = false;
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\\n') {
      stringComplete = true;
    }
  }
}`
  },
  {
    title: 'SerialPassthrough',
    description: 'Pass data from SoftwareSerial to HardwareSerial.',
    slug: 'serial-passthrough',
    icon: Share2,
    category: 'Communication',
    code: `#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11);

void setup() {
  Serial.begin(57600);
  mySerial.begin(4800);
}

void loop() {
  if (mySerial.available()) {
    Serial.write(mySerial.read());
  }
  if (Serial.available()) {
    mySerial.write(Serial.read());
  }
}`
  },
  {
    title: 'Virtual Color Mixer',
    description: 'Mix colors in Processing and see the result on an RGB LED.',
    slug: 'virtual-color-mixer',
    icon: Lightbulb,
    category: 'Communication',
    code: `const int redPin = 9;
const int greenPin = 10;
const int bluePin = 11;

void setup() {
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 2) {
    int red = Serial.parseInt();
    int green = Serial.parseInt();
    int blue = Serial.parseInt();
    if (Serial.read() == '\\n') {
      setColor(red, green, blue);
    }
  }
}

void setColor(int red, int green, int blue) {
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
}`
  },
  {
    title: 'Using Arrays',
    description: 'A demonstration of how to use an array to store and access data.',
    slug: 'using-arrays',
    icon: Rows,
    category: 'Control Structures',
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
    category: 'Control Structures',
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
    category: 'Control Structures',
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
    title: 'Switch Case 2',
    description: 'Use a switch-case statement with serial input.',
    slug: 'switch-case-2',
    icon: GitBranch,
    category: 'Control Structures',
    code: `void setup() {
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    char inChar = Serial.read();
    switch (inChar) {
      case 'a':
        Serial.println("Option A selected");
        break;
      case 'b':
        Serial.println("Option B selected");
        break;
      case 'c':
        Serial.println("Option C selected");
        break;
      default:
        Serial.println("Invalid option");
        break;
    }
  }
}`
  },
  {
    title: 'While Loop',
    description: 'Use a while loop to calibrate a sensor.',
    slug: 'while-loop',
    icon: IterationCw,
    category: 'Control Structures',
    code: `int sensorMin = 1023;
int sensorMax = 0;

void setup() {
  Serial.begin(9600);
  while (millis() < 5000) {
    int sensorVal = analogRead(A0);
    if (sensorVal > sensorMax) {
      sensorMax = sensorVal;
    }
    if (sensorVal < sensorMin) {
      sensorMin = sensorVal;
    }
  }
  Serial.print("Min: "); Serial.println(sensorMin);
  Serial.print("Max: "); Serial.println(sensorMax);
}

void loop() {}`
  },
  {
    title: 'If Statement',
    description: 'Use an if statement to make decisions.',
    slug: 'if-statement',
    icon: GitBranch,
    category: 'Control Structures',
    code: `int inputPin = 2;
int ledPin = 13;

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(inputPin, INPUT);
}

void loop() {
  if (digitalRead(inputPin) == HIGH) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
}`
  },
  {
    title: 'ADXL3xx Accelerometer',
    description: 'Read data from an ADXL3xx series accelerometer.',
    slug: 'adxl3xx-accelerometer',
    icon: Move3d,
    category: 'Sensors',
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
    title: 'Knock Sensor',
    description: 'Detect a knock with a piezo element.',
    slug: 'knock-sensor',
    icon: Volume2,
    category: 'Sensors',
    code: `const int knockSensor = A0;
const int threshold = 100;
int sensorReading = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  sensorReading = analogRead(knockSensor);
  if (sensorReading >= threshold) {
    Serial.println("Knock detected!");
  }
  delay(100);
}`
  },
  {
    title: 'Memsic 2125 Accelerometer',
    description: 'Read data from a Memsic 2125 dual-axis accelerometer.',
    slug: 'memsic-accelerometer',
    icon: Move3d,
    category: 'Sensors',
    code: `const int xPin = 2;
const int yPin = 4;

void setup() {
  Serial.begin(9600);
  pinMode(xPin, INPUT);
  pinMode(yPin, INPUT);
}

void loop() {
  long xPulse = pulseIn(xPin, HIGH);
  long yPulse = pulseIn(yPin, HIGH);
  
  Serial.print("X: ");
  Serial.print(xPulse);
  Serial.print("\tY: ");
  Serial.println(yPulse);
  
  delay(100);
}`
  },
  {
    title: 'Ping Ultrasonic Sensor',
    description: 'Measure distance with a Parallax PING)))™ ultrasonic sensor.',
    slug: 'ping-sensor',
    icon: Waves,
    category: 'Sensors',
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
    title: 'LED Bar Graph',
    description: 'Control an LED bar graph with an analog input.',
    slug: 'led-bar-graph',
    icon: Monitor,
    category: 'Display',
    code: `const int analogPin = A0;
const int ledCount = 10;
int ledPins[] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

void setup() {
  for (int thisLed = 0; thisLed < ledCount; thisLed++) {
    pinMode(ledPins[thisLed], OUTPUT);
  }
}

void loop() {
  int sensorReading = analogRead(analogPin);
  int ledLevel = map(sensorReading, 0, 1023, 0, ledCount);

  for (int thisLed = 0; thisLed < ledCount; thisLed++) {
    if (thisLed < ledLevel) {
      digitalWrite(ledPins[thisLed], HIGH);
    }
    else {
      digitalWrite(ledPins[thisLed], LOW);
    }
  }
}`
  },
    {
    title: '8x8 LED Matrix',
    description: 'Control an 8x8 matrix of LEDs.',
    slug: '8x8-led-matrix',
    icon: Monitor,
    category: 'Display',
    code: `// Requires the LedControl library
#include "LedControl.h"

// LedControl(dataPin, clockPin, csPin, numDevices)
LedControl lc = LedControl(12, 11, 10, 1);

unsigned long delayTime = 100;

void setup() {
  lc.shutdown(0, false);
  lc.setIntensity(0, 8);
  lc.clearDisplay(0);
}

void loop() {
  // Example pattern
  byte smile[8] = {0x3C, 0x42, 0x99, 0xA5, 0x81, 0xA5, 0x42, 0x3C};
  for (int i = 0; i < 8; i++) {
    lc.setRow(0, i, smile[i]);
  }
}`
  },
  {
    title: 'String Character Analysis',
    description: 'Analyze characters in a String object.',
    slug: 'string-character-analysis',
    icon: Text,
    category: 'Strings',
    code: `String myString = "Hello! 123";

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < myString.length(); i++) {
    char c = myString.charAt(i);
    Serial.print("Character '");
    Serial.print(c);
    Serial.print("' is a ");
    
    if (isAlphaNumeric(c)) Serial.print("AlphaNumeric ");
    if (isAlpha(c)) Serial.print("Alphabetic ");
    if (isAscii(c)) Serial.print("ASCII ");
    if (isWhitespace(c)) Serial.print("Whitespace ");
    if (isControl(c)) Serial.print("Control ");
    if (isDigit(c)) Serial.print("Digit ");
    if (isGraph(c)) Serial.print("Graph ");
    if (isLowerCase(c)) Serial.print("LowerCase ");
    if (isPrintable(c)) Serial.print("Printable ");
    if (isPunct(c)) Serial.print("Punctuation ");
    if (isSpace(c)) Serial.print("Space ");
    if (isUpperCase(c)) Serial.print("UpperCase ");
    if (isHexadecimalDigit(c)) Serial.print("HexDigit ");
    
    Serial.println();
  }
}

void loop() {}`
  },
  {
    title: 'String Addition Operator',
    description: 'Combine strings using the addition operator (+).',
    slug: 'string-addition-operator',
    icon: Plus,
    category: 'Strings',
    code: `void setup() {
  Serial.begin(9600);
  
  String str1 = "Hello, ";
  String str2 = "world!";
  String message = str1 + str2 + " From Arduino.";
  Serial.println(message);
  
  int sensorValue = analogRead(A0);
  String sensorString = "Sensor value: " + String(sensorValue);
  Serial.println(sensorString);
}

void loop() {}`
  },
  {
    title: 'String Appending Operators',
    description: 'Add to a string using the += operator and concat() method.',
    slug: 'string-appending-operators',
    icon: Plus,
    category: 'Strings',
    code: `void setup() {
  Serial.begin(9600);
  
  String myString = "A string: ";
  myString += "more text";
  Serial.println(myString);
  
  myString.concat(" even more");
  Serial.println(myString);
}

void loop() {}`
  },
  {
    title: 'String Case Changes',
    description: 'Change the case of a string.',
    slug: 'string-case-changes',
    icon: CaseUpper,
    category: 'Strings',
    code: `void setup() {
  Serial.begin(9600);
  String myString = "Mixed Case";
  
  Serial.println(myString);
  
  myString.toLowerCase();
  Serial.println(myString);

  myString.toUpperCase();
  Serial.println(myString);
}

void loop() {}`
  },
  {
    title: 'String Character Functions',
    description: 'Get, set, and remove characters from a string.',
    slug: 'string-character-functions',
    icon: Text,
    category: 'Strings',
    code: `void setup() {
  Serial.begin(9600);
  
  String myString = "abcde";
  Serial.println(myString);
  
  char firstChar = myString.charAt(0);
  Serial.print("First char: ");
  Serial.println(firstChar);
  
  myString.setCharAt(0, 'X');
  Serial.println(myString);
  
  myString.remove(2, 2); // remove 2 chars starting at index 2
  Serial.println(myString);
}

void loop() {}`
  },
  {
    title: 'String Comparison Operators',
    description: 'Compare strings for equality and order.',
    slug: 'string-comparison-operators',
    icon: Equal,
    category: 'Strings',
    code: `void setup() {
  Serial.begin(9600);
  
  String str1 = "apple";
  String str2 = "banana";
  String str3 = "apple";

  if (str1 == str3) Serial.println("str1 equals str3");
  if (str1 != str2) Serial.println("str1 does not equal str2");
  if (str1 < str2) Serial.println("str1 comes before str2 alphabetically");
  if (str2 > str1) Serial.println("str2 comes after str1 alphabetically");
}

void loop() {}`
  },
  {
    title: 'String Object Constructors',
    description: 'Different ways to create a String object.',
    slug: 'string-constructors',
    icon: Braces,
    category: 'Strings',
    code: `void setup() {
  Serial.begin(9600);
  
  String str1 = "This is a string";
  String str2 = String('a');
  String str3 = String("This is also a string");
  String str4 = String(str1 + " and so is this");
  String str5 = String(13, DEC);
  String str6 = String(1.23456, 3);
  
  Serial.println(str1);
  Serial.println(str2);
  Serial.println(str3);
  Serial.println(str4);
  Serial.println(str5);
  Serial.println(str6);
}

void loop() {}`
  },
  {
    title: 'String indexOf()',
    description: 'Find the position of a character or substring.',
    slug: 'string-indexof',
    icon: BetweenHorizontalStart,
    category: 'Strings',
    code: `void setup() {
  Serial.begin(9600);
  
  String myString = "The quick brown fox";
  
  int index1 = myString.indexOf('q');
  int index2 = myString.indexOf("brown");
  int index3 = myString.lastIndexOf('o');
  
  Serial.print("Index of 'q': "); Serial.println(index1);
  Serial.print("Index of 'brown': "); Serial.println(index2);
  Serial.print("Last index of 'o': "); Serial.println(index3);
}

void loop() {}`
  },
  {
    title: 'String length()',
    description: 'Get the length of a string.',
    slug: 'string-length',
    icon: BetweenHorizontalEnd,
    category: 'Strings',
    code: `void setup() {
  Serial.begin(9600);
  
  String myString = "Hello";
  Serial.print("Length of '");
  Serial.print(myString);
  Serial.print("': ");
  Serial.println(myString.length());
}

void loop() {}`
  },
  {
    title: 'String length() and trim()',
    description: 'Use length() and trim() to manage string size.',
    slug: 'string-length-trim',
    icon: BetweenHorizontalEnd,
    category: 'Strings',
    code: `void setup() {
  Serial.begin(9600);
  
  String myString = "  Hello  ";
  Serial.print("Original length: ");
  Serial.println(myString.length());

  myString.trim();
  
  Serial.print("Trimmed length: ");
  Serial.println(myString.length());
  Serial.println(myString);
}

void loop() {}`
  },
  {
    title: 'String replace()',
    description: 'Replace characters or substrings in a string.',
    slug: 'string-replace',
    icon: Replace,
    category: 'Strings',
    code: `void setup() {
  Serial.begin(9600);
  
  String myString = "The quick brown fox";
  Serial.println(myString);
  
  myString.replace("brown", "red");
  Serial.println(myString);
}

void loop() {}`
  },
  {
    title: 'String startsWith() and endsWith()',
    description: 'Check if a string starts or ends with a specific sequence.',
    slug: 'string-startswith-endswith',
    icon: Baseline,
    category: 'Strings',
    code: `void setup() {
  Serial.begin(9600);
  
  String myString = "FileName.txt";
  
  if (myString.startsWith("File")) {
    Serial.println("String starts with 'File'");
  }
  
  if (myString.endsWith(".txt")) {
    Serial.println("String ends with '.txt'");
  }
}

void loop() {}`
  },
  {
    title: 'String substring()',
    description: 'Extract a portion of a string.',
    slug: 'string-substring',
    icon: Minus,
    category: 'Strings',
    code: `void setup() {
  Serial.begin(9600);
  
  String myString = "The quick brown fox";
  
  String sub1 = myString.substring(4); // from index 4 to end
  String sub2 = myString.substring(4, 9); // from index 4 to 8
  
  Serial.println(sub1);
  Serial.println(sub2);
}

void loop() {}`
  },
  {
    title: 'String toInt()',
    description: 'Convert a string to an integer.',
    slug: 'string-toint',
    icon: FileJson,
    category: 'Strings',
    code: `void setup() {
  Serial.begin(9600);
  
  String myString = "123";
  int myInt = myString.toInt();
  
  Serial.print("Integer value: ");
  Serial.println(myInt);
  Serial.println(myInt * 2);
}

void loop() {}`
  },
  {
    title: 'Button Mouse Control',
    description: 'Control the mouse cursor with pushbuttons.',
    slug: 'button-mouse-control',
    icon: Mouse,
    category: 'USB',
    code: `// Requires Leonardo, Micro, or Due board
#include "Mouse.h"

const int upButton = 2;
const int downButton = 3;
const int leftButton = 4;
const int rightButton = 5;

int range = 5;

void setup() {
  Mouse.begin();
  pinMode(upButton, INPUT_PULLUP);
  pinMode(downButton, INPUT_PULLUP);
  pinMode(leftButton, INPUT_PULLUP);
  pinMode(rightButton, INPUT_PULLUP);
}

void loop() {
  int x = 0;
  int y = 0;
  if (digitalRead(upButton) == LOW) y = -range;
  if (digitalRead(downButton) == LOW) y = range;
  if (digitalRead(leftButton) == LOW) x = -range;
  if (digitalRead(rightButton) == LOW) x = range;
  
  Mouse.move(x, y, 0);
  delay(10);
}`
  },
  {
    title: 'Joystick Mouse Control',
    description: 'Control the mouse with a joystick.',
    slug: 'joystick-mouse-control',
    icon: Joystick,
    category: 'USB',
    code: `// Requires Leonardo, Micro, or Due board
#include "Mouse.h"

const int xAxis = A0;
const int yAxis = A1;

void setup() {
  Mouse.begin();
}

void loop() {
  int xReading = analogRead(xAxis);
  int yReading = analogRead(yAxis);
  
  int xMove = map(xReading, 0, 1023, -10, 10);
  int yMove = map(yReading, 0, 1023, -10, 10);
  
  Mouse.move(xMove, yMove, 0);
  delay(10);
}`
  },
  {
    title: 'Keyboard and Mouse Control',
    description: 'Control your computer\'s mouse and keyboard with an Arduino Leonardo, Micro, or Due.',
    slug: 'keyboard-mouse-control',
    icon: Keyboard,
    category: 'USB',
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
  },
    {
    title: 'Keyboard Logout',
    description: 'Logs out of the current computer session.',
    slug: 'keyboard-logout',
    icon: Keyboard,
    category: 'USB',
    code: `// Requires Leonardo, Micro, or Due board
#include "Keyboard.h"

void setup() {
  Keyboard.begin();
  delay(1000);

  // Example for Windows/Linux: Ctrl+Alt+Delete
  // Keyboard.press(KEY_LEFT_CTRL);
  // Keyboard.press(KEY_LEFT_ALT);
  // Keyboard.press(KEY_DELETE);
  // delay(100);
  // Keyboard.releaseAll();

  // Example for macOS: Command+Shift+Q
  Keyboard.press(KEY_LEFT_GUI); // Command key
  Keyboard.press(KEY_LEFT_SHIFT);
  Keyboard.press('q');
  delay(100);
  Keyboard.releaseAll();
}

void loop() {}`
  },
  {
    title: 'Keyboard Message',
    description: 'Send a text string when a button is pressed.',
    slug: 'keyboard-message',
    icon: Keyboard,
    category: 'USB',
    code: `// Requires Leonardo, Micro, or Due board
#include "Keyboard.h"

const int buttonPin = 2;

void setup() {
  pinMode(buttonPin, INPUT_PULLUP);
  Keyboard.begin();
}

void loop() {
  if (digitalRead(buttonPin) == LOW) {
    Keyboard.print("Hello from Arduino!");
    delay(500); // Debounce
    while(digitalRead(buttonPin) == LOW); // Wait for release
  }
}`
  },
  {
    title: 'Keyboard Reprogram',
    description: 'Reprogram the keyboard with a switch.',
    slug: 'keyboard-reprogram',
    icon: Keyboard,
    category: 'USB',
    code: `// Requires Leonardo, Micro, or Due board
#include "Keyboard.h"

const int reprogramPin = 2;

void setup() {
  pinMode(reprogramPin, INPUT_PULLUP);
  Keyboard.begin();
}

void loop() {
  if (digitalRead(reprogramPin) == LOW) {
    // Send reprogramming command (specific to your bootloader, e.g., double-tap reset)
    // This is a conceptual example, actual implementation varies.
  }
}`
  },
  {
    title: 'Keyboard Serial',
    description: 'Read serial data and type it out.',
    slug: 'keyboard-serial',
    icon: Keyboard,
    category: 'USB',
    code: `// Requires Leonardo, Micro, or Due board
#include "Keyboard.h"

void setup() {
  Serial.begin(9600);
  Keyboard.begin();
}

void loop() {
  if (Serial.available() > 0) {
    char inChar = Serial.read();
    Keyboard.write(inChar);
  }
}`
  },
  {
    title: 'Arduino Reference',
    description: 'A reference for core programming concepts like variables, data structures, and functions.',
    slug: 'arduino-reference',
    icon: GitBranch,
    category: 'Reference',
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

// Group learning items by category
const learningGroups = LEARNING.reduce((acc, item) => {
  const category = item.category || 'Other';
  if (!acc[category]) {
    acc[category] = [];
  }
  acc[category].push(item);
  return acc;
}, {} as Record<string, LibraryItem[]>);

const categoryIcons: Record<string, LucideIcon> = {
    'Basics': GraduationCap,
    'Digital': Power,
    'Analog': Waves,
    'Communication': Share2,
    'Control Structures': GitBranch,
    'Sensors': Thermometer,
    'Display': Monitor,
    'Strings': Text,
    'USB': Cable,
    'Reference': Book,
};

export const LEARNING_CATEGORIES = Object.keys(learningGroups).map(category => ({
  title: category,
  icon: categoryIcons[category] || Code,
  items: learningGroups[category],
}));
