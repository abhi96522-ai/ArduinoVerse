import { CircuitBoard, Cpu, GraduationCap, Lightbulb, Projector, Bot, Thermometer, Gauge, Waves, Move3d } from 'lucide-react';
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
  Serial.print("%\t");
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
    code: `#define echoPin 2
#define trigPin 3

long duration;
int distance;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  delay(100);
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
    title: 'PIR Motion Sensor',
    description: 'Detect movement with a Passive Infrared (PIR) sensor.',
    slug: 'pir-sensor',
    icon: Move3d,
    code: `int pirPin = 2;

void setup() {
  pinMode(pirPin, INPUT);
  Serial.begin(9600);
  Serial.println("PIR Motion Sensor Test");
}

void loop() {
  int pirState = digitalRead(pirPin);
  if (pirState == HIGH) {
    Serial.println("Motion detected!");
  }
  delay(1000);
}`
  },
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
]

export const LEARNING: LibraryItem[] = [
  {
    title: 'Getting Started',
    description: 'Your first Arduino sketch and basic concepts.',
    slug: 'getting-started',
    icon: GraduationCap,
    code: `// This is a comment. It's ignored by the Arduino.

/*
 * This is a multi-line comment.
 * setup() runs once when you press reset or power the board.
 * loop() runs over and over again forever.
*/

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}`
  },
  {
    title: 'Digital I/O',
    description: 'Learn about digital input and output signals.',
    slug: 'digital-io',
    icon: CircuitBoard,
    code: `int ledPin = 13;
int buttonPin = 7;
int buttonState = 0;

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);
}

void loop() {
  buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
}`
  },
  {
    title: 'Analog I/O',
    description: 'Understand analogRead and analogWrite (PWM).',
    slug: 'analog-io',
    icon: Waves,
    code: `int potPin = A0;
int ledPin = 9; // Must be a PWM pin
int potValue = 0;
int brightness = 0;

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  potValue = analogRead(potPin);
  // map the 10-bit analog value (0-1023) to an 8-bit PWM value (0-255)
  brightness = map(potValue, 0, 1023, 0, 255);
  analogWrite(ledPin, brightness);
  Serial.print("Potentiometer = ");
  Serial.print(potValue);
  Serial.print("\t Brightness = ");
  Serial.println(brightness);
  delay(10);
}`
  },
  {
    title: 'Serial Communication',
    description: 'Send and receive data between your Arduino and computer.',
    slug: 'serial-comm',
    icon: Cpu,
    code: `void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Hello, Computer!");
}

void loop() {
  // check for incoming data
  if (Serial.available() > 0) {
    // read the incoming byte:
    char incomingByte = Serial.read();

    // say what you got:
    Serial.print("I received: ");
    Serial.println(incomingByte, DEC);
  }
}`
  },
];
