import { CircuitBoard, Cpu, GraduationCap, Lightbulb, Projector, Bot, Thermometer, Gauge, Waves, Move3d, HardDrive, Book, GitBranch, Share2, Binary, Power, Display, SlidersHorizontal, Sun, Sound, RadioReceiver, Magnet, Fingerprint, Fuel, Cable } from 'lucide-react';
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
    title: 'Basic Programming',
    description: 'The fundamentals of Arduino programming, including setup() and loop().',
    slug: 'basic-programming',
    icon: HardDrive,
    code: `// setup() runs once when you press reset or power the board.
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

// loop() runs over and over again forever.
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off
  delay(1000);                       // wait for a second
}`
  },
  {
    title: 'Advanced Programming',
    description: 'Explore functions, arrays, and other advanced concepts.',
    slug: 'advanced-programming',
    icon: Book,
    code: `int numbers[] = {1, 2, 3, 4, 5};

void setup() {
  Serial.begin(9600);
  int sum = calculateSum(numbers, 5);
  Serial.print("Sum: ");
  Serial.println(sum);
}

void loop() {
  // Nothing here
}

// A function to calculate the sum of an array of integers
int calculateSum(int arr[], int size) {
  int total = 0;
  for (int i = 0; i < size; i++) {
    total += arr[i];
  }
  return total;
}`
  },
  {
    title: 'Serial Port',
    description: 'Communicate between your Arduino and a computer.',
    slug: 'serial-port',
    icon: Share2,
    code: `void setup() {
  Serial.begin(9600);
  Serial.println("Send me a character!");
}

void loop() {
  if (Serial.available() > 0) {
    char received = Serial.read();
    Serial.print("You sent: ");
    Serial.println(received);
  }
}`
  },
  {
    title: 'LEDs',
    description: 'Learn to control one or more Light Emitting Diodes.',
    slug: 'leds',
    icon: Lightbulb,
    code: `int ledPin = 9;

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // Fade in
  for (int fadeValue = 0 ; fadeValue <= 255; fadeValue += 5) {
    analogWrite(ledPin, fadeValue);
    delay(30);
  }
  // Fade out
  for (int fadeValue = 255 ; fadeValue >= 0; fadeValue -= 5) {
    analogWrite(ledPin, fadeValue);
    delay(30);
  }
}`
  },
  {
    title: 'Buttons, Digital Inputs',
    description: 'Read the state of buttons and other digital inputs.',
    slug: 'buttons-digital-inputs',
    icon: Binary,
    code: `const int buttonPin = 2;
const int ledPin = 13;
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
    description: 'Read analog sensors and control outputs with PWM.',
    slug: 'analog-io',
    icon: SlidersHorizontal,
    code: `int potPin = A0;
int ledPin = 9;
int potValue = 0;
int brightness = 0;

void setup() {
  pinMode(ledPin, OUTPUT);
}

void loop() {
  potValue = analogRead(potPin);
  brightness = map(potValue, 0, 1023, 0, 255);
  analogWrite(ledPin, brightness);
}`
  },
  {
    title: 'DC Motors and Relays',
    description: 'Control DC motors and high-power devices with relays.',
    slug: 'dc-motors-relays',
    icon: Power,
    code: `// This example uses an L293D motor driver
const int motorPin1 = 9;
const int motorPin2 = 10;
const int enablePin = 11;

void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, HIGH);
}

void loop() {
  // Spin forward
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
  delay(2000);
  // Spin backward
  digitalWrite(motorPin1, LOW);
  digitalWrite(motorPin2, HIGH);
  delay(2000);
}`
  },
  {
    title: 'LED Displays',
    description: 'Use 7-segment and other LED displays.',
    slug: 'led-displays',
    icon: Display,
    code: `// Example for a 7-segment display (common anode)
// A=2, B=3, C=4, D=5, E=6, F=7, G=8
// This code is simplified and doesn't use a library.

void setup() {
  for (int i = 2; i <= 8; i++) {
    pinMode(i, OUTPUT);
  }
}

void loop() {
  // Display '1'
  digitalWrite(2, HIGH); // A
  digitalWrite(3, LOW);  // B
  digitalWrite(4, LOW);  // C
  digitalWrite(5, HIGH); // D
  digitalWrite(6, HIGH); // E
  digitalWrite(7, HIGH); // F
  digitalWrite(8, HIGH); // G
}`
  },
  {
    title: 'LCD/OLED Displays',
    description: 'Show text and graphics on LCD or OLED screens.',
    slug: 'lcd-oled-displays',
    icon: Projector,
    code: `// Requires LiquidCrystal library for LCD
#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup() {
  lcd.begin(16, 2);
  lcd.print("Hello, LCD!");
}

void loop() {
  lcd.setCursor(0, 1);
  lcd.print(millis() / 1000);
}`
  },
  {
    title: 'Movement, Distance & Tilt',
    description: 'Use accelerometers, gyros, and ultrasonic sensors.',
    slug: 'movement-distance-tilt',
    icon: Move3d,
    code: `// Example for HC-SR04 Ultrasonic Sensor
#define echoPin 2
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
  Serial.println(distance);
  delay(100);
}`
  },
  {
    title: 'Light Sensors',
    description: 'Read data from photoresistors and other light sensors.',
    slug: 'light-sensors',
    icon: Sun,
    code: `int ldrPin = A0;
int ldrValue = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  ldrValue = analogRead(ldrPin);
  Serial.print("Light level: ");
  Serial.println(ldrValue);
  delay(500);
}`
  },
  {
    title: 'Sound Modules',
    description: 'Detect sound with microphone modules.',
    slug: 'sound-modules',
    icon: Sound,
    code: `const int soundSensorPin = A0;
const int ledPin = 13;
const int threshold = 500;

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  int sensorValue = analogRead(soundSensorPin);
  Serial.println(sensorValue);
  if (sensorValue > threshold) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
}`
  },
  {
    title: 'IR Transmitter/Receivers',
    description: 'Send and receive infrared signals, like a TV remote.',
    slug: 'ir-transmitter-receivers',
    icon: RadioReceiver,
    code: `// Requires the IRremote library
#include <IRremote.h>
int RECV_PIN = 11;
IRrecv irrecv(RECV_PIN);
decode_results results;

void setup() {
  Serial.begin(9600);
  irrecv.enableIRIn();
}

void loop() {
  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);
    irrecv.resume();
  }
}`
  },
  {
    title: 'Temperature & Humidity',
    description: 'Use sensors like DHT11/DHT22 to measure environmental data.',
    slug: 'temperature-humidity',
    icon: Thermometer,
    code: `// Requires the DHT library by Adafruit
#include <DHT.h>
#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  dht.begin();
}

void loop() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(t) && !isnan(h)) {
    Serial.print("Temp: ");
    Serial.print(t);
    Serial.print(" C, Humidity: ");
    Serial.println(h);
  }
  delay(2000);
}`
  },
  {
    title: 'Magnetic Field',
    description: 'Detect magnetic fields with Hall effect sensors.',
    slug: 'magnetic-field',
    icon: Magnet,
    code: `const int hallPin = 2;
int hallState = 0;

void setup() {
  pinMode(hallPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  hallState = digitalRead(hallPin);
  if (hallState == LOW) {
    Serial.println("Magnetic field detected");
  }
  delay(200);
}`
  },
  {
    title: 'Touch Sensors',
    description: 'Create touch-sensitive controls with capacitive sensors.',
    slug: 'touch-sensors',
    icon: Fingerprint,
    code: `// Requires the CapacitiveSensor library
#include <CapacitiveSensor.h>
CapacitiveSensor cs_4_2 = CapacitiveSensor(4,2);

void setup() {
  Serial.begin(9600);
}

void loop() {
  long total1 =  cs_4_2.capacitiveSensor(30);
  Serial.println(total1);
  if (total1 > 1000) {
    Serial.println("Touched!");
  }
  delay(10);
}`
  },
  {
    title: 'Gas Sensors',
    description: 'Detect various gases with MQ series sensors.',
    slug: 'gas-sensors',
    icon: Fuel,
    code: `const int gasSensorPin = A0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  int gasValue = analogRead(gasSensorPin);
  Serial.print("Gas Sensor Value: ");
  Serial.println(gasValue);
  delay(500);
}`
  },
  {
    title: 'I2C Communication',
    description: 'Communicate with multiple devices on a two-wire interface.',
    slug: 'i2c',
    icon: Cable,
    code: `// Requires the Wire library
#include <Wire.h>

void setup() {
  Wire.begin(); // join i2c bus (address optional for master)
  Serial.begin(9600);
}

void loop() {
  Wire.requestFrom(8, 6); // request 6 bytes from slave device #8
  while (Wire.available()) {
    char c = Wire.read();
    Serial.print(c);
  }
  delay(500);
}`
  }
];
