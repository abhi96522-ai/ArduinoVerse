
'use client';

import React from 'react';
import {
  SidebarProvider,
  Sidebar,
  SidebarInset,
  SidebarHeader,
  SidebarContent,
  SidebarMenu,
  SidebarMenuItem,
  SidebarMenuButton,
} from '@/components/ui/sidebar';
import { Code, FunctionSquare, Pilcrow, Type, Hash, PlusSlash, Sigma, Braces, Binary, GanttChartSquare, GitCompareArrows, Equal, Pointer, Repeat } from 'lucide-react';
import { CodeDisplay } from '@/components/code-display';

const REFERENCE_DATA = {
  structure: {
    title: 'Sketch Structure',
    icon: Pilcrow,
    items: {
      'setup': {
        title: 'setup()',
        description: 'The setup() function is called when a sketch starts. Use it to initialize variables, pin modes, start using libraries, etc. The setup function will only run once, after each powerup or reset of the Arduino board.',
        code: `void setup() {
  pinMode(LED_BUILTIN, OUTPUT); // Initialize the digital pin as an output.
  Serial.begin(9600);           // Start serial communication.
}`
      },
      'loop': {
        title: 'loop()',
        description: 'After creating a setup() function, which initializes and sets the initial values, the loop() function does precisely what its name suggests, and loops consecutively, allowing your program to change and respond. Use it to actively control the Arduino board.',
        code: `void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // Turn the LED on
  delay(1000);                       // Wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // Turn the LED off
  delay(1000);                       // Wait for a second
}`
      }
    }
  },
  controlStructures: {
    title: 'Control Structures',
    icon: GanttChartSquare,
    items: {
      'if': {
        title: 'if',
        description: 'The if statement allows for conditional execution of code. If the condition in the parentheses is true, the code inside the curly braces is executed.',
        code: `int value = 10;
if (value > 5) {
  // this code will run
}`
      },
      'else': {
        title: 'if...else',
        description: 'An else statement can follow an if statement to provide code that is executed when the if condition is false.',
        code: `int value = 3;
if (value > 5) {
  // this code will not run
} else {
  // this code will run
}`
      },
      'for': {
        title: 'for',
        description: 'The for loop is used to repeat a block of code a specific number of times.',
        code: `for (int i = 0; i < 5; i++) {
  Serial.println(i); // Prints 0, 1, 2, 3, 4
}`
      },
      'while': {
        title: 'while',
        description: 'A while loop will execute a block of code as long as a specified condition is true.',
        code: `int i = 0;
while (i < 5) {
  Serial.println(i);
  i++;
}`
      },
      'do-while': {
        title: 'do...while',
        description: 'Similar to a while loop, but the condition is checked at the end. This guarantees the loop will execute at least once.',
        code: `int i = 0;
do {
  Serial.println(i);
  i++;
} while (i < 5);`
      },
      'switch-case': {
        title: 'switch...case',
        description: 'A switch statement allows a variable to be tested for equality against a list of values (cases).',
        code: `int sensorValue = analogRead(A0);
switch (sensorValue) {
  case 0:
    Serial.println("Value is 0");
    break;
  case 100:
    Serial.println("Value is 100");
    break;
  default:
    Serial.println("Value is something else");
    break;
}`
      },
      'break': {
        title: 'break',
        description: 'The break keyword is used to exit from a for, while, do...while, or switch statement.',
        code: `for (int i = 0; i < 10; i++) {
  if (i == 5) {
    break; // Exits the loop when i is 5
  }
  Serial.println(i);
}`
      },
      'continue': {
        title: 'continue',
        description: 'The continue keyword skips the rest of the current iteration of a loop and proceeds to the next iteration.',
        code: `for (int i = 0; i < 10; i++) {
  if (i == 5) {
    continue; // Skips printing 5
  }
  Serial.println(i);
}`
      },
      'return': {
        title: 'return',
        description: 'The return keyword terminates a function and can return a value to the calling function.',
        code: `int add(int a, int b) {
  return a + b; // Returns the sum of a and b
}`
      },
       'goto': {
        title: 'goto',
        description: 'Transfers program control to a labeled statement. Its use is generally discouraged in favor of structured control flow like loops and if statements.',
        code: `void loop() {
  int sensorValue = analogRead(A0);
  if (sensorValue > 500) {
    goto error;
  }
  delay(100);
  return;

error:
  Serial.println("Sensor value too high!");
}`
      },
    }
  },
  syntax: {
    title: 'Further Syntax',
    icon: Hash,
    items: {
      'semicolon': {
        title: '; (semicolon)',
        description: 'Used to end a statement in Arduino (C++).',
        code: `int x = 13; // The semicolon ends the line.`
      },
      'curly-braces': {
        title: '{} (curly braces)',
        description: 'Defines a block of code for functions, loops, and conditional statements.',
        code: `void setup() {
  // This is a code block
}`
      },
      'single-comment': {
        title: '// (single line comment)',
        description: 'Anything after the // on the same line is ignored by the compiler.',
        code: `// This is a comment.`
      },
      'block-comment': {
        title: '/* */ (block comment)',
        description: 'Anything between /* and */ is ignored by the compiler, which can span multiple lines.',
        code: `/* This is a
   multi-line
   comment. */`
      },
      '#define': {
        title: '#define',
        description: 'A preprocessor directive used to create a macro or a constant. It replaces all occurrences of the defined name with the value before compilation.',
        code: `#define LED_PIN 13

void setup() {
  pinMode(LED_PIN, OUTPUT);
}`
      },
      '#include': {
        title: '#include',
        description: 'A preprocessor directive used to include external libraries in your sketch.',
        code: `#include <Servo.h> // Includes the Servo library`
      }
    }
  },
  arithmeticOperators: {
    title: 'Arithmetic Operators',
    icon: PlusSlash,
    items: {
      'assignment': {
        title: '= (assignment)',
        description: 'Stores the value to the right of the operator in the variable to the left.',
        code: `int sensorValue = analogRead(A0); // Assigns the reading to sensorValue`
      },
      'addition': {
        title: '+ (addition)',
        description: 'Adds two numbers.',
        code: `int result = 5 + 3; // result is 8`
      },
      'subtraction': {
        title: '- (subtraction)',
        description: 'Subtracts the second number from the first.',
        code: `int result = 5 - 3; // result is 2`
      },
      'multiplication': {
        title: '* (multiplication)',
        description: 'Multiplies two numbers.',
        code: `int result = 5 * 3; // result is 15`
      },
      'division': {
        title: '/ (division)',
        description: 'Divides the first number by the second. For integers, the remainder is discarded.',
        code: `int result = 5 / 3; // result is 1`
      },
      'remainder': {
        title: '% (modulo)',
        description: 'Calculates the remainder of the division of two integers.',
        code: `int result = 5 % 3; // result is 2`
      },
    }
  },
  comparisonOperators: {
    title: 'Comparison Operators',
    icon: Equal,
    items: {
      'equal-to': {
        title: '== (equal to)',
        description: 'Tests if two values are equal.',
        code: `if (x == 5) { /* ... */ }`
      },
      'not-equal-to': {
        title: '!= (not equal to)',
        description: 'Tests if two values are not equal.',
        code: `if (x != 5) { /* ... */ }`
      },
      'less-than': {
        title: '< (less than)',
        description: 'Tests if the left value is less than the right value.',
        code: `if (x < 5) { /* ... */ }`
      },
      'greater-than': {
        title: '> (greater than)',
        description: 'Tests if the left value is greater than the right value.',
        code: `if (x > 5) { /* ... */ }`
      },
      'less-than-equal': {
        title: '<= (less than or equal to)',
        description: 'Tests if the left value is less than or equal to the right value.',
        code: `if (x <= 5) { /* ... */ }`
      },
      'greater-than-equal': {
        title: '>= (greater than or equal to)',
        description: 'Tests if the left value is greater than or equal to the right value.',
        code: `if (x >= 5) { /* ... */ }`
      }
    }
  },
  booleanOperators: {
    title: 'Boolean Operators',
    icon: GitCompareArrows,
    items: {
      'and': {
        title: '&& (logical AND)',
        description: 'Returns true only if both conditions are true.',
        code: `if (x > 0 && y > 0) { /* ... */ }`
      },
      'or': {
        title: '|| (logical OR)',
        description: 'Returns true if at least one of the conditions is true.',
        code: `if (x > 0 || y > 0) { /* ... */ }`
      },
      'not': {
        title: '! (logical NOT)',
        description: 'Inverts a boolean value, turning true to false and false to true.',
        code: `bool isReady = false;
if (!isReady) { // True if isReady is false
  /* ... */
}`
      }
    }
  },
  pointerOperators: {
    title: 'Pointer Operators',
    icon: Pointer,
    items: {
      'reference': {
        title: '& (reference)',
        description: 'Returns the memory address of a variable.',
        code: `int myVar = 10;
int* myPointer = &myVar; // myPointer now holds the address of myVar`
      },
      'dereference': {
        title: '* (dereference)',
        description: 'Accesses the value stored at a memory address held by a pointer.',
        code: `int myVar = 10;
int* myPointer = &myVar;
int value = *myPointer; // value is now 10`
      }
    }
  },
  bitwiseOperators: {
    title: 'Bitwise Operators',
    icon: Binary,
    items: {
      'and': {
        title: '& (bitwise AND)',
        description: 'Performs a bitwise AND operation on each pair of corresponding bits.',
        code: `int a = 5;  // Binary: 0101
int b = 3;  // Binary: 0011
int result = a & b; // result is 1 (Binary: 0001)`
      },
      'or': {
        title: '| (bitwise OR)',
        description: 'Performs a bitwise OR operation on each pair of corresponding bits.',
        code: `int a = 5;  // Binary: 0101
int b = 3;  // Binary: 0011
int result = a | b; // result is 7 (Binary: 0111)`
      },
      'xor': {
        title: '^ (bitwise XOR)',
        description: 'Performs a bitwise exclusive OR (XOR) operation. The result is 1 only if the two bits are different.',
        code: `int a = 5;  // Binary: 0101
int b = 3;  // Binary: 0011
int result = a ^ b; // result is 6 (Binary: 0110)`
      },
      'not': {
        title: '~ (bitwise NOT)',
        description: 'Inverts all the bits of its operand.',
        code: `int a = 5;       // Binary: 00000101
int result = ~a; // result is -6 (Binary: 11111010 in two's complement)`
      },
      'left-shift': {
        title: '<< (bitshift left)',
        description: 'Shifts the bits of the left operand to the left by the number of positions specified by the right operand. Zeros are shifted in from the right.',
        code: `int a = 5; // Binary: 0101
int result = a << 1; // result is 10 (Binary: 1010)`
      },
      'right-shift': {
        title: '>> (bitshift right)',
        description: 'Shifts the bits of the left operand to the right. The nature of the bits shifted in depends on the type of the variable.',
        code: `int a = 5; // Binary: 0101
int result = a >> 1; // result is 2 (Binary: 0010)`
      }
    }
  },
  compoundOperators: {
    title: 'Compound Operators',
    icon: Repeat,
    items: {
      'increment': {
        title: '++ (increment)',
        description: 'Increases the value of an integer variable by one.',
        code: `int x = 2;
x++; // x is now 3`
      },
      'decrement': {
        title: '-- (decrement)',
        description: 'Decreases the value of an integer variable by one.',
        code: `int x = 2;
x--; // x is now 1`
      },
      'compound-addition': {
        title: '+= (compound addition)',
        description: 'Adds the right operand to the left operand and assigns the result to the left operand.',
        code: `int x = 2;
x += 3; // equivalent to x = x + 3; x is now 5`
      },
      'compound-subtraction': {
        title: '-= (compound subtraction)',
        description: 'Subtracts the right operand from the left operand and assigns the result to the left operand.',
        code: `int x = 5;
x -= 3; // equivalent to x = x - 3; x is now 2`
      },
      'compound-multiplication': {
        title: '*= (compound multiplication)',
        description: 'Multiplies the left operand by the right operand and assigns the result to the left operand.',
        code: `int x = 2;
x *= 3; // equivalent to x = x * 3; x is now 6`
      },
      'compound-division': {
        title: '/= (compound division)',
        description: 'Divides the left operand by the right operand and assigns the result to the left operand.',
        code: `int x = 6;
x /= 3; // equivalent to x = x / 3; x is now 2`
      },
      'compound-remainder': {
        title: '%= (compound remainder)',
        description: 'Calculates the modulo of the left operand with the right operand and assigns the result to the left operand.',
        code: `int x = 7;
x %= 3; // equivalent to x = x % 3; x is now 1`
      },
      'compound-bitwise-and': {
        title: '&= (compound bitwise AND)',
        description: 'Performs bitwise AND and assigns.',
        code: `int x = 5; // 0101
x &= 3; // 0011 -> result is 1 (0001)`
      },
      'compound-bitwise-or': {
        title: '|= (compound bitwise OR)',
        description: 'Performs bitwise OR and assigns.',
        code: `int x = 5; // 0101
x |= 3; // 0011 -> result is 7 (0111)`
      },
      'compound-bitwise-xor': {
        title: '^= (compound bitwise XOR)',
        description: 'Performs bitwise XOR and assigns.',
        code: `int x = 5; // 0101
x ^= 3; // 0011 -> result is 6 (0110)`
      }
    }
  },
   variables: {
    title: 'Variables',
    icon: Type,
    items: {
      'datatypes': {
        title: 'Data Types',
        description: 'Variables are used to store data. Arduino supports various data types.',
        code: `// --- Common Data Types ---
int integerVar = 123;          // Stores whole numbers (-32,768 to 32,767 on most Arduinos)
float floatVar = 3.14;         // Stores numbers with decimal points
char charVar = 'A';            // Stores a single character
String stringVar = "Hello!";   // Stores a sequence of characters (text)
bool boolVar = true;           // Stores true or false
long longVar = 123456789L;     // For larger integer values
unsigned int uIntVar = 60000;  // Stores only positive whole numbers (0 to 65,535)`
      },
      'struct': {
        title: 'struct',
        description: 'Structures (structs) allow you to group related variables into a single, convenient package.',
        code: `// Define a structure to hold sensor data
struct SensorReading {
  int id;
  float temperature;
  float humidity;
  bool isValid;
};

// Create a variable of the structure type
SensorReading myReading;

void setup() {
  Serial.begin(9600);
  myReading.id = 1;
  myReading.temperature = 25.5;
  myReading.humidity = 45.8;
  myReading.isValid = true;
  
  Serial.print("Sensor ID: ");
  Serial.println(myReading.id);
}`
      }
    }
  },
  functions: {
    title: 'Functions',
    icon: FunctionSquare,
    items: {
      'custom-functions': {
        title: 'Custom Functions',
        description: 'Functions are reusable blocks of code that perform a specific task. You can define your own functions to make your code more modular and readable.',
        code: `// This function takes two integers as input and returns their sum.
int add(int a, int b) {
  return a + b;
}

// This function does not return a value (void).
void printMessage(String msg) {
  Serial.println(msg);
}

void setup() {
  Serial.begin(9600);
  int result = add(5, 10); // Call the add function
  printMessage("The result is: " + String(result)); // Call the printMessage function
}

void loop() {}`
      },
      'digital-io': {
        title: 'Digital I/O',
        description: 'Functions for controlling digital pins.',
        code: `// pinMode(pin, mode) - Configures the specified pin to behave either as an input or an output.
// mode can be INPUT, OUTPUT, or INPUT_PULLUP
pinMode(13, OUTPUT);

// digitalWrite(pin, value) - Write a HIGH or a LOW value to a digital pin.
digitalWrite(13, HIGH);

// digitalRead(pin) - Reads the value from a specified digital pin, either HIGH or LOW.
int buttonState = digitalRead(2);`
      },
      'analog-io': {
        title: 'Analog I/O',
        description: 'Functions for reading analog inputs and using PWM for analog-like output.',
        code: `// analogRead(pin) - Reads the value from the specified analog pin (0-1023 on most Arduinos).
int sensorValue = analogRead(A0);

// analogWrite(pin, value) - Writes an analog value (PWM wave) to a PWM-capable pin (0-255).
analogWrite(9, 128); // Outputs a PWM wave with a 50% duty cycle`
      }
    }
  }
};

type ReferenceCategory = keyof typeof REFERENCE_DATA;
type ReferenceTopic = keyof (typeof REFERENCE_DATA)[ReferenceCategory]['items'];

export default function ReferencePage() {
  const [activeTopic, setActiveTopic] = React.useState<string>('structure/setup');

  const [category, topic] = activeTopic.split('/') as [ReferenceCategory, ReferenceTopic];
  const activeContent = REFERENCE_DATA[category]?.items[topic];

  const handleTopicClick = (cat: ReferenceCategory, top: ReferenceTopic) => {
    setActiveTopic(`${cat}/${top}`);
  };
  
  return (
    <SidebarProvider>
      <div className="flex min-h-[calc(100vh-3.5rem)]">
        <Sidebar collapsible="icon">
          <SidebarHeader>
            <div className="flex items-center gap-2 group-data-[collapsible=icon]:justify-center">
              <Code className="size-6 text-primary" />
              <h2 className="font-headline text-lg group-data-[collapsible=icon]:hidden">
                Language Reference
              </h2>
            </div>
          </SidebarHeader>
          <SidebarContent>
            <SidebarMenu>
              {Object.keys(REFERENCE_DATA).map((catKey) => {
                const cat = REFERENCE_DATA[catKey as ReferenceCategory];
                return (
                  <React.Fragment key={cat.title}>
                    <SidebarMenuItem className="p-2 group-data-[collapsible=icon]:p-0">
                      <div className="flex items-center gap-2 text-sm font-medium text-muted-foreground group-data-[collapsible=icon]:justify-center">
                        <cat.icon className="h-5 w-5" />
                        <span className="group-data-[collapsible=icon]:hidden">{cat.title}</span>
                      </div>
                    </SidebarMenuItem>
                    {Object.keys(cat.items).map((topicKey) => {
                       const topicItem = cat.items[topicKey as ReferenceTopic];
                       const fullSlug = `${catKey}/${topicKey}`;
                       return (
                        <SidebarMenuItem key={topicItem.title}>
                          <SidebarMenuButton
                            onClick={() => handleTopicClick(catKey as ReferenceCategory, topicKey as ReferenceTopic)}
                            isActive={activeTopic === fullSlug}
                            tooltip={{children: topicItem.title}}
                            className="justify-start"
                          >
                            <span className="group-data-[collapsible=icon]:hidden">{topicItem.title}</span>
                          </SidebarMenuButton>
                        </SidebarMenuItem>
                       );
                    })}
                  </React.Fragment>
                );
              })}
            </SidebarMenu>
          </SidebarContent>
        </Sidebar>
        <SidebarInset>
          <div className="p-4 md:p-8">
            {activeContent ? (
              <>
                <div className="mb-8">
                  <h1 className="font-headline text-4xl font-bold text-primary">{activeContent.title}</h1>
                  <p className="mt-2 text-lg text-muted-foreground">{activeContent.description}</p>
                </div>
                <CodeDisplay code={activeContent.code} title={activeContent.title} />
              </>
            ) : (
              <div className="text-center">
                <h1 className="font-headline text-3xl">Select a topic</h1>
                <p className="text-muted-foreground">Choose an item from the sidebar to see the reference.</p>
              </div>
            )}
          </div>
        </SidebarInset>
      </div>
    </SidebarProvider>
  );
}
