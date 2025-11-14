
'use client';

import React from 'react';
import {
  SidebarProvider,
  Sidebar,
  SidebarInset,
  SidebarHeader,
  SidebarTrigger,
  SidebarContent,
  SidebarMenu,
  SidebarMenuItem,
  SidebarMenuButton,
} from '@/components/ui/sidebar';
import { Code, FunctionSquare, Pilcrow, Type } from 'lucide-react';
import { CodeDisplay } from '@/components/code-display';

const REFERENCE_DATA = {
  structure: {
    title: 'Structure',
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
  variables: {
    title: 'Variables',
    icon: Type,
    items: {
      'datatypes': {
        title: 'Data Types',
        description: 'Variables are used to store data. Arduino supports various data types.',
        code: `// --- Common Data Types ---
int integerVar = 123;          // Stores whole numbers (-32,768 to 32,767)
float floatVar = 3.14;         // Stores numbers with decimal points
char charVar = 'A';            // Stores a single character
String stringVar = "Hello!";   // Stores a sequence of characters (text)
bool boolVar = true;           // Stores true or false
long longVar = 123456789L;     // For larger integer values`
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
pinMode(13, OUTPUT);

// digitalWrite(pin, value) - Write a HIGH or a LOW value to a digital pin.
digitalWrite(13, HIGH);

// digitalRead(pin) - Reads the value from a specified digital pin, either HIGH or LOW.
int buttonState = digitalRead(2);`
      },
      'analog-io': {
        title: 'Analog I/O',
        description: 'Functions for reading analog inputs and using PWM for analog-like output.',
        code: `// analogRead(pin) - Reads the value from the specified analog pin.
int sensorValue = analogRead(A0);

// analogWrite(pin, value) - Writes an analog value (PWM wave) to a pin.
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
            <div className="mb-8">
              <h1 className="font-headline text-4xl font-bold text-primary">{activeContent.title}</h1>
              <p className="mt-2 text-lg text-muted-foreground">{activeContent.description}</p>
            </div>
            <CodeDisplay code={activeContent.code} title={activeContent.title} />
          </div>
        </SidebarInset>
      </div>
    </SidebarProvider>
  );
}
