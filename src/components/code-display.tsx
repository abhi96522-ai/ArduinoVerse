"use client";

import { useState } from "react";
import { Check, Clipboard } from "lucide-react";
import { Button } from "./ui/button";
import { useToast } from "@/hooks/use-toast";

interface CodeDisplayProps {
  code: string;
  title?: string;
}

const arduinoKeywords = [
  '#include', '#define', 'if', 'else', 'for', 'while', 'do', 'switch', 'case', 'break',
  'continue', 'return', 'setup', 'loop', 'pinMode', 'digitalWrite', 'digitalRead',
  'analogWrite', 'analogRead', 'delay', 'millis', 'delayMicroseconds', 'pulseIn',
  'shiftOut', 'tone', 'noTone', 'const', 'volatile', 'static', 'unsigned', 'signed', 'sizeof'
];
const arduinoTypes = [
  'void', 'int', 'long', 'float', 'double', 'char', 'String', 'boolean', 'bool', 'byte',
  'short', 'word', 'unsigned int', 'unsigned long', 'size_t', 'sensors_event_t', 'Servo', 'LiquidCrystal',
  'DHT', 'Adafruit_MPU6050', 'SoftwareSerial', 'LedControl'
];

const highlightSyntax = (code: string) => {
  return code
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;')
    .replace(/'/g, '&#039;')
    .replace(/\b(\d+)\b/g, '<span class="text-amber-300">$1</span>') // Numbers
    .replace(/(\/\/.*$)/gm, '<span class="text-green-400/70">$1</span>') // Single line comments
    .replace(/(\/\*[\s\S]*?\*\/)/gm, '<span class="text-green-400/70">$1</span>') // Multi-line comments
    .replace(/"(.*?)"/g, '<span class="text-amber-300">"$1"</span>') // Strings
    .replace(/'(.*?)'/g, "<span class=\"text-amber-300\">'$1'</span>") // Chars
    .replace(new RegExp(`\\b(${arduinoKeywords.join('|')})\\b`, 'g'), '<span class="text-purple-400">$1</span>') // Keywords
    .replace(new RegExp(`\\b(${arduinoTypes.join('|')})\\b`, 'g'), '<span class="text-sky-400">$1</span>') // Types
    .replace(/([a-zA-Z_]\w*)\s*\(/g, (match, p1) => { // Functions
        if (arduinoKeywords.includes(p1) || arduinoTypes.includes(p1) || p1 === 'setup' || p1 === 'loop') {
            return match;
        }
        return `<span class="text-yellow-300">${p1}</span>(`;
    });
};

export function CodeDisplay({ code, title = "Arduino Sketch" }: CodeDisplayProps) {
  const [hasCopied, setHasCopied] = useState(false);
  const { toast } = useToast();
  const highlightedCode = highlightSyntax(code);
  const lines = highlightedCode.split('\n');


  const handleCopy = () => {
    navigator.clipboard.writeText(code).then(() => {
      setHasCopied(true);
      toast({ title: "Copied to clipboard!" });
      setTimeout(() => setHasCopied(false), 2000);
    });
  };

  return (
    <div className="rounded-lg border bg-gray-900/95 dark:bg-black/70 my-4 shadow-lg overflow-hidden">
      <div className="flex items-center justify-between px-4 py-2 bg-gray-800/20">
        <span className="text-xs font-mono text-gray-400">{title}</span>
        <Button variant="ghost" size="icon" onClick={handleCopy} className="text-gray-400 hover:text-white hover:bg-gray-700/50 h-8 w-8">
          {hasCopied ? <Check className="h-4 w-4 text-green-400" /> : <Clipboard className="h-4 w-4" />}
          <span className="sr-only">Copy code</span>
        </Button>
      </div>
      <div className="p-4 text-sm font-code text-gray-200 relative overflow-x-auto">
        <div className="flex">
          <div className="select-none text-right text-gray-500 pr-4">
            {lines.map((_, index) => (
              <div key={index}>{index + 1}</div>
            ))}
          </div>
          <pre className="flex-1"><code dangerouslySetInnerHTML={{ __html: highlightedCode }} /></pre>
        </div>
      </div>
    </div>
  );
}