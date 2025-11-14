"use client";

import { useState } from "react";
import { Check, Clipboard } from "lucide-react";
import { Button } from "./ui/button";
import { useToast } from "@/hooks/use-toast";

interface CodeDisplayProps {
  code: string;
  title?: string;
}

export function CodeDisplay({ code, title = "Arduino Sketch" }: CodeDisplayProps) {
  const [hasCopied, setHasCopied] = useState(false);
  const { toast } = useToast();

  const handleCopy = () => {
    navigator.clipboard.writeText(code).then(() => {
      setHasCopied(true);
      toast({ title: "Copied to clipboard!" });
      setTimeout(() => setHasCopied(false), 2000);
    });
  };

  const lines = code.split('\n');

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
          <pre className="flex-1"><code>{code}</code></pre>
        </div>
      </div>
    </div>
  );
}
