"use client";

import { useActionState, useEffect, useRef } from "react";
import { Bot, Loader, Sparkles } from "lucide-react";
import { useFormStatus } from "react-dom";

import { getAiGeneratedCode } from "@/app/actions";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Textarea } from "@/components/ui/textarea";
import { Label } from "./ui/label";
import { useToast } from "@/hooks/use-toast";
import { AnimatePresence, motion } from "framer-motion";
import { CodeDisplay } from "./code-display";

const initialState = {
  code: undefined,
  description: undefined,
  error: null,
};

function SubmitButton() {
  const { pending } = useFormStatus();
  return (
    <Button type="submit" disabled={pending}>
      {pending ? (
        <>
          <Loader className="mr-2 h-4 w-4 animate-spin" />
          Generating...
        </>
      ) : (
        <>
          <Sparkles className="mr-2 h-4 w-4" />
          Generate Code
        </>
      )}
    </Button>
  );
}

export function AiSuggestionTool() {
  const [state, formAction] = useActionState(getAiGeneratedCode, initialState);
  const { toast } = useToast();
  const formRef = useRef<HTMLFormElement>(null);

  useEffect(() => {
    if (state.error) {
      toast({
        variant: "destructive",
        title: "Oh no! Something went wrong.",
        description: state.error,
      });
    }
  }, [state.error, state.timestamp, toast]);

  useEffect(() => {
    if (state.code) {
      formRef.current?.reset();
    }
  }, [state.code, state.timestamp]);

  return (
    <Card className="max-w-3xl mx-auto shadow-lg">
      <CardHeader>
        <div className="flex items-center gap-4">
            <div className="p-3 bg-primary/10 rounded-full">
                <Bot className="w-8 h-8 text-primary" />
            </div>
            <div>
                <CardTitle className="font-headline text-2xl">AI Code Generator</CardTitle>
                <CardDescription>Describe your project, and I'll write the Arduino sketch for you.</CardDescription>
            </div>
        </div>
      </CardHeader>
      <form action={formAction} ref={formRef}>
        <CardContent>
          <div className="grid w-full gap-2">
            <Label htmlFor="description" className="sr-only">Project Description</Label>
            <Textarea
              id="description"
              name="description"
              placeholder="e.g., 'A program that blinks an LED on pin 13 every half second.'"
              required
              minLength={10}
              className="min-h-[100px] text-base"
            />
             {state.error && <p className="text-sm text-destructive">{state.error}</p>}
          </div>
        </CardContent>
        <CardFooter className="flex justify-end">
          <SubmitButton />
        </CardFooter>
      </form>
      
      <AnimatePresence>
        {state.code && (
            <motion.div
                key={state.timestamp}
                initial={{ opacity: 0, y: 20 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.5 }}
            >
                <CardContent>
                    <h3 className="font-headline text-lg mb-2 text-foreground">Generated Code:</h3>
                    <p className="text-sm text-muted-foreground mb-4">
                        Here is the Arduino sketch I generated for: "{state.description}"
                    </p>
                    <CodeDisplay code={state.code} title="Generated Arduino Sketch" />
                </CardContent>
            </motion.div>
        )}
      </AnimatePresence>
    </Card>
  );
}
