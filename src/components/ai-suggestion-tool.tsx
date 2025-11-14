"use client";

import { useActionState, useEffect, useRef, useState } from "react";
import { Bot, Loader, Sparkles, Wand2 } from "lucide-react";
import { useFormStatus } from "react-dom";

import { getAiSuggestions } from "@/app/actions";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardFooter, CardHeader, CardTitle } from "@/components/ui/card";
import { Textarea } from "@/components/ui/textarea";
import { Label } from "./ui/label";
import { useToast } from "@/hooks/use-toast";
import { AnimatePresence, motion } from "framer-motion";

const initialState = {
  suggestions: undefined,
  error: null,
};

function SubmitButton() {
  const { pending } = useFormStatus();
  return (
    <Button type="submit" disabled={pending}>
      {pending ? (
        <>
          <Loader className="mr-2 h-4 w-4 animate-spin" />
          Thinking...
        </>
      ) : (
        <>
          <Sparkles className="mr-2 h-4 w-4" />
          Get Suggestions
        </>
      )}
    </Button>
  );
}

export function AiSuggestionTool() {
  const [state, formAction] = useActionState(getAiSuggestions, initialState);
  const [history, setHistory] = useState<string[]>([]);
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
    if (state.suggestions && state.suggestions.length > 0) {
      setHistory(prev => [...prev, ...state.suggestions!]);
      formRef.current?.reset();
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [state.suggestions, state.timestamp]);

  return (
    <Card className="max-w-3xl mx-auto shadow-lg">
      <CardHeader>
        <div className="flex items-center gap-4">
            <div className="p-3 bg-primary/10 rounded-full">
                <Bot className="w-8 h-8 text-primary" />
            </div>
            <div>
                <CardTitle className="font-headline text-2xl">AI Example Finder</CardTitle>
                <CardDescription>Describe your project, and I'll suggest some code examples to get you started.</CardDescription>
            </div>
        </div>
      </CardHeader>
      <form action={formAction} ref={formRef}>
        <CardContent>
          <input type="hidden" name="history" value={JSON.stringify(history)} />
          <div className="grid w-full gap-2">
            <Label htmlFor="description" className="sr-only">Project Description</Label>
            <Textarea
              id="description"
              name="description"
              placeholder="e.g., 'I want to build a weather station that shows temperature on an LCD screen'"
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
      {state.suggestions && state.suggestions.length > 0 && (
        <CardContent>
            <h3 className="font-headline text-lg mb-4 text-foreground">Here are some ideas:</h3>
            <div className="space-y-3">
              <AnimatePresence>
                {state.suggestions.map((suggestion, index) => (
                  <motion.div
                    key={`${state.timestamp}-${index}`}
                    initial={{ opacity: 0, y: 20 }}
                    animate={{ opacity: 1, y: 0 }}
                    transition={{ duration: 0.3, delay: index * 0.1 }}
                    className="flex items-start gap-3 p-3 bg-background rounded-lg border"
                  >
                    <Wand2 className="h-5 w-5 text-accent mt-1 flex-shrink-0" />
                    <p className="text-muted-foreground">{suggestion}</p>
                  </motion.div>
                ))}
              </AnimatePresence>
            </div>
        </CardContent>
      )}
    </Card>
  );
}
