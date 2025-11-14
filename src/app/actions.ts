"use server";

import { suggestCodeExamples } from "@/ai/flows/ai-suggest-code-examples";
import { z } from "zod";

const SuggestionSchema = z.object({
  description: z.string().min(10, { message: "Please describe your project in at least 10 characters." }),
  history: z.string().optional(),
});

type State = {
  suggestions?: string[];
  error?: string | null;
  timestamp?: number;
};

export async function getAiSuggestions(prevState: State, formData: FormData): Promise<State> {
  const validatedFields = SuggestionSchema.safeParse({
    description: formData.get("description"),
    history: formData.get("history"),
  });

  if (!validatedFields.success) {
    return {
      error: validatedFields.error.flatten().fieldErrors.description?.[0] || "Invalid input.",
    };
  }

  const { description, history } = validatedFields.data;

  try {
    const result = await suggestCodeExamples({
      description,
      exampleDetails: history,
    });
    return { suggestions: result.suggestions, timestamp: Date.now() };
  } catch (e) {
    return { error: "Our AI is busy, please try again in a moment." };
  }
}
