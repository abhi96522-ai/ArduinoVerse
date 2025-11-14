"use server";

import { generateCode } from "@/ai/flows/ai-generate-code";
import { z } from "zod";

const GenerationSchema = z.object({
  description: z.string().min(10, { message: "Please describe your project in at least 10 characters." }),
});

type State = {
  code?: string;
  description?: string;
  error?: string | null;
  timestamp?: number;
};

export async function getAiGeneratedCode(prevState: State, formData: FormData): Promise<State> {
  const validatedFields = GenerationSchema.safeParse({
    description: formData.get("description"),
  });

  if (!validatedFields.success) {
    return {
      error: validatedFields.error.flatten().fieldErrors.description?.[0] || "Invalid input.",
    };
  }

  const { description } = validatedFields.data;

  try {
    const result = await generateCode({ description });
    return { code: result.code, description, timestamp: Date.now() };
  } catch (e) {
    return { error: "Our AI is busy, please try again in a moment.", description };
  }
}
