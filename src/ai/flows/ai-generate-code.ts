'use server';

/**
 * @fileOverview This file defines a Genkit flow for generating Arduino code based on a user's description.
 *
 * - generateCode - A function that takes a user's description and returns a complete Arduino sketch.
 * - GenerateCodeInput - The input type for the generateCode function.
 * - GenerateCodeOutput - The output type for the generateCode function.
 */

import {ai} from '@/ai/genkit';
import {z} from 'genkit';

const GenerateCodeInputSchema = z.object({
  description: z.string().describe('A detailed description of the desired Arduino project functionality.'),
});
export type GenerateCodeInput = z.infer<typeof GenerateCodeInputSchema>;

const GenerateCodeOutputSchema = z.object({
  code: z.string().describe('The complete, runnable Arduino source code (.ino) for the project.'),
});
export type GenerateCodeOutput = z.infer<typeof GenerateCodeOutputSchema>;

export async function generateCode(input: GenerateCodeInput): Promise<GenerateCodeOutput> {
  return generateCodeFlow(input);
}

const prompt = ai.definePrompt({
  name: 'generateCodePrompt',
  input: {schema: GenerateCodeInputSchema},
  output: {schema: GenerateCodeOutputSchema},
  prompt: `You are an expert Arduino programmer. A user wants to create a new Arduino project. Based on their description, generate a complete, well-commented, and runnable Arduino sketch (.ino file content).

The code should be easy to understand for a beginner. Include comments to explain the purpose of different parts of the code, such as pin initializations, logic in the loop, and any complex functions.

User's Project Description: {{{description}}}

Generate the full Arduino code.
`,
});

const generateCodeFlow = ai.defineFlow(
  {
    name: 'generateCodeFlow',
    inputSchema: GenerateCodeInputSchema,
    outputSchema: GenerateCodeOutputSchema,
  },
  async input => {
    const {output} = await prompt(input);
    return output!;
  }
);
