'use server';

/**
 * @fileOverview This file defines a Genkit flow for suggesting Arduino code examples based on a user's description.
 *
 * - suggestCodeExamples - A function that takes a user's description and returns a list of suggested code examples.
 * - SuggestCodeExamplesInput - The input type for the suggestCodeExamples function.
 * - SuggestCodeExamplesOutput - The output type for the suggestCodeExamples function.
 */

import {ai} from '@/ai/genkit';
import {z} from 'genkit';

const SuggestCodeExamplesInputSchema = z.object({
  description: z.string().describe('A description of the desired Arduino functionality.'),
  exampleDetails: z.string().optional().describe('Details of past examples to make relevant suggestions')
});
export type SuggestCodeExamplesInput = z.infer<typeof SuggestCodeExamplesInputSchema>;

const SuggestCodeExamplesOutputSchema = z.object({
  suggestions: z.array(z.string()).describe('A list of suggested code examples.'),
});
export type SuggestCodeExamplesOutput = z.infer<typeof SuggestCodeExamplesOutputSchema>;

export async function suggestCodeExamples(input: SuggestCodeExamplesInput): Promise<SuggestCodeExamplesOutput> {
  return suggestCodeExamplesFlow(input);
}

const prompt = ai.definePrompt({
  name: 'suggestCodeExamplesPrompt',
  input: {schema: SuggestCodeExamplesInputSchema},
  output: {schema: SuggestCodeExamplesOutputSchema},
  prompt: `You are an expert Arduino programmer. A user is looking for code examples to help them with their project.  Based on their description, suggest 3 relevant code examples.  

Description: {{{description}}}. 

Include details from past example suggestions to make future suggestions more relevant.

Past Example Details: {{{exampleDetails}}}

Return the suggestions as a JSON array of strings. Each string should be a short description of a code example.
`,
});

const suggestCodeExamplesFlow = ai.defineFlow(
  {
    name: 'suggestCodeExamplesFlow',
    inputSchema: SuggestCodeExamplesInputSchema,
    outputSchema: SuggestCodeExamplesOutputSchema,
  },
  async input => {
    const {output} = await prompt(input);
    return output!;
  }
);
