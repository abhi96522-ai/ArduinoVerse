import { ArrowRight } from "lucide-react";
import Link from "next/link";
import { AiSuggestionTool } from "@/components/ai-suggestion-tool";
import { Button } from "@/components/ui/button";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { CATEGORIES } from "@/lib/data";

export default function Home() {
  return (
    <div className="container mx-auto px-4 py-8 md:py-16">
      <section className="text-center mb-16 md:mb-24">
        <h1 className="text-4xl md:text-6xl font-bold font-headline tracking-tight text-primary mb-4">
          Welcome to ArduinoVerse
        </h1>
        <p className="text-lg md:text-xl text-muted-foreground max-w-3xl mx-auto">
          Your universe for Arduino code examples, tutorials, and inspiration. Get started by exploring our libraries or ask our AI for help.
        </p>
      </section>

      <section className="mb-16 md:mb-24">
        <AiSuggestionTool />
      </section>

      <section>
        <h2 className="text-3xl font-bold font-headline text-center mb-8">Explore Categories</h2>
        <div className="grid md:grid-cols-2 lg:grid-cols-4 gap-8">
          {CATEGORIES.map((category) => (
            <Card key={category.title} className="bg-card hover:border-primary/50 transition-all duration-300 transform hover:-translate-y-1 shadow-md hover:shadow-xl">
              <CardHeader className="flex flex-row items-center gap-4">
                <div className="bg-primary/10 p-3 rounded-full">
                  <category.icon className="w-8 h-8 text-primary" />
                </div>
                <div>
                  <CardTitle className="font-headline">{category.title}</CardTitle>
                  <CardDescription>{category.description}</CardDescription>
                </div>
              </CardHeader>
              <CardContent>
                <Button asChild variant="outline" className="w-full">
                  <Link href={category.href}>
                    Explore {category.title.split(' ')[0]} <ArrowRight className="ml-2 h-4 w-4" />
                  </Link>
                </Button>
              </CardContent>
            </Card>
          ))}
        </div>
      </section>
    </div>
  );
}
