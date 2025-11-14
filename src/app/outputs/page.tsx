import { CodeDisplay } from "@/components/code-display";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { OUTPUTS } from "@/lib/data";

export default function OutputsPage() {
  return (
    <div className="container mx-auto px-4 py-8 md:py-12">
      <header className="text-center mb-12">
        <h1 className="text-4xl md:text-5xl font-bold font-headline text-primary">
          Output Device Library
        </h1>
        <p className="text-lg md:text-xl text-muted-foreground mt-2 max-w-3xl mx-auto">
          Find code examples for common Arduino output devices like LEDs, motors, and displays.
        </p>
      </header>

      <div className="space-y-12">
        {OUTPUTS.map((output) => (
          <Card key={output.slug} id={output.slug} className="shadow-md transition-shadow hover:shadow-lg">
            <CardHeader>
              <div className="flex items-center gap-4">
                <div className="bg-primary/10 p-3 rounded-lg">
                    <output.icon className="w-8 h-8 text-primary" />
                </div>
                <div>
                    <CardTitle className="font-headline text-2xl">{output.title}</CardTitle>
                    <CardDescription>{output.description}</CardDescription>
                </div>
              </div>
            </CardHeader>
            <CardContent>
              <CodeDisplay code={output.code} title={output.title} />
            </CardContent>
          </Card>
        ))}
      </div>
    </div>
  );
}
