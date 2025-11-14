import { CodeDisplay } from "@/components/code-display";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { SENSORS } from "@/lib/data";

export default function SensorsPage() {
  return (
    <div className="container mx-auto px-4 py-8 md:py-12">
      <header className="text-center mb-12">
        <h1 className="text-4xl md:text-5xl font-bold font-headline text-primary">
          Sensor Code Library
        </h1>
        <p className="text-lg md:text-xl text-muted-foreground mt-2 max-w-3xl mx-auto">
          Browse our collection of code snippets for popular Arduino sensors.
        </p>
      </header>

      <div className="space-y-12">
        {SENSORS.map((sensor) => (
          <Card key={sensor.slug} id={sensor.slug} className="shadow-md transition-shadow hover:shadow-lg">
            <CardHeader>
              <div className="flex items-center gap-4">
                <div className="bg-primary/10 p-3 rounded-lg">
                    <sensor.icon className="w-8 h-8 text-primary" />
                </div>
                <div>
                    <CardTitle className="font-headline text-2xl">{sensor.title}</CardTitle>
                    <CardDescription>{sensor.description}</CardDescription>
                </div>
              </div>
            </CardHeader>
            <CardContent>
              <CodeDisplay code={sensor.code} title={sensor.title} />
            </CardContent>
          </Card>
        ))}
      </div>
    </div>
  );
}
