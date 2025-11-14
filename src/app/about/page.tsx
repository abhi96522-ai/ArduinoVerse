
import { Github, Linkedin, Mail, User } from 'lucide-react';
import { Avatar, AvatarFallback, AvatarImage } from '@/components/ui/avatar';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import Link from 'next/link';

export default function AboutPage() {
  return (
    <div className="container mx-auto px-4 py-8 md:py-16">
      <header className="text-center mb-12">
        <h1 className="text-4xl md:text-5xl font-bold font-headline text-primary">
          About Me
        </h1>
        <p className="text-lg md:text-xl text-muted-foreground mt-2 max-w-3xl mx-auto">
          A little bit about the creator of ArduinoVerse.
        </p>
      </header>

      <div className="max-w-4xl mx-auto">
        <Card className="shadow-lg">
          <CardContent className="p-6 md:p-8">
            <div className="flex flex-col md:flex-row items-center gap-8">
              <Avatar className="w-32 h-32 md:w-48 md:h-48 border-4 border-primary">
                <AvatarImage src="https://firebasestorage.googleapis.com/v0/b/teams-ai-test-project.appspot.com/o/studio%2Fstuidio-user-cc-icon-46543-1.png?alt=media&token=8660e513-3976-4be6-a197-2a4c8a29b019" alt="ABHINAV GURIJALA" data-ai-hint="professional portrait" />
                <AvatarFallback>
                  <User className="w-16 h-16" />
                </AvatarFallback>
              </Avatar>
              <div className="text-center md:text-left">
                <h2 className="text-3xl font-bold font-headline">ABHINAV GURIJALA</h2>
                <p className="text-xl text-muted-foreground mt-1">
                  Arduino Enthusiast & Software Developer
                </p>
                <p className="mt-4 text-foreground/80">
                  I'm passionate about making technology accessible and fun for everyone. I created ArduinoVerse as a personal project to help beginners and experienced makers alike find the code and inspiration they need for their next great creation.
                </p>
                <div className="mt-6 flex justify-center md:justify-start gap-4">
                  <Button asChild variant="outline" size="icon">
                    <Link href="https://github.com/abhi96522-ai" aria-label="GitHub Profile">
                      <Github className="h-5 w-5" />
                    </Link>
                  </Button>
                  <Button asChild variant="outline" size="icon">
                    <Link href="#" aria-label="LinkedIn Profile">
                      <Linkedin className="h-5 w-5" />
                    </Link>
                  </Button>
                  <Button asChild variant="outline" size="icon">
                    <Link href="mailto:your.email@example.com" aria-label="Email Me">
                      <Mail className="h-5 w-5" />
                    </Link>
                  </Button>
                </div>
              </div>
            </div>
          </CardContent>
        </Card>

        <section className="mt-12">
          <h2 className="text-3xl font-bold font-headline text-center mb-8">My Skills</h2>
          <div className="grid md:grid-cols-2 lg:grid-cols-3 gap-6">
            <Card>
              <CardHeader>
                <CardTitle className="text-lg font-headline">Arduino & C++</CardTitle>
              </CardHeader>
              <CardContent>
                <p className="text-muted-foreground">Expert in microcontroller programming and hardware integration.</p>
              </CardContent>
            </Card>
            <Card>
              <CardHeader>
                <CardTitle className="text-lg font-headline">React & Next.js</CardTitle>
              </CardHeader>
              <CardContent>
                <p className="text-muted-foreground">Building modern, responsive web applications like this one.</p>
              </CardContent>
            </Card>
            <Card>
              <CardHeader>
                <CardTitle className="text-lg font-headline">Generative AI</CardTitle>
              </CardHeader>
              <CardContent>
                <p className="text-muted-foreground">Integrating AI models to create smart, helpful features.</p>
              </CardContent>
            </card>
          </div>
        </section>
      </div>
    </div>
  );
}
