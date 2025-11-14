
'use client';

import React from 'react';
import {
  SidebarProvider,
  Sidebar,
  SidebarInset,
  SidebarHeader,
  SidebarContent,
  SidebarMenu,
  SidebarMenuItem,
  SidebarMenuButton,
} from '@/components/ui/sidebar';
import { CodeDisplay } from "@/components/code-display";
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from "@/components/ui/card";
import { LEARNING_CATEGORIES } from "@/lib/data";
import type { LibraryItem } from '@/lib/data';
import { GraduationCap } from 'lucide-react';


export default function LearningPage() {
  const firstTopic = LEARNING_CATEGORIES[0].items[0];
  const [activeTopic, setActiveTopic] = React.useState<LibraryItem>(firstTopic);

  const handleTopicClick = (item: LibraryItem) => {
    setActiveTopic(item);
  };
  
  return (
    <SidebarProvider>
      <div className="flex min-h-[calc(100vh-3.5rem)]">
        <Sidebar collapsible="icon">
          <SidebarHeader>
            <div className="flex items-center gap-2 group-data-[collapsible=icon]:justify-center">
              <GraduationCap className="size-6 text-primary" />
              <h2 className="font-headline text-lg group-data-[collapsible=icon]:hidden">
                Learning Path
              </h2>
            </div>
          </SidebarHeader>
          <SidebarContent>
            <SidebarMenu>
              {LEARNING_CATEGORIES.map((category) => (
                  <React.Fragment key={category.title}>
                    <SidebarMenuItem className="p-2 group-data-[collapsible=icon]:p-0">
                      <div className="flex items-center gap-2 text-sm font-medium text-muted-foreground group-data-[collapsible=icon]:justify-center">
                        <category.icon className="h-5 w-5" />
                        <span className="group-data-[collapsible=icon]:hidden">{category.title}</span>
                      </div>
                    </SidebarMenuItem>
                    {category.items.map((item) => (
                        <SidebarMenuItem key={item.slug}>
                          <SidebarMenuButton
                            onClick={() => handleTopicClick(item)}
                            isActive={activeTopic.slug === item.slug}
                            tooltip={{children: item.title}}
                            className="justify-start"
                          >
                            <span className="group-data-[collapsible=icon]:hidden">{item.title}</span>
                          </SidebarMenuButton>
                        </SidebarMenuItem>
                    ))}
                  </React.Fragment>
                ))}
            </SidebarMenu>
          </SidebarContent>
        </Sidebar>
        <SidebarInset>
          <div className="p-4 md:p-8">
                {activeTopic ? (
                <Card id={activeTopic.slug} className="shadow-md">
                    <CardHeader>
                    <div className="flex items-center gap-4">
                        <div className="bg-primary/10 p-3 rounded-lg">
                            <activeTopic.icon className="w-8 h-8 text-primary" />
                        </div>
                        <div>
                            <CardTitle className="font-headline text-2xl">{activeTopic.title}</CardTitle>
                            <CardDescription>{activeTopic.description}</CardDescription>
                        </div>
                    </div>
                    </CardHeader>
                    <CardContent>
                    <CodeDisplay code={activeTopic.code} title={activeTopic.title} />
                    </CardContent>
                </Card>
                ) : (
                <div className="text-center">
                    <h1 className="font-headline text-3xl">Select a topic</h1>
                    <p className="text-muted-foreground">Choose an item from the sidebar to see the tutorial.</p>
                </div>
                )}
          </div>
        </SidebarInset>
      </div>
    </SidebarProvider>
  );
}
