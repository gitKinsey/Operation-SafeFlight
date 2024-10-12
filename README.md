 # Operation-SafeFlight 
*This repository contains all the necessary code files for my final thesis at the Kantonschule Sursee.
Under development code you'll find all the code files that lead to the final code, all the inbetween coding that i did. And my previous tries to make the project work.
Under final code for project submission you'll find the two fnal code files, which both are neccessary for my project.*


# Warning:
If you don't no how to fly a drone, please do not start it. If something goes wrong and damage is done you'll have to replace the damaged parts, which in the worse case can mean that the whole drone needs to be replaced.

# Pictures of the Drone
![alt text](<side view.jpg>) 
![alt text](<front view.jpg>)
![alt text](<back view.jpg>)
![alt text](<top view.jpg>)
![alt text](<bottom view.jpg>)




# Credits to:
1. mjs513 (on GitHub): Modified the newPing library to work on teensy 4.x  (I'm mentioning him here anyway even though its fine for him if i don't)
    - https://github.com/mjs513/NewPing_t4/tree/master 
    - (to get it to work i needed to repalce the newPing library content (from the original one) with the content of mjs513's newPing library, was quite easy just under .pio/lipdeps/teensy41/newPing delete all the content   and replace it )

2. My Uncle for aiding me with questions that accured during development
3. My parents for the financial support
4. My Fater for correcting and giving his oppionion on the written part

# Disclaimer
Das entworfene Kollisionspräventionssystem, welches zusätzlich auf die Drohne montiert wurde und auf dem Teensy 4.1 läuft, wurde als eine unterstützende Technologie entwickelt, um den Piloten beim Erlernen des Fliegens zu entlasten und den sicheren Betrieb der Drohne zu vereinfachen. Es dient dazu, allfällige Kollisionen der Drohne zu vermeiden; dazu werden Sen-sordaten in Echtzeit analysiert und potenzielle Gefahren (Objekte, Wände, Boden) erkannt. Es ist trotz des Kollisionspräventionssystems wichtig zu betonen, dass der Pilot zu jeder Zeit die absolute Kontrolle über sein Fluggerät (hier eine Drohne) behalten muss. Das System greift nicht autonom in die Steuerung der Drohne ein, sondern manipuliert lediglich die PPM-Signale, welche für die Kommunikation zwischen Drohne und Pilot genutzt werden, um den Piloten in kritischen Situationen zu unterstützen.

Es wird ausdrücklich darauf hingewiesen, dass das entwickelte Kollisionspräventionssystem keine garantierte Sicherheit bieten kann. Obwohl es darauf ausgelegt ist, bei der Vermeidung potenzieller Kollisionen zu unterstützen, kann keine Garantie gegeben werden, dass alle Hin-dernisse korrekt erkannt und vermieden werden. Das System kann durch verschiedenste Fak-toren wie Wetterbedingungen, technische Störungen, nicht erkannte Objekte oder Signalver-lust beeinträchtigt werden.

Der Pilot trägt weiterhin die alleinige Verantwortung für den sicheren Betrieb der Drohne, ein-schliesslich der notwendigen Reaktionen auf unerwartete Situationen. Die unterstützende Funktion des Systems soll dem Piloten lediglich als Hilfsmittel dienen, ersetzt jedoch nicht die erforderliche Aufmerksamkeit und das aktive Eingreifen des Piloten.
Vom Gebrauch der Drohne wird kräftig abgeraten, da es sich um ein Konzept handelt und die-ses noch kein fertig ausgereiftes Produkt ist. Dazu wären vermehrte Tests und Sicherheits-massnahmen erforderlich, welche nicht in den Kapazitäten des Entwicklers liegen. Bitte be-achten Sie diese Warnung und gehen Sie vernünftig mit dem konzeptuellen Produkt um.

Für allfällige Funktionsuntüchtigkeiten des Systems wird keine Verantwortung übernommen, und es wird stark betont, nicht auf das Kollisionspräventionssystem zu vertrauen, welches aus verschiedensten Gründen nicht funktionieren kann. Wird diese Warnung missachtet, über-nimmt der Hersteller des Systems keine Haftung für allfällige Schäden oder Verletzungen. Mit dem Lesen dieses Disclaimers akzeptieren Sie diese Bedingungen und entbinden den Herstel-ler von allen Konsequenzen.
Für weitere Richtlinien siehe: https://www.bazl.admin.ch/bazl/de/home/drohnen.html



# License
MIT License

Copyright (c) 2024 Cedi

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
