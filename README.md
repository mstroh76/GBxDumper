# GBxDumper  
von **Martin Strohmayer**   
Licence: CC BY-NC 3.0 (https://creativecommons.org/licenses/by-nc/3.0/at/)

Das Programm ermöglicht das Auslesen des ROM- und RAM-Speichers eines Gameboy Advance (GBA) Moduls.
Als Hardware kommt eine Raspberry Pi zum Einsatz, als GPIO-Library wiringPi. Über den I2C-Bus werden zwei MCP23017 I2C-Port-Expander IC angesprochen.  
Der erste IC liest und schreibt die ersten 16 Adress-/Datenleitungen, also AD00-AD15. Der Port A des zweiten ICs dient zum Ansprechen der 
letzten 8 Adress-/Datenleitungen, also AD16-AD23. Die ersten 4 Ausgänge werden für die Steuerleitungen *RD, *WR, *CS und *CS2 verwendet.
Der Vorteil gegenüber Lösungen mit Arduino Uno/Mega sind die geringen Kosten der Raspberry Pi (Zero) und das keine zusätzlich PC-Software benötigt wird. Nachteil könnte die langsam Lesegeschwindigkeit von ca. 1 MiB/min (900 kHz I2C-Bus) sein.
Über die Shell-Scripte start.sh, prestore.sh und poststore.sh kann die ausgelesene Dateien kopiert oder verschoben werden.
Die Beispieldateien sind für die Funktion "USB Gadget - Mass Storage" der Raspberry Pi Zero vorkonfiguriert. 
Dadurch kann nach dem Auslesen des Moduls komfortabel über den OTG-USB-Anschluss auf die Datei von einem Host zugegriffen werden (ohne Programm oder Treiber). 
Siehe https://www.youtube.com/watch?v=v8-PXBbhPfY . 
(Die Größe der Ramdisk muss in der Datei /boot/cmdline.txt auf über 33 MiB gesetzt werden, z. B. ramdisk_size=49152).
   

GBA Modul Pinbelegung:
----------------------
- VCC (3.3V o. 5V) 
- PHI (unused)
- *WR
- *RD
- *CS
- AD00  
...  
- AD23
- *CS2
- IRQ (unused)
- GND
----------------------  

**Schaltplan:**  
![Schaltplan](GBxDumper-Schematics.png)

**Ablauf:**  
Der ROM-Speicher wird mit AD0-AD23 adressiert und mit AD0-AD15 werden die Daten gelesen (2 Byte). Der Speicher kann direkt adressiert oder auch sequenziell ausgelesen werden. Beim sequenziell Lesen muss nicht expliziert jede Adresse gesetzt werden, sondern die Adresse des ROM-Speichers wird automatisch inkrementiert.

**Lesevorgang mit Adresse:**
1. *CS, *RD auf High  
2. AD00-AD23 Output  
3. Adresse AD00-AD23 schreiben  
4. *CS auf Low  
5. AD00-AD15 Input  
6. *RD auf Low  
7. AD00-AD15 lesen (2 Byte)  
8. Sprung zu 1.  

**Lesevorgang automatisch inkrementiert:**  
1. *CS, *RD auf High  
2. AD00-AD23 Output  
3. Adresse AD00-AD23 auf Low schreiben  
4. *CS auf Low  
5. AD00-AD15 Input  
6. *RD auf Low  
7. AD00-AD15 lesen (2 Byte)  
8. *RD auf High  
9. Sprung zu 6.  

Hauptentscheidend für für die Lesegeschwindigkeit ist die Frequenz des I2C-Busses. Diese kann über die Datei  /boot/config.txt gesetzt werden. Dazu muss der Eintrag „dtparam=i2c1_baudrate=900000“ hinzugefügt werden. Üblich ist eine Frequenz von 100 und 400 kHz. Sie kann aber auch, je nach verwendeter I2C-Hardware (Pegelwandler), auf einen wesentlich höheren Wert gesetzt werden. Bei verschiedenen Tests kam es bei einer Frequenz von 1000 kHz zu Problemen, darum wird eine maximale Frequenz von 900 kHz empfohlen.  
Mit einer Raspberry Pi B+ und einer I2C-Frequenz von 900 kHz wird für einen Lesezyklus ca. 113 µs benötigt, dies entspricht ca. **1 MiB/min** Transferrate. Ein 32 MiB GBA-Modul benötigt also ca. 32 Minuten zum Auslesen.

**Programmparameter:**  
  r ... read pin via I2C instead of GPIO (have to match with board jumper, slow)  
  g <Pin> ...  GPIO Pin for read operation  
  e <Pin> ...  GPIO Pin for LED (0=off)  
  s <Pin> ...  GPIO Pin for switch (0=off)  
  n ... Don't use auto adress mode (slow)  
  i ... I2C number  
  l <hexvalue> ... IC1 (AD00-AD15) I2C-Address (22=0x22)  
  h <hexvalue> ... IC2 (AD16-AD23) I2C-Address (21=0x21)  
  f ... verify (need option n)  
  v ... verbose  
  a ... IC Bits AD0-7 swapped  
  b ... IC Bits AD8-15 swapped  
  c ... IC Bits AD16-23 swapped  
  x ... IC Byte AD0-7 and AD8-15 swapped  
  z ... force dump size (>=64 ... KiB, <=32 ... MiB)  
  d <path> ... save dumped file to path (same drive)  
  o <value> ... offset reading (need option z)  


Programmparameter **r** und **g**:  
Eine der wichtigsten Steuerleitungen ist *RD, dieser kann entweder über den I2C-Bus oder über einen GPIO-Kontakt des Raspberry Pi gesetzt werden. 
Die Variante mit GPIO ist schneller als per I2C-Bus. Die angeschlossene Hardware muss dies unterstützen.

Programmparameter **n**:  
Statt den ROM-Speicher sequenziell auszulesen, kann auch jede Adresse manuell gesetzt werden, dies ist allerdings wesentlich langsamer. 

Programmparameter **l** und **h**:  
Die I2C-Slave-Adresse der I2C-Port-Expander ICs kann vorgegeben werden. Sie muss der angeschlossenen Hardware entsprechen.

Programmparameter **e**:  
Das Programm unterstützt eine Status-LED über einen GPIO-Ausgang. 

Programmparameter **s**:  
Der Auslesevorgang kann optional über einen GPIO-Eingang gesteuert werden.
   
Programmparameter **z**:  
Leider ist die Modulgröße nicht im ROM-Speicher hinterlegt. Darum wird vom Programm die Datei "gbalist.csv" verwendet, die Informationen über alle bekannten GBA-Module enthält. Ist diese Datei nicht vorhanden bzw. das Modul unbekannt so kann die Modulgröße vorgegeben werden.  Angaben von 1 bis 32 entspricht der Größe MiB. Angabe von mehr als 64 entspricht der Größe in KiB.

Programmparameter **a**, **b** und **c**:  
Beim Verbinden der Anschlüsse der I2C-Port-Expander ICs kann es optimaler sein, dass die Bits verdreht werden. Darum kann diese Drehung im Programm ausgeglichen werden, sie muss der angeschlossenen Hardware entsprechen.
 
Programmparameter **x**:  
Beim Verbinden der Anschlüsse der I2C-Port-Expander ICs kann es optimaler sein, dass Port-A und Port-B von IC1 verdreht werden. Darum kann diese Drehung im Programm aktiviert werden,
 sie muss der angeschlossenen Hardware entsprechen.

*Beispiele Prototyp 0.9 Hardware (Keine LED, kein Schalter, Bits bei IC1 verdreht):*

**Auslesen (Prototyp 0.9):**  
./gbxdumper -e 0 -g 4 -s 0 -a -b -c

**Auslesen (Prototyp 0.9) der letzten Bytes eines 32 MIB Moduls mit Protokollierung:**  
sudo ./gbxdumper -e 0 -g 4 -s 0 -a -b -c -z 32 -o 16777088 -f -n -v > debug.txt

*Beispiele PCB Hardware von FSA:*

**Programm Start (PCB: Pi GBx-Dumper 1.0 by FSA):**  
./gbxdumper -a 

**Auslesen (PCB: Pi GBx-Dumper 2.0 by FSA):**  
./gbxdumper -b -c -x  

 --------------------------------------------------
**Funktionen:**  
 - Automatische Erkennung von GB(C) und GBA Modulen
 - Auslesen der RAM-Speichers GBA (Speicherstände)
	- EEPROM 4 kibibit und 64 kibibit
	- SRAM/FRAM (256 kibibit)
	- Flash 512 kibibit
 - Auslesen ROM-Speichers GBA (Spiel)
 - Auslesen ROM-Speichers GB(C) mit 32 KiB (Spiel)

 --------------------------------------------------
**Offene Arbeiten:**  
 - Auslesen RAM-Speichers GBA Flash 1024 kibibit (Spielstände) 
 - Auslesen RAM/ROM-Speichers NES Classic GBA 
 - Auslesen ROM-Speichers GB(C) Module mit Bank-Switch  
 - Konfigurationsdatei für Parametrierungen

 --------------------------------------------------
**Hardware:**

PCB Pi GBx-Dumper by FSA: https://forum-raspberrypi.de/forum/thread/33947-game-boy-advance-spiele-auslesen/
 
 
 (c)2017 mstroh 
