\# theFBSiM AT-DSP Firmware Project



\## What this is



Firmware for the AT-DSP (autothrottle display) board — part of theFBSiM

TBM 940 simulator hardware suite. Round display + dual concentric encoder

sits on top of the throttle quadrant, shows AT mode + target IAS + current

IAS + phase + envelope alerts.



\## Hardware



\- MCU: WeAct Black Pill F411CE (STM32F411) with USB-C

\- Display: GC9A01 240x240 round TFT (raw SPI driver, not TFT\_eSPI)

\- Encoder: Elma E37 dual concentric (outer ring + inner knob + pushbutton)

\- Serial: USB CDC (Generic Serial, supersede U(S)ART) @ 250000 baud



\## Current baseline



\- FBSiM\_ATDSP\_v0\_4\_1.ino — full integration firmware

\- Features: raw GC9A01 driver, 7-seg + 5x7 font renderers, hardware

&#x20; quadrature on TIM2/TIM3, debounced button, spec @ATD: protocol both

&#x20; directions, NO DATA watchdog, demo mode fallback



\## Protocol (@ATD: prefix, per SPEC\_2026-04-21\_PCAPP\_v1\_5\_0.md)



Board emits:

\- @ATD:VER:x.y.z on boot

\- @ATD:ACK every 2s (heartbeat)

\- @ATD:INNER:+n / @ATD:INNER:-n (inner knob detent events)

\- @ATD:OUTER:+n / @ATD:OUTER:-n (outer ring detent events)

\- @ATD:BTN:PRESS (single-event, no release)

\- @ATD:ERR:{PARSE,TIMEOUT,OVERRUN}



PC emits compound:

\- @ATD:<state>,<mode>,<target>,<ias>,<phase>,<envelope>\\n

\- state: OFF / ARMED / MAN / FLC / TO / GA

\- mode:  MAN / FLC / -

\- target: 0-500 kt (clamped 75-250 locally during demo)

\- phase: OFF / TOGA / CLIMB / CRUISE / DESCENT / APPROACH / GA

\- envelope: OK / TRQ / ITT / N1 / OVR / UND / BLW



\## Hard rules (DO NOT VIOLATE)



1\. Every flash gets a new version number. No exceptions.

2\. Surgical edits only. No rewrites.

3\. Firmware is ground truth for pin assignments — not netlists, not

&#x20;  datasheets. Check actual pin #defines.

4\. Raw GC9A01 driver is proven foundation — don't touch the init sequence.

5\. SPI frequency stays at 10 MHz (breadboard margin).

6\. DETENTS\_PER\_COUNT=4 (x4 quadrature decoding). E37 spec-confirmed.

7\. Board is DUMB I/O per spec — no local state machine. Encoders emit

&#x20;  events to PC App, PC App owns all logic, PC App sends rendered state

&#x20;  back. Do not add autonomous mode/target mutation in the board.

8\. Arduino IDE auto-prototype bug: any function using user-defined struct

&#x20;  type needs `const struct TypeName\&` in signature, not just

&#x20;  `const TypeName\&`. Applies to DisplayState, Glyph, any future structs.



\## Pin assignments (firmware is ground truth)



\- Display: CS=PA3 DC=PA2 RST=PB0 SCLK=PA5 MOSI=PA7

\- Outer encoder: A=PA0 B=PA1 (TIM2 hardware quadrature)

\- Inner encoder: A=PB4 B=PB5 (TIM3 hardware quadrature)

\- Button: PB3 (INPUT\_PULLUP, active-low)

\- Power: 3V3 to display VCC+BL, GND to display GND and encoder COMN



\## Iteration workflow



1\. Edit .ino file (Claude Code or Arduino IDE)

2\. In Arduino IDE: File → Open → navigate to this .ino → Upload

3\. DFU entry: hold BOOT0, tap NRST, release BOOT0

4\. Verify @ATD:VER:x.y.z in Serial Monitor @ 250000 baud

5\. Test via Serial Monitor commands (line ending = Newline)



\## Arduino IDE board settings



\- Board: Generic STM32F4 → BlackPill F411CE

\- USB support: CDC (generic Serial supersede U(S)ART)

\- Upload method: STM32CubeProgrammer (DFU)

\- Optimize: Smallest (-Os default)



\## Deferred items (not now)



\- ST7789 2.0" rectangular display port (when XIITIA units arrive)

\- DMA-based SPI for faster fill\_rect (responsiveness polish)

\- Color tuning (magenta violet-tilt on current GC9A01 panel)



\## Sibling project docs in theFBSiM Claude project



\- SPEC\_2026-04-21\_PCAPP\_v1\_5\_0.md — PC App side of the @ATD: protocol

\- SITREP\_2026-04-21\_ATDSP.md — team status

\- BUILD\_2026-04-21\_AT\_DSP\_v3\_ELMA\_E37.md — hardware design

\- CHARTER\_COORDINATION\_TEAM.md — cross-team conventions

