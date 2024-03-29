# Ultrasonic cat repeller 🐈
This project uses ATtiny85 to generate sound waves in a buzzer in ultrasonic range (20 to 30 kHz). This frequency may repel cats [(see literature)](https://doi.org/10.1016/j.gecco.2018.e00444). This project can be used as a low-power low-budget attempt to repel cats from a garden.

## Power consumption
- Project was optimized to be run with cell batteries. Connect (1 or 2) 3V cell battery as power supply.
- The microcontroller to a "beep cycle" for about 1 min. Then it enters in sleep mode for 2 minutes and start the beep cycle again.
- Average power consumption:
  - 4.2 mA when active and beeping
  - 3.5 mA when active but not beeping
  - 0.008 mA in sleep mode

# References
## Reused code
This project uses code from following authors/ pages:
- [Technoblogy - Simple Tones for ATtiny (2014-04-04)](http://www.technoblogy.com/show?KVO)
- [Renewable Energy Innovation - Sleep Modes on ATTiny85](https://www.re-innovation.co.uk/docs/sleep-modes-on-attiny85/)

## How to's
- [Arjun Ganesan - Programming ATtiny85 with Arduino Uno](https://create.arduino.cc/projecthub/arjun/programming-attiny85-with-arduino-uno-afb829)

## Tech specs
Use this documention to understand better the registers used in the code
- [Microchip Technology - ATtiny85 documentation](https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-2586-AVR-8-bit-Microcontroller-ATtiny25-ATtiny45-ATtiny85_Datasheet.pdf)