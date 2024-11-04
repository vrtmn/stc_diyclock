# 6 digit (HH:MM:SS) version

![Board 1](6-digit-board-demo.jpg)

Based on the version without the music chip. 

![Circuit](6-digit-circuit.png)

Additional segments are connected to pins 21 and 22 of the MCU.

## Board

The main board is extended with a piece of a pcb on which additional segments, transistors and buttons are mounted.

![Board 1](6-digit-board-1.jpg)
![Board 2](6-digit-board-2.jpg)
![Board 3](6-digit-board-3.jpg)
![Board 4](6-digit-board-4.jpg)
![Board 5](6-digit-board-5.jpg)

## Firmware

The firmware is updated to display seconds on additional segments. To the enable the 6 digit support is must be built with the SIX_DIGITS flag.

The operational flow and the screens are updated as well.

![Operational flow](6-digit-clock-operational-flow.png)