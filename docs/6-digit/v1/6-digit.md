# 6 digit (HH:MM:SS) modification

1. [Board](#board)
2. [Schematic](#schematic)
3. [Firmware](#firmware)
4. [Operation flow](#operation-flow)
5. [Time synchronization](#time-synchronization)
5. [Case](#case)

![Front](images/6d-front-2.gif)

## Schematic

The device is based on the version without the music chip - in this version, the MCU has enough free pins to connect additional segments and an NMEA device. 

[![schematic](schematic/6-digit-v1.jpg)](schematic/6-digit-v1.jpg)

Additional segments are connected to **pins 21 (P3.6)** and **22 (P3.7)** of the MCU. 
The S2 button was originally connected to **pin 15 (P.3)**, but I had to connect to the **pin 7 (P1.4)** instead. The reason for this was that I wanted to receive data from an NMEA device on **pin 15 (P3.0)**. This mapping occurs when the constant `MAP_SW2_TO_P1_4` is defined.

> If you are not planning to synchronize time via NTP or GPS, then the S2 button can be connected to pin 15.

I wanted the NMEA device to be on only during the synchronization process. To achieve this, I added a transistor switch (Q7 on the schematic) connected to **pin 6 (P1.3)**. There is also an LED (D1) indicating that the NMEA device is on. 

The rest of the schematic is as original.

## Board

The main board is extended with a piece of a prototyping board on which additional segments, transistors and buttons are mounted.

![Board 1](images/6-digit-board-1.jpg)

<details>
    <summary>More photos here</summary>
    <IMG src="images/6-digit-board-2.jpg" alt="Board 2"/>
    <IMG src="images/6-digit-board-3.jpg" alt="Board 3"/>
    <IMG src="images/6-digit-board-4.jpg" alt="Board 4"/>
    <IMG src="images/6-digit-board-5.jpg" alt="Board 5"/>
    <IMG src="images/6-digit-front.jpg" alt="Front"/>
    <IMG src="images/6-digit-board-demo.jpg" alt="Front"/>
</details>

## Firmware

> **Note:** The firmware is fully backward compatible with the original 4-digit version.

The firmware is updated to display seconds on additional segments. To the enable the 6 digit support it must be built with the `SIX_DIGITS` constant defined. Almost all screens were updated to take advantage of the additional segments.

### New screens
- Main
![Main](images/6d-front.JPG)

- Temperature
![Temperature](images/6d-temperature.JPG)

- Date
![Date](images/6d-date.JPG)

- Alarm
![Alarm](images/6d-alarm.JPG)

- NMEA settings
  - Timezone (1 h)
  ![NMEA timezone is set to -12 hours](images/6d-nmea_tz_1h.JPG)
  - Timezone (-12 h)
  ![NMEA timezone is set to -12 hours](images/6d-nmea_tz_m12h.JPG)
  - DST (daylight saving time) is ON
  ![NMEA DST is on](images/6d-nmea_dst_on.JPG)
  - DST (daylight saving time) is OFF
  ![NMEA DST is off](images/6d-nmea_dst_off.JPG)
  - Automatic time synchronization is set to 3 hours
  ![NMEA autoupdate is set to 3h](images/6d-nmea_upd_3h.JPG)
  - Automatic time synchronization is OFF
  ![NMEA autoupdate is off](images/6d-nmea_upd_off.JPG)

## Operation flow
The operation flow is shown on the following diagram:

[![Operation flow](images/6-digit-clock-operation-flow.png)](images/6-digit-clock-operation-flow.png)

## Time synchronization

Time can be synchronized with an NMEA device - either a GPS receiver or a microcontroller using the NTP protocol. For the second option, you need to use an ESP8266 Wi-Fi module.

I consider the first version of the clock as experimental, so I decided to place an NMEA receiver together with the transistor switch on a separate prototyping board which is attached to the main board. In this way I gave myself the opportunity to experiment, in a stable version it could be done better and more compactly.

### GPS synchronization

More detailed information can be found in the [GPS synchronization](../nmea/NMEA.md#gps-syncronisation) section.

![NEO6MV2 1](images/6-digit-NEO6MV2-1.jpg)
![NEO6MV2 2](images/6-digit-NEO6MV2-2.jpg)

### NTP Syncronisation

More detailed information can be found in the [NTP synchronization](../nmea/NMEA.md#ntp-syncronisation) section.

![WeMos D1 mini 1](images/6-digit-WeMos-D1-mini-1.jpg)
![WeMos D1 mini 2](images/6-digit-WeMos-D1-mini-2.jpg)

## Case

The case is made from 3mm thick clear acrylic using laser cutting. It consist of 7 parts, designed in Inkscape, the design can be found [here](case/6-digit-case.svg). The parts are glued together to form the front and back sides, which are connected with 2mm screws and spacers.

![Front](images/6d-persep.JPG)

![Back](images/6d-back.JPG)
