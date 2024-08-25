<h1>Features</h1>

- 4 Drives indication
- 3 LEDs per Drive
- SGPIO support
- 'SES over I2C' support
- Automatic protocol detection
- The ability to select one of the 4 addresses for I2C exchange
- The ability to combine Boards into a stack
- Molex IDE or Floppy power connector option
- Power-On Demo-Test of LEDs
- MCU Temperature Sensor
- MCU Vcc Sensor

The panel is compatible with 2 popular protocols and displays the status of 4 disks, with three indicators for each: activity, error and identification. It can automatically detect the protocol, and if SES over I2C is used, the panel allows you to select one of the 4 addresses with which it will be exchanged by the controller. And, as a bonus, it sends its temperature and power supply voltage to the controller.

In addition, such panels can be stacked: they can be placed several levels above each other and several pieces in a row on each level. In this case, the power supply circuit can be soldered only on one of the boards in the stack. Besides, everything is assembled from very budget components.

See the detailed review of the project on my YT-channel (English subtitles): https://youtu.be/YXgrQts7U3w

[![Click to see](https://img.youtube.com/vi/YXgrQts7U3w/maxresdefault.jpg)](https://www.youtube.com/watch?v=YXgrQts7U3w)

<h1>Details</h1>

You can find the project schematic, PCB drawing and other materials here: https://oshwlab.com/sneer2sneer/sgpio-ses-indicator-board

Device schematic:

![Schematic_SGPIO_SES_Indicator_Board](https://github.com/DmitryMuravyev/SGPIO-SES-Indicator-Board/assets/152902525/3f2b5ec6-a1be-4fb3-8f86-c35fd10bcca7)

If the device operates via 'SES over I2C' protocol, then errors are indicated according to the following table (this is not exactly similar to IBPI, but you can freely make changes to the source code so that the indication becomes what you like):

![Errors and LEDs](https://github.com/DmitryMuravyev/SGPIO-SES-Indicator-Board/assets/152902525/ce31319a-7841-478b-847f-2fd9d8f7cadc)

All the code is written in the STM32CubeMX/IDE using the HAL library.

Since all the pins of the microcontroller are used for data/control functions, reprogramming of the board is possible only with the 'Connect Under Reset' option selected. To do this, you have to use a version of the ST Link programmer for 32-bit MCUs, connected according to the following schematic:

![ST Link](https://github.com/DmitryMuravyev/SGPIO-SES-Indicator-Board/assets/152902525/782c30b3-467f-4863-81f9-d09c63202cc4)

<h1>Links</h1>

Project - https://oshwlab.com/sneer2sneer/sgpio-ses-indicator-board

Additional project files - https://drive.google.com/drive/folders/1VnPzOl7hxJ_D-UxZYw9dO9BYb0g93dh4

STM32F070 Datasheet - https://www.st.com/resource/en/datasheet/stm32f070c6.pdf

SGPIO - https://en.wikipedia.org/wiki/SGPIO

SES - https://en.wikipedia.org/wiki/SES-2_Enclosure_Management

