# Embedded Systems 2018 Project
This project template contains CMake files useful for compiling your code as well
as needed libraries and a modified version of the Pololu Zumo library.

We recommend using CLion for your project. It is free for students and 
integrates well with the provided CMake files.

For documentation regarding CMake, the toolchain, and libraries please consult
the respective standard documentation on these tools.

## Installation
You need to install AVR-GCC [1], AVR Libc [2], and Avrdude [3]. For Windows and
Mac OS we recommend installing the Arduino SDK from [4] which includes everything
you need. For Linux distributions you need to use your package manager.

[1] https://gcc.gnu.org/wiki/avr-gcc
[2] https://www.nongnu.org/avr-libc/
[3] http://savannah.nongnu.org/projects/avrdude
[4] https://www.arduino.cc/en/Main/Software

## Configuration
To configure the project, copy `Config.cmake.txt` to `Config.cmake` and adjust it
accordingly to match your setup. The configuration automatically detects your
operating system.

For the pink programmer set `AVRDUDE_PROGRAMMER` to `usbasp` and for the green
programmer to `avrispv2`. 
Further, specify the serial port of your programmer in `AVRDUDE_PORT`.

## Building & Flashing
Please uncomment either `#define TARGET ZUMO_ROBOT` or `#define TARGET PI_ROBOT` in `hdr/target.h`, then select either `scout.hex` or `collector.hex` from CLion's drop-down menu. Click `Build` (or press `Ctrl+F9`) to generate the respective .hex file. Flash using either `flash_scout.bat` or `flash_collector.bat` (found in the `out` folder).