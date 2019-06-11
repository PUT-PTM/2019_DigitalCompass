# 2019_DigitalCompass

## Overview
The project is a digital compass equipped with a monochrome display showing the geographical angle and direction.

## Description
The full-working compass consists of STM32, accelerometer + magnetometer, monochrome display and PC application.  
Main modules used:
- STM32F407VGTx,  
- LSM303D 3-axis accelerometer + magnetometer IMU 6DoF I2C/SPI,  
- Nokia 5110 84x48px LCD display.  

The used methodology assumed to get the readings of all 3 axes of the magnetometer, use specific mathematic formulas to convert magnetic field into compass heading, and display all the data on the display screen. The magnetometer is connected via I2C, while the screen is connected via SPI.

## Tools
Main tools used:
- Eclipse IDE for C/C++ Developers, Version 3.6.3, Neon.3 Release,  
- CubeMX, Version 5.2.1  
- STM Studio, Version 3.6.0

## How to run
Assuming that the compiled program is uploaded to the board the only thing to run it is to power the board with 5V, e.g. from the USB port.

## How to compile
To compile the program you have to simply copy the code and run it in the Eclipse IDE, with board connected to the computer.

## Future improvements
The main disadvantage of the project is that the compass is very sensitive to changes in the X and Y axes and therefore it can display false angle and direction. It can be improved by using the data read from the accelerometer to have the Z axe counter the changes in the X and Y plain as the solution to remain stable in any position.

## Attributions
Sources the code fragments were taken from or based on:
- https://forbot.pl/blog/kurs-stm32-10-spi-w-praktyce-wyswietlacz-graficzny-2-id9964,
- https://forbot.pl/blog/kurs-stm32-12-i2c-w-praktyce-akcelerometr-id10644,
- https://github.com/pololu/lsm303-arduino/blob/master/LSM303.cpp.

Setting correct registers and ports was possible thanks to the information given in the LSM303D and STM32F407VG datasheets.

## License
See the license [here](LICENSE)

## Credits
Project made by Michał Najborowski and Wojciech Młyńczak.

The project was conducted during the Microprocessor Lab course held by the Institute of Control and Information Engineering, Poznan University of Technology.  
Supervisor: Marek Kraft/Michał Fularz/Tomasz Mańkowski/Adam Bondyra
