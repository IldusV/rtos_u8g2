# rtos_u8g2

#### **Overview**

Integration of u8g2 monochrome display library into the FreeRTOS project.

#### **Prerequisites**
Development of this project is done under Ubuntu 18.04.4 LT OS.
  * ##### _Compiler setup:_
  * ##### _OpenOCD setup:_

#### Board Setup
![](IMG_20200507_165628.jpg)

#### Compile and build
  1. To build the project from the ```Projects``` directory execute: ```$ make``` ;
  2. To remove output files from ```/build``` directory execute: ```$ make clean```;



#### Flashing and on target debugging
  1. Start OpenOCD server by: ```$ openocd -f "board/stm32f0discovery.cfg"```;
  2. Execute sripts: ```$ ./flash``` for flashing or: ```$ ./dbg``` to debug the project;
  3. Serial interface (UART) debugging: ```cu -l /dev/ttyUSB0 -s 9600```
