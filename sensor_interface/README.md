# STM32 Coding Framework
This repo consists of mainly a Makefile which is oriented to be used to compile and upload programs to a STM32 device using a USB to Serial converter with the help of DFU supports on STM32 devices, hence without the need for a STLink debugger.

The compiler arguments are configured towards programming for the STM32F103C8. Why? This chip is commonly available for very low cost on different form of development boards (ex: `bluepill`)(see https://wiki.stm32duino.com/index.php?title=STM32F103_boards), and it is also supported by the Arduino IDE. This project is useful if you do not want to use commercial IDEs (most of which restrict code size) nor the Arduino IDE, and enables compiling and loading custom program onto the microcontroller entirely through the terminal.

## **!!!!**
This branch assumes a STM32F103 board is connected to a NanoPi Duo over UART1(A9,A10) on the STM and S1(RX1,TX1) on the NPi. There are also connections between Reset(RST),BOOT0 on STM to 203,363 on NPi respectively. As a result the make file called a separately compiled program to reset the device or put it into bootloading mode. For pin information on the NanoPi please refer http://wiki.friendlyarm.com/wiki/index.php/NanoPi_Duo. For pin information on the sensor interface board please refer board layout and schematic in the `sensor_interface_board` directory


## Requirements
- Install compiler (https://developer.arm.com/open-source/gnu-toolchain/gnu-rm)
```sh
sudo apt-get install gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi
```
- Install firmware loader (https://sourceforge.net/p/stm32flash/wiki/Home/)
```sh
sudo apt-get install stm32flash
```
- STM32 Standard Peripheral Library (https://www.st.com/en/embedded-software/stsw-stm32054.html)
search for "STSW-STM32054" and download into this folder
- Install serial monitor (https://www.gnu.org/software/screen/manual/screen.html)
```sh
sudo apt-get install screen
```
- ~~A USB to Serial/TTY/UART adapter (https://www.adafruit.com/product/954, there are several other cheaper options as well)~~

## Usage:
##### Typical Usage
- Put all source and header files into `src` and `include` directories respectively. Source files can be `C` or `C++` files
- ~~When using the **Bluepill STM32 board**, connect `A9(TX)` and `A10(RX)` from the board to RX and TX on a **USB to Serial converter** respectively~~
- ~~Connect power(+5/+3.3 and Ground) to the STM board to power it up~~
- Run ```make``` to compile
- ~~Set BOOT0 pinto `1` on the STM board and press the reset button~~
- Run ```make upload``` to upload to STM board over TTY
- When using serial, monitor it using ```screen <device> <baud>```
  ex: ```screen /dev/ttyS1 115200```
- Exit from serial monitor using `Ctrl+a` followed by `k` and `y`

##### Compile only(use all cores)
`make -j`
##### Show details of compiled project (size, ram etc)
`make detail`
##### Cleanup
`make clean`
##### Upload to device
`make upload`
##### Send run command to device (When device is in bootloader mode)
`make run`
##### Use serial monitor
`make serial`

## NOTE
- The default serial port is selected to be `/dev/ttyS1`, you may need to change this in case your **USB to Serial converter** is connected to a different port. For example if its connected to `/dev/ttyUSB0` use as: ```USB_DEVICE=/dev/ttyUSB0 make upload```
- Make sure there exists a common ground between STM board and the USB serial converter
- When defining Interrupt Service Routines (ISR) make sure to enclose the method
	inside ```extern "C" {/*ISR defined here*/}``` when using CPP


## More configurations:
- define project name as ```PROJECT=my_project make```, defaults to `project`
- define build directory as ```BUILD_DIR=bd make```, defaults to ```build```
- define build directory as ```SOURCE_DIR=my_sources make```, defaults to ```src```
- define build directory as ```INCLUDE_DIR=my_includes make```, defaults to ```include```
- define library to use as ```STMLIB=<STM32STD/STM32CUBE> make```, defaults to ```STM32STD```
- switch to using gcc instead of g++ with ```COMPILER=CC make```, defaults to ```CPP```, read this to make a decision to do so http://warp.povusers.org/grrr/cplusplus_vs_c.html


## Code Files
All source files (C,C++) should be placed in `src` folder, or if put in a different folder should be indicated while calling make as shown above. Similarly all user header files should be located in the `include` folder. The compiler expects a `main` function to start with. A demo program is provided which uses the on-board LED, communication over UART(serial) and timing delays.









## ~`/\`~
This work is largely inspired by others:
- https://github.com/artem-smotrakov/stm32f103-template.git
- https://github.com/RuanJG/cotex-m3-project.git

## Changes
- Added `DEBUG='true/false'` as an argument while calling make, to configure debug prints
- interrupt based values seem to be more stable than DMA, 4 channel work without problem
- changes the global buffers to correspond to the timer associated with them, same goes for indexes(only used when not using DMA)
- A buffer pointer is added to the TIM_PWM_CAPTURE_t to point to the global buffer index when using interrupts, if not then set to 0(this is checked to configure DMA/Interrupts in input capture)
- ISR for each timer defined
- process_sensor_data() is modified to take into account indexing using DMA or global indexes
- to unify the axes when using DMA, the x and y are flipped and negated. Make sure sign conventions are proper when using this data
- timer 3 added
- added a rule in Makefile to upload only without resetting (useful when uploading using a laptop and resetting the STM manually)
- actually format and proccess the OOTX data now(though not really used in the solver, but available)
- serial update is performed only when `#` is received, helps sync the processing on the processor end
- fixed binary uart method which missed the first bit
- fixed led init method which called reset and entered an infinite loop of methods
- modularized methods to check and process x/y data for each timer and method to process ootx frame
- struct definitions of compare capture(and x,y values) and ootx data
- fixed timer 1 issue with correct RCC initialization
- ootx processing can be disabled by setting corresponding state
- ootx decode takes one timer to process frame
- tested with timer 1,2,4, all working simultaneously, with individual buffers
- made init_timer_capture() configurable using a struct
- added resolution to the capture (when using too high,>4, might need to switch from uint16 to uint32)
- some methods to display uint,int,binary numbers without stdlib, hence faster
- added a ootx state to modularize the ootx decode process into bits, need to restructure that
- ootx has a stopped state to avoid processing it when not needed
- checked working for timer 2 and 4, on pins PA0 and PB6
- timer 3 can do pwm capture but is not connected to DMA (TIM3_CH2)
- timer 1 is not tested, need to change the timer divider and configure to use different RCC
- lots of debug code, need to clean up


- Two types of delays, one using interrupt based Systick and another uses loops, default delay can be configured using comiler option -DDELAY_TYPE_NOP or -DDELAY_TYPE_SYSTICK
