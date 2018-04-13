# FreeRTOS cell for the jailhouse hypervisor

[FreeRTOS](http://www.freertos.org/) is a real time operation system
for embedded systems. It is widely used on ARM based microprocessor boards.
[Jailhouse](https://github.com/siemens/jailhouse) is a partitioning Hypervisor based on Linux.
For more information on both systems refer to the corresponding web pages.

This project aims at getting both systems run together on a multicore ARM processor system.
It allows to combine the general purpose OS Linux with a hard real time OS. Both systems are
almost isolated from each other by the underlying hypervisor.

# Prerequisites

- At the moment the system runs only on a [Banana Pi board](https://www.google.de/#q=banana+pi)
- Make sure Jailhouse is running correctly on this board. Installation instructions can
  be found [here](https://github.com/siemens/jailhouse#setup-on-banana-pi-arm-board).
- A working cross compiler toolchain for your target platform

# Setup

First of all clone this repository

    FREERTOS_CELL_DIR=$HOME/freertos-cell
    git clone https://github.com/siemens/freertos-cell.git $FREERTOS_CELL_DIR

## Jailhouse configuration

Let's assume your Jailhouse sources reside in the directory

    JAILHOUSE_DIR=$HOME/jailhouse
    
then do the following to build new cell description files for the hypervisor

1. Root cell description
    
          cp $FREERTOS_CELL_DIR/jailhouse-configs/bananapi.c $JAILHOUSE_DIR/configs/arm/

2. RTOS cell description

          cp $FREERTOS_CELL_DIR/jailhouse-configs/bananapi-freertos-demo.c $JAILHOUSE_DIR/configs/arm/

3. Rebuild your jailhouse subsystem. In the directory "$JAILHOUSE_DIR/configs/arm" you will get
   two new files which are used in the next step.

4. Setup your jailhouse instances

This step has to be executed on the ARM target. You have to transfer the cell files to the target machine.

          jailhouse enable $JAILHOUSE_DIR/configs/arm/bananapi.cell
          jailhouse cell create $JAILHOUSE_DIR/configs/arm/bananapi-freertos-demo.cell

## FreeRTOS code generation

### Cross compiling

The build system assumes that your cross compiler prefix is "arm-linux-gnueabihf-".
If this condition is true then you simply do

    cd $FREERTOS_CELL_DIR
    make

If your cross compiler prefix is something different then you can build the whole by invoking

    cd $FREERTOS_CELL_DIR
    make CROSS_COMPILE='xxx-yyy-zzz-'

### Native compile step on the target

It is also possible to build the code directly on the target machine by doing the following

    cd $FREERTOS_CELL_DIR
    make CROSS_COMPILE=''

## Startup of the application

Now you are ready to start your FreeRTOS demo under the hypervisor.
Once again this step has to be executed on the ARM target.

    jailhouse cell load FreeRTOS $FREERTOS_CELL_DIR/freertos-demo.bin
    jailhouse cell start FreeRTOS

After performing these 2 steps you should get some output from the FreeRTOS demo application
on the __second__ serial interface (baudrate: 115200) of the Banana Pi board.

Moreover the _green_ LED on the board should start to blink.

The __first__ serial interface is used by the hypervisor and Linux.

# Porting to other platforms

If you plan to get the whole system running on other hardware bases you have to take
into consideration the following items:

- Modify the serial interface access
- Your hardware needs a GIC (Generic Interrupt Controller) module
- Of course you need a SoC with virtualization support
- It must be possible to switch the CPUs online/offline
