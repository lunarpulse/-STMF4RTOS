#!/bin/sh
SRC_DIR=`pwd`
BIN=${SRC_DIR}/target/STM32M4RTOS.elf
GDB=arm-none-eabi-gdb
GDBTP="target remote localhost:3333"
xterm -e "/usr/local/bin/openocd -f openocd_jlink.cfg " &
sleep 1
xterm -e "/usr/bin/telnet localhost 4444" &
sleep 1
$GDB $BIN -ex "target remote localhost:3333"
