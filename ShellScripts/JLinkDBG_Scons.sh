#!/bin/sh
SRC_DIR=`pwd`
GDB=arm-none-eabi-gdb
BIN_ELF=`find ${SRC_DIR}/target -type f -name "*.elf"`
xterm -e "/usr/local/bin/openocd -f openocd_jlink.cfg " &
sleep 1
xterm -e "/usr/bin/telnet localhost 4444" &
sleep 1
uxterm -e "$GDB $BIN_ELF -ex \"target remote localhost:3333\""
