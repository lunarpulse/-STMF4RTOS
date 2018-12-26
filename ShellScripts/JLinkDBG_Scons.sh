#!/bin/sh
SRC_DIR=`pwd`
GDB=arm-none-eabi-gdb
OPENOCD=openocd
TELNET=telnet
BIN_ELF=`find ${SRC_DIR}/target -type f -name "*.elf"`
OPENOCD_CFG=${SRC_DIR}/ShellScripts/openocd_jlink_stm32f4x.cfg
FILENAME=$(basename -- "$BIN_ELF")
Extension="${FILENAME##*.}"
Filename="${FILENAME%.*}"
arm-none-eabi-objcopy -O binary $BIN_ELF ${SRC_DIR}/target/$Filename.bin &
echo "Deleting Memory Bank and Programming Chip..." &
xterm -e "$OPENOCD -f $OPENOCD_CFG -c \"init; reset halt; stm32f4x mass_erase 0; flash write_bank 0 ./target/$Filename.bin 0; reset run\"" &
sleep 20
xterm -e "$TELNET localhost 4444" &
echo "Loading Telnet...\n\tLoading GDB...\n\t\tHappy and Lucky Debugging!" &
sleep 1
uxterm -e "$GDB $BIN_ELF -ex \"target remote localhost:3333\""
