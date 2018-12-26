#!/bin/sh
SRC_DIR=`pwd`
GDB=arm-none-eabi-gdb
OPENOCD=openocd
TELNET=telnet
BIN_ELF=`find ${SRC_DIR}/Debug -type f -name "*.elf"`
OPENOCD_CFG=${SRC_DIR}/ShellScripts/openocd_stlinkv2_stm32f4Disco.cfg
#OPENOCD_CFG=board/stm32f4discovery.cfg
FILENAME=$(basename -- "$BIN_ELF")
Extension="${FILENAME##*.}"
Filename="${FILENAME%.*}"
#echo $BIN_ELF
#echo ${SRC_DIR}/Debug/$Filename.bin
arm-none-eabi-objcopy -O binary $BIN_ELF ${SRC_DIR}/Debug/$Filename.bin &
echo "Deleting Memory Bank and Programming Chip..." &
xterm -e "$OPENOCD -f $OPENOCD_CFG -c \"init; reset halt; stm32f4x mass_erase 0; flash write_bank 0 ./Debug/$Filename.bin 0; reset run\" " &
sleep 20
xterm -e "$TELNET localhost 4444" &
echo "Loading Telnet...\n\tLoading GDB...\n\t\tHappy and Lucky Debugging!" &
sleep 1
uxterm -e "$GDB $BIN_ELF -ex \"target remote localhost:3333\""
