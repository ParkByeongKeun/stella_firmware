#!/bin/bash

#python esptool.py -p (PORT) -b 460800 --before default_reset --after hard_reset --chip esp32  write_flash --flash_mode dio --flash_size detect --flash_freq 40m 0x1000 build/bootloader/bootloader.bin 0x8000 build/partition_table/partition-table.bin 0x10000 build/hello_world.bin

if [ -z $1 ] ; then
	echo ""
	echo "	specify fw_image name ( like build/uart_select.bin )"
	echo ""
	exit
fi

filename=$1

echo ""
echo "	UART1 set to GPIO for ESP32 Programming'"
echo "			raspi-gpio set 4,5 ip"
echo ""
raspi-gpio set 4,5 ip
raspi-gpio get 4,5 
esptool.py -p /dev/ttyUSB0 \
            -b 460800 \
            --before default_reset \
            --after hard_reset \
            --chip esp32s3 \
            write_flash \
            --flash_mode dio \
            --flash_size detect \
            --flash_freq 40m \
            0x0000  build/bootloader/bootloader.bin \
            0x8000  build/partition_table/partition-table.bin \
            0x10000 $filename

#          0x10000 build/hello_world.bin
echo ""
echo "	UART1 set to UART again'"
echo "			raspi-gpio set 4,5 a4"
echo ""
raspi-gpio set 4,5 a4
raspi-gpio get 4,5 

