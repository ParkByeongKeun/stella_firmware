#!/bin/bash

if [ -z $1 ] ; then
	echo ""
	echo "	specify hostname or ip_address"
	echo ""
	exit
fi
hostname=$1
scp build/stella_firmware.bin                 stella@$hostname:/home/stella/stella_firmware/build/stella_firmware.bin
scp build/bootloader/bootloader.bin           stella@$hostname:/home/stella/stella_firmware/build/bootloader/bootloader.bin
scp build/partition_table/partition-table.bin stella@$hostname:/home/stella/stella_firmware/build/partition_table/partition-table.bin
