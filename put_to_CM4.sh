#!/bin/bash

if [ -z $1 ] ; then
	echo ""
	echo "	specify hostname or ip_address"
	echo ""
	exit
fi
hostname=$1

tar czf - \
	build/stella_firmware.bin \
	build/bootloader/bootloader.bin \
	build/partition_table/partition-table.bin \
| ssh stella@$hostname 'mkdir -p /home/stella/stella_firmware/build/bootloader /home/stella/stella_firmware/build/partition_table && tar xzf - -C /home/stella/stella_firmware'
