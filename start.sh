#!/bin/bash
# please add ramdisk_size=49152 to kernel parameter (/boot/cmdline.txt)
echo GBx-Dumper Start-Script
echo -n "System: "
uname -a
echo -n "User: "
whoami
echo Prepare ramdisk:
(echo o; echo n; echo p; echo; echo; echo; echo t; echo c; echo w) | sudo fdisk /dev/ram0 > /dev/null

