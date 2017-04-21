#!/bin/bash
mv -v "$1" /ramdisk
umount /ramdisk
modprobe g_mass_storage file=/dev/ram0 stall=0 ro=1

