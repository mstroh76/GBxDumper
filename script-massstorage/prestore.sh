#/bin/bash
sudo rmmod g_mass_storage
sudo mkdosfs /dev/ram0p1
sudo mount /dev/ram0p1 /ramdisk

