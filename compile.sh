#!/bin/bash
if [ ! -d "./mbed-os" ]; then
ln -s ../BLE_LED/mbed-os .
fi
mbed compile -t GCC_ARM -m LEAPRO2 | tee log
grep Image log | awk '{print "cp "$2 " /media/human/JLINK"}' > cphex.sh
chmod +x cphex.sh
./cphex.sh
