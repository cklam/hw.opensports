#!/bin/bash
if [ ! -d "./mbed-os" ]; then
ln -s ../BLE_LED/mbed-os .
fi
mbed compile -t GCC_ARM -m NRF51_DK | tee log
grep Image log | awk '{print "cp "$2 " /media/human/JLINK"}' > cphex.sh
chmod +x cphex.sh
./cphex.sh
