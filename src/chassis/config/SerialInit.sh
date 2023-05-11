#!/bin/bash
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", GROUP:="dialout",  SYMLINK+="chassis_serial"' > /etc/udev/rules.d/chassis_serial.rules

systemctl daemon-reload
service udev reload
sleep 1
service udev restart