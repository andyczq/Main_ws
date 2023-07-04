#!/bin/bash
# echo 'KERNEL=="video*", ATTRS{idVendor}=="32e4", ATTRS{idProduct}=="9230", MODE:="0666", GROUP:="dialout",  SYMLINK+="camera_200w"' > /etc/udev/rules.d/camera_200w.rules
# UVC cameras
echo 'SUBSYSTEMS=="usb", ENV{DEVTYPE}=="usb_device", ATTRS{idVendor}=="32e4", ATTRS{idProduct}=="9230", MODE="0666"' > /etc/udev/rules.d/shl_camera_uvc.rules
# ^ Change the vendor and product IDs to match your camera.

systemctl daemon-reload
service udev reload
sleep 1
service udev restart