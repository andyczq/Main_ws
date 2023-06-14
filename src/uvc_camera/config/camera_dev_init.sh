#!/bin/bash
echo 'KERNEL=="video*", ATTRS{idVendor}=="32e4", ATTRS{idProduct}=="9230", MODE:="0666", GROUP:="dialout",  SYMLINK+="camera_200w"' > /etc/udev/rules.d/camera_200w.rules

systemctl daemon-reload
service udev reload
sleep 1
service udev restart