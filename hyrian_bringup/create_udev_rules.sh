#!/bin/bash

echo ""
echo "This script copies Hyrian udev rules to /etc/udev/rules.d/"
echo ""

echo "Motor Driver (USB Serial from RS232) : /dev/ttyTHS1 to /dev/ttyMCU :"
if [ -f "/etc/udev/rules.d/98-hyrian-mcu" ]; then
    echo "98-hyrian-mcu file already exist."
else 
    echo 'KERNEL=="ttyTHS1", MODE:="0666", GROUP:="dialout", SYMLINK+="ttyMCU"' > /etc/udev/rules.d/98-hyrian-mcu.rules   
    echo '98-hyrian-mcu created'
fi

echo ""
echo "YD LiDAR (USB Serial) : /dev/ttyUSBx to /dev/ttyLiDAR :"
if [ -f "/etc/udev/rules.d/97-hyrian-lidar.rules" ]; then
    echo "97-hyrian-lidar.rules file already exist."
else 
    echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout",  SYMLINK+="ttyLiDAR"' >/etc/udev/rules.d/97-hyrian-lidar.rules
    
    echo '97-hyrian-lidar.rules created'
fi

echo ""
echo "Hyrian IMU (USB Serial) : /dev/ttyUSBx to /dev/ttyIMU :"
if [ -f "/etc/udev/rules.d/96-hyrian-imu.rules" ]; then
    echo "hyrian-imu.rules file already exist."
else 
    echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4" ATTRS{idProduct}=="ea60", MODE:="0666", GROUP:="dialout", SYMLINK+="ttyIMU" ' > /etc/udev/rules.d/96-hyrian-imu.rules

    echo '96-hyrian_imu.rules created'
fi


systemctl stop nvgetty
systemctl disable nvgetty

echo ""
echo "Reload rules"
echo ""
sudo udevadm control --reload-rules
sudo udevadm trigger