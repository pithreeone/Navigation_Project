#!/bin/bash

# originated from :https://github.com/sunfu-chou/eurobot_ros_ws/blob/master/rename_RPI_USB_ports.sh

# | /dev/USB2 | /dev/USB1 | /dev/USB0 |

# echo  'KERNEL=="ttyUSB*", KERNELS=="1-2:1.0", MODE:="0777", SYMLINK+="USB0"' >> /etc/udev/rules.d/rpi-usb.rules
echo  'KERNEL=="ttyUSB*", KERNELS=="1-2.1:1.0", MODE:="0777", SYMLINK+="USB0-0"' >> /etc/udev/rules.d/rpi-usb.rules
echo  'KERNEL=="ttyUSB*", KERNELS=="1-2.2:1.0", MODE:="0777", SYMLINK+="USB0-1"' >> /etc/udev/rules.d/rpi-usb.rules
echo  'KERNEL=="ttyUSB*", KERNELS=="1-2.3:1.0", MODE:="0777", SYMLINK+="USB0-2"' >> /etc/udev/rules.d/rpi-usb.rules
echo  'KERNEL=="ttyUSB*", KERNELS=="1-2.4:1.0", MODE:="0777", SYMLINK+="USB0-3"' >> /etc/udev/rules.d/rpi-usb.rules

echo  'KERNEL=="ttyACM*", KERNELS=="1-2.1:1.2", MODE:="0777", SYMLINK+="USB0-0"' >> /etc/udev/rules.d/rpi-usb.rules
echo  'KERNEL=="ttyACM*", KERNELS=="1-2.2:1.2", MODE:="0777", SYMLINK+="USB0-1"' >> /etc/udev/rules.d/rpi-usb.rules
echo  'KERNEL=="ttyACM*", KERNELS=="1-2.3:1.2", MODE:="0777", SYMLINK+="USB0-2"' >> /etc/udev/rules.d/rpi-usb.rules
echo  'KERNEL=="ttyACM*", KERNELS=="1-2.4:1.2", MODE:="0777", SYMLINK+="USB0-3"' >> /etc/udev/rules.d/rpi-usb.rules

echo  'KERNEL=="ttyUSB*", KERNELS=="1-3:1.0", MODE:="0777", SYMLINK+="USB1"' >> /etc/udev/rules.d/rpi-usb.rules
echo  'KERNEL=="ttyUSB*", KERNELS=="1-1:1.0", MODE:="0777", SYMLINK+="USB2"' >> /etc/udev/rules.d/rpi-usb.rules
echo  'KERNEL=="ttyS*", MODE:="0777", GROUP:="dialout",  SYMLINK+="SERIAL0"' >> /etc/udev/rules.d/rpi-usb.rules

service udev reload
echo 'wait for fix LATTEPANDA port...'

sudo systemctl stop serial-getty@ttyS0.service
sudo systemctl disable serial-getty@ttyS0.service
sudo systemctl mask serial-getty@ttyS0.service

sleep 2
service udev restart
cat /etc/udev/rules.d/rpi-usb.rules
echo 'finished.'
