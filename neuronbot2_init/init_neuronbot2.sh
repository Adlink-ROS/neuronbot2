#!/bin/bash

# udev for NeuronBot2  TTY
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", GROUP:="dialout",  SYMLINK+="neuronbot2"' >/etc/udev/rules.d/neuronbot2.rules
status=$?
if [ $status -eq 0 ] 
then 
    echo "Initialized NeuronBot successfully"
else 
    echo "Failed to initilize NeuronBot!"
    exit 1
fi

# udev for RPLidar
sudo cp rplidar.rules /etc/udev/rules.d/
status=$?
if [ $status -eq 0 ]
then 
    echo "Initialized RPLidar successfully"
else
    echo "Failed to initilize RPLidar!"
    exit 1
fi

# Trgiier udev
sudo udevadm control --reload
sudo udevadm trigger
status=$?
if [ $status -eq 0 ]
then
    echo "Udev restarted successfully."
else
    echo "Failed to restart udev!"
    exit 1
fi
