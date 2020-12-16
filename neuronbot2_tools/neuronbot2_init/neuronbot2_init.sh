#!/bin/bash

# udev for NeuronBot2 TTY
echo  'KERNEL=="ttyUSB*", KERNELS!="1-5", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", GROUP:="dialout",  SYMLINK+="neuronbot2"' > /etc/udev/rules.d/neuronbot2.rules
status=$?
if [ $status -eq 0 ] 
then 
    echo "Initialized NeuronBot successfully"
else 
    echo "Failed to initialize NeuronBot!"
    exit 1
fi

# udev for Arduino LED type 1 & 2
echo 'KERNEL=="ttyUSB*", KERNELS=="1-5", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE:="0666", GROUP:="dialout", SYMLINK+="neuronbotLED"' > /etc/udev/rules.d/neuronbotLED.rules
echo 'KERNEL=="ttyUSB*", KERNELS=="1-5", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", GROUP:="dialout", SYMLINK+="neuronbotLED"' >> /etc/udev/rules.d/neuronbotLED.rules
status=$?
if [ $status -eq 0 ]
then 
    echo "Initialized LED controller successfully"
else
    echo "Failed to initialize LED controller!"
    exit 1
fi

# LED program depends on python-serial
sudo apt install -y python3 python3-serial
status=$?
if [ $status -eq 0 ]
then
    echo "Dependent packages for NeuronBot LED have been installed successfully"
else
    echo "Failed to download and install NeuronBot LED dependent packages!"
    exit 1
fi

# udev for RPLidar
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="rplidar"' > /etc/udev/rules.d/rplidar.rules
status=$?
if [ $status -eq 0 ]
then 
    echo "Initialized RPLidar successfully"
else
    echo "Failed to initialize RPLidar!"
    exit 1
fi

# Trgiier udev
sudo udevadm control --reload
sudo udevadm trigger
status=$?
if [ $status -eq 0 ]
then
    echo "Udev restarted successfully"
else
    echo "Failed to restart udev!"
    exit 1
fi
