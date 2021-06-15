#!/bin/bash
# this file is here to run on first bootup as sudo 
# the purpose is to change a few things to better differentiate the Pi's from one another
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root"
    exit
fi

if [ $# -ne 1 ] 
    then echo "Usage: ./first_setup.sh {new_hostname}"
    exit
fi

echo $1 > /etc/hostname
rm /etc/machine-id
systemd-machine-id-setup
echo "setup complete. please reboot the system"
