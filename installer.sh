#!/bin/sh
# Rehydrate INSTALLER
# WARNING: This software makes significant changes to the system behavior
# DISCLAIMER: This software is distributed with no warranty.

read -p "Are you sure you want to install [y/n]? " ans
if [ $ans = y -o $ans = Y -o $ans = yes -o $ans = Yes -o $ans = YES ]
    then
        echo "Removing LightDM..."
        update-rc.d -f lightdm remove
        
        echo "Disabling Screen Blanking..."
        xset -dpms
        xset s noblank
        xset s off
        apt-get install unclutter
        cp configs/unclutter /etc/default/
        
        echo "Installing to Boot Path..."
        cp configs/rc.local /etc/
        chmod +x /etc/rc.local
        
        echo "Updating Custom Grub..."
        cp configs/grub /etc/default
        update-grub
        
        echo "Reconfiguring Network Interfaces..."
        cp configs/interfaces /etc/network
fi
if [ $ans = n -o $ans = N -o $ans = no -o $ans = No -o $ans = NO ]
    then
        echo "Aborting..."
fi
