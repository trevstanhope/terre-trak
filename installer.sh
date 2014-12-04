#!/bin/sh

update-rc.d -f lightdm remove
xset -dpms
xset s noblank
xset s off
apt-get install unclutter
cp configs/rc.local /etc/
chmod +x /etc/rc.local
cp configs/unclutter /etc/default/
cp configs/grub /etc/default
update-grub
cp configs/interfaces /etc/network


