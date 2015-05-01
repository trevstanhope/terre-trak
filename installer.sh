#!/bin/sh
# Rehydrate INSTALLER
# WARNING: This software makes significant changes to the system behavior
# DISCLAIMER: This software is distributed with no warranty.

echo "Creating clone of repository..."
cd ~
git clone https://github.com/trevstanhope/agri-vision
cd agri-vision

echo "Installing Dependencies via Apt..."
apt-get update
apt-get upgrade -y -qq
apt-get install unzip -y -qq
apt-get install build-essential -y -qq
apt-get install python-dev -y -qq
apt-get install cmake -y -qq
apt-get install python-serial -y -qq
apt-get install python-pip -y -qq
apt-get install python-gps -y -qq # python dependencies
apt-get install mongodb -y -qq# MongoDB
apt-get install gpsd -y -qq
apt-get install gpsd-clients -y -qq
apt-get install python-gps -y  -qq# GPS
apt-get install python-matplotlib -y -qq
apt-get install libgtk2.0-dev -y -qq
apt-get install python-numpy -y -qq
apt-get install libqt4-dev -y -qq
apt-get install libopencv-dev -y -qq
apt-get install build-essential -y -qq
apt-get install checkinstall -y -qq
apt-get install pkg-config -y -qq
apt-get install yasm -y -qq
apt-get install libjpeg-dev -y -qq
apt-get install libjasper-dev -y -qq
apt-get install libavcodec-dev -y -qq
apt-get install libavformat-dev -y -qq
apt-get install libswscale-dev -y -qq
apt-get install libdc1394-22-dev -y -qq
apt-get install libxine-dev -y -qq
apt-get install libgstreamer0.10-dev -y -qq
apt-get install libgstreamer-plugins-base0.10-dev -y -qq
apt-get install libv4l-dev -y -qq
apt-get install python-numpy -y -qq
apt-get install libtbb-dev -y -qq
apt-get install libqt4-dev -y -qq
apt-get install libgtk2.0-dev -y -qq
apt-get install libfaac-dev -y -qq
apt-get install libmp3lame-dev -y -qq
apt-get install libopencore-amrnb-dev -y -qq
apt-get install libopencore-amrwb-dev -y -qq
apt-get install libtheora-dev -y -qq
apt-get install libvorbis-dev -y -qq
apt-get install libxvidcore-dev -y -qq
apt-get install x264 -y -qq
apt-get install v4l-utils -y -qq
apt-get install arduino arduino-mk -qq

# OpenCV Dependency Fix
echo "Fix for AVformat"
cd /usr/include/linux
ln -s ../libv4l1-videodev.h videodev.h
ln -s ../libavformat/avformat.h avformat.h

# OpenCV
echo "Installing OpenCV..."
cd ~/agrivision
wget http://downloads.sourceforge.net/project/opencvlibrary/opencv-unix/2.4.9/opencv-2.4.9.zip
unzip opencv-2.4.9.zip
cd opencv-2.4.9
mkdir release
cd release
cmake -D CMAKE_BUILD_TYPE=RELEASE CMAKE_INSTALL_PREFIX=/usr/local ..
make -j4
make install

## Pip
echo "Installing Python modules..."
pip install pymongo

read -p "Are you sure you want to install [y/n]? This will REMOVE your DM from services! " ans
if [ $ans = y -o $ans = Y -o $ans = yes -o $ans = Yes -o $ans = YES ]
    then
        echo "Removing LightDM..."
        update-rc.d -f lightdm remove
        
        echo "Disabling Screen Blanking..."
        xset -dpms
        xset s noblank
        xset s off
        
        echo "Disabling Mouse ..."
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
