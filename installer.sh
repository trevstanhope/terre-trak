#!/bin/sh
# Rehydrate INSTALLER
# WARNING: This software makes significant changes to the system behavior
# DISCLAIMER: This software is distributed with no warranty.

INSTALL_PATH=/root/agri-vision
cd $INSTALL_PATH

# Settings File
read -p "Create settings.cfg [y/n]? " ans
if [ $ans = y -o $ans = Y -o $ans = yes -o $ans = Yes -o $ans = YES ]
    then
        read -p "Enter the relative path to the settings file: " ans
        echo $ans > settings.cfg
fi
if [ $ans = n -o $ans = N -o $ans = no -o $ans = No -o $ans = NO ]
    then
        echo "Aborting..."
fi


# LightDM
read -p "Do you want to disable LightDM [y/n]? " ans
if [ $ans = y -o $ans = Y -o $ans = yes -o $ans = Yes -o $ans = YES ]
    then
        echo "Removing LightDM..."
        update-rc.d -f lightdm remove || service lightdm remove
fi
if [ $ans = n -o $ans = N -o $ans = no -o $ans = No -o $ans = NO ]
    then
        echo "Aborting..."
fi

# Mouse
read -p "Disable on-screen mouse [y/n]? " ans
if [ $ans = y -o $ans = Y -o $ans = yes -o $ans = Yes -o $ans = YES ]
    then     
        echo "Disabling Mouse ..."
        cd $INSTALL_PATH
        apt-get install unclutter
        cp configs/unclutter /etc/default/
fi
if [ $ans = n -o $ans = N -o $ans = no -o $ans = No -o $ans = NO ]
    then
        echo "Aborting..."
fi

# Boot
read -p "Start AgriVision on boot? [y/n] " ans
if [ $ans = y -o $ans = Y -o $ans = yes -o $ans = Yes -o $ans = YES ]
    then
        echo "Installing to Boot Path..."
        cd $INSTALL_PATH
        cp bin/rc.local /etc/
        chmod +x /etc/rc.local
	cp configs/Agri-Vision.desktop /root/.config/autostart
fi
if [ $ans = n -o $ans = N -o $ans = no -o $ans = No -o $ans = NO ]
    then
        echo "Aborting..."
fi

# GRUB
read -p "Enable fast GRUB? [y/n] " ans
if [ $ans = y -o $ans = Y -o $ans = yes -o $ans = Yes -o $ans = YES ]
    then
        echo "Updating Custom Grub..."
        cd $INSTALL_PATH
        cp configs/grub /etc/default
        update-grub
fi
if [ $ans = n -o $ans = N -o $ans = no -o $ans = No -o $ans = NO ]
    then
        echo "Aborting..."
fi

# Network Setup
read -p "Setup default network? [y/n] " ans
if [ $ans = y -o $ans = Y -o $ans = yes -o $ans = Yes -o $ans = YES ]
    then
        echo "Reconfiguring Network Interfaces..."
        cd $INSTALL_PATH
        cp configs/interfaces /etc/network
fi
if [ $ans = n -o $ans = N -o $ans = no -o $ans = No -o $ans = NO ]
    then
        echo "Aborting..."
fi

# APT Packages
read -p "Update APT dependencies? [y/n] " ans
if [ $ans = y -o $ans = Y -o $ans = yes -o $ans = Yes -o $ans = YES ]
    then
        echo "Installing dependencies via mirror ..."
        apt-get update
        apt-get upgrade -y -qq
        apt-get install unzip -y -qq
        apt-get install build-essential -y -qq
        apt-get install python-dev -y -qq
        apt-get install cmake -y -qq
        apt-get install python-serial -y -qq
        apt-get install python-pip -y -qq
        apt-get install python-gps -y -qq # python dependencies
        apt-get install mongodb -y -qq # MongoDB
        apt-get install gpsd -y -qq
        apt-get install gpsd-clients -y -qq
        apt-get install python-gps -y  -qq # GPS
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
        apt-get install x11-xserver-utils -y -qq
fi
if [ $ans = n -o $ans = N -o $ans = no -o $ans = No -o $ans = NO ]
    then
        echo "Aborting..."
fi

# Pip Packages
read -p "Update Pip modules? [y/n] " ans
if [ $ans = y -o $ans = Y -o $ans = yes -o $ans = Yes -o $ans = YES ]
    then
        echo "Installing Python modules..."
        pip install pymongo
fi
if [ $ans = n -o $ans = N -o $ans = no -o $ans = No -o $ans = NO ]
    then
        echo "Aborting..."
fi

# OpenCV
read -p "Recompile OpenCV? [y/n] " ans
if [ $ans = y -o $ans = Y -o $ans = yes -o $ans = Yes -o $ans = YES ]
    then
        echo "Running fix for AVformat ..."
        cd /usr/include/linux
        ln -s ../libv4l1-videodev.h videodev.h
        ln -s ../libavformat/avformat.h avformat.h
        echo "Installing OpenCV ..."
        wget http://downloads.sourceforge.net/project/opencvlibrary/opencv-unix/2.4.9/opencv-2.4.9.zip -q
        unzip -qq opencv-2.4.9.zip
        cd opencv-2.4.9
        mkdir release
        cd release
        cmake -D CMAKE_BUILD_TYPE=RELEASE CMAKE_INSTALL_PREFIX=/usr/local ..
        make -j4
        make install
fi
if [ $ans = n -o $ans = N -o $ans = no -o $ans = No -o $ans = NO ]
    then
        echo "Aborting..."
fi

# Done Message
echo "Installation Complete!"
