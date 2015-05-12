# Agri-Vision
Multi-camera computer-vision guidance system for row-crop cultivators

## Installation
* Requires SciPy 0.11 or higher
* Requires OpenCV 2.4.6 or higher
* Requires NumPy 1.8 or higher
* Requires Python 2.7.6 __(NOT Python 3.x.x)__

To install the system, simply run the install script:
    
    sh install.sh

## Configure Asutostart launcher
Open the Settings --> Session and Startup --> Application Autostart
Add a new launcher named "AgriVision" and set the command to:

    ./root/agri-vision/autostart &
    
IMPORTANT: Disable the Power Manager. This will maximize CPU usage and prevent screen blanking.
