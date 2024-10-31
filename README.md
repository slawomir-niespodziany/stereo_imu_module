# Kernel module for Nvidia Jetson "stereo_imu" shield

Handles SPI communication with MMC5983MA magnetometer sensor and exposes data in /dev/stereo_imu file.

Setup:
    make
    sudo make install
    sudo depmod
    # append "imu"
    sudo vi /etc/modules-load.d/modules.conf
