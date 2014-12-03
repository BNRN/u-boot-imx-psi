#!/bin/sh
export CC=/opt/gcc-linaro-arm-linux-gnueabihf-4.9-2014.09_linux/bin/arm-linux-gnueabihf-

make ARCH=arm CROSS_COMPILE=${CC} distclean
make ARCH=arm CROSS_COMPILE=${CC} petersime_ihmi_config
make ARCH=arm CROSS_COMPILE=${CC} 


