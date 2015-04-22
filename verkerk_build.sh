#!/bin/sh
export CC=/opt/gcc-linaro-arm-linux-gnueabihf-4.9-2014.09_linux/bin/arm-linux-gnueabihf-

export PRODUCT_ID_NUMBER=15004  #<product_id_number>
export PRODUCT_VERSION_NUMBER=0      #<product_version_number>
export PRODUCT_REVISION_NUMBER=0      #<product_revision_number>
export PRODUCT_TEST_NUMBER=0      #<product_test_number>

echo ": psiversion: $PRODUCT_ID_NUMBER.$PRODUCT_VERSION_NUMBER.$PRODUCT_REVISION_NUMBER.$PRODUCT_TEST_NUMBER" > localversion

make ARCH=arm CROSS_COMPILE=${CC} distclean
make ARCH=arm CROSS_COMPILE=${CC} verkerk_vipbox-III_config
make ARCH=arm CROSS_COMPILE=${CC}
make ARCH=arm CROSS_COMPILE=${CC} env

mv u-boot.imx u-boot_verkerk.imx