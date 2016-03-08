#!/bin/bash
export CC=/opt/gcc-linaro-arm-linux-gnueabihf-4.9-2014.09_linux/bin/arm-linux-gnueabihf-

export PRODUCT_ID_NUMBER=15004  #<product_id_number>
export PRODUCT_VERSION_NUMBER=0      #<product_version_number>
export PRODUCT_REVISION_NUMBER=0      #<product_revision_number>
export PRODUCT_TEST_NUMBER=4      #<product_test_number>

echo ": psiversion: $PRODUCT_ID_NUMBER.$PRODUCT_VERSION_NUMBER.$PRODUCT_REVISION_NUMBER.$PRODUCT_TEST_NUMBER" > localversion

make ARCH=arm CROSS_COMPILE=${CC} distclean
make ARCH=arm CROSS_COMPILE=${CC} verkerk_vipbox-III_spl_config
make ARCH=arm CROSS_COMPILE=${CC}
make ARCH=arm CROSS_COMPILE=${CC} env

./tools/mkimage -n board/psicontrol/verkerk_vipbox-III_spl/verkerk_vipbox-III.cfg.cfgtmp -T imximage -e 0x00908000 -d spl/u-boot-spl.bin spl.img

dd if=/dev/zero count=500 bs=1K | tr '\000' '\377' > verkerk_spl_firmware.bin
dd if=spl.img of=verkerk_spl_firmware.bin bs=1K conv=notrunc && dd if=u-boot.img of=verkerk_spl_firmware.bin bs=1K seek=63 conv=notrunc

#mv u-boot.imx u-boot_petersime.imx
