#!/bin/bash
export CC=/opt/gcc-linaro-arm-linux-gnueabihf-4.9-2014.09_linux/bin/arm-linux-gnueabihf-

export PRODUCT_ID_NUMBER=17065  #<product_id_number>
export PRODUCT_VERSION_NUMBER=0      #<product_version_number>
export PRODUCT_REVISION_NUMBER=0      #<product_revision_number>
export PRODUCT_TEST_NUMBER=1      #<product_test_number>

echo ": psiversion: $PRODUCT_ID_NUMBER.$PRODUCT_VERSION_NUMBER.$PRODUCT_REVISION_NUMBER.$PRODUCT_TEST_NUMBER" > localversion

make ARCH=arm CROSS_COMPILE=${CC} distclean
make ARCH=arm CROSS_COMPILE=${CC} hotraco_hrfocus_spl_config
make ARCH=arm CROSS_COMPILE=${CC}
make ARCH=arm CROSS_COMPILE=${CC} env

./tools/mkimage -n board/psicontrol/hotraco_hrfocus_spl/hotraco_hrfocus.cfg.cfgtmp -T imximage -e 0x00908000 -d spl/u-boot-spl.bin spl.img

# combine SPL & U-boot into one image to be programmed at offset 0
dd if=/dev/zero count=1 bs=1M | tr '\000' '\377' > hotraco_spl_firmware.bin
dd if=spl.img of=hotraco_spl_firmware.bin bs=1K seek=1 conv=notrunc && dd if=u-boot.img of=hotraco_spl_firmware.bin bs=1K seek=64 conv=notrunc
