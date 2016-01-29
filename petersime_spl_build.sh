#!/bin/bash
export CC=/opt/gcc-linaro-arm-linux-gnueabihf-4.9-2014.09_linux/bin/arm-linux-gnueabihf-

export PRODUCT_ID_NUMBER=14081  #<product_id_number>
export PRODUCT_VERSION_NUMBER=0      #<product_version_number>
export PRODUCT_REVISION_NUMBER=0      #<product_revision_number>
export PRODUCT_TEST_NUMBER=5      #<product_test_number>

echo ": psiversion: $PRODUCT_ID_NUMBER.$PRODUCT_VERSION_NUMBER.$PRODUCT_REVISION_NUMBER.$PRODUCT_TEST_NUMBER" > localversion

make ARCH=arm CROSS_COMPILE=${CC} distclean
make ARCH=arm CROSS_COMPILE=${CC} petersime_ihmi_spl_config
make ARCH=arm CROSS_COMPILE=${CC}
make ARCH=arm CROSS_COMPILE=${CC} env

./tools/mkimage -n board/psicontrol/petersime_ihmi_spl/petersime_ihmi.cfg.cfgtmp -T imximage -e 0x00908000 -d spl/u-boot-spl.bin spl.img

# bitmap is 2.5 MB, at offset 2 MB. So I will allocate 5 MB to be sure.
dd if=/dev/zero count=5 bs=1M | tr '\000' '\377' > petersime_spl_firmware.bin
dd if=spl.img of=petersime_spl_firmware.bin bs=1K seek=1 conv=notrunc && dd if=u-boot.img of=petersime_spl_firmware.bin bs=1K seek=64 conv=notrunc && dd if=no_sd_1024.bmp of=petersime_spl_firmware.bin bs=1K conv=notrunc seek=2048

#mv u-boot.imx u-boot_petersime.imx
