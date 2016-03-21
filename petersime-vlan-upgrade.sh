#!/bin/bash
#

#
echo "Starting petersime-vlan-upgrade.sh"
#
fw_setenv bootcmd_mmc 'mii write 1 0 1820;mii write 2 0 1820;fatload mmc 0 ${fdt_addr} /${fdt_file_name};fatload mmc 0 ${loadaddr} /uImage;bootm ${loadaddr} - ${fdt_addr};'
#
mount /dev/root / -o remount,rw
sed -i 's/echo "Starting petersime-switch.sh"/echo "Starting petersime-switch.sh"\nifconfig eth0 up\n/' /usr/share/vlan-scripts/petersime-switch.sh
echo 'write-mii-reg 29 0x0  #enable power on Port 1' >> /usr/share/vlan-scripts/petersime-switch.sh
echo 'write-mii-reg 45 0x0  #enable power on Port 2' >> /usr/share/vlan-scripts/petersime-switch.sh
sync
mount /dev/root / -o remount,ro
#

