/*
 * Config file for Compulab CM-FX6 board
 *
 * Copyright (C) 2014, Compulab Ltd - http://compulab.co.il/
 *
 * Author: Nikita Kiryanov <nikita@compulab.co.il>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include <asm/arch/imx-regs.h>
#include <config_distro_defaults.h>
#include "mx6_common.h"

/* Machine config */
#define CONFIG_MX6
#define CONFIG_SYS_LITTLE_ENDIAN
#define CONFIG_MACH_TYPE		3769
#define CONFIG_SYS_HZ			1000

/* Display information on boot */
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO
#define CONFIG_TIMESTAMP

/* CMD */
#include <config_cmd_default.h>
#define CONFIG_CMD_GREPENV
#undef CONFIG_CMD_FLASH
#undef CONFIG_CMD_LOADB
#undef CONFIG_CMD_LOADS
#undef CONFIG_CMD_XIMG
#undef CONFIG_CMD_FPGA
#undef CONFIG_CMD_IMLS

/* MMC */
#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_USDHC_NUM	1
#define CONFIG_SYS_FSL_ESDHC_ADDR	0

/* RAM */
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR
#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_NR_DRAM_BANKS		1
#define CONFIG_SYS_MEMTEST_START	0x10000000
#define CONFIG_SYS_MEMTEST_END		0x10010000
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE
#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* Serial console */
#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART1_BASE
#define CONFIG_BAUDRATE			115200
#define CONFIG_SYS_BAUDRATE_TABLE	{9600, 19200, 38400, 57600, 115200}

/* Shell */
#define CONFIG_SYS_PROMPT	"U-Boot > "
#define CONFIG_SYS_CBSIZE	1024
#define CONFIG_SYS_MAXARGS	48
#define CONFIG_SYS_BARGSIZE	CONFIG_SYS_CBSIZE
#define CONFIG_SYS_PBSIZE	(CONFIG_SYS_CBSIZE + \
					sizeof(CONFIG_SYS_PROMPT) + 16)

/* SPI */
#define CONFIG_MXC_SPI
#define CONFIG_SYS_NO_FLASH

/* Environment */
#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_ENV_OVERWRITE
#define CONFIG_ENV_SIZE			(64 * 1024)
#define CONFIG_ENV_OFFSET		(512 * 1024) /* skip enough to make sure we're not writing in the U-boot image (500kb) */
#define CONFIG_SYS_MMC_ENV_DEV		0

/* Boot */
#define CONFIG_ZERO_BOOTDELAY_CHECK
#define CONFIG_LOADADDR			0x12000000
#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_CMDLINE_TAG		/* enable passing of ATAGs */
#define CONFIG_SYS_BOOTMAPSZ	        (8 << 20)
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

#define CONFIG_EXTRA_ENV_SETTINGS \
    "autoload=no\0" \
    "image=zImage\0" \
    "bootfile=vmlinuz-default\0" \
	"fdt_file=verkerk_vipbox-III_default.dtb\0" \
	"fdt_addr=0x18000000\0" \
	"fdt_high=0xffffffff\0"	  \
	"initrd_high=0xffffffff\0" \
	"rdaddr=0x12A00000\0" \
    "autostart=no\0" \
    "loadaddr=0x12000000\0" \
	"console=ttymxc1\0" \
	"ethaddr=00:04:9f:10:fe:1b\0" \
	"fec_addr=00:04:9f:10:fe:1b\0" \
	"loadimage=load mmc 0 ${loadaddr} /${bootfile}\0" \
	"loadrd=load mmc 0 ${rdaddr} /${rdfile}; setenv rdsize ${filesize}\0" \
	"loadfdt=echo loading /${fdt_file} ...;  load mmc 0 ${fdt_addr} /${fdt_file}\0" \
    "bootargs=console=ttymxc0,115200 root=/dev/mmcblk2p2 rootwait\0" \
    "mmcboot=run update_uenv; run uname_boot;\0" \
    "update_uenv=echo Checking for: /uEnv.txt ...;" \
			"if test -e mmc 0 /uEnv.txt; then " \
				"load mmc 0 ${loadaddr} /uEnv.txt;" \
				"env import -t ${loadaddr} ${filesize};" \
				"echo Loaded environment from /uEnv.txt;" \
				"echo Checking if uenvcmd is set ...;" \
				"if test -n ${uenvcmd}; then " \
					"echo Running uenvcmd ...;" \
					"run uenvcmd;" \
				"fi;" \
                "if test -n ${dtb}; then " \
                    "setenv fdt_file ${dtb};" \
                    "echo Using: dtb=${fdt_file} ...;" \
                "fi;" \
                "echo Checking if uname_r is set in /boot/uEnv.txt...;" \
                "if test -n ${uname_r}; then " \
                    "setenv bootfile vmlinuz-${uname_r}; " \
                    "echo Using: bootfile=${bootfile} ...;" \
                "fi;" \
			"fi;\0" \
    "uname_boot=echo Attempting boot...;"\
        "if test -e mmc 0 /${bootfile}; then " \
            "echo loading /${bootfile} ...; "\
			"run loadimage;" \
        	"if test -e mmc 0 /${fdt_file}; then " \
				"run loadfdt;" \
            "fi;" \
			"setenv rdfile initrd.img-${uname_r}; " \
            "if test -e mmc 0 /${rdfile}; then " \
				"echo loading /${rdfile} ...; "\
				"run loadrd;" \
				"if test -n ${uuid}; then " \
					"setenv mmcroot UUID=${uuid} ro;" \
				"fi;" \
				"echo debug: [${bootargs}] ... ;" \
				"echo debug: [bootz ${loadaddr} ${rdaddr}:${rdsize} ${fdt_addr}] ... ;" \
				"bootz ${loadaddr} ${rdaddr}:${rdsize} ${fdt_addr}; " \
			"else " \
				"echo debug: [${bootargs}] ... ;" \
				"echo debug: [bootz ${loadaddr} - ${fdt_addr}] ... ;" \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"fi;" \
		"fi;\0" \
    "bootcmd=run mmcboot;\0"


/* Ethernet */
#define CONFIG_FEC_MXC
#define CONFIG_FEC_MXC_PHYADDR		0
#define CONFIG_FEC_XCV_TYPE		RMII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL
#define CONFIG_MII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_ARP_TIMEOUT		200UL
#define CONFIG_NET_RETRY_COUNT		5

/* USB */
#define CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0
#define CONFIG_USB_MAX_CONTROLLER_COUNT	2
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET	/* For OTG port */

/* I2C */
#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_SPEED		100000

/* PMIC */
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE100
#define CONFIG_POWER_PFUZE100_I2C_ADDR	0x08


/* Eeprom */
#define CONFIG_CMD_EEPROM
/* these are for the ST M24C02 2kbit serial i2c eeprom */
#define CONFIG_SYS_I2C_EEPROM_ADDR	0x50		/* base address */
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN	1		/* bytes of address */
/* mask of address bits that overflow into the "EEPROM chip address"    */
/* #define CONFIG_SYS_I2C_EEPROM_ADDR_OVERFLOW	0x07 */

#define CONFIG_SYS_EEPROM_PAGE_WRITE_BITS	4	/* 16 byte write page size */
#define CONFIG_SYS_EEPROM_PAGE_WRITE_DELAY_MS	10	/* and takes up to 10 msec */


/* GPIO */
#define CONFIG_MXC_GPIO
#define CONFIG_CMD_GPIO

/* misc */
#define CONFIG_SYS_GENERIC_BOARD
#define CONFIG_STACKSIZE			(128 * 1024)
#define CONFIG_SYS_MALLOC_LEN			(10 * 1024 * 1024)
#define CONFIG_SYS_U_BOOT_MAX_SIZE_SECTORS	800 /* 400 KB */

/* SPL */
#include "imx6_spl.h"
#define CONFIG_SPL_BOARD_INIT
#define CONFIG_SPL_MMC_SUPPORT
#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR	0x80 /* offset 64 kb */
#define CONFIG_SYS_MONITOR_LEN	(CONFIG_SYS_U_BOOT_MAX_SIZE_SECTORS / 2 * 1024)

#endif	/* __CONFIG_CM_FX6_H */
