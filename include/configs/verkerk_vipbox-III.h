/*
 * Copyright (C) 2010-2011 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the PeterSime IHMI
 * and Freescale i.MX6Q Sabre Lite boards.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"
#define CONFIG_MX6
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO
#define CONFIG_SHOW_BOOT_PROGRESS
#define CONFIG_BOOTSTAGE 
#define CONFIG_BOOTSTAGE_REPORT

#define CONFIG_MACH_TYPE	3769

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h> 

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG
#define CONFIG_SYS_GENERIC_BOARD

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(100 * 1024 * 1024)


#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_MISC_INIT_R
#define CONFIG_MXC_GPIO
#define CONFIG_CMD_GPIO
#define CONFIG_CI_UDC
#define CONFIG_USBD_HS
#define CONFIG_USB_GADGET_DUALSPEED
#define CONFIG_USB_ETHER
#define CONFIG_USB_ETH_CDC
#define CONFIG_NETCONSOLE

/*#define CONFIG_HW_WATCHDOG*/
/* #define CONFIG_IMX_WATCHDOG */
/* #define CONFIG_WATCHDOG_TIMEOUT_MSECS 2000 */

#define CONFIG_CMD_FUSE
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE	       UART1_BASE

/* I2C Configs */
#define CONFIG_CMD_I2C
#define CONFIG_SYS_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_SPEED		100000
#define CONFIG_I2C_EDID

/* Eeprom */
#define CONFIG_CMD_EEPROM
/* these are for the ST M24C02 2kbit serial i2c eeprom */
#define CONFIG_SYS_I2C_EEPROM_ADDR	0x50		/* base address */
#define CONFIG_SYS_I2C_EEPROM_ADDR_LEN	1		/* bytes of address */
/* mask of address bits that overflow into the "EEPROM chip address"    */
/* #define CONFIG_SYS_I2C_EEPROM_ADDR_OVERFLOW	0x07 */

#define CONFIG_SYS_EEPROM_PAGE_WRITE_BITS	4	/* 16 byte write page size */
#define CONFIG_SYS_EEPROM_PAGE_WRITE_DELAY_MS	10	/* and takes up to 10 msec */

/* PMIC */
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE100
#define CONFIG_POWER_PFUZE100_I2C_ADDR	0x08

/* MMC Configs */
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR      0
#define CONFIG_SYS_FSL_USDHC_NUM       1

#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_EXT4
#define CONFIG_CMD_EXT4_WRITE
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION

/* ENET */
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		1
#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL
#define CONFIG_PHY_MICREL_KSZ8873

/* USB Configs */
#define CONFIG_CMD_USB
#define CONFIG_SUPPORT_VFAT
#define CONFIG_CMD_FAT
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE

#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET	/* For OTG port */
#define CONFIG_MXC_USB_PORTSC	(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS	0
#define CONFIG_SYS_USB_EVENT_POLL_VIA_CONTROL_EP

#define CONFIG_USB_GADGET
#define CONFIG_CMD_USB_MASS_STORAGE
#define CONFIG_USB_GADGET_MASS_STORAGE
#define CONFIG_USBDOWNLOAD_GADGET
#define CONFIG_USB_GADGET_VBUS_DRAW	2

/* Netchip IDs */
#define CONFIG_G_DNL_VENDOR_NUM 0x0525
#define CONFIG_G_DNL_PRODUCT_NUM 0xa4a5
#define CONFIG_G_DNL_MANUFACTURER "Boundary"

/* Miscellaneous commands */
#define CONFIG_CMD_SETEXPR

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX	       1
#define CONFIG_BAUDRATE			       115200

/* Command definition */
#include <config_cmd_default.h>

#undef CONFIG_CMD_IMLS

#define CONFIG_BOOTDELAY	       1

#define CONFIG_LOADADDR			       0x12000000
#define CONFIG_SYS_TEXT_BASE	       0x47800000

#ifdef CONFIG_CMD_MMC
#define CONFIG_DRIVE_MMC "mmc "
#else
#define CONFIG_DRIVE_MMC
#endif

#ifdef CONFIG_USB_STORAGE
#define CONFIG_DRIVE_USB "usb "
#else
#define CONFIG_DRIVE_USB
#endif

#define CONFIG_DRIVE_TYPES CONFIG_DRIVE_MMC CONFIG_DRIVE_USB
#define CONFIG_UMSDEVS CONFIG_DRIVE_MMC

#define CONFIG_EXTRA_ENV_SETTINGS \
	"bootdevs=" CONFIG_DRIVE_TYPES "\0" \
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
	"umsdevs=" CONFIG_UMSDEVS "\0" \
	"console=ttymxc1\0" \
	"ethaddr=00:04:9f:10:fe:1b\0" \
	"fec_addr=00:04:9f:10:fe:1b\0" \
	"loadimage=load mmc 0 ${loadaddr} /${bootfile}\0" \
	"loadrd=load mmc 0 ${rdaddr} /${rdfile}; setenv rdsize ${filesize}\0" \
	"loadfdt=echo loading /${fdt_file} ...;  load mmc 0 ${fdt_addr} /${fdt_file}\0" \
    "bootargs=console=ttymxc0,115200 root=/dev/mmcblk0p2 rootwait galcore.gpuProfiler=1;\0" \
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

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT	       "U-Boot > "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE	       1024

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS	       48
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_LOAD_ADDR	       CONFIG_LOADADDR

#define CONFIG_CMDLINE_EDITING

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS	       1
#define PHYS_SDRAM		       MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE	       PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR       IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE       IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE - 0x16)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)


#define CONFIG_SYS_MEMTEST_START       0x10000000
#define CONFIG_SYS_MEMTEST_END	       0x50000000
#define CONFIG_SYS_MEMTEST_SCRATCH     0x47800000
/* #define CONFIG_POST (CONFIG_SYS_POST_MEMORY|CONFIG_SYS_POST_MEM_REGIONS) */
/* #define CONFIG_SYS_POST_WORD_ADDR      CONFIG_SYS_INIT_SP_ADDR + GENERATED_GBL_DATA_SIZE + 0x8 */

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#define CONFIG_ENV_SIZE			(64 * 1024)

#define CONFIG_ENV_IS_IN_MMC

#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET		(6 * 64 * 1024)
#define CONFIG_SYS_MMC_ENV_DEV		0

#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_OFFSET		(8128 * 1024)
#define CONFIG_ENV_SECT_SIZE		(64 * 1024)
#define CONFIG_ENV_SPI_BUS		CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS		CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#endif

#define CONFIG_OF_LIBFDT

#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif


#define CONFIG_CMD_MEMTEST
#define CONFIG_SYS_ALT_MEMTEST

#define CONFIG_CMD_BOOTZ
#define CONFIG_SUPPORT_RAW_INITRD
#define CONFIG_CMD_FS_GENERIC
#define CONFIG_CMD_ELF

#endif	       /* __CONFIG_H */
