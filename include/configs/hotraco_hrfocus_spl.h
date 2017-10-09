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
#define CONFIG_SYS_FSL_USDHC_NUM	2
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
#define CONFIG_SPI_FLASH
#define CONFIG_SPI_FLASH_SPANSION
#define CONFIG_SPI_FLASH_STMICRO
#define CONFIG_SPI_FLASH_ATMEL
#define CONFIG_SPI_FLASH_EON
#define CONFIG_SPI_FLASH_GIGADEVICE
#define CONFIG_SPI_FLASH_MACRONIX
#define CONFIG_SPI_FLASH_WINBOND

/* SPI flash */
#define CONFIG_SYS_NO_FLASH
#define CONFIG_CMD_SF
#define CONFIG_SF_DEFAULT_BUS		2
#define CONFIG_SF_DEFAULT_CS		1
#define CONFIG_SF_DEFAULT_SPEED		25000000
#define CONFIG_SF_DEFAULT_MODE		(SPI_MODE_0)

/* Environment */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_ENV_IS_IN_SPI_FLASH
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_BUS		CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS		CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SECT_SIZE		(64 * 1024)
#define CONFIG_ENV_SIZE			(64 * 1024)
#define CONFIG_ENV_OFFSET		(8128 * 1024)

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
    "autostart=no\0" \
    "loadaddr=0x12000000\0" \
    "fdt_addr=20000000\0" \
	"console=ttymxc1\0" \
    "enable_pdp_panel=0\0" \
    "enable_7in_panel=0\0" \
	"fdt_high=0xffffffff\0" \
	"ethaddr=00:04:9f:00:ea:d4\0" \
	"fec_addr=00:04:9f:00:ea:d4\0" \
    "serverip=10.0.182.252\0" \
    "fdt_file_name=hotraco_hrfocus.dtb\0" \
    "mmc_device=1\0" \
    "bootargs_emmc=console=ttymxc0,115200 root=/dev/mmcblk3p2 rootwait\0" \
    "bootargs_mmc=console=ttymxc0,115200 root=/dev/mmcblk2p2 rootwait\0" \
    "bootargs_usb=console=ttymxc0,115200 root=/dev/sda2 rootwait\0" \
    "bootargs=${bootargs_emmc}\0" \
    "bootcmd=run save_env_first_boot; run check_boot_device; run bootcmd_mmc;\0" \
    "image=petersime.bmp\0" \
    "loadimage_mmc=fatload mmc 0 11000000 ${image}; bmp disp 11000000\0" \
    "bootcmd_mmc=fatload mmc ${mmc_device} ${fdt_addr} /${fdt_file_name};fatload mmc ${mmc_device} ${loadaddr} /uImage;bootm ${loadaddr} - ${fdt_addr};\0" \
    "bootcmd_usb=usb start;setenv bootargs ${bootargs_usb};fatload usb 0:1 ${fdt_addr} /${fdt_file_name};fatload usb 0:1 ${loadaddr} /uImage;bootm ${loadaddr} - ${fdt_addr};\0" \
    "environment_written=0\0" \
    "save_env_first_boot=if test ${environment_written} != \"1\"; then " \
        "setenv environment_written 1;" \
        "saveenv;" \
        "fi;\0" \
    "check_boot_device=if test ${mmc_device} = \"0\"; then " \
            "setenv bootargs ${bootargs_mmc};" \
        "fi;\0" \
	"reflash_uboot=sf probe;" \
		"sf erase 0x00 +0x600000;" \
		"tftpboot 0x11000000 hotraco_hr_focus_spl.bin;" \
		"sf write 0x11000000 0x0 0x600000\0"
		

#define CONFIG_BOOTCOMMAND \
	"run setboottypem; run boot"

/* Ethernet */
#define CONFIG_FEC_MXC
#define CONFIG_FEC_MXC_PHYADDR		1
#define CONFIG_FEC_XCV_TYPE		RMII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL
#define CONFIG_PHY_MICREL_KSZ8873
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

/* RTC */
#define CONFIG_CMD_DATE
#define CONFIG_RTC_M41T11
#define CONFIG_SYS_I2C_RTC_ADDR		    0x68
#define CONFIG_SYS_M41T11_BASE_YEAR     2000


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

/* Video */
/* Framebuffer and LCD */
#define CONFIG_VIDEO /* enable video */
#define CONFIG_VIDEO_IPUV3 /* enable i.MX6 image processing unit */
#define CONFIG_CFB_CONSOLE /* set console on display */
#define CONFIG_IMX_VIDEO_SKIP /* activate the i.MX6 panel detection, which we override */
#define CONFIG_VGA_AS_SINGLE_DEVICE /* don't load keyboard driver , only display */
#define CONFIG_CONSOLE_MUX /* muxing the console between serial and display */
#define CONFIG_SYS_CONSOLE_IS_IN_ENV /* first say it can be in environment */
/*#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE*/ /*  then override it and say it's serial anyways */
#define CONFIG_VIDEO_BMP_RLE8  /* 8 bit color characters! */
#define CONFIG_SPLASH_SCREEN /* enable splash screen */
#define CONFIG_BMP_16BPP /* in 16BPP */
#define CONFIG_IPUV3_CLK 43857143 /* dummy value, not actually true */
#define CONFIG_CMD_BMP

/* backlight PWM */
#define CONFIG_PWM_IMX
#define CONFIG_IMX6_PWM_PER_CLK	66000000

/* misc */
#define CONFIG_SYS_GENERIC_BOARD
#define CONFIG_STACKSIZE			(128 * 1024)
#define CONFIG_SYS_MALLOC_LEN			(10 * 1024 * 1024)
#define CONFIG_SYS_U_BOOT_MAX_SIZE_SECTORS	800 /* 400 KB */
#define CONFIG_BOARD_LATE_INIT   /* use late init to guarantee delay between LVDS enable and backlight enable */

/* SPL */
#include "imx6_spl.h"
#define CONFIG_SPL_BOARD_INIT
#define CONFIG_SPL_MMC_SUPPORT
#define CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR	0x80 /* offset 64 kb */
#define CONFIG_SYS_MONITOR_LEN	(CONFIG_SYS_U_BOOT_MAX_SIZE_SECTORS / 2 * 1024)

#define CONFIG_SPL_SPI_SUPPORT
#define CONFIG_SPL_SPI_FLASH_SUPPORT
#define CONFIG_SYS_SPI_U_BOOT_OFFS	(64 * 1024)
#define CONFIG_SPL_SPI_LOAD



#endif	/* __CONFIG_CM_FX6_H */
