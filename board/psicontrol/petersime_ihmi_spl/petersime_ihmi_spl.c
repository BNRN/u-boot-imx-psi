/*
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <fsl_esdhc.h>
#include <malloc.h>
#include <miiphy.h>
#include <micrel.h>
#include <netdev.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/iomux.h>
#include <asm/imx-common/mxc_i2c.h>
#include <asm/imx-common/video.h>
#include <pwm.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <i2c.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include "common.h"

DECLARE_GLOBAL_DATA_PTR;


#ifdef CONFIG_SYS_I2C_MXC
#define I2C_PAD_CTRL	(PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | \
			PAD_CTL_DSE_40ohm | PAD_CTL_HYS | \
			PAD_CTL_ODE | PAD_CTL_SRE_FAST)

			
/* I2C1, Eeprom & RTC */
static struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		.i2c_mode = MX6_PAD_CSI0_DAT9__I2C1_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_CSI0_DAT9__GPIO5_IO27 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(5, 27)
	},
	.sda = {
		.i2c_mode = MX6_PAD_CSI0_DAT8__I2C1_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_CSI0_DAT8__GPIO5_IO26 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(5, 26)
	}
};

/* I2C2 PMIC */
static struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_KEY_COL3__I2C2_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_KEY_COL3__GPIO4_IO12 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = MX6_PAD_KEY_ROW3__I2C2_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_KEY_ROW3__GPIO4_IO13 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 13)
	}
};

/* I2C3 testpad */
static struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = MX6_PAD_EIM_D17__I2C3_SCL | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_EIM_D17__GPIO3_IO17 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(3, 17)
	},
	.sda = {
		.i2c_mode = MX6_PAD_EIM_D18__I2C3_SDA | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gpio_mode = MX6_PAD_EIM_D18__GPIO3_IO18 | MUX_PAD_CTRL(I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(3, 18)
	}
};	
		

static void petersime_ihmi_spl_setup_i2c(void)
{
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0); // RTC , eeprom
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1); // PMIC
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2); // testpad
}
#else
static void petersime_ihmi_spl_setup_i2c(void) { }
#endif

#ifdef CONFIG_USB_EHCI_MX6
int board_ehci_hcd_init(int port)
{
	return 0;
}

int board_ehci_power(int port, int on)
{
	return 0;
}

static iomux_v3_cfg_t const usb_otg_pads[] = {
	MX6_PAD_GPIO_1__USB_OTG_ID		| MUX_PAD_CTRL(WEAK_PULLUP),
	MX6_PAD_EIM_D21__USB_OTG_OC		| MUX_PAD_CTRL(WEAK_PULLUP),
	/* OTG Power enable */
	MX6_PAD_EIM_D22__GPIO3_IO22		| MUX_PAD_CTRL(WEAK_PULLDOWN)
};

static void setup_usb_otg(void)
{
	/* initialise USB OTG ID line */
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;

	clrsetbits_le32(&iomuxc_regs->gpr[1],
			IOMUXC_GPR1_OTG_ID_MASK,
			IOMUXC_GPR1_OTG_ID_GPIO1);

	imx_iomux_v3_setup_multiple_pads(usb_otg_pads, ARRAY_SIZE(usb_otg_pads));
}
#else
static void setup_usb_otg(void) {}
#endif

#ifdef CONFIG_FEC_MXC
#define ENET_PAD_CTRL		(PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED | \
				 PAD_CTL_DSE_40ohm | PAD_CTL_HYS)
#define ENET_PAD_CTRL_PULLDOWN  (PAD_CTL_PUS_100K_DOWN |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

static iomux_v3_cfg_t const enet_pads[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_MDC__ENET_MDC	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_TXD0__ENET_TX_DATA0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_TXD1__ENET_TX_DATA1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_TX_EN__ENET_TX_EN	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_RX_ER__ENET_RX_ER	| MUX_PAD_CTRL(ENET_PAD_CTRL_PULLDOWN),
	MX6_PAD_ENET_RXD0__ENET_RX_DATA0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_RXD1__ENET_RX_DATA1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET_CRS_DV__ENET_RX_EN	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_GPIO_19__ENET_TX_ER	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_GPIO_16__ENET_REF_CLK	| MUX_PAD_CTRL(ENET_PAD_CTRL),
	
	/* Micrel KS8873 switch Reset */
	MX6_PAD_NANDF_CS0__GPIO6_IO11	| MUX_PAD_CTRL(NO_PAD_CTRL),
	
};

static void setup_iomux_enet(void)
{

	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));
	/* Reset Micrel KS8873 switch */
	gpio_direction_output(IMX_GPIO_NR(6, 11) , 0);
	udelay(500);
	gpio_set_value(IMX_GPIO_NR(6, 11), 1);

}

int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
    {
	   phydev->drv->config(phydev);
	   printf("phy: %s\n",phydev->drv->name );
	}  
	return 0;
}

int board_eth_init(bd_t *bis)
{
	uint32_t base = IMX_FEC_BASE;
	struct mii_dev *bus = NULL;
	struct phy_device *phydev = NULL;
	int ret;
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;

	setup_iomux_enet();
	
	// ENKEL VOOR OMGEBOUWDE BOARDS
    char* enable_enet_clk_env = getenv("enable_enet_clk");
	if (NULL != enable_enet_clk_env && 0 == strcmp(enable_enet_clk_env, "1"))
    {
        printf("Enabling i.MX6 Ethernet clock\n");
        
        // enable enet clock gating (on for all power modes)
        enable_enet_clk(1);  
        
        // set PLL at 50 MHz
        enable_fec_anatop_clock(ENET_50MHz);
        
        // enable PLL (set gpr1[ENET_CLK_SEL])
        setbits_le32(&iomuxc_regs->gpr[1], IOMUXC_GPR1_ENET_CLK_SEL_MASK);          
    }

	bus = fec_get_miibus(base, -1);
	if (!bus)
	{
		printf("%s: fec_get_miibus failed\n", __func__);
	    return -ENOMEM;
    }
	/* check if phy registers are readable through RMII */
	phydev = phy_find_by_mask(bus, (0x0001 << CONFIG_FEC_MXC_PHYADDR), PHY_INTERFACE_MODE_RMII);
	if (!phydev) {
		printf("%s: can not find micrel switch\n", __func__);
		free(bus);
		return -ENOMEM;
	}

	ret = fec_probe(bis, -1, base, bus, phydev);
	if (ret) {
		printf("FEC MXC: %s:failed\n", __func__);
		free(phydev);
		free(bus);
		return -ENOMEM;		
	}
    ret = eth_init(bis);
	
	return ret;
}
#endif


#ifdef CONFIG_POWER_PFUZE100
static int pfuze_init(void)
{
	struct pmic *p;
	int ret;
	unsigned int reg;

	ret = power_pfuze100_init(1);
	if (ret)
		return ret;

	p = pmic_get("PFUZE100");
	ret = pmic_probe(p);
	if (ret)
		return ret;

	pmic_reg_read(p, PFUZE100_DEVICEID, &reg);
	printf("PMIC:  PFUZE100 ID=0x%02x\n", reg);
	return 0;
}
#else
static int pfuze_init(void) {}
#endif

#if defined(CONFIG_VIDEO_IPUV3)

static iomux_v3_cfg_t const backlight_pads[] = {
	/* Backlight on LVDS connector */
    /* BL_EN */
	MX6_PAD_DISP0_DAT10__GPIO4_IO31		| MUX_PAD_CTRL(OUTPUT_40OHM),
#define LVDS_BACKLIGHT_GP IMX_GPIO_NR(4, 31)
    /* PWM */
	MX6_PAD_DISP0_DAT9__PWM2_OUT		| MUX_PAD_CTRL(WEAK_PULLUP)
#define LVDS_BACKLIGHT_PWM  1
};

static int detect_chimei(struct display_info_t const *dev)
{
    char* panel = getenv("enable_pdp_panel");
    //printf("detecting PDP panel: %s\n", panel);
	return (NULL != panel && 0 == strcmp(panel, "1"));
}

static int detect_chefree_7in(struct display_info_t const *dev)
{
    char* panel = getenv("enable_7in_panel");
    //printf("detecting PDP panel: %s\n", panel);
	return (NULL != panel && 0 == strcmp(panel, "1"));
}

static void enable_lvds(struct display_info_t const *dev)
{
	struct iomuxc *iomux = (struct iomuxc *)
				IOMUXC_BASE_ADDR;
	u32 reg = readl(&iomux->gpr[2]);
	reg |= IOMUXC_GPR2_DATA_WIDTH_CH0_24BIT;
	writel(reg, &iomux->gpr[2]);
	gpio_direction_output(LVDS_BACKLIGHT_GP, 1);
}

struct display_info_t const displays[] = {{
    .bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= NULL,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "Texim Chefree CH121ILGL",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385, /* 65 MHz = 15385 */
		.left_margin    = 220,
		.right_margin   = 100,
		.upper_margin   = 21,
		.lower_margin   = 17,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 0,
	.addr	= 1,
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_chimei,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "Chimei G121S1-L02",
		.refresh        = 60,
		.xres           = 800,
		.yres           = 600,
		.pixclock       = 25000, /* 40 MHz */
		.left_margin    = 156,
		.right_margin   = 100,
		.upper_margin   = 145,
		.lower_margin   = 18,
		.hsync_len      = 60,
		.vsync_len      = 145,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 0,
	.addr	= 1,
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= detect_chefree_7in,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "Texim Chefree CH070DLDL-RT2",
		.refresh        = 60,
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 30303, /* 33 MHz = 30303 */
		.left_margin    = 128,
		.right_margin   = 128,
		.upper_margin   = 22,
		.lower_margin   = 23,
		.hsync_len      = 60,
		.vsync_len      = 22,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
/*} }, {
	.bus	= 2,
	.addr	= 0x4,
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= detect_i2c,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "Hannstar-XGA",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= NULL,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "LG-9.7",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 768,
		.pixclock       = 15385, 
		.left_margin    = 480,
		.right_margin   = 260,
		.upper_margin   = 16,
		.lower_margin   = 6,
		.hsync_len      = 250,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 2,
	.addr	= 0x38,
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= detect_i2c,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "wsvga-lvds",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 600,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 2,
	.addr	= 0x41,
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= detect_i2c,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "amp1024x600",
		.refresh        = 60,
		.xres           = 1024,
		.yres           = 600,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED
} }, {
	.bus	= 0,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_LVDS666,
	.detect	= 0,
	.enable	= enable_lvds,
	.mode	= {
		.name           = "wvga-lvds",
		.refresh        = 57,
		.xres           = 800,
		.yres           = 480,
		.pixclock       = 15385,
		.left_margin    = 220,
		.right_margin   = 40,
		.upper_margin   = 21,
		.lower_margin   = 7,
		.hsync_len      = 60,
		.vsync_len      = 10,
		.sync           = FB_SYNC_EXT,
		.vmode          = FB_VMODE_NONINTERLACED*/
} } };
size_t display_count = ARRAY_SIZE(displays);

int board_cfb_skip(void)
{
	return NULL != getenv("novideo");
}

static void setup_display(void)
{
	struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
	struct iomuxc *iomux = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int reg;
    int cs2cdr_value;

	enable_ipu_clock();
	/* imx_setup_hdmi(); */
	/* Turn on LDB0,IPU,IPU DI0 clocks */
	reg = __raw_readl(&mxc_ccm->CCGR3);
	reg |=  MXC_CCM_CCGR3_LDB_DI0_MASK;
	writel(reg, &mxc_ccm->CCGR3);
        
    if (detect_chimei(NULL) || detect_chefree_7in(NULL)) 
    {
        /* set LDB0 clk select to 001 -> select PFD0 306,58 MHz */
        /* then divide by 7 -> 43,7 MHz */
        cs2cdr_value = 1;
    }
    else
    {
        /* set LDB0 clk select to 100 -> select PLL3 480 MHz */
        /* then divide by 7 -> 68,57 MHz */
        cs2cdr_value = 4;
    }    
    
	reg = readl(&mxc_ccm->cs2cdr); 
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK); 
	reg |= (cs2cdr_value<<MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_OFFSET); 
	writel(reg, &mxc_ccm->cs2cdr); 

    /*set ldb_di0_ipu_div to 1 -> divide by 7 */
	reg = readl(&mxc_ccm->cscmr2); 
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV; 
	writel(reg, &mxc_ccm->cscmr2); 

    /* select ldb_di0_clk for ipu_di_clk_sel */
	reg = readl(&mxc_ccm->chsccdr); 
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0 
		<<MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr); 
    	
    
	reg = IOMUXC_GPR2_BGREF_RRMODE_EXTERNAL_RES
	     |IOMUXC_GPR2_DI1_VS_POLARITY_ACTIVE_HIGH
	     |IOMUXC_GPR2_DI0_VS_POLARITY_ACTIVE_LOW
	     |IOMUXC_GPR2_BIT_MAPPING_CH1_SPWG
	     |IOMUXC_GPR2_DATA_WIDTH_CH1_18BIT
	     |IOMUXC_GPR2_BIT_MAPPING_CH0_SPWG
	     |IOMUXC_GPR2_DATA_WIDTH_CH0_18BIT
	     |IOMUXC_GPR2_LVDS_CH1_MODE_DISABLED
	     |IOMUXC_GPR2_LVDS_CH0_MODE_ENABLED_DI0;
	writel(reg, &iomux->gpr[2]);

	reg = readl(&iomux->gpr[3]);
	reg = ( (reg & ~(IOMUXC_GPR3_LVDS0_MUX_CTL_MASK))
	       | (IOMUXC_GPR3_MUX_SRC_IPU1_DI0 <<IOMUXC_GPR3_LVDS0_MUX_CTL_OFFSET));
	writel(reg, &iomux->gpr[3]);

	/* backlight off for now */
	imx_iomux_v3_setup_multiple_pads(backlight_pads,
					 ARRAY_SIZE(backlight_pads));
	gpio_direction_input(LVDS_BACKLIGHT_GP);
    
    pwm_init(LVDS_BACKLIGHT_PWM, 0, 0);
    if (detect_chimei(NULL)) 
    {
        /* Chimei is 200 Hz */
        pwm_config(LVDS_BACKLIGHT_PWM, 4000000, 5000000);
    }
    else
    {
        /* Chefree is 20 KHz */
        pwm_config(LVDS_BACKLIGHT_PWM, 40000, 50000);
    }
    pwm_enable(LVDS_BACKLIGHT_PWM);
}


static int change_backlight_pwm(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
    uint32_t duty_cycle;
    uint32_t period;
    
    if (argc > 2)
    {
        duty_cycle = simple_strtoul(argv[1], NULL, 10);
        period = simple_strtoul(argv[2], NULL, 10);
    
        printf("change backlight: \n    duty cycle: %u ns\n    period: %u ns\n", duty_cycle, period);
        pwm_disable(LVDS_BACKLIGHT_PWM);
        pwm_config(LVDS_BACKLIGHT_PWM, duty_cycle, period);
        pwm_enable(LVDS_BACKLIGHT_PWM);
    }
    else
    {
        printf("change backlight pwm: too few arguments\n\n");
        return 1;
    }
    
    return 0;
}

static int change_backlight_enable(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
    uint32_t enable;

    if (argc > 1)
    {
        enable = (simple_strtoul(argv[1], NULL, 10) & 0x1);
        printf("set backlight enable to: %u\n\n", enable);
        gpio_set_value(LVDS_BACKLIGHT_GP, enable);
		return 0;
    }
    else
    {
        printf("change backlight enable: too few arguments\n\n");
        return 1;
    }
	
}


U_BOOT_CMD(
	blpwm, 3, 1, change_backlight_pwm,
	"Configure the backlight pwm: blpwm duty_cycle_in_ns clock_period_in_ns",
	"duty_cycle_in_ns clock_period_in_ns"
);

U_BOOT_CMD(
	blen, 2, 1, change_backlight_enable,
	"Enable/disable the backlight pwm: blen 1/0",
	"1/0"
);
#endif

int board_init(void)
{	
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	
	petersime_ihmi_spl_setup_spi();

	petersime_ihmi_spl_setup_i2c();
	
	setup_usb_otg();
	
	pfuze_init();	
	
#if defined(CONFIG_VIDEO_IPUV3)
	setup_display();
#endif

	enable_led(0, 0);
	
	return 0;
}

int checkboard(void)
{
    puts("Board: PeterSime HMI (with SPL)\n");

	return 0;
}

int dram_init(void)
{
	gd->ram_size = ((ulong)CONFIG_DDR_MB * 1024 * 1024);

	return 0;
}
