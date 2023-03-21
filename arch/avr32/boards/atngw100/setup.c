/*
 * Board-specific setup code for the ATNGW100 Network Gateway
 *
 * Copyright (C) 2005-2006 Atmel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/clk.h>
#include <linux/etherdevice.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/i2c-gpio.h>
#include <linux/init.h>
#include <linux/linkage.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/leds.h>
#include <linux/mmc/core.h>
#include <linux/mmc/host.h>
#include <linux/spi/mmc_spi.h>
#include <linux/spi/spi.h>
#include <linux/atmel-mci.h>
#include <linux/usb/atmel_usba_udc.h>
#include <linux/er_tft023_1.h>

#include <asm/io.h>
#include <asm/setup.h>

#include <mach/at32ap700x.h>
#include <mach/board.h>
#include <mach/init.h>
#include <mach/portmux.h>

#include <sound/atmel-abdac.h>

#include <video/atmel_lcdc.h>

static struct fb_videomode __initdata ltv350qv_modes[] = {
	{
		.name		= "320x240 @ 75",
		.refresh	= 75,
		.xres		= 320,		.yres		= 240,
		.pixclock	= KHZ2PICOS(6891),

		.left_margin	= 17,		.right_margin	= 33,
		.upper_margin	= 10,		.lower_margin	= 10,
		.hsync_len	= 16,		.vsync_len	= 1,

		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
};

static struct fb_monspecs __initdata atstk1000_default_monspecs = {
	.manufacturer		= "SNG",
	.monitor		= "LTV350QV",
	.modedb			= ltv350qv_modes,
	.modedb_len		= ARRAY_SIZE(ltv350qv_modes),
	.hfmin			= 14820,
	.hfmax			= 22230,
	.vfmin			= 60,
	.vfmax			= 90,
	.dclkmax		= 30000000,
};

static struct fb_videomode __initdata vga_modes[] = {
	{
		.name		= "640x480 @ 60",
		.refresh	= 60,
		.xres		= 640,		.yres		= 480,
		.pixclock	= KHZ2PICOS(25175),

		.left_margin	= 48,		.right_margin	= 16,
		.upper_margin	= 31,		.lower_margin	= 11,
		.hsync_len	= 96,		.vsync_len	= 2,

		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
};

static struct fb_monspecs __initdata vga_default_monspecs = {
	.manufacturer		= "VESA",
	.monitor		= "VGA",
	.modedb			= vga_modes,
	.modedb_len		= ARRAY_SIZE(vga_modes),
	.hfmin			= 31000,
	.hfmax			= 32000,
	.vfmin			= 59,
	.vfmax			= 61,
	.dclkmax		= 25200000,
};

static struct fb_videomode __initdata vga_13h_modes[] = {
	{
		.name		= "160x480 @ 60",
		.refresh	= 60,
		.xres		= 160,		.yres		= 480,
		.pixclock	= KHZ2PICOS(6294),

		.left_margin	= 12,		.right_margin	= 4,
		.upper_margin	= 31,		.lower_margin	= 11,
		.hsync_len	= 24,		.vsync_len	= 2,

		.sync		= /* FB_SYNC_VERT_HIGH_ACT */ 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
};

static struct fb_monspecs __initdata vga_13h_monspecs = {
	.manufacturer		= "VESA",
	.monitor		= "VGA",
	.modedb			= vga_13h_modes,
	.modedb_len		= ARRAY_SIZE(vga_13h_modes),
	.hfmin			= 28000,
	.hfmax			= 32000,
	.vfmin			= 59,
	.vfmax			= 61,
	.dclkmax		= 6500,
};

static struct fb_videomode __initdata er_tft023_1_modes[] = {
	{
		.name		= "320x240 @ 70",
		.refresh	= 70,
		.xres		= 320,		.yres		= 240,
		.pixclock	= KHZ2PICOS(6350),

		.left_margin	= 10,		.right_margin	= 20,
		.upper_margin	= 1,		.lower_margin	= 5,
		.hsync_len	= 10,		.vsync_len	= 2,

		.sync		= /* FB_SYNC_VERT_HIGH_ACT */ 0,
		.vmode		= FB_VMODE_NONINTERLACED,
	},
};

static struct fb_monspecs __initdata er_tft023_1_monspecs = {
	.manufacturer		= "EastRising",
	.monitor		= "RGB panel",
	.modedb			= er_tft023_1_modes,
	.modedb_len		= ARRAY_SIZE(er_tft023_1_modes),
	.hfmin			= 28000,
	.hfmax			= 32000,
	.vfmin			= 68,
	.vfmax			= 72,
	.dclkmax		= 6500,
};

struct atmel_lcdfb_pdata __initdata atngw_lcdc_data = {
	.default_bpp		= 16,
	.default_dmacon		= ATMEL_LCDC_DMAEN | ATMEL_LCDC_DMA2DEN,
	.default_lcdcon2	= (ATMEL_LCDC_DISTYPE_TFT
				   | ATMEL_LCDC_INVCLK
				   | ATMEL_LCDC_CLKMOD_ALWAYSACTIVE
				   | ATMEL_LCDC_MEMOR_BIG),
	.default_monspecs	= &er_tft023_1_monspecs,
	.guard_time		= 2,
	.lcd_wiring_mode	= ATMEL_LCDC_WIRING_RGB,
};

/*
#define LCDC_CONTROL (							\
		ATMEL_LCDC(PC, HSYNC) | ATMEL_LCDC(PC, VSYNC) |		\
		ATMEL_LCDC(PE, DVAL))

#define LCDC_DATA_RED (							\
		ATMEL_LCDC(PD, DATA7) | ATMEL_LCDC(PD, DATA6) |		\
		ATMEL_LCDC(PC, DATA5))

#define LCDC_DATA_GREEN (						\
		ATMEL_LCDC(PD, DATA15) | ATMEL_LCDC(PD, DATA14) |	\
		ATMEL_LCDC(PD, DATA13))

#define LCDC_DATA_BLUE (						\
		ATMEL_LCDC(PD, DATA23) | ATMEL_LCDC(PD, DATA22) |	\
		ATMEL_LCDC(PE, DATA21))
*/

#define LCDC_CONTROL (							\
		ATMEL_LCDC(PC, HSYNC) | ATMEL_LCDC(PC, PCLK) |		\
		ATMEL_LCDC(PC, VSYNC) | ATMEL_LCDC(PC, DVAL))

#define LCDC_DATA_RED (							\
		ATMEL_LCDC(PD, DATA14) | ATMEL_LCDC(PD, DATA21) |	\
		ATMEL_LCDC(PD, DATA22) | ATMEL_LCDC(PD, DATA23) |	\
		ATMEL_LCDC(PC, DATA2))

#define LCDC_DATA_GREEN (						\
		ATMEL_LCDC(PD, DATA11) | ATMEL_LCDC(PD, DATA12) |	\
		ATMEL_LCDC(PD, DATA13) | ATMEL_LCDC(PD, DATA14) |	\
		ATMEL_LCDC(PD, DATA15) | ATMEL_LCDC(PD, DATA19))

#define LCDC_DATA_BLUE (						\
		ATMEL_LCDC(PC, DATA3) | ATMEL_LCDC(PC, DATA4) |		\
		ATMEL_LCDC(PC, DATA5) | ATMEL_LCDC(PD, DATA6) |		\
		ATMEL_LCDC(PD, DATA7))

#define LCDC_IOMUX (LCDC_DATA_RED | LCDC_DATA_GREEN | LCDC_DATA_BLUE | LCDC_CONTROL)

/* Oscillator frequencies. These are board-specific */
unsigned long at32_board_osc_rates[3] = {
	[0] = 32768,	/* 32.768 kHz on RTC osc */
	[1] = 20000000,	/* 20 MHz on osc0 */
	[2] = 12000000,	/* 12 MHz on osc1 */
};

/*
 * The ATNGW100 mkII is very similar to the ATNGW100. Both have the AT32AP7000
 * chip on board; the difference is that the ATNGW100 mkII has 128 MB 32-bit
 * SDRAM (the ATNGW100 has 32 MB 16-bit SDRAM) and 256 MB 16-bit NAND flash
 * (the ATNGW100 has none.)
 *
 * The RAM difference is handled by the boot loader, so the only difference we
 * end up handling here is the NAND flash, EBI pin reservation and if LCDC or
 * MACB1 should be enabled.
 */
#ifdef CONFIG_BOARD_ATNGW100_MKII
#include <linux/mtd/partitions.h>
#include <mach/smc.h>

static struct smc_timing nand_timing __initdata = {
	.ncs_read_setup		= 0,
	.nrd_setup		= 10,
	.ncs_write_setup	= 0,
	.nwe_setup		= 10,

	.ncs_read_pulse		= 30,
	.nrd_pulse		= 15,
	.ncs_write_pulse	= 30,
	.nwe_pulse		= 15,

	.read_cycle		= 30,
	.write_cycle		= 30,

	.ncs_read_recover	= 0,
	.nrd_recover		= 15,
	.ncs_write_recover	= 0,
	/* WE# high -> RE# low min 60 ns */
	.nwe_recover		= 50,
};

static struct smc_config nand_config __initdata = {
	.bus_width		= 2,
	.nrd_controlled		= 1,
	.nwe_controlled		= 1,
	.nwait_mode		= 0,
	.byte_write		= 0,
	.tdf_cycles		= 2,
	.tdf_mode		= 0,
};

static struct mtd_partition nand_partitions[] = {
	{
		.name		= "main",
		.offset		= 0x00000000,
		.size		= MTDPART_SIZ_FULL,
	},
};


static struct atmel_nand_data atngw100mkii_nand_data __initdata = {
	.cle		= 21,
	.ale		= 22,
	.det_pin	= -1,
	.rdy_pin	= GPIO_PIN_PB(28),
	.enable_pin	= GPIO_PIN_PE(23),
	.bus_width_16	= true,
	.ecc_mode	= NAND_ECC_SOFT,
	.parts		= nand_partitions,
	.num_parts	= ARRAY_SIZE(nand_partitions),
//	.has_dma	= true
};
#endif

/* Initialized by bootloader-specific startup code. */
struct tag *bootloader_tags __initdata;

struct eth_addr {
	u8 addr[6];
};
static struct eth_addr __initdata hw_addr[2];
static struct macb_platform_data __initdata eth_data[2];

static const struct er_tft023_1_data panel_data = {
	.reset_gpio = GPIO_PIN_PA(8),
	.dc_gpio = GPIO_PIN_PA(5)
};

static struct spi_board_info spi0_board_info[] __initdata = {
	{
		.modalias	= "mtd_dataflash",
		.max_speed_hz	= 8000000,
		.chip_select	= 0,
	},
	{
		.modalias	= "panel_er_tft023_1",
		.max_speed_hz	= 1000000,
		.chip_select	= 1,
		.platform_data	= &panel_data,
	},
};

struct mmc_spi_platform_data mmc_spi_pdata = {
	.caps = MMC_CAP_NEEDS_POLL
};

static struct spi_board_info spi1_board_info[] __initdata = {
	{
		.modalias	= "mmc_spi",
		.max_speed_hz	= 25000000,
		.chip_select	= 0,
		.platform_data	= &mmc_spi_pdata,
	},
};

static struct mci_platform_data __initdata mci0_data = {
	.slot[0] = {
		.bus_width	= 4,
#if defined(CONFIG_BOARD_ATNGW100_MKII)
		.detect_pin	= GPIO_PIN_PC(25),
		.wp_pin		= GPIO_PIN_PE(22),
#else
		.detect_pin	= GPIO_PIN_PC(25),
		.wp_pin		= GPIO_PIN_PE(0),
#endif
	},
};

static struct usba_platform_data atngw100_usba_data __initdata = {
#if defined(CONFIG_BOARD_ATNGW100_MKII)
	.vbus_pin	= GPIO_PIN_PE(26),
#else
	.vbus_pin	= -ENODEV,
#endif
};

/*
 * The next two functions should go away as the boot loader is
 * supposed to initialize the macb address registers with a valid
 * ethernet address. But we need to keep it around for a while until
 * we can be reasonably sure the boot loader does this.
 *
 * The phy_id is ignored as the driver will probe for it.
 */
static int __init parse_tag_ethernet(struct tag *tag)
{
	int i;

	i = tag->u.ethernet.mac_index;
	if (i < ARRAY_SIZE(hw_addr))
		memcpy(hw_addr[i].addr, tag->u.ethernet.hw_address,
		       sizeof(hw_addr[i].addr));

	return 0;
}
__tagtable(ATAG_ETHERNET, parse_tag_ethernet);

static void __init set_hw_addr(struct platform_device *pdev)
{
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	const u8 *addr;
	void __iomem *regs;
	struct clk *pclk;

	if (!res)
		return;
	if (pdev->id >= ARRAY_SIZE(hw_addr))
		return;

	addr = hw_addr[pdev->id].addr;
	if (!is_valid_ether_addr(addr))
		return;

	/*
	 * Since this is board-specific code, we'll cheat and use the
	 * physical address directly as we happen to know that it's
	 * the same as the virtual address.
	 */
	regs = (void __iomem __force *)res->start;
	pclk = clk_get(&pdev->dev, "pclk");
	if (IS_ERR(pclk))
		return;

	clk_enable(pclk);
	__raw_writel((addr[3] << 24) | (addr[2] << 16)
		     | (addr[1] << 8) | addr[0], regs + 0x98);
	__raw_writel((addr[5] << 8) | addr[4], regs + 0x9c);
	clk_disable(pclk);
	clk_put(pclk);
}

void __init setup_board(void)
{
	at32_map_usart(1, 0, 0);	/* USART 1: /dev/ttyS0, DB9 */
	at32_setup_serial_console(0);

//	at32_map_usart(0, 1, 0);	/* USART 0: /dev/ttyS1, J5, IrDA */
//	at32_map_usart(2, 2, 0);	/* USART 2: /dev/ttyS2, J6, IrDA */
}

static const struct gpio_led ngw_leds[] = {
	{ .name = "sys", .gpio = GPIO_PIN_PA(16), .active_low = 1,
		.default_trigger = "heartbeat",
	},
	{ .name = "a", .gpio = GPIO_PIN_PA(19), .active_low = 1, },
	{ .name = "b", .gpio = GPIO_PIN_PE(19), .active_low = 1, },
};

static const struct gpio_led_platform_data ngw_led_data = {
	.num_leds =	ARRAY_SIZE(ngw_leds),
	.leds =		(void *) ngw_leds,
};

static struct platform_device ngw_gpio_leds = {
	.name =		"leds-gpio",
	.id =		-1,
	.dev = {
		.platform_data = (void *) &ngw_led_data,
	}
};

static struct i2c_gpio_platform_data i2c_gpio_data = {
	.sda_pin		= GPIO_PIN_PA(6),
	.scl_pin		= GPIO_PIN_PA(7),
	.sda_is_open_drain	= 1,
	.scl_is_open_drain	= 1,
	.udelay			= 2,	/* close to 100 kHz */
};

static struct platform_device i2c_gpio_device = {
	.name		= "i2c-gpio",
	.id		= 0,
	.dev		= {
		.platform_data	= &i2c_gpio_data,
	},
};

static struct i2c_board_info __initdata i2c_info[] = {
	/* NOTE:  original ATtiny24 firmware is at address 0x0b */
};

static struct atmel_abdac_pdata __initdata abdac0_data = {
};

static int __init set_abdac_rate(struct platform_device *pdev)
{
	int retval;
	struct clk *osc1;
	struct clk *pll1;
	struct clk *abdac;

	if (pdev == NULL)
		return -ENXIO;

	osc1 = clk_get(NULL, "osc1");
	if (IS_ERR(osc1)) {
		retval = PTR_ERR(osc1);
		goto out;
	}

	pll1 = clk_get(NULL, "pll1");
	if (IS_ERR(pll1)) {
		retval = PTR_ERR(pll1);
		goto out_osc1;
	}

	abdac = clk_get(&pdev->dev, "sample_clk");
	if (IS_ERR(abdac)) {
		retval = PTR_ERR(abdac);
		goto out_pll1;
	}

	retval = clk_set_parent(pll1, osc1);
	if (retval != 0)
		goto out_abdac;

	/*
	 * Rate is 32000 to 50000 and ABDAC oversamples 256x. Multiply, in
	 * power of 2, to a value above 80 MHz. Power of 2 so it is possible
	 * for the generic clock to divide it down again and 80 MHz is the
	 * lowest frequency for the PLL.
	 */
	retval = clk_round_rate(pll1,
			44100 * 256 * 16);
	if (retval <= 0) {
		retval = -EINVAL;
		goto out_abdac;
	}

	retval = clk_set_rate(pll1, retval);
	if (retval != 0)
		goto out_abdac;

	retval = clk_set_parent(abdac, pll1);
	if (retval != 0)
		goto out_abdac;

out_abdac:
	clk_put(abdac);
out_pll1:
	clk_put(pll1);
out_osc1:
	clk_put(osc1);
out:
	return retval;
}

static int __init atngw100_init(void)
{
	unsigned	i;

	/*
	 * ATNGW100 mkII uses 32-bit SDRAM interface. Reserve the
	 * SDRAM-specific pins so that nobody messes with them.
	 */
#ifdef CONFIG_BOARD_ATNGW100_MKII
	at32_reserve_pin(GPIO_PIOE_BASE, ATMEL_EBI_PE_DATA_ALL);

	smc_set_timing(&nand_config, &nand_timing);
	smc_set_configuration(3, &nand_config);
	pr_info("Adding NAND flash\n");
	at32_add_device_nand(0, &atngw100mkii_nand_data);
#endif

	at32_add_device_usart(0); /* Line 0 (ttyS0), USART 1 */
//	at32_add_device_usart(1); /* Line 1 (ttyS1), USART 0 */
//	at32_add_device_usart(2); /* Line 2 (ttyS2), USART 2 */

	set_hw_addr(at32_add_device_eth(0, &eth_data[0]));
#ifndef CONFIG_BOARD_ATNGW100_MKII_LCD
	set_hw_addr(at32_add_device_eth(1, &eth_data[1]));
#endif

	at32_add_device_spi(0, spi0_board_info, ARRAY_SIZE(spi0_board_info));
/*
	mmc_spi_pdata.ocr_mask = mmc_vddrange_to_ocrmask(3300, 3400);
	at32_add_device_spi(1, spi1_board_info, ARRAY_SIZE(spi1_board_info));
*/
	at32_add_device_mci(0, &mci0_data);
	at32_add_device_usba(0, &atngw100_usba_data);

	for (i = 0; i < ARRAY_SIZE(ngw_leds); i++) {
		at32_select_gpio(ngw_leds[i].gpio,
				AT32_GPIOF_OUTPUT | AT32_GPIOF_HIGH);
	}
	platform_device_register(&ngw_gpio_leds);

	/* all these i2c/smbus pins should have external pullups for
	 * open-drain sharing among all I2C devices.  SDA and SCL do;
	 * PB28/EXTINT3 (ATNGW100) and PE21 (ATNGW100 mkII) doesn't; it should
	 * be SMBALERT# (for PMBus), but it's not available off-board.
	 */
#ifdef CONFIG_BOARD_ATNGW100_MKII
	at32_select_periph(GPIO_PIOE_BASE, 1 << 21, 0, AT32_GPIOF_PULLUP);
#else
	at32_select_periph(GPIO_PIOB_BASE, 1 << 28, 0, AT32_GPIOF_PULLUP);
#endif
	at32_select_gpio(i2c_gpio_data.sda_pin,
		AT32_GPIOF_MULTIDRV | AT32_GPIOF_OUTPUT | AT32_GPIOF_HIGH);
	at32_select_gpio(i2c_gpio_data.scl_pin,
		AT32_GPIOF_MULTIDRV | AT32_GPIOF_OUTPUT | AT32_GPIOF_HIGH);
	platform_device_register(&i2c_gpio_device);
	i2c_register_board_info(0, i2c_info, ARRAY_SIZE(i2c_info));

	set_abdac_rate(at32_add_device_abdac(0, &abdac0_data));
	at32_add_device_lcdc(0, &atngw_lcdc_data,
			     fbmem_start, fbmem_size,
			     LCDC_IOMUX);
/*
	// Use LCDC without IOs for now
	at32_add_device_lcdc(0, &atngw_lcdc_data,
			     fbmem_start, fbmem_size,
			     ATMEL_LCDC(PD, DATA14));
*/
	return 0;
}
postcore_initcall(atngw100_init);

static int __init atngw100_arch_init(void)
{
	/* PB30 (ATNGW100) and PE30 (ATNGW100 mkII) is the otherwise unused
	 * jumper on the mainboard, with an external pullup; the jumper grounds
	 * it. Use it however you like, including letting U-Boot or Linux tweak
	 * boot sequences.
	 */
#ifdef CONFIG_BOARD_ATNGW100_MKII
	at32_select_gpio(GPIO_PIN_PE(30), 0);
	gpio_request(GPIO_PIN_PE(30), "j15");
	gpio_direction_input(GPIO_PIN_PE(30));
	gpio_export(GPIO_PIN_PE(30), false);
#else
	at32_select_gpio(GPIO_PIN_PB(30), 0);
	gpio_request(GPIO_PIN_PB(30), "j15");
	gpio_direction_input(GPIO_PIN_PB(30));
	gpio_export(GPIO_PIN_PB(30), false);
#endif

	/* set_irq_type() after the arch_initcall for EIC has run, and
	 * before the I2C subsystem could try using this IRQ.
	 */
	return irq_set_irq_type(AT32_EXTINT(3), IRQ_TYPE_EDGE_FALLING);
}
arch_initcall(atngw100_arch_init);
