// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * XTM330 board setup
 *
 * Copyright 2009,2012-2013 Freescale Semiconductor Inc.
 * Copyright 2024 Tobias Schramm <t.schramm@manjaro.org>
 */

#include <linux/stddef.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/kdev_t.h>
#include <linux/delay.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/fsl/guts.h>

#include <asm/time.h>
#include <asm/machdep.h>
#include <asm/pci-bridge.h>
#include <asm-generic/early_ioremap.h>
#include <mm/mmu_decl.h>
#include <asm/udbg.h>
#include <asm/mpic.h>
#include <soc/fsl/qe/qe.h>

#include <sysdev/fsl_soc.h>
#include <sysdev/fsl_pci.h>
#include "smp.h"

#include "mpc85xx.h"

static void __init xtm330_pic_init(void)
{
	struct mpic *mpic;
	int flags = MPIC_BIG_ENDIAN | MPIC_SINGLE_DEST_CPU;

	mpic = mpic_alloc(NULL, 0, flags, 0, 256, " OpenPIC  ");

	if (WARN_ON(!mpic))
		return;

	mpic_init(mpic);
}

#define IOREBAR	0xffe00000
#define GPIO_BASE (IOREBAR + 0xf000)
#define MMIO32(x_) (*(volatile u32*)(x_))
#define GPIO_DDR(base_) MMIO32((unsigned long)(base_) + 0x00)
#define GPIO_DATA(base_) MMIO32((unsigned long)(base_) + 0x08)
#define GPIO_BIT(gpio_) (1UL << (31 - (gpio_)))

/*
 * Setup the architecture
 */
static void __init xtm330_setup_arch(void)
{
	if (ppc_md.progress)
		ppc_md.progress("xtm330_setup_arch()", 0);

	mpc85xx_smp_init();

	fsl_pci_assign_primary();

	void __iomem *gpiobase = early_ioremap(GPIO_BASE, 0x100);

	if (gpiobase) {
		u32 gpio_data = GPIO_DATA(gpiobase);
		u32 gpio_ddr = GPIO_DDR(gpiobase);
		gpio_data &= gpio_ddr;

		/*
		 * Switch 0 and switch 1 share a reset.
		 * Can't use driver-level gpio hardware reset.
		 * Reset switches once here.
		 */
		gpio_data &= ~GPIO_BIT(13);
		pr_info("Putting switches into reset (0x%08x)...\n", gpio_data);
		GPIO_DATA(gpiobase) = gpio_data;
		mdelay(10);

		gpio_data |= GPIO_BIT(13);
		pr_info("Taking switches out of reset (0x%08x)...\n", gpio_data);
		GPIO_DATA(gpiobase) = gpio_data;
		mdelay(100);

		early_iounmap(gpiobase, 0x100);
	} else {
		pr_err("Failed to remap GPIO memory for platform reset\n");
	}

	mpc85xx_qe_par_io_init();

	pr_info("WatchGuard XTM330\n");
}

machine_arch_initcall(xtm330, mpc85xx_common_publish_devices);

define_machine(xtm330) {
	.name			= "WatchGuard XTM330",
	.compatible		= "watchguard,xtm330",
	.setup_arch		= xtm330_setup_arch,
	.init_IRQ		= xtm330_pic_init,
#ifdef CONFIG_PCI
	.pcibios_fixup_bus	= fsl_pcibios_fixup_bus,
	.pcibios_fixup_phb      = fsl_pcibios_fixup_phb,
#endif
	.get_irq		= mpic_get_irq,
	.progress		= udbg_progress,
};
