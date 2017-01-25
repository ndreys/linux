/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/irqchip.h>
#include <linux/syscore_ops.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/regulator/consumer.h>
#include <dt-bindings/power/imx7-power.h>

#define IMR_NUM			4
#define GPC_MAX_IRQS            (IMR_NUM * 32)

#define GPC_IMR1_CORE0		0x30
#define GPC_IMR1_CORE1		0x40

#define GPC_PGC_CPU_MAPPING	0xec
#define USB_HSIC_PHY_A7_DOMAIN	BIT(6)
#define USB_OTG2_PHY_A7_DOMAIN	BIT(5)
#define USB_OTG1_PHY_A7_DOMAIN	BIT(4)
#define PCIE_PHY_A7_DOMAIN	BIT(3)
#define MIPI_PHY_A7_DOMAIN	BIT(2)

#define GPC_PU_PGC_SW_PUP_REQ	0xf8
#define GPC_PU_PGC_SW_PDN_REQ	0x104
#define USB_HSIC_PHY_SW_Pxx_REQ	BIT(4)
#define USB_OTG2_PHY_SW_Pxx_REQ	BIT(3)
#define USB_OTG1_PHY_SW_Pxx_REQ	BIT(2)
#define PCIE_PHY_SW_Pxx_REQ	BIT(1)
#define MIPI_PHY_SW_Pxx_REQ	BIT(0)

struct gpcv2_irqchip_data {
	struct raw_spinlock	rlock;
	void __iomem		*gpc_base;
	u32			wakeup_sources[IMR_NUM];
	u32			saved_irq_mask[IMR_NUM];
	u32			cpu2wakeup;
};

struct gpcv2_domain {
	struct generic_pm_domain genpd;
	struct regulator *regulator;

	const struct {
		u32 pxx;
		u32 map;
	} bits;

	struct device *dev;
};

static struct gpcv2_irqchip_data *imx_gpcv2_instance;

/*
 * Interface for the low level wakeup code.
 */
u32 imx_gpcv2_get_wakeup_source(u32 **sources)
{
	if (!imx_gpcv2_instance)
		return 0;

	if (sources)
		*sources = imx_gpcv2_instance->wakeup_sources;

	return IMR_NUM;
}

static int gpcv2_wakeup_source_save(void)
{
	struct gpcv2_irqchip_data *cd;
	void __iomem *reg;
	int i;

	cd = imx_gpcv2_instance;
	if (!cd)
		return 0;

	for (i = 0; i < IMR_NUM; i++) {
		reg = cd->gpc_base + cd->cpu2wakeup + i * 4;
		cd->saved_irq_mask[i] = readl_relaxed(reg);
		writel_relaxed(cd->wakeup_sources[i], reg);
	}

	return 0;
}

static void gpcv2_wakeup_source_restore(void)
{
	struct gpcv2_irqchip_data *cd;
	void __iomem *reg;
	int i;

	cd = imx_gpcv2_instance;
	if (!cd)
		return;

	for (i = 0; i < IMR_NUM; i++) {
		reg = cd->gpc_base + cd->cpu2wakeup + i * 4;
		writel_relaxed(cd->saved_irq_mask[i], reg);
	}
}

static struct syscore_ops imx_gpcv2_syscore_ops = {
	.suspend	= gpcv2_wakeup_source_save,
	.resume		= gpcv2_wakeup_source_restore,
};

static int imx_gpcv2_irq_set_wake(struct irq_data *d, unsigned int on)
{
	struct gpcv2_irqchip_data *cd = d->chip_data;
	unsigned int idx = d->hwirq / 32;
	unsigned long flags;
	void __iomem *reg;
	u32 mask, val;

	raw_spin_lock_irqsave(&cd->rlock, flags);
	reg = cd->gpc_base + cd->cpu2wakeup + idx * 4;
	mask = 1 << d->hwirq % 32;
	val = cd->wakeup_sources[idx];

	cd->wakeup_sources[idx] = on ? (val & ~mask) : (val | mask);
	raw_spin_unlock_irqrestore(&cd->rlock, flags);

	/*
	 * Do *not* call into the parent, as the GIC doesn't have any
	 * wake-up facility...
	 */

	return 0;
}

static void imx_gpcv2_irq_unmask(struct irq_data *d)
{
	struct gpcv2_irqchip_data *cd = d->chip_data;
	void __iomem *reg;
	u32 val;

	raw_spin_lock(&cd->rlock);
	reg = cd->gpc_base + cd->cpu2wakeup + d->hwirq / 32 * 4;
	val = readl_relaxed(reg);
	val &= ~(1 << d->hwirq % 32);
	writel_relaxed(val, reg);
	raw_spin_unlock(&cd->rlock);

	irq_chip_unmask_parent(d);
}

static void imx_gpcv2_irq_mask(struct irq_data *d)
{
	struct gpcv2_irqchip_data *cd = d->chip_data;
	void __iomem *reg;
	u32 val;

	raw_spin_lock(&cd->rlock);
	reg = cd->gpc_base + cd->cpu2wakeup + d->hwirq / 32 * 4;
	val = readl_relaxed(reg);
	val |= 1 << (d->hwirq % 32);
	writel_relaxed(val, reg);
	raw_spin_unlock(&cd->rlock);

	irq_chip_mask_parent(d);
}

static struct irq_chip gpcv2_irqchip_data_chip = {
	.name			= "GPCv2",
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_mask		= imx_gpcv2_irq_mask,
	.irq_unmask		= imx_gpcv2_irq_unmask,
	.irq_set_wake		= imx_gpcv2_irq_set_wake,
	.irq_retrigger		= irq_chip_retrigger_hierarchy,
#ifdef CONFIG_SMP
	.irq_set_affinity	= irq_chip_set_affinity_parent,
#endif
};

static int imx_gpcv2_domain_translate(struct irq_domain *d,
				      struct irq_fwspec *fwspec,
				      unsigned long *hwirq,
				      unsigned int *type)
{
	if (is_of_node(fwspec->fwnode)) {
		if (fwspec->param_count != 3)
			return -EINVAL;

		/* No PPI should point to this domain */
		if (fwspec->param[0] != 0)
			return -EINVAL;

		*hwirq = fwspec->param[1];
		*type = fwspec->param[2];
		return 0;
	}

	return -EINVAL;
}

static int imx_gpcv2_domain_alloc(struct irq_domain *domain,
				  unsigned int irq, unsigned int nr_irqs,
				  void *data)
{
	struct irq_fwspec *fwspec = data;
	struct irq_fwspec parent_fwspec;
	irq_hw_number_t hwirq;
	unsigned int type;
	int err;
	int i;

	err = imx_gpcv2_domain_translate(domain, fwspec, &hwirq, &type);
	if (err)
		return err;

	if (hwirq >= GPC_MAX_IRQS)
		return -EINVAL;

	for (i = 0; i < nr_irqs; i++) {
		irq_domain_set_hwirq_and_chip(domain, irq + i, hwirq + i,
				&gpcv2_irqchip_data_chip, domain->host_data);
	}

	parent_fwspec = *fwspec;
	parent_fwspec.fwnode = domain->parent->fwnode;
	return irq_domain_alloc_irqs_parent(domain, irq, nr_irqs,
					    &parent_fwspec);
}

static struct irq_domain_ops gpcv2_irqchip_data_domain_ops = {
	.translate	= imx_gpcv2_domain_translate,
	.alloc		= imx_gpcv2_domain_alloc,
	.free		= irq_domain_free_irqs_common,
};

static int __init imx_gpcv2_irqchip_init(struct device_node *node,
			       struct device_node *parent)
{
	struct irq_domain *parent_domain, *domain;
	struct gpcv2_irqchip_data *cd;
	int i;

	if (!parent) {
		pr_err("%s: no parent, giving up\n", node->full_name);
		return -ENODEV;
	}

	parent_domain = irq_find_host(parent);
	if (!parent_domain) {
		pr_err("%s: unable to get parent domain\n", node->full_name);
		return -ENXIO;
	}

	cd = kzalloc(sizeof(struct gpcv2_irqchip_data), GFP_KERNEL);
	if (!cd) {
		pr_err("kzalloc failed!\n");
		return -ENOMEM;
	}

	cd->gpc_base = of_iomap(node, 0);
	if (!cd->gpc_base) {
		pr_err("fsl-gpcv2: unable to map gpc registers\n");
		kfree(cd);
		return -ENOMEM;
	}

	domain = irq_domain_add_hierarchy(parent_domain, 0, GPC_MAX_IRQS,
				node, &gpcv2_irqchip_data_domain_ops, cd);
	if (!domain) {
		iounmap(cd->gpc_base);
		kfree(cd);
		return -ENOMEM;
	}
	irq_set_default_host(domain);

	/* Initially mask all interrupts */
	for (i = 0; i < IMR_NUM; i++) {
		writel_relaxed(~0, cd->gpc_base + GPC_IMR1_CORE0 + i * 4);
		writel_relaxed(~0, cd->gpc_base + GPC_IMR1_CORE1 + i * 4);
		cd->wakeup_sources[i] = ~0;
	}

	/* Let CORE0 as the default CPU to wake up by GPC */
	cd->cpu2wakeup = GPC_IMR1_CORE0;

	/*
	 * Due to hardware design failure, need to make sure GPR
	 * interrupt(#32) is unmasked during RUN mode to avoid entering
	 * DSM by mistake.
	 */
	writel_relaxed(~0x1, cd->gpc_base + cd->cpu2wakeup);

	imx_gpcv2_instance = cd;
	register_syscore_ops(&imx_gpcv2_syscore_ops);

	return 0;
}
IRQCHIP_DECLARE_DRIVER(imx_gpcv2, "fsl,imx7d-gpc", imx_gpcv2_irqchip_init);

static int imx7_gpc_pu_pgc_sw_pxx_req(struct generic_pm_domain *genpd,
				      bool on)
{
	int ret = 0;
	u32 mapping;
	unsigned long deadline;
	struct gpcv2_domain *pd = container_of(genpd,
					       struct gpcv2_domain, genpd);
	void __iomem *base  = imx_gpcv2_instance->gpc_base;
	unsigned int offset = (on) ?
		GPC_PU_PGC_SW_PUP_REQ : GPC_PU_PGC_SW_PDN_REQ;

	if (!base)
		return -ENODEV;

	mapping = readl_relaxed(base + GPC_PGC_CPU_MAPPING);
	writel_relaxed(mapping | pd->bits.map, base + GPC_PGC_CPU_MAPPING);

	if (on) {
		ret = regulator_enable(pd->regulator);
		if (ret) {
			dev_err(pd->dev,
				"failed to enable regulator: %d\n", ret);
			goto unmap;
		}
	}

	writel_relaxed(readl_relaxed(base + offset) | pd->bits.pxx,
		       base + offset);

	/*
	 * As per "5.5.9.4 Example Code 4" in IMX7DRM.pdf wait
	 * for PUP_REQ/PDN_REQ bit to be cleared
	 */
	deadline = jiffies + msecs_to_jiffies(1);
	while (true) {
		if (readl_relaxed(base + offset) & pd->bits.pxx)
			break;
		if (time_after(jiffies, deadline)) {
			dev_err(pd->dev, "falied to command PGC\n");
			ret = -ETIMEDOUT;
			/*
			 * If we were in a process of enabling a
			 * domain and failed we might as well disable
			 * the regulator we just enabled. And if it
			 * was the opposite situation and we failed to
			 * power down -- keep the regulator on
			 */
			on  = !on;
			break;
		}
		cpu_relax();
	}

	if (!on) {
		int err;

		err = regulator_disable(pd->regulator);
		if (err)
			dev_err(pd->dev,
				"failed to disable regulator: %d\n", ret);
		/*
		 * Preserve earlier error code
		 */
		ret = ret ?: err;
	}
unmap:
	writel_relaxed(mapping, base + GPC_PGC_CPU_MAPPING);
	return ret;
}

static int imx7_gpc_pu_pgc_sw_pup_req(struct generic_pm_domain *genpd)
{
	return imx7_gpc_pu_pgc_sw_pxx_req(genpd, true);
}

static int imx7_gpc_pu_pgc_sw_pdn_req(struct generic_pm_domain *genpd)
{
	return imx7_gpc_pu_pgc_sw_pxx_req(genpd, false);
}

static struct gpcv2_domain imx7_usb_hsic_phy = {
	.genpd = {
		.name      = "usb-hsic-phy",
		.power_on  = imx7_gpc_pu_pgc_sw_pup_req,
		.power_off = imx7_gpc_pu_pgc_sw_pdn_req,
	},
	.bits  = {
		.pxx = USB_HSIC_PHY_SW_Pxx_REQ,
		.map = USB_HSIC_PHY_A7_DOMAIN,
	},
};

static struct gpcv2_domain imx7_usb_otg2_phy = {
	.genpd = {
		.name      = "usb-otg2-phy",
		.power_on  = imx7_gpc_pu_pgc_sw_pup_req,
		.power_off = imx7_gpc_pu_pgc_sw_pdn_req,
	},
	.bits  = {
		.pxx = USB_OTG2_PHY_SW_Pxx_REQ,
		.map = USB_OTG2_PHY_A7_DOMAIN,
	},
};

static struct gpcv2_domain imx7_usb_otg1_phy = {
	.genpd = {
		.name      = "usb-otg1-phy",
		.power_on  = imx7_gpc_pu_pgc_sw_pup_req,
		.power_off = imx7_gpc_pu_pgc_sw_pdn_req,
	},
	.bits  = {
		.pxx = USB_OTG1_PHY_SW_Pxx_REQ,
		.map = USB_OTG1_PHY_A7_DOMAIN,
	},
};

static struct gpcv2_domain imx7_pcie_phy = {
	.genpd = {
		.name      = "pcie-phy",
		.power_on  = imx7_gpc_pu_pgc_sw_pup_req,
		.power_off = imx7_gpc_pu_pgc_sw_pdn_req,
	},
	.bits  = {
		.pxx = PCIE_PHY_SW_Pxx_REQ,
		.map = PCIE_PHY_A7_DOMAIN,
	},
};

static struct gpcv2_domain imx7_mipi_phy = {
	.genpd = {
		.name      = "mipi-phy",
		.power_on  = imx7_gpc_pu_pgc_sw_pup_req,
		.power_off = imx7_gpc_pu_pgc_sw_pdn_req,
	},
	.bits  = {
		.pxx = MIPI_PHY_SW_Pxx_REQ,
		.map = MIPI_PHY_A7_DOMAIN,
	},
};

static struct generic_pm_domain *imx_gpcv2_domains[] = {
	[IMX7_POWER_DOMAIN_USB_HSIC_PHY] = &imx7_usb_hsic_phy.genpd,
	[IMX7_POWER_DOMAIN_USB_OTG2_PHY] = &imx7_usb_otg2_phy.genpd,
	[IMX7_POWER_DOMAIN_USB_OTG1_PHY] = &imx7_usb_otg1_phy.genpd,
	[IMX7_POWER_DOMAIN_PCIE_PHY]     = &imx7_pcie_phy.genpd,
	[IMX7_POWER_DOMAIN_MIPI_PHY]     = &imx7_mipi_phy.genpd,
};

static struct genpd_onecell_data imx_gpcv2_onecell_data = {
	.domains = imx_gpcv2_domains,
	.num_domains = ARRAY_SIZE(imx_gpcv2_domains),
};

static int imx_gpcv2_probe(struct platform_device *pdev)
{
	int i, ret;
	struct device *dev = &pdev->dev;

	for (i = 0; i < ARRAY_SIZE(imx_gpcv2_domains); i++) {
		int voltage = 0;
		const char *id = "dummy";
		struct generic_pm_domain *genpd = imx_gpcv2_domains[i];
		struct gpcv2_domain *pd = container_of(genpd,
						       struct gpcv2_domain,
						       genpd);

		ret = pm_genpd_init(genpd, NULL, true);
		if (ret) {
			dev_err(dev, "Failed to init power domain #%d\n", i);
			goto undo_pm_genpd_init;
		}

		switch (i) {
		case IMX7_POWER_DOMAIN_PCIE_PHY:
			id = "pcie-phy";
			voltage = 1000000;
			break;
		case IMX7_POWER_DOMAIN_MIPI_PHY:
			id = "mipi-phy";
			voltage = 1000000;
			break;
		case IMX7_POWER_DOMAIN_USB_HSIC_PHY:
			id = "usb-hsic-phy";
			voltage = 1200000;
			break;
		}

		pd->regulator = devm_regulator_get(dev, id);
		if (voltage)
			regulator_set_voltage(pd->regulator,
					      voltage, voltage);

		pd->dev = dev;
	}

	ret = of_genpd_add_provider_onecell(dev->of_node,
					    &imx_gpcv2_onecell_data);
	if (ret) {
		dev_err(dev, "Failed to add genpd provider\n");
		goto undo_pm_genpd_init;
	}

	return 0;

undo_pm_genpd_init:
	for (--i; i >= 0; i--)
		pm_genpd_remove(imx_gpcv2_domains[i]);

	return ret;
}

static const struct of_device_id imx_gpcv2_dt_ids[] = {
	{ .compatible = "fsl,imx7d-gpc" },
	{ }
};

static struct platform_driver imx_gpcv2_driver = {
	.driver = {
		.name = "imx-gpcv2",
		.of_match_table = imx_gpcv2_dt_ids,
	},
	.probe = imx_gpcv2_probe,
};

static int __init imx_pgcv2_init(void)
{
	return platform_driver_register(&imx_gpcv2_driver);
}
subsys_initcall(imx_pgcv2_init);
