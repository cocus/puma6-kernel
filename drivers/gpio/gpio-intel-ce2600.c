/*
 *  GPIO interface for Intel SOC CE2600.
 *
 *  Copyright (c) 2010, 2012 Intel Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License 2 as published
 *  by the Free Software Foundation.
 *
 */

#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/platform_device.h>

#ifdef CONFIG_HW_MUTEXES
#include <linux/hw_mutex.h>
#endif



#define INTELCE_GPIO_DRV_NAME		"intelce_gpio"
#define PCI_INTELCE_GPIO_DEVICE_ID	0x2e67

#define INTELCE_GPIO_BAR        0

#define INTELCE_GPIO_IRQ_BASE 128 /* Search free IRQS start from this number*/

struct intelce_gpio_chip{
	int irq_base;
	void __iomem *reg_base;
	void __iomem *high_base;
	void __iomem *mux_ctl_base;
	struct gpio_chip chip;
	struct spinlock lock;
};

static inline uint32_t intelce_gpio_mmio_read32(const volatile void  __iomem *addr)
{
	return readl(addr);
}

static inline void  intelce_gpio_mmio_write32(uint32_t value, volatile void __iomem *addr)
{
	writel(value, addr);
}

static inline uint32_t intelce_gpio_port_read32(int port)
{
	return inl(port);
}

static inline void  intelce_gpio_port_write32(uint32_t value, int port)
{
	outl(value, port);
}

static inline struct intelce_gpio_chip *to_intelce_gpio_chip(struct gpio_chip *c)
{
     return container_of(c, struct intelce_gpio_chip, chip);;
}

/*
 *  GPIO interface for Intel SOC CE2600.
 *
 *  Copyright (c) 2010, 2012 Intel Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License 2 as published
 *  by the Free Software Foundation.
 *
 */


#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>

#define CE2600_PUB_GPIOS_PER_BANK        32
#define CE2600_PUB_GPIO_BANKS            4

#define CE2600_PUB_GPIO_BANK0_BASE		0x0
#define CE2600_PUB_GPIO_BANK1_BASE		0x20
#define CE2600_PUB_GPIO_BANK2_BASE		0x40
#define CE2600_PUB_GPIO_BANK3_BASE		0x60

#define CE2600_PUB_GPIO_OUT				0x00
#define CE2600_PUB_GPIO_OUT_EN			0x04
#define CE2600_PUB_GPIO_INPUT			0x08
#define CE2600_PUB_GPIO_INT_STAT		0x0c

#define CE2600_PUB_GPIO_INT_EN			0x10
#define CE2600_PUB_GPIO_INT_MODE_LE		0x14
#define CE2600_PUB_GPIO_INT_MODE_RF		0x18

#define CE2600_PUB_GPIO_MUX_CTL			0x1c

#define CE2600_PUB_GPIO_BANK0_HIGH_BASE		0x80
#define CE2600_PUB_GPIO_BANK1_HIGH_BASE		0x90
#define CE2600_PUB_GPIO_BANK2_HIGH_BASE		0xA0
#define CE2600_PUB_GPIO_BANK3_HIGH_BASE		0xB0

#define CE2600_PUB_GPIO_CLEAR       		0x00
#define CE2600_PUB_GPIO_SET     		  	0x04


#define CE2600_PUB_GPIO_POLARITY0	0x88
#define CE2600_PUB_GPIO_POLARITY1	0x98
#define CE2600_PUB_GPIO_POLARITY2	0xa8
#define CE2600_PUB_GPIO_POLARITY3	0xb8

#define CE2600_PUB_GPIO_INT_ROUTER  0xbc

/* Core Well GPIO Group in North brige */
/* Core Well GPIO[7:0] */
#define CE2600_CORE_WELL_GPIO_CGEN		0x00
#define CE2600_CORE_WELL_GPIO_CGIO		0x04
#define CE2600_CORE_WELL_GPIO_CGLV		0x08
#define CE2600_CORE_WELL_GPIO_CGTPE		0x0C
#define CE2600_CORE_WELL_GPIO_CGTNE		0x10
#define CE2600_CORE_WELL_GPIO_CGGPE		0x14
#define CE2600_CORE_WELL_GPIO_CGSMI		0x18
#define CE2600_CORE_WELL_GPIO_CGTS		0x1C


void ce2600_gpio_set(struct gpio_chip *chip, unsigned offset, int value);
int ce2600_gpio_direction_output(struct gpio_chip *chip, unsigned offset, int value);
int ce2600_gpio_direction_input(struct gpio_chip *chip, unsigned offset);
int ce2600_set_multi_function(struct gpio_chip *chip, unsigned offset,  int fn_num);
int ce2600_get_multi_function(struct gpio_chip *chip, unsigned offset);
int ce2600_gpio_irq_setup(struct intelce_gpio_chip *c, struct pci_dev *pdev);

int ce2600_gpio_suspend(void *io_mem, unsigned short io_port);
int ce2600_gpio_resume(void *io_mem, unsigned short io_port);


int ce2600_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	void *reg_base = to_intelce_gpio_chip(chip)->reg_base;

	return (intelce_gpio_mmio_read32(reg_base + CE2600_PUB_GPIO_INPUT) >> offset) & 0x1;
}
void ce2600_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct intelce_gpio_chip *c = to_intelce_gpio_chip(chip);
	void *reg_base = c->high_base;

	if (value ) {
		intelce_gpio_mmio_write32(1 << offset, reg_base + CE2600_PUB_GPIO_SET);
	} else {
		intelce_gpio_mmio_write32(1 << offset, reg_base + CE2600_PUB_GPIO_CLEAR);
	}
}

int ce2600_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct intelce_gpio_chip *c = to_intelce_gpio_chip(chip);
	void *reg_base = c->reg_base;
	uint32_t orig;
	unsigned long flags;

#ifdef CONFIG_HW_MUTEXES
	hw_mutex_lock(HW_MUTEX_GPIO);
#endif
	spin_lock_irqsave(&c->lock, flags);
	orig = intelce_gpio_mmio_read32(reg_base + CE2600_PUB_GPIO_OUT_EN);
	orig &= ~(1 << offset);
	intelce_gpio_mmio_write32(orig, reg_base + CE2600_PUB_GPIO_OUT_EN);
	spin_unlock_irqrestore(&c->lock, flags);
#ifdef CONFIG_HW_MUTEXES
	hw_mutex_unlock(HW_MUTEX_GPIO);
#endif
	return 0;
}

int ce2600_gpio_direction_output(struct gpio_chip *chip, unsigned offset, int value)
{
	struct intelce_gpio_chip *c = to_intelce_gpio_chip(chip);
	void *reg_base = c->reg_base;
	uint32_t orig;
	unsigned long flags;

#ifdef CONFIG_HW_MUTEXES
	hw_mutex_lock(HW_MUTEX_GPIO);
#endif
	spin_lock_irqsave(&c->lock, flags);
        orig = intelce_gpio_mmio_read32(reg_base + CE2600_PUB_GPIO_OUT);
        if (value) {
                        orig |= (1 << offset);
                } else {
                        orig &= ~(1 << offset);
        }
        intelce_gpio_mmio_write32(orig, reg_base + CE2600_PUB_GPIO_OUT);

	orig = intelce_gpio_mmio_read32(reg_base + CE2600_PUB_GPIO_OUT_EN);
	orig |= (1 << offset);
	intelce_gpio_mmio_write32(orig, reg_base + CE2600_PUB_GPIO_OUT_EN);

	spin_unlock_irqrestore(&c->lock, flags);
#ifdef CONFIG_HW_MUTEXES
	hw_mutex_unlock(HW_MUTEX_GPIO);
#endif
	return 0;
}

int ce2600_set_multi_function(struct gpio_chip *chip, unsigned offset,  int fn_num)
{
	struct intelce_gpio_chip *c = to_intelce_gpio_chip(chip);
	void *reg_base = c->mux_ctl_base;
	uint32_t curr_reg_value = 0;
	uint32_t new_reg_value = 0;
	uint32_t bit_num = 0;
	uint32_t revert = 0;
	uint32_t gpio = chip->base + offset;
	unsigned long flags;

	switch (gpio)
	{
		case 50:
			bit_num = 0;
			break;
		case 116:
		case 117:
		case 118:
		case 38:
		case 39:
		case 40:
		case 41:
			bit_num = 3;
			break;
		case 42:
		case 43:
			bit_num = 4;
			break;
		case 48:
			bit_num = 7;
			break;
		case 55:
			bit_num = 12;
			revert = 1;
			break;
		case 54:
			bit_num = 13;
			revert = 1;
			break;
		case 53:
			bit_num = 14;
			revert = 1;
			break;
		case 52:
			bit_num = 15;
			revert = 1;
			break;
		case 56:
			bit_num = 12;
			break;
		case 57:
			bit_num = 13;
			break;
		case 58:
			bit_num = 14;
			break;
		case 93:
			bit_num = 15;
			break;
		case 51:
			bit_num = 16;
			break;
		default:
			return -EINVAL;
	}
	if (revert)
		fn_num = !fn_num;

#ifdef CONFIG_HW_MUTEXES
	hw_mutex_lock(HW_MUTEX_GPIO);
#endif
	spin_lock_irqsave(&c->lock, flags);
	curr_reg_value = intelce_gpio_mmio_read32(reg_base);
	if (0 == fn_num)
		new_reg_value = curr_reg_value & ~(1 << bit_num);
	else
		new_reg_value = curr_reg_value | (1 << bit_num);
	intelce_gpio_mmio_write32(new_reg_value, reg_base);
	spin_unlock_irqrestore(&c->lock, flags);
#ifdef CONFIG_HW_MUTEXES
	hw_mutex_unlock(HW_MUTEX_GPIO);
#endif
	return 0;
}

int ce2600_get_multi_function(struct gpio_chip *chip, unsigned offset)
{
/*
 *  This function will set the pin associated with the gpio_num to it's alternate function.
 *  bit_num-----function [0] / [1]
 *  0 --- -disable / enable UART1_TXD_GPIO_50
 *  3 ---- GPIO fucntions /Smart Card0  GPIO 116, 117,118, 38, 39, 40, 41
 *  4 ---- gbe link assigned to 1 / GBE use LOS (loss of signal) pin as gbe link GPIO 42, 43
 *  7--- - disable / enable UART0_TXD_GPIO_48
 *  16---- disable  /eanble UART0 RTS GPIO 51
 */
	struct intelce_gpio_chip *c = to_intelce_gpio_chip(chip);
	void *reg_base = c->mux_ctl_base;
	uint32_t gpio = chip->base + offset;
	int bit_num = -1;
	int revert = 0;

	switch (gpio) {
		case 50:
			bit_num = 0;
			break;
		case 38 ... 41:
		case 116 ... 118:
			bit_num = 3;
			break;
		case 42 ... 43:
			bit_num = 4;
			break;
		case 48:
			bit_num = 7;
			break;
		case 51:
			bit_num = 16;
			break;
		case 52:
			bit_num = 15;
			revert = 1;
			break;
		case 53:
			bit_num = 14;
			revert = 1;
			break;
		case 54:
			bit_num = 13;
			revert = 1;
			break;
		case 55:
			bit_num = 12;
			revert = 1;
			break;
		case 56:
			bit_num = 12;
			break;
		case 57:
			bit_num = 13;
			break;
		case 58:
			bit_num = 14;
			break;
		case 93:
			bit_num = 15;
			break;
		default:
			break;
	}
	if (-1 == bit_num) return -EINVAL;

	if (revert)
		return !((intelce_gpio_mmio_read32(reg_base) >> bit_num) & 0x1);
	else
		return (intelce_gpio_mmio_read32(reg_base) >> bit_num) & 0x1;

}

static int ce2600_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct intelce_gpio_chip *c = irq_data_get_irq_chip_data(d);
	void __iomem *mode_le_reg, *mode_rf_reg;
	uint32_t irq_offs = d->irq - c->irq_base;
	uint32_t reg1, reg2;
	int ret = 0;

	if (irq_offs > 32)
		return -EINVAL;

	mode_le_reg = c->reg_base + CE2600_PUB_GPIO_INT_MODE_LE;
	mode_rf_reg = c->reg_base + CE2600_PUB_GPIO_INT_MODE_RF;

#ifdef CONFIG_HW_MUTEXES
	hw_mutex_lock(HW_MUTEX_GPIO);
#endif
	reg1 = intelce_gpio_mmio_read32(mode_le_reg);
	reg2 = intelce_gpio_mmio_read32(mode_rf_reg);

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		reg1 |= BIT(irq_offs);
		reg2 &= ~BIT(irq_offs);
		break;

	case IRQ_TYPE_EDGE_FALLING:
		reg1 |= BIT(irq_offs);
		reg2 |= BIT(irq_offs);
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		reg1 &= ~BIT(irq_offs);
		reg2 &= ~BIT(irq_offs);
		break;

	case IRQ_TYPE_LEVEL_LOW:
		reg1 &= ~BIT(irq_offs);
		reg2 |= BIT(irq_offs);
		break;

	default:
		ret =-EINVAL;
		break;
	}
	intelce_gpio_mmio_write32(reg1, mode_le_reg);
	intelce_gpio_mmio_write32(reg2, mode_rf_reg);
#ifdef CONFIG_HW_MUTEXES
	hw_mutex_unlock(HW_MUTEX_GPIO);
#endif
	return ret;
}

int ce4100_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	return to_intelce_gpio_chip(chip)->irq_base + offset;
}

irqreturn_t ce2600_gpio_irq_handler(int irq, void *data)
{
	struct intelce_gpio_chip *c = data;
	u32 irq_stat = intelce_gpio_mmio_read32(c->reg_base + CE2600_PUB_GPIO_INT_STAT);

	irq_stat &= intelce_gpio_mmio_read32(c->reg_base + CE2600_PUB_GPIO_INT_EN);
	if (!irq_stat)
		return IRQ_NONE;

	while (irq_stat) {
		u32 irq_bit = __fls(irq_stat);

		irq_stat &= ~BIT(irq_bit);
		//printk("interterrupt %d coming\n", c->irq_base + irq_bit);
		generic_handle_irq(c->irq_base + irq_bit);
	}

	return IRQ_HANDLED;
}
void ce2600_gpio_irq_mask(struct irq_data *data)
{
	struct intelce_gpio_chip *c = irq_data_get_irq_chip_data(data);
	void *reg_base = c->reg_base;
	u32 orig;

	orig = intelce_gpio_mmio_read32(reg_base + CE2600_PUB_GPIO_INT_EN);
	orig &= ~(1 << (data->irq - c->irq_base));
	intelce_gpio_mmio_write32(orig, reg_base + CE2600_PUB_GPIO_INT_EN);
}

void ce2600_gpio_irq_unmask(struct irq_data *data)
{
	struct intelce_gpio_chip *c = irq_data_get_irq_chip_data(data);
	void *reg_base = c->reg_base;
	u32 orig;

	orig = intelce_gpio_mmio_read32(reg_base + CE2600_PUB_GPIO_INT_EN);
	orig |= (1 << (data->irq - c->irq_base));
	intelce_gpio_mmio_write32(orig, reg_base + CE2600_PUB_GPIO_INT_EN);
}

void ce2600_gpio_irq_eoi(struct irq_data *data)
{
	struct intelce_gpio_chip *c = irq_data_get_irq_chip_data(data);
	void *reg_base = c->reg_base;

	intelce_gpio_mmio_write32(1 << (data->irq -c->irq_base), reg_base + CE2600_PUB_GPIO_INT_STAT);
}

static struct irq_chip ce2600_irq_chip = {
	.irq_mask = ce2600_gpio_irq_mask,
	.irq_unmask = ce2600_gpio_irq_unmask,
	.irq_eoi = ce2600_gpio_irq_eoi,
	.irq_set_type = ce2600_gpio_irq_set_type,
};

__init int ce2600_gpio_irq_setup(struct intelce_gpio_chip *c, struct pci_dev *pdev)
{
	int i;
	int irq;
	int ret;

	c->irq_base = irq_alloc_descs(-1, INTELCE_GPIO_IRQ_BASE, CE2600_PUB_GPIOS_PER_BANK, -1);
	if (c->irq_base < 0)
		return c->irq_base;

	/* mask + ACK all interrupt sources */
	intelce_gpio_mmio_write32(0, c->reg_base + CE2600_PUB_GPIO_INT_EN);
	intelce_gpio_mmio_write32(0xFFFFFFFF, c->reg_base + CE2600_PUB_GPIO_INT_STAT);

	ret = request_irq(pdev->irq, ce2600_gpio_irq_handler, IRQF_SHARED, "ce2600_gpio", c);
	if (ret)
		goto out_free_desc;

	/*
	 * This gpio irq controller latches level irqs. Testing shows that if
	 * we unmask & ACK the IRQ before the source of the interrupt is gone
	 * then the interrupt is active again.
	 */
	irq = c->irq_base;
	for (i=0; i < c->chip.ngpio; i++) {
		irq_set_chip_and_handler_name(irq, &ce2600_irq_chip, handle_fasteoi_irq, "gpio_irq");
		irq_set_chip_data(irq, c);
		irq++;
	}
	return 0;

out_free_desc:
	irq_free_descs(c->irq_base, CE2600_PUB_GPIOS_PER_BANK);
	return ret;
}

struct gpio_group {
	uint32_t input;
	uint32_t output;
	uint32_t output_enable;
	uint32_t int_status;
	uint32_t int_enable;
	uint32_t mode_le;
	uint32_t mode_rf;
	uint32_t polarity;
};

struct _gpio {
	uint32_t cgen;
	uint32_t cgio;
	uint32_t cglv;
	uint32_t cgtpe;
	uint32_t cgtne;
	uint32_t cggpe;
	uint32_t cgsmi;
	uint32_t cgts;

	struct gpio_group group[4];
	uint32_t mux_ctl;
	uint32_t int_router;
};

static struct _gpio  _gpio;

/*CE2600 gpio suspend routine */
int ce2600_gpio_suspend(void *io_mem, unsigned short io_port)
{
	char *virt_io_mem = (char *)io_mem;
	int i;

    /* Keep status of Core Well GPIO*/
	_gpio.cgen = intelce_gpio_port_read32(io_port + CE2600_CORE_WELL_GPIO_CGEN);
	_gpio.cgio = intelce_gpio_port_read32(io_port + CE2600_CORE_WELL_GPIO_CGIO);
	_gpio.cglv = intelce_gpio_port_read32(io_port + CE2600_CORE_WELL_GPIO_CGLV);
	_gpio.cgtpe = intelce_gpio_port_read32(io_port + CE2600_CORE_WELL_GPIO_CGTPE);
	_gpio.cgtne = intelce_gpio_port_read32(io_port + CE2600_CORE_WELL_GPIO_CGTNE);
	_gpio.cggpe = intelce_gpio_port_read32(io_port + CE2600_CORE_WELL_GPIO_CGGPE);
	_gpio.cgsmi = intelce_gpio_port_read32(io_port + CE2600_CORE_WELL_GPIO_CGSMI);


	/* Keep status of general purpose GPIO*/
	_gpio.mux_ctl = intelce_gpio_mmio_read32(virt_io_mem + CE2600_PUB_GPIO_MUX_CTL);
	_gpio.group[0].polarity = intelce_gpio_mmio_read32(virt_io_mem + CE2600_PUB_GPIO_POLARITY0);
	_gpio.group[1].polarity = intelce_gpio_mmio_read32(virt_io_mem + CE2600_PUB_GPIO_POLARITY1);
	_gpio.group[2].polarity = intelce_gpio_mmio_read32(virt_io_mem + CE2600_PUB_GPIO_POLARITY2);
	_gpio.group[3].polarity = intelce_gpio_mmio_read32(virt_io_mem + CE2600_PUB_GPIO_POLARITY3);
	_gpio.int_router = intelce_gpio_mmio_read32(virt_io_mem + CE2600_PUB_GPIO_INT_ROUTER);
	for (i=0; i < 4; i++) {
		_gpio.group[i].output = intelce_gpio_mmio_read32(virt_io_mem + CE2600_PUB_GPIO_OUT);
		_gpio.group[i].output_enable = intelce_gpio_mmio_read32(virt_io_mem + CE2600_PUB_GPIO_OUT_EN);
		_gpio.group[i].int_enable = intelce_gpio_mmio_read32(virt_io_mem + CE2600_PUB_GPIO_INT_EN);
		_gpio.group[i].mode_le = intelce_gpio_mmio_read32(virt_io_mem + CE2600_PUB_GPIO_INT_MODE_LE);
		_gpio.group[i].mode_rf = intelce_gpio_mmio_read32(virt_io_mem + CE2600_PUB_GPIO_INT_MODE_RF);
		virt_io_mem += 0x20;
	}
	return 0;
}

/* CE2600 gpio resume routine */
int ce2600_gpio_resume(void *io_mem, unsigned short io_port)
{
	char *virt_io_mem = (char *)io_mem;
	int i;

    /* Restore status of general purpose GPIO*/
	intelce_gpio_mmio_write32(_gpio.mux_ctl, virt_io_mem + CE2600_PUB_GPIO_MUX_CTL);
	intelce_gpio_mmio_write32(_gpio.group[0].polarity, virt_io_mem + CE2600_PUB_GPIO_POLARITY0);
	intelce_gpio_mmio_write32(_gpio.group[1].polarity, virt_io_mem + CE2600_PUB_GPIO_POLARITY1);
	intelce_gpio_mmio_write32(_gpio.group[2].polarity, virt_io_mem + CE2600_PUB_GPIO_POLARITY2);
	intelce_gpio_mmio_write32(_gpio.group[3].polarity, virt_io_mem + CE2600_PUB_GPIO_POLARITY3);
	intelce_gpio_mmio_write32(_gpio.int_router, virt_io_mem + CE2600_PUB_GPIO_INT_ROUTER);
	for (i=0; i < 4; i++) {
		intelce_gpio_mmio_write32(_gpio.group[i].output, virt_io_mem + CE2600_PUB_GPIO_OUT);
		intelce_gpio_mmio_write32(_gpio.group[i].output_enable, virt_io_mem + CE2600_PUB_GPIO_OUT_EN);
		intelce_gpio_mmio_write32(_gpio.group[i].int_enable, virt_io_mem + CE2600_PUB_GPIO_INT_EN);
		intelce_gpio_mmio_write32(_gpio.group[i].mode_le, virt_io_mem + CE2600_PUB_GPIO_INT_MODE_LE);
		intelce_gpio_mmio_write32(_gpio.group[i].mode_rf, virt_io_mem + CE2600_PUB_GPIO_INT_MODE_RF);
		virt_io_mem += 0x20;

	}

    /* Restore status of Core Well GPIO*/
	intelce_gpio_port_write32(_gpio.cgen & 0x3FF, io_port + CE2600_CORE_WELL_GPIO_CGEN);
	intelce_gpio_port_write32(_gpio.cgio & 0x3FF, io_port + CE2600_CORE_WELL_GPIO_CGIO);
	intelce_gpio_port_write32(_gpio.cglv & 0x3FF, io_port + CE2600_CORE_WELL_GPIO_CGLV);
	intelce_gpio_port_write32(_gpio.cgtpe & 0x3FF, io_port + CE2600_CORE_WELL_GPIO_CGTPE);
	intelce_gpio_port_write32(_gpio.cgtne & 0x3FF, io_port + CE2600_CORE_WELL_GPIO_CGTNE);
	intelce_gpio_port_write32(_gpio.cggpe & 0x3FF, io_port + CE2600_CORE_WELL_GPIO_CGGPE);
	intelce_gpio_port_write32(_gpio.cgsmi & 0x3FF, io_port + CE2600_CORE_WELL_GPIO_CGSMI);
	return 0;
}



static int banks, gpios_per_bank;
static char *label[CE2600_PUB_GPIO_BANKS] = {
	"gpio_bank0",
	"gpio_bank1",
	"gpio_bank2",
	"gpio_bank3",
};
#if 0
static struct resource sch_gpio_resource[] = {
	[0] = {
		.start = 1048,
		.end   = 1048 + 64 - 1,
		.flags = IORESOURCE_IO,
	},
	[1] = {
		.start = 0,
		.end   = 0,
		.flags = IORESOURCE_IRQ,
	},
};
static struct platform_device sch_device_gpio = {
	.name		  = "sch_gpio_ce",
	.id		  = PCI_DEVICE_ID_INTEL_SCH_LPC,
	.num_resources	  = ARRAY_SIZE(sch_gpio_resource),
	.resource	  = sch_gpio_resource,
};

static uint16_t  legacy_iobase = 1048;
static int sch_gpio_setup(struct pci_dev *pdev, int gpio_base)
{
    struct pci_dev *lpc;

	sch_device_gpio.dev.parent = get_device(&pdev->dev);

    lpc = pci_get_domain_bus_and_slot(0, 0, PCI_DEVFN(31, 0));
    if (NULL == lpc) {
       printk(KERN_ERR "Can't detect the  LPC PCI device!!");
       return -ENODEV;
    }
    pci_read_config_word(lpc, 0x44, &legacy_iobase);
    pci_dev_put(lpc);
	legacy_iobase &= ~(64 - 1);
	sch_gpio_resource[0].start = legacy_iobase;
	sch_gpio_resource[0].end = legacy_iobase + 64 - 1;
	sch_gpio_resource[1].start = gpio_base;
	sch_gpio_resource[1].end = gpio_base;

	return platform_device_register(&sch_device_gpio);
}
#endif

static int (*intelce_gpio_suspend)(void *io_mem, unsigned short io_port);
static int (*intelce_gpio_resume)(void *io_mem, unsigned short io_port);

static int intelce_gpio_probe(struct pci_dev *pdev, const struct pci_device_id *pci_id)
{
	struct intelce_gpio_chip *c, *p;
	unsigned long paddr;
	void *vaddr;
	int i;
	int gpio_base = 0;
	int mux_ctl_offset = CE2600_PUB_GPIO_MUX_CTL;
	int ret;

    banks = CE2600_PUB_GPIO_BANKS;
    gpios_per_bank = CE2600_PUB_GPIOS_PER_BANK;
    mux_ctl_offset = CE2600_PUB_GPIO_MUX_CTL;
    intelce_gpio_suspend = NULL;
    intelce_gpio_resume = NULL;
    dev_info(&pdev->dev, "CE2600 GPIO controller detected.\n");

	c = kzalloc(sizeof(struct intelce_gpio_chip)*banks, GFP_KERNEL);
	if (!c)
		return -ENOMEM;
	ret = pci_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "can't enable device.\n");
		goto done;
	}

	ret = pci_request_region(pdev, INTELCE_GPIO_BAR, INTELCE_GPIO_DRV_NAME);
	if (ret) {
		dev_err(&pdev->dev, "can't alloc PCI BAR #%d\n", INTELCE_GPIO_BAR);
		goto disable_pci;
	}

	paddr = pci_resource_start(pdev, INTELCE_GPIO_BAR);
	if (!paddr)
		goto release_reg;
	vaddr = ioremap(paddr, pci_resource_len(pdev, INTELCE_GPIO_BAR));

	for (i=0; i < banks; i++) {
		p = c + i;
		spin_lock_init(&p->lock);
		p->reg_base = vaddr + 0x20*i;
		p->high_base = vaddr + 0x80 + 0x10*i;
		p->mux_ctl_base = vaddr + mux_ctl_offset;
		p->chip.ngpio = gpios_per_bank;
		//p->chip.set_multi_function = gpio_set_multi_function;
		//p->chip.get_multi_function = gpio_get_multi_function;
		p->chip.set = ce2600_gpio_set;
		p->chip.get = ce2600_gpio_get;
		p->chip.direction_output = ce2600_gpio_direction_output;
		p->chip.direction_input = ce2600_gpio_direction_input;
//		p->chip.to_irq = ce4100_gpio_to_irq;
		p->chip.label = label[i];
		p->chip.base = gpio_base;
		ret = gpiochip_add(&p->chip);
		if (ret)
			goto unmap;
		//ret = ce2600_gpio_irq_setup(p, pdev);
		if (ret) {
			gpiochip_remove(&p->chip);
			goto unmap;
		}
		gpio_base += p->chip.ngpio;
	}
	//sch_gpio_setup(pdev, gpio_base);
	pci_set_drvdata(pdev, c);
	return 0;

unmap:
	for (i--; i >= 0; i--) {
		p = c + i ;
		free_irq(pdev->irq, p);
		irq_free_descs(p->irq_base, gpios_per_bank);
		gpiochip_remove(&p->chip);
	}
	iounmap(c->reg_base);
release_reg:
	pci_release_region(pdev, INTELCE_GPIO_BAR);
disable_pci:
	pci_disable_device(pdev);
done:
	kfree(c);
	return ret;
}

static void intelce_gpio_remove(struct pci_dev *pdev)
{
	struct intelce_gpio_chip *c = pci_get_drvdata(pdev);
	struct intelce_gpio_chip *p;
	int i;

	//platform_device_del(&sch_device_gpio);
	put_device(&pdev->dev);
	for (i=0; i < banks; i++) {
		p = c + i;
		free_irq(pdev->irq, p);
		irq_free_descs(p->irq_base, gpios_per_bank);
		gpiochip_remove(&p->chip);
	}
	pci_release_region(pdev, INTELCE_GPIO_BAR);
	iounmap(c->reg_base);
	pci_disable_device(pdev);
	kfree(c);
	pci_set_drvdata(pdev, NULL);
}

static struct pci_device_id intelce_gpio_pci_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_INTELCE_GPIO_DEVICE_ID) },
	{ 0, },
};

#ifdef CONFIG_PM

int intelce_gpio_device_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct intelce_gpio_chip *c = pci_get_drvdata(pdev);
	int ret = 0;

    /*gpio suspend */
	if (intelce_gpio_suspend) {
		//ret = intelce_gpio_suspend(c->reg_base, legacy_iobase);
		if (ret)
			 return ret;
	}
	/*pci device save*/
	pci_save_state(pdev);
	pci_disable_device(pdev);
	pci_set_power_state(pdev, PCI_D3hot);
	return 0;
}

int intelce_gpio_device_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct intelce_gpio_chip *c = pci_get_drvdata(pdev);
	int ret = 0;

	/*pci device restore*/
	pci_set_power_state(pdev, PCI_D0);
	pci_restore_state(pdev);
	ret = pci_enable_device(pdev);
	if (ret)
		return ret;
	/*gpio resume */
	if (intelce_gpio_resume) {
		//return intelce_gpio_resume(c->reg_base, legacy_iobase);
        return -1;
	} else {
		return 0;
	}
}

static const struct dev_pm_ops intelce_gpio_pm_ops = {
	.suspend	= intelce_gpio_device_suspend,
	.resume		= intelce_gpio_device_resume,
};
#endif

static struct pci_driver intelce_gpio_driver = {
	.name = INTELCE_GPIO_DRV_NAME,
	.id_table = intelce_gpio_pci_ids,
	.probe = intelce_gpio_probe,
	.remove = intelce_gpio_remove,
#ifdef CONFIG_PM
	.driver.pm = &intelce_gpio_pm_ops,
#endif
};

static int __init intelce_gpio_init(void)
{
	return pci_register_driver(&intelce_gpio_driver);

}
module_init(intelce_gpio_init);

static void __exit intelce_gpio_exit(void)
{
	pci_unregister_driver(&intelce_gpio_driver);
}
module_exit(intelce_gpio_exit);

MODULE_DESCRIPTION("GPIO interface for Intel intelce SoCs");
MODULE_LICENSE("GPL v2");
