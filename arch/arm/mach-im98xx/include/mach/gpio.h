#include <mach/magic.h>
#include <mach/mem_map.h>
#ifndef GPIO_H
#define GPIO_H

#define GPIO_IN			          0
#define GPIO_OUT		          1
#define GPIO_LOW		          0
#define GPIO_HIGH		          1
#define GPIO_PULL_LOW_ENABLE      0
#define GPIO_PULL_LOW_DISENABLE   1
#define GPIO_NO_INT_GEN           0
#define GPIO_INT_GEN              1
#define RISING_EDGE_TRIGGER       0x00
#define FALLING_EDGE_TRIGGER      0x01
#define HIGH_LEVEL_TRIGGER        0x02
#define LOW_LEVEL_TRIGGER         0x03
#define BOTH_EDGE_TRIGGER         0x04

#define EXTERNAL_INT_SET          1
#define EXTERNAL_INT_CLEAR        0

static inline unsigned int gpio_line_get_input(unsigned int line)
{
	return (readl(GPIO_IN_REG) & (1 << line));
}

static inline void gpio_line_config(unsigned int line, unsigned int direction)
{
	unsigned int val = readl(GPIO_OEN_REG);

	if (direction == GPIO_INPUT) {
		val |= (1 << line);
	} else if (direction == GPIO_OUTPUT) {
		val &= ~(1 << line);
	} else {
		return;
	}

	writel(val, GPIO_OEN_REG);
}

static inline void gpio_line_set_output(unsigned int line, unsigned int value)
{
	unsigned int val = readl(GPIO_OUT_REG);

	if (value == GPIO_LOW) {
		val &= ~(1 << line);
	} else if (value == GPIO_HIGH) {
		val |= (1 << line);
	} else {
		return;
	}

	writel(val, GPIO_OUT_REG);
}

static inline void gpio_line_set_pull_low(int line, int value)
{
    //unsigned long flags;
	//local_irq_save(flags);
	if (value == GPIO_PULL_LOW_ENABLE) 
    {
		writel(readl(GPIO_REN_REG) & ~(1 << line),GPIO_REN_REG);
	} 
    else if (value == GPIO_PULL_LOW_DISENABLE) 
	{
		writel(readl(GPIO_REN_REG) | (1 << line), GPIO_REN_REG);
	}
	//local_irq_restore(flags);
    
}

static inline void gpio_line_set_interrupt_enable_disable(int line, int value)
{
    //unsigned long flags;
	//local_irq_save(flags);
	if (value == GPIO_NO_INT_GEN) 
    {
		writel(readl(GPIO_EINT_REG) & ~(1 << line),GPIO_EINT_REG);
	} 
    else if (value == GPIO_INT_GEN) 
	{
		writel(readl(GPIO_EINT_REG) | (1 << line), GPIO_EINT_REG);
	}
	//local_irq_restore(flags);
    
}


static inline void external_int_line_clear_set(int line, int value)
{
	//unsigned long flags;
	//local_irq_save(flags);
	//#define EXTERNAL_INT_SET          1
	//#define EXTERNAL_INT_CLEAR        0 
	//writel(readl(GPIO_ITR_FLAG_REG) | EXTER_INT_REG_GPIO1, GPIO_ITR_FLAG_REG);
	if (value == EXTERNAL_INT_SET) 
	{
		/* RD3, bohung.wu, 20101216, external interrupt cannot be set by itself */ 
		//writel(readl(GPIO_ITR_FLAG_REG) & ~(1 << line),GPIO_ITR_FLAG_REG);
	} 
	else if (value == EXTERNAL_INT_CLEAR) 
	{
		/* RD3, bohung.wu, 20101216, fix incorrect external interrupt clear usage */
		//writel(readl(GPIO_ITR_FLAG_REG) | (1 << line), GPIO_ITR_FLAG_REG);
		writel((0x1 << line), GPIO_ITR_FLAG_REG);
	}
	//local_irq_restore(flags);
}


static inline int external_int_line_detect(int line)
{
    return (readl(GPIO_ITR_FLAG_REG) & (1 << line)) >> line;
}


static inline void gpio_line_set_De_Bounce(int value1, int value2)
{
    //unsigned long flags;
	//local_irq_save(flags);
	writel(value1, GPIO_XITR0_DE_REG); 
    writel(value2, GPIO_XITR1_DE_REG); 
	//local_irq_restore(flags);
    
}


static inline void gpio_line_set_edgr_tri(int line, int trigstate)
{
#if 0
    if((line >= 0) && (line <= 7))
    {
        writel(readl(GPIO_ITR_CFG0_REG) | tgigstate << (line * 4), GPIO_ITR_CFG0_REG);
    }
    else if((line >= 8) && (line <= 15))
    {
        writel(readl(GPIO_ITR_CFG0_REG) | ((line - 8) * 4) << tgigstate, GPIO_ITR_CFG1_REG);
    }
    else if((line >= 16) && (line <= 23))
    {
        writel(readl(GPIO_ITR_CFG0_REG) | ((line - 16) * 4) << tgigstate, GPIO_ITR_CFG2_REG);
    }
#else // ian.hung 20110509
    int offset;
    uint32_t reg_addr, conf;

    if((line >= 0) && (line <= 7))
    {
	offset = (line) << 2;
	reg_addr = GPIO_ITR_CFG0_REG;
    }
    else if((line >= 8) && (line <= 15))
    {
	offset = (line - 8) << 2;
	reg_addr = GPIO_ITR_CFG1_REG;
    }
    else if((line >= 16) && (line <= 23))
    {
	offset = (line - 16) << 2;
	reg_addr = GPIO_ITR_CFG2_REG;
    }

    conf = readl(reg_addr);
    mb();
    conf &= ~(0x7 << offset); // clear setting
    conf |= (trigstate << offset);
    writel(conf, reg_addr);
#endif
    return;
}
#endif
