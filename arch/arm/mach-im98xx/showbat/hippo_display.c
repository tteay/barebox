#include <common.h>
#include <asm/io.h>
#include <mach/magic.h>
#include <mach/hippo_display.h>
#include <mach/hippo_showbat.h>

#define IO_READ(base, reg)		readl((unsigned int)(base)+(reg))
#define IO_WRITE(val, base, reg)	writel(val,(unsigned int)(base)+(reg))

extern void im98xx_sleep2itr(unsigned int u32_microsecond);

/* AHB CLOCK configuration */
#if 0
#if defined(CONFIG_IM98XX_ARM9_CLOCK_520_CONFIG)
#define AHB_CLK 260000000			// 260MHz
#elif defined(CONFIG_IM98XX_ARM9_CLOCK_494_CONFIG)
#define AHB_CLK 247000000			// 247MHz
#elif defined(CONFIG_IM98XX_ARM9_CLOCK_416_CONFIG)
#define AHB_CLK 208000000			// 208MHz
#elif defined(CONFIG_IM98XX_ARM9_CLOCK_624_CONFIG)
#define AHB_CLK 208000000
#elif defined(CONFIG_IM98XX_ARM9_CLOCK_650_CONFIG)
#define AHB_CLK 208000000
#elif defined(CONFIG_IM98XX_ARM9_CLOCK_702_CONFIG)
#define AHB_CLK 208000000
#elif defined(CONFIG_IM98XX_ARM9_CLOCK_728_CONFIG)
#define AHB_CLK 208000000
#elif defined(CONFIG_IM98XX_ARM9_CLOCK_754_CONFIG)
#define AHB_CLK 208000000
#elif defined(CONFIG_IM98XX_ARM9_CLOCK_780_CONFIG)
#define AHB_CLK 208000000
#elif defined(CONFIG_IM98XX_ARM9_CLOCK_806_CONFIG)
#define AHB_CLK 268000000
#elif defined(CONFIG_IM98XX_ARM9_CLOCK_910_CONFIG)
#define AHB_CLK 227500000
#else
#error "No proper configuration for AHB_CLK"
#endif
#endif

/* XMEM CLOCK configuration */
#if defined(CONFIG_IM98XX_XMEM_CLOCK_198_CONFIG)
#define SDRAM_CLK		198000000	// 198 MHz
#define SYSTEM_CLK_TIME		6		// flooring((10^9 + SDRAM_CLK - 1)/SDRAM_CLK)
#elif defined(CONFIG_IM98XX_XMEM_CLOCK_130_CONFIG)
#define SDRAM_CLK		130000000	// 130 MHz
#define SYSTEM_CLK_TIME		8		// flooring((10^9 + SDRAM_CLK - 1)/SDRAM_CLK)
#elif defined(CONFIG_IM98XX_XMEM_CLOCK_104_CONFIG)
#define SDRAM_CLK		104000000	// 104 MHz
#define SYSTEM_CLK_TIME		10		// flooring((10^9 + SDRAM_CLK - 1)/SDRAM_CLK)
#else
#error "No proper configuration for XMEM CLOCK"
#endif

#define WAITING_TIME	25000

/* Reference to panel configuration */
static const void *pcfg = NULL;

/* Registration API for individual panel */
void register_panel(const void *panel_cfg)
{
	pcfg = panel_cfg;
}

#ifdef CONFIG_IM98XX_CONTROL_IF_I80
/************************/
/* If IF_TYPE is IF_I80 */
/************************/

/* Trigger a RS+WR signal */
static void lcmSendCmd(int cmd)
{
	writel(cmd, LCD_WR_CMD_TRG_REG);
}

/* Trigger a WR signal */
static void lcmSendData(int data)
{
	writel(data, LCD_WR_DAT_TRG_REG);
}

/* Initiailize panel with lcm if */
static void initialize_lcm(void)
{
	const hippo_lcm_panel_t *lcm_panel = (const hippo_lcm_panel_t*)pcfg;
	pif_ctl_reg cur_pif_ctl;
	unsigned int width = ACQUIRE_PANEL_XRES(pcfg) - 1;
	unsigned int height = ACQUIRE_PANEL_YRES(pcfg) - 1;
	unsigned int pixel_fmt = ACQUIRE_PANEL_PIXFMT(pcfg);

	/* LCD_PIF#_CTL_REG */
	cur_pif_ctl.reg_field.rplw		= lcm_panel->rplw / SYSTEM_CLK_TIME;
	cur_pif_ctl.reg_field.rphw		= lcm_panel->rphw / SYSTEM_CLK_TIME;
	cur_pif_ctl.reg_field.wplw		= lcm_panel->wplw / SYSTEM_CLK_TIME;
	cur_pif_ctl.reg_field.wphw		= lcm_panel->wphw / SYSTEM_CLK_TIME;
	cur_pif_ctl.reg_field.pif_width		= lcm_panel->pif_width & 0x1;
	cur_pif_ctl.reg_field.short_in_msb	= lcm_panel->pif_endian & 0x1;
	IO_WRITE(cur_pif_ctl.reg_value, LCD_PIF1_CTL_REG, lcm_panel->pif_id << 2);

	/* LCD_MAN_SLV_SEL_REG */
	writel(lcm_panel->pif_id << 4, LCD_MAN_SLV_SEL_REG);

	/* Initialize display controller */
	writel(0x80000080, 0xF8004E00);
	writel(0x2|(pixel_fmt << 10), 0xF8004E08);		// Only MID
	writel(width | (height << 16), 0xF8004E10);

	/* Do panel initialization */
	lcm_panel->init(lcmSendCmd, lcmSendData);
}
#endif

#ifdef CONFIG_IM98XX_CONTROL_IF_RGB
/************************/
/* If IF_TYPE is IF_RGB */
/************************/
void lcd_resume_spi(void)
{
	writel(0x0, HIF_SPI_MANPAUS_REG);
}

void lcd_pause_spi(void)
{
	writel(0x1, HIF_SPI_MANPAUS_REG);
}

void lcd_set_bt_count(int count)
{
	writel(count & 0x3FFFFF, HIF_SPI2_TRSCNT_REG);
}

/* 1-bit D/CX(=0) + 8-bit Transmission Byte */
/* D/CX | TB | D/CX | TB | D/CX | TB | ...  */
/* This is paired with "9bits_DATA."        */
static void lcd_write_dataport_9bits_CMD(int cmd_data)
{
	writel(0x8, HIF_SPI_TAGDAW_REG);

	writel(0x0 | cmd_data, HIF_SPI_WTDAT_REG);
}

/* 1-bit D/CX(=1) + 8-bit Transmission Byte */
/* D/CX | TB | D/CX | TB | D/CX | TB | ...  */
static void lcd_write_dataport_9bits_DATA(int cmd_data)
{
	writel(0x8, HIF_SPI_TAGDAW_REG);

	writel(0x100 | cmd_data, HIF_SPI_WTDAT_REG);
}

// calculate AHB from current Frequency and HCLK_DIV value
extern unsigned long calculated_AHB;

static void initialize_lcd(void)
{
	const hippo_lcd_panel_t *lcd_panel = (const hippo_lcd_panel_t*)pcfg;
	rgb_if_ctl_reg cur_rgb_if_ctl;
	rgb_sync_ctl_reg rgb_vsync_ctl;
	rgb_sync_ctl_reg rgb_hsync_ctl;
	spi_ctl_reg spi_ctl;
	unsigned int pixel_clk, serial_clk;

	unsigned int width = ACQUIRE_PANEL_XRES(pcfg) - 1;
	unsigned int height = ACQUIRE_PANEL_YRES(pcfg) - 1;
	unsigned int pixel_fmt = ACQUIRE_PANEL_PIXFMT(pcfg);

	/* Init RGB IF */
	pixel_clk = 1000000000 / lcd_panel->dclk;
	cur_rgb_if_ctl.reg_field.pixel_clk_divider = (( SDRAM_CLK / pixel_clk ) + 1) & 0x3F;
	cur_rgb_if_ctl.reg_field.pixel_clk_polarity = lcd_panel->pico_polarity & 0x1;
	cur_rgb_if_ctl.reg_field.vsync_polarity = lcd_panel->vsync_polarity & 0x1;
	cur_rgb_if_ctl.reg_field.hsync_polarity = lcd_panel->hsync_polarity & 0x1;
	writel(cur_rgb_if_ctl.reg_value, LCD_CLK_CTRL_REG);

	rgb_vsync_ctl.reg_field.spw = lcd_panel->vspw & 0xFF;
	rgb_vsync_ctl.reg_field.fpd = lcd_panel->vfpd & 0xFF;
	rgb_vsync_ctl.reg_field.bpd = lcd_panel->vbpd & 0xFF;
	writel(rgb_vsync_ctl.reg_value, LCD_VSYNC_CTRL_REG);

	rgb_hsync_ctl.reg_field.spw = lcd_panel->hspw & 0xFF;
	rgb_hsync_ctl.reg_field.fpd = lcd_panel->hfpd & 0xFF;
	rgb_hsync_ctl.reg_field.bpd = lcd_panel->hbpd & 0xFF;
	writel(rgb_hsync_ctl.reg_value, LCD_HSYNC_CTRL_REG);

	/* Init SPI2 Control */
	serial_clk = 2000000000 / lcd_panel->sck;
	spi_ctl.reg_field.sc_divisor = (calculated_AHB / serial_clk) & 0x3F;	// calculate AHB from current Frequency and HCLK_DIV value
	spi_ctl.reg_field.clk_polarity = lcd_panel->cpol & 0x1;
	spi_ctl.reg_field.clk_phase = lcd_panel->cpha & 0x1;
	writel(spi_ctl.reg_value, HIF_SPI2_CTL_REG);
	writel((lcd_panel->sdw - 1) & 0xF, HIF_SPI_TAGDAW_REG);

	/* Select SPI2 */
	writel(0x2, HIF_SPI_TAGSL_REG);
	
	/* Initialize display controller */
	writel(0x80000070, 0xF8004E00);
	writel(0x2 | (pixel_fmt << 10), 0xF8004E08);		// Only MID
	writel(width | (height << 16), 0xF8004E10);

	/* Do panel initialization */
	lcd_panel->init(lcd_write_dataport_9bits_CMD, lcd_write_dataport_9bits_DATA);
}
#endif

void to_initialize_lcmlcd(void)
{
	printf("\nLCM/LCD enable\n");
#ifdef CONFIG_IM98XX_CONTROL_IF_I80
	initialize_lcm();
#endif

#ifdef CONFIG_IM98XX_CONTROL_IF_RGB
	initialize_lcd();
#endif
}

void turn_lcmlcd(bool on)
{
#ifdef CONFIG_IM98XX_CONTROL_IF_I80
	const hippo_lcm_panel_t *lcm_panel = (const hippo_lcm_panel_t*)pcfg;

	if (on) {
		lcm_panel->awake(lcmSendCmd, lcmSendData);
	} else {
		lcm_panel->sleep(lcmSendCmd, lcmSendData);
	}
#endif

#ifdef CONFIG_IM98XX_CONTROL_IF_RGB
	const hippo_lcd_panel_t *lcd_panel = (const hippo_lcd_panel_t*)pcfg;

	if (on) {
		lcd_panel->awake(lcd_write_dataport_9bits_CMD, lcd_write_dataport_9bits_DATA);
	} else {
		lcd_panel->sleep(lcd_write_dataport_9bits_CMD, lcd_write_dataport_9bits_DATA);
	}
#endif
}

/* Main entry */
void to_show_battery(struct image_grp *igrp)
{
	int t;
	/* forbidden cases */
	if (!pcfg || !igrp)
		return;

	/* To display battery here */
	switch (igrp->which_layer) {
		case LAY_TOP:
			/* No use */
			break;
		case LAY_MID:
			/* Configure X/Y offset */
			igrp->xofft = (ACQUIRE_PANEL_XRES(pcfg) - igrp->xres) >> 1;
			igrp->yofft = (ACQUIRE_PANEL_YRES(pcfg) - igrp->yres) >> 1;
			/* Set display controller */
			writel(((0x1 << 24) | (igrp->pixel_fmt << 16) | 0x100), 0xF8004E40);
			writel((unsigned int)igrp->base, 0xF8004E44);
			writel((igrp->xres - 1) | ((igrp->yres - 1) << 16), 0xF8004E48);
			writel(igrp->xofft | (igrp->yofft << 16), 0xF8004E50);
			writel((igrp->xofft + igrp->xres - 1) | ((igrp->yofft + igrp->yres - 1) << 16), 0xF8004E54);
			break;
		case LAY_BTM:
			/* No use */
			break;
		default:
			/* Do nothing */
			break;
	}

	/* Start to display */
#ifdef CONFIG_IM98XX_CONTROL_IF_I80
	writel(0x1, 0xF8004E04);
#endif

#ifdef CONFIG_IM98XX_CONTROL_IF_RGB
	writel(0x80000010, 0xF8004E00);
#endif
	for (t = 0; (((readl(0xF8004E0C) & 0x2) == 0) && (t < WAITING_TIME)); t++)
		im98xx_sleep2itr(1);

	if (t >= WAITING_TIME)
		printk("%s - To indicate a frame transfer to LCD panel has been finished FAIL!!!\n", __func__);
	else
		printk("%s - t = %d!!!\n", __func__, t);

	writel(0x7, 0xF8004E0C);

#ifdef CONFIG_IM98XX_CONTROL_IF_RGB
	writel(0x80000070, 0xF8004E00);
	writel(0x80000010, 0xF8004E00);
	while ((readl(0xF8004E0C) & 0x2) == 0);
	writel(0x7, 0xF8004E0C);
	writel(0x80000070, 0xF8004E00);
#endif
}
