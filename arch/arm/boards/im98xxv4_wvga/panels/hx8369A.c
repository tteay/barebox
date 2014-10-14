#include <stdio.h>
#include <common.h>
#include <mach/hippo_display.h>
#include <mach/hippo_showbat.h>

static void lcd_init(PANEL_CMD lcd_cmd, PANEL_DATA lcd_data)
{
	printf("%s()\n", __FUNCTION__);

	/* Resume SPI port TX FIFO output */
	lcd_resume_spi();

	/* SW Reset */
	lcd_cmd(0x01);
	mdelay(10);

	/* Set Extension Cmd */
	lcd_set_bt_count(4);
	lcd_cmd  (0xB9);
	lcd_data (0xFF);
	lcd_data (0x83);
	lcd_data (0x69);

	/* Set Power */
	lcd_set_bt_count(20);
	lcd_cmd  (0xB1);
	lcd_data (0x01);
	lcd_data (0x00);
	lcd_data (0x34);
	lcd_data (0x06);
	lcd_data (0x00);
	lcd_data (0x0F);
	lcd_data (0x0F);
	lcd_data (0x1A);
	lcd_data (0x21);
	lcd_data (0x3F);
	lcd_data (0x3F);
	lcd_data (0x07);
	lcd_data (0x23);
	lcd_data (0x01);
	lcd_data (0xE6);
	lcd_data (0xE6);
	lcd_data (0xE6);
	lcd_data (0xE6);
	lcd_data (0xE6);

	/* Set Display Related Regs */
	lcd_set_bt_count(16);
	lcd_cmd  (0xB2);
	lcd_data (0x00);
	lcd_data (0x28);
	lcd_data (0x06);	// ? BP lines
	lcd_data (0x06);	// ? FP lines
	lcd_data (0x70);
	lcd_data (0x00);
	lcd_data (0xFF);
	lcd_data (0x00);
	lcd_data (0x00);
	lcd_data (0x00);
	lcd_data (0x00);
	lcd_data (0x03);
	lcd_data (0x03);
	lcd_data (0x00);
	lcd_data (0x01);

	/* Set Display Waveform Cycle */
	lcd_set_bt_count(6);
	lcd_cmd  (0xB4);
	lcd_data (0x00);
	lcd_data (0x0C);
	lcd_data (0x84);
	lcd_data (0x0C);
	lcd_data (0x01);

	/* Set VCOM voltage */
	lcd_set_bt_count(3);
	lcd_cmd  (0xB6);
	lcd_data (0x21);
	lcd_data (0x21);

	/* Set GIP */
	lcd_set_bt_count(27);
	lcd_cmd  (0xD5);
	lcd_data (0x00);	// SHR_0
	lcd_data (0x05);
	lcd_data (0x03);
	lcd_data (0x00);
	lcd_data (0x01);	// SPD
	lcd_data (0x09);
	lcd_data (0x10);
	lcd_data (0x80);
	lcd_data (0x37);	// SHP
	lcd_data (0x37);
	lcd_data (0x20);
	lcd_data (0x31);
	lcd_data (0x46);
	lcd_data (0x8A);
	lcd_data (0x57);
	lcd_data (0x9B);
	lcd_data (0x20);
	lcd_data (0x31);
	lcd_data (0x46);
	lcd_data (0x8A);
	lcd_data (0x57);
	lcd_data (0x9B);
	lcd_data (0x07);
	lcd_data (0x0F);
	lcd_data (0x07);
	lcd_data (0x00);

	/* Set gamma curve related setting */
	lcd_set_bt_count(35) ;
	lcd_cmd  (0xE0);
	lcd_data (0x00);
	lcd_data (0x01);
	lcd_data (0x03);
	lcd_data (0x2B);
	lcd_data (0x33);
	lcd_data (0x3F);
	lcd_data (0x0D);
	lcd_data (0x30);
	lcd_data (0x06);
	lcd_data (0x0B);
	lcd_data (0x0D);
	lcd_data (0x10);
	lcd_data (0x13);
	lcd_data (0x11);
	lcd_data (0x13);
	lcd_data (0x11);
	lcd_data (0x17);
	lcd_data (0x00);
	lcd_data (0x01);
	lcd_data (0x03);
	lcd_data (0x2B);
	lcd_data (0x33);
	lcd_data (0x3F);
	lcd_data (0x0D);
	lcd_data (0x30);
	lcd_data (0x06);
	lcd_data (0x0B);
	lcd_data (0x0D);
	lcd_data (0x10);
	lcd_data (0x13);
	lcd_data (0x11);
	lcd_data (0x13);
	lcd_data (0x11);
	lcd_data (0x17);

	/* Set Pixel Format */
	lcd_set_bt_count(2);
	lcd_cmd  (0x3A);
	lcd_data (0x66);	// 18-bit

	/* Exit Sleep Mode */
	lcd_cmd  (0x11);
	mdelay(120);

	/* Set Display On */
	lcd_cmd  (0x29);

	/* Set Column Address */
	lcd_set_bt_count(5);
	lcd_cmd  (0x2A);
	lcd_data (0x00);
	lcd_data (0x00);
	lcd_data (0x01);
	lcd_data (0xDF);
	mdelay(10);

	/* Set Page Address */
	lcd_set_bt_count(5);
	lcd_cmd  (0x2B);
	lcd_data (0x00);
	lcd_data (0x00);
	lcd_data (0x03);
	lcd_data (0x1F);
	mdelay(10);

//	lcd_set_bt_count(4801);
	/* Write Memory Start */
	lcd_cmd(0x2C);
#if 0
	{
		int i ;
		for (i = 0 ; i < 4800;++i)
			lcd_data(0xFF) ;
	}
#endif
	/* Pause SPI port TX FIFO output */
	lcd_pause_spi();
}

static void lcd_sleep(PANEL_CMD lcd_cmd, PANEL_DATA lcd_data)
{
	printf("%s()\n", __FUNCTION__);
}

static void lcd_awake(PANEL_CMD lcd_cmd, PANEL_DATA lcd_data)
{
	printf("%s()\n", __FUNCTION__);
}

static const hippo_lcd_panel_t lcd =
{
	.panel_attribute = {
		.if_type	= IF_RGB,
		.pixel_fmt	= RGB666,
		.xres		= 480,
		.yres		= 800,
	},

	.init		= lcd_init,
	.sleep		= lcd_sleep,
	.awake		= lcd_awake,

	.dclk		= 40,		/* in ns */
	.pico_polarity	= FALLING,
	.hspw		= 4,		/* in dclks */
	.hfpd		= 4,		/* in dclks */
	.hbpd		= 4,		/* in dclks */
	.vspw		= 3,		/* in lines */
	.vfpd		= 7,		/* in lines */
	.vbpd		= 7,		/* in lines */
	.hsync_polarity	= ACTIVE_LOW,
	.vsync_polarity	= ACTIVE_LOW,
	.sck		= 150,		/* in ns *//* in MHz, which is computed as 6.67 MHz */
	.cpol		= 0x0,
	.cpha		= 0x0,
	.sdw		= 9		/* serial data width */
};

/* Panel entry function */
void panel_main_entry(void)
{
	printf("HX8369A  %s()\n", __FUNCTION__);
	register_panel(&lcd);
}
