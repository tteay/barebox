#include <stdio.h>
#include <common.h>
#include <mach/hippo_display.h>
#include <mach/hippo_showbat.h>

static const hippo_lcm_panel_t lcm;

static void lcm_init(PANEL_CMD lcm_cmd, PANEL_DATA lcm_data)
{
	unsigned short lt_x = 0;
	unsigned short lt_y = 0;
	unsigned short rb_x = lcm.panel_attribute.xres - 1;
	unsigned short rb_y = lcm.panel_attribute.yres - 1;

	printf("%s()\n",__FUNCTION__);

	lcm_cmd(0xb9);
	lcm_data(0xff);
	lcm_data(0x83);
	lcm_data(0x63);
	mdelay(10);

	lcm_cmd(0xb1);
	lcm_data(0x01);
	lcm_data(0x00);
	lcm_data(0x44);
	lcm_data(0x07);
	lcm_data(0x01);
	lcm_data(0x11);
	lcm_data(0x11);
	lcm_data(0x3a);
	lcm_data(0x42);
	lcm_data(0x3f);
	lcm_data(0x3f);
	lcm_data(0x40);
	lcm_data(0x32);
	lcm_data(0x00);
	lcm_data(0xe6);
	lcm_data(0xe6);
	lcm_data(0xe6);
	lcm_data(0xe6);
	lcm_data(0xe6);

	lcm_cmd(0xb2);
	lcm_data(0x08);
	lcm_data(0x00);
	mdelay(10);

	lcm_cmd(0xb4);
	lcm_data(0x09);
	lcm_data(0x18);
	lcm_data(0x9c);
	lcm_data(0x08);
	lcm_data(0x18);
	lcm_data(0x04);
	lcm_data(0x72);
	mdelay(10);

	lcm_cmd(0xbf);
	lcm_data(0x05);
	lcm_data(0x60);
	lcm_data(0x00);
	lcm_data(0x10);
	mdelay(10);

	lcm_cmd(0xb6);
	lcm_data(0x0a);

	lcm_cmd(0xcc);
	lcm_data(0x01);
	mdelay(10);

	lcm_cmd(0xE0);
	lcm_data(0x00);
	lcm_data(0x2D);
	lcm_data(0x31);
	lcm_data(0x2C);
	lcm_data(0x29);
	lcm_data(0x29);
	lcm_data(0x47);
	lcm_data(0x4E);
	lcm_data(0xCA);
	lcm_data(0x98);
	lcm_data(0xD2);
	lcm_data(0xD7);
	lcm_data(0xD9);
	lcm_data(0x17);
	lcm_data(0x17);
	lcm_data(0x14);
	lcm_data(0x14);
	lcm_data(0x00);
	lcm_data(0x2D);
	lcm_data(0x31);
	lcm_data(0x2C);
	lcm_data(0x29);
	lcm_data(0x29);
	lcm_data(0x47);
	lcm_data(0x4E);
	lcm_data(0xCA);
	lcm_data(0x98);
	lcm_data(0xD2);
	lcm_data(0xD7);
	lcm_data(0xD9);
	lcm_data(0x17);
	lcm_data(0x17);
	lcm_data(0x14);
	lcm_data(0x14);
	mdelay(20);

	lcm_cmd(0xc1);
	lcm_data(0x01);
	lcm_data(0x02);
	lcm_data(0x08);
	lcm_data(0x0e);
	lcm_data(0x18);
	lcm_data(0x1f);
	lcm_data(0x28);
	lcm_data(0x31);
	lcm_data(0x39);
	lcm_data(0x41);
	lcm_data(0x4a);
	lcm_data(0x52);
	lcm_data(0x5a);
	lcm_data(0x63);
	lcm_data(0x6a);
	lcm_data(0x72);
	lcm_data(0x7a);
	lcm_data(0x81);
	lcm_data(0x89);
	lcm_data(0x92);
	lcm_data(0x9b);
	lcm_data(0xa2);
	lcm_data(0xaa);
	lcm_data(0xb2);
	lcm_data(0xb9);
	lcm_data(0xc1);
	lcm_data(0xc9);
	lcm_data(0xd0);
	lcm_data(0xd8);
	lcm_data(0xe1);
	lcm_data(0xe7);
	lcm_data(0xef);
	lcm_data(0xf8);
	lcm_data(0xff);
	lcm_data(0x79);
	lcm_data(0xb4);
	lcm_data(0xeb);
	lcm_data(0xb5);
	lcm_data(0xd0);
	lcm_data(0x4f);
	lcm_data(0x38);
	lcm_data(0x28);
	lcm_data(0xc0);
	lcm_data(0x02);
	lcm_data(0x08);
	lcm_data(0x0e);
	lcm_data(0x18);
	lcm_data(0x1f);
	lcm_data(0x28);
	lcm_data(0x31);
	lcm_data(0x39);
	lcm_data(0x41);
	lcm_data(0x4a);
	lcm_data(0x52);
	lcm_data(0x5a);
	lcm_data(0x63);
	lcm_data(0x6a);
	lcm_data(0x72);
	lcm_data(0x7a);
	lcm_data(0x81);
	lcm_data(0x89);
	lcm_data(0x92);
	lcm_data(0x9b);
	lcm_data(0xa2);
	lcm_data(0xaa);
	lcm_data(0xb2);
	lcm_data(0xb9);
	lcm_data(0xc1);
	lcm_data(0xc9);
	lcm_data(0xd0);
	lcm_data(0xd8);
	lcm_data(0xe1);
	lcm_data(0xe7);
	lcm_data(0xef);
	lcm_data(0xf8);
	lcm_data(0xff);
	lcm_data(0x79);
	lcm_data(0xb4);
	lcm_data(0xeb);
	lcm_data(0xb5);
	lcm_data(0xd0);
	lcm_data(0x4f);
	lcm_data(0x38);
	lcm_data(0x28);
	lcm_data(0xc0);

	lcm_cmd(0xc2);
	lcm_data(0x02);

	lcm_cmd(0x3a);
	lcm_data(0x55);

	lcm_cmd(0x36);
	lcm_data(0x01);

//	lcm_cmd( 0xCC);
//	lcm_data( 0x03);

	lcm_cmd(0x21);
	mdelay(10);

	lcm_cmd(0x11); //Exit Sleep
	mdelay(20);

	lcm_cmd(0x29); //display on
	mdelay(10);

	/* Set display region */
	lcm_cmd(0x2a);
	lcm_data(lt_x >> 8);
	lcm_data(lt_x & 0xff);
	lcm_data(rb_x >> 8);
	lcm_data(rb_x & 0xff);

	lcm_cmd(0x2b);
	lcm_data(lt_y >> 8);
	lcm_data(lt_y & 0xff);
	lcm_data(rb_y >> 8);
	lcm_data(rb_y & 0xff);

	lcm_cmd(0x2c);
}

static void lcm_sleep(PANEL_CMD lcm_cmd, PANEL_DATA lcm_data)
{
	printf("%s()\n",__FUNCTION__);

	lcm_cmd(0x28);	// Display OFF
	mdelay(10);

	lcm_cmd(0x10);	// Sleep IN
}

static void lcm_awake(PANEL_CMD lcm_cmd, PANEL_DATA lcm_data)
{
	unsigned short lt_x = 0;
	unsigned short lt_y = 0;
	unsigned short rb_x = lcm.panel_attribute.xres - 1;
	unsigned short rb_y = lcm.panel_attribute.yres - 1;

	printf("%s()\n",__FUNCTION__);

	lcm_cmd(0x11);	// Sleep OUT
	mdelay(120);

	lcm_cmd(0x29);	//Display ON
	mdelay(10);

	/* Set display region */
	lcm_cmd(0x2A);
	lcm_data(lt_x >> 8);
	lcm_data(lt_x & 0xff);
	lcm_data(rb_x >> 8);
	lcm_data(rb_x & 0xff);

	lcm_cmd(0x2B);
	lcm_data(lt_y >> 8);
	lcm_data(lt_y & 0xff);
	lcm_data(rb_y >> 8);
	lcm_data(rb_y & 0xff);

	// Write memory start
	lcm_cmd(0x2C);
}

/* Struct for panel registration */
static const hippo_lcm_panel_t lcm = {
	.panel_attribute = {
		.if_type	= IF_I80,
		.pixel_fmt	= RGB565,
		.xres		= 480,
		.yres		= 800,
	},

	.init		= lcm_init,
	.sleep		= lcm_sleep,
	.awake		= lcm_awake,

	.pif_id		= 0,	/* <0 means system should probe which IF plugged panel uses firstly */

	.rplw		= 355, //45,	/* in ns */
	.rphw		= 90,		/* in ns */
	.wplw		= 25,		/*  in ns */
	.wphw		= 25,		/*  in ns */ /* wplw + wphw >= 50 ns */

	.pif_width	= _16BIT,
	.pif_endian	= ALIGN_LSB,
};

/* Panel entry function */
void panel_main_entry(void)
{
	printf("HX8363B  %s()\n",__FUNCTION__);
	register_panel(&lcm);
}
