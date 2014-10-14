#include <stdio.h>
#include <common.h>
#include <mach/hippo_display.h>
#include <mach/hippo_showbat.h>

static void lcm_init(PANEL_CMD lcm_cmd, PANEL_DATA lcm_data)
{
	int i, j;

	// Power off sequence
	lcm_cmd(0xD0) ;
	lcm_data(0x0) ;
	lcm_data(0x0) ;
	lcm_data(0x0) ;
	lcm_cmd(0xD1) ;
	lcm_data(0x0) ;
	lcm_data(0x0) ;
	lcm_data(0x0) ;
	lcm_cmd(0xD2) ;
	lcm_data(0x0) ;
	lcm_data(0x0) ;
	mdelay(20) ;

	// Exit sleep mode
	lcm_cmd(0x11) ;
	mdelay(20) ;

	// Power setting
	lcm_cmd(0xD0) ;
	lcm_data(0x07) ;
	lcm_data(0x43) ;
	lcm_data(0x1D) ;
	lcm_cmd(0xD1) ;
	lcm_data(0x00) ;
	lcm_data(0x1B) ;
	lcm_data(0x12) ;
	lcm_cmd(0xD2) ;
	lcm_data(0x01) ;
	lcm_data(0x01) ;

	// Panel driving setting
	lcm_cmd(0xC0) ;
	lcm_data(0x10) ;
	lcm_data(0x3B) ;
	lcm_data(0x00) ;
	lcm_data(0x12) ;
	lcm_data(0x01) ;
	lcm_cmd(0xC1) ;
	lcm_data(0x10) ;
	lcm_data(0x13) ;
	lcm_data(0x88) ;
	lcm_cmd(0xC5) ;
	lcm_data(0x02) ;

	// Gamma setting
	lcm_cmd(0xC8) ;	
	lcm_data(0x02) ;
	lcm_data(0x46) ;
	lcm_data(0x14) ;
	lcm_data(0x31) ;
	lcm_data(0x0A) ;
	lcm_data(0x04) ;
	lcm_data(0x37) ;
	lcm_data(0x24) ;
	lcm_data(0x57) ;
	lcm_data(0x13) ;
	lcm_data(0x06) ;
	lcm_data(0x0C) ;
	
	// ???
	lcm_cmd(0xF3) ;
	lcm_data(0x40) ;
	lcm_data(0x0A) ;
	lcm_cmd(0xF6) ;
	lcm_data(0x80) ;
	lcm_cmd(0xF7) ;
	lcm_data(0x80) ;
	lcm_cmd(0x36) ;
	lcm_data(0x0A) ;
	mdelay(120) ;

	// Set pixel format 
	lcm_cmd(0x3A) ;
	lcm_data(0x66) ;  // 0x66 for RGB666, 0x55 for RGB565
	
	// Set display on
	lcm_cmd(0x29) ;

	/* Set fill region */
	lcm_cmd(0x2A) ;	
	lcm_data(0) ;
	lcm_data(0) ;
	lcm_data(0x1) ;
	lcm_data(0x3F) ;
 	
	lcm_cmd(0x2B) ;
	lcm_data(0) ;
	lcm_data(0) ;
	lcm_data(0x1) ;
	lcm_data(0xDF) ;
	mdelay(10) ;

	// Write memory start
	lcm_cmd(0x2C) ;
#if 0
	/* Fill Panel with BLACK */
	for (i=0;i<320;i++){
		for (j=0 ;j<480;j++){
			lcm_data(0x0) ;
		}
	}
	
	mdelay(500) ;
	/* Fill Panel with BLUE */
	for (i=0;i<320;i++){
		for (j=0 ;j<480;j++){
			lcm_data(0xFFFF) ;
		}
	}
#endif
	mdelay(10) ;
}

static int lcm_sleep(PANEL_CMD lcm_cmd, PANEL_DATA lcm_data)
{
	lcm_cmd(0x10) ;
	mdelay(20) ;
	lcm_cmd(0x28) ;
	mdelay(10) ;
	lcm_cmd(0xD0) ;
	lcm_data(0x0) ;
	lcm_data(0x0) ;
	lcm_data(0x0) ;
	lcm_cmd(0xD1) ;
	lcm_data(0x0) ;
	lcm_data(0x0) ;
	lcm_data(0x0) ;
	lcm_cmd(0xD2) ;
	lcm_data(0x0) ;
	lcm_data(0x0) ;
	mdelay(120) ;

	/* Enter deep standby mode */
	lcm_cmd(0xB1) ;
	lcm_data(0x1) ;
	mdelay(20) ;

	return 0 ;
}

static int lcm_awake(PANEL_CMD lcm_cmd, PANEL_DATA lcm_data)
{
	/* Exit deep standby mode */
	lcm_cmd(0xB1) ;
	lcm_data(0x0) ;
	mdelay(120) ;
	
	lcm_cmd(0x11) ;
	mdelay(20) ;
	lcm_cmd(0xD0) ;
	lcm_data(0x07) ;
	lcm_data(0x41) ;
	lcm_data(0x1D) ;
	lcm_cmd(0xD1) ;
	lcm_data(0x00) ;
	lcm_data(0x2B) ;
	lcm_data(0x1F) ;
	lcm_cmd(0xD2) ;
	lcm_data(0x01) ;
	lcm_data(0x11) ;
	
	/* Restore address mode */
	lcm_cmd(0x36) ;
	mdelay(1) ;
	lcm_data(0x0A) ;
	mdelay(120) ;
	
	lcm_cmd(0x29) ;

	/* Set fill region */
	lcm_cmd(0x2A) ;	
	lcm_data(0) ;
	lcm_data(0) ;
	lcm_data(0x1) ;
	lcm_data(0x3F) ;
 	
	lcm_cmd(0x2B) ;
	lcm_data(0) ;
	lcm_data(0) ;
	lcm_data(0x1) ;
	lcm_data(0xDF) ;
	mdelay(10) ;
	
	// Write memory start
	lcm_cmd(0x2C) ;

	return 0 ;
}

/* Struct for panel registration */
const hippo_lcm_panel_t lcm = {

	.panel_attribute = {
		.if_type 	= IF_I80,
		.pixel_fmt 	= RGB666,
		.xres		= 320,
		.yres		= 480,
	},

	.init		= lcm_init,
	.sleep		= lcm_sleep,
	.awake		= lcm_awake,

	.pif_id 	= 0,

	.rplw		= 170,		/* in ns */
	.rphw 		= 250,		/* in ns */
	.wplw 		= 45,		/* 25,(+ 20) in ns */
	.wphw 		= 55,		/* 30,(+ 25) in ns */ /* wplw + wphw >= 100 ns */
	
	.pif_width 	= _18BIT,
	.pif_endian 	= ALIGN_LSB,
};

/* Panel entry function */
void panel_main_entry(void)
{
	printf("%s\n",__FUNCTION__) ;
	register_panel(&lcm) ;
}
