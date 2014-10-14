#ifndef HIPPO_DISPLAY_H
#define HIPPO_DISPLAY_H

/***** Internal Used Prototypes *****/
/* PIF1_CTL, PIF2_CTL */
typedef union
{
	struct {
		unsigned int rplw:5;
		unsigned int rphw:5;
		unsigned int wplw:5;
		unsigned int wphw:5;
		unsigned int internal_dout_enable_delay:2;
		unsigned int pif_width:1;
		unsigned int short_in_msb:1;
		unsigned int _reserved0:8;
	} reg_field;

	unsigned int reg_value;
} pif_ctl_reg;

/* RGB_IF_CTRL */
typedef union
{
	struct {
		unsigned int pixel_clk_divider:6;
		unsigned int _reserved0:2;
		unsigned int pixel_clk_polarity:1;
		unsigned int vsync_polarity:1;
		unsigned int hsync_polarity:1;
		unsigned int _reserved1:21;
	} reg_field;
	
	unsigned int reg_value;
} rgb_if_ctl_reg;

/* RGB_VSYNC_CTL, RGB_HSYNC_CTL */
typedef union
{
	struct {
		unsigned int spw:8;
		unsigned int fpd:8;
		unsigned int bpd:8;
		unsigned int _reserved0:8;
	} reg_field;
	
	unsigned int reg_value;
} rgb_sync_ctl_reg;

/* SPI Control */
typedef union
{
	struct {
		unsigned int sc_divisor:6;
		unsigned int _reserved0:2;
		unsigned int clk_polarity:1;
		unsigned int clk_phase:1;
		unsigned int _reserved1:22;
	} reg_field;

	unsigned int reg_value;
} spi_ctl_reg;

/***** Internal Used Prototypes *****/

enum _IF_TYPE{
	IF_I80,
	IF_RGB,
};

enum _PIXEL_FMT{
	RGB565 = 0,
	RGB666,
	RGB888,
	ARGB8888,
	PIXEL_FMT_CNT
};

struct _panel_attribute{
	enum _IF_TYPE	if_type;
	enum _PIXEL_FMT	pixel_fmt;
	unsigned short	xres;
	unsigned short	yres;
};

typedef struct _panel_attribute hippo_panel_attribute_t;

/* Panel control functions */
typedef void (*PANEL_CMD)	(int cmd);
typedef void (*PANEL_DATA)	(int data);

/* LCD Panel */
struct _lcd_panel {
	/* Basic panel attribution */
	hippo_panel_attribute_t panel_attribute;

	/* Callback Functions */
	void (*init)	(PANEL_CMD lcd_cmd, PANEL_DATA lcd_data);
	void (*sleep)	(PANEL_CMD lcd_cmd, PANEL_DATA lcd_data);
	void (*awake)	(PANEL_CMD lcd_cmd, PANEL_DATA lcd_data);

	/* RGB IF attributes */
	int dclk;		/* in ns */
	enum _PICO_POLAR {
		FALLING = 0,
		RISING
	} pico_polarity;
	int hspw;		/* in dclks */
	int hfpd;		/* in dclks */
	int hbpd;		/* in dclks */
	int vspw;		/* in lines */
	int vfpd;		/* in lines */
	int vbpd;		/* in lines */
	enum _SYNC_POLAR {
		ACTIVE_LOW,
		ACTIVE_HIGH
	} hsync_polarity, vsync_polarity;

	/* SPI attributes */
	int sck;		/* in ns */
	int cpol;		/* default is 0 */
	int cpha;		/* default is 0 */
	int sdw;		/* serial data width, default = 9 */
};
typedef struct _lcd_panel hippo_lcd_panel_t ;

/* LCM Panel */
struct _lcm_panel {
	/* Basic panel attribution */
	hippo_panel_attribute_t panel_attribute;

	/* Callback Functions */
	void (*init)	(PANEL_CMD lcm_cmd, PANEL_DATA lcm_data);
	void (*sleep)	(PANEL_CMD lcm_cmd, PANEL_DATA lcm_data);
	void (*awake)	(PANEL_CMD lcm_cmd, PANEL_DATA lcm_data);

	/* PIF setting */
	int pif_id;	/* We have 2 PIFs for connecting LCM panels, 0 is for PIF1, 1 is for PIF2 */
	int rplw;	/* in ns */
	int rphw;	/* in ns */
	int wplw;	/* in ns */
	int wphw;	/* in ns */

	enum _PIF_WIDTH {
		_16BIT,
		_18BIT
	} pif_width;

	enum _PIF_ENDIAN {
		ALIGN_LSB,
		ALIGN_MSB
	} pif_endian;
};
typedef struct _lcm_panel hippo_lcm_panel_t;

#define ACQUIRE_PANEL_IFTYPE(arg)	(((hippo_panel_attribute_t*)arg)->if_type)
#define ACQUIRE_PANEL_PIXFMT(arg)	(((hippo_panel_attribute_t*)arg)->pixel_fmt)
#define ACQUIRE_PANEL_XRES(arg)		(((hippo_panel_attribute_t*)arg)->xres)
#define ACQUIRE_PANEL_YRES(arg)		(((hippo_panel_attribute_t*)arg)->yres)

/* Calls for internal operations */
extern void register_panel(const void *panel_cfg);
extern void lcd_set_bt_count(int count);
extern void lcd_resume_spi(void);
extern void lcd_pause_spi(void);

#endif//HIPPO_DISPLAY_H
