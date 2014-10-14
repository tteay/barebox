#ifndef HIPPO_SHOWBAT_H
#define HIPPO_SHOWBAT_H

/* Image pixel format */
#define IMG_RGB565	0
#define IMG_RGB888	1
#define IMG_ARGB8888	2

/* Layers for battery display */
#define LAY_BTM		0
#define LAY_MID		1
#define LAY_TOP		2

/* Struct for keeping battery image information */
struct image_grp {
	void *base;
	unsigned short pixel_fmt;
	unsigned short which_layer;
	unsigned short xres;
	unsigned short yres;
	unsigned short xofft;
	unsigned short yofft;
} ;

/* This should be the "ONLY" panel at which we want to show battery */
extern void panel_main_entry(void);

/* APIs for showing battery */
extern void to_show_battery(struct image_grp *igrp);

#endif	/* HIPPO_SHOWBAT_H */
