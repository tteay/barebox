/*
 * Platform data definitions for Infomax im98xx gadget driver.
 */
#ifndef __LINUX_USB_IM98XX_UDC_H
#define __LINUX_USB_IM98XX_UDC_H

enum ep_trsactn_num_per_frm {
	EP_TRSACTION_ONE = 0,
	EP_TRSACTION_TWO,
	EP_TRSACTION_THREE,
	EP_TRSACTION_INVLID,
};

enum ep_in_mod {
	EP_IN_MODE_AUTO = 0,
	EP_IN_MODE_MANU,
	EP_IN_MODE_FLY,
	EP_IN_MODE_RESERVED,
};

struct im98xx_ep_data {
	char	*name;
	int	addr;
	int mps;
	int	can_dma;
	int	type;
	int fifo_start;
	int	fifo_size;
	enum ep_trsactn_num_per_frm ep_trsactn_num;
	enum ep_in_mod	ep_in_mod;
};

struct im98xx_udc_platform_data {
	int			vbus_pin;
	int			num_ep;
	unsigned	usb_cable_gpio;/* The GPIO number used to detect USB */
	struct im98xx_ep_data	*ep;
};

/* ep0 + ep1 + ep2+ .. + ep6 */
#define NR_IM98XX_UDC_ENDPOINTS (1 + 6)	

struct im98xx_udc_data {
	struct im98xx_udc_platform_data pdata;
	struct im98xx_ep_data ep[NR_IM98XX_UDC_ENDPOINTS];
};

#define EP(nam, _addr, _mps, _fifo_start, _fifosize, dma, _type, trsactn_num, _ep_in_mod) \
	[_addr] = {						\
		.name		= nam,				\
		.addr		= _addr,				\
		.mps		= _mps,				\
		.fifo_start	= _fifo_start,		\
		.fifo_size	= _fifosize,			\
		.can_dma	= dma,				\
		.type	= _type,				\
		.ep_trsactn_num = trsactn_num, \
		.ep_in_mod = _ep_in_mod,			\
	}

#endif /* __LINUX_USB_IM98XX_UDC_H */

