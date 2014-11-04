/*
 * linux/drivers/usb/gadget/im98xx_udc.h
 * Infomax iM98xx on-chip High speed USB device controller
 *
 * Copyright (C) 2010 Infomax Communication CO., LTD
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307	 USA
 */

#ifndef __LINUX_USB_GADGET_IM98XX_UDC_H
#define __LINUX_USB_GADGET_IM98XX_UDC_H

#include <types.h>
#include <common.h>
#include <io.h>
//#include <usb/gadget.h>

/* VBUS parameters */
enum
{
  DEV_TYPE_VSYS = 0,
  DEV_TYPE_VIO18,
  DEV_TYPE_VIO28,
  DEV_TYPE_VUSB,
  DEV_TYPE_VTCXO,
  DEV_TYPE_VA  

};

#define VUSB_SEL_MASK   (0x03L<<8)
#define VUSB_SEL_3_2V   (0x02L<<8)
#define VUSB_SEL_3_3V   (0x03L<<8)

/* USB PLL parameters */
#define  USB_PLL_DIVR     (0)
#define  USB_PLL_DIVF     (11L << 8)
#define  USB_PLL_DIVQ     (0 << 16)
#define  USB_PLL_BS       (0  << 19)
#define  USB_PLL_FSEL     (0x1L<< 20)
#define  USB_PLL_PD       (0x1L << 24)

/**
 * Register definitions
 * Access Main Control Registers/USB Control Registers/
 * Control Endpoint Registers(ep0)/DMA Registers
 * by means of udc_readl/udc_writel, udc_readb/udc_writeb
 *
 * While access Non-Control Endpoint Registers Offsets
 * by udc_ep_readl/udc_ep_writel, udc_ep_readb/udc_ep_writeb
 */
/* Main Control Registers */
#define	IRQ_STAT_L		0x00		/* Interrupt Status Low Register */
#define	IRQ_STAT_H		0x04		/* Interrupt Status High Register */
#define	IRQ_ENB_L		0x08		/* Interrupt Enable Low Register */
#define	IRQ_ENB_H		0x0c		/* Interrupt Enable High Register */
/* USB Control Registers */
#define	USB_IRQ_STAT	0x10		/* USB Interrupt Status register */
#define	USB_IRQ_ENB		0x14		/* USB Interrupt Enable register */
#define	USB_OPER		0x18		/* USB operational register */
#define	USB_FRAME_CNT	0x1c		/* USB frame count register */
#define	USB_ADDR		0x20		/* USB address register */
#define	USB_TEST		0x24		/* USB test mode register */
/* Control Endpoint Registers */
#define	CEP_DATA_BUF	0x28		/* Control-ep Data Buffer */
#define	CEP_CTRL_STAT	0x2c		/* Control-ep Control and Status */
#define	CEP_IRQ_ENB		0x30		/* Control-ep Interrupt Enable */
#define	CEP_IRQ_STAT	0x34		/* Control-ep Interrupt Status */
#define	IN_TRNSFR_CNT	0x38		/* In-transfer data count */
#define	OUT_TRNSFR_CNT	0x3c		/* Out-transfer data count */
#define	CEP_DATA_AVAIL	0x40		/* Data count  available in CEP buffer */
#define	SETUP1_0		0x44		/* Setupbyte1 & byte0 */
#define	SETUP3_2		0x48		/* Setupbyte3 & byte2 */
#define	SETUP5_4		0x4c		/* Setupbyte5 & byte4 */
#define	SETUP7_6		0x50		/* Setupbyte7 & byte6 */
#define	CEP_START_ADDR	0x54		/* Control EP's RAM start address */
#define	CEP_END_ADDR	0x58		/* Control EP's RAM end address */
/* DMA Registers */
#define	DMA_CTRL_STS	0x5c		/* DMA control and status register */
#define	DMA_CNT			0x60		/* DMA count register */
#define	AHB_DMA_ADDR	0x0700		/* AHB_DMA address register */
#define	DMA_TIMEOUT_SOF	0x0258		/* DMA Timeout Limited SOFs(not in IP doc) */
#define DMA_REMAIN_BYTES	0x025c	/* DMA remained bytes(not in IP doc) */

/* Non-Control Endpoint Registers Offsets */
#define	EP_DATA_BUF			0		/* Endpoint data register */
#define	EP_IRQ_STAT			1		/* Endpoint Interrupt status register */
#define	EP_IRQ_ENB			2		/* Endpoint Interrupt enable register */
#define	EP_DATA_CNT			3		/* Data count available in endpoint buffer */
#define	EP_RSP_SC			4		/* Endpoint response register set/clear */
#define	EP_MPS				5		/* Endpoint maximum packet size register */
#define	EP_CNT				6		/* Endpoint transfer count register */
#define	EP_CFG				7		/* Endpoint configuration register */
#define	EP_START_ADDR		8		/* Endpoint's RAM start address */
#define	EP_END_ADDR			9		/* Endpoint's RAM end address */

/* Bitfields in IRQ_ENB_L/IRQ_STAT_L */
#define USB_INT_EN		(1 << 0)
#define CEP_INT_EN		(1 << 1)

#define EP1_INT_EN		(1 << 2)
#define EP2_INT_EN		(1 << 3)
#define EP3_INT_EN		(1 << 4)
#define EP4_INT_EN		(1 << 5)
#define EP5_INT_EN		(1 << 6)
#define EP6_INT_EN		(1 << 7)
#define MAIN_IRQ_EPS_INT	(EP1_INT_EN | EP2_INT_EN | EP3_INT_EN | \
							EP4_INT_EN | EP5_INT_EN | EP6_INT_EN)
#define MAIN_IRQ_INT_MASK	(USB_INT_EN | CEP_INT_EN | \
								EP1_INT_EN | EP2_INT_EN | EP3_INT_EN | \
								EP4_INT_EN | EP5_INT_EN | EP6_INT_EN)

#define EP_INT_EN(addr)	EP##addr##_INT_EN

/* Bitfields in USB_IRQ_ENB/USB_IRQ_STAT */
#define SOF_INT_EN			(1 << 0)
#define	RST_INT_EN			(1 << 1)
#define RSUM_INT_EN			(1 << 2)
#define SPNT_INT_EN			(1 << 3)
#define	HSS_INT_EN			(1 << 4)
#define DMA_CMPET_INT_EN 	(1 << 5)
#define	USEBLE_CLK_EN		(1 << 6)
#if 0
#define	USB_IRQ_INT_MASK	(SOF_INT_EN | RST_INT_EN | RSUM_INT_EN | \
								SPNT_INT_EN | HSS_INT_EN | DMA_CMPET_INT_EN | \
								USEBLE_CLK_EN)
#endif
#if 0 //no SPNT_INT_EN | SOF_INT_EN
#define	USB_IRQ_INT_MASK	(RST_INT_EN | RSUM_INT_EN | \
								HSS_INT_EN | DMA_CMPET_INT_EN | \
								USEBLE_CLK_EN)
#endif

#if 0 //no USEBLE_CLK_EN | SOF_INT_EN
#define	USB_IRQ_INT_MASK	(RSUM_INT_EN | RST_INT_EN | SPNT_INT_EN | \
								HSS_INT_EN | DMA_CMPET_INT_EN)
#endif

#if 1 //no USEBLE_CLK_EN | SOF_INT_EN  | DMA_CMPET_INT_EN
#define	USB_IRQ_INT_MASK	(RSUM_INT_EN | RST_INT_EN | SPNT_INT_EN | \
								HSS_INT_EN)
#endif


/* Bitfields in USB_OPER */
#define GEN_RSUM	(1 << 0)
#define USB_HSPED	(1 << 1)
#define	USB_CSPED	(1 << 2)
#define SOFT_CONN	(1 << 3)

/* Bitfields in CEP_CTRL_STAT */
#define NAK_CLR		(1 << 0)
#define STALL		(1 << 1)
#define	ZEROLEN		(1 << 2)
#define	CEP_FLUSH	(1 << 3)

/* Bitfields in CEP_IRQ_ENB/CEP_IRQ_STAT */
#define	CEP_SETUP_TOKEN_INT_EN		(1 << 0)
#define	CEP_SETUP_PKT_INT_EN		(1 << 1)
#define	CEP_OUT_TOKEN_INT_EN		(1 << 2)
#define	CEP_IN_TOKEN_INT_EN			(1 << 3)
#define	CEP_PING_TOKEN_INT_EN		(1 << 4)
#define	CEP_DATAPKT_TRSMIT_INT_EN	(1 << 5)
#define	CEP_DATAPKT_RECIVE_INT_EN	(1 << 6)
#define	CEP_NAK						(1 << 7)
#define	CEP_STALL					(1 << 8)
#define	CEP_USB_ERR					(1 << 9)
#define	CEP_STAUS_COMPET_INT_EN		(1 << 10)
#define	CEP_BUF_FULL_INT_EN			(1 << 11)
#define	CEP_BUF_EMPT_INT_EN			(1 << 12)
#define	CEP_DMA_TIMEOUT_INT_EN		(1 << 13)
#if 0  // no CEP_BUF_FULL_INT_EN | CEP_BUF_EMPT_INT_EN
#define	CEP_IRQ_MASK		(CEP_SETUP_TOKEN_INT_EN | CEP_SETUP_PKT_INT_EN | \
						CEP_OUT_TOKEN_INT_EN | CEP_IN_TOKEN_INT_EN | \
						CEP_PING_TOKEN_INT_EN | CEP_DATAPKT_TRSMIT_INT_EN | \
						CEP_DATAPKT_RECIVE_INT_EN | CEP_NAK | CEP_STALL | \
						CEP_USB_ERR | CEP_STAUS_COMPET_INT_EN | \
						CEP_DMA_TIMEOUT_INT_EN) 
#endif
#if 0 // all interrupts
#define	CEP_IRQ_MASK		(CEP_SETUP_TOKEN_INT_EN | CEP_SETUP_PKT_INT_EN | \
						CEP_OUT_TOKEN_INT_EN | CEP_IN_TOKEN_INT_EN | \
						CEP_PING_TOKEN_INT_EN | CEP_DATAPKT_TRSMIT_INT_EN | \
						CEP_DATAPKT_RECIVE_INT_EN | CEP_NAK | CEP_STALL | \
						CEP_USB_ERR | CEP_STAUS_COMPET_INT_EN | \
						CEP_BUF_FULL_INT_EN | CEP_BUF_EMPT_INT_EN | \
						CEP_DMA_TIMEOUT_INT_EN) 
#endif
#if 0 // no CEP_SETUP_TOKEN_INT_EN
#define	CEP_IRQ_MASK		(CEP_SETUP_PKT_INT_EN | \
						CEP_OUT_TOKEN_INT_EN | CEP_IN_TOKEN_INT_EN | \
						CEP_PING_TOKEN_INT_EN | CEP_DATAPKT_TRSMIT_INT_EN | \
						CEP_DATAPKT_RECIVE_INT_EN | CEP_NAK | CEP_STALL | \
						CEP_USB_ERR | CEP_STAUS_COMPET_INT_EN | \
						CEP_BUF_FULL_INT_EN | CEP_BUF_EMPT_INT_EN | \
						CEP_DMA_TIMEOUT_INT_EN) 
#endif

#if 0 // no CEP_SETUP_TOKEN_INT_EN | CEP_BUF_EMPT_INT_EN
#define	CEP_IRQ_MASK		(CEP_SETUP_PKT_INT_EN | \
						CEP_OUT_TOKEN_INT_EN | CEP_IN_TOKEN_INT_EN | \
						CEP_PING_TOKEN_INT_EN | CEP_DATAPKT_TRSMIT_INT_EN | \
						CEP_DATAPKT_RECIVE_INT_EN | CEP_NAK | CEP_STALL | \
						CEP_USB_ERR | CEP_STAUS_COMPET_INT_EN | \
						CEP_BUF_FULL_INT_EN | \
						CEP_DMA_TIMEOUT_INT_EN) 
#endif
#if 0 // no CEP_SETUP_TOKEN_INT_EN | CEP_BUF_EMPT_INT_EN | no token interrupt
#define	CEP_IRQ_MASK		(CEP_SETUP_PKT_INT_EN | \
						CEP_DATAPKT_TRSMIT_INT_EN | \
						CEP_DATAPKT_RECIVE_INT_EN | CEP_STALL | \
						CEP_USB_ERR | CEP_STAUS_COMPET_INT_EN | \
						CEP_BUF_FULL_INT_EN | \
						CEP_DMA_TIMEOUT_INT_EN) 
#endif

#if 1 //no token interrupt,  no CEP_BUF_EMPT/FULL_INT_EN | CEP_DATAPKT_RECIVE_INT_EN
#define	CEP_IRQ_MASK		(CEP_SETUP_PKT_INT_EN | \
						CEP_DATAPKT_TRSMIT_INT_EN | \
						CEP_STALL | \
						CEP_USB_ERR | CEP_STAUS_COMPET_INT_EN | \
						CEP_DMA_TIMEOUT_INT_EN)
#endif

/* Bitfields in DMA_CTRL_STS */
#define	DMA_READ	(1 << 4)
#define DMA_EN		(1 << 5)
#define DMA_ABORT	(1 << 6)

/* Bitfields in EP_IRQ_ENB/EP_IRQ_STAT */
#define	EP_BUF_FULL_INT_EN				(1 << 0)
#define	EP_BUF_EMPT_INT_EN				(1 << 1)
#define	EP_SHORTPKT_TRANSFERED_INT_EN	(1 << 2)
#define	EP_DATAPKT_TRANSMIT_INT_EN		(1 << 3)
#define	EP_DATAPKT_RECEIVED_INT_EN		(1 << 4)
#define	EP_DATAOUT_TOKEN_INT_EN			(1 << 5)
#define	EP_DATAIN_TOKEN_INT_EN			(1 << 6)
#define	EP_PING_TOKEN_INT_EN			(1 << 7)
#define	EP_USB_NAKSENT_INT_EN			(1 << 8)
#define	EP_USB_STALLSENT_INT_EN			(1 << 9)
#define	EP_NYET_INT_EN					(1 << 10)
#define	EP_ERR_INT_EN					(1 << 11)
#if 0
#define	EP_IRQ_MASK		(EP_BUF_FULL_INT_EN | EP_BUF_EMPT_INT_EN | \
				EP_SHORTPKT_TRANSFERED_INT_EN | EP_DATAPKT_TRANSMIT_INT_EN | \
				EP_DATAPKT_RECEIVED_INT_EN | EP_DATAOUT_TOKEN_INT_EN | \
				EP_DATAIN_TOKEN_INT_EN | EP_PING_TOKEN_INT_EN | \
				EP_USB_NAKSENT_INT_EN | EP_USB_STALLSENT_INT_EN | \
				EP_NYET_INT_EN | EP_ERR_INT_EN)
#endif				
#if 0 //no token interrupts
#define	EP_IRQ_MASK		(EP_BUF_FULL_INT_EN | EP_BUF_EMPT_INT_EN | \
				EP_SHORTPKT_TRANSFERED_INT_EN | EP_DATAPKT_TRANSMIT_INT_EN | \
				EP_DATAPKT_RECEIVED_INT_EN | \
				EP_USB_STALLSENT_INT_EN | \
				EP_ERR_INT_EN)
#endif
#if 0//no token interrupts /no EP_BUF_EMPT_INT_EN
#define	EP_IRQ_MASK		(EP_BUF_FULL_INT_EN | \
				EP_SHORTPKT_TRANSFERED_INT_EN | EP_DATAPKT_TRANSMIT_INT_EN | \
				EP_DATAPKT_RECEIVED_INT_EN | \
				EP_USB_STALLSENT_INT_EN | \
				EP_ERR_INT_EN)
#endif
#if 0//no token interrupts /no EP_BUF_EMPT_INT_EN /EP_BUF_FULL_INT_EN /EP_USB_STALLSENT_INT_EN
#define	EP_IRQ_MASK		(EP_SHORTPKT_TRANSFERED_INT_EN | EP_DATAPKT_TRANSMIT_INT_EN | \
				EP_DATAPKT_RECEIVED_INT_EN | \
				EP_ERR_INT_EN)
#endif
#if 0//no token interrupts /no EP_BUF_EMPT_INT_EN /EP_BUF_FULL_INT_EN /EP_USB_STALLSENT_INT_EN/EP_SHORTPKT_TRANSFERED_INT_EN/EP_ERR_INT_EN
#define	EP_IRQ_MASK		(EP_DATAPKT_TRANSMIT_INT_EN | EP_DATAPKT_RECEIVED_INT_EN | \
				EP_USB_NAKSENT_INT_EN | EP_NYET_INT_EN)
#endif

#if 0
#define	EP_IRQ_MASK		(EP_DATAPKT_TRANSMIT_INT_EN | EP_DATAPKT_RECEIVED_INT_EN | \
				EP_DATAIN_TOKEN_INT_EN | EP_USB_STALLSENT_INT_EN | EP_ERR_INT_EN)
#endif

#if 0
#define	EP_IRQ_MASK		(EP_DATAPKT_TRANSMIT_INT_EN | EP_DATAPKT_RECEIVED_INT_EN | \
				EP_DATAIN_TOKEN_INT_EN)
#endif

#if 1
#define	EP_IN_IRQ_MASK		EP_DATAPKT_TRANSMIT_INT_EN
#define	EP_OUT_IRQ_MASK		EP_DATAPKT_RECEIVED_INT_EN
#endif

#if 0
#define	EP_IRQ_MASK		(EP_DATAIN_TOKEN_INT_EN | EP_DATAPKT_RECEIVED_INT_EN)
#endif

/* Bitfields in EP_RSP_SC */
#define	EP_BUF_FLUSH		(1 << 0)
#define	EP_IN_MOD_SHIFT		1
//#define	EP_IN_MOD_MASK		(3 << 1)
//#define	EP_IN_MOD_AUTO		(0 << 1)
//#define	EP_IN_MOD_MAN		(1 << 1)
//#define	EP_IN_MOD_FLY		(2 << 1)
#define	EP_TOGGLE			(1 << 3)
#define	EP_HALT				(1 << 4)
#define	EP_IN_ZEROLEN		(1 << 5)
#define EP_PKTEND			(1 << 6)

/* Bitfields in EP_CFG */
#define	EP_VALID			(1 << 0)

#define	EP_TYPE_MASK		(3 << 1)
#define EP_TYPE_SHIFT		1
#define	EP_TYPE_BULK		(1 << 1)
#define	EP_TYPE_INT			(2 << 1)
#define	EP_TYPE_ISO			(3 << 1)

#define	EP_DIR_IN			(1 << 3)

#define	EP_ADDR_MASK		(0x0f << 4)
#define EP_ADDR_SHIFT		4

#define	EP_MULT_MASK		(3 << 8)
#define EP_MULT_SHIFT		8

//#define	EP_MULT_ONE_TRANS	(0 << 8)
//#define	EP_MULT_TWO_TRANS	(1 << 8)
//#define	EP_MULT_THREE_TRANS	(2 << 8)
#define	EP_MULT_INVALID		(3 << 8)

#define	EP_MPKTS				(1024)

/**
 * Register access macros
 * "10" denotes the count of regs for each non-control register
 * "2" denotes multiply of 4 
 */
#define EP1_OFF			0x64	/* The offset of EP1 */
#define EP_OFF(addr, reg)		(EP1_OFF + (10 << 2)*((addr) - 1) + (reg << 2)) 

#define udc_ep_readl(ep, reg)	\
	__raw_readl((ep)->udc->regs + EP_OFF((ep)->addr, reg))
#define udc_ep_writel(ep, reg, value)	\
	__raw_writel((value), (ep)->udc->regs + EP_OFF((ep)->addr, reg))
#define udc_ep_readw(ep, reg)	\
	__raw_readw((ep)->udc->regs + EP_OFF((ep)->addr, reg))
#define udc_ep_writew(ep, reg, value)	\
	__raw_writew((value), (ep)->udc->regs + EP_OFF((ep)->addr, reg))	
#define udc_ep_readb(ep, reg)	\
	__raw_readb((ep)->udc->regs + EP_OFF((ep)->addr, reg))
#define udc_ep_writeb(ep, reg, value)	\
	__raw_writeb((value), (ep)->udc->regs + EP_OFF((ep)->addr, reg))
	
#define udc_readl(udc, reg)	\
	__raw_readl((udc)->regs + (reg))
#define udc_writel(udc, reg, value)	\
	__raw_writel((value), (udc)->regs + (reg))
#define udc_readw(udc, reg)	\
	__raw_readw((udc)->regs + (reg))
#define udc_writew(udc, reg, value)	\
	__raw_writew((value), (udc)->regs + (reg))	
#define udc_readb(udc, reg)	\
	__raw_readb((udc)->regs + (reg))
#define udc_writeb(udc, reg, value)	\
	__raw_writeb((value), (udc)->regs + (reg))

#define EPADDR(ep)	(ep->addr)
#define EPXFERTYPE(ep)	(ep->type)
#define EPNAME(ep)	(ep->name)
//#define is_ep0(ep)	(!strcmp(EPNAME(ep), "ep0"))
#define is_ep0(ep)	(!ep->addr)

#define EPXFERTYPE_is_ISO(ep) (EPXFERTYPE(ep) == USB_ENDPOINT_XFER_ISOC)
#define EPXFERTYPE_is_BULK(ep) (EPXFERTYPE(ep) == USB_ENDPOINT_XFER_BULK)

struct stats {
	unsigned long in_ops;
	unsigned long out_ops;
	unsigned long in_bytes;
	unsigned long out_bytes;
	unsigned long irqs;
};

/**
 * struct im98xx_ep - im98xx endpoint
 * @dev: udc device
 * @queue: requests queue
 * @lock: lock to im98xx_ep data (queues and stats)
 * @enabled: true when endpoint enabled (not stopped by gadget layer)
 * @name: endpoint name (for trace/debug purpose)
 * @dir_in: 1 if IN endpoint, 0 if OUT endpoint
 * @addr: usb endpoint number
 * @config: configuration in which this endpoint is active
 * @interface: interface in which this endpoint is active
 * @alternate: altsetting in which this endpoitn is active
 * @fifo_size: max packet size in the endpoint fifo
 * @type: endpoint type (bulk, iso, int, ...)
 * @udccsr_value: save register of UDCCSR0 for suspend/resume
 * @udccr_value: save register of UDCCR for suspend/resume
 * @stats: endpoint statistics
 *
 */
struct im98xx_ep {
	struct usb_ep usb_ep;

	struct im98xx_udc		*udc;

	struct list_head	queue; //im98xx_request list
		const struct usb_endpoint_descriptor	*desc;
		
						/* (queues, stats) */
	unsigned		enabled:1;

	char			*name;

	/*
	 * Specific im98xx endpoint data, needed for hardware initialization
	 */
	unsigned		can_dma:1;
	unsigned		valid:1;
	unsigned		dir_in:1;
	unsigned		addr:3;
	unsigned		mult:2;
	
	unsigned		config:2;
	unsigned		interface:3;
	unsigned		alternate:3;
	
	unsigned		type;
	
	enum ep_in_mod	ep_in_mod;
	enum ep_trsactn_num_per_frm	ep_trsactn_num;
	u32				mps;
	
	u16				ep_start_addr;
	u16				ep_end_addr;
	unsigned		fifo_size;

	unsigned 		is_last_shortpkt:1;
	unsigned 		is_last_pkt:1;
	unsigned		is_full:1;
	unsigned 		is_empty:1;
	unsigned				cep_datapkt_transmitted:1;
	unsigned				cep_datapkt_received:1;
	unsigned				cep_status_completion:1;
	unsigned				cep_buf_full:1;
	unsigned				cep_buf_empty:1;
	unsigned				cep_dma_timeout:1;
	
	struct stats		stats;
};

/**
 * struct im98xx_request - container of each usb_request structure
 * @req: usb request
 * @udc_usb_ep: usb endpoint the request was submitted on
 * @in_use: sanity check if request already queued on an im98xx_ep
 * @queue: linked list of requests, linked on im98xx_ep->queue
 */
struct im98xx_request {
	struct usb_request			req;
	unsigned				in_use:1;
	struct list_head			queue;

	unsigned int				submitted:1;
	unsigned int				last_transaction:1;
	unsigned int				using_dma:1;
	unsigned int				mapped:1;
};

enum ep0_state {
	WAIT_FOR_SETUP = 0,
	SETUP_STAGE,
	IN_DATA_STAGE,
	OUT_DATA_STAGE,
	IN_STATUS_STAGE,
	OUT_STATUS_STAGE,
	UDC_STALL,
	WAIT_ACK_SET_CONF_INTERF,
};

static char *ep0_state_name[] = {
	"WAIT_FOR_SETUP", "SETUP_STAGE", "IN_DATA_STAGE", "OUT_DATA_STAGE",
	"IN_STATUS_STAGE", "OUT_STATUS_STAGE", "STALL",
	"WAIT_ACK_SET_CONF_INTERF"
};
#define EP0_STNAME(udc) ep0_state_name[(udc)->ep0state]

#define EP0_FIFO_SIZE	64U
//#define EP0_FIFO_SIZE	256U
#define BULK_FIFO_SIZE	512U
#define ISO_FIFO_SIZE	256U
#define INT_FIFO_SIZE	16U

struct udc_stats {
	unsigned long	irqs_reset;
	unsigned long	irqs_suspend;
	unsigned long	irqs_resume;
	unsigned long	irqs_reconfig;
};

#define NR_IM98XX_UDC_ENDPOINTS (1 + 6)	/* ep0 + ep1 + ep2+ .. + ep6 */

/**
 * struct im98xx_udc - udc structure
 * @regs: mapped IO space
 * @irq: udc irq
 * @clk: udc clock
 * @usb_gadget: udc gadget structure
 * @driver: bound gadget (zero, g_ether, g_file_storage, ...)
 * @dev: device
 * @mach: machine info, used to activate specific GPIO
 * @ep0state: control endpoint state machine state
 * @stats: statistics on udc usage
 * @udc_usb_ep: array of usb endpoints offered by the gadget
 * @im98xx_ep: array of im98xx available endpoints
 * @config: UDC active configuration
 * @last_interface: UDC interface of the last SET_INTERFACE host request
 * @last_alternate: UDC altsetting of the last SET_INTERFACE host request
 * @udccsr0: save of udccsr0 in case of suspend
 * @debugfs_root: root entry of debug filesystem
 * @debugfs_state: debugfs entry for "udcstate"
 * @debugfs_queues: debugfs entry for "queues"
 * @debugfs_eps: debugfs entry for "epstate"
 */

struct im98xx_udc {
	void __iomem	*regs;
	int				irq;
	struct clk		*clk;
	
	struct usb_gadget			gadget;
	struct usb_gadget_driver	*driver;
	struct device_d				*dev;
	struct im98xx_udc_data *udc_data;

	enum ep0_state			ep0state;
	struct udc_stats 		stats;
	struct usb_request	*ep0_req;	/* for internal request */
	__le16			ep0_data;	/* for internal request */

	struct im98xx_ep im98xx_ep[NR_IM98XX_UDC_ENDPOINTS];
	unsigned				enabled:1;
	unsigned				pullup_on:1;
	unsigned				pullup_resume:1;
	unsigned				vbus_sensed:1;
	unsigned				usb_cable_gpio; /* The GPIO number used to detect USB */
	unsigned				usb_cable_sensed:1; /* indicated the current cable status */
	u16				        devstatus;

	unsigned config:2;
	unsigned last_interface:3;
	unsigned last_alternate:3;
};

static inline struct im98xx_ep *to_im98xx_ep(struct usb_ep *ep)
{
	return container_of(ep, struct im98xx_ep, usb_ep);
}

static inline struct im98xx_request *to_im98xx_req(struct usb_request *req)
{
	return container_of(req, struct im98xx_request, req);
}

static inline struct im98xx_udc *to_im98xx_udc(struct usb_gadget *gadget)
{
	return container_of(gadget, struct im98xx_udc, gadget);
}

/*
 * REVISIT: Try to eliminate this value. Can we rely on req->mapped to
 * provide this information?
 */
#define DMA_ADDR_INVALID (~(dma_addr_t)0)

/*
 * Debugging/message support
 */
//#define DEBUG
//#define VERBOSE_DEBUG
#define ep_dbg(ep, fmt, arg...) \
	dev_dbg(ep->udc->dev, "%s:%s: " fmt, EPNAME(ep), __func__, ## arg)
#define ep_vdbg(ep, fmt, arg...) \
	dev_vdbg(ep->udc->dev, "%s:%s: " fmt, EPNAME(ep), __func__, ## arg)
#define ep_err(ep, fmt, arg...) \
	dev_err(ep->udc->dev, "%s:%s: " fmt, EPNAME(ep), __func__, ## arg)
#define ep_info(ep, fmt, arg...) \
	dev_info(ep->udc->dev, "%s:%s: " fmt, EPNAME(ep), __func__, ## arg)
#define ep_warn(ep, fmt, arg...) \
	dev_warn(ep->udc->dev, "%s:%s:" fmt, EPNAME(ep), __func__, ## arg)
	
#endif /* __LINUX_USB_GADGET_IM98XX_UDC_H */
