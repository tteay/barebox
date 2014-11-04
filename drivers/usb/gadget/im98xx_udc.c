/*
 * Handles the Infomax iM98xx USB Device Controller (UDC)
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
//#define VERBOSE_DEBUG
#include <common.h>
#include <errno.h>
#include <clock.h>
#include <io.h>
#include <gpio.h>
#include <init.h>
#include <poller.h>

#include <usb/ch9.h>
#include <usb/gadget.h>

#include <mach/im98xx_udc.h>
#include <mach/gpio.h>


#include "im98xx_udc.h"
#include <mach/magic.h>
//#include <mach/gpio.h>


/*
 * This driver handles the USB Device Controller (UDC) in Intel's iM98xx
 * series processors.
 *
 * Such controller drivers work with a gadget driver.  The gadget driver
 * returns descriptors, implements configuration and data protocols used
 * by the host to interact with this device, and allocates endpoints to
 * the different protocol interfaces.  The controller driver virtualizes
 * usb hardware so that the gadget drivers will be more portable.
 *
 * This UDC hardware wants to implement a bit too much USB protocol. The
 * biggest issues are:  that the endpoints have to be set up before the
 * controller can be enabled (minor, and not uncommon); and each endpoint
 * can only have one configuration, interface and alternative interface
 * number (major, and very unusual). Once set up, these cannot be changed
 * without a controller reset.
 *
 * The workaround is to setup all combinations necessary for the gadgets which
 * will work with this driver. This is done in im98xx_udc structure, statically.
 * See im98xx_udc, udc_usb_ep versus im98xx_ep, and matching function find_im98xx_ep.
 * (You could modify this if needed.  Some drivers have a "fifo_mode" module
 * parameter to facilitate such changes.)
 *
 * The combinations have been tested with these gadgets :
 *  - zero gadget
 *  - file storage gadget
 *  - ether gadget
 *
 * The driver doesn't use DMA, only IO access and IRQ callbacks. No use is
 * made of UDC's double buffering either. USB "On-The-Go" is not implemented.
 *
 * All the requests are handled the same way :
 *  - the drivers tries to handle the request directly to the IO
 *  - if the IO fifo is not big enough, the remaining is send/received in
 *    interrupt handling.
 */

#define	DRIVER_VERSION	"2010-08-05"
#define	DRIVER_DESC	"Infomax iM98xx USB Device Controller driver"

static const char driver_name[] = "im98xx_udc";
static struct im98xx_udc *the_controller;

#define EP_WAIT_DELAY 10
#define EP_WAIT_RETRY 300000   //max: wait write 3s

#define EP0_WAIT_DELAY 10
#define EP0_WAIT_RETRY 20   //max: wait 200us
/* EP data receive/transmit return codes */
enum ep_rw_stat {
	EP_RW_OK = 0,     /* ok */
	EP_RW_SUSPEND,          /* device will enter suspend*/
	EP_RW_TIMEOUT,          /* write timeout */
};
typedef enum ep_rw_stat ep_rw_status;
//static DECLARE_WAIT_QUEUE_HEAD(in_waitq);


/*ep name is important in gadget, it should obey the convention of ep_match()*/
static const char *const ep_name[] = {
	/* everyone has ep0 */
	"ep0", 
	/* 6 configurable endpoints */
	"ep1",
	"ep2",
	"ep3",
	"ep4",
	"ep5",
	"ep6",
};
void print_all_registers(struct im98xx_udc *udc);

static void handle_ep(struct im98xx_ep *ep, u16 ep_irq_stat);
static void stop_activity(struct im98xx_udc *udc, struct usb_gadget_driver *driver);
static void clear_reqs(struct im98xx_udc *udc, int status);
void usb_device_reset(struct im98xx_udc *udc);


static struct delayed_work suspend_handler ;
static int ep_configured = 0;
static int udc_suspended = 0;



static void dplus_pullup(struct im98xx_udc *udc, int on);
static void ep0_nak_clr(struct im98xx_ep *ep);
static void usb_cable_status(struct im98xx_udc *udc);
static inline void irq_udc_suspend(struct im98xx_udc *udc, int timeout);

struct im98xx_ep *crnt_ep = NULL;

/**
 * pio_irq_enable - Enables irq generation for one endpoint
 * @ep: udc endpoint
 */
static void 
pio_irq_enable(struct im98xx_ep *ep)
{
	struct im98xx_udc *udc = ep->udc;
	u8 ep_addr = (u8)ep->addr;
	u16 irq_enb_l, usb_irq_enb, cep_irq_enb, ep_irq_enb;

	irq_enb_l = udc_readw(udc, IRQ_ENB_L);

	if(ep_addr == 0) {
		irq_enb_l |= CEP_INT_EN;

		cep_irq_enb = udc_readw(udc, CEP_IRQ_ENB);
		cep_irq_enb |= CEP_IRQ_MASK;
		udc_writew(udc, CEP_IRQ_ENB, cep_irq_enb);
		ep_dbg(ep, "%s(%d): CEP_IRQ_ENB=0x%x\n",
			__FUNCTION__, __LINE__, udc_readw(udc, CEP_IRQ_ENB));

	} else {
		irq_enb_l |= (1 << (ep_addr + 1));

		ep_irq_enb = udc_ep_readw(ep, EP_IRQ_ENB);
		if (ep->dir_in)
			ep_irq_enb |= EP_IN_IRQ_MASK;
		else
			ep_irq_enb |= EP_OUT_IRQ_MASK;

		udc_ep_writew(ep, EP_IRQ_ENB, ep_irq_enb);
		//ep_dbg(ep, "EP_IRQ_ENB=0x%x\n", udc_ep_readw(ep, EP_IRQ_ENB));
	}

	usb_irq_enb = udc_readb(udc, USB_IRQ_ENB);
	usb_irq_enb |= USB_IRQ_INT_MASK;
	udc_writeb(udc, USB_IRQ_ENB, usb_irq_enb);

	irq_enb_l |= USB_INT_EN;
	udc_writew(udc, IRQ_ENB_L, irq_enb_l);
	//ep_dbg(ep, "IRQ_ENB_L=0x%x\n", udc_readw(udc, IRQ_ENB_L));
}

/**
 * pio_irq_disable - Disables irq generation for one endpoint
 * @ep: udc endpoint
 */
static void 
pio_irq_disable(struct im98xx_ep *ep)
{
	struct im98xx_udc *udc = ep->udc;
	u8 ep_addr = (u8)ep->addr;
	u16 irq_enb_l;

	irq_enb_l = udc_readw(udc, IRQ_ENB_L);

	if(ep_addr == 0) {
		irq_enb_l &= ~CEP_INT_EN;
	} else {
		irq_enb_l &= ~(1 << (ep_addr + 1));
	}
	
	udc_writew(udc, IRQ_ENB_L, irq_enb_l);
	//ep_dbg(ep, "IRQ_ENB_L=0x%x\n", udc_readw(udc, IRQ_ENB_L));
}

/**
 * ep_count_bytes_remain - get how many bytes in udc endpoint
 * @ep: udc endpoint
 *
 * Returns number of bytes in OUT fifos.
 */
static int ep_count_bytes_remain(struct im98xx_ep *ep)
{
	if (is_ep0(ep))
		return udc_readw(ep->udc, OUT_TRNSFR_CNT) & 0xffff;
	else
		return udc_ep_readw(ep, EP_DATA_CNT) & 0x7ff;
}

/**
 * ep_is_empty - checks if there are bytes in the
 * endpoint fifo.
 * @ep: udc endpoint
 *
 * Returns 0 if ep not empty, 1 if ep empty
 */
static inline int ep_is_empty(struct im98xx_ep *ep)
{
	if (is_ep0(ep)) {
		return !!(udc_readw(ep->udc, CEP_IRQ_STAT) & CEP_BUF_EMPT_INT_EN);
	} else {
		return !!(udc_ep_readw(ep, EP_IRQ_STAT) & EP_BUF_EMPT_INT_EN);
	}

}

/**
 * ep_is_full - checks if ep has place to write bytes
 * @ep: udc endpoint
 *
 * If endpoint is not the control endpoint and is an IN endpoint, checks if
 * there is place to write bytes into the endpoint.
 *
 * Returns 0 if ep not full, 1 if ep full, -EOPNOTSUPP if OUT endpoint
 */
static inline int ep_is_full(struct im98xx_ep *ep)
{
	if (is_ep0(ep)) {
		return !!(udc_readw(ep->udc, CEP_IRQ_STAT) & CEP_BUF_FULL_INT_EN);
	} else {
		return !!(udc_ep_readw(ep, EP_IRQ_STAT) & EP_BUF_FULL_INT_EN);
	}
}

/**
 * udc_is_suspending - checks if UDC will enter suspend mode
 * @udc
 *
 * If udc enter suspend mode, any access register will cause system hold issue
 *
 * Returns 0 if udc not suspending, 1 if suspending
 */
static inline int udc_is_suspending(struct im98xx_udc *udc)
{
	return !!(udc_readb(udc, USB_IRQ_STAT) & SPNT_INT_EN);
}


/**
 * set_ep0state - Set ep0 automata state
 * @dev: udc device
 * @state: state
 */
static void set_ep0state(struct im98xx_udc *udc, int state)
{
	struct im98xx_ep *ep = &udc->im98xx_ep[0];
	char *old_stname = EP0_STNAME(udc);

	udc->ep0state = state;
	ep_dbg(ep, "state=%s->%s\n", old_stname, EP0_STNAME(udc));
}

/**
 * ep0_idle - Put control endpoint into idle state
 * @dev: udc device
 */
static void ep0_idle(struct im98xx_udc *udc)
{
	set_ep0state(udc, WAIT_FOR_SETUP);
}

/**
 * inc_ep_stats_reqs - Update ep stats counts
 * @ep: physical endpoint
 * @req: usb request
 * @is_in: ep direction (USB_DIR_IN or 0)
 *
 */
static void inc_ep_stats_reqs(struct im98xx_ep *ep, int is_in)
{
	if (is_in)
		ep->stats.in_ops++;
	else
		ep->stats.out_ops++;
}

/**
 * inc_ep_stats_bytes - Update ep stats counts
 * @ep: physical endpoint
 * @count: bytes transfered on endpoint
 * @is_in: ep direction (USB_DIR_IN or 0)
 */
static void inc_ep_stats_bytes(struct im98xx_ep *ep, int count, int is_in)
{
	if (is_in)
		ep->stats.in_bytes += count;
	else
		ep->stats.out_bytes += count;
}

static int ep_speed_setting(struct im98xx_ep *ep, const struct usb_endpoint_descriptor *desc)
{
	u16 ep_cfg = 0;
	int	retval = 0;

	if ((ep->udc->gadget.speed == USB_SPEED_UNKNOWN)  || (ep_configured == -1)) {
		ep_dbg(ep, " failed setting\n");
		return -ESHUTDOWN;
	}

	ep_cfg = udc_ep_readw(ep, EP_CFG);
	ep->mps = le16_to_cpu(desc->wMaxPacketSize) & 0xfff;
	ep_dbg(ep, "wMaxPacketSize = %d\n", desc->wMaxPacketSize);

	/* sanity check wMaxPacketSize */
	ep_cfg |= EP_VALID;
	ep_cfg &= ~(EP_TYPE_MASK);
	switch (usb_endpoint_type(desc)) {
	case USB_ENDPOINT_XFER_CONTROL:
		retval = -EINVAL;
		goto done;
	case USB_ENDPOINT_XFER_ISOC:
		ep_cfg |= EP_TYPE_ISO;
		break;
	case USB_ENDPOINT_XFER_BULK:
		ep_cfg |= EP_TYPE_BULK;

		if (ep->udc->gadget.speed == USB_SPEED_FULL
				&& ep->mps > 64) {
			ep_info(ep, "Error: ep->mps=%d\n", ep->mps);
			ep->mps = le16_to_cpu(64) & 0xfff;
		}
		break;
	case USB_ENDPOINT_XFER_INT:
		ep_cfg |= EP_TYPE_INT;
		break;
	default:
		retval = -EINVAL;
		goto done;
	}
	ep->type = usb_endpoint_type(desc);

	udc_ep_writew(ep, EP_MPS, ep->mps);
	ep->ep_end_addr = ep->ep_start_addr + (ep->mps / 2) - 1;
	udc_ep_writew(ep, EP_START_ADDR, ep->ep_start_addr);
	udc_ep_writew(ep, EP_END_ADDR, ep->ep_end_addr);

	ep_cfg &= ~EP_DIR_IN;
	if (usb_endpoint_dir_in(desc)) {
		ep->dir_in = 1;
		ep_cfg |= EP_DIR_IN;
	} else if (usb_endpoint_dir_out(desc)) {
		ep->dir_in = 0;
		ep_cfg &= ~EP_DIR_IN;
	}

	ep_cfg &= ~EP_ADDR_MASK;
	ep_cfg |= (ep->addr << 4);
	ep_cfg &= ~EP_MULT_MASK;
	ep_cfg |= (ep->ep_trsactn_num << 8);
	ep_dbg(ep, "ep_cfg=0x%04x\n", ep_cfg);
	udc_ep_writew(ep, EP_CFG, ep_cfg);
	ep_dbg(ep, "ep_cfg=0x%04x\n", udc_ep_readw(ep, EP_CFG));
done:
	return retval;
}

static int ep_rsp_setting(struct im98xx_ep *ep)
{
	u8 ep_rsp_sc = 0;

	if ((ep->udc->gadget.speed == USB_SPEED_UNKNOWN)  || (ep_configured == -1)) {
		ep_dbg(ep, " failed setting\n");
		return -ESHUTDOWN;
	}

	ep_rsp_sc = udc_ep_readb(ep, EP_RSP_SC);
	ep_rsp_sc &= ~(EP_IN_MODE_RESERVED << EP_IN_MOD_SHIFT);
	ep_rsp_sc |= (ep->ep_in_mod << EP_IN_MOD_SHIFT) | EP_BUF_FLUSH | EP_TOGGLE;
	udc_ep_writeb(ep, EP_RSP_SC, ep_rsp_sc);
	ep_dbg(ep, "EP_RSP_SC=0x%x\n", udc_ep_readb(ep, EP_RSP_SC));

	ep_dbg(ep, "%p, direction:%s, %s, type: %d, ep_in_mod:%d, "
		"[MPS:%dB, ep_fifo_size:%d], ep_ram_range:[0x%x, 0x%x], EP_CFG=0x%x\n",
		ep, ep->dir_in?"IN":"OUT", ep->can_dma?"DMA":"PIO", ep->type, ep->ep_in_mod,
		udc_ep_readw(ep, EP_MPS), ep->fifo_size, ep->ep_start_addr, ep->ep_end_addr,
		udc_ep_readw(ep, EP_CFG));

	printk("%s: ep%d,dir:%s,%s,type:%d, [MPS:%dB, EFS:%dB],ep_ram_range:[0x%x,0x%x]\n",
		__func__, ep->addr, ep->dir_in?"IN":"OUT", ep->can_dma?"DMA":"PIO", ep->type, udc_ep_readw(ep, EP_MPS), ep->fifo_size,
		udc_ep_readw(ep, EP_START_ADDR), udc_ep_readw(ep, EP_END_ADDR));
	return 0;
}

/**
 * im98xx_ep0_setup - Sets up control endpoint0
 * @ep: im98xx physical endpoint
 *
 * Find the physical im98xx ep, and set it up.
 */
static int im98xx_ep0_setup(struct im98xx_ep *ep)
{

	/* ep0 only needs to set its buf size, physically */
	udc_writew(ep->udc, CEP_START_ADDR, ep->ep_start_addr);
	udc_writew(ep->udc, CEP_END_ADDR, ep->ep_end_addr);
	udc_writeb(ep->udc, CEP_CTRL_STAT, (NAK_CLR | CEP_FLUSH));

	ep_dbg(ep, "[MPS:%d, cep_fifo_size:%d], cep_ram_range:[0x%x, 0x%x]\n",
		ep->mps, ep->fifo_size, ep->ep_start_addr, ep->ep_end_addr);

	/*printk("%s: ep%d,[MPS:%dB, CEFS:%dB], cep_ram_range:[0x%x,0x%x]\n",
		__func__, ep->addr, ep->mps, ep->fifo_size, ep->ep_start_addr, ep->ep_end_addr);*/
	return 0;
}

/**
 * im98xx_ep_setup - Sets up an usb physical endpoint
 * @ep: im98xx physical endpoint
 *
 * Find the physical im98xx ep, and set it up.
 */
static int im98xx_ep_setup(struct im98xx_ep *ep, int addr, 
	const struct usb_endpoint_descriptor *desc)
{
	int	retval = 0;

	retval = ep_speed_setting(ep, desc);
	if (retval != 0) {
		return -ESHUTDOWN;
	}

	retval = ep_rsp_setting(ep);
	if (retval != 0) {
		return -ESHUTDOWN;
	}

	return retval;
}

/**
 * im98xx_ep_alloc_request - Allocate usb request
 * @_ep: usb endpoint
 * @gfp_flags:
 *
 * For the im98xx, these can just wrap kmalloc/kfree.  gadget drivers
 * must still pass correctly initialized endpoints, since other controller
 * drivers may care about how it's currently set up (dma issues etc).
  */
static struct usb_request *
im98xx_ep_alloc_request(struct usb_ep *_ep, gfp_t gfp_flags)
{
	struct im98xx_request *req;
	struct im98xx_ep *ep = to_im98xx_ep(_ep);
	
	ep_vdbg(ep, "ep_alloc_request: %p, 0x%x\n", _ep, gfp_flags);

	req = kzalloc(sizeof(*req), gfp_flags);
	if (!req)
		return NULL;
	
	req->req.dma = DMA_ADDR_INVALID;

	INIT_LIST_HEAD(&req->queue);
	req->in_use = 0;

	return &req->req;
}

/**
 * im98xx_ep_free_request - Free usb request
 * @_ep: usb endpoint
 * @_req: usb request
 *
 * Wrapper around kfree to free _req
 */
static void im98xx_ep_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct im98xx_request *req;
	struct im98xx_ep *ep = to_im98xx_ep(_ep);

	ep_vdbg(ep, "ep_free_request: %p, %p\n", _ep, _req);

	req = to_im98xx_req(_req);
	WARN_ON(!list_empty(&req->queue));
	kfree(req);
}

#if 0
/**
 * ep_add_request - add a request to the endpoint's queue
 * @ep: usb endpoint
 * @req: usb request
 *
 * Context: ep->lock held
 *
 * Queues the request in the endpoint's queue, and enables the interrupts
 * on the endpoint.
 */
static void ep_add_request(struct im98xx_ep *ep, struct im98xx_request *req)
{
	unsigned usb_detect_cable = ep->udc->usb_cable_gpio;

	if (unlikely(!req))
		return;
	ep_dbg(ep, "req:%p, lg=%d\n", req, req->req.length);
	req->in_use = 1;
	list_add_tail(&req->queue, &ep->queue);

	/* Before access USB IP registers, detect the existence of USB cable */
	gpio_direction_input(usb_detect_cable);
	if (__gpio_get_value(usb_detect_cable))
		ep->udc->usb_cable_sensed = 1;
	else
		ep->udc->usb_cable_sensed = 0;
	gpio_line_set_interrupt_enable_disable(usb_detect_cable, GPIO_INT_GEN);

	if (ep->udc->usb_cable_sensed && !is_ep0(ep)) {
		pio_irq_enable(ep);
	}
}
#else
/**
 * ep_add_request - add a request to the endpoint's queue
 * @ep: usb endpoint
 * @req: usb request
 *
 * Context: ep->lock held
 *
 * Queues the request in the endpoint's queue, and enables the interrupts
 * on the endpoint.
 */
static void ep_add_request(struct im98xx_ep *ep, struct im98xx_request *req)
{
	if (unlikely(!req))
		return;

	ep_dbg(ep, "req:%p, lg=%d\n", req, req->req.length);

	req->in_use = 1;
	list_add_tail(&req->queue, &ep->queue);

	if (!is_ep0(ep)) {
		if (unlikely(udc_is_suspending(ep->udc))) {
			ep_info(ep, "udc will enter suspend\n");
			return;
		}
		pio_irq_enable(ep);
	}
}

#endif
/**
 * ep_del_request - removes a request from the endpoint's queue
 * @ep: usb endpoint
 * @req: usb request
 *
 * Context: ep->lock held
 *
 * Unqueue the request from the endpoint's queue. If there are no more requests
 * on the endpoint, and if it's not the control endpoint, interrupts are
 * disabled on the endpoint.
 */
static void ep_del_request(struct im98xx_ep *ep, struct im98xx_request *req)
{
	if (unlikely(!req))
		return;

	ep_dbg(ep, "req:%p, lg=%d\n", req, req->req.length);
	list_del_init(&req->queue);
	req->in_use = 0;
	if (likely(ep->udc->usb_cable_sensed) && !is_ep0(ep) && list_empty(&ep->queue)) {
		if (unlikely(udc_is_suspending(ep->udc))) {
			return;
		}
		pio_irq_disable(ep);
	}
}

/**
 * req_done - Complete an usb request
 * @ep: im98xx physical endpoint
 * @req: im98xx request
 * @status: usb request status sent to gadget API
 *
 * Context: ep->lock held
 *
 * Retire a im98xx usb request. Endpoint must be locked.
 */
static void req_done(struct im98xx_ep *ep, struct im98xx_request *req, int status)
{
	ep_del_request(ep, req);
	/*
	if (ep->dir_in && (ep->type == USB_ENDPOINT_XFER_BULK))
		wake_up_interruptible(&in_waitq);
	*/
	if (likely(req->req.status == -EINPROGRESS))
		req->req.status = status;
	else
		status = req->req.status;

	if (status && status != -ESHUTDOWN)
		ep_dbg(ep, "complete req %p stat %d len %u/%u\n",
			&req->req, status,
			req->req.actual, req->req.length);
	req->req.complete(&ep->usb_ep, &req->req);
}

/**
 * ep_end_out_req - Ends endpoint OUT request
 * @ep: physical endpoint
 * @req: im98xx request
 *
 * Context: ep->lock held
 *
 * Ends endpoint OUT request (completes usb request).
 */
static void ep_end_out_req(struct im98xx_ep *ep, struct im98xx_request *req)
{
	inc_ep_stats_reqs(ep, !USB_DIR_IN);
	req_done(ep, req, 0);
}

/**
 * ep0_end_out_req - Ends control endpoint OUT request (ends data stage)
 * @ep: physical endpoint
 * @req: im98xx request
 *
 * Context: ep->lock held
 *
 * Ends control endpoint OUT request (completes usb request), and puts
 * control endpoint into idle state
 */
static void ep0_end_out_req(struct im98xx_ep *ep, struct im98xx_request *req)
{
	set_ep0state(ep->udc, OUT_STATUS_STAGE);
	ep_end_out_req(ep, req);
}

/**
 * ep_end_in_req - Ends endpoint IN request
 * @ep: physical endpoint
 * @req: im98xx request
 *
 * Context: ep->lock held
 *
 * Ends endpoint IN request (completes usb request).
 */
static void ep_end_in_req(struct im98xx_ep *ep, struct im98xx_request *req)
{
	inc_ep_stats_reqs(ep, USB_DIR_IN);
	req_done(ep, req, 0);
}

static void ep0_nak_clr(struct im98xx_ep *ep)
{
	u8 cep_ctl_stat = 0;

	cep_ctl_stat = udc_readb(ep->udc, CEP_CTRL_STAT);
	//printk("%s(%d):CEP_CTRL_STAT=0x%x\n", __FUNCTION__, __LINE__, cep_ctl_stat);
	cep_ctl_stat &= ~NAK_CLR;
	udc_writeb(ep->udc, CEP_CTRL_STAT, cep_ctl_stat);
	cep_ctl_stat = udc_readb(ep->udc, CEP_CTRL_STAT);
	//printk("%s(%d):CEP_CTRL_STAT=0x%x\n", __FUNCTION__, __LINE__, cep_ctl_stat);
}

#if 0
static void ep0_in_zlp_pkg(struct im98xx_ep *ep)
{
	u8 cep_ctl_stat = 0;
	
	cep_ctl_stat = udc_readb(ep->udc, CEP_CTRL_STAT);
	//printk("%s(%d):CEP_CTRL_STAT=0x%x\n", __FUNCTION__, __LINE__, cep_ctl_stat);
	cep_ctl_stat |= ZEROLEN;
	udc_writeb(ep->udc, CEP_CTRL_STAT, cep_ctl_stat);
	cep_ctl_stat = udc_readb(ep->udc, CEP_CTRL_STAT);
	//printk("%s(%d):CEP_CTRL_STAT=0x%x\n", __FUNCTION__, __LINE__, cep_ctl_stat);
}
#endif
/**
 * ep0_end_in_req - Ends control endpoint IN request (ends data stage)
 * @ep: physical endpoint
 * @req: im98xx request
 *
 * Context: ep->lock held
 *
 * Ends control endpoint IN request (completes usb request), and puts
 * control endpoint into status state
 */
static void ep0_end_in_req(struct im98xx_ep *ep, struct im98xx_request *req)
{
	set_ep0state(ep->udc, IN_STATUS_STAGE);
	ep_end_in_req(ep, req);
}

/**
 * nuke - Dequeue all requests
 * @ep: im98xx endpoint
 * @status: usb request status
 *
 * Context: ep->lock held
 *
 * Dequeues all requests on an endpoint. As a side effect, interrupts will be
 * disabled on that endpoint (because no more requests).
 */
static void nuke(struct im98xx_ep *ep, int status)
{
	struct im98xx_request *req;

	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, struct im98xx_request, queue);
		req_done(ep, req, status);
	}
}

/**
 * read_packet - transfer 1 packet from an OUT endpoint into request
 * @ep: im98xx physical endpoint
 * @req: usb request
 *
 * Takes bytes from OUT endpoint and transfers them info the usb request.
 * If there is less space in request than bytes received in OUT endpoint,
 * bytes are left in the OUT endpoint.
 *
 * Returns how many bytes were actually transfered
 */
static int read_packet(struct im98xx_ep *ep, struct im98xx_request *req)
{
	u16 *buf;
	u8 *buf_8;
	int bytes_ep, bufferspace, count, i, remain, length;

	bytes_ep = ep_count_bytes_remain(ep);
	bufferspace = req->req.length - req->req.actual;

	//ep_dbg(ep, "bytes:%d, bufferspace:%d\n", bytes_ep, bufferspace);

	buf = (u16 *)(req->req.buf + req->req.actual);
	//prefetchw(buf);

	length = min(bytes_ep, bufferspace);
	remain = length & 0x1;
	count = length & ~(0x1);

	for (i = count; i > 0; i -= 2) {
		*buf++ = udc_ep_readw(ep, EP_DATA_BUF);
	}

	if (1 == remain) {
		buf_8 = (u8 *)buf;
		*buf_8 = udc_ep_readb(ep, EP_DATA_BUF);
	}
	req->req.actual += length;

	//ep_dbg(ep, "length=%d+%d\n", count, remain);

	return length;
}

/**
 * write_packet - transfer 1 packet from request into an IN endpoint
 * @ep: im98xx physical endpoint
 * @req: usb request
 * @max: max bytes that fit into endpoint
 *
 * Takes bytes from usb request, and transfers them into the physical
 * endpoint. If there are no bytes to transfer, doesn't write anything
 * to physical endpoint.
 *
 * Returns how many bytes were actually transfered.
 */
static int write_packet(struct im98xx_ep *ep, struct im98xx_request *req,
			unsigned int max)
{
	int length, count=0, remain=0, i;
	u16 *buf;
	u8 *buf_8;

	buf = (u16 *)(req->req.buf + req->req.actual);
	prefetch(buf);

	length = min(req->req.length - req->req.actual, max);
	req->req.actual += length;

	if (0 != length) {
		remain = length & 0x1;
		count = length & ~(0x1);
		for (i = count; i > 0 ; i -= 2)
			udc_ep_writew(ep, EP_DATA_BUF, *buf++);
		if (1 == remain) {
			buf_8 = (u8 *)buf;
			udc_ep_writeb(ep, EP_DATA_BUF, *buf_8);
		}
	}

	//ep_dbg(ep, "length(%d)=%d+%d\n", length, count, remain);
	return length;
}

/**
 * read_fifo - Transfer packets from OUT endpoint into usb request
 * @ep: im98xx physical endpoint
 * @req: usb request
 *
 * Context: callable when in_interrupt()
 *
 * Unload as many packets as possible from the fifo we use for usb OUT
 * transfers and put them into the request. Caller should have made sure
 * there's at least one packet ready.
 * Doesn't complete the request, that's the caller's job
 *
 * Returns 1 if the request completed, 0 otherwise
 */
static int read_fifo(struct im98xx_ep *ep, struct im98xx_request *req)
{
	int count, completed = 0;
	int is_short = 0;

	if (ep_is_empty(ep)) {
		ep_dbg(ep, "Buffer empty, return.\n");
		return 0;
	}

	count = read_packet(ep, req);
	//inc_ep_stats_bytes(ep, count, !USB_DIR_IN);
	is_short = count < ep->mps;

	ep_dbg(ep, "count:%d bytes%s req %p %d/%d\n",
		count, is_short ? "/S" : "",
		&req->req, req->req.actual, req->req.length);

	/* completion */
	if (unlikely(is_short || req->req.actual == req->req.length)) {
		completed = 1;
	}

	return completed;
}

/**
 * write_fifo - transfer packets from usb request into an IN endpoint
 * @ep: im98xx physical endpoint
 * @req: im98xx usb request
 *
 * Write to an IN endpoint fifo, as many packets as possible.
 * irqs will use this to write the rest later.
 * caller guarantees at least one packet buffer is ready (or a zlp).
 * Doesn't complete the request, that's the caller's job
 *
 * Returns 1 if request fully transfered, 0 if partial transfer
 */
static int write_fifo(struct im98xx_ep *ep, struct im98xx_request *req)
{
	unsigned max;
	int count=0, is_short, is_last = 0, completed = 0;

	max = ep->mps;
	is_short = 0;

	count = write_packet(ep, req, max);
	//inc_ep_stats_bytes(ep, count, USB_DIR_IN);

	/* last packet is usually short (or a zlp) */
	if (likely(count < max)) {
		is_last = 1;
		is_short = 1;
	} else {
		if (likely(req->req.length > req->req.actual)
				|| req->req.zero)
			is_last = 0;
		else
			is_last = 1;

		/* interrupt/iso maxpacket may not fill the fifo */
#if 0
		if (unlikely(!EPXFERTYPE_is_BULK(ep))) {
			ep_dbg(ep, "not bulk, FIXME: for int/iso\n");
			is_short = unlikely(max < ep->fifo_size);
         }
#endif
	}
	if (0 != count) {
		if (is_short)
			udc_ep_writeb(ep, EP_RSP_SC, EP_PKTEND);
	} else
		udc_ep_writeb(ep, EP_RSP_SC, EP_IN_ZEROLEN);
	/* requests complete when all IN data is in the FIFO */
	if (is_last)
		completed = 1;

	ep_dbg(ep, "wrote count:%d bytes%s%s, left:%d req=%p\n",
		count, is_last ? "/L" : "", is_short ? "/S" : "",
		req->req.length - req->req.actual, &req->req);

	return completed;
}

/**
 * read_ep0_packet - transfer 1 packet from an OUT endpoint into request
 * @ep: im98xx physical endpoint
 * @req: usb request
 *
 * Takes bytes from OUT endpoint and transfers them info the usb request.
 * If there is less space in request than bytes received in OUT endpoint,
 * bytes are left in the OUT endpoint.
 *
 * Returns how many bytes were actually transfered
 */
static int read_ep0_packet(struct im98xx_ep *ep, struct im98xx_request *req)
{
	u16 *buf;
	int bytes_ep, bufferspace, count, i;

	//bytes_ep = ep_count_bytes_remain(ep);
	bytes_ep = udc_readw(ep->udc, OUT_TRNSFR_CNT);
	bufferspace = req->req.length - req->req.actual;

	buf = (u16 *)(req->req.buf + req->req.actual);
	//prefetchw(buf);

	if (likely(!ep_is_empty(ep)))
		count = min(bytes_ep, bufferspace);
	else /* zlp */
		count = 0;

	for (i = count; i > 0; i -= 2)
		*buf++ = udc_readw(ep->udc, CEP_DATA_BUF);
	req->req.actual += count;

	return count;
}

/**
 * write_ep0_packet - transfer 1 packet from request into an IN endpoint
 * @ep: im98xx physical endpoint
 * @req: usb request
 * @max: max bytes that fit into endpoint
 *
 * Takes bytes from usb request, and transfers them into the physical
 * endpoint. If there are no bytes to transfer, doesn't write anything
 * to physical endpoint.
 *
 * Returns how many bytes were actually transfered.
 */
static int write_ep0_packet(struct im98xx_ep *ep, struct im98xx_request *req,
			unsigned int max)
{
	int length, count, remain, i;
	u16 *buf;
	u8 *buf_8;

	buf = (u16 *)(req->req.buf + req->req.actual);
	prefetch(buf);

	length = min(req->req.length - req->req.actual, max);
	req->req.actual += length;

	remain = length & 0x1;
	count = length & ~(0x1);
	for (i = count; i > 0 ; i -= 2)
		udc_writew(ep->udc, CEP_DATA_BUF, *buf++);
	if (1 == remain) {
		buf_8 = (u8 *)buf;
		udc_writeb(ep->udc, CEP_DATA_BUF, *buf_8++);
	}

	/* Trigger the ADC to respond to the forthcoming IN-token */
	udc_writew(ep->udc, IN_TRNSFR_CNT, length);
	//print_all_registers(ep->udc);
	
	//ep_dbg(ep, "length=%d+%d\n", count, remain);
	return length;
}

/**
 * read_ep0_fifo - Transfer packets from control endpoint into usb request
 * @ep: control endpoint
 * @req: im98xx usb request
 *
 * Special ep0 version of the above read_fifo. Reads as many bytes from control
 * endpoint as can be read, and stores them into usb request (limited by request
 * maximum length).
 *
 * Returns 0 if usb request only partially filled, 1 if fully filled
 */
static int read_ep0_fifo(struct im98xx_ep *ep, struct im98xx_request *req)
{
	int count, completed = 0;
	int is_short = 0;

	if (ep_is_empty(ep)) {
		ep_info(ep, "Buffer empty, return.\n");
		return 0;
	}

	count = read_ep0_packet(ep, req);
	inc_ep_stats_bytes(ep, count, !USB_DIR_IN);

	is_short = count < ep->mps;
	ep_dbg(ep, "read count:%d bytes req %p %d/%d\n",
		count, &req->req, req->req.actual, req->req.length);

	if (likely(is_short || req->req.actual >= req->req.length)) {
		completed = 1;
	}

	return completed;
}

static ep_rw_status wait_read_ep0_data_done(struct im98xx_udc *udc)
{
	int count;
	ep_rw_status status = EP_RW_OK;

	for (count = 0; count < EP0_WAIT_RETRY; count++) {
		if (unlikely(udc_is_suspending(udc))) {
			status = EP_RW_SUSPEND;
			break;
		}

		if ((udc_readw(udc, CEP_IRQ_STAT) & CEP_DATAPKT_RECIVE_INT_EN) != 0)
			break;
		im98xx_26MHz_halt(10,GPT1);//udelay(EP0_WAIT_DELAY);
		
	}
	if(count > 1)
		dev_dbg(udc->dev, "ep0 count: %d\n", count);
	if (EP0_WAIT_RETRY == count) {
		status = EP_RW_TIMEOUT;
	}
	return status;
}

/**
 * write_ep0_fifo - Send a request to control endpoint (ep0 in)
 * @ep: control endpoint
 * @req: request
 *
 * Context: callable when in_interrupt()
 *
 * Sends a request (or a part of the request) to the control endpoint (ep0 in).
 * If the request doesn't fit, the remaining part will be sent from irq.
 * The request is considered fully written only if either :
 *   - last write transfered all remaining bytes, but fifo was not fully filled
 *   - last write was a 0 length write
 *
 * Returns 1 if request fully written, 0 if request only partially sent
 */
static int write_ep0_fifo(struct im98xx_ep *ep, struct im98xx_request *req)
{
	unsigned	count;
	int		is_last, is_short, is_zero;

	count = write_ep0_packet(ep, req, ep->mps);
	inc_ep_stats_bytes(ep, count, USB_DIR_IN);

	is_short = (count < ep->mps);
	is_zero = (count == 0);
	is_last = ((count == 0) || (count < ep->mps));

	/* Sends a zlp(zero length packet) */
	if (unlikely(is_zero))
		udc_writeb(ep->udc, CEP_CTRL_STAT, ZEROLEN);
#if 0
	if (is_last)
		ep0_nak_clr(ep);
#endif
	ep_dbg(ep, "in %d bytes%s%s, %d left, req=%p\n",
		count, is_short ? "/S" : "", is_last ? "/L" : "",
		req->req.length - req->req.actual, &req->req);
	return is_last;
}


/**
 * im98xx_ep_queue - Queue a request into an IN endpoint
 * @_ep: usb endpoint
 * @_req: usb request
 * @gfp_flags: flags
 *
 * Context: normally called when !in_interrupt, but callable when in_interrupt()
 * in the special case of ep0 setup :
 *   (irq->handle_ep0_ctrl_req->handle_ep0_setup->gadget_setup->im98xx_ep_queue)
 *
 * Returns 0 if succedeed, error otherwise
 */
static int im98xx_ep_queue(struct usb_ep *_ep, struct usb_request *_req,
			gfp_t gfp_flags)
{
	struct im98xx_ep		*ep;
	struct im98xx_request	*req;
	struct im98xx_udc		*udc;
	int			rc = 0;
	int			is_first_req;
	unsigned		length;
	int completed = 0;
	u16 cep_irq_enb = 0;
	ep_rw_status out_status = EP_RW_OK;

	req = to_im98xx_req(_req);
	ep = to_im98xx_ep(_ep);

	if (unlikely(!_req || !_req->complete || !_req->buf)) {
		printk("im98xx_ep_queue, bad params\n");
		return -EINVAL;
	}

	if (unlikely(!_ep)) {
		printk("im98xx_ep_queue, bad _ep params\n");
		return -EINVAL;
	}

	udc = ep->udc;
	ep_dbg(ep, "entered\n");

	if (unlikely(!udc->driver || (udc->gadget.speed == USB_SPEED_UNKNOWN))) {
		ep_dbg(ep, "bogus device state\n");
		return -ESHUTDOWN;
	}


	is_first_req = list_empty(&ep->queue);
	if (unlikely(!is_first_req)) {
		ep_dbg(ep, "queue req %p(first=%s), len %d buf %p\n",
			_req, is_first_req ? "yes" : "no",
			_req->length, _req->buf);
		/*
		if (ep->dir_in && (ep->type == USB_ENDPOINT_XFER_BULK)) {
			wait_event_interruptible(in_waitq, list_empty(&ep->queue));
			ep_dbg(ep, "leave wait req = %p\n", _req);
		}
		*/
	}

	if (unlikely(!ep->enabled || (ep_configured == -1))) {
		ep_info(ep, "device suspend state, exit\n");
		_req->status = -ESHUTDOWN;
		rc = -ESHUTDOWN;
		goto out;
	}

	if (req->in_use) {
		ep_info(ep, "refusing to queue req %p (already queued)\n", req);
		goto out;
	}

	_req->status = -EINPROGRESS;
	_req->actual = 0;

	ep_add_request(ep, req);

	if (is_ep0(ep)) {
		length = _req->length;
		switch (udc->ep0state) {
		case WAIT_ACK_SET_CONF_INTERF:
			if (length == 0) {
				ep_end_in_req(ep, req);
			} else {
				ep_info(ep, "got a request of %d bytes while"
					"in state WAIT_ACK_SET_CONF_INTERF\n",
					length);
				ep_del_request(ep, req);
				rc = -EL2HLT;
			}
			ep0_idle(ep->udc);
			break;
		case IN_DATA_STAGE:
			if (!ep_is_full(ep))
				ep->is_last_pkt = write_ep0_fifo(ep, req);
			break;
		case OUT_DATA_STAGE:
			cep_irq_enb = udc_readw(udc, CEP_IRQ_ENB);
			cep_irq_enb |= CEP_DATAPKT_RECIVE_INT_EN;
			udc_writew(udc, CEP_IRQ_ENB, cep_irq_enb);

			if ((udc_readw(udc, CEP_IRQ_STAT) & CEP_BUF_EMPT_INT_EN) == 0) {
				out_status = wait_read_ep0_data_done(udc);
				switch (out_status) {
				case EP_RW_OK:
					dev_dbg(udc->dev, "EP0_OUT: OK, CEP_IRQ_STAT:0x%08x\n", udc_readw(udc, CEP_IRQ_STAT));
					break;
				case EP_RW_SUSPEND:
					ep_info(ep, "EP0_OUT: ERROR -- cable unstable, will suspend\n");
					udc_writeb(ep->udc, CEP_CTRL_STAT, (udc_readb(ep->udc, CEP_CTRL_STAT) | CEP_FLUSH));
					nuke(ep, -EPIPE);
					break;
				case EP_RW_TIMEOUT:
					dev_info(udc->dev, "Queue: EP0_OUT_DATA_INT\n");
					if (unlikely(req->req.length == 0)) {
						dev_info(udc->dev, "Queue:length == 0\n");
						ep0_end_out_req(ep, req);
					} else {
						ep->is_last_pkt = read_ep0_fifo(ep, req);
						if (ep->is_last_pkt == 1) {
							ep->is_last_pkt = 0;
							ep0_end_out_req(ep, req);
						}
					}
					if (udc->ep0state == OUT_STATUS_STAGE) {
						cep_irq_enb = udc_readw(udc, CEP_IRQ_ENB);
						cep_irq_enb &= ~CEP_DATAPKT_RECIVE_INT_EN;
						udc_writew(udc, CEP_IRQ_ENB, cep_irq_enb);
						ep0_nak_clr(ep);
					}
					break;
				}
			}
			dev_dbg(udc->dev, "Queue:CEP_IRQ_STAT=0x%08x, OUT_TRNSFR_CNT =%d\n", udc_readw(udc, CEP_IRQ_STAT), udc_readw(ep->udc, OUT_TRNSFR_CNT));
			break;
		case IN_STATUS_STAGE:
			ep_end_in_req(ep, req);
			ep0_nak_clr(ep);
			break;
		case OUT_STATUS_STAGE:
			ep_end_out_req(ep, req);
			ep0_nak_clr(ep);
			ep_dbg(ep, "SET_CONFIGURATION end\n");
			break;
		default:
			ep_info(ep, "odd state %s to send me a request\n",
				EP0_STNAME(ep->udc));
			ep_del_request(ep, req);
			ep0_nak_clr(ep);
			rc = 0;
			break;
		}
	} else if (ep->dir_in && ep_is_empty(ep)) {
		if (_req->length == 0) {
			udc_ep_writeb(ep, EP_RSP_SC, EP_IN_ZEROLEN);
			ep->is_last_pkt = 1;
			ep_dbg(ep, "in_len_0: -- EP_IRQ_STAT=0x%08x\n", udc_ep_readw(ep, EP_IRQ_STAT));
		} else {
			ep->is_last_pkt = write_fifo(ep, req);
		}
	} else if (!ep->dir_in) {
		if (unlikely(_req->length == 0)) {
			ep_info(ep, "out_len_0: -- EP_IRQ_STAT=0x%08x\n", udc_ep_readw(ep, EP_IRQ_STAT));
			ep_end_out_req(ep, req);
		} else if (((udc_ep_readw(ep, EP_DATA_CNT) & 0x7ff) != 0) && 
                ((udc_ep_readw(ep, EP_IRQ_STAT) & EP_DATAPKT_RECEIVED_INT_EN) == 0)) {

			completed = read_fifo(ep, req);
			//ep_dbg(ep, "completed=%d\n", completed);
			if (completed) {
				ep_info(ep, "completed: EP_DATA_CNT=%d, EP_IRQ_STAT=0x%08x\n",
					(udc_ep_readw(ep, EP_DATA_CNT) & 0x7ff), udc_ep_readw(ep, EP_IRQ_STAT));
				ep_end_out_req(ep, req);
			}
		}
	}

out:
	return rc;
}

/**
 * im98xx_ep_dequeue - Dequeue one request
 * @_ep: usb endpoint
 * @_req: usb request
 *
 * Return 0 if no error, -EINVAL or -ECONNRESET otherwise
 */
static int im98xx_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct im98xx_ep		*ep;
	struct im98xx_request	*req;
	int	rc = -EINVAL;

	if (!_ep)
		return rc;
	ep = to_im98xx_ep(_ep);
	if (!ep || is_ep0(ep))
		return rc;


	/* make sure it's actually queued on this endpoint */
	list_for_each_entry(req, &ep->queue, queue) {
		if (&req->req == _req) {
			req_done(ep, req, -ECONNRESET);
			rc = 0;
			break;
		}
	}

	return rc;
}

#if 0
/**
 * im98xx_ep_set_halt - Halts operations on one endpoint
 * @_ep: usb endpoint
 * @value:
 *
 * Returns 0 if no error, -EINVAL, -EROFS, -EAGAIN otherwise
 */
static int im98xx_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct im98xx_ep		*ep;
	u8 cep_ctl_stat = 0;
	unsigned long flags;
	int rc;

	if (!_ep)
		return -EINVAL;
	ep = to_im98xx_ep(_ep);

	if (!ep->udc->usb_cable_sensed || !ep || is_ep0(ep))
		return -EINVAL;
	ep_info(ep, "im98xx_ep_set_halt() called\n");

	if (value == 0) {
		/*
		 * This path (reset toggle+halt) is needed to implement
		 * SET_INTERFACE on normal hardware.  but it can't be
		 * done from software on the im98xx UDC, and the hardware
		 * forgets to do it as part of SET_INTERFACE automagic.
		 */
		ep_info(ep, "only host can clear halt\n");
		return -EROFS;
	}


	rc = -EAGAIN;
	if (ep->dir_in	&& (ep_is_full(ep) || !list_empty(&ep->queue)))
		goto out;

	/* Flush, set STALL bits for control and non control endpoints */
	rc = 0;
	if (is_ep0(ep)) {
		set_ep0state(ep->udc, UDC_STALL);
		/*Flush control EP0*/
		udc_writeb(ep->udc, CEP_CTRL_STAT, (NAK_CLR | CEP_FLUSH));
		/* STALL handshake needs to be sent to the host */
		cep_ctl_stat = udc_readb(ep->udc, CEP_CTRL_STAT);
		cep_ctl_stat &= ~NAK_CLR;
		cep_ctl_stat |= STALL;
		udc_writeb(ep->udc, CEP_CTRL_STAT, cep_ctl_stat);
	} else {
		udc_ep_writeb(ep, EP_RSP_SC, 
			((udc_ep_readb(ep, EP_RSP_SC) & (~EP_TOGGLE)) | EP_BUF_FLUSH | EP_HALT));
	}
out:
	return rc;
}
#else
/* endpoint set/clear halt */
static void ep_set_halt(struct im98xx_ep *ep, int value)
{
	struct im98xx_udc *udc = ep->udc;
	u8   cep_ctl_stat = 0;

	/* value: 1 - set halt, 0 - clear halt */
	if (value) {
		/* set the stall bit */
        if (is_ep0(ep)) {
            set_ep0state(ep->udc, UDC_STALL);
            /* STALL handshake needs to be sent to the host */
            cep_ctl_stat = udc_readb(ep->udc, CEP_CTRL_STAT);
            cep_ctl_stat &= ~NAK_CLR;
            cep_ctl_stat |= STALL;
            udc_writeb(ep->udc, CEP_CTRL_STAT, cep_ctl_stat);
        } else {
            udc_ep_writeb(ep, EP_RSP_SC, 
                ((udc_ep_readb(ep, EP_RSP_SC)) | EP_TOGGLE | EP_HALT));
            ep0_nak_clr(ep);
        }
	} else {
        /* clear the stall bit*/
        if (is_ep0(ep)) {
            ep0_idle(udc);
            /* clear STALL */
            cep_ctl_stat = udc_readb(ep->udc, CEP_CTRL_STAT);
            cep_ctl_stat &= ~(NAK_CLR | STALL);
            udc_writeb(ep->udc, CEP_CTRL_STAT, cep_ctl_stat);
        } else {
            udc_ep_writeb(ep, EP_RSP_SC, 
                (udc_ep_readb(ep, EP_RSP_SC) & ~EP_HALT));
            ep0_nak_clr(ep);
        }
	}
}

/* set the endpoint halt feature */
static int im98xx_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct im98xx_ep	*ep;
	struct im98xx_udc	*udc;
	int			retval = 0;

	if (!_ep)
		return -EINVAL;
	ep = to_im98xx_ep(_ep);

	if (!ep || !ep->udc->usb_cable_sensed || !ep->desc)
		return -EINVAL;
	ep_dbg(ep, "called\n");
	udc = ep->udc;

	if (!udc->driver || (udc->gadget.speed == USB_SPEED_UNKNOWN) || (ep_configured == -1))
		return -ESHUTDOWN;

	if (ep->desc && (ep->desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
			== USB_ENDPOINT_XFER_ISOC)
		return  -EOPNOTSUPP;


	/*
	 * attempt to halt IN ep will fail if any transfer requests
	 * are still queue
	 */
	if (!list_empty(&ep->queue) && ep->dir_in && value) {
		/* IN endpoint FIFO holds bytes */
		ep_info(ep, " FIFO holds bytes\n");
		retval = -EAGAIN;
		goto done;
	}

	/* endpoint set/clear halt */
	ep_set_halt(ep, value);
done:
	ep_dbg(ep, "%s, retval=%d\n", value ? "set" : "clear", retval);
	return retval;
}
#endif
/**
 * im98xx_ep_fifo_status - Get how many bytes in physical endpoint
 * @_ep: usb endpoint
 *
 * Returns number of bytes in OUT fifos. Broken for IN fifos.
 */
static int im98xx_ep_fifo_status(struct usb_ep *_ep)
{
	struct im98xx_ep		*ep;

	if (!_ep)
		return -ENODEV;
	ep = to_im98xx_ep(_ep);
	if (!ep->udc->usb_cable_sensed || !ep || is_ep0(ep))
		return -ENODEV;
	return ep_count_bytes_remain(ep);
}

/**
 * im98xx_ep_fifo_flush - Flushes one endpoint
 * @_ep: usb endpoint
 *
 * Discards all data in one endpoint(IN or OUT), except control endpoint.
 */
static void im98xx_ep_fifo_flush(struct usb_ep *_ep)
{
#if 1
	return ;
#else
	struct im98xx_ep		*ep;
	unsigned long		flags;

	if (!_ep)
		return;
	ep = to_im98xx_ep(_ep);
	if (!ep || is_ep0(ep))
		return;


	if (unlikely(!list_empty(&ep->queue)))
		ep_info(ep, "called while queue list not empty\n");
	ep_dbg(ep, "called\n");

	/* for OUT, just read and discard the FIFO contents. */
	if (ep->udc->usb_cable_sensed) {
		while (!ep_is_empty(ep)) {
			udc_ep_writeb(ep, EP_RSP_SC, 
				(udc_ep_readb(ep, EP_RSP_SC) | EP_BUF_FLUSH));
			ep_dbg(ep, "Flush buffer\n");
		}
	}


	return;
#endif
}

/**
 * im98xx_ep_enable - Enables usb endpoint
 * @_ep: usb endpoint
 * @desc: usb endpoint descriptor
 *
 * Nothing much to do here, as ep configuration is done once and for all
 * before udc is enabled. After udc enable, no physical endpoint configuration
 * can be changed.
 * Function makes sanity checks and flushes the endpoint.
 */
static int im98xx_ep_enable(struct usb_ep *_ep,
	const struct usb_endpoint_descriptor *desc)
{
	struct im98xx_ep		*ep;
	struct im98xx_udc		*udc;
	int retval = 0;
	unsigned char epnum;

	if (!_ep || !desc) {
		return -EINVAL;
	}

	ep = to_im98xx_ep(_ep);

	/* catch various bogus parameters */
	if (!ep || is_ep0(ep))
		return -EINVAL;

	if ((desc->bDescriptorType != USB_DT_ENDPOINT)) {
		ep_info(ep, "type mismatch\n");
		return -EINVAL;
	}

	if (ep->fifo_size < le16_to_cpu(desc->wMaxPacketSize)) {
		ep_info(ep, "bad maxpacket\n");
		return -ERANGE;
	}

	udc = ep->udc;
	if (!udc->driver || (udc->gadget.speed == USB_SPEED_UNKNOWN) || (ep_configured == -1)) {
		return -ESHUTDOWN;
	}

	ep_dbg(ep, "start ep->desc: %p, desc: %p\n", ep->desc, desc);
	ep->desc = desc;

	epnum = (u8)desc->bEndpointAddress & 0xF;

	retval = im98xx_ep_setup(ep, epnum, desc);
	if (retval != 0) {
		dev_info(udc->dev, "setup ep%d failed\n", epnum);
		return -ESHUTDOWN;
	}
	ep->enabled = 1;
	ep_dbg(ep, "end, driver: %p, speed: %d\n", udc->driver, udc->gadget.speed);
	return 0;
}

/**
 * im98xx_ep_disable - Disable usb endpoint
 * @_ep: usb endpoint
 *
 * Same as for im98xx_ep_enable, no physical endpoint configuration can be
 * changed.
 * Function flushes the endpoint and related requests.
 */
static int im98xx_ep_disable(struct usb_ep *_ep)
{
	struct im98xx_ep		*ep;

	if (!_ep)
		return -EINVAL;

	ep = to_im98xx_ep(_ep);

	if (!ep || !list_empty(&ep->queue))
		ep_info(ep, " queue not empty\n");
	/*if (!ep || is_ep0(ep) || !list_empty(&ep->queue))
		return -EINVAL;*/
	if (!ep->desc) {
		printk("%s, %s not enabled\n", __func__,
			_ep ? ep->usb_ep.name : NULL);
		return -EINVAL;
	}

	ep->enabled = 0;
	nuke(ep, -ESHUTDOWN);

	//im98xx_ep_fifo_flush(_ep);

	ep->desc = NULL;
	ep_dbg(ep, "end %p\n", ep->desc);
	return 0;
}

static struct usb_ep_ops im98xx_ep_ops = {
	.enable		= im98xx_ep_enable,
	.disable	= im98xx_ep_disable,

	.alloc_request	= im98xx_ep_alloc_request,
	.free_request	= im98xx_ep_free_request,

	.queue		= im98xx_ep_queue,
	.dequeue	= im98xx_ep_dequeue,

	.set_halt	= im98xx_ep_set_halt,
	.fifo_status	= im98xx_ep_fifo_status,
	.fifo_flush	= im98xx_ep_fifo_flush,
};

/**
 * dplus_pullup - Connect or disconnect pullup resistor to D+ pin
 * @udc: udc device
 * @on: 0 if disconnect pullup resistor, 1 otherwise
 * Context: any
 *
 * Handle D+ pullup resistor, make the device visible to the usb bus, and
 * declare it as a full speed usb device
 */
static void dplus_pullup(struct im98xx_udc *udc, int on)
{
	u8 v;

	if (unlikely(!udc->usb_cable_sensed))
		return;

	v = udc_readb(udc, USB_OPER);
	if (on) {
		v |= SOFT_CONN;
		udc_writeb(udc, USB_OPER, v);
	} else {
		v &= ~SOFT_CONN;
		udc_writeb(udc, USB_OPER, v);
	}
	udc->pullup_on = on;
}

/**
 * im98xx_udc_get_frame - Returns usb frame number
 * @_gadget: usb gadget
 */
static int im98xx_udc_get_frame(struct usb_gadget *_gadget)
{
	struct im98xx_udc *udc = to_im98xx_udc(_gadget);

	return ((udc_readw(udc, USB_FRAME_CNT) & 0x3fff) >> 3);
}

/**
 * im98xx_udc_wakeup - Force udc device out of suspend
 * @_gadget: usb gadget
 *
 * Returns 0 if succesfull, error code otherwise
 */
#if 0
static int im98xx_udc_wakeup(struct usb_gadget *_gadget)
{
	//struct im98xx_udc *udc = to_im98xx_udc(_gadget);

	return 0;
}
#endif
static void udc_enable(struct im98xx_udc *udc);
static void udc_disable(struct im98xx_udc *udc);
static void pullup_udc_enable(struct im98xx_udc *udc);
static void pullup_udc_disable(struct im98xx_udc *udc);

/**
 * should_enable_udc - Tells if UDC should be enabled
 * @udc: udc device
 * Context: any
 *
 * The UDC should be enabled if :

 *  - the pullup resistor is connected
 *  - and a gadget driver is bound
 *  - and vbus is sensed (or no vbus sense is available)
 *
 * Returns 1 if UDC should be enabled, 0 otherwise
 */
static int should_enable_udc(struct im98xx_udc *udc)
{
#if 0
	int put_on = udc->pullup_on;
#else //bob
	int put_on = udc->driver && udc->usb_cable_sensed;
#endif
	return put_on;
}

#if 0
/**
 * should_disable_udc - Tells if UDC should be disabled
 * @udc: udc device
 * Context: any
 *
 * The UDC should be disabled if :
 *  - the pullup resistor is not connected
 *  - or no gadget driver is bound
 *  - or no vbus is sensed (when vbus sesing is available)
 *
 * Returns 1 if UDC should be disabled
 */
static int should_disable_udc(struct im98xx_udc *udc)
{
	int put_off = 0;

	put_off = ((!udc->pullup_on) || (!udc->driver));

	return put_off;
}
#endif
/**
 * im98xx_udc_pullup - Offer manual D+ pullup control
 * @_gadget: usb gadget using the control
 * @is_active: 0 if disconnect, else connect D+ pullup resistor
 * Context: !in_interrupt()
 *
 * Returns 0 if OK, -EOPNOTSUPP if udc driver doesn't handle D+ pullup
 */
static int im98xx_udc_pullup(struct usb_gadget *_gadget, int is_active)
{
	struct im98xx_udc *udc = to_im98xx_udc(_gadget);
	
	dev_dbg(udc->dev, "%s enter*****************\n", __FUNCTION__);
	

	ep_configured = -1;
	usb_cable_status(udc);
	if (is_active)
		pullup_udc_enable(udc);

	dplus_pullup(udc, is_active);

	if (!is_active) {
		stop_activity(udc, udc->driver);
		pullup_udc_disable(udc);
	}

	return 0;
}

static int im98xx_udc_start(struct usb_gadget *gadget, struct usb_gadget_driver *driver);
static int im98xx_udc_stop(struct usb_gadget *gadget, struct usb_gadget_driver *driver);
static const struct usb_gadget_ops im98xx_udc_ops = {
	.get_frame	= im98xx_udc_get_frame,
	//.wakeup		= im98xx_udc_wakeup,
	.pullup		= im98xx_udc_pullup,
	.udc_start	= im98xx_udc_start,
	.udc_stop	= im98xx_udc_stop,
};

void pmuvolctl(struct im98xx_udc *udc, u32 select_ctl_dev, u32 vol_sel)
{
	u32 v;

	switch(select_ctl_dev) {
	case DEV_TYPE_VUSB: {
		v = readl(ABB_PMU_LDOVS4_REG);
		v &= ~VUSB_SEL_MASK;
		v |= vol_sel;
		writel(v, ABB_PMU_LDOVS4_REG);
		break;
	}
	}
}
void config_usb_pll(struct im98xx_udc *udc)
{
	u32 v;

#if 1
	v = (USB_PLL_DIVR|USB_PLL_DIVF|USB_PLL_DIVQ|USB_PLL_BS|
			USB_PLL_FSEL|USB_PLL_PD);
	writel(v, USB_PLL_REG);
	udelay(10);

	/* De-assert RESET */
	v = readl(USB_PLL_REG) & 0xffffff;
	writel(v, USB_PLL_REG);

	/* Wait PLL stable */
	do {
		v = readl(USB_PLL_REG);
	} while((v & 0x4000000) == 0);
#else
	clk_enable(udc->clk);
#endif	
	v = (0x0 << 16)|  // 0:/13 , 1:/26
		(0x1 << 14)|  // 0:disable, 1: enable
		(0x0 <<  4);  // 0:run , 1: suspend
	writel(v, USBDY_CLK_CTL_REG);

}

void disable_usb_pll(struct im98xx_udc *udc)
{
	u32 v;


	v = (0x0 << 16)|  // 0:/13 , 1:/26
	    (0x0 << 14)|  // 0:disable, 1: enable
	    (0x1 <<  4);  // 0:run , 1: suspend
	writel(v, USBDY_CLK_CTL_REG);
	udelay(10);

#if 1        	
	//power off PLL
	v = (USB_PLL_DIVR|USB_PLL_DIVF|USB_PLL_DIVQ|USB_PLL_BS|
			USB_PLL_FSEL|USB_PLL_PD);
	writel(v, USB_PLL_REG);
	udelay(10);
#else
	clk_disable(udc->clk);
#endif
	dev_dbg(udc->dev, "%s: usb pll disabled\n", __FUNCTION__);

}
/**
 * usb_clock_enable -- Enable USB Device Clock 
 *
 * @udc: struct im98xx_udc *udc
 *
 **/
void usb_clock_enable(struct im98xx_udc *udc)
{
	int v;

	v = readl(A9_APB_GCLK_REG);
	v &= ~0x08;
	writel(v, A9_APB_GCLK_REG);
}

void usb_pll_clk_init(struct im98xx_udc *udc)
{
	config_usb_pll(udc);
	usb_clock_enable(udc);
}

void usb_device_reset(struct im98xx_udc *udc)
{
	int v;

	/* Assert USB Device Reset */
	v = readl(SYS_ARM9_RST_REG);
	v |= 0x02;
	writel(v, SYS_ARM9_RST_REG);

	udelay(500);

	/* Release USB Device Reset */
	v = readl(SYS_ARM9_RST_REG);
	v &= ~0x02;
	writel(v, SYS_ARM9_RST_REG);
	v = readl(SYS_ARM9_RST_REG);

	dev_dbg(udc->dev, "%s: passed\n", __FUNCTION__);

}

void usbip_resetinit(struct im98xx_udc *udc)
{
	u8 v;

	udc_writeb(udc, USB_ADDR, 0x00);

	v = udc_readb(udc, USB_OPER);
	v |= SOFT_CONN;
	udc_writeb(udc, USB_OPER, v);

	//udc_writew(udc, DMA_CTRL_STS, DMA_ABORT);

	dev_info(udc->dev, "%s: passed\n", __FUNCTION__);
}

/**
 * udc_enable - Enables the udc device
 * @dev: udc device
 *
 * Enables the udc device : enables clocks, udc interrupts, control endpoint
 * interrupts, sets usb as UDC client and setups endpoints.
 */
static void udc_enable(struct im98xx_udc *udc)
{
	u8 usb_irq_enb = 0;

	if (udc->enabled) {
		return;
	}
	
	dev_dbg(udc->dev, "%s enter*****************\n", __FUNCTION__);
	//clk_enable(udc->clk);
	ep0_idle(udc);
	memset(&udc->stats, 0, sizeof(udc->stats));
	pmuvolctl(udc, DEV_TYPE_VUSB, VUSB_SEL_3_2V);
	usb_pll_clk_init(udc);
	usb_device_reset(udc);
	/* enable ep0 irqs */
#if 1
	pio_irq_enable(&udc->im98xx_ep[0]);
	usb_irq_enb = udc_readb(udc, USB_IRQ_ENB);
	usb_irq_enb &= ~(SPNT_INT_EN | USEBLE_CLK_EN);
	udc_writeb(udc, USB_IRQ_ENB, usb_irq_enb);
#endif
	im98xx_ep0_setup(&udc->im98xx_ep[0]);
	dplus_pullup(udc, 1);

	udc->enabled = 1;
	dev_dbg(udc->dev, "%s: passed\n", __FUNCTION__);
}

static void pullup_udc_enable(struct im98xx_udc *udc)
{
	u8 usb_irq_enb = 0;

	dev_dbg(udc->dev, "%s enter\n", __FUNCTION__);
	if (unlikely(!udc->usb_cable_sensed)) {
		return;
	}

	ep0_idle(udc);
	memset(&udc->stats, 0, sizeof(udc->stats));
	pmuvolctl(udc, DEV_TYPE_VUSB, VUSB_SEL_3_2V);
	usb_device_reset(udc);
	/* enable ep0 irqs */
	pio_irq_enable(&udc->im98xx_ep[0]);
	usb_irq_enb = udc_readb(udc, USB_IRQ_ENB);
	usb_irq_enb &= ~(SPNT_INT_EN | USEBLE_CLK_EN);
	udc_writeb(udc, USB_IRQ_ENB, usb_irq_enb);
	im98xx_ep0_setup(&udc->im98xx_ep[0]);

	dev_dbg(udc->dev, "%s: passed\n", __FUNCTION__);
}


/**
 * udc_disable - disable udc device controller
 * @udc: udc device
 * Context: any
 *
 * Disables the udc device : disables clocks, udc interrupts, control endpoint
 * interrupts.
 */
static void udc_disable(struct im98xx_udc *udc)
{

	dev_dbg(udc->dev, "%s enter\n", __FUNCTION__);
	if (!udc->enabled) {
		return;
	}
	ep0_idle(udc);
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	udc->enabled = 0;
	usb_device_reset(udc);
	udc_writeb(udc, USB_IRQ_ENB, 0);
	dev_dbg(udc->dev, "USB_IRQ_ENB: 0x%02x, IRQ_ENB_L: 0x%04x\n", udc_readb(udc, USB_IRQ_ENB), udc_readw(udc, IRQ_ENB_L));
	dev_info(udc->dev, "put USB DEVICE PHY into suspend mode\n");
#if 1
	udc_writeb(udc, USB_OPER, 0xa);
	udelay(50);  //wait USB PHY to enter suspend mode
#else
	udc_writeb(udc, USB_OPER, 0x2);  //from Jan and ic team
	udc_writeb(udc, USB_ADDR, 0x0);
	udc_writew(udc, CEP_START_ADDR, 0x0);
	udc_writew(udc, CEP_END_ADDR, 0x3f);
	udc_writew(udc, IRQ_ENB_L, 0x3);
	udc_writew(udc, IRQ_ENB_H, 0x0);
	udc_writeb(udc, USB_IRQ_ENB, 0x40);
	udc_writeb(udc, USB_IRQ_ENB, 0x72);
	udc_writew(udc, CEP_IRQ_ENB, 0x46E);
	udc_writeb(udc, USB_OPER, 0x2);
	udc_writeb(udc, USB_OPER, 0xA);
	udelay(80);
	writel(readl(GPIO_OUT_REG) | (1 << 5),
              GPIO_OUT_REG);
	udelay(10);
	mdelay(2);

	writel(readl(GPIO_OUT_REG) | (1 << 0),
              GPIO_OUT_REG);
#endif
	disable_usb_pll(udc);
	pmuvolctl(udc, DEV_TYPE_VUSB, 0);

	dev_dbg(udc->dev, "%s: passed\n", __FUNCTION__);
}

static void pullup_udc_disable(struct im98xx_udc *udc)
{

	dev_dbg(udc->dev, "%s enter\n", __FUNCTION__);
	if (unlikely(!udc->usb_cable_sensed)) {
		return;
	}
	ep0_idle(udc);
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	usb_device_reset(udc);
	udc_writeb(udc, USB_IRQ_ENB, 0);
	pmuvolctl(udc, DEV_TYPE_VUSB, 0);
	dev_dbg(udc->dev, "%s: passed\n", __FUNCTION__);
}

/**
 * udc_init_data - Initialize udc device data structures
 * @dev: udc device
 *
 * Initializes gadget endpoint list, endpoints locks. No action is taken
 * on the hardware.
 */
static __init void udc_init_data(struct im98xx_udc *udc)
{
	int i;
	struct im98xx_ep *ep;
	struct im98xx_udc_data *udc_data = udc->udc_data;
	struct im98xx_ep_data *ep_data;

	/* here comes the stand operations for probe */
	udc->gadget.ops = &im98xx_udc_ops;

	/* gadget.ep0 is a pointer */
	udc->gadget.ep0 = &udc->im98xx_ep[0].usb_ep;

	/* name: Identifies the controller hardware type. */
	udc->gadget.name = driver_name;


	/* device/ep0 records init */
	INIT_LIST_HEAD(&udc->gadget.ep_list);
	INIT_LIST_HEAD(&udc->gadget.ep0->ep_list);

	/* obtain GPIO number for USB cable detection */
	udc->usb_cable_gpio = udc_data->pdata.usb_cable_gpio;
	udc->devstatus = 1 << USB_DEVICE_SELF_POWERED; /* Self powered */
	udc->devstatus |= 1 << USB_DEVICE_REMOTE_WAKEUP; /* Remote wakeup */

	//memcpy(udc_data->ep, udc_data->pdata.ep, udc_data->pdata.num_ep*(sizeof(struct im98xx_ep_data)));

	/* im98xx endpoints init */
	for (i = 0; i < NR_IM98XX_UDC_ENDPOINTS; i++) {
		ep = &udc->im98xx_ep[i];
		ep_data = &udc_data->ep[i];
		ep->udc = udc;
		ep->addr = i;
		ep->name = ep_data->name;
		ep->mps = udc_data->ep[i].mps;
		ep->fifo_size = udc_data->ep[i].fifo_size;
		ep->ep_start_addr = udc_data->ep[i].fifo_start;
		ep->ep_end_addr = udc_data->ep[i].fifo_start + 
			(udc_data->ep[i].fifo_size / 2) - 1;
		ep->can_dma = udc_data->ep[i].can_dma;
		ep->type = udc_data->ep[i].type;
		ep->ep_in_mod = udc_data->ep[i].ep_in_mod;
		ep->ep_trsactn_num = udc_data->ep[i].ep_trsactn_num;
		ep->usb_ep.name = udc_data->ep[i].name;
		ep->usb_ep.ops = &im98xx_ep_ops;
		ep->usb_ep.maxpacket = udc_data->ep[i].mps;

		dev_dbg(udc->dev, "%s: i = %d, ep->mps = %d \n", __FUNCTION__, i, ep->mps);
		/* the queue lists any req for this ep */
		ep->enabled = is_ep0(ep);
		INIT_LIST_HEAD(&ep->queue);
		spin_lock_init(&ep->lock);
		
		/* gagdet.ep_list used for ep_autoconfig so no ep0*/
		if (i != 0)
			list_add_tail(&udc->im98xx_ep[i].usb_ep.ep_list,
				&udc->gadget.ep_list);
	}

	ep0_idle(udc);
	
}

 static int im98xx_udc_start(struct usb_gadget *gadget, struct usb_gadget_driver *driver)
 {
	 struct im98xx_udc *udc = the_controller;
 
 	dev_dbg(udc->dev, "%s enter*****************\n", __FUNCTION__);
	 /* first hook up the driver ... */
	 udc->driver = driver;
 
	 dev_dbg(udc->dev, "registered gadget function '%s'\n",
		 driver->function);
 
	 if (should_enable_udc(udc))
		 udc_enable(udc);
	 return 0;
 }

 static int im98xx_udc_stop(struct usb_gadget *gadget, struct usb_gadget_driver *driver)
 {
	 struct im98xx_udc *udc = the_controller;
  	
	dev_dbg(udc->dev, "%s enter*****************\n", __FUNCTION__);
 	
	 if (!udc)
		 return -ENODEV;
	 if (!driver || driver != udc->driver || !driver->unbind)
		 return -EINVAL;
 
	 stop_activity(udc, driver);
	 udc_disable(udc);
 #if 0 //try 0
	 driver->disconnect(&udc->gadget);
	 driver->unbind(&udc->gadget);
 #endif
	 udc->driver = NULL;
 
	 /*
	 dev_info(udc->dev, "unregistered gadget driver '%s'\n",
		  driver->driver.name);
	 */
	 return 0;
 }

/**
 * stop_activity - Stops udc endpoints
 * @udc: udc device
 * @driver: gadget driver
 *
 * Disables all udc endpoints (even control endpoint), report disconnect to
 * the gadget user.
 */
static void stop_activity(struct im98xx_udc *udc, struct usb_gadget_driver *driver)
{
	dev_dbg(udc->dev, "%s<------\n", __FUNCTION__);

	/* don't disconnect drivers more than once */
	if (udc->gadget.speed == USB_SPEED_UNKNOWN)
		driver = NULL;
	udc->gadget.speed = USB_SPEED_UNKNOWN;

	/* Clear remaining REQs on all the EPs */
	clear_reqs(udc, -ESHUTDOWN);

	if (driver) {
		dev_dbg(udc->dev, "driver->disconnect()\n");
		driver->disconnect(&udc->gadget);
	}

	dev_dbg(udc->dev, "%s------>\n", __FUNCTION__);
}

/**
 * clear_reqs - clear reqs queued on all the EPs
 * @udc: udc device
 * @status: the status of req handed over to the up layer
 *
 * Clear up all the reqs queued on all the EPs while the USB reset occurs.
 *
 */
static void clear_reqs(struct im98xx_udc *udc, int status)
{
	int i;

	dev_dbg(udc->dev, "%s<------\n", __FUNCTION__);
	for (i = 0; i < NR_IM98XX_UDC_ENDPOINTS; i++) {
		nuke(&udc->im98xx_ep[i], status);
	}
	dev_dbg(udc->dev, "%s------>\n", __FUNCTION__);
}


/* Called with interrupts disabled and udc->lock held */
static inline void set_protocol_stall(struct im98xx_udc *udc, struct im98xx_ep *ep)
{
	//usba_ep_writel(ep, SET_STA, USBA_FORCE_STALL);
	//ep->state = WAIT_FOR_SETUP;
}

/* return whether endpoint is stalled, 0: not stalled; 1: stalled */
static int ep_is_stall(struct im98xx_ep *ep)
{
	int			retval;

	if (is_ep0(ep)) {
		retval = (udc_readb(ep->udc, CEP_CTRL_STAT) & STALL) ? 1 : 0;
	} else {
		retval = (udc_ep_readb(ep, EP_RSP_SC) & EP_HALT) ? 1 : 0;
	}
	ep_info(ep, " %d\n", retval);
	return retval;
}


static inline void set_address(struct im98xx_udc *udc, unsigned int addr)
{
	u8 address = 0;

	address = (addr & 0x7F);
	udc_writeb(udc, USB_ADDR, address);
}

static struct im98xx_ep *get_ep_by_windex(struct im98xx_udc *udc, u16 wIndex)
{
	struct im98xx_ep *ep;

	if ((wIndex & USB_ENDPOINT_NUMBER_MASK) == 0)
		return to_im98xx_ep(udc->gadget.ep0);

	list_for_each_entry (ep, &udc->gadget.ep_list, usb_ep.ep_list) {
		u8 bEndpointAddress;

		if (!ep->desc)
			continue;
		bEndpointAddress = ep->desc->bEndpointAddress;
		if ((wIndex ^ bEndpointAddress) & USB_DIR_IN)
			continue;
		if ((bEndpointAddress & USB_ENDPOINT_NUMBER_MASK)
				== (wIndex & USB_ENDPOINT_NUMBER_MASK))
			return ep;
	}

	return NULL;
}

/* Avoid overly long expressions */
static inline bool feature_is_dev_remote_wakeup(struct usb_ctrlrequest *crq)
{
	if (crq->wValue == cpu_to_le16(USB_DEVICE_REMOTE_WAKEUP))
		return true;
	return false;
}

static inline bool feature_is_ep_halt(struct usb_ctrlrequest *crq)
{
	if (crq->wValue == cpu_to_le16(USB_ENDPOINT_HALT))
		return true;
	return false;
}

static inline bool feature_is_dev_test_mode(struct usb_ctrlrequest *crq)
{
	if (crq->wValue == cpu_to_le16(USB_DEVICE_TEST_MODE))
		return true;
	return false;
}

static void req_get_status_complete(struct usb_ep *ep, struct usb_request *req)
{
}

static int handle_ep0_setup(struct im98xx_udc *udc, struct im98xx_ep *ep,
		struct usb_ctrlrequest *crq)
{
	unsigned int usb_address = 0;
	int retval = 0;

	switch (crq->bRequest) {
	case USB_REQ_GET_STATUS: {
		/* get status, DATA and STATUS phase */
		u16 status_data = 0;    /* 16 bits cpu view status data */
		if (crq->bRequestType == (USB_DIR_IN | USB_RECIP_DEVICE)) {
			/* get device status */
			status_data = cpu_to_le16(udc->devstatus);
			dev_dbg(udc->dev, "get device status: 0x%x\n", status_data);
		} else if (crq->bRequestType
				== (USB_DIR_IN | USB_RECIP_INTERFACE)) {
			/* get interface status */
			dev_dbg(udc->dev, "get interface status\n");
			status_data = 0;
		} else if (crq->bRequestType
				== (USB_DIR_IN | USB_RECIP_ENDPOINT)) {
			/* get endpoint status */
			struct im98xx_ep  *epn;

			dev_dbg(udc->dev, "get endpoint status\n");
			epn = get_ep_by_windex(udc, le16_to_cpu(crq->wIndex));
			/* stall if endpoint doesn't exist */
			if (!epn)
				goto stall;

			status_data = ep_is_stall(epn) << USB_ENDPOINT_HALT;

		} else
			goto delegate;

		if (crq->wLength != cpu_to_le16(sizeof(status_data)))
			goto stall;

		udc->ep0_data = cpu_to_le16(status_data);
		udc->ep0_req->buf = &udc->ep0_data;
		udc->ep0_req->length = 2;
		udc->ep0_req->actual = 0;
		im98xx_ep_queue(udc->gadget.ep0, udc->ep0_req, GFP_KERNEL);
		if (retval < 0) {
			dev_info(udc->dev, "Failed: GET_STATUS queue: %d\n", retval);
			udc->ep0_req->status = 0;
		}
		break;
	}

	case USB_REQ_CLEAR_FEATURE: {
		/* only STATUS phase, no data phase*/
		if (crq->bRequestType == USB_RECIP_DEVICE) {
			dev_dbg(udc->dev, "clear device feature\n");
			if (feature_is_dev_remote_wakeup(crq)) {
				udc->devstatus &= ~(1 << USB_DEVICE_REMOTE_WAKEUP);
				ep0_nak_clr(ep);
			} else
				goto stall;
		} else if (crq->bRequestType == USB_RECIP_ENDPOINT) {
			struct im98xx_ep *epn;

			dev_dbg(udc->dev, "clear endpoint feature\n");
			if (crq->wLength != cpu_to_le16(0)
					|| !feature_is_ep_halt(crq))
				goto stall;
			epn = get_ep_by_windex(udc, le16_to_cpu(crq->wIndex));
			if (!epn)
				goto stall;
			retval = im98xx_ep_set_halt(&epn->usb_ep, 0);
			if (retval < 0)
				goto stall;
			else {
				dev_info(udc->dev, "clear endpoint feature successfully\n");
			}
		} else {
			goto delegate;
		}

		break;
	}

	case USB_REQ_SET_FEATURE: {
		if (crq->bRequestType == USB_RECIP_DEVICE) {
			dev_info(udc->dev, "set device feature\n");
			if (feature_is_dev_remote_wakeup(crq)) {
				udc->devstatus |= 1 << USB_DEVICE_REMOTE_WAKEUP;
				ep0_nak_clr(ep);
			} else {
				goto stall;
			}
		} else if (crq->bRequestType == USB_RECIP_ENDPOINT) {
			struct im98xx_ep *epn;

			dev_info(udc->dev, "set endpoint feature\n");
			if (crq->wLength != cpu_to_le16(0)
					|| !feature_is_ep_halt(crq))
				goto stall;

			epn = get_ep_by_windex(udc, le16_to_cpu(crq->wIndex));
			if (!epn)
				goto stall;

			retval = im98xx_ep_set_halt(&epn->usb_ep, 1);
			if (retval < 0)
				goto stall;
			else {
				dev_info(udc->dev, "set endpoint feature successfully\n");
			}
		} else
			goto delegate;

		break;
	}

	case USB_REQ_SET_ADDRESS:
		if (crq->bRequestType != (USB_DIR_OUT | USB_RECIP_DEVICE))
			goto delegate;

		/* The NAKCLR bit must be cleared before the new address is set up , or the UDC will 
			always responds NAK for the incoming IN token */
		ep0_nak_clr(ep);

		/* write address delayed (will take effect in the next SETUP) */
		usb_address = le16_to_cpu(crq->wValue);
		printk("setting address %u...\n", usb_address);
		set_address(udc, usb_address);
		break;

	default:
delegate:
		/* pass request up to the gadget driver */
		if (udc->driver){
			retval = udc->driver->setup(&udc->gadget, crq);
			if(retval)dev_info(udc->dev, "%s %d\n",__func__,retval);
		}else
			retval = -ENODEV;
	}
	return retval;

stall:
	ep_dbg(ep, "Invalid setup request: %02x.%02x v%04x i%04x l%d, Stall...\n",
		crq->bRequestType, crq->bRequest,
		le16_to_cpu(crq->wValue), le16_to_cpu(crq->wIndex),
		le16_to_cpu(crq->wLength));
	return -1;
}

/**
 * handle_ep0_ctrl_req - handle control endpoint control request
 * @udc: udc device
 * @req: control request
 */
static void handle_ep0_ctrl_req(struct im98xx_udc *udc)
{
	struct im98xx_ep *ep = &udc->im98xx_ep[0];
	u8 cep_ctl_stat = 0;
	union {
		struct usb_ctrlrequest	r;
		u16			setup_8bytes[4];
	} u;
	int retval = 0;

	/* read SETUP packet */
	u.setup_8bytes[0] = udc_readw(ep->udc, SETUP1_0);
	u.setup_8bytes[1] = udc_readw(ep->udc, SETUP3_2);
	u.setup_8bytes[2] = udc_readw(ep->udc, SETUP5_4);
	u.setup_8bytes[3] = udc_readw(ep->udc, SETUP7_6);

	ep_dbg(ep, "SETUP %02x.%02x v%04x i%04x l%04x\n",
		u.r.bRequestType, u.r.bRequest,
		le16_to_cpu(u.r.wValue), le16_to_cpu(u.r.wIndex),
		le16_to_cpu(u.r.wLength));

	if (u.r.bRequestType & USB_DIR_IN) {
		if (u.r.wLength != cpu_to_le16(0))
			set_ep0state(udc, IN_DATA_STAGE);
		else
			set_ep0state(udc, IN_STATUS_STAGE);  //fix me
	} else {
		if (u.r.wLength != cpu_to_le16(0))			
			set_ep0state(udc, OUT_DATA_STAGE);
		else {
			if (u.r.bRequest == cpu_to_le16(USB_REQ_SET_CONFIGURATION)) {
				ep_dbg(ep, "SET_CONFIGURATION start\n");
			}
			set_ep0state(udc, OUT_STATUS_STAGE);
		}
	}
#if 0
	retval = udc->driver->setup(&udc->gadget, &u.r);
	if (retval < 0)
		goto stall;
#else
	retval = handle_ep0_setup(udc, ep, &u.r);

	if (retval < 0) {
		dev_dbg(udc->dev, "handle_ep0_setup error\n");
		if (ep->udc->gadget.speed == USB_SPEED_UNKNOWN) {
			dev_dbg(udc->dev, "goto out\n");
			goto out;
		} else {
			dev_dbg(udc->dev, "goto stall\n");
			goto stall;
		}
    }
#endif
out:
	return;
stall:
	ep_info(ep, "protocol STALL, err %d\n", retval);
	set_ep0state(udc, UDC_STALL);
	/* STALL handshake needs to be sent to the host */
	cep_ctl_stat = udc_readb(ep->udc, CEP_CTRL_STAT);
	cep_ctl_stat &= ~NAK_CLR;
	cep_ctl_stat |= STALL;
	udc_writeb(ep->udc, CEP_CTRL_STAT, cep_ctl_stat);
	goto out;
}

/**
 * handle_ep0 - Handle control endpoint data transfers
 * @udc: udc device
 *
 * Context : when in_interrupt() or with ep->lock held
 *
 * Tries to transfer all pending request data into the endpoint and/or
 * transfer all pending data in the endpoint into usb requests.
 * Handles states of ep0 automata.
 *
 */
static void handle_ep0(struct im98xx_udc *udc, u16 cep_irq_stat)
{
	struct im98xx_ep		*ep = &udc->im98xx_ep[0];
	struct im98xx_request	*req = NULL;
	u16 cep_irq_enb = 0;

	if (!list_empty(&ep->queue))
		req = list_entry(ep->queue.next, struct im98xx_request, queue);

	//ep_dbg(ep, "state=%s, req=%p, cep_irq_stat=0x%x, CEP_IRQ_STAT=%x\n",
		//EP0_STNAME(udc), req, cep_irq_stat, udc_readw(udc, CEP_IRQ_STAT));

	if (cep_irq_stat & CEP_USB_ERR) {
		ep_dbg(ep, "USB Error occurred\n");
		/*nuke(ep, -EPIPE);
		ep0_idle(udc);
		irq_udc_suspend(ep->udc, 1000);  //1s
		return;*/
	}

	if (udc_readw(udc, CEP_IRQ_STAT) & CEP_BUF_FULL_INT_EN) {
		ep_dbg(ep, "CEP Buf full\n");
		ep->cep_buf_full = 1;
	}
	if (cep_irq_stat & CEP_STALL) {
		ep_info(ep, "STALL token sent\n");
		nuke(ep, -EPIPE);
		ep0_idle(udc);
	}
	if (cep_irq_stat & CEP_DMA_TIMEOUT_INT_EN) {
		ep_dbg(ep, "CEP DMA Timeout\n");
		ep->cep_dma_timeout = 1;
	}
	if (cep_irq_stat & CEP_SETUP_TOKEN_INT_EN) {
		ep_dbg(ep, "SETUP token received--for debug only\n");
	}
	if (cep_irq_stat & CEP_DATAPKT_TRSMIT_INT_EN) {
		ep_dbg(ep, "data packet transmitted.\n");
		ep->cep_datapkt_transmitted = 1;
		ep->cep_buf_full = 0;
		if (req) {
			if (ep->is_last_pkt) {
				ep->is_last_pkt = 0;
				ep0_end_in_req(ep, req);
			} else if (!ep_is_full(ep)) {
				ep->is_last_pkt = write_ep0_fifo(ep, req);
			}
		} else {
			ep_info(ep, "IN: req==null, nak_clean\n");
			set_ep0state(ep->udc, IN_STATUS_STAGE);
		}
	}
	if (cep_irq_stat & CEP_STAUS_COMPET_INT_EN) {
		ep_dbg(ep, "Status stage compelete\n");
		ep0_idle(udc);
	}
	if (cep_irq_stat & CEP_DATAPKT_RECIVE_INT_EN) {
		ep_dbg(ep, "data packet received %d.\n", udc_readw(ep->udc, OUT_TRNSFR_CNT));
		ep->cep_datapkt_received = 1;
		if (req) {
			if (req->req.length == 0) {
				ep_dbg(ep, "length == 0\n");
				ep0_end_out_req(ep, req);
			} else {
				ep->is_last_pkt = read_ep0_fifo(ep, req);
				if (ep->is_last_pkt) {
					ep->is_last_pkt = 0;
					ep0_end_out_req(ep, req);
				}
			}
		} else {
			ep_dbg(ep, "OUT: req==null, nak_clean\n");
			set_ep0state(ep->udc, OUT_STATUS_STAGE);
		}
	}
	if (cep_irq_stat & CEP_SETUP_PKT_INT_EN) {
		ep_dbg(ep, "8 SETUPdata bytes received.\n");
		nuke(ep, 0);
		set_ep0state(udc, SETUP_STAGE);
	}
	switch (udc->ep0state) {
	case WAIT_FOR_SETUP:
		break;
	case SETUP_STAGE:
		handle_ep0_ctrl_req(udc);
		break;
	case IN_DATA_STAGE:
		break;
	case OUT_DATA_STAGE:
		break;
	case UDC_STALL:
		ep_dbg(ep, "UDC_STALL...\n");
		break;
	case IN_STATUS_STAGE:
		ep_dbg(ep, "IN_STATUS_STAGE\n");
		ep0_nak_clr(ep);
		break;
	case OUT_STATUS_STAGE:
		ep_dbg(ep, "OUT_STATUS_STAGE\n");
		cep_irq_enb = udc_readw(udc, CEP_IRQ_ENB);
		cep_irq_enb &= ~CEP_DATAPKT_RECIVE_INT_EN;
		udc_writew(udc, CEP_IRQ_ENB, cep_irq_enb);
		ep0_nak_clr(ep);
		break;
	case WAIT_ACK_SET_CONF_INTERF:
		ep_warn(ep, "should never get in %s state here!!!\n",
				EP0_STNAME(ep->udc));
		ep0_idle(udc);
		break;
	}
}

/**
 * handle_ep - Handle endpoint data tranfers
 * @ep: im98xx physical endpoint
 *
 * Tries to transfer all pending request data into the endpoint and/or
 * transfer all pending data in the endpoint into usb requests.
 *
 * Is always called when in_interrupt() or with ep->lock held.
 */
static void handle_ep(struct im98xx_ep *ep, u16 ep_irq_stat)
{
	struct im98xx_request	*req = NULL;
	int completed = 0;
	int is_in = ep->dir_in;

	if (udc_ep_readw(ep, EP_IRQ_STAT) & EP_USB_STALLSENT_INT_EN) {
		udc_ep_writew(ep, EP_IRQ_STAT, EP_USB_STALLSENT_INT_EN);
		ep_info(ep, "usb STALL sent\n");
	}

	if (udc_ep_readw(ep, EP_IRQ_STAT) & EP_ERR_INT_EN) {
		ep_dbg(ep, "usb_error\n");
		udc_ep_writew(ep, EP_IRQ_STAT, EP_ERR_INT_EN);
	}

	completed = 0;
	if (likely(!list_empty(&ep->queue)))
		req = list_entry(ep->queue.next,
				struct im98xx_request, queue);
	else
		req = NULL;

	if (!req) {
		ep_info(ep, "no REQ prepared, return\n");
		return;
	}

	if (is_in) {  //EP IN

		if (ep_irq_stat & EP_DATAPKT_TRANSMIT_INT_EN) {
			if (udc_ep_readw(ep, EP_DATA_CNT) & 0x7ff)
				ep_info(ep, "Error: DATA Transmit left = %d\n", (udc_ep_readw(ep, EP_DATA_CNT) & 0x7ff));
			//PIO mode, IN
			if (ep->is_last_pkt) {
				udc_ep_writew(ep, EP_IRQ_STAT, EP_DATAPKT_TRANSMIT_INT_EN);
				if (req->req.length == 0)
					ep_dbg(ep, "last OK -- EP_DATA_CNT=%d, EP_IRQ_STAT=0x%08x\n",
							(udc_ep_readw(ep, EP_DATA_CNT) & 0xfff), udc_ep_readw(ep, EP_IRQ_STAT));
				ep_end_in_req(ep, req);
				ep->is_last_pkt = 0;
			} else if (ep_is_empty(ep)) {
				udc_ep_writew(ep, EP_IRQ_STAT, EP_DATAPKT_TRANSMIT_INT_EN);
				//ep_dbg(ep, "OK -- EP_DATA_CNT=%d, EP_IRQ_STAT=0x%08x\n",
						//(udc_ep_readw(ep, EP_DATA_CNT) & 0xfff), udc_ep_readw(ep, EP_IRQ_STAT));
				ep->is_last_pkt = write_fifo(ep, req);
			}

		}

	} else { //EP OUT
		if (ep_irq_stat & EP_DATAPKT_RECEIVED_INT_EN) {
			if (likely((udc_ep_readw(ep, EP_DATA_CNT) & 0x7ff) <= req->req.length)) {
				udc_ep_writew(ep, EP_IRQ_STAT, EP_DATAPKT_RECEIVED_INT_EN);
				//ep_dbg(ep, "Data Packet received %d, req=%p\n", (udc_ep_readw(ep, EP_DATA_CNT) & 0x7ff), req);
			} else {
				ep_info(ep, "Data Packet received %d, req.len=%d\n", (udc_ep_readw(ep, EP_DATA_CNT) & 0x7ff), req->req.length);
			}
			//PIO mode, OUT
			completed = read_fifo(ep, req);
			//ep_dbg(ep, "completed=%d\n", completed);
			if (completed) {
				ep_end_out_req(ep, req);
			}

			if (unlikely((udc_ep_readw(ep, EP_DATA_CNT) & 0x7ff) != 0)
					&& ((udc_ep_readw(ep, EP_IRQ_STAT) & EP_DATAPKT_RECEIVED_INT_EN) == 0)) {
				ep_info(ep, "OUT_Error: completed = %d, EP_DATA_CNT=%d, EP_IRQ_STAT=0x%08x\n", completed,
					(udc_ep_readw(ep, EP_DATA_CNT) & 0x7ff), udc_ep_readw(ep, EP_IRQ_STAT));
			}
		}
	}
	//ep_dbg(ep, "\n\n");
}

#if 0 
/**
 * im98xx_change_configuration - Handle SET_CONF usb request notification
 * @udc: udc device
 * @config: usb configuration
 *
 * Post the request to upper level.
 * Don't use any im98xx specific harware configuration capabilities
 */
static void im98xx_change_configuration(struct im98xx_udc *udc, int config)
{
	struct usb_ctrlrequest req ;

	dev_dbg(udc->dev, "config=%d\n", config);

	udc->config = config;
	udc->last_interface = 0;
	udc->last_alternate = 0;

	req.bRequestType = 0;
	req.bRequest = USB_REQ_SET_CONFIGURATION;
	req.wValue = config;
	req.wIndex = 0;
	req.wLength = 0;

	set_ep0state(udc, WAIT_ACK_SET_CONF_INTERF);
	udc->driver->setup(&udc->gadget, &req);
}

/**
 * im98xx_change_interface - Handle SET_INTERF usb request notification
 * @udc: udc device
 * @iface: interface number
 * @alt: alternate setting number
 *
 * Post the request to upper level.
 * Don't use any im98xx specific harware configuration capabilities
 */
static void im98xx_change_interface(struct im98xx_udc *udc, int iface, int alt)
{
	struct usb_ctrlrequest  req;

	dev_dbg(udc->dev, "interface=%d, alternate setting=%d\n", iface, alt);

	udc->last_interface = iface;
	udc->last_alternate = alt;

	req.bRequestType = USB_RECIP_INTERFACE;
	req.bRequest = USB_REQ_SET_INTERFACE;
	req.wValue = alt;
	req.wIndex = iface;
	req.wLength = 0;

	set_ep0state(udc, WAIT_ACK_SET_CONF_INTERF);
	udc->driver->setup(&udc->gadget, &req);
}
#endif

/**
 * irq_udc_suspend - Handle IRQ "UDC Suspend"
 * @udc: udc device
 */
static inline void irq_udc_suspend(struct im98xx_udc *udc, int timeout)
{
	dev_dbg(udc->dev, "\n\n");
	dev_dbg(udc->dev, "irq_udc_suspend\n");

	if (unlikely(!udc->usb_cable_sensed)) {
		dev_info(udc->dev, "irq_udc_suspend, no cable, return\n");
		return;
	}
/*
	if (udc->gadget.speed != USB_SPEED_UNKNOWN
			&& udc->driver && udc->driver->suspend)
		udc->driver->suspend(&udc->gadget);
	ep0_idle(udc);

*/

	//udc_writeb(udc, USB_OPER, 0x3);
	usb_device_reset(udc);
	udc_writeb(udc, USB_IRQ_ENB, 0);
	udc_writew(udc, DMA_CTRL_STS, DMA_ABORT);
	ep_configured = -1;
	//copy from work queue
	im98xx_26MHz_halt(10000, GPT1);//im98xx_26MHz_halt(timeout*1000, GPT1);
	dplus_pullup(udc, 0);
	stop_activity(udc, udc->driver);
	pullup_udc_disable(udc);

	im98xx_26MHz_halt(10000, GPT1);//10ms

	pullup_udc_enable(udc);
	dplus_pullup(udc, 1);
	//copy end


	//udc_writeb(udc, USB_IRQ_STAT, SPNT_INT_EN);
	dev_dbg(udc->dev, "UDC suspend detected end\n");
}

/**
  * irq_udc_resume - Handle IRQ "UDC Resume"
  * @udc: udc device
  */
static void irq_udc_resume(struct im98xx_udc *udc)
{
	dev_dbg(udc->dev, "UDC Resume detected\n");

	irq_udc_suspend(udc, 500);  //0.5s
/*
	if (udc->gadget.speed != USB_SPEED_UNKNOWN
		&& udc->driver && udc->driver->resume)
	udc->driver->resume(&udc->gadget);
*/
}

/**
 * irq_udc_reset - Handle IRQ "UDC Reset"
 * @udc: udc device
 */
static void irq_udc_reset(struct im98xx_udc *udc)
{
	struct im98xx_ep *ep = &udc->im98xx_ep[0];
	u8 usb_irq_enb = 0;

	dev_info(udc->dev, "UDC reset speed: %d, irqs_reset = %lu\n", udc->gadget.speed, udc->stats.irqs_reset);

	if (udc->stats.irqs_reset == 0) {
		udc_writeb(udc, USB_OPER, 0xb);
		usb_irq_enb = udc_readb(udc, USB_IRQ_ENB);
		usb_irq_enb |= SPNT_INT_EN;
		udc_writeb(udc, USB_IRQ_ENB, usb_irq_enb);
        udc->gadget.speed = USB_SPEED_FULL;
	}
	ep_configured = 0;
	udc->stats.irqs_reset++;
	nuke(ep, -EPROTO);
	ep0_idle(udc);
}

/**
 * irq_udc_sof - Handle IRQ "UDC Start Of Frame"
 * @udc: udc device
 */
static void irq_udc_sof(struct im98xx_udc *udc)
{
	dev_info(udc->dev, "UDC Start Of Frame detected\n");
}

/**
 * irq_udc_hss - Handle IRQ "UDC High Speed Settle"
 * @udc: udc device
 */
static void irq_udc_hss(struct im98xx_udc *udc)
{
	dev_info(udc->dev, "UDC High Speed Settle detected\n");
	udc->gadget.speed = USB_SPEED_HIGH;
}


/**
 * irq_udc_dma_cmpet - Handle IRQ "UDC DMA compete"
 * @udc: udc device
 */
static void irq_udc_dma_cmpet(struct im98xx_udc *udc)
{
}

/**
 * irq_udc_usble_clk - Handle IRQ "UDC Usble Clock"
 * @udc: udc device
 */
static void irq_udc_usble_clk(struct im98xx_udc *udc)
{
	dev_info(udc->dev, "UDC Usble Clock detected speed: %d\n", udc->gadget.speed);
	udc->gadget.speed = USB_SPEED_FULL;
}

/*
 * irq_handle_data - Handle usb irq
 * @irq: irq IRQ number
 * @udc: dev im98xx_udc device structure
 *
 * Called from irq handler, transferts data to or from endpoint to queue
 */
static void irq_handle_usb(int irq, struct im98xx_udc *udc)
{
	u16	usb_irq_stat = udc_readb(udc, USB_IRQ_STAT);
	u16	usb_irq_enable = udc_readb(udc, USB_IRQ_ENB);

	dev_dbg(udc->dev, "Interrupt: USB_IRQ_STAT:0x%08x, usb_irq_enable:0x%08x\n", usb_irq_stat, usb_irq_enable);
	udc_writeb(udc, USB_IRQ_STAT, usb_irq_stat);
	/* Some interrupt flags will be triggered even the interrupt is disabled
		we just care the interrupt flags with its interrupt enabled */
	usb_irq_stat &= usb_irq_enable;
	if (usb_irq_stat & USB_IRQ_INT_MASK) {
		if (usb_irq_stat & RSUM_INT_EN) {
			dev_info(udc->dev, "Interrupt: Resume\n");
			irq_udc_resume(udc);
			return;
		}
		if (usb_irq_stat & SPNT_INT_EN) {
			dev_info(udc->dev, "Interrupt: Suspend\n");
			irq_udc_suspend(udc, 1000);  //1s
			return;
		}
		if (usb_irq_stat & SOF_INT_EN)
			irq_udc_sof(udc);
		if (usb_irq_stat & USEBLE_CLK_EN)
			irq_udc_usble_clk(udc);
		if (usb_irq_stat & HSS_INT_EN)
			irq_udc_hss(udc);
		if (usb_irq_stat & RST_INT_EN)
			irq_udc_reset(udc);
		if (usb_irq_stat & DMA_CMPET_INT_EN)
			irq_udc_dma_cmpet(udc);
	}
}

/*
 * irq_handle_data - Handle data transfer of cep
 * @irq: irq IRQ number
 * @udc: dev im98xx_udc device structure
 *
 * Called from irq handler, transferts data to or from control endpoint to queue
 */
static void irq_handle_cep(int irq, struct im98xx_udc *udc)
{
	u16 cep_irq_stat = udc_readw(udc, CEP_IRQ_STAT);
	u16 cep_irq_enable = udc_readw(udc, CEP_IRQ_ENB);
	dev_dbg(udc->dev, "Interrupt: CEP_IRQ_STAT:0x%08x, cep_irq_stat:0x%08x\n", udc_readw(udc, CEP_IRQ_STAT), cep_irq_stat);
	dev_dbg(udc->dev, "Interrupt: CEP_IRQ_MASK:0x%08x, cep_irq_enable:0x%08x\n", CEP_IRQ_MASK, cep_irq_enable);
	cep_irq_stat &= cep_irq_enable;
	udc_writew(udc, CEP_IRQ_STAT, cep_irq_stat);
	/* Some interrupt flags will be triggered even the interrupt is disabled
		we just care the interrupt flags with its interrupt enabled */
	udc->im98xx_ep[0].stats.irqs++;
	handle_ep0(udc, cep_irq_stat);
}

/*
 * irq_handle_data - Handle data transfer of ep
 * @irq: irq IRQ number
 * @udc: dev im98xx_udc device structure
 *
 * Called from irq handler, transferts data to or from endpoint to queue
 */

#if 0
static void irq_handle_ep(int irq, struct im98xx_udc *udc)
{
	int i;
	struct im98xx_ep *ep;
	u16 irq_enb_l, ep_irq_stat = 0;
	u16 ep_irq_enb = 0;

	irq_enb_l = udc_readw(udc, IRQ_ENB_L);
	for (i = 1; i < NR_IM98XX_UDC_ENDPOINTS; i++) {
		ep = &udc->im98xx_ep[i];
		ep_irq_stat = udc_ep_readw(ep, EP_IRQ_STAT) & EP_IRQ_MASK;
		ep_irq_enb = udc_ep_readw(ep, EP_IRQ_ENB);
		if ((irq_enb_l & (1<<(i+1))) && ep_irq_stat) {
			dev_dbg(udc->dev, "Interrupt: ep%d, IRQ_ENB_L:0x%08x, EP_IRQ_STAT:0x%08x\n",
						i, irq_enb_l, ep_irq_stat);
			dev_dbg(udc->dev, "Interrupt: ep%d, reg: EP_IRQ_STAT:0x%08x, EP_IRQ_ENB:0x%08x\n", i,
						udc_ep_readw(ep, EP_IRQ_STAT), udc_ep_readw(ep, EP_IRQ_ENB));
			ep->stats.irqs++;
			if (ep->dir_in)
			udc_ep_writew(ep, EP_IRQ_STAT, ep_irq_stat | ~EP_IRQ_MASK);
			ep_irq_stat &= ep_irq_enb;
			handle_ep(ep, ep_irq_stat);
		}
	}
}

#else
static void irq_handle_ep(int irq, struct im98xx_ep *ep)
{
	u16 ep_irq_stat = 0;
	u16 ep_irq_enb = 0;

	ep_irq_stat = udc_ep_readw(ep, EP_IRQ_STAT);
	ep_irq_enb = udc_ep_readw(ep, EP_IRQ_ENB);
	ep_irq_stat &= ep_irq_enb;
	if (ep_irq_stat) {
        dev_dbg(ep->udc->dev, "%s\n",__func__);
		ep_dbg(ep, "Interrupt: EP_DATA_CNT=%d,EP_IRQ_STAT:0x%08x & EP_IRQ_ENB = 0x%08x\n",
			(udc_ep_readw(ep, EP_DATA_CNT) & 0x7ff), udc_ep_readw(ep, EP_IRQ_STAT), ep_irq_stat);
		ep->stats.irqs++;
		handle_ep(ep, ep_irq_stat);
	}
}

#endif

int usb_gadget_poll(void)
{
	struct im98xx_udc *udc = the_controller;
	struct im98xx_ep *ep = NULL;
	u16 irq_stat_l = 0;
	u16 irq_enb_l = 0;
	static int i=0;
	//dev_info(udc->dev, "%s enter\n",__func__);


	usb_cable_status(udc);//udc->vbus_sensed = udc->mach->udc_is_connected();
	if(udc->usb_cable_sensed){
		if (should_enable_udc(udc))
			udc_enable(udc);
	}else{
		if (!should_enable_udc(udc)) {
			stop_activity(udc, udc->driver);
			udc_disable(udc);
		}
	}

	if (!udc->enabled){
		if(i++ == 150000 ){
		i=0;
		dev_dbg(udc->dev, "not enabled\n");
	}
		return -EIO;
	}

#if defined(CONFIG_USB_OHCI_HCD)
	extern int im98xx_usb_host_isr_flag;

	if(udc->usb_cable_sensed == 0 || im98xx_usb_host_isr_flag == 1)
	{
		im98xx_usb_host_isr_flag = 0;
		dev_dbg(udc->dev, "not my irq, device don't care\n");
		return 0;
	}
#else
	if(udc->usb_cable_sensed == 0)  // USB host irq, device don't care
	{
		dev_info(udc->dev, "not my irq, device don't care\n");
		return 0;
	}
#endif

	irq_stat_l = udc_readw(udc, IRQ_STAT_L);
	irq_enb_l = udc_readw(udc, IRQ_ENB_L);

	irq_stat_l &= irq_enb_l;
	if (irq_stat_l) {
		udc_writew(udc, IRQ_STAT_L, irq_stat_l);
		if (irq_stat_l & CEP_INT_EN) {
			irq_handle_cep(0, udc);
		}

		if (irq_stat_l & USB_INT_EN) {
			irq_handle_usb(0, udc);
		}

		if (irq_stat_l & EP1_INT_EN) {
			ep = &udc->im98xx_ep[1];
			irq_handle_ep(0, ep);
		}
		if (irq_stat_l & EP2_INT_EN) {
			ep = &udc->im98xx_ep[2];
			irq_handle_ep(0, ep);
		}
		if (irq_stat_l & EP3_INT_EN) {
			ep = &udc->im98xx_ep[3];
			irq_handle_ep(0, ep);
		}
		if (irq_stat_l & EP4_INT_EN) {
			ep = &udc->im98xx_ep[4];
			irq_handle_ep(0, ep);
		}
		if (irq_stat_l & EP5_INT_EN) {
			ep = &udc->im98xx_ep[5];
			irq_handle_ep(0, ep);
		}
		if (irq_stat_l & EP6_INT_EN) {
			ep = &udc->im98xx_ep[6];
			irq_handle_ep(0, ep);
		}

		//dev_dbg(udc->dev, "\n\n");
		return 1;
	}
	//dev_dbg(udc->dev, "\n\n\n");

	return 0;
}

void print_all_registers(struct im98xx_udc *udc)
{
#define LAST_REGS	0x150/4
	int i, j;

	dev_dbg(udc->dev, "\n******************************************************"
			"**************************************************************\n");

	for (i=0, j=0; i<=LAST_REGS; i++, j++) {
		if (0 == i) {
			dev_dbg(udc->dev, "Main Control Registers:\n");
			dev_dbg(udc->dev, "==================================\n");
			j = 0;
		}
		if (4 == i) {
			dev_dbg(udc->dev, "\n\nUSB Control Registers:\n");
			dev_dbg(udc->dev, "==================================\n");
			
			j = 0;
		}
		if (10 == i) {
			dev_dbg(udc->dev, "\n\nControl EP Registers:\n");
			dev_dbg(udc->dev, "==================================\n");
			j = 0;
		}
		if (23 == i) {
			dev_dbg(udc->dev, "\n\nDMA Registers:\n");
			dev_dbg(udc->dev, "==================================\n");
			dev_dbg(udc->dev, "reg[0x%x][%d]=0x%-4x\t", 
				(unsigned int)(udc->regs + 0x258), i, udc_readl(udc, 0x258));
			dev_dbg(udc->dev, "reg[0x%x][%d]=0x%-4x\t", 
				(unsigned int)(udc->regs + 0x25c), i, udc_readl(udc, 0x25c));
			j = 0;
		}
		if (25 == i || 35 == i || 45 == i || 55 == i || 65 == i || 75 == i) {
			dev_dbg(udc->dev, "\n\nEP[%d] Registers:\n", i/10-1);
			dev_dbg(udc->dev, "==================================\n");
			j = 0;
		}
		if (j%4 == 0 && j != 0)
			dev_dbg(udc->dev, "\n");
		dev_dbg(udc->dev, "reg[0x%x][%d]=0x%-4x\t", 
			(unsigned int)(udc->regs + i*4), i, udc_readl(udc, i*4));
	}
	dev_dbg(udc->dev, "\n******************************************************"
			"**************************************************************\n");
}



/**
 * usb_cable_status - checks if USB cable is connected or not
 *
 * Returns 0 if cable disconnected, 1 if cable connected
 */
static void usb_cable_status(struct im98xx_udc *udc)
{
	unsigned usb_detect_cable = udc->usb_cable_gpio;
	static int i=0;
	/* Before access USB IP registers, detect the existence of USB cable */
	gpio_line_config(usb_detect_cable,GPIO_INPUT);//gpio_direction_input(usb_detect_cable);
	
	if (gpio_line_get_input(usb_detect_cable))
		udc->usb_cable_sensed = 1;
	else
		udc->usb_cable_sensed = 0;
	if(i++ == 100000 ){
	i=0;
	dev_dbg(udc->dev, "usb_cable_status: cable = %d. \n", udc->usb_cable_sensed);
	}
	return ;
}
/*
static irqreturn_t usb_detect_irq(int irq, void *dev_id)
{
	struct im98xx_udc *udc = (struct im98xx_udc *)dev_id;
	unsigned usb_detect_cable = udc->usb_cable_gpio;

	if (udc_suspended == 0 && external_int_line_detect(usb_detect_cable)) {
		dev_dbg(udc->dev, "USB cable irq\n");
		external_int_line_clear_set(usb_detect_cable, EXTERNAL_INT_CLEAR);
		gpio_direction_input(usb_detect_cable);
		if (__gpio_get_value(usb_detect_cable)) {
			dev_info(udc->dev, "\n\n");
			dev_info(udc->dev, "USB cable PLUG-IN detected\n");
			if (delayed_work_pending(&suspend_handler)) {
				dev_info(udc->dev, "USB: cancel suspend workqueue\n");
				cancel_delayed_work(&suspend_handler);
			}
			udc->usb_cable_sensed = 1;
			udc_enable(udc);
		} else {
			dev_info(udc->dev, "USB cable UN-PLUG detected.\n");
			udc->usb_cable_sensed = 0;

			stop_activity(udc, udc->driver);
			udc_disable(udc);

		}
		gpio_line_set_interrupt_enable_disable(usb_detect_cable, GPIO_INT_GEN);
	} else {
		dev_dbg(udc->dev, "not my irq, don't care---->\n");
		return IRQ_NONE;
	}
	dev_dbg(udc->dev, "usb_detect_irq---->\n");

	return IRQ_HANDLED;
}
*/
/**
 * usb_detect_init - initialize GPIO into interrupt mode
 * @udc: udc device
 * @usb_detect_cable: GPIO number obtain from the device end
 *
 * Initialize GPIO into interrupt mode to detect USB cable plugin/unplug.
 *
 */
static int usb_detect_init(struct im98xx_udc *udc, unsigned usb_detect_cable)
{
	gpio_line_config(usb_detect_cable,GPIO_INPUT);//ret = gpio_direction_input(usb_detect_cable);

	/* Initially set usb_cable_sensed to zero */
	udc->usb_cable_sensed = 0;

	dev_info(udc->dev, "USB device detection cable: gpio(%d)\n", usb_detect_cable);
/*
	ret = request_irq(usb_gpio_irq, usb_detect_irq, IRQF_SHARED, "usb_cable", (void*)udc);
	if (ret < 0) {
		dev_err(udc->dev, "USB detection irq request failed\n");
		goto err_request_detect_irq;
	}
	gpio_line_set_pull_low(usb_detect_cable, GPIO_PULL_LOW_DISENABLE);
	gpio_line_set_edgr_tri(usb_detect_cable, BOTH_EDGE_TRIGGER);
	gpio_line_set_interrupt_enable_disable(usb_detect_cable, GPIO_INT_GEN);
*/
	return 0;

}

static void im98xx_udc_poller(struct poller_struct *poller)
{
	usb_gadget_poll();
}
static struct poller_struct poller = {
	.func		= im98xx_udc_poller
};



/**
 * im98xx_udc_probe - probes the udc device
 * @_dev: platform device
 *
 * Perform basic init : allocates udc clock, creates sysfs files, requests
 * irq.
 */
static int __init im98xx_udc_probe(struct device_d *dev)
{
	struct im98xx_udc *udc;
	u8 usb_irq_enb;
	int retval = 0;

	udc = kzalloc(sizeof(*udc), GFP_KERNEL);
	if (udc == NULL) {
		printk( "%s malloc udc failed\n",__func__);
		return -ENOMEM;
	}
	udc->regs = dev_request_mem_region(dev,0);
	if (!udc->regs){
	}
	udc->irq = (dev_get_resource(dev, IORESOURCE_IRQ, 0)->start);

	udc->dev = dev;

	the_controller = udc;

	udc->udc_data= (struct im98xx_udc_data *) dev->platform_data;
	if (!udc->udc_data) {
		return -ENODEV;
	}
	udc_init_data(udc);
	udc->ep0_req = im98xx_ep_alloc_request(&udc->im98xx_ep[0].usb_ep, GFP_KERNEL);
	if (udc->ep0_req == NULL) {
		printk( "im98xx_ep_alloc_request failed\n");
		return -ENOMEM;
	}
	udc->ep0_req->complete = req_get_status_complete;

	/* Initialize a work */

#if 1
	usb_irq_enb = udc_readb(udc, USB_IRQ_ENB);
	//usb_irq_enb &= ~(USEBLE_CLK_EN | SOF_INT_EN | RSUM_INT_EN | SPNT_INT_EN | DMA_CMPET_INT_EN);
	usb_irq_enb &= ~(USEBLE_CLK_EN);
	udc_writeb(udc, USB_IRQ_ENB, usb_irq_enb);
#endif
	/* irq setup after old hardware state is cleaned up */
	dev_dbg(udc->dev, "%s: USB IRQ NO=%d\n", __FUNCTION__, udc->irq);

#if 1
	/* Set up USB detect cable */
	usb_detect_init(udc, udc->usb_cable_gpio);
#endif

	udc_suspended = 0;
	poller_register(&poller);
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	udc->gadget.max_speed = USB_SPEED_HIGH;
	udc->gadget.name = "im98xx-udc";

	retval = usb_add_gadget_udc_release(dev, &udc->gadget, NULL);
	if (retval)
		return retval;
		
	return 0;

}



static struct driver_d im98xx_udc_driver = {
	.name	= "im98xx-udc",
	.probe	= im98xx_udc_probe,
};
device_platform_driver(im98xx_udc_driver);
