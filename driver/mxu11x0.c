/*
 *
 * MOXA UPort 11x0 USB to Serial Hub Driver
 *
 * Copyright (C) 2007 MOXA Technologies Co., Ltd.
 *
 * This driver is based on the Linux io_ti driver, which is
 *   Copyright (C) 2000-2002 Inside Out Networks
 *   Copyright (C) 2001-2002 Greg Kroah-Hartman
 *   Copyright (C) 2004-2006 Al Borchers
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/ioctl.h>
#include <linux/serial.h>
#include <linux/circ_buf.h>
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
#include <asm/uaccess.h>
#include <asm/semaphore.h>
#else
#include <linux/uaccess.h>
#include <linux/semaphore.h>
#endif
#if(LINUX_VERSION_CODE > KERNEL_VERSION(4,11,0))
#include <linux/sched/signal.h>
#endif

#include <linux/usb.h>

#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,18))
#include "usb-serial.h"
#else
#include <linux/usb/serial.h>
#endif
#include "mxu11x0.h"
#include "mxu1110_fw.h"
#include "mxu1130_fw.h"
#include "mxu1131_fw.h"
#include "mxu1150_fw.h"
#include "mxu1151_fw.h"


/* Defines */

#define MXU1_DRIVER_VERSION	"1.3.11"
#define MXU1_DRIVER_AUTHOR	"Ken Huang"
#define MXU1_DRIVER_DESC	"MOXA UPort 11x0 USB to Serial Hub Driver"

#define MXU1_FIRMWARE_BUF_SIZE	16284

#define MXU1_WRITE_BUF_SIZE	1024

#define MXU1_TRANSFER_TIMEOUT	2
#define MXU1_MSR_WAIT_TIMEOUT	(5 * HZ)

#define MXU1_DEFAULT_LOW_LATENCY	1
#define MXU1_DEFAULT_CLOSING_WAIT	4000		/* in .01 secs */

/* supported setserial flags */
#define MXU1_SET_SERIAL_FLAGS	(ASYNC_LOW_LATENCY)

/* read urb states */
#define MXU1_READ_URB_RUNNING	0
#define MXU1_READ_URB_STOPPING	1
#define MXU1_READ_URB_STOPPED	2


/* Structures */

struct mxu1_port {
	int			mxp_is_open;
	__u8			mxp_msr;
	__u8			mxp_lsr;
	__u8			mxp_shadow_mcr;
	__u8			mxp_uart_mode;	/* 232 or 485 modes */
	__u8			mxp_user_get_uart_mode;
	unsigned int		mxp_uart_base_addr;
	int			mxp_flags;
	int			mxp_msr_wait_flags;
	int			mxp_closing_wait;/* in .01 secs */
	struct async_icount	mxp_icount;
	wait_queue_head_t	mxp_msr_wait;	/* wait for msr change */
	wait_queue_head_t	mxp_write_wait;
	struct mxu1_device	*mxp_mxdev;
	struct usb_serial_port	*mxp_port;
	spinlock_t		mxp_lock;
	int			mxp_read_urb_state;
	int			mxp_write_urb_in_use;
	struct circ_buf		*mxp_write_buf;
	int			mxp_send_break;
	int			mxp_set_B0;
};

struct mxu1_device {
	struct semaphore	mxd_open_close_sem;
	int			mxd_open_port_count;
	struct usb_serial	*mxd_serial;
	int			mxd_model_name;
	int			mxd_urb_error;
};


/* Function Declarations */

static int mxu1_startup(struct usb_serial *serial);

#ifdef ASYNCB_FIRST_KERNEL
static void mxu1_disconnect(struct usb_serial *serial);
static void mxu1_release(struct usb_serial *serial);
#else
static void mxu1_shutdown(struct usb_serial *serial);
#endif

#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static int mxu1_open(struct usb_serial_port *port, struct file *file);
#elif(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31))
static int mxu1_open(struct tty_struct *tty, struct usb_serial_port *port);
#else
static int mxu1_open(struct tty_struct *tty, struct usb_serial_port *port,
			struct file *file);
#endif
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static void mxu1_close(struct usb_serial_port *port, struct file *file);
#elif(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,30))
static void mxu1_close(struct usb_serial_port *port);
#else
static void mxu1_close(struct tty_struct *tty, struct usb_serial_port *port,
			struct file *file);
#endif

#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10))
static int mxu1_write (struct usb_serial_port *port, int from_user,const unsigned char *data, int count);
#else
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static int mxu1_write(struct usb_serial_port *port, const unsigned char *data,int count);
#else
static int mxu1_write(struct tty_struct *tty, struct usb_serial_port *port, const unsigned char *data,int count);
#endif
#endif
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static int mxu1_write_room(struct usb_serial_port *port);
static int mxu1_chars_in_buffer(struct usb_serial_port *port);
static void mxu1_throttle(struct usb_serial_port *port);
static void mxu1_unthrottle(struct usb_serial_port *port);
static int mxu1_ioctl(struct usb_serial_port *port, struct file *file, unsigned int cmd, unsigned long arg);
#else
static int mxu1_write_room(struct tty_struct *tty);
static int mxu1_chars_in_buffer(struct tty_struct *tty);
static void mxu1_throttle(struct tty_struct *tty);
static void mxu1_unthrottle(struct tty_struct *tty);
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39))
static int mxu1_ioctl(struct tty_struct *tty, struct file *file, unsigned int cmd, unsigned long arg);
#else
static int mxu1_ioctl(struct tty_struct *tty, unsigned int cmd, unsigned long arg);
#endif
#endif
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20))
static void mxu1_set_termios(struct usb_serial_port *port,
	struct termios *old_termios);
#else
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static void mxu1_set_termios(struct usb_serial_port *port,
	struct ktermios *old_termios);
#else
static void mxu1_set_termios(struct tty_struct *tty1,struct usb_serial_port *port, 
	struct ktermios *old_termios);
#endif	
#endif
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static int mxu1_tiocmget(struct usb_serial_port *port, struct file *file);
static int mxu1_tiocmset(struct usb_serial_port *port, struct file *file,
	unsigned int set, unsigned int clear);
static void mxu1_break(struct usb_serial_port *port, int break_state);
#else
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39))
static int mxu1_tiocmget(struct tty_struct *tty, struct file *file);
static int mxu1_tiocmset(struct tty_struct *tty,  struct file *file,
	unsigned int set, unsigned int clear);
#else
static int mxu1_tiocmget(struct tty_struct *tty);
static int mxu1_tiocmset(struct tty_struct *tty,
	unsigned int set, unsigned int clear);
#endif
static void mxu1_break(struct tty_struct *tty, int break_state);
#endif

#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))
static void mxu1_interrupt_callback(struct urb *urb, struct pt_regs *regs);
static void mxu1_bulk_in_callback(struct urb *urb, struct pt_regs *regs);
static void mxu1_bulk_out_callback(struct urb *urb, struct pt_regs *regs);
#else
static void mxu1_interrupt_callback(struct urb *urb);
static void mxu1_bulk_in_callback(struct urb *urb);
static void mxu1_bulk_out_callback(struct urb *urb);
#endif

static void mxu1_recv(struct mxu1_port *mxport,
	                  unsigned char *data, int length);
static void mxu1_send(struct mxu1_port *mxport);
static int mxu1_set_mcr(struct mxu1_port *mxport, unsigned int mcr);
static int mxu1_get_lsr(struct mxu1_port *mxport);
static int mxu1_get_serial_info(struct mxu1_port *mxport,
	struct serial_struct __user *ret_arg);
static int mxu1_set_serial_info(struct mxu1_port *mxport,
	struct serial_struct __user *new_arg);
static void mxu1_handle_new_msr(struct mxu1_port *mxport, __u8 msr);

static void mxu1_drain(struct mxu1_port *mxport, unsigned long timeout, int flush);

static void mxu1_stop_read(struct mxu1_port *mxport, struct tty_struct *tty);
static int mxu1_restart_read(struct mxu1_port *mxport, struct tty_struct *tty);

static int mxu1_command_out_sync(struct mxu1_device *mxdev, __u8 command,
	__u16 moduleid, __u16 value, __u8 *data, int size);
static int mxu1_command_in_sync(struct mxu1_device *mxdev, __u8 command,
	__u16 moduleid, __u16 value, __u8 *data, int size);

static int mxu1_write_byte(struct mxu1_device *mxdev, unsigned long addr,
	__u8 mask, __u8 byte);

static int mxu1_download_firmware(struct mxu1_device *mxdev,
	unsigned char *firmware, unsigned int firmware_size);

#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9))
static signed long mxu1_schedule_timeout_interruptible(signed long timeout);

static unsigned long mxu1_jsleep_interruptible(unsigned long timeout);
#endif

#if(LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39))
static int mxu1_get_icount(struct tty_struct *tty,
		struct serial_icounter_struct *icount);
#endif		

/* circular buffer */
static struct circ_buf *mxu1_buf_alloc(void);
static void mxu1_buf_free(struct circ_buf *cb);
static void mxu1_buf_clear(struct circ_buf *cb);
static int mxu1_buf_data_avail(struct circ_buf *cb);
static int mxu1_buf_space_avail(struct circ_buf *cb);
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10))
static int mxu1_buf_put(struct mxu1_port *mxport, const char *buf, int count, int from_user);
#else
static int mxu1_buf_put(struct mxu1_port *mxport, const char *buf, int count);
#endif
static int mxu1_buf_get(struct circ_buf *cb, char *buf, int count);


/* Data */

/* module parameters */
#if(LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0))
static int debug;
#endif
/* supported devices */

static struct usb_device_id mxu1_id_table[] = {
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1110_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1130_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1150_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1151_PRODUCT_ID) },
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1131_PRODUCT_ID) },
	{ }
};

static struct usb_device_id mxu1110_id_table[] = {
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1110_PRODUCT_ID) },
	{ }
};

static struct usb_device_id mxu1130_id_table[] = {
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1130_PRODUCT_ID) },
	{ }
};

static struct usb_device_id mxu1150_id_table[] = {
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1150_PRODUCT_ID) },
	{ }
};

static struct usb_device_id mxu1151_id_table[] = {
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1151_PRODUCT_ID) },
	{ }
};

static struct usb_device_id mxu1131_id_table[] = {
	{ USB_DEVICE(MXU1_VENDOR_ID, MXU1_1131_PRODUCT_ID) },
	{ }
};

#if(LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0))
static struct usb_driver mxu1_usb_driver = {
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,15))
	.owner			= THIS_MODULE,
#endif
	.name			= "mxusb",
	.probe			= usb_serial_probe,
	.disconnect		= usb_serial_disconnect,
	.id_table		= mxu1_id_table,
#if(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,15)) && \
	(LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0))
	.no_dynamic_id = 1,
#endif
};
#endif

#if(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,13))
static struct usb_serial_driver mxu1110_1port_device = {
	.driver={
		.owner		= THIS_MODULE,
		.name		= "mxu1110",
	},
	.description		= "MOXA UPort 1110",
#if(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,20)) && \
	(LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0))
	.usb_driver		= &mxu1_usb_driver,
#endif
#else
static struct usb_serial_device_type mxu1110_1port_device = {
	.owner			= THIS_MODULE,
	.name			= "MOXA UPort 1110",
#endif
	.id_table		= mxu1110_id_table,
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25))	
	.num_interrupt_in	= NUM_DONT_CARE,
	.num_bulk_in		= NUM_DONT_CARE,
	.num_bulk_out		= NUM_DONT_CARE,
#endif	
	.num_ports		= 1,
	.attach			= mxu1_startup,

#ifdef ASYNCB_FIRST_KERNEL
	.disconnect		= mxu1_disconnect,
	.release		= mxu1_release,
#else
	.shutdown		= mxu1_shutdown,
#endif
	.open			= mxu1_open,
	.close			= mxu1_close,
	.write			= mxu1_write,
	.write_room		= mxu1_write_room,
	.chars_in_buffer	= mxu1_chars_in_buffer,
	.throttle		= mxu1_throttle,
	.unthrottle		= mxu1_unthrottle,
	.ioctl			= mxu1_ioctl,
	.set_termios		= mxu1_set_termios,
	.tiocmget		= mxu1_tiocmget,
	.tiocmset		= mxu1_tiocmset,
#if(LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39))	
	.get_icount		= mxu1_get_icount,
#endif	
	.break_ctl		= mxu1_break,
	.read_int_callback	= mxu1_interrupt_callback,
	.read_bulk_callback	= mxu1_bulk_in_callback,
	.write_bulk_callback	= mxu1_bulk_out_callback,
};

#if(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,13))
static struct usb_serial_driver mxu1130_1port_device = {
	.driver={
		.owner		= THIS_MODULE,
		.name		= "mxu1130",
	},
	.description		= "MOXA UPort 1130",
#if(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,20)) && \
	(LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0))
	.usb_driver		= &mxu1_usb_driver,
#endif
#else
static struct usb_serial_device_type mxu1130_1port_device = {
	.owner			= THIS_MODULE,
	.name			= "MOXA UPort 1130",
#endif
	.id_table		= mxu1130_id_table,
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25))	
	.num_interrupt_in	= NUM_DONT_CARE,
	.num_bulk_in		= NUM_DONT_CARE,
	.num_bulk_out		= NUM_DONT_CARE,
#endif	
	.num_ports		= 1,
	.attach			= mxu1_startup,
#ifdef ASYNCB_FIRST_KERNEL
	.disconnect		= mxu1_disconnect,
	.release		= mxu1_release,
#else
	.shutdown		= mxu1_shutdown,
#endif
	.open			= mxu1_open,
	.close			= mxu1_close,
	.write			= mxu1_write,
	.write_room		= mxu1_write_room,
	.chars_in_buffer	= mxu1_chars_in_buffer,
	.throttle		= mxu1_throttle,
	.unthrottle		= mxu1_unthrottle,
	.ioctl			= mxu1_ioctl,
	.set_termios		= mxu1_set_termios,
	.tiocmget		= mxu1_tiocmget,
	.tiocmset		= mxu1_tiocmset,
#if(LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39))	
	.get_icount		= mxu1_get_icount,
#endif		
	.break_ctl		= mxu1_break,
	.read_int_callback	= mxu1_interrupt_callback,
	.read_bulk_callback	= mxu1_bulk_in_callback,
	.write_bulk_callback	= mxu1_bulk_out_callback,
};

#if(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,13))
static struct usb_serial_driver mxu1150_1port_device = {
	.driver={
		.owner		= THIS_MODULE,
		.name		= "mxu1150",
	},
	.description		= "MOXA UPort 1150",
#if(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,20)) && \
	(LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0))
	.usb_driver		= &mxu1_usb_driver,
#endif
#else
static struct usb_serial_device_type mxu1150_1port_device = {
	.owner			= THIS_MODULE,
	.name			= "MOXA UPort 1150",
#endif
	.id_table		= mxu1150_id_table,
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25))	
	.num_interrupt_in	= NUM_DONT_CARE,
	.num_bulk_in		= NUM_DONT_CARE,
	.num_bulk_out		= NUM_DONT_CARE,
#endif	
	.num_ports		= 1,
	.attach			= mxu1_startup,
	.open			= mxu1_open,
#ifdef ASYNCB_FIRST_KERNEL
	.disconnect		= mxu1_disconnect,
	.release		= mxu1_release,
#else
	.shutdown		= mxu1_shutdown,
#endif
	.close			= mxu1_close,
	.write			= mxu1_write,
	.write_room		= mxu1_write_room,
	.chars_in_buffer	= mxu1_chars_in_buffer,
	.throttle		= mxu1_throttle,
	.unthrottle		= mxu1_unthrottle,
	.ioctl			= mxu1_ioctl,
	.set_termios		= mxu1_set_termios,
	.tiocmget		= mxu1_tiocmget,
	.tiocmset		= mxu1_tiocmset,
#if(LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39))	
	.get_icount		= mxu1_get_icount,
#endif		
	.break_ctl		= mxu1_break,
	.read_int_callback	= mxu1_interrupt_callback,
	.read_bulk_callback	= mxu1_bulk_in_callback,
	.write_bulk_callback	= mxu1_bulk_out_callback,
};

#if(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,13))
static struct usb_serial_driver mxu1151_1port_device = {
	.driver={
		.owner		= THIS_MODULE,
		.name		= "mxu1151",
	},
	.description		= "MOXA UPort 1150I",
#if(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,20)) && \
	(LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0))
	.usb_driver		= &mxu1_usb_driver,
#endif
#else
static struct usb_serial_device_type mxu1151_1port_device = {
	.owner			= THIS_MODULE,
	.name			= "MOXA UPort 1150I",
#endif
	.id_table		= mxu1151_id_table,
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25))	
	.num_interrupt_in	= NUM_DONT_CARE,
	.num_bulk_in		= NUM_DONT_CARE,
	.num_bulk_out		= NUM_DONT_CARE,
#endif	
	.num_ports		= 1,
	.attach			= mxu1_startup,
#ifdef ASYNCB_FIRST_KERNEL
	.disconnect		= mxu1_disconnect,
	.release		= mxu1_release,
#else
	.shutdown		= mxu1_shutdown,
#endif
	.open			= mxu1_open,
	.close			= mxu1_close,
	.write			= mxu1_write,
	.write_room		= mxu1_write_room,
	.chars_in_buffer	= mxu1_chars_in_buffer,
	.throttle		= mxu1_throttle,
	.unthrottle		= mxu1_unthrottle,
	.ioctl			= mxu1_ioctl,
	.set_termios		= mxu1_set_termios,
	.tiocmget		= mxu1_tiocmget,
	.tiocmset		= mxu1_tiocmset,
#if(LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39))	
	.get_icount		= mxu1_get_icount,
#endif		
	.break_ctl		= mxu1_break,
	.read_int_callback	= mxu1_interrupt_callback,
	.read_bulk_callback	= mxu1_bulk_in_callback,
	.write_bulk_callback	= mxu1_bulk_out_callback,
};
#if(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,13))
static struct usb_serial_driver mxu1131_1port_device = {
	.driver={
		.owner		= THIS_MODULE,
		.name		= "mxu1131",
	},
	.description		= "MOXA UPort 1130I",
#if(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,20)) && \
	(LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0))
	.usb_driver		= &mxu1_usb_driver,
#endif
#else
static struct usb_serial_device_type mxu1131_1port_device = {
	.owner			= THIS_MODULE,
	.name			= "MOXA UPort 1130I",
#endif
	.id_table		= mxu1131_id_table,
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,25))	
	.num_interrupt_in	= NUM_DONT_CARE,
	.num_bulk_in		= NUM_DONT_CARE,
	.num_bulk_out		= NUM_DONT_CARE,
#endif	
	.num_ports		= 1,
	.attach			= mxu1_startup,
#ifdef ASYNCB_FIRST_KERNEL
	.disconnect		= mxu1_disconnect,
	.release		= mxu1_release,
#else
	.shutdown		= mxu1_shutdown,
#endif
	.open			= mxu1_open,
	.close			= mxu1_close,
	.write			= mxu1_write,
	.write_room		= mxu1_write_room,
	.chars_in_buffer	= mxu1_chars_in_buffer,
	.throttle		= mxu1_throttle,
	.unthrottle		= mxu1_unthrottle,
	.ioctl			= mxu1_ioctl,
	.set_termios		= mxu1_set_termios,
	.tiocmget		= mxu1_tiocmget,
	.tiocmset		= mxu1_tiocmset,
#if(LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39))	
	.get_icount		= mxu1_get_icount,
#endif		
	.break_ctl		= mxu1_break,
	.read_int_callback	= mxu1_interrupt_callback,
	.read_bulk_callback	= mxu1_bulk_in_callback,
	.write_bulk_callback	= mxu1_bulk_out_callback,
};

#if(LINUX_VERSION_CODE >= KERNEL_VERSION(3,4,0))
static struct usb_serial_driver * const serial_drivers[] = {
	&mxu1110_1port_device, &mxu1130_1port_device, &mxu1150_1port_device,
	&mxu1151_1port_device, &mxu1131_1port_device, NULL
};
#endif

/* Module */

MODULE_AUTHOR(MXU1_DRIVER_AUTHOR);
MODULE_DESCRIPTION(MXU1_DRIVER_DESC);
MODULE_VERSION(MXU1_DRIVER_VERSION);
MODULE_LICENSE("GPL");

MODULE_DEVICE_TABLE(usb, mxu1_id_table);
MODULE_DEVICE_TABLE(usb, mxu1110_id_table);
MODULE_DEVICE_TABLE(usb, mxu1130_id_table);
MODULE_DEVICE_TABLE(usb, mxu1150_id_table);
MODULE_DEVICE_TABLE(usb, mxu1151_id_table);
MODULE_DEVICE_TABLE(usb, mxu1131_id_table);

/* Functions */

#if(LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0))
static int __init mxu1_init(void)
{
	int ret;

	ret = usb_serial_register(&mxu1110_1port_device);
	if (ret)
		goto failed_1110;

	ret = usb_serial_register(&mxu1130_1port_device);
	if (ret)
		goto failed_1130;

	ret = usb_serial_register(&mxu1150_1port_device);
	if (ret)
		goto failed_1150;

	ret = usb_serial_register(&mxu1151_1port_device);
	if (ret)
		goto failed_1151;
		
	ret = usb_serial_register(&mxu1131_1port_device);
	if (ret)
		goto failed_1131;	

	ret = usb_register(&mxu1_usb_driver);
	if (ret)
		goto failed_usb;
#if(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,27))
	printk(KERN_INFO KBUILD_MODNAME ": " MXU1_DRIVER_VERSION ":"
	       MXU1_DRIVER_DESC "\n");
#else	       
	info(MXU1_DRIVER_DESC " " MXU1_DRIVER_VERSION);      
#endif
	return 0;

failed_usb:
	usb_serial_deregister(&mxu1131_1port_device);
failed_1131:
	usb_serial_deregister(&mxu1151_1port_device);
failed_1151:
	usb_serial_deregister(&mxu1150_1port_device);
failed_1150:
	usb_serial_deregister(&mxu1130_1port_device);
failed_1130:
	usb_serial_deregister(&mxu1110_1port_device);
failed_1110:

	return ret;
}

static void __exit mxu1_exit(void)
{
	usb_deregister(&mxu1_usb_driver);
	usb_serial_deregister(&mxu1110_1port_device);
	usb_serial_deregister(&mxu1130_1port_device);
	usb_serial_deregister(&mxu1150_1port_device);
	usb_serial_deregister(&mxu1151_1port_device);
	usb_serial_deregister(&mxu1131_1port_device);
}
#else
static int __init mxu1_init(void)
{
	int ret;
#if(LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0))	
	ret = usb_serial_register_drivers(&mxu1_usb_driver, serial_drivers);
#else
	ret = usb_serial_register_drivers(serial_drivers, KBUILD_MODNAME, mxu1_id_table);
#endif
	
	if(ret == 0)
		printk(KERN_INFO KBUILD_MODNAME ": " MXU1_DRIVER_VERSION ":"
	       MXU1_DRIVER_DESC "\n");
		   
	return ret;
}

static void __exit mxu1_exit(void)
{
#if(LINUX_VERSION_CODE < KERNEL_VERSION(3,5,0))	
	usb_serial_deregister_drivers(&mxu1_usb_driver, serial_drivers);
#else
	usb_serial_deregister_drivers(serial_drivers);
#endif	
}
#endif





module_init(mxu1_init);
module_exit(mxu1_exit);


static int mxu1_startup(struct usb_serial *serial)
{
	struct mxu1_device *mxdev;
	struct mxu1_port *mxport;
	struct usb_device *dev = serial->dev;
	int status = 0;
	int i;

	dbg("%s - product 0x%4X, num configurations %d, configuration value %d",
	    __FUNCTION__, le16_to_cpu(dev->descriptor.idProduct),
	    dev->descriptor.bNumConfigurations,
	    dev->actconfig->desc.bConfigurationValue);

	/* create device structure */
	mxdev = kmalloc(sizeof(struct mxu1_device), GFP_KERNEL);
	if (mxdev == NULL) {
		dev_err(&dev->dev, "%s - out of memory\n", __FUNCTION__);
		return -ENOMEM;
	}
	memset(mxdev, 0, sizeof(struct mxu1_device));
	sema_init(&mxdev->mxd_open_close_sem, 1);
	mxdev->mxd_serial = serial;
	usb_set_serial_data(serial, mxdev);

	/* determine device type */
	if (!usb_match_id(serial->interface, mxu1_id_table)){
			return 0;
	}
	
	switch(dev->descriptor.idProduct){
		case MXU1_1110_PRODUCT_ID:
			mxdev->mxd_model_name = MXU1_MODEL_1110;
			break;
		case MXU1_1130_PRODUCT_ID:
			mxdev->mxd_model_name = MXU1_MODEL_1130;
			break;
		case MXU1_1131_PRODUCT_ID:
			mxdev->mxd_model_name = MXU1_MODEL_1131;
			break;
		case MXU1_1150_PRODUCT_ID:
			mxdev->mxd_model_name = MXU1_MODEL_1150;
			break;
		case MXU1_1151_PRODUCT_ID:
			mxdev->mxd_model_name = MXU1_MODEL_1151;
			break;
		default:
			return -ENODEV;
	}

	/* if we have only 1 configuration, download firmware */
	if (dev->config->interface[0]->cur_altsetting->desc.bNumEndpoints == 1) {
		switch(mxdev->mxd_model_name){
			case MXU1_MODEL_1110:
			status = mxu1_download_firmware(mxdev, mxu1110FWImage, sizeof(mxu1110FWImage));
				break;
			case MXU1_MODEL_1130:
			status = mxu1_download_firmware(mxdev, mxu1130FWImage, sizeof(mxu1130FWImage));
				break;
			case MXU1_MODEL_1131:
			status = mxu1_download_firmware(mxdev, mxu1131FWImage, sizeof(mxu1131FWImage));
				break;	
			case MXU1_MODEL_1150:
			status = mxu1_download_firmware(mxdev, mxu1150FWImage, sizeof(mxu1150FWImage));
				break;
			case MXU1_MODEL_1151:
			status = mxu1_download_firmware(mxdev, mxu1151FWImage, sizeof(mxu1151FWImage));
				break;
		}
			
		if (status)
			goto free_mxdev;

#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9))
		mxu1_jsleep_interruptible(1000);
#else
		msleep_interruptible(100);
#endif
		status = mxu1_command_out_sync(mxdev,MXU1_RESET_EXT_DEVICE,(__u16)0,0,NULL,0);

		status = -ENODEV;
		goto free_mxdev;
	}
 
	/* set up port structures */
	for (i = 0; i < serial->num_ports; ++i) {
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20))
		mxport = kmalloc(sizeof(struct mxu1_port), GFP_KERNEL);
#else
		mxport = kzalloc(sizeof(struct mxu1_port), GFP_KERNEL);
#endif
		if (mxport == NULL) {
			dev_err(&dev->dev, "%s - out of memory\n", __FUNCTION__);
			status = -ENOMEM;
			goto free_mxports;
		}
		memset(mxport, 0, sizeof(struct mxu1_port));
		spin_lock_init(&mxport->mxp_lock);
		mxport->mxp_uart_base_addr = MXU1_UART1_BASE_ADDR;
		mxport->mxp_flags = ASYNC_LOW_LATENCY;
		mxport->mxp_closing_wait = MXU1_DEFAULT_CLOSING_WAIT;
		init_waitqueue_head(&mxport->mxp_msr_wait);
		init_waitqueue_head(&mxport->mxp_write_wait);
		mxport->mxp_write_buf = mxu1_buf_alloc();
		if (mxport->mxp_write_buf == NULL) {
			dev_err(&dev->dev, "%s - out of memory\n", __FUNCTION__);
			kfree(mxport);
			status = -ENOMEM;
			goto free_mxports;
		}
		mxport->mxp_port = serial->port[i];
		mxport->mxp_mxdev = mxdev;
		usb_set_serial_port_data(serial->port[i], mxport);
		if(mxdev->mxd_model_name != MXU1_MODEL_1130 && mxdev->mxd_model_name != MXU1_MODEL_1131){ //UPort 1110, 1150, 1150I
			mxport->mxp_uart_mode = MXU1_UART_232;
		}
		else{ //UPort 1130
			mxport->mxp_uart_mode = MXU1_UART_485_RECEIVER_DISABLED;
			mxport->mxp_user_get_uart_mode = MXU1_RS4852W;
		}
	}
	
	return 0;

free_mxports:
	for (--i; i>=0; --i) {
		mxport = usb_get_serial_port_data(serial->port[i]);
		mxu1_buf_free(mxport->mxp_write_buf);
		kfree(mxport);
		usb_set_serial_port_data(serial->port[i], NULL);
	}
free_mxdev:
	kfree(mxdev);
	usb_set_serial_data(serial, NULL);
	return status;
}


#ifdef ASYNCB_FIRST_KERNEL

static void mxu1_disconnect(struct usb_serial *serial)
{
	struct usb_serial_port *port = serial->port[0];
	dbg("%s", __FUNCTION__);

#if(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,8))
	usb_kill_urb(port->read_urb);
	usb_kill_urb(port->write_urb);
#else
	usb_unlink_urb(port->read_urb);
	usb_unlink_urb(port->write_urb);
#endif

}

static void mxu1_release(struct usb_serial *serial)
{	
	int i;
	struct mxu1_device *mxdev = usb_get_serial_data(serial);
	struct mxu1_port *mxport;

	dbg("%s", __FUNCTION__);

	for (i=0; i < serial->num_ports; ++i) {
		mxport = usb_get_serial_port_data(serial->port[i]);
		if (mxport) {
			mxu1_buf_free(mxport->mxp_write_buf);
			kfree(mxport);
			usb_set_serial_port_data(serial->port[i], NULL);
		}
	}

	kfree(mxdev);
	usb_set_serial_data(serial, NULL);
}

#else

static void mxu1_shutdown(struct usb_serial *serial)
{
	int i;
	struct mxu1_device *mxdev = usb_get_serial_data(serial);
	struct mxu1_port *mxport;

	dbg("%s", __FUNCTION__);

	for (i=0; i < serial->num_ports; ++i) {
		mxport = usb_get_serial_port_data(serial->port[i]);
		if (mxport) {
			mxu1_buf_free(mxport->mxp_write_buf);
			kfree(mxport);
			usb_set_serial_port_data(serial->port[i], NULL);
		}
	}

	kfree(mxdev);
	usb_set_serial_data(serial, NULL);
}
#endif

#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static int mxu1_open(struct usb_serial_port *port, struct file *file)
#elif(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,31))
static int mxu1_open(struct tty_struct *tty, struct usb_serial_port *port)
#else
static int mxu1_open(struct tty_struct *tty, struct usb_serial_port *port, struct file *file)
#endif
{
	struct mxu1_port *mxport = usb_get_serial_port_data(port);
	struct mxu1_device *mxdev;
	struct usb_device *dev;
	struct urb *urb;
	int port_number;
	int status = 0;
	__u16 open_settings = (__u8)(MXU1_PIPE_MODE_CONTINOUS | 
			     MXU1_PIPE_TIMEOUT_ENABLE | 
			     (MXU1_TRANSFER_TIMEOUT << 2));

	//dbg("%s - port %d", __FUNCTION__, port->number);
	dbg("%s", "<< IN >>");

	if (mxport == NULL)
		return -ENODEV;

	dev = port->serial->dev;
	mxdev = mxport->mxp_mxdev;

	/* only one open on any port on a device at a time */
	if (down_interruptible(&mxdev->mxd_open_close_sem))
		return -ERESTARTSYS;
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
	if (port->tty)
		port->tty->low_latency = MXU1_DEFAULT_LOW_LATENCY;
#else
//	if (port->port.tty)
//		port->port.tty->low_latency = MXU1_DEFAULT_LOW_LATENCY;
#endif	

	dbg("port_number: %d, port->minor: %d", port->port_number, port->minor);

#if(LINUX_VERSION_CODE > KERNEL_VERSION(3,10,0))
	port_number = port->port_number;
#else
	port_number = port->number - port->serial->minor;
#endif
	memset(&(mxport->mxp_icount), 0x00, sizeof(mxport->mxp_icount));

	mxport->mxp_msr = 0;
	mxport->mxp_shadow_mcr |= (MXU1_MCR_RTS | MXU1_MCR_DTR);

	/* start interrupt urb the first time a port is opened on this device */
	if (mxdev->mxd_open_port_count == 0) {
		dbg("%s - start interrupt in urb", __FUNCTION__);
		urb = mxdev->mxd_serial->port[0]->interrupt_in_urb;
		if (!urb) {
			dev_err(&port->dev, "%s - no interrupt urb\n", __FUNCTION__);
			status = -EINVAL;
			goto up_sem;
		}
		urb->complete = mxu1_interrupt_callback;
		urb->context = mxdev;
		urb->dev = dev;
		status = usb_submit_urb(urb, GFP_KERNEL);
		if (status) {
			dev_err(&port->dev, "%s - submit interrupt urb failed, %d\n", __FUNCTION__, status);
			goto up_sem;
		}
	}

#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
	mxu1_set_termios(port, NULL);
#else
	mxu1_set_termios(NULL, port, NULL);
#endif	

	dbg("%s - sending MXU1_OPEN_PORT", __FUNCTION__);
	status = mxu1_command_out_sync(mxdev, MXU1_OPEN_PORT,
		(__u8)(MXU1_UART1_PORT + port_number), open_settings, NULL, 0);
	if (status) {
		dev_err(&port->dev, "%s - cannot send open command, %d\n", __FUNCTION__, status);
		goto unlink_int_urb;
	}

	dbg("%s - sending MXU1_START_PORT", __FUNCTION__);
	status = mxu1_command_out_sync(mxdev, MXU1_START_PORT,
		(__u8)(MXU1_UART1_PORT + port_number), 0, NULL, 0);
	if (status) {
		dev_err(&port->dev, "%s - cannot send start command, %d\n", __FUNCTION__, status);
		goto unlink_int_urb;
	}

	dbg("%s - sending MXU1_PURGE_PORT", __FUNCTION__);
	status = mxu1_command_out_sync(mxdev, MXU1_PURGE_PORT,
		(__u8)(MXU1_UART1_PORT + port_number), MXU1_PURGE_INPUT, NULL, 0);
	if (status) {
		dev_err(&port->dev, "%s - cannot clear input buffers, %d\n", __FUNCTION__, status);
		goto unlink_int_urb;
	}
	status = mxu1_command_out_sync(mxdev, MXU1_PURGE_PORT,
		(__u8)(MXU1_UART1_PORT + port_number), MXU1_PURGE_OUTPUT, NULL, 0);
	if (status) {
		dev_err(&port->dev, "%s - cannot clear output buffers, %d\n", __FUNCTION__, status);
		goto unlink_int_urb;
	}

	/* reset the data toggle on the bulk endpoints to work around bug in
	 * host controllers where things get out of sync some times */
	usb_clear_halt(dev, port->write_urb->pipe);
	usb_clear_halt(dev, port->read_urb->pipe);
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
	mxu1_set_termios(port, NULL);
#else
	mxu1_set_termios(NULL,port, NULL);
#endif	

	dbg("%s - sending MXU1_OPEN_PORT (2)", __FUNCTION__);
	status = mxu1_command_out_sync(mxdev, MXU1_OPEN_PORT,
		(__u8)(MXU1_UART1_PORT + port_number), open_settings, NULL, 0);
	if (status) {
		dev_err(&port->dev, "%s - cannot send open command (2), %d\n", __FUNCTION__, status);
		goto unlink_int_urb;
	}

	dbg("%s - sending MXU1_START_PORT (2)", __FUNCTION__);
	status = mxu1_command_out_sync(mxdev, MXU1_START_PORT,
		(__u8)(MXU1_UART1_PORT + port_number), 0, NULL, 0);
	if (status) {
		dev_err(&port->dev, "%s - cannot send start command (2), %d\n", __FUNCTION__, status);
		goto unlink_int_urb;
	}

	/* start read urb */
	dbg("%s - start read urb", __FUNCTION__);
	urb = port->read_urb;
	if (!urb) {
		dev_err(&port->dev, "%s - no read urb\n", __FUNCTION__);
		status = -EINVAL;
		goto unlink_int_urb;
	}
	mxport->mxp_read_urb_state = MXU1_READ_URB_RUNNING;
	urb->complete = mxu1_bulk_in_callback;
	urb->context = mxport;
	urb->dev = dev;
	status = usb_submit_urb(urb, GFP_KERNEL);
	if (status) {
		dev_err(&port->dev, "%s - submit read urb failed, %d\n", __FUNCTION__, status);
		goto unlink_int_urb;
	}

	mxport->mxp_is_open = 1;
	++mxdev->mxd_open_port_count;

	goto up_sem;

unlink_int_urb:
	if (mxdev->mxd_open_port_count == 0){
#if(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,8))
		usb_kill_urb(port->serial->port[0]->interrupt_in_urb);
#else
		usb_unlink_urb(port->serial->port[0]->interrupt_in_urb);
#endif
	}
up_sem:
	up(&mxdev->mxd_open_close_sem);
	//dbg("%s - exit %d", __FUNCTION__, status);
	dbg("<< OUT %d >>", status);
	return status;
}

#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static void mxu1_close(struct usb_serial_port *port, struct file *file)
#elif(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,30))
static void mxu1_close(struct usb_serial_port *port)
#else
static void mxu1_close(struct tty_struct *tty, struct usb_serial_port *port, struct file *file)
#endif
{
	struct mxu1_device *mxdev;
	struct mxu1_port *mxport;
	int port_number;
	int status = 0;
	int do_up;

	//dbg("%s - port %d", __FUNCTION__, port->number);
			 
	mxdev = usb_get_serial_data(port->serial);
	mxport = usb_get_serial_port_data(port);
	if (mxdev == NULL || mxport == NULL)
		return;

	mxport->mxp_is_open = 0;

	mxu1_drain(mxport, (mxport->mxp_closing_wait*HZ)/100, 1);

#if(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,8))
	usb_kill_urb(port->read_urb);
	usb_kill_urb(port->write_urb);
#else
	usb_unlink_urb(port->read_urb);
	usb_unlink_urb(port->write_urb);
#endif
	
	mxport->mxp_write_urb_in_use = 0;

#if(LINUX_VERSION_CODE > KERNEL_VERSION(3,10,0))
	port_number = port->port_number;
#else
	port_number = port->number - port->serial->minor;
#endif

	dbg("%s - sending MXU1_CLOSE_PORT", __FUNCTION__);
	status = mxu1_command_out_sync(mxdev, MXU1_CLOSE_PORT,
		     (__u8)(MXU1_UART1_PORT + port_number), 0, NULL, 0);
	if (status)
		dev_err(&port->dev, "%s - cannot send close port command, %d\n" , __FUNCTION__, status);

	/* if down is interrupted, continue anyway */
	do_up = !down_interruptible(&mxdev->mxd_open_close_sem);
	--mxport->mxp_mxdev->mxd_open_port_count;
	if (mxport->mxp_mxdev->mxd_open_port_count <= 0) {
		/* last port is closed, shut down interrupt urb */
#if(LINUX_VERSION_CODE > KERNEL_VERSION(2,6,8))
		usb_kill_urb(port->serial->port[0]->interrupt_in_urb);
#else
		usb_unlink_urb(port->serial->port[0]->interrupt_in_urb);
#endif
		mxport->mxp_mxdev->mxd_open_port_count = 0;
	}
	if (do_up)
		up(&mxdev->mxd_open_close_sem);

	dbg("%s - exit", __FUNCTION__);
}

#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10))
static int mxu1_write (struct usb_serial_port *port, int from_user,
			const unsigned char *data, int count)
#else
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static int mxu1_write(struct usb_serial_port *port, const unsigned char *data,
	int count)
#else
static int mxu1_write(struct tty_struct *tty, struct usb_serial_port *port, const unsigned char *data,
	int count)
#endif	
#endif
{
	struct mxu1_port *mxport = usb_get_serial_port_data(port);

	dbg("%s - port %d", __FUNCTION__, port->port_number);

	if (count == 0) {
		dbg("%s - write request of 0 bytes", __FUNCTION__);
		return 0;
	}

	if (mxport == NULL || !mxport->mxp_is_open){
		return -ENODEV;
	}

#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10))
	count = mxu1_buf_put(mxport, data, count, from_user);
#else
	count = mxu1_buf_put(mxport, data, count);
#endif

	mxu1_send(mxport);

	return count;
}

#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static int mxu1_write_room(struct usb_serial_port *port)
{
#else
static int mxu1_write_room(struct tty_struct *tty)
{
struct usb_serial_port *port = tty->driver_data;
#endif

	
	struct mxu1_port *mxport = usb_get_serial_port_data(port);
	int room = 0;
	unsigned long flags;

	dbg("%s - port %d", __FUNCTION__, port->port_number);

	if (mxport == NULL)
		return -ENODEV;
	
	spin_lock_irqsave(&mxport->mxp_lock, flags);
	room = mxu1_buf_space_avail(mxport->mxp_write_buf);
	spin_unlock_irqrestore(&mxport->mxp_lock, flags);

	dbg("%s - returns %d", __FUNCTION__, room);

	return room;
}

#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static int mxu1_chars_in_buffer(struct usb_serial_port *port)
{
#else
static int mxu1_chars_in_buffer(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
#endif

	struct mxu1_port *mxport = usb_get_serial_port_data(port);
	int chars = 0;
	unsigned long flags;

	dbg("%s - port %d", __FUNCTION__, port->port_number);

	if (mxport == NULL)
		return -ENODEV;

	spin_lock_irqsave(&mxport->mxp_lock, flags);
	chars = mxu1_buf_data_avail(mxport->mxp_write_buf);
	spin_unlock_irqrestore(&mxport->mxp_lock, flags);

	dbg("%s - returns %d", __FUNCTION__, chars);

	return chars;
}

#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static void mxu1_throttle(struct usb_serial_port *port)
{
#else
static void mxu1_throttle(struct tty_struct *tty)
{
struct usb_serial_port *port = tty->driver_data;
#endif

	
	struct mxu1_port *mxport = usb_get_serial_port_data(port);
	
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
	struct tty_struct *tty;
#endif
	dbg("%s - port %d", __FUNCTION__, port->port_number);

	if (mxport == NULL)
		return;
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
	tty = port->tty;
#else
	tty = port->port.tty;
#endif	
	if (!tty) {
		dbg("%s - no tty", __FUNCTION__);
		return;
	}

	if (I_IXOFF(tty) || C_CRTSCTS(tty))
		mxu1_stop_read(mxport, tty);

}

#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static void mxu1_unthrottle(struct usb_serial_port *port)
{
#else
static void mxu1_unthrottle(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
#endif

	struct mxu1_port *mxport = usb_get_serial_port_data(port);
	
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))	
	struct tty_struct *tty;
#endif		
	int status = 0;

	dbg("%s - port %d", __FUNCTION__, port->port_number);

	if (mxport == NULL)
		return;

#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
	tty = port->tty;
#else
	tty = port->port.tty;
#endif	
	if (!tty) {
		dbg("%s - no tty", __FUNCTION__);
		return;
	}

	if (I_IXOFF(tty) || C_CRTSCTS(tty)) {
		status = mxu1_restart_read(mxport, tty);
		if (status)
			dev_err(&port->dev, "%s - cannot restart read, %d\n", __FUNCTION__, status);
	}
}

#if(LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,39))
static int mxu1_get_icount(struct tty_struct *tty,
		struct serial_icounter_struct *icount)
{
	struct usb_serial_port *port = tty->driver_data;
	struct mxu1_port *mxport = usb_get_serial_port_data(port);
	struct async_icount cnow = mxport->mxp_icount;

	dbg("%s - (%d) TIOCGICOUNT RX=%d, TX=%d",
		__func__, port->port_number,
		cnow.rx, cnow.tx);

	icount->cts = cnow.cts;
	icount->dsr = cnow.dsr;
	icount->rng = cnow.rng;
	icount->dcd = cnow.dcd;
	icount->rx = cnow.rx;
	icount->tx = cnow.tx;
	icount->frame = cnow.frame;
	icount->overrun = cnow.overrun;
	icount->parity = cnow.parity;
	icount->brk = cnow.brk;
	icount->buf_overrun = cnow.buf_overrun;

	return 0;
}
#endif

#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static int mxu1_ioctl(struct usb_serial_port *port, struct file *file,
	unsigned int cmd, unsigned long arg)
{	
#else
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39))
static int mxu1_ioctl(struct tty_struct *tty,struct file *file,
	unsigned int cmd, unsigned long arg)
#else
static int mxu1_ioctl(struct tty_struct *tty,
	unsigned int cmd, unsigned long arg)
#endif
{	
	struct usb_serial_port *port = tty->driver_data;
#endif	

	struct mxu1_port *mxport = usb_get_serial_port_data(port);
	struct async_icount cnow;
	struct async_icount cprev;

	dbg("%s - port %d, cmd = 0x%04X", __FUNCTION__, port->port_number, cmd);

	if (mxport == NULL)
		return -ENODEV;

	switch (cmd) {
		case TIOCGSERIAL:
			dbg("%s - (%d) TIOCGSERIAL", __FUNCTION__, port->port_number);
			return mxu1_get_serial_info(mxport, (struct serial_struct __user *)arg);

		case TIOCSSERIAL:
			dbg("%s - (%d) TIOCSSERIAL", __FUNCTION__, port->port_number);
			return mxu1_set_serial_info(mxport, (struct serial_struct __user *)arg);

		case TIOCMIWAIT:
			dbg("%s - (%d) TIOCMIWAIT", __FUNCTION__, port->port_number);
			cprev = mxport->mxp_icount;
			mxport->mxp_msr_wait_flags = 1;
			while (1) {
				wait_event_interruptible_timeout(mxport->mxp_msr_wait,(mxport->mxp_msr_wait_flags == 0),MXU1_MSR_WAIT_TIMEOUT);
				if (signal_pending(current))
					return -ERESTARTSYS;
				cnow = mxport->mxp_icount;
				if (cnow.rng == cprev.rng && cnow.dsr == cprev.dsr &&
				    cnow.dcd == cprev.dcd && cnow.cts == cprev.cts)
					return -EIO; /* no change => error */
				if (((arg & TIOCM_RNG) && (cnow.rng != cprev.rng)) ||
				    ((arg & TIOCM_DSR) && (cnow.dsr != cprev.dsr)) ||
				    ((arg & TIOCM_CD)  && (cnow.dcd != cprev.dcd)) ||
				    ((arg & TIOCM_CTS) && (cnow.cts != cprev.cts)) ) {
					return 0;
				}
				cprev = cnow;
			}
			break;

		case TIOCGICOUNT:
			dbg("%s - (%d) TIOCGICOUNT RX=%d, TX=%d", __FUNCTION__, port->port_number, mxport->mxp_icount.rx, mxport->mxp_icount.tx);
			if (copy_to_user((void __user *)arg, &mxport->mxp_icount, sizeof(mxport->mxp_icount)))
				return -EFAULT;
			return 0;

		case MOXA_SET_INTERFACE:
			dbg("%s - port%d MOXA_SET_INTERFACE=%d",__FUNCTION__,port->port_number,(int)arg);
			
			if(mxport->mxp_mxdev->mxd_model_name != MXU1_MODEL_1110){ //UPort 1130, 1150, 1150I
				switch(arg){
					case MXU1_RS232: //UPort 1150, 1150I
						if(mxport->mxp_mxdev->mxd_model_name == MXU1_MODEL_1130 || 
						mxport->mxp_mxdev->mxd_model_name == MXU1_MODEL_1131){
							return -EINVAL;
						}
						mxport->mxp_uart_mode = MXU1_UART_232;
						break;
					case MXU1_RS422: //UPort 1130, 1150, 1150I
					case MXU1_RS4854W: //UPort 1130, 1150, 1150I
						mxport->mxp_uart_mode = MXU1_UART_485_RECEIVER_ENABLED;
						break;

					case MXU1_RS4852W: //UPort 1130, 1150, 1150I
						mxport->mxp_uart_mode = MXU1_UART_485_RECEIVER_DISABLED;
						break;
					default:
						return -EINVAL;
                     		}
			}else{ //UPort 1110
				if(arg != MXU1_RS232)
					return -EINVAL;
			}
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
			mxu1_set_termios(port, NULL);
#else
			mxu1_set_termios(NULL, port, NULL);
#endif			
	}

	return -ENOIOCTLCMD;
}

#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20))		
static void mxu1_set_termios(struct usb_serial_port *port,
	struct termios *old_termios)
#else
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static void mxu1_set_termios(struct usb_serial_port *port,
	struct ktermios *old_termios)
#else
static void mxu1_set_termios(struct tty_struct *tty1, struct usb_serial_port *port,
	struct ktermios *old_termios)
#endif	
#endif
{
	struct mxu1_port *mxport = usb_get_serial_port_data(port);
	
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
	struct tty_struct *tty = port->tty;
#else
	struct tty_struct *tty = port->port.tty;

#endif
	
	struct mxu1_uart_config *config;
	tcflag_t cflag,iflag;
	int baud;
	int status = 0;
	int port_number;
	unsigned int mcr;

	dbg("%s", "<< IN >>");

#if(LINUX_VERSION_CODE > KERNEL_VERSION(3,10,0))
	port_number = port->port_number;
#else
	port_number = port->number - port->serial->minor;
#endif

	dbg("port_number %d, port %d, minor %d", port_number, port->port_number, port->minor);

#if(LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0))
	if (!tty || !tty->termios) {
#else
	if (!tty) {
#endif
		dbg("%s - no tty or termios", __FUNCTION__);
		return;
	}
#if(LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0))
	cflag = tty->termios->c_cflag;
	iflag = tty->termios->c_iflag;
#else
	cflag = tty->termios.c_cflag;
	iflag = tty->termios.c_iflag;
#endif
	if (old_termios && cflag == old_termios->c_cflag
	&& iflag == old_termios->c_iflag) {
		dbg("%s - nothing to change", __FUNCTION__);
		return;
	}

	dbg("%s - clfag %08x, iflag %08x", __FUNCTION__, cflag, iflag);

	if (old_termios)
		dbg("%s - old clfag %08x, old iflag %08x", __FUNCTION__, old_termios->c_cflag, old_termios->c_iflag);

	if (mxport == NULL)
		return;

	config = kmalloc(sizeof(*config), GFP_KERNEL);
	if (!config) {
		dev_err(&port->dev, "%s - out of memory\n", __FUNCTION__);
		return;
	}

	config->wFlags = 0;

	/* these flags must be set */
	config->wFlags |= MXU1_UART_ENABLE_MS_INTS;
	config->wFlags |= MXU1_UART_ENABLE_AUTO_START_DMA;
	if(mxport->mxp_send_break == MXU1_LCR_BREAK)
		config->wFlags |= MXU1_UART_SEND_BREAK_SIGNAL;
	config->bUartMode = (__u8)(mxport->mxp_uart_mode);

	switch (cflag & CSIZE) {
		case CS5:
			    config->bDataBits = MXU1_UART_5_DATA_BITS;
			    break;
		case CS6:
			    config->bDataBits = MXU1_UART_6_DATA_BITS;
			    break;
		case CS7:
			    config->bDataBits = MXU1_UART_7_DATA_BITS;
			    break;
		default:
		case CS8:
			    config->bDataBits = MXU1_UART_8_DATA_BITS;
			    break;
	}

	if (cflag & PARENB) {
		if (cflag & PARODD) {
			config->wFlags |= MXU1_UART_ENABLE_PARITY_CHECKING;
			config->bParity = MXU1_UART_ODD_PARITY;
		} else {
			config->wFlags |= MXU1_UART_ENABLE_PARITY_CHECKING;
			config->bParity = MXU1_UART_EVEN_PARITY;
		}
	} else {
		config->wFlags &= ~MXU1_UART_ENABLE_PARITY_CHECKING;
		config->bParity = MXU1_UART_NO_PARITY; 	
	}

	if (cflag & CSTOPB)
		config->bStopBits = MXU1_UART_2_STOP_BITS;
	else
		config->bStopBits = MXU1_UART_1_STOP_BITS;

	if (cflag & CRTSCTS) {
		/* RTS flow control must be off to drop RTS for baud rate B0 */
		if ((cflag & CBAUD) != B0)
			config->wFlags |= MXU1_UART_ENABLE_RTS_IN;
		config->wFlags |= MXU1_UART_ENABLE_CTS_OUT;
	} else {
		tty->hw_stopped = 0;
		mxu1_restart_read(mxport, tty);
	}

	if (I_IXOFF(tty) || I_IXON(tty)) {
		config->cXon  = START_CHAR(tty);
		config->cXoff = STOP_CHAR(tty);

		if (I_IXOFF(tty))
			config->wFlags |= MXU1_UART_ENABLE_X_IN;
		else
			mxu1_restart_read(mxport, tty);

		if (I_IXON(tty))
			config->wFlags |= MXU1_UART_ENABLE_X_OUT;
	}

	baud = tty_get_baud_rate(tty);
	if (!baud) baud = 9600;
	config->wBaudRate = (__u16)(923077 / baud);

	dbg("%s - BaudRate=%d, wBaudRate=%d, wFlags=0x%04X, bDataBits=%d, bParity=%d, bStopBits=%d, cXon=%d, cXoff=%d, bUartMode=%d",
	__FUNCTION__, baud, config->wBaudRate, config->wFlags, config->bDataBits, config->bParity, config->bStopBits, config->cXon, config->cXoff, config->bUartMode);

	cpu_to_be16s(&config->wBaudRate);
	cpu_to_be16s(&config->wFlags);
	status = mxu1_command_out_sync(mxport->mxp_mxdev, MXU1_SET_CONFIG,
		(__u8)(MXU1_UART1_PORT + port_number), 0, (__u8 *)config,
		sizeof(*config));
	if (status)
		dev_err(&port->dev, "%s - cannot set config on port %d, %d\n", __FUNCTION__, port_number, status);

	/* SET_CONFIG asserts RTS and DTR, reset them correctly */
	mcr = mxport->mxp_shadow_mcr;
	/* if baud rate is B0, clear RTS and DTR */
	if ((cflag & CBAUD) == B0) {

		mcr &= ~(MXU1_MCR_DTR | MXU1_MCR_RTS);
		mxport->mxp_set_B0 = 1;
	}
	else {
		if(mxport->mxp_set_B0)
			mcr |= MXU1_MCR_DTR;
		
		mxport->mxp_set_B0 = 0;
	}

	status = mxu1_set_mcr(mxport, mcr);

	if (status)
		dev_err(&port->dev, "%s - cannot set modem control on port %d, %d\n", __FUNCTION__, port_number, status);

	dbg("%s", "<< OUT >>");


	kfree(config);
}

#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static int mxu1_tiocmget(struct usb_serial_port *port, struct file *file)
{
#else
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39))
static int mxu1_tiocmget(struct tty_struct *tty, struct file *file)
#else
static int mxu1_tiocmget(struct tty_struct *tty)
#endif
{
	struct usb_serial_port *port = tty->driver_data;
#endif

	struct mxu1_port *mxport = usb_get_serial_port_data(port);
	unsigned int result;
	unsigned int msr;
	unsigned int mcr;

	dbg("%s - port %d", __FUNCTION__, port->port_number);

	if (mxport == NULL)
		return -ENODEV;

	msr = mxport->mxp_msr;
	mcr = mxport->mxp_shadow_mcr;

	result = ((mcr & MXU1_MCR_DTR) ? TIOCM_DTR : 0)
		| ((mcr & MXU1_MCR_RTS) ? TIOCM_RTS : 0)
		| ((mcr & MXU1_MCR_LOOP) ? TIOCM_LOOP : 0)
		| ((msr & MXU1_MSR_CTS) ? TIOCM_CTS : 0)
		| ((msr & MXU1_MSR_CD) ? TIOCM_CAR : 0)
		| ((msr & MXU1_MSR_RI) ? TIOCM_RI : 0)
		| ((msr & MXU1_MSR_DSR) ? TIOCM_DSR : 0);

	dbg("%s - 0x%04X", __FUNCTION__, result);

	return result;
}

#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static int mxu1_tiocmset(struct usb_serial_port *port, struct file *file,
	unsigned int set, unsigned int clear)
{
#else
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,39))
static int mxu1_tiocmset(struct tty_struct *tty, struct file *file,
	unsigned int set, unsigned int clear)
#else
static int mxu1_tiocmset(struct tty_struct *tty,
	unsigned int set, unsigned int clear)
#endif
{	
	struct usb_serial_port *port = tty->driver_data;
#endif	

	struct mxu1_port *mxport = usb_get_serial_port_data(port);
	unsigned int mcr;

	dbg("%s - port %d", __FUNCTION__, port->port_number);

	if (mxport == NULL)
		return -ENODEV;

	mcr = mxport->mxp_shadow_mcr;

	if (set & TIOCM_RTS)
		mcr |= MXU1_MCR_RTS;
	if (set & TIOCM_DTR)
		mcr |= MXU1_MCR_DTR;
	if (set & TIOCM_LOOP)
		mcr |= MXU1_MCR_LOOP;

	if (clear & TIOCM_RTS)
		mcr &= ~MXU1_MCR_RTS;
	if (clear & TIOCM_DTR)
		mcr &= ~MXU1_MCR_DTR;
	if (clear & TIOCM_LOOP)
		mcr &= ~MXU1_MCR_LOOP;

	return mxu1_set_mcr(mxport, mcr);
}

#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
static void mxu1_break(struct usb_serial_port *port, int break_state)
{
#else
static void mxu1_break(struct tty_struct *tty, int break_state)
{
	struct usb_serial_port *port = tty->driver_data;
#endif

	struct mxu1_port *mxport = usb_get_serial_port_data(port);

	dbg("%s - state = %d", __FUNCTION__, break_state);

	if (mxport == NULL)
		return;

	mxu1_drain(mxport, (mxport->mxp_closing_wait*HZ)/100, 0);

	if(break_state == -1)
		mxport->mxp_send_break = MXU1_LCR_BREAK;
	else
		mxport->mxp_send_break = 0;
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
	mxu1_set_termios(mxport->mxp_port,NULL);
#else
	mxu1_set_termios(NULL, mxport->mxp_port,NULL);
#endif	
}


#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))		
static void mxu1_interrupt_callback(struct urb *urb, struct pt_regs *regs)
#else
static void mxu1_interrupt_callback(struct urb *urb)
#endif
{
	struct mxu1_device *mxdev = (struct mxu1_device *)urb->context;
	struct usb_serial_port *port;
	struct usb_serial *serial = mxdev->mxd_serial;
	struct mxu1_port *mxport;
	struct device *dev = &urb->dev->dev;
	unsigned char *data = urb->transfer_buffer;
	int length = urb->actual_length;
	int port_number;
	int function;
	int status = 0;
	__u8 msr;

	dbg("%s", __FUNCTION__);
	
	/* Check port is valid or not */
	if(mxdev == NULL)
		return;
	

	switch (urb->status) {
	case 0:
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		dbg("%s - urb shutting down, %d", __FUNCTION__, urb->status);
		mxdev->mxd_urb_error = 1;
		return;
	default:
		dev_err(dev, "%s - nonzero urb status, %d\n", __FUNCTION__, urb->status);
		mxdev->mxd_urb_error = 1;
		goto exit;
	}

	if (length != 2) {
		dbg("%s - bad packet size, %d", __FUNCTION__, length);
		goto exit;
	}

	if (data[0] == MXU1_CODE_HARDWARE_ERROR) {
		dev_err(dev, "%s - hardware error, %d\n", __FUNCTION__, data[1]);
		goto exit;
	}

	port_number = MXU1_GET_PORT_FROM_CODE(data[0]);
	function = MXU1_GET_FUNC_FROM_CODE(data[0]);

	dbg("%s - port_number %d, function %d, data 0x%02X", __FUNCTION__, port_number, function, data[1]);

	if (port_number >= serial->num_ports) {
		dev_err(dev, "%s - bad port number, %d\n", __FUNCTION__, port_number);
		goto exit;
	}

	port = serial->port[port_number];

	mxport = usb_get_serial_port_data(port);
	if (!mxport)
		goto exit;

	switch (function) {
	case MXU1_CODE_DATA_ERROR:
		dbg("%s - DATA ERROR, port %d, data 0x%02X\n", __FUNCTION__, port_number, data[1]);
		break;

	case MXU1_CODE_MODEM_STATUS:
		msr = data[1];
		dbg("%s - port %d, msr 0x%02X", __FUNCTION__, port_number, msr);
		mxu1_handle_new_msr(mxport, msr);
		break;

	default:
		dev_err(dev, "%s - unknown interrupt code, 0x%02X\n", __FUNCTION__, data[1]);
		break;
	}

exit:
	status = usb_submit_urb(urb, GFP_ATOMIC);
	if (status)
		dev_err(dev, "%s - resubmit interrupt urb failed, %d\n", __FUNCTION__, status);
}


#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))		
static void mxu1_bulk_in_callback(struct urb *urb, struct pt_regs *regs)
#else
static void mxu1_bulk_in_callback(struct urb *urb)
#endif
{
	struct mxu1_port *mxport = (struct mxu1_port *)urb->context;
	struct usb_serial_port *port = mxport->mxp_port;
	struct device *dev = &urb->dev->dev;
	int status = 0;

	dbg("%s", __FUNCTION__);
	
	/*Check port is valid or not*/
	if(mxport == NULL)
		return;

	switch (urb->status) {
	case 0:
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		dbg("%s - urb shutting down, %d", __FUNCTION__, urb->status);
		mxport->mxp_mxdev->mxd_urb_error = 1;
		wake_up_interruptible(&mxport->mxp_write_wait);
		return;
	default:
		dev_err(dev, "%s - nonzero urb status, %d\n", __FUNCTION__, urb->status );
		mxport->mxp_mxdev->mxd_urb_error = 1;
		wake_up_interruptible(&mxport->mxp_write_wait);
	}

	if (urb->status == -EPIPE)
		goto exit;

	if (urb->status) {
		dev_err(dev, "%s - stopping read!\n", __FUNCTION__);
		return;
	}
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
	if (port->tty && urb->actual_length) {
#else
	if (port->port.tty && urb->actual_length) {
#endif
		
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9))		
		usb_serial_debug_data(dev->driver->name, __FUNCTION__,
			urb->actual_length, urb->transfer_buffer);
#elif(LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0))
		usb_serial_debug_data(debug, dev, __FUNCTION__,
			urb->actual_length, urb->transfer_buffer);
#else
		usb_serial_debug_data(dev, __FUNCTION__,
			urb->actual_length, urb->transfer_buffer);
#endif

		if (!mxport->mxp_is_open)
			dbg("%s - port closed, dropping data", __FUNCTION__);
		else
		    mxu1_recv(mxport, urb->transfer_buffer, urb->actual_length);
			//mxu1_recv(&urb->dev->dev, port->tty, urb->transfer_buffer,urb->actual_length);
		spin_lock(&mxport->mxp_lock);
		mxport->mxp_icount.rx += urb->actual_length;
		spin_unlock(&mxport->mxp_lock);
	}

exit:
	/* continue to read unless stopping */
	spin_lock(&mxport->mxp_lock);
	if (mxport->mxp_read_urb_state == MXU1_READ_URB_RUNNING) {
		urb->dev = port->serial->dev;
		status = usb_submit_urb(urb, GFP_ATOMIC);
	} else if (mxport->mxp_read_urb_state == MXU1_READ_URB_STOPPING) {
		mxport->mxp_read_urb_state = MXU1_READ_URB_STOPPED;
	}
	spin_unlock(&mxport->mxp_lock);
	if (status)
		dev_err(dev, "%s - resubmit read urb failed, %d\n", __FUNCTION__, status);
}


#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19))		
static void mxu1_bulk_out_callback(struct urb *urb, struct pt_regs *regs)
#else
static void mxu1_bulk_out_callback(struct urb *urb)
#endif
{
	struct mxu1_port *mxport = (struct mxu1_port *)urb->context;
	struct usb_serial_port *port = mxport->mxp_port;
#if(LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0))			
	struct device *dev = &urb->dev->dev;
#endif

	dbg("%s - port %d", __FUNCTION__, port->port_number);

	/* Check port is valid or not */
	if(mxport == NULL)
		return;
	
	mxport->mxp_write_urb_in_use = 0;

	switch (urb->status) {
	case 0:
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		dbg("%s - urb shutting down, %d", __FUNCTION__, urb->status);
		mxport->mxp_mxdev->mxd_urb_error = 1;
		wake_up_interruptible(&mxport->mxp_write_wait);
		return;
	default:
#if(LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0))		
		dev_err(dev, "%s - nonzero urb status, %d\n", __FUNCTION__, urb->status);
#else
		dev_err_console(port, "%s - nonzero urb status, %d\n", __FUNCTION__, urb->status);
#endif		
		mxport->mxp_mxdev->mxd_urb_error = 1;
		wake_up_interruptible(&mxport->mxp_write_wait);
	}

	/* send any buffered data */
	mxu1_send(mxport);
}

#if 1
#if defined(_SCREEN_INFO_H) || (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,15))
static void mxu1_recv(struct mxu1_port *mxport,
	                  unsigned char *data, int length)
{
	int cnt;
	struct tty_struct *tty;
#if(LINUX_VERSION_CODE > KERNEL_VERSION(3,10,0))
	struct tty_port *tty_port = &mxport->mxp_port->port;
#endif
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
	tty = mxport->mxp_port->tty;
#else

	tty = mxport->mxp_port->port.tty;
#endif	

	do{
	    	if (mxport->mxp_read_urb_state == MXU1_READ_URB_STOPPING){
        	    	dbg("%s - [1] dropping data, %d bytes lost\n", __FUNCTION__, length);			    
            		break;
        	}
		
#if(LINUX_VERSION_CODE > KERNEL_VERSION(3,10,0))
		cnt = tty_buffer_request_room(tty_port, length);
#else
		cnt = tty_buffer_request_room(tty, length);
#endif
	        if(cnt < length){
		            dbg("%s - [2] dropping data, %d bytes lost\n", __FUNCTION__, length);			    
		            break;            
		}

		cnt = length;

#if(LINUX_VERSION_CODE > KERNEL_VERSION(3,10,0))
		tty_insert_flip_string(tty_port, data, cnt);
#else
		tty_insert_flip_string(tty, data, cnt);
#endif
		length -= cnt;

#if(LINUX_VERSION_CODE > KERNEL_VERSION(3,10,0))
		tty_flip_buffer_push(tty_port);
#else
		tty_flip_buffer_push(tty);
#endif
	}while(length > 0);

}
#else
static void mxu1_recv(struct mxu1_port *mxport,
	                  unsigned char *data, int length)
{
	int i, cnt;
    struct tty_struct   *tty;
    char            	*fp;
    unsigned char   	*ch;

    tty = mxport->mxp_port->tty;

    ch = tty->flip.char_buf;
    fp = tty->flip.flag_buf;

	do {

	    	if (mxport->mxp_read_urb_state == MXU1_READ_URB_STOPPING){
        	    	dbg("%s - [1] dropping data, %d bytes lost\n", __FUNCTION__, length);			    
            		break;
        	}
        
	        cnt = tty->ldisc.receive_room(tty);
        
	        if(cnt < length){
		            dbg("%s - [2] dropping data, %d bytes lost\n", __FUNCTION__, length);			    
		            break;            
		}
        
	        cnt = length;                    
        
	        for(i=0; i<cnt; i++){
	            *ch++=data[i];
	            *fp++=TTY_NORMAL;
	            tty->flip.count++;
	        }

		length -= cnt;

        	tty->ldisc.receive_buf(tty, 
                	               tty->flip.char_buf, 
                        	       tty->flip.flag_buf,
                               		cnt); 
		
	} while (length > 0);

}
#endif
#else
static void mxu1_recv(struct device *dev, struct tty_struct *tty,
	unsigned char *data, int length)
{
	int cnt;

	do {
#ifdef _SCREEN_INFO_H
		cnt = tty_buffer_request_room(tty, length);
		if(cnt<length){
			dev_err(dev, "%s - dropping data, %d bytes lost\n", __FUNCTION__, length-cnt);
			if(cnt==0)
				break;
		}
		tty_insert_flip_string(tty,data,cnt);
		tty_flip_buffer_push(tty);
#else
		if (tty->flip.count >= TTY_FLIPBUF_SIZE) {
			tty_flip_buffer_push(tty);
			if (tty->flip.count >= TTY_FLIPBUF_SIZE) {
				dev_err(dev, "%s - dropping data, %d bytes lost\n", __FUNCTION__, length);
				return;
			}
		}
		cnt = min(length, TTY_FLIPBUF_SIZE - tty->flip.count);
		memcpy(tty->flip.char_buf_ptr, data, cnt);
		memset(tty->flip.flag_buf_ptr, 0, cnt);
		tty->flip.char_buf_ptr += cnt;
		tty->flip.flag_buf_ptr += cnt;
		tty->flip.count += cnt;
#endif
		data += cnt;
		length -= cnt;
	} while (length > 0);
#ifndef _SCREEN_INFO_H
	tty_flip_buffer_push(tty);
#endif
}
#endif

static void mxu1_send(struct mxu1_port *mxport)
{
	int count, result;
	struct usb_serial_port *port = mxport->mxp_port;
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
	struct tty_struct *tty = port->tty;
#else
	struct tty_struct *tty = port->port.tty;

#endif	
	unsigned long flags;

	dbg("%s - port %d", __FUNCTION__, port->port_number);

	spin_lock_irqsave(&mxport->mxp_lock, flags);

	if (mxport->mxp_write_urb_in_use) {
		spin_unlock_irqrestore(&mxport->mxp_lock, flags);
		return;
	}

	count = mxu1_buf_get(mxport->mxp_write_buf,
				port->write_urb->transfer_buffer,
				port->bulk_out_size);

	if (count == 0) {
		spin_unlock_irqrestore(&mxport->mxp_lock, flags);
		return;
	}

	mxport->mxp_write_urb_in_use = 1;

	spin_unlock_irqrestore(&mxport->mxp_lock, flags);
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9))
	usb_serial_debug_data(&port->number, __FUNCTION__, count, port->write_urb->transfer_buffer);
#elif(LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0))
	usb_serial_debug_data(debug, &port->dev, __FUNCTION__, count, port->write_urb->transfer_buffer);
#else
	usb_serial_debug_data(&port->dev, __FUNCTION__, count, port->write_urb->transfer_buffer);
#endif
	
	usb_fill_bulk_urb(port->write_urb, port->serial->dev,
			   usb_sndbulkpipe(port->serial->dev,
					    port->bulk_out_endpointAddress),
			   port->write_urb->transfer_buffer, count,
			   mxu1_bulk_out_callback, mxport);

	result = usb_submit_urb(port->write_urb, GFP_ATOMIC);
	if (result) {
#if(LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0))	
		dev_err(&port->dev, "%s - submit write urb failed, %d\n", __FUNCTION__, result);
#else
		dev_err_console(port, "%s - submit write urb failed, %d\n", __FUNCTION__, result);
#endif		
		mxport->mxp_write_urb_in_use = 0; 
		/* TODO: reschedule mxu1_send */
	} else {
		spin_lock_irqsave(&mxport->mxp_lock, flags);
		mxport->mxp_icount.tx += count;
		spin_unlock_irqrestore(&mxport->mxp_lock, flags);
	}

	/* more room in the buffer for new writes, wakeup */
	if (tty){
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9))
		schedule_work(&port->work);
#else
		tty_wakeup(tty);
#endif
	}
	wake_up_interruptible(&mxport->mxp_write_wait);
}


static int mxu1_set_mcr(struct mxu1_port *mxport, unsigned int mcr)
{
	int status = 0;

	status = mxu1_write_byte(mxport->mxp_mxdev,
		mxport->mxp_uart_base_addr + MXU1_UART_OFFSET_MCR,
		MXU1_MCR_RTS | MXU1_MCR_DTR | MXU1_MCR_LOOP, mcr);

	if (!status)
		mxport->mxp_shadow_mcr = mcr;

	return status;
}


static int mxu1_get_lsr(struct mxu1_port *mxport)
{
	int size,status;
	struct mxu1_device *mxdev = mxport->mxp_mxdev;
	struct usb_serial_port *port = mxport->mxp_port;
#if(LINUX_VERSION_CODE > KERNEL_VERSION(3,10,0))
	int port_number = port->port_number;
#else
	int port_number = port->number - port->serial->minor;
#endif
	struct mxu1_port_status *data;

	dbg("%s - port %d", __FUNCTION__, port->port_number);

	size = sizeof(struct mxu1_port_status);
	data = kmalloc(size, GFP_KERNEL);
	if (!data) {
		dev_err(&port->dev, "%s - out of memory\n", __FUNCTION__);
		return -ENOMEM;
	}

	status = mxu1_command_in_sync(mxdev, MXU1_GET_PORT_STATUS,
		(__u8)(MXU1_UART1_PORT+port_number), 0, (__u8 *)data, size);
	if (status) {
		dev_err(&port->dev, "%s - get port status command failed, %d\n", __FUNCTION__, status);
		goto free_data;
	}

	dbg("%s - lsr 0x%02X", __FUNCTION__, data->bLSR);

	mxport->mxp_lsr = data->bLSR;

free_data:
	kfree(data);
	return status;
}


static int mxu1_get_serial_info(struct mxu1_port *mxport,
	struct serial_struct __user *ret_arg)
{
	struct usb_serial_port *port = mxport->mxp_port;
	struct serial_struct ret_serial;

	if (!ret_arg)
		return -EFAULT;

	memset(&ret_serial, 0, sizeof(ret_serial));

	ret_serial.type = PORT_16550A;
#if(LINUX_VERSION_CODE > KERNEL_VERSION(3,10,0))
	ret_serial.line = port->minor;
#else
	ret_serial.line = port->serial->minor;
#endif
	ret_serial.port = mxport->mxp_user_get_uart_mode;
	ret_serial.flags = mxport->mxp_flags;
	ret_serial.xmit_fifo_size = MXU1_WRITE_BUF_SIZE;
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))	
	ret_serial.baud_base = tty_get_baud_rate(mxport->mxp_port->tty);
#else
	ret_serial.baud_base = tty_get_baud_rate(mxport->mxp_port->port.tty);
#endif	
	ret_serial.closing_wait = mxport->mxp_closing_wait;

	if (copy_to_user(ret_arg, &ret_serial, sizeof(*ret_arg)))
		return -EFAULT;

	return 0;
}


static int mxu1_set_serial_info(struct mxu1_port *mxport,
	struct serial_struct __user *new_arg)
{
	struct serial_struct new_serial;

	if (copy_from_user(&new_serial, new_arg, sizeof(new_serial)))
		return -EFAULT;

	mxport->mxp_flags = new_serial.flags & MXU1_SET_SERIAL_FLAGS;
	mxport->mxp_closing_wait = new_serial.closing_wait;

	if(mxport->mxp_mxdev->mxd_model_name != MXU1_MODEL_1110){ //UPort 1130, 1150, 1150I
		switch(new_serial.port){
			case MXU1_RS232: //UPort 1150, 1150I
				if(mxport->mxp_mxdev->mxd_model_name == MXU1_MODEL_1130 || mxport->mxp_mxdev->mxd_model_name == MXU1_MODEL_1131){
					return -EINVAL;
				}

				mxport->mxp_uart_mode = MXU1_UART_232;
				mxport->mxp_user_get_uart_mode = MXU1_RS232;
				break;
			case MXU1_RS422: //UPort 1130, 1150, 1150I
				mxport->mxp_uart_mode = MXU1_UART_485_RECEIVER_ENABLED;
				mxport->mxp_user_get_uart_mode = MXU1_RS422;
				break;
			case MXU1_RS4854W: //UPort 1130, 1150, 1150I
				mxport->mxp_uart_mode = MXU1_UART_485_RECEIVER_ENABLED;
				mxport->mxp_user_get_uart_mode = MXU1_RS4854W;
				break;
			case MXU1_RS4852W: //UPort 1130, 1150, 1150I
				mxport->mxp_uart_mode = MXU1_UART_485_RECEIVER_DISABLED;
				mxport->mxp_user_get_uart_mode = MXU1_RS4852W;
				break;
			default:
				return -EINVAL;
               	}
	}else{ //UPort 1110
		if(new_serial.port != MXU1_RS232)
			return -EINVAL;
	}
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))
	mxu1_set_termios(mxport->mxp_port, NULL);
#else
	mxu1_set_termios(NULL, mxport->mxp_port, NULL);
#endif	

	return 0;
}


static void mxu1_handle_new_msr(struct mxu1_port *mxport, __u8 msr)
{
	struct async_icount *icount;
	struct tty_struct *tty;
	unsigned long flags;

	dbg("%s - msr 0x%02X", __FUNCTION__, msr);

	if (msr & MXU1_MSR_DELTA_MASK) {
		spin_lock_irqsave(&mxport->mxp_lock, flags);
		icount = &mxport->mxp_icount;
		if (msr & MXU1_MSR_DELTA_CTS)
			icount->cts++;
		if (msr & MXU1_MSR_DELTA_DSR)
			icount->dsr++;
		if (msr & MXU1_MSR_DELTA_CD)
			icount->dcd++;
		if (msr & MXU1_MSR_DELTA_RI)
			icount->rng++;
		if(mxport->mxp_msr_wait_flags == 1){
			mxport->mxp_msr_wait_flags = 0;
			wake_up_interruptible(&mxport->mxp_msr_wait);
		}
		spin_unlock_irqrestore(&mxport->mxp_lock, flags);
	}

	mxport->mxp_msr = msr & MXU1_MSR_MASK;

	/* handle CTS flow control */
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27))	
	tty = mxport->mxp_port->tty;
#else
	tty = mxport->mxp_port->port.tty;
#endif	
	if (tty && C_CRTSCTS(tty)) {
		if (msr & MXU1_MSR_CTS) {
			tty->hw_stopped = 0;
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9))
			schedule_work(&mxport->mxp_port->work);
#else
			tty_wakeup(tty);
#endif
		} else {
			tty->hw_stopped = 1;
		}
	}
}


static void mxu1_drain(struct mxu1_port *mxport, unsigned long timeout, int flush)
{
	struct mxu1_device *mxdev = mxport->mxp_mxdev;
	struct usb_serial_port *port = mxport->mxp_port;
	wait_queue_t wait;
	unsigned long flags;

	dbg("%s - port %d", __FUNCTION__, port->port_number);

	spin_lock_irqsave(&mxport->mxp_lock, flags);

	/* wait for data to drain from the buffer */
	mxdev->mxd_urb_error = 0;
	init_waitqueue_entry(&wait, current);
	add_wait_queue(&mxport->mxp_write_wait, &wait);
	for (;;) {
		set_current_state(TASK_INTERRUPTIBLE);
		if (mxu1_buf_data_avail(mxport->mxp_write_buf) == 0
		|| timeout == 0 || signal_pending(current)
		|| mxdev->mxd_urb_error
		|| !usb_get_intfdata(port->serial->interface))  /* disconnect */
			break;
		spin_unlock_irqrestore(&mxport->mxp_lock, flags);
		timeout = schedule_timeout(timeout);
		spin_lock_irqsave(&mxport->mxp_lock, flags);
	}
	set_current_state(TASK_RUNNING);
	remove_wait_queue(&mxport->mxp_write_wait, &wait);

	/* flush any remaining data in the buffer */
	if (flush)
		mxu1_buf_clear(mxport->mxp_write_buf);

	spin_unlock_irqrestore(&mxport->mxp_lock, flags);

	/* wait for data to drain from the device */
	/* wait for empty tx register, plus 20 ms */
	timeout += jiffies;
	mxport->mxp_lsr &= ~MXU1_LSR_TX_EMPTY;
	while ((long)(jiffies - timeout) < 0 && !signal_pending(current)
	&& !(mxport->mxp_lsr&MXU1_LSR_TX_EMPTY) && !mxdev->mxd_urb_error
	&& usb_get_intfdata(port->serial->interface)) {  /* not disconnected */
		if (mxu1_get_lsr(mxport))
			break;
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9))
		mxu1_jsleep_interruptible(200);
#else
		msleep_interruptible(20);
#endif
	}
}


static void mxu1_stop_read(struct mxu1_port *mxport, struct tty_struct *tty)
{
	unsigned long flags;

	spin_lock_irqsave(&mxport->mxp_lock, flags);

	if (mxport->mxp_read_urb_state == MXU1_READ_URB_RUNNING)
		mxport->mxp_read_urb_state = MXU1_READ_URB_STOPPING;

	spin_unlock_irqrestore(&mxport->mxp_lock, flags);
}


static int mxu1_restart_read(struct mxu1_port *mxport, struct tty_struct *tty)
{
	struct urb *urb;
	int status = 0;
	unsigned long flags;

	spin_lock_irqsave(&mxport->mxp_lock, flags);

	if (mxport->mxp_read_urb_state == MXU1_READ_URB_STOPPED) {
		mxport->mxp_read_urb_state = MXU1_READ_URB_RUNNING;
		urb = mxport->mxp_port->read_urb;
		spin_unlock_irqrestore(&mxport->mxp_lock, flags);
		urb->complete = mxu1_bulk_in_callback;
		urb->context = mxport;
		urb->dev = mxport->mxp_port->serial->dev;
		status = usb_submit_urb(urb, GFP_KERNEL);
	}
	else{
		mxport->mxp_read_urb_state = MXU1_READ_URB_RUNNING;
		spin_unlock_irqrestore(&mxport->mxp_lock, flags);
	}

	return status;
}


static int mxu1_command_out_sync(struct mxu1_device *mxdev, __u8 command,
	__u16 moduleid, __u16 value, __u8 *data, int size)
{
	int status = 0;

	status = usb_control_msg(mxdev->mxd_serial->dev,
		usb_sndctrlpipe(mxdev->mxd_serial->dev, 0), command,
		(USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT),
		value, moduleid, data, size, 1000);

	if (status == size)
		status = 0;

	if (status > 0)
		status = -ECOMM;

	return status;
}


static int mxu1_command_in_sync(struct mxu1_device *mxdev, __u8 command,
	__u16 moduleid, __u16 value, __u8 *data, int size)
{
	int status = 0;

	status = usb_control_msg(mxdev->mxd_serial->dev,
		usb_rcvctrlpipe(mxdev->mxd_serial->dev, 0), command,
		(USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_IN),
		value, moduleid, data, size, HZ);

	if (status == size)
		status = 0;

	if (status > 0)
		status = -ECOMM;

	return status;
}


static int mxu1_write_byte(struct mxu1_device *mxdev, unsigned long addr,
	__u8 mask, __u8 byte)
{
	int status = 0;
	unsigned int size;
	struct mxu1_write_data_bytes *data;
	struct device *dev = &mxdev->mxd_serial->dev->dev;

	dbg("%s - addr 0x%08lX, mask 0x%02X, byte 0x%02X", __FUNCTION__, addr, mask, byte);

	size = sizeof(struct mxu1_write_data_bytes) + 2;
	data = kmalloc(size, GFP_KERNEL);
	if (!data) {
		dev_err(dev, "%s - out of memory\n", __FUNCTION__);
		return -ENOMEM;
	}

	data->bAddrType = MXU1_RW_DATA_ADDR_XDATA;
	data->bDataType = MXU1_RW_DATA_BYTE;
	data->bDataCounter = 1;
	data->wBaseAddrHi = cpu_to_be16(addr>>16);
	data->wBaseAddrLo = cpu_to_be16(addr);
	data->bData[0] = mask;
	data->bData[1] = byte;

	status = mxu1_command_out_sync(mxdev, MXU1_WRITE_DATA, MXU1_RAM_PORT, 0,
		(__u8 *)data, size);

	if (status < 0)
		dev_err(dev, "%s - failed, %d\n", __FUNCTION__, status);

	kfree(data);

	return status;
}


static int mxu1_download_firmware(struct mxu1_device *mxdev,
	unsigned char *firmware, unsigned int firmware_size)
{
	int status = 0;
	int buffer_size;
	int pos;
	int len;
	int done;
	__u8 cs = 0;
	__u8 *buffer;
	struct usb_device *dev = mxdev->mxd_serial->dev;
	struct mxu1_firmware_header *header;
	unsigned int pipe = usb_sndbulkpipe(dev,
		mxdev->mxd_serial->port[0]->bulk_out_endpointAddress);

	
	buffer_size = MXU1_FIRMWARE_BUF_SIZE + sizeof(struct mxu1_firmware_header);
	buffer = kmalloc(buffer_size, GFP_KERNEL);
	if (!buffer) {
		dev_err(&dev->dev, "%s - out of memory\n", __FUNCTION__);
		return -ENOMEM;
	}

	memcpy(buffer, firmware, firmware_size);
	memset(buffer+firmware_size, 0xff, buffer_size-firmware_size);

	for(pos = sizeof(struct mxu1_firmware_header); pos < buffer_size; pos++)
		cs = (__u8)(cs + buffer[pos]);

	header = (struct mxu1_firmware_header *)buffer;
	header->wLength = cpu_to_le16((__u16)(buffer_size - sizeof(struct mxu1_firmware_header)));
	header->bCheckSum = cs;

	dbg("%s - downloading firmware", __FUNCTION__);
	for (pos = 0; pos < buffer_size; pos += done) {
		len = min(buffer_size - pos, MXU1_DOWNLOAD_MAX_PACKET_SIZE);

		status = usb_bulk_msg(dev, pipe, buffer+pos, len, &done, 1000);

		if (status)
			break;
	}

	kfree(buffer);

	if (status) {
		dev_err(&dev->dev, "%s - error downloading firmware, %d\n", __FUNCTION__, status);
		return status;
	}

	dbg("%s - download successful", __FUNCTION__);

	return 0;
}


/* Circular Buffer Functions */

/*
 * mxu1_buf_alloc
 *
 * Allocate a circular buffer and all associated memory.
 */

static struct circ_buf *mxu1_buf_alloc(void)
{
	struct circ_buf *cb;

	cb = (struct circ_buf *)kmalloc(sizeof(struct circ_buf), GFP_KERNEL);
	if (cb == NULL)
		return NULL;

	cb->buf = kmalloc(MXU1_WRITE_BUF_SIZE, GFP_KERNEL);
	if (cb->buf == NULL) {
		kfree(cb);
		return NULL;
	}

	mxu1_buf_clear(cb);

	return cb;
}


/*
 * mxu1_buf_free
 *
 * Free the buffer and all associated memory.
 */

static void mxu1_buf_free(struct circ_buf *cb)
{
	kfree(cb->buf);
	kfree(cb);
}


/*
 * mxu1_buf_clear
 *
 * Clear out all data in the circular buffer.
 */

static void mxu1_buf_clear(struct circ_buf *cb)
{
	cb->head = cb->tail = 0;
}


/*
 * mxu1_buf_data_avail
 *
 * Return the number of bytes of data available in the circular
 * buffer.
 */

static int mxu1_buf_data_avail(struct circ_buf *cb)
{
	return CIRC_CNT(cb->head,cb->tail,MXU1_WRITE_BUF_SIZE);
}


/*
 * mxu1_buf_space_avail
 *
 * Return the number of bytes of space available in the circular
 * buffer.
 */

static int mxu1_buf_space_avail(struct circ_buf *cb)
{
	return CIRC_SPACE(cb->head,cb->tail,MXU1_WRITE_BUF_SIZE);
}


/*
 * mxu1_buf_put
 *
 * Copy data data from a user buffer and put it into the circular buffer.
 * Restrict to the amount of space available.
 *
 * Return the number of bytes copied.
 */
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10))
static int mxu1_buf_put(struct mxu1_port *mxport, const char *buf, int count, int from_user)
#else
static int mxu1_buf_put(struct mxu1_port *mxport, const char *buf, int count)
#endif
{
	int c, ret = 0;
	struct circ_buf *cb;
	unsigned long flags;
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10))
	char *tmp_buf;
#endif

	cb = mxport->mxp_write_buf;

	while (1) {
		spin_lock_irqsave(&mxport->mxp_lock, flags);
		c = CIRC_SPACE_TO_END(cb->head, cb->tail, MXU1_WRITE_BUF_SIZE);
		spin_unlock_irqrestore(&mxport->mxp_lock, flags);

		if (count < c)
			c = count;
		if (c <= 0)
			break;
#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,10))

		tmp_buf = kmalloc(c, GFP_ATOMIC);

		if(from_user){
			if(copy_from_user(tmp_buf, buf, c))
				return -EFAULT;

			spin_lock_irqsave(&mxport->mxp_lock, flags);
			memcpy(cb->buf + cb->head, tmp_buf, c);
			spin_unlock_irqrestore(&mxport->mxp_lock, flags);
		}
		else{
			spin_lock_irqsave(&mxport->mxp_lock, flags);
			memcpy(cb->buf + cb->head, buf, c);
			spin_unlock_irqrestore(&mxport->mxp_lock, flags);
		}

		kfree(tmp_buf);
#else
		spin_lock_irqsave(&mxport->mxp_lock, flags);
		memcpy(cb->buf + cb->head, buf, c);
		spin_unlock_irqrestore(&mxport->mxp_lock, flags);
#endif
		spin_lock_irqsave(&mxport->mxp_lock, flags);
		cb->head = (cb->head + c) & (MXU1_WRITE_BUF_SIZE-1);
		spin_unlock_irqrestore(&mxport->mxp_lock, flags);
		buf += c;
		count -= c;
		ret += c;
	}
	
	return ret;
}


/*
 * mxu1_buf_get
 *
 * Get data from the circular buffer and copy to the given buffer.
 * Restrict to the amount of data available.
 *
 * Return the number of bytes copied.
 */

static int mxu1_buf_get(struct circ_buf *cb, char *buf, int count)
{
	int c, ret = 0;

	while (1) {
		c = CIRC_CNT_TO_END(cb->head, cb->tail, MXU1_WRITE_BUF_SIZE);
		if (count < c)
			c = count;
		if (c <= 0)
			break;
		memcpy(buf, cb->buf + cb->tail, c);
		cb->tail = (cb->tail + c) & (MXU1_WRITE_BUF_SIZE-1);
		buf += c;
		count -= c;
		ret += c;
	}

	return ret;
}

#if(LINUX_VERSION_CODE < KERNEL_VERSION(2,6,9))
static signed long mxu1_schedule_timeout_interruptible(signed long timeout)
{
	__set_current_state(TASK_INTERRUPTIBLE);
	return schedule_timeout(timeout);
}

static unsigned long mxu1_jsleep_interruptible(unsigned long timeout)
{
	while(timeout && !signal_pending(current))
		timeout = mxu1_schedule_timeout_interruptible(timeout);
	return timeout;
}
#endif
