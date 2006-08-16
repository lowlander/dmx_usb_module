/*
 * DMX USB driver
 *
 * Copyright (C) 2004,2006 Erwin Rol (erwin@erwinrol.com)
 *
 * This driver is based on the usb-skeleton driver;
 *
 * Copyright (C) 2001-2003 Greg Kroah-Hartman (greg@kroah.com)
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License as
 *	published by the Free Software Foundation, version 2.
 *
 * $Id: dmx_usb.c 41 2004-09-14 23:35:25Z erwin $ 
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/smp_lock.h>
#include <linux/completion.h>
#include <asm/uaccess.h>
#include <linux/usb.h>
#include <linux/version.h>

#include "dmx_usb.h"

#ifdef CONFIG_USB_DEBUG
	static int debug = 1;
#else
	static int debug;
#endif

/* Use our own dbg macro */
#undef dbg
#define dbg(format, arg...) do { if (debug) printk(KERN_DEBUG __FILE__ ": " format "\n" , ## arg); } while (0)


/* Version Information */
#define DRIVER_VERSION "v0.1.20060816"
#define DRIVER_AUTHOR "Erwin Rol, erwin@erwinrol.com"
#define DRIVER_DESC "DMX USB Driver"

/* Module parameters */
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16) )
MODULE_PARM(debug, "i");
MODULE_PARM_DESC(debug, "Debug enabled or not");
#else
module_param(debug, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(debug, "Debug enabled or not");
#endif

static struct usb_device_id dmx_usb_table [] = {
	{ USB_DEVICE_VER(FTDI_VID, FTDI_8U232AM_PID, 0x400, 0xffff) },
	{ USB_DEVICE_VER(FTDI_VID, FTDI_8U232AM_ALT_PID, 0x400, 0xffff) },
	{ }                                             /* Terminating entry */
};

MODULE_DEVICE_TABLE (usb, dmx_usb_table);

/* Get a minor range for your devices from the usb maintainer */
#define DMX_USB_MINOR_BASE	192

/* Structure to hold all of our device specific stuff */
struct dmx_usb_device {
	struct usb_device *	udev;			/* save off the usb device pointer */
	struct usb_interface *	interface;		/* the interface for this device */
	unsigned char		minor;			/* the starting minor number for this device */
	unsigned char		num_ports;		/* the number of ports this device has */
	char			num_interrupt_in;	/* number of interrupt in endpoints we have */
	char			num_bulk_in;		/* number of bulk in endpoints we have */
	char			num_bulk_out;		/* number of bulk out endpoints we have */

	unsigned char *		bulk_in_buffer;		/* the buffer to receive data */
	size_t			bulk_in_size;		/* the size of the receive buffer */
	__u8			bulk_in_endpointAddr;	/* the address of the bulk in endpoint */

	unsigned char *		bulk_out_buffer;	/* the buffer to send data */
	size_t			bulk_out_size;		/* the size of the send buffer */
	struct urb *		write_urb;		/* the urb used to send data */
	__u8			bulk_out_endpointAddr;	/* the address of the bulk out endpoint */
	atomic_t		write_busy;		/* true iff write urb is busy */
	struct completion	write_finished;		/* wait for the write to finish */

	int			open;			/* if the port is open or not */
	int			present;		/* if the device is not disconnected */
	struct semaphore	sem;			/* locks this structure */
};


/* prevent races between open() and disconnect() */
static DECLARE_MUTEX (disconnect_sem);

/* local function prototypes */
//static ssize_t dmx_usb_read	(struct file *file, char *buffer, size_t count, loff_t *ppos);
static ssize_t dmx_usb_write	(struct file *file, const char *buffer, size_t count, loff_t *ppos);
static int dmx_usb_ioctl	(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
static int dmx_usb_open		(struct inode *inode, struct file *file);
static int dmx_usb_release	(struct inode *inode, struct file *file);

static int dmx_usb_probe	(struct usb_interface *interface, const struct usb_device_id *id);
static void dmx_usb_disconnect	(struct usb_interface *interface);

static void dmx_usb_write_bulk_callback	(struct urb *urb, struct pt_regs *regs);

static struct file_operations dmx_usb_fops = {
	/*
	 * The owner field is part of the module-locking
	 * mechanism. The idea is that the kernel knows
	 * which module to increment the use-counter of
	 * BEFORE it calls the device's open() function.
	 * This also means that the kernel can decrement
	 * the use-counter again before calling release()
	 * or should the open() function fail.
	 */
	.owner =	THIS_MODULE,

	/* .read =		dmx_usb_read, */ 
	.write =	dmx_usb_write,
	.ioctl =	dmx_usb_ioctl,
	.open =		dmx_usb_open,
	.release =	dmx_usb_release,
};

/* 
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with devfs and the driver core
 */
static struct usb_class_driver dmx_usb_class = {
	.name =		"usb/dmx%d",
	.fops =		&dmx_usb_fops,
	.minor_base =	DMX_USB_MINOR_BASE,
};

/* usb specific object needed to register this driver with the usb subsystem */
static struct usb_driver dmx_usb_driver = {
#if ( LINUX_VERSION_CODE < KERNEL_VERSION(2,6,16) )
	.owner =	THIS_MODULE,
#endif
	.name =		"dmx_usb",
	.probe =	dmx_usb_probe,
	.disconnect =	dmx_usb_disconnect,
	.id_table =	dmx_usb_table,
};


/**
 */
static inline void dmx_usb_debug_data (const char *function, int size, const unsigned char *data)
{
	int i;

	if (!debug)
		return;

	printk (KERN_DEBUG __FILE__": %s - length = %d, data = ",
		function, size);
	for (i = 0; i < size; ++i) {
		printk ("%.2x ", data[i]);
	}
	printk ("\n");
}

static __u32 dmx_usb_baud_to_divisor(int baud)
{
	static const unsigned char divfrac[8] = { 0, 3, 2, 4, 1, 5, 6, 7 };
	__u32 divisor;
	int divisor3 = 48000000 / 2 / baud; // divisor shifted 3 bits to the left
	divisor = divisor3 >> 3;
	divisor |= (__u32)divfrac[divisor3 & 0x7] << 14;
	/* Deal with special cases for highest baud rates. */
	if (divisor == 1) divisor = 0; else     // 1.0
	if (divisor == 0x4001) divisor = 1;     // 1.5
	return divisor;
}

static int dmx_usb_set_speed(struct dmx_usb_device* dev)
{
	char *buf;
	__u16 urb_value;
	__u16 urb_index;
	__u32 urb_index_value;
	int rv;

	buf = kmalloc(1, GFP_NOIO);
	if (!buf)
		return -ENOMEM;

	urb_index_value = dmx_usb_baud_to_divisor(250000);
	urb_value = (__u16)urb_index_value;
	urb_index = (__u16)(urb_index_value >> 16);

	rv = usb_control_msg(dev->udev,
				usb_sndctrlpipe(dev->udev, 0),
				FTDI_SIO_SET_BAUDRATE_REQUEST,
				FTDI_SIO_SET_BAUDRATE_REQUEST_TYPE,
				urb_value, urb_index,
				buf, 0, HZ*10);

	kfree(buf);
	return rv;
}

static int dmx_usb_setup(struct dmx_usb_device* dev)
{
	__u16 urb_value;
	char buf[1];

	urb_value = FTDI_SIO_SET_DATA_STOP_BITS_2 | FTDI_SIO_SET_DATA_PARITY_NONE;
	urb_value |= 8; // number of data bits

	if (usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0),
				FTDI_SIO_SET_DATA_REQUEST,
				FTDI_SIO_SET_DATA_REQUEST_TYPE,
				urb_value , 0,
				buf, 0, HZ*10) < 0) {
		err("%s FAILED to set databits/stopbits/parity", __FUNCTION__);
	}

	if (usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0),
				FTDI_SIO_SET_FLOW_CTRL_REQUEST,
				FTDI_SIO_SET_FLOW_CTRL_REQUEST_TYPE,
				0, 0,
				buf, 0, HZ*10) < 0) {
		err("%s error from disable flowcontrol urb", __FUNCTION__);
	}

	dmx_usb_set_speed(dev);

	return 0;
}

static void dmx_usb_set_break(struct dmx_usb_device* dev, int break_state)
{
	__u16 urb_value = FTDI_SIO_SET_DATA_STOP_BITS_2 | FTDI_SIO_SET_DATA_PARITY_NONE | 8;

	char buf[2];

	if (break_state) {
		urb_value |= FTDI_SIO_SET_BREAK;
	}

	if (usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0),
				FTDI_SIO_SET_DATA_REQUEST,
				FTDI_SIO_SET_DATA_REQUEST_TYPE,
				urb_value , 0,
				buf, 2, HZ*10) < 0) {
		err("%s FAILED to enable/disable break state (state was %d)", __FUNCTION__,break_state);
	}


	dbg("%s break state is %d - urb is %d", __FUNCTION__,break_state, urb_value);
}

/**
 */
static inline void dmx_usb_delete (struct dmx_usb_device *dev)
{
	kfree (dev->bulk_in_buffer);
	usb_buffer_free (dev->udev, dev->bulk_out_size,
				dev->bulk_out_buffer,
				dev->write_urb->transfer_dma);
	usb_free_urb (dev->write_urb);
	kfree (dev);
}


/**
 */
static int dmx_usb_open (struct inode *inode, struct file *file)
{
	struct dmx_usb_device *dev = NULL;
	struct usb_interface *interface;
	int subminor;
	int retval = 0;

	dbg("%s", __FUNCTION__);

	subminor = iminor(inode);

	/* prevent disconnects */
	down (&disconnect_sem);

	interface = usb_find_interface (&dmx_usb_driver, subminor);
	if (!interface) {
		err ("%s - error, can't find device for minor %d",
		     __FUNCTION__, subminor);
		retval = -ENODEV;
		goto exit_no_device;
	}

	dev = usb_get_intfdata(interface);
	if (!dev) {
		retval = -ENODEV;
		goto exit_no_device;
	}

	/* lock this device */
	down (&dev->sem);

	/* increment our usage count for the driver */
	++dev->open;

	/* save our object in the file's private structure */
	file->private_data = dev;

	/* unlock this device */
	up (&dev->sem);

exit_no_device:
	up (&disconnect_sem);
	return retval;
}


/**
 */
static int dmx_usb_release (struct inode *inode, struct file *file)
{
	struct dmx_usb_device *dev;
	int retval = 0;

	dev = (struct dmx_usb_device *)file->private_data;
	if (dev == NULL) {
		dbg ("%s - object is NULL", __FUNCTION__);
		return -ENODEV;
	}

	dbg("%s - minor %d", __FUNCTION__, dev->minor);

	/* lock our device */
	down (&dev->sem);

	if (dev->open <= 0) {
		dbg ("%s - device not opened", __FUNCTION__);
		retval = -ENODEV;
		goto exit_not_opened;
	}

	/* wait for any bulk writes that might be going on to finish up */
	if (atomic_read (&dev->write_busy))
		wait_for_completion (&dev->write_finished);

	--dev->open;

	if (!dev->present && !dev->open) {
		/* the device was unplugged before the file was released */
		up (&dev->sem);
		dmx_usb_delete (dev);
		return 0;
	}

exit_not_opened:
	up (&dev->sem);

	return retval;
}

#if 0 

Read is not yet supported

/**
 */
static ssize_t dmx_usb_read (struct file *file, char *buffer, size_t count, loff_t *ppos)
{
	struct dmx_usb_device *dev;
	int retval = 0;
	int bytes_read;

	dev = (struct dmx_usb_device *)file->private_data;

	dbg("%s - minor %d, count = %Zd", __FUNCTION__, dev->minor, count);

	/* lock this object */
	down (&dev->sem);

	/* verify that the device wasn't unplugged */
	if (!dev->present) {
		up (&dev->sem);
		return -ENODEV;
	}

	/* do a blocking bulk read to get data from the device */
	retval = usb_bulk_msg (dev->udev,
			       usb_rcvbulkpipe (dev->udev,
						dev->bulk_in_endpointAddr),
			       dev->bulk_in_buffer,
			       min (dev->bulk_in_size, count),
			       &bytes_read, HZ*10);

	/* if the read was successful, copy the data to userspace */
	if (!retval) {
		if (copy_to_user (buffer, dev->bulk_in_buffer+2, bytes_read-2))
			retval = -EFAULT;
		else
			retval = bytes_read;
	}

	/* unlock the device */
	up (&dev->sem);
	return retval;
}

#endif

static __u16 dmx_usb_get_status(struct dmx_usb_device* dev)
{
	int retval = 0;
	int count = 0;
	__u16 buf;

	retval = usb_bulk_msg (dev->udev,
				usb_rcvbulkpipe (dev->udev, dev->bulk_in_endpointAddr),
				&buf, 2, &count, HZ*10);

	if (retval)
		return 0;
	
	return buf;
}



/**
 *	dmx_usb_write
 *
 *	A device driver has to decide how to report I/O errors back to the
 *	user.  The safest course is to wait for the transfer to finish before
 *	returning so that any errors will be reported reliably.  dmx_usb_read()
 *	works like this.  But waiting for I/O is slow, so many drivers only
 *	check for errors during I/O initiation and do not report problems
 *	that occur during the actual transfer.  That's what we will do here.
 *
 *	A driver concerned with maximum I/O throughput would use double-
 *	buffering:  Two urbs would be devoted to write transfers, so that
 *	one urb could always be active while the other was waiting for the
 *	user to send more data.
 */
static ssize_t dmx_usb_write (struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
	struct dmx_usb_device *dev;
	ssize_t bytes_written = 0;
	int retval = 0;
	__u16 stat;

	dev = (struct dmx_usb_device *)file->private_data;

	dbg("%s - minor %d, count = %Zd", __FUNCTION__, dev->minor, count);

	/* lock this object */
	down (&dev->sem);

	/* verify that the device wasn't unplugged */
	if (!dev->present) {
		retval = -ENODEV;
		goto exit;
	}

	/* verify that we actually have some data to write */
	if (count == 0) {
		dbg("%s - write request of 0 bytes", __FUNCTION__);
		goto exit;
	}

	/* wait for a previous write to finish up; we don't use a timeout
	 * and so a nonresponsive device can delay us indefinitely.
	 */
	if (atomic_read (&dev->write_busy))
		wait_for_completion (&dev->write_finished);

	/* we can only write as much as our buffer will hold */
	bytes_written = min (dev->bulk_out_size, count);

	/* copy the data from userspace into our transfer buffer;
	 * this is the only copy required.
	 */
	if (copy_from_user(dev->write_urb->transfer_buffer, buffer,
			   bytes_written)) {
		retval = -EFAULT;
		goto exit;
	}

	dmx_usb_debug_data (__FUNCTION__, bytes_written,
			     dev->write_urb->transfer_buffer);

	/* this urb was already set up, except for this write size */
	dev->write_urb->transfer_buffer_length = bytes_written;

	/* Poll the device to see if the transmit buffer is empty */
	do {
		stat = dmx_usb_get_status(dev);
		if (stat == 0) {
			retval = -EFAULT;
			goto exit;
		}
	} while ( (stat & ((FTDI_RS_TEMT) << 8) ) == 0 ) ;

	/* the transmit buffer is empty, now toggle the break */
	dmx_usb_set_break(dev, 1);
	dmx_usb_set_break(dev, 0);

	/* send the data out the bulk port */
	/* a character device write uses GFP_KERNEL,
	 unless a spinlock is held */
	init_completion (&dev->write_finished);
	atomic_set (&dev->write_busy, 1);
	retval = usb_submit_urb(dev->write_urb, GFP_KERNEL);
	if (retval) {
		atomic_set (&dev->write_busy, 0);
		err("%s - failed submitting write urb, error %d",
		    __FUNCTION__, retval);
	} else {
		retval = bytes_written;
	}

exit:
	/* unlock the device */
	up (&dev->sem);

	return retval;
}


/**
 */
static int dmx_usb_ioctl (struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct dmx_usb_device *dev;

	dev = (struct dmx_usb_device *)file->private_data;

	/* lock this object */
	down (&dev->sem);

	/* verify that the device wasn't unplugged */
	if (!dev->present) {
		up (&dev->sem);
		return -ENODEV;
	}

	dbg("%s - minor %d, cmd 0x%.4x, arg %ld", __FUNCTION__,
	    dev->minor, cmd, arg);

	/* fill in your device specific stuff here */

	/* unlock the device */
	up (&dev->sem);

	/* return that we did not understand this ioctl call */
	return -ENOTTY;
}


/**
 */
static void dmx_usb_write_bulk_callback (struct urb *urb, struct pt_regs *regs)
{
	struct dmx_usb_device *dev = (struct dmx_usb_device *)urb->context;

	dbg("%s - minor %d", __FUNCTION__, dev->minor);

	/* sync/async unlink faults aren't errors */
	if (urb->status && !(urb->status == -ENOENT ||
				urb->status == -ECONNRESET)) {
		dbg("%s - nonzero write bulk status received: %d",
		    __FUNCTION__, urb->status);
	}

	/* notify anyone waiting that the write has finished */
	atomic_set (&dev->write_busy, 0);
	complete (&dev->write_finished);
}

/**
 *
 *	Called by the usb core when a new device is connected that it thinks
 *	this driver might be interested in.
 */
static int dmx_usb_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(interface);
	struct dmx_usb_device *dev = NULL;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
	size_t buffer_size;
	int i;
	int retval = -ENOMEM;

	/* See if the device offered us matches what we can accept */
	if ((udev->descriptor.idVendor != FTDI_VID) ||
	    (udev->descriptor.idProduct != FTDI_8U232AM_PID)) {
		return -ENODEV;
	}

	/* allocate memory for our device state and initialize it */
	dev = kmalloc (sizeof(struct dmx_usb_device), GFP_KERNEL);
	if (dev == NULL) {
		err ("Out of memory");
		return -ENOMEM;
	}
	memset (dev, 0x00, sizeof (*dev));

	init_MUTEX (&dev->sem);
	dev->udev = udev;
	dev->interface = interface;

	/* set up the endpoint information */
	/* check out the endpoints */
	/* use only the first bulk-in and bulk-out endpoints */
	iface_desc = &interface->altsetting[0];
	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;

		if (!dev->bulk_in_endpointAddr &&
		    (endpoint->bEndpointAddress & USB_DIR_IN) &&
		    ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
					== USB_ENDPOINT_XFER_BULK)) {
			/* we found a bulk in endpoint */
			buffer_size = endpoint->wMaxPacketSize;
			dev->bulk_in_size = buffer_size;
			dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
			dev->bulk_in_buffer = kmalloc (buffer_size, GFP_KERNEL);
			if (!dev->bulk_in_buffer) {
				err("Couldn't allocate bulk_in_buffer");
				goto error;
			}
		}

		if (!dev->bulk_out_endpointAddr &&
		    !(endpoint->bEndpointAddress & USB_DIR_IN) &&
		    ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK)
					== USB_ENDPOINT_XFER_BULK)) {
			/* we found a bulk out endpoint */
			/* a probe() may sleep and has no restrictions on memory allocations */
			dev->write_urb = usb_alloc_urb(0, GFP_KERNEL);
			if (!dev->write_urb) {
				err("No free urbs available");
				goto error;
			}
			dev->bulk_out_endpointAddr = endpoint->bEndpointAddress;

			/* on some platforms using this kind of buffer alloc
			 * call eliminates a dma "bounce buffer".
			 *
			 * NOTE: you'd normally want i/o buffers that hold
			 * more than one packet, so that i/o delays between
			 * packets don't hurt throughput.
			 */
			buffer_size = endpoint->wMaxPacketSize;
			dev->bulk_out_size = 513;
			dev->write_urb->transfer_flags = URB_NO_TRANSFER_DMA_MAP;
			dev->bulk_out_buffer = usb_buffer_alloc (udev,
					buffer_size, GFP_KERNEL,
					&dev->write_urb->transfer_dma);
			if (!dev->bulk_out_buffer) {
				err("Couldn't allocate bulk_out_buffer");
				goto error;
			}
			usb_fill_bulk_urb(dev->write_urb, udev,
				      usb_sndbulkpipe(udev,
						      endpoint->bEndpointAddress),
				      dev->bulk_out_buffer, buffer_size,
				      dmx_usb_write_bulk_callback, dev);
		}
	}
	if (!(dev->bulk_in_endpointAddr && dev->bulk_out_endpointAddr)) {
		err("Couldn't find both bulk-in and bulk-out endpoints");
		goto error;
	}

	dmx_usb_setup(dev);

	/* allow device read, write and ioctl */
	dev->present = 1;

	/* we can register the device now, as it is ready */
	usb_set_intfdata (interface, dev);
	retval = usb_register_dev (interface, &dmx_usb_class);
	if (retval) {
		/* something prevented us from registering this driver */
		err ("Not able to get a minor for this device.");
		usb_set_intfdata (interface, NULL);
		goto error;
	}

	dev->minor = interface->minor;

	/* let the user know what node this device is now attached to */
	info ("DMX USB device now attached to dmx%d", dev->minor);
	return 0;

error:
	dmx_usb_delete (dev);
	return retval;
}


/**
 *
 *	Called by the usb core when the device is removed from the system.
 *
 *	This routine guarantees that the driver will not submit any more urbs
 *	by clearing dev->udev.  It is also supposed to terminate any currently
 *	active urbs.  Unfortunately, usb_bulk_msg(), used in dmx_usb_read(), does
 *	not provide any way to do this.  But at least we can cancel an active
 *	write.
 */
static void dmx_usb_disconnect(struct usb_interface *interface)
{
	struct dmx_usb_device *dev;
	int minor;

	/* prevent races with open() */
	down (&disconnect_sem);

	dev = usb_get_intfdata (interface);
	usb_set_intfdata (interface, NULL);

	down (&dev->sem);

	minor = dev->minor;

	/* give back our minor */
	usb_deregister_dev (interface, &dmx_usb_class);

	/* terminate an ongoing write */
	if (atomic_read (&dev->write_busy)) {
		usb_unlink_urb (dev->write_urb);
		wait_for_completion (&dev->write_finished);
	}

	/* prevent device read, write and ioctl */
	dev->present = 0;

	up (&dev->sem);

	/* if the device is opened, dmx_usb_release will clean this up */
	if (!dev->open)
		dmx_usb_delete (dev);

	up (&disconnect_sem);

	info("DMX USB #%d now disconnected", minor);
}



/**
 *	dmx_usb_init
 */
static int __init dmx_usb_init(void)
{
	int result;

	/* register this driver with the USB subsystem */
	result = usb_register(&dmx_usb_driver);
	if (result) {
		err("usb_register failed. Error number %d",
		    result);
		return result;
	}

	info(DRIVER_DESC " " DRIVER_VERSION);
	return 0;
}


/**
 *	dmx_usb_exit
 */
static void __exit dmx_usb_exit(void)
{
	/* deregister this driver with the USB subsystem */
	usb_deregister(&dmx_usb_driver);
}


module_init (dmx_usb_init);
module_exit (dmx_usb_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

