/*
 * Driver for the CH341 USB-I2C adapter
 *
 * Copyright (c) 2016 Martin K. Schröder <mkschreder.uk@gmail.com>
 *  - use proper usb signaling for performance
 *  - respect the maximum 32 byte packet length the chip supports
 *  - code cleanup
 * 
 * Copyright (c) 2014 Marco Gittler
 *
 * Derived from:
 *  i2c-tiny-usb.c
 *  Copyright (C) 2006-2007 Till Harbaum (Till@Harbaum.org)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, version 2.
 */

/* 
Notes about this driver

This chips supports usb transfers of up to 32 bytes. This includes control
bytes as well. This means we need to split all communication with the chip into
32 byte packets. The throughput is limited to how many such packets we can send
(my measurements indicate that there is 2ms delay between each usb packet). 

Luckily, the actual i2c packet size is not limited because we are indeed
allowed to send continuation packets and split larger messages into smaller
blocks while not having to deal with the messages being split on the other end.

So far tested with: 
	- ssd1306 display
	- 24c32 eeprom

// Martin
*/ 

#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/i2c.h>

#define DRIVER_NAME     "ch341-i2c"

#define USB_VENDOR_ID_CH341     0x1a86
#define USB_DEVICE_ID_CH341_U2C 0x5512

#define DEFAULT_CONFIGURATION       0x01

// this timeout is used to wait for the chip to be ready 
#define DEFAULT_TIMEOUT             50

#define                CH341_PACKET_LENGTH        32
#define                CH341_PKT_LEN_SHORT        8

#define                CH341_ENDP_INTER_UP        0x81
#define                CH341_ENDP_INTER_DOWN        0x01
#define                CH341_ENDP_DATA_UP                0x82
#define                CH341_ENDP_DATA_DOWN        0x02

#define                CH341_VENDOR_READ                0xC0
#define                CH341_VENDOR_WRITE                0x40

#define                CH341_PARA_INIT                0xB1
#define                CH341_I2C_STATUS                0x52
#define                CH341_I2C_COMMAND                0x53

#define                CH341_PARA_CMD_R0                0xAC
#define                CH341_PARA_CMD_R1                0xAD
#define                CH341_PARA_CMD_W0                0xA6
#define                CH341_PARA_CMD_W1                0xA7
#define                CH341_PARA_CMD_STS                0xA0

#define CH341A_CMD_READ_STATUS 0xF0

#define                CH341A_CMD_SET_OUTPUT        0xA1
#define                CH341A_CMD_IO_ADDR                0xA2
#define                CH341A_CMD_PRINT_OUT        0xA3
#define                CH341A_CMD_SPI_STREAM        0xA8
#define                CH341A_CMD_SIO_STREAM        0xA9
#define                CH341A_CMD_I2C_STREAM        0xAA
#define                CH341A_CMD_UIO_STREAM        0xAB

#define                CH341A_BUF_CLEAR                0xB2
#define                CH341A_I2C_CMD_X                0x54
#define                CH341A_DELAY_MS                0x5E
#define                CH341A_GET_VER                        0x5F

#define                CH341_EPP_IO_MAX                ( CH341_PACKET_LENGTH - 1 )
#define                CH341A_EPP_IO_MAX                0xFF

#define                CH341A_CMD_IO_ADDR_W        0x00
#define                CH341A_CMD_IO_ADDR_R        0x80

#define                CH341A_CMD_I2C_STM_STA        0x74
#define                CH341A_CMD_I2C_STM_STO        0x75
#define                CH341A_CMD_I2C_STM_OUT        0x80
#define                CH341A_CMD_I2C_STM_IN        0xC0
#define                CH341A_CMD_I2C_STM_MAX        ( min( 0x3F, CH341_PACKET_LENGTH ) )
#define                CH341A_CMD_I2C_STM_SET        0x60
#define                CH341A_CMD_I2C_STM_US        0x40
#define                CH341A_CMD_I2C_STM_MS        0x50
#define                CH341A_CMD_I2C_STM_DLY        0x0F
#define                CH341A_CMD_I2C_STM_END        0x00

#define                CH341A_CMD_UIO_STM_IN        0x00
#define                CH341A_CMD_UIO_STM_DIR        0x40
#define                CH341A_CMD_UIO_STM_OUT        0x80
#define                CH341A_CMD_UIO_STM_US        0xC0
#define                CH341A_CMD_UIO_STM_END        0x20

#define                CH341_PARA_MODE_EPP        0x00
#define                CH341_PARA_MODE_EPP17        0x00
#define                CH341_PARA_MODE_EPP19        0x01
#define                CH341_PARA_MODE_MEM        0x02


#define CH341_I2C_LOW_SPEED 0               // low speed - 20kHz               
#define CH341_I2C_STANDARD_SPEED 1          // standard speed - 100kHz
#define CH341_I2C_FAST_SPEED 2              // fast speed - 400kHz
#define CH341_I2C_HIGH_SPEED 3              // high speed - 750kHz

#define U2C_I2C_FREQ_FAST 400000
#define U2C_I2C_FREQ_STD  100000
#define U2C_I2C_FREQ(s)   (1000000 / (2 * (s - 1) + 10))

#define RESP_OK         0x00
#define RESP_FAILED     0x01
#define RESP_BAD_MEMADDR    0x04
#define RESP_DATA_ERR       0x05
#define RESP_NOT_IMPLEMENTED    0x06
#define RESP_NACK       0x07
#define RESP_TIMEOUT        0x09

#define CH341A_DATA_CHUNK_SIZE 25 // chips supports max 32 bytes but we have to account for command bytes as well 
#define CH341A_BUF_LEN    64

#undef TRACE
#define TRACE(...) {}
//#define TRACE(...) printk(__VA_ARGS__)

struct ch341_data {
	u8 buffer[CH341A_BUF_LEN]; 	/* temporary buffer for transaction data */ 
    int ep_in, ep_out;              /* Endpoints    */
    struct usb_device *usb_dev; /* the usb device for this device */
    struct usb_interface *interface;/* the interface for this device */
    struct i2c_adapter adapter; /* i2c related things */
	
	// must be locked whenever we are sending data and unlocked afterwards
	struct mutex i2c_lock; 
};

static uint frequency = U2C_I2C_FREQ_STD;   /* I2C clock frequency in Hz */

module_param(frequency, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(frequency, "I2C clock frequency in hertz");

static int ch341_usb_cmd_poll_device(struct ch341_data *dev, u8 addr) {
	return 1; 
	// TOO SLOW
	for(int c = 0; c < 10; c++){
		u8 *msg = dev->buffer; 
		int msgsize = 0, ret, actual; 

		msg[msgsize++] = CH341A_CMD_I2C_STREAM;
		msg[msgsize++] = CH341A_CMD_I2C_STM_STA;
		msg[msgsize++] = CH341A_CMD_I2C_STM_OUT | 0;  // NOTE: must be zero length otherwise it messes up the device
		msg[msgsize++] = addr | 0x01; // use a read 
		msg[msgsize++] = CH341A_CMD_I2C_STM_IN;  // zero length here as well
		msg[msgsize++] = CH341A_CMD_I2C_STM_STO;
		msg[msgsize++] = CH341A_CMD_I2C_STM_END;
		
		ret = usb_bulk_msg(dev->usb_dev,
				   usb_sndbulkpipe(dev->usb_dev, dev->ep_out),
				   msg, msgsize, &actual,
				   DEFAULT_TIMEOUT);

		memset(msg, 0, sizeof(dev->buffer)); 
		ret = usb_bulk_msg(dev->usb_dev,
					   usb_rcvbulkpipe(dev->usb_dev, dev->ep_in),
					   dev->buffer, CH341A_BUF_LEN, &actual,
					   DEFAULT_TIMEOUT);
		
		TRACE("STATUS: fail:%d ret:%d, len:%d, buf:%02x:%02x\n", !!(msg[0] & 0x80), ret, actual, msg[0], msg[1]); 
		if(!(msg[0] & 0x80)) return 1;  
	}
	return 0; 
}

/* 
* Issues a writeread through the i2c. (note: this function is currently broken)
* locks i2c_lock 
*/
static int ch341_usb_cmd_read_i2c(struct ch341_data *dev, u8 addr, u8 *data, u8 datalen){
	u8 *msg0 = dev->buffer; 
    int ret, actual = 0;
	int transfered = 0; 

	mutex_lock(&dev->i2c_lock); 

	if(ch341_usb_cmd_poll_device(dev, addr) == 0){
		mutex_unlock(&dev->i2c_lock); 
		return -EIO; 
	}

	TRACE("ch341: issuing i2c read of %d bytes to %02x..\n", datalen, addr >> 1); 

	while(transfered < datalen) {
		int msgsize = 0; 

		/*if(datalen > CH341A_DATA_CHUNK_SIZE){
			dev_err(&dev->interface->dev, "ch341: data length > %d not supported!\n", CH341A_DATA_CHUNK_SIZE); 
			datalen = CH341A_DATA_CHUNK_SIZE; 
		}*/
		
		int chunk_size = CH341A_DATA_CHUNK_SIZE; 
		if(transfered != 0) chunk_size = 32 - 3; 
		if(transfered + chunk_size >= datalen) chunk_size = datalen - transfered; 
		
		if(transfered == 0){
			msg0[msgsize++] = CH341A_CMD_I2C_STREAM;
			msg0[msgsize++] = CH341A_CMD_I2C_STM_STA;
			msg0[msgsize++] = CH341A_CMD_I2C_STM_OUT | 1; // 1 address byte
			msg0[msgsize++] = addr | 0x01; // i2c read

			msg0[msgsize++] = CH341A_CMD_I2C_STM_IN | (chunk_size & 0x1f);

			if(transfered + chunk_size >= datalen) msg0[msgsize++] = CH341A_CMD_I2C_STM_STO;
			msg0[msgsize++] = CH341A_CMD_I2C_STM_END;
		} else {
			msg0[msgsize++] = CH341A_CMD_I2C_STREAM;
			msg0[msgsize++] = CH341A_CMD_I2C_STM_IN | (chunk_size & 0x1f);
			if(transfered + chunk_size >= datalen) msg0[msgsize++] = CH341A_CMD_I2C_STM_STO;
			msg0[msgsize++] = CH341A_CMD_I2C_STM_END; 
		}
			
		ret = usb_bulk_msg(dev->usb_dev,
				   usb_sndbulkpipe(dev->usb_dev, dev->ep_out),
				   dev->buffer, msgsize, &actual,
				   DEFAULT_TIMEOUT);
	
		if(ret < 0 || actual < msgsize) goto error; 

		//TRACE("ch341: sent read command: count:%d addr:%02x\n", actual, addr >> 1); 

		memset(dev->buffer, 0, sizeof(dev->buffer)); 

		ret = usb_bulk_msg(dev->usb_dev,
				   usb_rcvbulkpipe(dev->usb_dev, dev->ep_in),
				   dev->buffer, CH341A_BUF_LEN, &actual,
				   DEFAULT_TIMEOUT);
		
		TRACE("ch341: read result: ret:%d, len:%d\n", ret, actual); 

		if(ret < 0 || actual == 0) goto error; 
		
		//actual--; // first byte is status byte!

		TRACE("ch341: read success: count:%d addr:%02x status:%02x data: ", actual, addr >> 1, dev->buffer[0]); 
		for(int c = 0; c < actual; c++) TRACE("%02x ", dev->buffer[c]); 
		TRACE("\n"); 
		
		memcpy(data + transfered, dev->buffer, actual); 

		transfered += actual; 
		//break; 
	}

	mutex_unlock(&dev->i2c_lock); 
	return datalen; 
error: 
	if(ret < 0) dev_err(&dev->interface->dev, "ch341: could not read device: %d\n", ret); 
	mutex_unlock(&dev->i2c_lock); 
	return -EIO; 
}

static int ch341_usb_cmd_write_i2c(struct ch341_data *dev, u8 addr, u8 *data, u8 datalen){
	int ret;
	int transfered = 0; 
    u8 *msg0 = dev->buffer; 

	mutex_lock(&dev->i2c_lock); 

	if(ch341_usb_cmd_poll_device(dev, addr) == 0){
		mutex_unlock(&dev->i2c_lock); 
		return -EIO; 
	}

	while(transfered < datalen){
		int msgsize = 0, actual = 0; 
		int chunk_size = CH341A_DATA_CHUNK_SIZE; 
		if(transfered + chunk_size > datalen) chunk_size = datalen - transfered; 

		//TRACE("ch341: issuing i2c write of %d bytes to %02x.. %02x %02x %02x\n", datalen, addr >> 1, data[0], data[1], data[2]); 

		TRACE("ch341: i2c write chunk size:%d, total:%d\n", chunk_size, datalen); 
		
		// TODO: can this somehow handle multipart packets??
		if(transfered == 0){
			msg0[msgsize++] = CH341A_CMD_I2C_STREAM; 
			msg0[msgsize++] = CH341A_CMD_I2C_STM_STA; 

			msg0[msgsize++] = CH341A_CMD_I2C_STM_OUT | ((chunk_size + 1) & 0x1f); //(((datalen > 0)?(datalen + 1):0) & 0x1f); // 5 bit length of output data + 1 addr byte
			msg0[msgsize++] = addr & 0xfe,
			memcpy(msg0 + msgsize, data, chunk_size);
			msgsize += chunk_size;
			
			if(transfered + chunk_size >= datalen) msg0[msgsize++] = CH341A_CMD_I2C_STM_STO;
			msg0[msgsize++] = CH341A_CMD_I2C_STM_END;
		} else {
			msg0[msgsize++] = CH341A_CMD_I2C_STREAM; 
			msg0[msgsize++] = CH341A_CMD_I2C_STM_OUT | ((chunk_size) & 0x1f); //(((datalen > 0)?(datalen + 1):0) & 0x1f); // 5 bit length of output data + 1 addr byte
			memcpy(msg0 + msgsize, data + transfered, chunk_size);
			msgsize += chunk_size;
			
			if(transfered + chunk_size >= datalen) msg0[msgsize++] = CH341A_CMD_I2C_STM_STO;
			msg0[msgsize++] = CH341A_CMD_I2C_STM_END;
		}

		ret = usb_bulk_msg(dev->usb_dev,
				   usb_sndbulkpipe(dev->usb_dev, dev->ep_out),
				   msg0, msgsize, &actual,
				   DEFAULT_TIMEOUT);
		
		TRACE("ch341: read result: ret:%d, len:%d\n", ret, actual); 
		if(ret < 0 || actual != msgsize) goto error; 

		transfered += chunk_size; 
	}
/*
	{
		int read = 0; 
		// extra read
		int r = usb_bulk_msg(dev->usb_dev,
				   usb_rcvbulkpipe(dev->usb_dev, dev->ep_in),
				   msg0, CH341A_BUF_LEN, &read,
				   100);

		if(r >= 0 && read) TRACE("ch341: extra read got %d\n", read); 
	}
*/

	mutex_unlock(&dev->i2c_lock); 
	
	TRACE("ch341: successfully wrote %d bytes\n", datalen); 
	return datalen; 
error: 
	mutex_unlock(&dev->i2c_lock); 
	return -EIO; 
}

static int ch341_set_speed(struct ch341_data *dev, u8 speed){
	int actual = 0, ret; 

    u8 msg[] = {
        CH341A_CMD_I2C_STREAM,
        CH341A_CMD_I2C_STM_SET | (speed & 0x03),
        CH341A_CMD_I2C_STM_END
    };

	mutex_lock(&dev->i2c_lock); 

	memcpy(dev->buffer, msg, 3); 

	ret = usb_bulk_msg(dev->usb_dev,
			   usb_sndbulkpipe(dev->usb_dev, dev->ep_out),
			   dev->buffer, 3, &actual,
			   DEFAULT_TIMEOUT);
	
	TRACE("ch341: set_speed: %d - %d\n", ret, actual); 

	mutex_unlock(&dev->i2c_lock); 

    if(ret < 0 || actual != 3) ret = -EIO; 

	return 0; 
}


static int ch341_init(struct ch341_data *dev)
{
    int speed, freq;
    if (frequency >= 750000) {
        speed = CH341_I2C_HIGH_SPEED;
        freq = frequency;
    } else if (frequency >= 400000) {
        speed = CH341_I2C_FAST_SPEED;
        freq = frequency;
    } else if (frequency >= 200000 || frequency == 0) {
        speed = CH341_I2C_STANDARD_SPEED;
        freq = frequency;
    } else {
        speed = CH341_I2C_LOW_SPEED;
        freq = frequency;
    }

    /* Set I2C speed */
    if(ch341_set_speed(dev, speed) < 0) {
		dev_err(&dev->interface->dev, "ch341: failed to set i2c speed!\n"); 
		return -EIO; 
	}

    dev_info(&dev->interface->dev,
             "CH341 U2C at USB bus %03d address %03d speed %d Hz\n",
             dev->usb_dev->bus->busnum, dev->usb_dev->devnum, freq);

    return 0;
}

/* i2c layer */

static int ch341_usb_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs,
                          int num){
    struct ch341_data *dev = i2c_get_adapdata(adapter);
    struct i2c_msg *pmsg;
    int i, ret = 0;

    for (i = 0; i < num; i++) {
        pmsg = &msgs[i];

        if (pmsg->flags & I2C_M_RD) {
			// current do not support reading!
			//dev_err(&dev->interface->dev, "ch341: I2C READ CURRENTLY NOT SUPPORTED!\n"); 
			// I did not test the code below but it has so far not been used
			if(pmsg->flags & I2C_M_RECV_LEN) printk("RECV LEN\n"); 
			ret = ch341_usb_cmd_read_i2c(dev, pmsg->addr << 1, pmsg->buf, pmsg->len); 
			if(ret < 0) return ret; 
			if(ret > 0) pmsg->len = ret; 
        } else {
            ret = ch341_usb_cmd_write_i2c(dev, pmsg->addr << 1, pmsg->buf, pmsg->len);
            if(ret < 0) return ret; 
        }
    }

    return num;
}

/*
 * Return list of supported functionality.
 */
static u32 ch341_usb_func(struct i2c_adapter *a)
{
    return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL ;/*|
           I2C_FUNC_SMBUS_READ_BLOCK_DATA | I2C_FUNC_SMBUS_BLOCK_PROC_CALL;*/
}

static const struct i2c_algorithm ch341_usb_algorithm = {
    .master_xfer = ch341_usb_xfer,
    .functionality = ch341_usb_func,
};

/* only supported on newer kernels */
/*
static const struct i2c_adapter_quirks ch341_quirks = {
	.max_write_len = 24
}; 
*/

/* device layer */

static const struct usb_device_id ch341_i2c_table[] = {
    { USB_DEVICE(USB_VENDOR_ID_CH341, USB_DEVICE_ID_CH341_U2C) },
    { }
};

MODULE_DEVICE_TABLE(usb, ch341_i2c_table);

static int ch341_i2c_probe(struct usb_interface *interface,
                           const struct usb_device_id *id){
    struct usb_device *udev;
    struct usb_host_interface *hostif = interface->cur_altsetting;
    struct ch341_data *dev;
    const int ifnum = interface->altsetting[DEFAULT_CONFIGURATION].desc.bInterfaceNumber;
    char *speed;
    int ret;

    if (hostif->desc.bInterfaceNumber != 0
            || hostif->desc.bNumEndpoints < 2) {
        return -ENODEV;
    }

    /* allocate memory for our device state and initialize it */
    dev = kzalloc(sizeof(*dev), GFP_KERNEL);
    if (dev == NULL) {
        dev_err(&interface->dev, "no memory for device state\n");
        ret = -ENOMEM;
        goto error;
    }
	mutex_init(&dev->i2c_lock); 

    dev->ep_out = hostif->endpoint[1].desc.bEndpointAddress;
    dev_info(&interface->dev, "ep_out=%x\n", dev->ep_out);
    dev->ep_in = hostif->endpoint[0].desc.bEndpointAddress;
    dev_info(&interface->dev, "ep_in=%x\n", dev->ep_in);

    dev->usb_dev = usb_get_dev(interface_to_usbdev(interface));
    dev->interface = interface;

    udev = dev->usb_dev;
    switch (udev->speed) {
    case USB_SPEED_LOW:
        speed = "1.5";
        break;
    case USB_SPEED_UNKNOWN:
    case USB_SPEED_FULL:
        speed = "12";
        break;
    case USB_SPEED_HIGH:
        speed = "480";
        break;
    default:
        speed = "unknown";
    }
    printk(KERN_INFO DRIVER_NAME
           ": New device %s %s @ %s Mbps "
           "(%04x:%04x, interface %d, class %d, version %d.%02d)\n",
           udev->manufacturer ? udev->manufacturer : "",
           udev->product ? udev->product : "",
           speed,
           le16_to_cpu(udev->descriptor.idVendor),
           le16_to_cpu(udev->descriptor.idProduct),
           ifnum,
           interface->altsetting->desc.bInterfaceNumber,
           le16_to_cpu(udev->descriptor.bcdDevice % 0xff),
           le16_to_cpu(udev->descriptor.bcdDevice >> 8 % 0xff)
          );

    /* save our data pointer in this interface device */
    usb_set_intfdata(interface, dev);

    /* setup i2c adapter description */
    dev->adapter.owner = THIS_MODULE;
    dev->adapter.class = I2C_CLASS_HWMON;
    dev->adapter.algo = &ch341_usb_algorithm;
	//dev->adapter.quirks = &ch341_quirks; 
    i2c_set_adapdata(&dev->adapter, dev);
    snprintf(dev->adapter.name, sizeof(dev->adapter.name),
             DRIVER_NAME " at bus %03d device %03d",
             dev->usb_dev->bus->busnum, dev->usb_dev->devnum);

    dev->adapter.dev.parent = &dev->interface->dev;
    /* initialize ch341 i2c interface */
    ret = ch341_init(dev);

    if (ret < 0) {
        dev_err(&interface->dev, "failed to initialize adapter\n");
        goto error_free;
    }
    /* and finally attach to i2c layer */
    ret = i2c_add_adapter(&dev->adapter);
    if (ret < 0) {
        dev_err(&interface->dev, "failed to add I2C adapter\n");
        goto error_free;
    }

    dev_dbg(&interface->dev, "initialized " DRIVER_NAME "\n");

    return 0;

error_free:
    usb_set_intfdata(interface, NULL);
	usb_put_dev(dev->usb_dev);
	mutex_destroy(&dev->i2c_lock); 
    kfree(dev);
error:
    return ret;
}

static void ch341_i2c_disconnect(struct usb_interface *interface){
    struct ch341_data *dev = usb_get_intfdata(interface);

    i2c_del_adapter(&dev->adapter);
    usb_set_intfdata(interface, NULL);
	usb_put_dev(dev->usb_dev);
	mutex_destroy(&dev->i2c_lock); 
    kfree(dev);

    printk(KERN_INFO "ch341: driver removed!\n");
}

static struct usb_driver ch341_i2c_driver = {
    .name = DRIVER_NAME,
    .probe = ch341_i2c_probe,
    .disconnect = ch341_i2c_disconnect,
    .id_table = ch341_i2c_table,
};

module_usb_driver(ch341_i2c_driver);

MODULE_AUTHOR("Marco Gittler <g.marco@freenet.de>");
MODULE_DESCRIPTION(DRIVER_NAME " driver");
MODULE_LICENSE("GPL");
