/*
  Linux driver for an OMAP3 McBSP controller to operate multiple motors using 
  a SPI protocol. The application is for one of Elias Crespin's kinetic art
  works.

  Copyright (C) 2011, Scott Ellis
 
  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#define DEBUG
#include <linux/device.h>
#include <plat/dma.h>
#include <plat/mcbsp.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/moduleparam.h>

#include "ecbsp.h"


#define DEFAULT_DMA_QUEUE_THRESHOLD 10


/* mcbsp3->iclk is 83 MHz, 
  a divider of 83 gives ~1MHz clock 
  a divider of 5 gives ~16MHz clock
*/
#define DEFAULT_CLKDIV 5

#define MCBSP_REQUESTED		(1 << 0)
#define MCBSP_CONFIG_SET	(1 << 1)
#define MCBSP_STARTED		(1 << 2)
#define DMA_RUNNING		(1 << 3)


#define TXEN 1
#define RXEN 0

#define DEFAULT_NUM_MOTORS 768
static int motors = DEFAULT_NUM_MOTORS;
module_param(motors, int, S_IRUGO);
MODULE_PARM_DESC(motors, "Number of motors being controlled");

#define DEFAULT_DELAY_US 50
static int delay_us = DEFAULT_DELAY_US;
module_param(delay_us, int, S_IRUGO);
MODULE_PARM_DESC(delay_us, "Delay between blocks in microseconds");

/* manual control for testing only */
static int use_hrtimer = 0;
module_param(use_hrtimer, int, S_IRUGO);
MODULE_PARM_DESC(use_hrtimer, "Use hrtimer instead of udelay");


static DECLARE_WAIT_QUEUE_HEAD(wq);

struct ecbsp_dma {
	dma_addr_t dma_handle;
	u8 *data;
};

static struct ecbsp_dma dma_block[NUM_DMA_BLOCKS];

#define QUEUE_SIZE (NUM_DMA_BLOCKS - 1)

int q_head;
int q_tail;

static int q_empty(void)
{
	return (q_head == q_tail);
}

static int q_count(void)
{
	return ((NUM_DMA_BLOCKS + q_tail) - q_head) % NUM_DMA_BLOCKS;
}

static int q_full(void)
{
	return QUEUE_SIZE == q_count();
}

static int q_add(void)
{
	if (q_full())
		return -1;

	q_tail = (q_tail + 1) % NUM_DMA_BLOCKS;

	return q_tail;
}

static int q_remove(void)
{
	int ret;

	if (q_empty())
		return -1;

	ret = q_head;

	q_head = (q_head + 1) % NUM_DMA_BLOCKS;

	return ret;
}

struct ecbsp {
	dev_t devt;
	struct device *dev;
	struct cdev cdev;
	struct semaphore sem;
	struct class *class;
	struct hrtimer timer;
	
	/* dma stuff */
	unsigned long tx_reg;
	u8 dma_tx_sync;
	short dma_channel;
	short current_dma_idx;
	
	int write_count;
	unsigned short mcbsp_clkdiv;
	int num_motors;
	int queue_threshold;
	volatile unsigned int state;
	char *user_buff;
};

static struct ecbsp ecbsp;


static struct omap_mcbsp_reg_cfg mcbsp_config = {
        .spcr2 = XINTM(3),
        .spcr1 = 0,
        .xcr2  = 0,
        .xcr1  = XFRLEN1(0) | XWDLEN1(OMAP_MCBSP_WORD_32),
        .srgr1 = FWID(31) | CLKGDV(DEFAULT_CLKDIV),
        .srgr2 = CLKSM | FPER(33),
	.mcr2 = 0,
	.mcr1 = 0,
        .pcr0  = FSXM | CLKXM | FSXP,
	.xccr = XDMAEN | XDISABLE,
	.rccr = 0,
};

/*
  Configure for 32 bit 'word' lengths with a frame sync pulse of
  two CLKDV cycles between each 'word'
*/
static int ecbsp_set_mcbsp_config(void)
{
	if (ecbsp.state & MCBSP_CONFIG_SET) {
		printk(KERN_INFO "mcbsp already configured\n");
		return 0;
	}

	/* num bytes in a 32-bit 'word' - 1 */
	omap_mcbsp_set_tx_threshold(OMAP_MCBSP3, 3);

	/* num bits in our 32-bit 'word' - 1 */
	mcbsp_config.srgr1 = FWID(31) | CLKGDV(ecbsp.mcbsp_clkdiv);

	omap_mcbsp_config(OMAP_MCBSP3, &mcbsp_config); 

	ecbsp.state |= MCBSP_CONFIG_SET;

	return 0;
}

static int ecbsp_map_dma_block(int idx)
{
	if (idx < 0 || idx >= NUM_DMA_BLOCKS) {
		printk(KERN_ERR "Bad index to map_dma_block: %d\n", idx);
		return -EINVAL;
	}

	if (!dma_block[idx].data) {
		printk(KERN_ERR "dma_block[%d].data is NULL, can't map\n", 
			idx);
		return -ENOMEM;
	}

	if (dma_block[idx].dma_handle) {
		printk(KERN_ERR "dma_block[%d] already mapped\n", idx);
		return -EINVAL;
	}

	dma_block[idx].dma_handle = dma_map_single(ecbsp.dev, 
						dma_block[idx].data, 
						ecbsp.num_motors / 4, 
						DMA_TO_DEVICE);

	if (!dma_block[idx].dma_handle) {
		printk(KERN_ERR "Failed to map dma handle\n");
		return -ENOMEM;
	}
						
	return 0;
}

static void ecbsp_unmap_dma_block(int idx)
{
	if (idx < 0 || idx >= NUM_DMA_BLOCKS) {
		printk(KERN_ALERT "Bad index to unmap_dma_block: %d\n", idx);
		return;
	}

	if (dma_block[idx].dma_handle) {
		dma_unmap_single(ecbsp.dev, 
				dma_block[idx].dma_handle, 
				ecbsp.num_motors / 4, 
				DMA_TO_DEVICE);

		dma_block[idx].dma_handle = 0;
	}
}

static void ecbsp_mcbsp_dma_write(int idx)
{
	omap_set_dma_src_params(ecbsp.dma_channel,
				0,
				OMAP_DMA_AMODE_POST_INC,
				dma_block[idx].dma_handle,
				0, 
				0);

	omap_start_dma(ecbsp.dma_channel);

	ecbsp.state |= DMA_RUNNING;
}

static int ecbsp_mcbsp_stop(void)
{
	int i;

	if (ecbsp.state & MCBSP_STARTED) {
		ecbsp.state &= ~MCBSP_STARTED;
		omap_mcbsp_stop(OMAP_MCBSP3, TXEN, RXEN);
	}

	if (ecbsp.dma_channel != -1) {
		omap_stop_dma(ecbsp.dma_channel);
		omap_free_dma(ecbsp.dma_channel);
		ecbsp.dma_channel = -1;
	}	

	for (i = 0; i < NUM_DMA_BLOCKS; i++)
		ecbsp_unmap_dma_block(i);

	return 0;
}

/* this is executing inside of the dma irq_handler */
static void ecbsp_dma_callback(int lch, u16 ch_status, void *data)
{
	if (ch_status != 0x0020)
		printk(KERN_ERR 
			"ecbsp_dma_callback ch_status [CSR%d]: 0x%04X\n", 
			lch, ch_status);

	ecbsp.write_count++;
	
	if (!(ecbsp.state & MCBSP_STARTED))
		return;

	if (use_hrtimer)
		return;

	if (q_empty()) {
		ecbsp.state &= ~DMA_RUNNING;
		return;
	}

	ecbsp.current_dma_idx = q_remove();
	udelay(delay_us);
	ecbsp_mcbsp_dma_write(ecbsp.current_dma_idx);
}

static enum hrtimer_restart ecbsp_timer_callback(struct hrtimer *timer)
{
	if (!(ecbsp.state & MCBSP_STARTED))
		return HRTIMER_NORESTART;

	if (q_empty()) {
		ecbsp.state &= ~DMA_RUNNING;
		return HRTIMER_NORESTART;
	}

	ecbsp.current_dma_idx = q_remove();
	ecbsp_mcbsp_dma_write(ecbsp.current_dma_idx);
		
	hrtimer_forward_now(&ecbsp.timer, ktime_set(0, delay_us * 1000));

	return HRTIMER_RESTART;
}

static int ecbsp_kickstart_dma(void)
{	
	if (!(ecbsp.state & MCBSP_STARTED))
		return 0;

	if (ecbsp.state & DMA_RUNNING)
		return 0;

	if (q_count() < ecbsp.queue_threshold)
		return 0;

	ecbsp.current_dma_idx = q_remove();

	if (use_hrtimer)
		hrtimer_start(&ecbsp.timer, ktime_set(0, delay_us * 1000), 
				HRTIMER_MODE_REL);

	ecbsp_mcbsp_dma_write(ecbsp.current_dma_idx);

	return 0;
}

static int ecbsp_mcbsp_start(void)
{
	int dma_channel, ret;

	if (ecbsp.state & MCBSP_STARTED)
		return 0;

	if (!(ecbsp.state & MCBSP_REQUESTED)) {
		printk(KERN_ERR "Error: start called before request\n");
		return -EINVAL;
	}

	if (ecbsp.dma_channel == -1) {
		ret = omap_request_dma(ecbsp.dma_tx_sync, 
				"McBSP TX",
				ecbsp_dma_callback,
				0,
				&dma_channel);

		if (ret) {
			printk(KERN_ERR "DMA channel request failed\n");
			return ret;
		}
	
		omap_set_dma_transfer_params(dma_channel,
						OMAP_DMA_DATA_TYPE_S32,
						ecbsp.num_motors / 16,
						1,
						OMAP_DMA_SYNC_BLOCK,
						ecbsp.dma_tx_sync, 
						0);

		omap_set_dma_dest_params(dma_channel,
					0,
					OMAP_DMA_AMODE_CONSTANT,
					ecbsp.tx_reg,
					0, 
					0);

		omap_disable_dma_irq(dma_channel, OMAP_DMA_DROP_IRQ);
		ecbsp.dma_channel = dma_channel;
	}


	if (!(ecbsp.state & MCBSP_CONFIG_SET)) {
		if (ecbsp_set_mcbsp_config()) {
			printk(KERN_ERR "set_mcbsp_config() failed\n");
			return -EINVAL;
		}
	}

	omap_mcbsp_start(OMAP_MCBSP3, TXEN, RXEN);
	ecbsp.state |= MCBSP_STARTED;

	return ecbsp_kickstart_dma();
}

static int ecbsp_queue_data(unsigned char *data)
{
	if (q_full())
		return -EAGAIN;

	// unmap the dma memory
	ecbsp_unmap_dma_block(q_tail);

	// copy the new user data into dma_block[q_tail].data
	// I'll clean this up shortly, need to skip over the delay
	// but only copy as much as we'll be transmitting
	memcpy(dma_block[q_tail].data, data + 4, ecbsp.num_motors / 4);

	// then map the buffer again
	if (ecbsp_map_dma_block(q_tail)) {
		printk(KERN_ERR "DMA mapping failed in ecbsp_queue_data()\n");
		return -ENOMEM;
	}

	q_add();

	return 0;
}

static int ecbsp_mcbsp_request(void)
{
	int ret;
	
	/* 
	We don't want POLL_IO, but we don't want the mcbsp driver
        allocating irq handlers for McBSP events that we might need.
	Have to call this before omap_mcbsp_request().
	*/
	ret = omap_mcbsp_set_io_type(OMAP_MCBSP3, OMAP_MCBSP_POLL_IO);
	
	if (ret < 0) {
		printk(KERN_ERR "omap_mcbsp_set_io_type failed\n");
		return ret;
	}

	ret = omap_mcbsp_request(OMAP_MCBSP3);

	if (ret < 0) {
		printk(KERN_ERR "omap_mcbsp_request failed\n");
		return ret;
	}

	ecbsp.tx_reg = OMAP34XX_MCBSP3_BASE + OMAP_MCBSP_REG_DXR;
	ecbsp.dma_tx_sync = OMAP24XX_DMA_MCBSP3_TX;

	ecbsp.state |= MCBSP_REQUESTED;
	ecbsp.state &= ~MCBSP_CONFIG_SET;
	
	return 0;
}

static ssize_t ecbsp_write(struct file *filp, const char __user *buff,
		size_t count, loff_t *f_pos)
{
	int i, len, cmd_size;
	ssize_t status;

	if (count == 0)
		return 0;

	if (down_interruptible(&ecbsp.sem))
		return -ERESTARTSYS;

	if (count > USER_BUFF_SIZE)
		len = USER_BUFF_SIZE;
	else
		len = count;
	
	cmd_size = 4 + (ecbsp.num_motors / 4);
	printk(KERN_ALERT "Expecting cmd_size = %d\n", cmd_size);

	if ((len % cmd_size) != 0) {
		printk(KERN_ERR "Invalid write len %d\n", len);
		status = -EINVAL;
		goto ecbsp_write_done;
	}

	printk(KERN_ALERT "Copying %d bytes from user\n", len);

	if (copy_from_user(ecbsp.user_buff, buff, len)) {
		status = -EFAULT;
		goto ecbsp_write_done;
	}

	for (i = 0; i < len; i += cmd_size) {
		// printk(KERN_ALERT "Queueing command %d\n", i / cmd_size);
 
		status = ecbsp_queue_data(ecbsp.user_buff + i);
		if (status < 0) {
			if (status == -EAGAIN)
				break;
			else
				goto ecbsp_write_done;
		}
	}
		
	ecbsp_kickstart_dma();

	status = i;
	*f_pos += i;

ecbsp_write_done:

	up(&ecbsp.sem);

	return status;
}

static ssize_t ecbsp_read(struct file *filp, char __user *buff, 
				size_t count, loff_t *offp)
{
	ssize_t len;

	if (*offp > 0)
		return 0;

	if (down_interruptible(&ecbsp.sem))
		return -ERESTARTSYS;

	len = 1 + sprintf(ecbsp.user_buff, "ecbsp state: 0x%02X  writes = %d\n", 
				ecbsp.state, ecbsp.write_count);

	if (count < len)
		len = count;

	if (copy_to_user(buff, ecbsp.user_buff, len)) {
		len = -EFAULT;
		goto ecbsp_read_done;
	}

	*offp += len;

ecbsp_read_done:
			
	up(&ecbsp.sem);
	
	return len;	
}

static long ecbsp_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int val;
	int result = -ENOTTY;

	if (down_interruptible(&ecbsp.sem)) 
		return -ERESTARTSYS;

	switch (cmd) {
	case ECBSP_START:
		result = ecbsp_mcbsp_start();
		break;

	case ECBSP_STOP:
		result = ecbsp_mcbsp_stop();
		break;

	case ECBSP_RD_NUM_MOTORS:
		result = put_user(ecbsp.num_motors, (int __user*) arg);
		break;

	case ECBSP_WR_NUM_MOTORS:
		if (ecbsp.state & MCBSP_STARTED) {
			result = -EBUSY;
			break;
		}

		result = get_user(val, (int __user*) arg);
		if (result)
			break;

		if (val <= 0 || val > MAX_MOTORS || (val % 16) != 0)
			result = -EINVAL;
		else
			ecbsp.num_motors = val;

		break;

	case ECBSP_RD_QUEUE_THRESH:
		result = put_user(ecbsp.queue_threshold, (int __user*) arg);
		break;

	case ECBSP_WR_QUEUE_THRESH:
		if (ecbsp.state & MCBSP_STARTED) {
			result = -EBUSY;
			break;
		}

		result = get_user(val, (int __user*) arg);
		if (result)
			break;

		if (val < 1 || val > QUEUE_SIZE)
			result = -EINVAL;
		else
			ecbsp.queue_threshold = val;

		break;

	default:
		printk(KERN_ERR "Unhandled ioctl command %d\n", cmd);
		break;
	}

	up(&ecbsp.sem);

	return result;
}

static int ecbsp_open(struct inode *inode, struct file *filp)
{
	int i;	
	int status = 0;

	if (down_interruptible(&ecbsp.sem)) 
		return -ERESTARTSYS;
	
	for (i = 0; i < NUM_DMA_BLOCKS; i++) {
		if (!dma_block[i].data) {
			dma_block[i].data = kzalloc(DMA_BLOCK_SIZE, 
							GFP_KERNEL | GFP_DMA);
		
			if (!dma_block[i].data) {
				printk(KERN_ALERT "data buff alloc failed\n");
				return -ENOMEM;
			}
		}
	}

	if (!ecbsp.user_buff) {
		ecbsp.user_buff = kmalloc(USER_BUFF_SIZE, GFP_KERNEL);

		if (!ecbsp.user_buff) {
			printk(KERN_ERR 
				"ecbsp_open: user_buff alloc failed\n");

			status = -ENOMEM;
		}
	}

	up(&ecbsp.sem);

	return status;
}

static const struct file_operations ecbsp_fops = {
	.owner = THIS_MODULE,
	.open =	ecbsp_open,	
	.read =	ecbsp_read,
	.write = ecbsp_write,
	.unlocked_ioctl = ecbsp_ioctl,
};

static int __init ecbsp_init_cdev(void)
{
	int error;

	ecbsp.devt = MKDEV(0, 0);

	error = alloc_chrdev_region(&ecbsp.devt, 0, 1, "ecbsp");
	if (error < 0) {
		printk(KERN_ERR "alloc_chrdev_region() failed: error = %d \n", 
			error);
		
		return -1;
	}

	cdev_init(&ecbsp.cdev, &ecbsp_fops);
	ecbsp.cdev.owner = THIS_MODULE;

	error = cdev_add(&ecbsp.cdev, ecbsp.devt, 1);
	if (error) {
		printk(KERN_ERR "cdev_add() failed: error = %d\n", error);
		unregister_chrdev_region(ecbsp.devt, 1);
		return -1;
	}	

	return 0;
}

static int __init ecbsp_init_class(void)
{
	ecbsp.class = class_create(THIS_MODULE, "ecbsp");

	if (!ecbsp.class) {
		printk(KERN_ERR "class_create(ecbsp) failed\n");
		return -1;
	}

	ecbsp.dev = device_create(ecbsp.class, NULL, ecbsp.devt, NULL, "ecbsp");

	if (!ecbsp.dev) {
		class_destroy(ecbsp.class);
		return -1;
	}

	return 0;
}

static int __init ecbsp_init(void)
{
	memset(&ecbsp, 0, sizeof(struct ecbsp));

	ecbsp.mcbsp_clkdiv = DEFAULT_CLKDIV;

	ecbsp.dma_channel = -1;
	ecbsp.current_dma_idx = -1;
	ecbsp.queue_threshold = DEFAULT_DMA_QUEUE_THRESHOLD;

	sema_init(&ecbsp.sem, 1);

	if (motors == 0 || motors > MAX_MOTORS || (motors % 16) != 0) {
		printk(KERN_INFO "Invalid motors %d, reset to %d\n",
			motors, DEFAULT_NUM_MOTORS);
		motors = DEFAULT_NUM_MOTORS;
	}

	ecbsp.num_motors = motors;

	if (delay_us < 1 || delay_us > 900000) {
		printk(KERN_INFO "Invalid delay_us %d, reset to %d\n",
			delay_us, DEFAULT_DELAY_US);

		delay_us = DEFAULT_DELAY_US;
	}

	if (use_hrtimer != 1)
		use_hrtimer = 0;

	if (ecbsp_init_cdev())
		goto init_fail_1;

	if (ecbsp_init_class())
		goto init_fail_2;

	if (ecbsp_mcbsp_request()) {
		printk(KERN_ALERT "ecbsp_mcbsp_request() failed\n");
		goto init_fail_3;
	}

	/* force hrtimer for long delays */
	if (delay_us > 300)
		use_hrtimer = 1;

	if (use_hrtimer) {
		hrtimer_init(&ecbsp.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ecbsp.timer.function = ecbsp_timer_callback;
	}
	
	printk(KERN_INFO "use_hrtimer = %d  delay_us = %d\n",
		use_hrtimer, delay_us);

	/* for debug convenience */
	if (ecbsp_mcbsp_start())
		printk(KERN_ERR "Auto ecbsp_mcbsp_start() failed\n");

	return 0;


init_fail_3:
	device_destroy(ecbsp.class, ecbsp.devt);
  	class_destroy(ecbsp.class);

init_fail_2:
	cdev_del(&ecbsp.cdev);
	unregister_chrdev_region(ecbsp.devt, 1);

init_fail_1:

	return -1;
}
module_init(ecbsp_init);

static void __exit ecbsp_exit(void)
{
	int i;

	if (ecbsp.state & MCBSP_REQUESTED) {
		ecbsp_mcbsp_stop();
		omap_mcbsp_free(OMAP_MCBSP3);
		ecbsp.state &= ~MCBSP_REQUESTED;
	}

	device_destroy(ecbsp.class, ecbsp.devt);
  	class_destroy(ecbsp.class);

	cdev_del(&ecbsp.cdev);
	unregister_chrdev_region(ecbsp.devt, 1);

	if (ecbsp.user_buff)
		kfree(ecbsp.user_buff);

	for (i = 0; i < NUM_DMA_BLOCKS; i++) {
		if (dma_block[i].data) {
			kfree(dma_block[i].data);
			dma_block[i].data = 0;
		}
	}
}
module_exit(ecbsp_exit);


MODULE_AUTHOR("Scott Ellis");
MODULE_DESCRIPTION("Gumstix McBSP driver for Elias Crespin artwork");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

