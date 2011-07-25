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
#include <linux/device.h>
#include <plat/dma.h>
#include <plat/mcbsp.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/moduleparam.h>

#include "ecbsp.h"


#define DEFAULT_DMA_QUEUE_THRESHOLD 8


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

#define DEFAULT_NUM_MOTORS_PER_ROW 256
static int motors = DEFAULT_NUM_MOTORS_PER_ROW;
module_param(motors, int, S_IRUGO);
MODULE_PARM_DESC(motors, "Number of motors per row");


static DECLARE_WAIT_QUEUE_HEAD(wq);


static unsigned char *step_data;
static unsigned int step_delay[STEPS_IN_QUEUE];
static dma_addr_t dma_handle[NUM_DMA_PAGES];

static int q_empty(struct qctl *q)
{
	return (q->tx_head == q->tail);
}

static int q_remove(struct qctl *q)
{
	int ret;

	if (q_empty(q))
		return -1;

	ret = q->tx_head;

	q->tx_head = (q->tx_head + 1) % STEPS_IN_QUEUE;

	return ret;
}

static int q_unmap_page(struct qctl *q)
{
	int ret, steps;

	steps = ((STEPS_IN_QUEUE + q->tx_head) - q->head) % STEPS_IN_QUEUE; 

	if (steps < STEPS_IN_PAGE)
		return -1;

	ret = q->head;

	q->head = (q->head + STEPS_IN_PAGE) % STEPS_IN_QUEUE;

	return ret;
}

static int q_count(struct qctl *q)
{
	return ((STEPS_IN_QUEUE + q->tail) - q->head) % STEPS_IN_QUEUE;
}

static int q_full(struct qctl *q)
{
	return QUEUE_SIZE == q_count(q);
}

static int q_add(struct qctl *q)
{
	if (q_full(q))
		return -1;

	q->tail = (q->tail + STEPS_IN_PAGE) % STEPS_IN_QUEUE;

	return q->tail;
}

#define TX_FIRST_ENABLE_BLOCK	1
#define TX_DATA_BLOCK		2
#define TX_SECOND_ENABLE_BLOCK	3

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
	int motors_per_row;
	int queue_threshold;
	volatile unsigned int state;
	int tx_sequence;
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
  two CLKDV cycles between each 'word'. 
  The 32 bit word lengths are determined with FWID(31).
  The two CLKDV cycles between words is determined by FPER(33).
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

static int ecbsp_map_dma_block(int didx)
{
	if (didx < 0 || didx >= NUM_DMA_PAGES) {
		printk(KERN_ERR "Invalid idx to map_dma_block: %d\n", didx);
		return -1;
	}

	if (dma_handle[didx]) {
		printk(KERN_ERR "dma block already mapped\n");
		return -EINVAL;
	}

	dma_handle[didx] = dma_map_single(ecbsp.dev, 
					step_data + (didx * DMA_BLOCK_SIZE), 
					DMA_BLOCK_SIZE, 
					DMA_TO_DEVICE);

	if (!dma_handle[sidx]) {
		printk(KERN_ERR "Failed to map dma handle\n");
		return -ENOMEM;
	}

						
	return 0;
}

static void ecbsp_unmap_dma_block(int didx)
{
	if (didx < 0 || didx >= NUM_DMA_PAGES) {
		printk(KERN_ERR "Invalid idx to unmap_dma_block: %d\n", didx);
		return -1;
	}

	if (dma_handle[didx]) {
		dma_unmap_single(ecbsp.dev, 
				dma_handle[didx], 
				DMA_BLOCK_SIZE, 
				DMA_TO_DEVICE);

		dma_handle[didx] = 0;
	}
}

static void ecbsp_mcbsp_dma_write(int didx)
{
	omap_set_dma_src_params(ecbsp.dma_channel,
				0,
				OMAP_DMA_AMODE_POST_INC,
				dma_handle[didx],
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

	for (i = 0; i < NUM_DMA_BLOCKS; i++) {
		ecbsp_unmap_dma_block(&data_block[i]);
		ecbsp_unmap_dma_block(&enable_block[i]);
	}

	return 0;
}

static inline void ecbsp_start_dma_with_delay(void)
{
	ecbsp.current_dma_idx = q_remove();
		
	// block delay is in microseconds

	// for short delays, use udelay
	if (block_delay[ecbsp.current_dma_idx] < 100) {
		// no delay is okay, so check
		if (block_delay[ecbsp.current_dma_idx] > 0)
			udelay(block_delay[ecbsp.current_dma_idx]);

		ecbsp_mcbsp_dma_write(&enable_block[ecbsp.current_dma_idx]);
		ecbsp.tx_sequence = TX_FIRST_ENABLE_BLOCK;
	}
	// for delays longer then 100 us start a timer, 
	else {
		hrtimer_start(&ecbsp.timer, 
			ktime_set(block_delay[ecbsp.current_dma_idx] / 1000000, 
				(block_delay[ecbsp.current_dma_idx] % 1000000)
					* 1000), 
			HRTIMER_MODE_REL);
	}
}

/* this is executing inside of the dma irq_handler */
static void ecbsp_dma_callback(int lch, u16 ch_status, void *data)
{
	if (ch_status != 0x0020)
		printk(KERN_ERR 
			"ecbsp_dma_callback ch_status [CSR%d]: 0x%04X\n", 
			lch, ch_status);

	if (!(ecbsp.state & MCBSP_STARTED))
		return;

	if (ecbsp.tx_sequence == TX_FIRST_ENABLE_BLOCK) {
		ecbsp_mcbsp_dma_write(&data_block[ecbsp.current_dma_idx]);
		ecbsp.tx_sequence = TX_DATA_BLOCK;
		return;
	}
	else if (ecbsp.tx_sequence == TX_DATA_BLOCK) {
		ecbsp_mcbsp_dma_write(&enable_block[ecbsp.current_dma_idx]);
		ecbsp.tx_sequence = TX_SECOND_ENABLE_BLOCK;
		return;
	}

	// only count when we complete all three sequences
	ecbsp.write_count++;

	if (q_empty()) {
		ecbsp.state &= ~DMA_RUNNING;
		return;
	}

	ecbsp_start_dma_with_delay();
}

static enum hrtimer_restart ecbsp_timer_callback(struct hrtimer *timer)
{
	if (ecbsp.state & MCBSP_STARTED) {
		ecbsp_mcbsp_dma_write(&enable_block[ecbsp.current_dma_idx]);
		ecbsp.tx_sequence = TX_FIRST_ENABLE_BLOCK;
	}

	return HRTIMER_NORESTART;
}

static int ecbsp_kickstart_dma(void)
{	
	if (!(ecbsp.state & MCBSP_STARTED))
		return 0;

	if (ecbsp.state & DMA_RUNNING)
		return 0;

	if (q_count() < ecbsp.queue_threshold)
		return 0;

	ecbsp_start_dma_with_delay();

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
						ecbsp.motors_per_row / 16,
						1,
						OMAP_DMA_SYNC_BLOCK,
						ecbsp.dma_tx_sync, 
						OMAP_DMA_DST_SYNC);

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

	// block delay is the first 4 bytes
	block_delay[q_tail] = *(u32 *)data;

	// unmap the dma memory
	ecbsp_unmap_dma_block(&data_block[q_tail]);
	ecbsp

	// copy the new user data into data_block[q_tail].data
	memcpy(data_block[q_tail].data, data + 4, ecbsp.motors_per_row / 4);

	// then map the buffer again
	if (ecbsp_map_dma_block(&data_block[q_tail])) {
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
	We aren't using this yet, but might in the future.
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
	
	cmd_size = 4 + (ecbsp.motors_per_row / 4);
	// printk(KERN_ALERT "Expecting cmd_size = %d\n", cmd_size);

	if ((len % cmd_size) != 0) {
		printk(KERN_ERR "Invalid write len %d\n", len);
		status = -EINVAL;
		goto ecbsp_write_done;
	}

	//printk(KERN_ALERT "Copying %d bytes from user\n", len);

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

	case ECBSP_RD_MOTORS_PER_ROW:
		result = put_user(ecbsp.motors_per_row, (int __user*) arg);
		break;

	case ECBSP_WR_MOTORS_PER_ROW:
		if (ecbsp.state & MCBSP_STARTED) {
			result = -EBUSY;
			break;
		}

		result = get_user(val, (int __user*) arg);
		if (result)
			break;

		if (val <= 0 || val > MAX_MOTORS_PER_ROW || (val % 16) != 0)
			result = -EINVAL;
		else
			ecbsp.motors_per_row = val;

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
/*
static int ecbsp_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long size;
	int ret;

	size = vma->vm_end - vma->vm_start;

	if (size > mt9p001_dev.reserved_mem_size) {
		printk(KERN_ALERT "mmap request too big: 0x%lx\n", size);
		return -EINVAL;
	}

	
//	printk(KERN_ALERT "pgoff=0x%lx  start=0x%lx  end=0x%lx  size=%lu\n", 
//		vma->vm_pgoff, vma->vm_start, vma->vm_end, size);

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_flags |= VM_RESERVED | VM_IO;

        ret = remap_pfn_range(vma,
				vma->vm_start,
				mt9p001_dev.reserved_mem_start >> PAGE_SHIFT,
				size,
				vma->vm_page_prot);

	if (ret < 0)
		printk(KERN_ALERT "remap_pfn_range failed\n");

	return ret;	
}
*/
static int ecbsp_open(struct inode *inode, struct file *filp)
{
	int i;	
	int status = 0;

	if (down_interruptible(&ecbsp.sem)) 
		return -ERESTARTSYS;
	
	for (i = 0; i < NUM_DMA_BLOCKS; i++) {
		if (!data_block[i].data) {
			data_block[i].data = kzalloc(DMA_BLOCK_SIZE, 
							GFP_KERNEL | GFP_DMA);
		
			if (!data_block[i].data) {
				printk(KERN_ALERT "data buff alloc failed\n");
				return -ENOMEM;
			}
		}
	}

	if (!enable_block.data) {
		enable_block.data = kmalloc(DMA_BLOCK_SIZE, 
						GFP_KERNEL | GFP_DMA);

		if (!enable_block.data) {
			printk(KERN_ALERT "enable_block buff alloc failed\n");
			return -ENOMEM;
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
//	.mmap = ecbsp_mmap,
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

	if (motors == 0 || motors > MAX_MOTORS_PER_ROW || (motors % 16 != 0)) {
		printk(KERN_INFO "Invalid motors %d, reset to %d\n",
			motors, DEFAULT_NUM_MOTORS_PER_ROW);
		motors = DEFAULT_NUM_MOTORS_PER_ROW;
	}

	ecbsp.motors_per_row = motors;

	if (ecbsp_init_cdev())
		goto init_fail_1;

	if (ecbsp_init_class())
		goto init_fail_2;

	if (ecbsp_mcbsp_request()) {
		printk(KERN_ALERT "ecbsp_mcbsp_request() failed\n");
		goto init_fail_3;
	}

	hrtimer_init(&ecbsp.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ecbsp.timer.function = ecbsp_timer_callback;

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
		if (data_block[i].data)
			kfree(data_block[i].data);
	}

	if (enable_block.data)
		kfree(enable_block.data);
}
module_exit(ecbsp_exit);


MODULE_AUTHOR("Scott Ellis");
MODULE_DESCRIPTION("Gumstix McBSP driver for Elias Crespin artwork");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

