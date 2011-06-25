/*
 * Linux driver for an OMAP3 McBSP controller to communicate using a SPI
 * protocol to external hardware.
 *
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
#define DEBUG
#include <linux/device.h>
#include <plat/dma.h>
#include <plat/mcbsp.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/moduleparam.h>


#define USER_BUFF_SIZE 4096
#define NUM_DMA_BLOCKS 1
#define DMA_BLOCK_SIZE 4096

#define DEFAULT_NUM_MOTORS 1
#define DEFAULT_CLKDIV 80

#define MCBSP_REQUESTED		(1 << 0)
#define MCBSP_RUNNING		(1 << 1)
#define MCBSP_USER_STALLED	(1 << 2)
#define MCBSP_CONFIG_SET	(1 << 3)

#define TXEN 1
#define RXEN 0


struct ecbsp_dma {
	dma_addr_t dma_handle;
	u32 *data;
	int write_count;
};

static struct ecbsp_dma dma_block;


struct ecbsp_config {
	unsigned short num_motors;
	unsigned short mcbsp_clkdiv;
	unsigned short mcbsp_fper;
};

struct ecbsp {
	dev_t devt;
	struct device *dev;
	struct cdev cdev;
	struct semaphore sem;
	struct class *class;
	struct ecbsp_config config;

	/* dma stuff */
	unsigned long tx_reg;
	u8 dma_tx_sync;
	short dma_channel;

	unsigned int state;
	char *user_buff;
};

static struct ecbsp ecbsp;


static struct omap_mcbsp_reg_cfg mcbsp_test_config = {
        .spcr2 = XINTM(3),
        .spcr1 = 0,
        .xcr2  = XDATDLY(1),
        .xcr1  = XFRLEN1(0) | XWDLEN1(OMAP_MCBSP_WORD_32),
        .srgr1 = FWID(31) | CLKGDV(80),
        .srgr2 = CLKSM | FPER(33),
	.mcr2 = 0,
	.mcr1 = 0,
        .pcr0  = FSXM | CLKXM | FSXP,
	.xccr = XDMAEN | XDISABLE,
	.rccr = 0,
};

static int ecbsp_set_mcbsp_config(void)
{
	printk(KERN_ALERT "ecbsp_set_mcbsp_config\n");

	if (ecbsp.state & MCBSP_CONFIG_SET) {
		printk(KERN_ALERT "mcbsp already configured\n");
	}
	else {
		printk(KERN_ALERT "    omap_mcbsp_set_tx_threshold()\n");
		omap_mcbsp_set_tx_threshold(OMAP_MCBSP3, 3);

		printk(KERN_ALERT "    omap_mcbsp_config()\n");
		omap_mcbsp_config(OMAP_MCBSP3, &mcbsp_test_config); 

		ecbsp.state |= MCBSP_CONFIG_SET;
	}

	return 0;
}

static int ecbsp_alloc_dma_blocks(void)
{
	printk(KERN_ALERT "Initializing dma block\n");

	dma_block.data = kmalloc(DMA_BLOCK_SIZE, GFP_KERNEL | GFP_DMA);
		
	if (!dma_block.data) {
		printk(KERN_ALERT "data buff alloc failed\n");
		return -ENOMEM;
	}
		
	if (!IS_ALIGNED((unsigned long)dma_block.data, DMA_BLOCK_SIZE)) {
		printk(KERN_ALERT "memory not aligned: %p\n", dma_block.data);
		return -ENOMEM;
	}

	/**
	 * dma_map_single - map a single buffer for streaming DMA
	 * @dev: valid struct device pointer, or NULL for ISA and EISA-like devices
	 * @cpu_addr: CPU direct mapped address of buffer
	 * @size: size of buffer to map
	 * @dir: DMA transfer direction
	 *
	 * Ensure that any data held in the cache is appropriately discarded
	 * or written back.
	 *
	 * The device owns this memory once this call has completed.  The CPU
	 * can regain ownership by calling dma_unmap_single() or
	 * dma_sync_single_for_cpu().
	 */
	dma_block.dma_handle = dma_map_single(ecbsp.dev, 
						dma_block.data, 
						DMA_BLOCK_SIZE, 
						DMA_TO_DEVICE);

	if (!dma_block.dma_handle) {
		printk(KERN_ALERT "Failed to map dma handle\n");
		return -ENOMEM;
	}
					
	printk(KERN_ALERT "    data ptr: 0x%p  dma handle: 0x%08X\n", 
		dma_block.data, dma_block.dma_handle);

	return 0;
}

static void ecbsp_free_dma_blocks(void)
{
	if (dma_block.dma_handle) {
		/**
		 * dma_unmap_single - unmap a single buffer previously mapped
		 * @dev: valid struct device pointer, or NULL for ISA and EISA-like devices
		 * @handle: DMA address of buffer
		 * @size: size of buffer (same as passed to dma_map_single)
		 * @dir: DMA transfer direction (same as passed to dma_map_single)
		 *
		 * Unmap a single streaming mode DMA translation.  The handle and size
		 * must match what was provided in the previous dma_map_single() call.
		 * All other usages are undefined.
		 *
		 * After this call, reads by the CPU to the buffer are guaranteed to see
		 * whatever the device wrote there.
		 */
		dma_unmap_single(ecbsp.dev, 
				dma_block.dma_handle, 
				DMA_BLOCK_SIZE, 
				DMA_TO_DEVICE);

		dma_block.dma_handle = 0;
	}
	
	if (dma_block.data) {
		kfree(dma_block.data);
		dma_block.data = 0;
		
	}		
}

static void ecbsp_mcbsp_dma_write(void)
{
	printk(KERN_ALERT "ecbsp_mcbsp_dma_write()\n");

	dma_block.write_count = 0;

	printk(KERN_ALERT "    omap_set_dma_src_params()\n");
	omap_set_dma_src_params(ecbsp.dma_channel,
				0,
				OMAP_DMA_AMODE_POST_INC,
				dma_block.dma_handle,
				0, 0);

	printk(KERN_ALERT "    omap_start_dma()\n");
	omap_start_dma(ecbsp.dma_channel);
}

static void ecbsp_mcbsp_stop(void)
{
	printk(KERN_ALERT "ecbsp_mcbsp_stop\n");

	if (ecbsp.state & MCBSP_RUNNING) {
		ecbsp.state &= ~MCBSP_RUNNING;
	
		printk(KERN_ALERT "    omap_mcbsp_stop()\n");
		omap_mcbsp_stop(OMAP_MCBSP3, TXEN, RXEN);
	}

	if (ecbsp.dma_channel != -1) {
		printk(KERN_ALERT "    omap_stop_dma()\n");
		omap_stop_dma(ecbsp.dma_channel);

		printk(KERN_ALERT "    omap_free_dma()\n");
		omap_free_dma(ecbsp.dma_channel);
		ecbsp.dma_channel = -1;
	}	
}

/* 
  This function is executing inside of the dma irq_handler.
*/
static void ecbsp_dma_callback(int lch, u16 ch_status, void *data)
{
	printk(KERN_ALERT "ecbsp_dma_callback ch_status [CSR%d]: 0x%04X\n", 
		lch, ch_status);

}

static void ecbsp_mcbsp_start(void)
{
	int dma_channel;

	printk(KERN_ALERT "ecbsp_mcbsp_start\n");

	if (ecbsp.state & MCBSP_RUNNING) {
		printk(KERN_ALERT "Already running\n");
		return;
	}

	if (!(ecbsp.state & MCBSP_REQUESTED)) {
		printk(KERN_ALERT "Error: start called before request\n");
		return;
	}

	if (!(ecbsp.state & MCBSP_CONFIG_SET)) {
		if (ecbsp_set_mcbsp_config()) {
			printk(KERN_ALERT "set_mcbsp_config() failed\n");
			return;
		}
	}

	if (ecbsp.dma_channel == -1) {
		printk(KERN_ALERT "    omap_request_dma()\n");
		//int omap_request_dma(int dev_id, 
                //                     const char *dev_name,
		//		     void (*callback)(int lch, u16 ch_status, void *data),
		//		     void *data, 
                //                     int *dma_ch_out)
		if (omap_request_dma(ecbsp.dma_tx_sync, 
                                     "McBSP TX",
				     ecbsp_dma_callback,
				     0,
				     &dma_channel)) {
			printk(KERN_ALERT "DMA channel request failed\n");
			return;
		}

		ecbsp.dma_channel = dma_channel;
		printk(KERN_ALERT "    dma_channel = %d\n", dma_channel);

		printk(KERN_ALERT "    omap_set_dma_transfer_params()\n");
		//void omap_set_dma_transfer_params(int lch, 
                //                                  int data_type, 
                //                                  int elem_count,
		//				    int frame_count, 
                //                                  int sync_mode,
		//				    int dma_trigger, 
                //                                  int src_or_dst_synch)
		omap_set_dma_transfer_params(dma_channel,
						OMAP_DMA_DATA_TYPE_S32,
						1,
						1,
						OMAP_DMA_SYNC_ELEMENT,
						ecbsp.dma_tx_sync, 
						0);

		printk(KERN_ALERT "    omap_set_dma_dest_params()\n");
		//void omap_set_dma_dest_params(int lch, 
                //                              int dest_port, 
                //                              int dest_amode,
		//			        unsigned long dest_start,
		//			        int dst_ei, 
                //                              int dst_fi)
		omap_set_dma_dest_params(dma_channel,
					0,
					OMAP_DMA_AMODE_CONSTANT,
					ecbsp.tx_reg,
					0, 
                                        0);
	}
	
	ecbsp.state &= ~MCBSP_USER_STALLED;

	ecbsp_mcbsp_dma_write();

	printk(KERN_ALERT "    omap_mcbsp_start()\n");
	omap_mcbsp_start(OMAP_MCBSP3, TXEN, RXEN);
	ecbsp.state |= MCBSP_RUNNING;	
	//ecbsp_mcbsp_dma_write();

}

static int ecbsp_mcbsp_request(void)
{
	int ret;
	
	printk(KERN_ALERT "ecbsp_mcbsp_request\n");

	printk(KERN_ALERT "    omap_mcbsp_request()\n");
	ret = omap_mcbsp_request(OMAP_MCBSP3);

	if (ret < 0) {
		printk(KERN_ALERT "omap_mcbsp_request failed\n");
		return ret;
	}

	/* 
	  Find a better way to get this later, though this is how the in-tree
	  SOC sound drivers do it 
	*/
	ecbsp.tx_reg = OMAP34XX_MCBSP3_BASE + OMAP_MCBSP_REG_DXR;
	ecbsp.dma_tx_sync = OMAP24XX_DMA_MCBSP3_TX;
	
	printk(KERN_ALERT "    tx_reg [MCBSP3.DXR] = 0x%08lX  dma_tx_sync = %u\n",
		ecbsp.tx_reg, ecbsp.dma_tx_sync);

	ecbsp.state |= MCBSP_REQUESTED;
	ecbsp.state &= ~MCBSP_CONFIG_SET;
	
	return 0;
}

static ssize_t ecbsp_write(struct file *filp, const char __user *buff,
		size_t count, loff_t *f_pos)
{
	ssize_t status;
	size_t len = USER_BUFF_SIZE - 1;

	if (count == 0)
		return 0;

	if (down_interruptible(&ecbsp.sem))
		return -ERESTARTSYS;

	if (len > count)
		len = count;
	
	memset(ecbsp.user_buff, 0, USER_BUFF_SIZE);
	
	if (copy_from_user(ecbsp.user_buff, buff, len)) {
		status = -EFAULT;
		goto ecbsp_write_done;
	}

	if (!strncmp(ecbsp.user_buff, "request", 7)) {
		ecbsp_mcbsp_request();
	}
	else if (!strncmp(ecbsp.user_buff, "config", 6)) {
		ecbsp_set_mcbsp_config();
	}
	else if (!strncmp(ecbsp.user_buff, "free", 4)) {
		omap_mcbsp_free(OMAP_MCBSP3);
		ecbsp.state &= ~(MCBSP_REQUESTED | MCBSP_RUNNING);
	}
	else if (!strncmp(ecbsp.user_buff, "start", 5)) {
		ecbsp_mcbsp_start();
	}
	else if (!strncmp(ecbsp.user_buff, "stop", 4)) {
		ecbsp_mcbsp_stop();
	}
	else {
		printk(KERN_ALERT "What???\n");
	}
	
	status = len;
	*f_pos += len;

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

	len = 1 + sprintf(ecbsp.user_buff, "mcbsp3 state: 0x%02X\n", 
				ecbsp.state);

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

static int ecbsp_open(struct inode *inode, struct file *filp)
{	
	int status = 0;

	if (down_interruptible(&ecbsp.sem)) 
		return -ERESTARTSYS;
	
	if (!ecbsp.user_buff) {
		ecbsp.user_buff = kmalloc(USER_BUFF_SIZE, GFP_KERNEL);

		if (!ecbsp.user_buff) {
			printk(KERN_ALERT 
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
};

static int __init ecbsp_init_cdev(void)
{
	int error;

	ecbsp.devt = MKDEV(0, 0);

	error = alloc_chrdev_region(&ecbsp.devt, 0, 1, "ecbsp");
	if (error < 0) {
		printk(KERN_ALERT 
			"alloc_chrdev_region() failed: error = %d \n", 
			error);
		
		return -1;
	}

	cdev_init(&ecbsp.cdev, &ecbsp_fops);
	ecbsp.cdev.owner = THIS_MODULE;

	error = cdev_add(&ecbsp.cdev, ecbsp.devt, 1);
	if (error) {
		printk(KERN_ALERT "cdev_add() failed: error = %d\n", error);
		unregister_chrdev_region(ecbsp.devt, 1);
		return -1;
	}	

	return 0;
}

static int __init ecbsp_init_class(void)
{
	ecbsp.class = class_create(THIS_MODULE, "ecbsp");

	if (!ecbsp.class) {
		printk(KERN_ALERT "class_create(ecbsp) failed\n");
		return -1;
	}

	ecbsp.dev = device_create(ecbsp.class, NULL, 
					ecbsp.devt, NULL, "ecbsp");

	if (!ecbsp.dev) {
		class_destroy(ecbsp.class);
		return -1;
	}

	return 0;
}

static int __init ecbsp_init(void)
{
	memset(&ecbsp, 0, sizeof(struct ecbsp));

	ecbsp.config.mcbsp_fper = 255;
	ecbsp.config.mcbsp_clkdiv = DEFAULT_CLKDIV;
	ecbsp.dma_channel = -1;

	sema_init(&ecbsp.sem, 1);

	if (ecbsp_init_cdev())
		goto init_fail_1;

	if (ecbsp_init_class())
		goto init_fail_2;

	if (ecbsp_mcbsp_request()) {
		printk(KERN_ALERT "ecbsp_mcbsp_request() failed\n");
		goto init_fail_3;
	}

	if (ecbsp_alloc_dma_blocks()) {
		printk(KERN_ALERT "dma block allocation failed\n");
		goto init_fail_4;
	}

	return 0;

init_fail_4:
	omap_mcbsp_free(OMAP_MCBSP3);

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
	if (ecbsp.state & MCBSP_REQUESTED) {
		ecbsp_mcbsp_stop();
		omap_mcbsp_free(OMAP_MCBSP3);
		ecbsp.state &= ~MCBSP_REQUESTED;
	}

	ecbsp_free_dma_blocks();

	device_destroy(ecbsp.class, ecbsp.devt);
  	class_destroy(ecbsp.class);

	cdev_del(&ecbsp.cdev);
	unregister_chrdev_region(ecbsp.devt, 1);

	if (ecbsp.user_buff)
		kfree(ecbsp.user_buff);
}
module_exit(ecbsp_exit);


MODULE_AUTHOR("Scott Ellis");
MODULE_DESCRIPTION("Gumstix driver for Elias Crespin artwork");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

