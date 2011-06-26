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

#define DEFAULT_CLKDIV 80

#define MCBSP_REQUESTED		(1 << 0)
#define MCBSP_RUNNING		(1 << 1)
#define MCBSP_USER_STALLED	(1 << 2)
#define MCBSP_CONFIG_SET	(1 << 3)

#define TXEN 1
#define RXEN 0

#define DEFAULT_NUM_MOTORS 4
static int num_motors = DEFAULT_NUM_MOTORS;
module_param(num_motors, int, S_IRUGO);
MODULE_PARM_DESC(num_motors, "Number of motors being controlled");

static DECLARE_WAIT_QUEUE_HEAD(wq);

struct ecbsp_dma {
	unsigned long write_count;
	/* unsigned long page; */
	dma_addr_t dma_handle;
	u32 *data;
};

static struct ecbsp_dma dma_block[NUM_DMA_BLOCKS];

#define QUEUE_SIZE (NUM_DMA_BLOCKS - 1)

int q_head;
int q_tail;

static int q_empty(void)
{
	return (q_head == q_tail);
}

static int q_full(void)
{
	return QUEUE_SIZE == (((NUM_DMA_BLOCKS + q_tail) - q_head) 
					% NUM_DMA_BLOCKS);
}

struct ecbsp {
	dev_t devt;
	struct device *dev;
	struct cdev cdev;
	struct semaphore sem;
	struct class *class;

	/* dma stuff */
	unsigned long tx_reg;
	u8 dma_tx_sync;
	short dma_channel;
	short current_dma_idx;

	unsigned short mcbsp_clkdiv;

	unsigned int state;
	char *user_buff;
};

static struct ecbsp ecbsp;


static struct omap_mcbsp_reg_cfg mcbsp_config = {
        .spcr2 = XINTM(3),
        .spcr1 = 0,
        .xcr2  = 0,
        .xcr1  = XFRLEN1(0) | XWDLEN1(OMAP_MCBSP_WORD_32),
        .srgr1 = FWID(31) | CLKGDV(80),
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
	printk(KERN_ALERT "ecbsp_set_mcbsp_config\n");

	if (ecbsp.state & MCBSP_CONFIG_SET) {
		printk(KERN_ALERT "mcbsp already configured\n");
		return 0;
	}

	printk(KERN_ALERT "    omap_mcbsp_set_tx_threshold()\n");
	/* num bytes in a 32-bit 'word' - 1 */
	omap_mcbsp_set_tx_threshold(OMAP_MCBSP3, 3);

	printk(KERN_ALERT "    omap_mcbsp_config()\n");

	/* num bits in our 32-bit 'word' - 1 */
	mcbsp_config.srgr1 = FWID(31) | CLKGDV(ecbsp.mcbsp_clkdiv);

	omap_mcbsp_config(OMAP_MCBSP3, &mcbsp_config); 

	ecbsp.state |= MCBSP_CONFIG_SET;

	return 0;
}

static int ecbsp_alloc_dma_blocks(void)
{
	int i;

	printk(KERN_ALERT "Initializing dma blocks\n");

	for (i = 0; i < NUM_DMA_BLOCKS; i++) {
		dma_block[i].data = kzalloc(DMA_BLOCK_SIZE, 
						GFP_KERNEL | GFP_DMA);
		
		if (!dma_block[i].data) {
			printk(KERN_ALERT "data buff alloc failed\n");
			return -ENOMEM;
		}

		dma_block[i].dma_handle = dma_map_single(ecbsp.dev, 
							dma_block[i].data, 
							DMA_BLOCK_SIZE, 
							DMA_TO_DEVICE);

		if (!dma_block[i].dma_handle) {
			printk(KERN_ALERT "Failed to map dma handle\n");
			return -ENOMEM;
		}
						
		printk(KERN_ALERT "block[%d] data ptr: %p  dma handle: 0x%08X\n", 
			i, dma_block[i].data, dma_block[i].dma_handle);
	}

	return 0;
}

static void ecbsp_free_dma_blocks(void)
{
	int i;

	for (i = 0; i < NUM_DMA_BLOCKS; i++) {
		if (dma_block[i].dma_handle) {
			dma_unmap_single(ecbsp.dev, 
					dma_block[i].dma_handle, 
					DMA_BLOCK_SIZE, 
					DMA_TO_DEVICE);

			dma_block[i].dma_handle = 0;
		}
		
		if (dma_block[i].data) {
			kfree(dma_block[i].data);
			dma_block[i].data = 0;
			
		}		
	}
}

static void ecbsp_mcbsp_dma_write(int idx)
{
	printk(KERN_ALERT "ecbsp_mcbsp_dma_write(%d)\n", idx);

	printk(KERN_ALERT "    omap_set_dma_src_params()\n");
	omap_set_dma_src_params(ecbsp.dma_channel,
				0,
				OMAP_DMA_AMODE_POST_INC,
				dma_block[idx].dma_handle,
				0, 
				0);

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

	dma_block[ecbsp.current_dma_idx].write_count++;
/*
	This is all reader logic. Need to modify for a writer engine.

	// this adds an element to the queue, now safe for reader to use
	// the old q_tail
	q_tail = (q_tail + 1) % NUM_DMA_BLOCKS;

	if (ecbsp.state & MCBSP_RUNNING) {
		if (!q_full()) {
			ecbsp.current_dma_idx = q_tail;
			ecbsp_mcbsp_dma_write(q_tail);
		}
		else {
			ecbsp_mcbsp_stop();
		}
	}
	else {
		ecbsp.current_dma_idx = -1;
	}

	//if (ecbsp.state & MCBSP_USER_STALLED)
	//	wake_up_interruptible(&wq);
*/
}

static void ecbsp_mcbsp_start(void)
{
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

	printk(KERN_ALERT "    omap_mcbsp_start()\n");
	omap_mcbsp_start(OMAP_MCBSP3, TXEN, RXEN);
	ecbsp.state |= MCBSP_RUNNING;	
}

static void ecbsp_write_data(void)
{
	int dma_channel;

	if (!(ecbsp.state & MCBSP_RUNNING)) {
		printk(KERN_ALERT "Not running\n");
		return;
	}

	if (ecbsp.dma_channel == -1) {
		if (omap_request_dma(ecbsp.dma_tx_sync, 
				"McBSP TX",
				ecbsp_dma_callback,
				0,
				&dma_channel)) {
			printk(KERN_ALERT "DMA channel request failed\n");
			return;
		}
	
		omap_set_dma_transfer_params(dma_channel,
						OMAP_DMA_DATA_TYPE_S32,
						num_motors,
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

		/* hack??? - get dma irq sync warnings without this */
		omap_disable_dma_irq(dma_channel, OMAP_DMA_DROP_IRQ);

		ecbsp.dma_channel = dma_channel;

		printk(KERN_ALERT "dma_channel = %d\n", dma_channel);
	}

	q_head = q_tail = 0;
	ecbsp.current_dma_idx = 0;

	ecbsp_mcbsp_dma_write(0);
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

	ecbsp.tx_reg = OMAP34XX_MCBSP3_BASE + OMAP_MCBSP_REG_DXR;
	ecbsp.dma_tx_sync = OMAP24XX_DMA_MCBSP3_TX;

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
	else if (!strncmp(ecbsp.user_buff, "write", 5)) {
		ecbsp_write_data();
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

	ecbsp.mcbsp_clkdiv = DEFAULT_CLKDIV;

	ecbsp.dma_channel = -1;
	ecbsp.current_dma_idx = -1;

	sema_init(&ecbsp.sem, 1);

	if (num_motors < 1 || num_motors > 1024) {
		printk(KERN_ALERT "Invalid num_motors %d, reset to %d\n",
			num_motors, DEFAULT_NUM_MOTORS);
		num_motors = DEFAULT_NUM_MOTORS;
	}

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

	ecbsp_mcbsp_start();

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

