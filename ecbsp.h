/*
  I/O definitions for ecbsp driver interface.
*/

#ifndef ECBSP_H
#define ECBSP_H

/*
 The McBSP3 transmit buffer XB is 128×32 bit words.

 To keep things simpler for our udelay calculation, lets restrict the 
 number of motors to fit in XB in one copy.

 This is not a hard limit and the DMA engine should handle the details,
 but we'll start here until timing gets worked out.

 128 x 4 = 512 bytes is our max required DMA block size
 512 bytes x 4 motors per byte = 2048 max motors 
*/
#define DMA_BLOCK_SIZE 512
#define MAX_MOTORS 2048

#define NUM_DMA_BLOCKS 1024
#define USER_BUFF_SIZE (NUM_DMA_BLOCKS * DMA_BLOCK_SIZE)


/* ioctl commands */
#define ECBSP_IOC_MAGIC		0xEC

#define ECBSP_START		_IO(ECBSP_IOC_MAGIC, 1)
#define ECBSP_STOP		_IO(ECBSP_IOC_MAGIC, 2)

#define ECBSP_RD_NUM_MOTORS	_IOR(ECBSP_IOC_MAGIC, 3, int)
#define ECBSP_WR_NUM_MOTORS	_IOW(ECBSP_IOC_MAGIC, 4, int)

#define ECBSP_RD_QUEUE_THRESH	_IOR(ECBSP_IOC_MAGIC, 5, int)
#define ECBSP_WR_QUEUE_THRESH	_IOW(ECBSP_IOC_MAGIC, 5, int)


#endif /* ifndef ECBSP_H */

