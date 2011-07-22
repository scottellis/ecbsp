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

 McBSP3 has a 512 byte TX buffer

 Each motor takes 6 bits to control - mask enable(2)/data(2)/mask enable(2)

 512 bytes * 8 bits/1 byte * 1 motor/6 bits = 682.66 motors

 So a convenient max for now is

 MAX_MOTORS_PER_ROW = 682

 We'll probably change this later.

 Keep DMA_BLOCK_SIZE at PAGE_SIZE to ensure cache alignment.
*/
#define DMA_BLOCK_SIZE 4096
#define MAX_MOTORS_PER_ROW 682

#define NUM_DMA_BLOCKS 1024
#define USER_BUFF_SIZE (NUM_DMA_BLOCKS * DMA_BLOCK_SIZE)


/* ioctl commands */
#define ECBSP_IOC_MAGIC			0xEC

#define ECBSP_START			_IO(ECBSP_IOC_MAGIC, 1)
#define ECBSP_STOP			_IO(ECBSP_IOC_MAGIC, 2)

#define ECBSP_RD_MOTORS_PER_ROW		_IOR(ECBSP_IOC_MAGIC, 3, int)
#define ECBSP_WR_MOTORS_PER_ROW		_IOW(ECBSP_IOC_MAGIC, 4, int)

#define ECBSP_RD_QUEUE_THRESH		_IOR(ECBSP_IOC_MAGIC, 5, int)
#define ECBSP_WR_QUEUE_THRESH		_IOW(ECBSP_IOC_MAGIC, 5, int)


#endif /* ifndef ECBSP_H */

