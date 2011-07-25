/*
  I/O definitions for ecbsp driver interface.
*/

#ifndef ECBSP_H
#define ECBSP_H

#define MOTORS_PER_BYTE 4
#define MAX_MOTORS_PER_ROW 2032
#define NUM_DMA_PAGES 256

// 1 MB, PAGE_SIZE = 4096
#define MMAP_MEM_SIZE (NUM_DMA_PAGES * PAGE_SIZE)

#define DMA_BLOCK_SIZE 512
#define STEP_DATA_SIZE (DMA_BLOCK_SIZE - 4)

// enable block and control block
#define NUM_BLOCKS_PER_STEP 2

// 1024
#define STEPS_IN_QUEUE (MMAP_MEM_SIZE / (DMA_BLOCK_SIZE * NUM_BLOCKS_PER_STEP))

// 1023, our queue always has one empty position
#define QUEUE_SIZE (STEPS_IN_QUEUE - 1)


struct q_ctl {
	int head;
        int tail;
        int tx_head;
	int tx_count.
};

struct q_step_add {
	struct q_ctl q_ctl;
	unsigned int count;
};


/* ioctl commands */
#define ECBSP_IOC_MAGIC			0xEC

#define ECBSP_START			_IO(ECBSP_IOC_MAGIC, 1)
#define ECBSP_STOP			_IO(ECBSP_IOC_MAGIC, 2)

#define ECBSP_RD_MOTORS_PER_ROW		_IOR(ECBSP_IOC_MAGIC, 3, int)
#define ECBSP_WR_MOTORS_PER_ROW		_IOW(ECBSP_IOC_MAGIC, 4, int)

#define ECBSP_RD_QUEUE_THRESH	_IOR(ECBSP_IOC_MAGIC, 5, int)
#define ECBSP_WR_QUEUE_THRESH	_IOW(ECBSP_IOC_MAGIC, 6, int)

#define ECBSP_RD_QUEUE_QUERY	_IOR(ECBSP_IOC_MAGIC, 7, struct q_ctl)
#define ECBSP_WR_QUEUE_ADD	_IOR(ECBSP_IOC_MAGIC, 8, struct q_step_add)


#endif /* ifndef ECBSP_H */

