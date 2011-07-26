/*
  I/O definitions for ecbsp driver interface.
*/

#ifndef ECBSP_H
#define ECBSP_H

#define MOTORS_PER_BYTE 4
#define MAX_MOTORS_PER_ROW 2032

#define PAGE_ALLOC_ORDER 8
// 256
#define NUM_DMA_PAGES (1 << PAGE_ALLOC_ORDER)

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
	__u32 head;
        __u32 tail;
        __u32 tx_head;
	__u32 tx_count.
};

struct q_step_add {
	struct q_ctl q_ctl;
	__u32 count;
};

static inline __u8 *get_step_enable_mem(__u8 *mem, int qidx)
{
	if (qidx < 0 || qidx > QUEUE_SIZE)
		return NULL;

	return &mem[2 * DMA_BLOCK_SIZE * qidx];
}

static inline __u8 *get_step_control_mem(__u8 *mem, int qidx)
{
	if (qidx < 0 || qidx > QUEUE_SIZE)
		return NULL;

	return &mem[(2 * DMA_BLOCK_SIZE * qidx) + DMA_BLOCK_SIZE];
}

static inline __u32 get_step_delay(__u8 *mem, int qidx)
{
	__u8 *p;

	if (qidx < 0 || qidx > QUEUE_SIZE)
		return NULL;

	p = &mem[(2 * DMA_BLOCK_SIZE * qidx) + STEP_DATA_SIZE];

	return *((__u32 *)p);
}

static inline void set_step_delay(__u8 *mem, int qidx, __u32 delay)
{
	__u8 *p;

	if (qidx < 0 || qidx > QUEUE_SIZE)
		return;
	
	p = &mem[(2 * DMA_BLOCK_SIZE * qidx) + STEP_DATA_SIZE];

	*((__u32 *)p) = delay;
}		


/* ioctl commands */
#define ECBSP_IOC_MAGIC			0xEC

#define ECBSP_START			_IO(ECBSP_IOC_MAGIC, 1)
#define ECBSP_STOP			_IO(ECBSP_IOC_MAGIC, 2)

#define ECBSP_RD_MOTORS_PER_ROW		_IOR(ECBSP_IOC_MAGIC, 3, int)
#define ECBSP_WR_MOTORS_PER_ROW		_IOW(ECBSP_IOC_MAGIC, 4, int)

#define ECBSP_RD_QUEUE_THRESH	_IOR(ECBSP_IOC_MAGIC, 5, int)
#define ECBSP_WR_QUEUE_THRESH	_IOW(ECBSP_IOC_MAGIC, 6, int)

#define ECBSP_QUEUE_QUERY	_IO(ECBSP_IOC_MAGIC, 7, struct q_ctl)
#define ECBSP_QUEUE_ADD		_IO(ECBSP_IOC_MAGIC, 8, struct q_step_add)


#endif /* ifndef ECBSP_H */

