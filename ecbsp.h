/*
  I/O definitions for ecbsp driver interface.
*/

#ifndef ECBSP_H
#define ECBSP_H

/* ioctl commands */
#define ECBSP_IOC_MAGIC		0xEC

#define ECBSP_START		_IO(ECBSP_IOC_MAGIC, 1)
#define ECBSP_STOP		_IO(ECBSP_IOC_MAGIC, 2)

#define ECBSP_RD_NUM_MOTORS	_IOR(ECBSP_IOC_MAGIC, 3, int)
#define ECBSP_WR_NUM_MOTORS	_IOW(ECBSP_IOC_MAGIC, 4, int)

#define ECBSP_RD_QUEUE_THRESH	_IOR(ECBSP_IOC_MAGIC, 5, int)
#define ECBSP_WR_QUEUE_THRESH	_IOW(ECBSP_IOC_MAGIC, 5, int)


#endif /* ifndef ECBSP_H */

