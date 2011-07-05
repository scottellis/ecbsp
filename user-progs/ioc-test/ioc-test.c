#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>

#include "ecbsp.h"

const char device_name[] = "/dev/ecbsp";

void run_tests(int fd)
{
	int val;

	printf("Stopping the ecbsp device\n\n");	

	if (ioctl(fd, ECBSP_STOP, NULL) < 0) {
		perror("ioctl(ECBSP_STOP)");
		return;
	}

	// Number of motors per row, must be a multiple of 16, range 16 - 2048
	if (ioctl(fd, ECBSP_RD_MOTORS_PER_ROW, &val) < 0) {
		perror("ioctl(ECBSP_RD_MOTORS_PER_ROW)");
		return;
	}

	printf("Current motors per row = %d\n", val);

	// make a change
	if (val >= 2048)
		val = 16;
	else
		val = val + 16;

	printf("Setting motors to %d\n", val);

	if (ioctl(fd, ECBSP_WR_MOTORS_PER_ROW, &val) < 0) {
		perror("ioctl(ECBSP_WR_MOTORS_PER_ROW)");
		return;
	}

	if (ioctl(fd, ECBSP_RD_MOTORS_PER_ROW, &val) < 0) {
		perror("ioctl(ECBSP_RD_MOTORS_PER_ROW)");
		return;
	}

	printf("New motors per row = %d\n\n", val);


	// queue threshold, the number of queued commands before starting
	// the dma engine, range 1 - 1023
	if (ioctl(fd, ECBSP_RD_QUEUE_THRESH, &val) < 0) {
		perror("ioctl(ECBSP_RD_QUEUE_THRESH)");
		return;
	}

	printf("Current queue threshold = %d\n", val);

	if (val >= 50)
		val = 1;
	else
		val = val + 5;

	printf("Setting queue threshold to %d\n", val);

	if (ioctl(fd, ECBSP_WR_QUEUE_THRESH, &val) < 0) {
		perror("ioctl(ECBSP_WR_QUEUE_THRESH)");
		return;
	}

	if (ioctl(fd, ECBSP_RD_QUEUE_THRESH, &val) < 0) {
		perror("ioctl(ECBSP_RD_QUEUE_THRESH)");
		return;
	}

	printf("New queue threshold = %d\n", val);



	printf("\nStarting the ecbsp device\n");	

	if (ioctl(fd, ECBSP_START, NULL) < 0) {
		perror("ioctl(ECBSP_START)");
		return;
	}

	printf("\n");
}

int main(int argc, char **argv)
{
	int fd;
	
	fd = open(device_name, O_RDWR);

	if (fd < 0) {
		perror("open");
		exit(1);
	}

	run_tests(fd);

	close(fd);

	return 0;
}


