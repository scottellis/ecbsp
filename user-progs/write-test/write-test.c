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

// one BLOCK represents all the commands for NUM_MOTORS + a delay
#define NUM_BLOCKS 512
//#define NUM_BLOCKS 8

#define NUM_MOTORS 768
//#define NUM_MOTORS 16

const char device_name[] = "/dev/ecbsp";

int write_cmd_buff(int fd, unsigned char *data, int num_blocks, int block_size)
{
	int total_len, sent, this_try, n;

	total_len = num_blocks * block_size;

	sent = 0;

	while (sent < total_len) {
		this_try = total_len - sent;
		if (this_try > USER_BUFF_SIZE)
			this_try = block_size * (USER_BUFF_SIZE / block_size);

		n = write(fd, data + sent, this_try);
		if (n < 0) {
			perror("write");
			return n;
		}
		
		if (n < this_try)
			sleep(1);

		sent += n;
	}

	return 0;
}

unsigned char* prepare_data(int num_blocks, int block_size)
{
	int size_required, i, j;
	unsigned int delay;
	unsigned char bits;
	unsigned char *data;

	size_required = num_blocks * block_size;

	data = malloc(size_required);

	if (!data) {
		perror("malloc");
		return NULL;
	}

	// fake some data
	bits = 0x02;
	delay = 500;

	for (i = 0; i < size_required; i += block_size) {
		// the 4-byte delay
		*((unsigned int *)&data[i]) = delay;

		// the motor commands, 4 per byte
		for (j = i + 4; j < i + block_size; j++)
			data[j] = bits;


		// change the data a little		
		if (delay >= 1500)
			delay = 500;
		else
			delay += 100;

		
		if (bits > 0xf0)
			bits = 1;
		else
			bits = (bits << 1) | 0x02;
	}	

	return data;
}

void run_tests(int fd)
{
	int num_motors, thresh, i;
	int num_blocks, block_size;
	unsigned char *data;

	printf("Stopping the ecbsp device\n\n");	

	if (ioctl(fd, ECBSP_STOP, NULL) < 0) {
		perror("ioctl(ECBSP_STOP)");
		return;
	}


	num_motors = NUM_MOTORS;
	printf("Setting motors to %d\n", num_motors);

	if (ioctl(fd, ECBSP_WR_NUM_MOTORS, &num_motors) < 0) {
		perror("ioctl(ECBSP_WR_NUM_MOTORS)");
		return;
	}

	thresh = 4;
	printf("Setting queue threshold = %d\n", thresh);

	if (ioctl(fd, ECBSP_WR_QUEUE_THRESH, &thresh) < 0) {
		perror("ioctl(ECBSP_WR_QUEUE_THRESH)");
		return;
	}

	printf("\nStarting the ecbsp device\n");	

	if (ioctl(fd, ECBSP_START, NULL) < 0) {
		perror("ioctl(ECBSP_START)");
		return;
	}

	num_blocks = NUM_BLOCKS;
	block_size = 4 + (num_motors / 4);

	printf("You have ten seconds to setup the scope\n");

	sleep(10);

	data = prepare_data(num_blocks, block_size);
	if (!data)
		return;
	
	for (i = 0; i < 1; i++) {
		if (write_cmd_buff(fd, data, num_blocks, block_size)) {
			printf("Failed to write motor commands\n");
			break;
		}
	}

	free(data);
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


