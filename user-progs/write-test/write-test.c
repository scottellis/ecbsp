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

#define NUM_COMMANDS 1024
#define NUM_MOTORS 2048

const char device_name[] = "/dev/ecbsp";

int write_cmd_buff(int fd, unsigned char *data, int total_len, int cmd_len)
{
	int sent, this_try, n;

	sent = 0;

	while (sent < total_len) {
		this_try = total_len - sent;
		if (this_try > 4096)
			this_try = cmd_len * (4096 / cmd_len);

		n = write(fd, data + sent, this_try);
		if (n < 0) {
			perror("write");
			return n;
		}

		sent += n;
	}

	return 0;
}

unsigned char* prepare_data(int num_cmds, int num_motors, int *total_size)
{
	int size_required, size_of_one_cmd, i, j;
	unsigned char bits;
	unsigned char *data;

	size_of_one_cmd = 4 + (num_motors / 4);

	size_required = num_cmds * size_of_one_cmd;

	data = malloc(size_required);

	if (!data) {
		perror("malloc");
		return NULL;
	}

	bits = 0x02;

	for (i = 0; i < size_required; i += size_of_one_cmd) {
		// the 4-byte delay
		*((unsigned int *)&data[i]) = 500;
		
		// the motor commands, 4 per byte
		for (j = i + 4; j < i + size_of_one_cmd; j++) {
			data[j] = bits;
		}

		
		if (bits > 0xf0)
			bits = 1;
		else
			bits = (bits << 1) | 0x02;
	}	

	*total_size = size_required;

	return data;
}

void run_tests(int fd)
{
	int num_motors, thresh, total_size;
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


	printf("You have ten seconds to setup the scope\n");

	sleep(10);

	data = prepare_data(NUM_COMMANDS, num_motors, &total_size);
	if (!data)
		return;
	
	if (write_cmd_buff(fd, data, total_size, 4 + (num_motors / 4)))
		printf("Failed to write motor commands\n");

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


