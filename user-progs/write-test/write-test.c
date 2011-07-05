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

// one ROW represents all the commands for NUM_MOTORS_PER_ROW + a delay
#define NUM_ROWS 16

#define NUM_MOTORS_PER_ROW 16

const char device_name[] = "/dev/ecbsp";

int write_cmd_buff(int fd, unsigned char *data, int num_rows, int row_size)
{
	int total_len, sent, this_try, n;

	total_len = num_rows * row_size;

	sent = 0;

	while (sent < total_len) {
		this_try = total_len - sent;

		// we try to send as many complete rows as we can in each write
		if (this_try > USER_BUFF_SIZE)
			this_try = row_size * (USER_BUFF_SIZE / row_size);

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

unsigned char* prepare_data(int num_rows, int row_size)
{
	int size_required, i;
	unsigned int delay;
	unsigned char *data;

	size_required = num_rows * row_size;

	data = malloc(size_required);

	if (!data) {
		perror("malloc");
		return NULL;
	}

	memset(data, 0, size_required);

	// fake some delay data
	delay = 200;

	for (i = 0; i < size_required; i += row_size) {
		// no delay for first block
		if (i > 0)
			// the 4-byte delay
			*((unsigned int *)&data[i]) = delay;

		// set the data here, all zeros for now

		// change delay a little for each block
		if (delay >= 1000)
			delay = 300;
		else
			delay += 100;
	}	

	return data;
}

void run_tests(int fd)
{
	int num_motors_per_row, thresh, i;
	int num_rows, row_size;
	unsigned char *data;

	printf("Stopping the ecbsp device\n\n");	

	if (ioctl(fd, ECBSP_STOP, NULL) < 0) {
		perror("ioctl(ECBSP_STOP)");
		return;
	}


	num_motors_per_row = NUM_MOTORS_PER_ROW;
	printf("Setting num motors per row to %d\n", num_motors_per_row);

	if (ioctl(fd, ECBSP_WR_MOTORS_PER_ROW, &num_motors_per_row) < 0) {
		perror("ioctl(ECBSP_WR_MOTORS_PER_ROW)");
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

	num_rows = NUM_ROWS;
	row_size = 4 + (num_motors_per_row / 4);

	printf("You have ten seconds to setup the scope\n");

	sleep(10);

	data = prepare_data(num_rows, row_size);
	if (!data)
		return;
	
	for (i = 0; i < 1; i++) {
		if (write_cmd_buff(fd, data, num_rows, row_size)) {
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


