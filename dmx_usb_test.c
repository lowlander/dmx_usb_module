/*
 * $Id: dmx_usb_test.c 40 2004-09-11 11:16:39Z erwin $ 
 */

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>

int main(int argc, char* argv[])
{
	unsigned char buffer[513];
	int fd;
	int res;
	int i;

	fd = open("/dev/dmx0",O_WRONLY);
	if (fd < 0) {
		perror("open");
		exit(-1);
	}

	for (i = 0; i < 513;i++) {
		buffer[i] = 0x00;	
	}

	buffer[512] = 0x55;
	
	while(1) {
		res = write(fd, buffer, 513);

		if (res < 0){
			perror("write");
			exit(-1);
		}
	}

	close(fd);

	return 0;
}

