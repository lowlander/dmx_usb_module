/*
 * $Id: dmx_usb_test.c 40 2004-09-11 11:16:39Z erwin $ 
 */

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>

static const char* dev = "/dev/dmx0";
static int level = 255;
static int test = 1;

static unsigned char buffer[513];
static int fd = -1;

static void open_dev(void)
{
	fd = open(dev, O_WRONLY);
	if (fd < 0) {
		perror("open");
		exit(-1);
	}
}

static void write_buffer(void)
{
	if (write(fd, buffer, 513) < 0) {
		perror("write");
		exit(-1);
	}
}

static void print_help(void)
{
	printf("\nDMX test\n");
	printf("-d <dev> - select device, default /dev/dmx0\n");
	printf("-l <level> - select level for tests, default 255\n");
	printf("-t <test> = select test nr, default 1\n");
	printf("\n");
	printf("Tests:\n");
	printf("1: set all channels to <level>\n");
	printf("2: set one moving channel to <level>\n");
	printf("3: fade all channels from 0 to <level>\n");
	printf("\n\n");

	exit(-1);
}

static void test_1(void)
{
	int i;

	for (i = 1; i < 513;i++) {
		buffer[i] = level;
	}

	while(1) {
		write_buffer();
	}
}

static void test_2(void)
{
	int i;
	int chan = 1;

	for (i = 1; i < 513;i++) {
		buffer[i] = 0;
	}

	while(1) {
		if (chan == 513)
			chan = 1;

		buffer[chan] = level;

		write_buffer();

		buffer[chan] = 0;

		chan++;
	}
}

static void test_3(void)
{
	int i;
	int l = 0;

	while(1) {
		for (i = 1; i < 513;i++) {
			buffer[i] = l;
		}

		write_buffer();

		l++;

		if (l > level)
			l = 0;
	}
}

int main(int argc, char* argv[])
{
	int opt;

	while ((opt = getopt(argc, argv, "hd:t:l:")) != -1) {
		switch(opt) {
		case 'd':
			dev = optarg;
			break;

		case 't':
			test = atoi(optarg);
			break;

		case 'l':
			level = atoi(optarg);
			break;

		case 'h':
		case '?':
			print_help();
			break;
		}
	}

	open_dev();

	/* the first byte has to be 0x00 for normal DMX */
	buffer[0] = 0x00;

	switch(test) {

	case 1: test_1(); break;
	case 2: test_2(); break;
	case 3: test_3(); break;

	default:
		print_help();
		break;
	}

	close(fd);

	return 0;
}

