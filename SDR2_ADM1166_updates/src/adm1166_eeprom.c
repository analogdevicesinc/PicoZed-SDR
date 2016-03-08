/*
 * Copyright (C) 2015-2016 Analog Devices, Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * */

#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <malloc.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>

struct ihex_chunk {
	struct ihex_chunk *next;
	unsigned short addr;
	unsigned char len;
	unsigned char checksum;
	unsigned char data[];
};

struct ihex_file {
	struct ihex_chunk *first;
	struct ihex_chunk *last;
};

enum state {
	STATE_START_OF_LINE,
	STATE_LINE_LENGTH,
	STATE_ADDRESS,
	STATE_TYPE,
	STATE_DATA,
	STATE_CHECKSUM,
	STATE_END_OF_FILE,
	STATE_ERROR,
	STATE_DONE,
};

static int get_token(int fd)
{
	unsigned char c;
	int ret;

	ret = read(fd, &c, 1);
	if (ret <= 0)
		return -1;
	return c;
}

static int get_hex_token(int fd, unsigned int len, unsigned int *val)
{
	unsigned int i;
	int c;

	*val = 0;

	for (i = 0; i < len; i++) {
		c = get_token(fd);
		if (c < 0)
			return c;
		*val <<= 4;
		if ((c >= '0' && c <= '9')) {
			*val |= c - '0';
		} else if (c >= 'A' && c <= 'F') {
			*val |= c - 'A' + 10;
		} else {
			fprintf(stderr, "Unexpected character: %x\n", c);
			return -1;
		}
	}

	return 0;
}

int parse_ihex(int fd, struct ihex_file *file)
{
	enum state state = STATE_START_OF_LINE;
	struct ihex_chunk *chunk = NULL;
	unsigned int tmp;
	unsigned int i;
	int c;
	int ret;

	file->first = NULL;
	file->last = NULL;

	while (state != STATE_ERROR && state != STATE_DONE) {
		switch (state) {
		case STATE_START_OF_LINE:
			c = get_token(fd);
			switch (c) {
			case '\n':
			case '\r':
			case '\t':
			case ' ':
				break;
			case ':':
				state = STATE_LINE_LENGTH;
				break;
			default:
				fprintf(stderr, "Unexpected character: %x\n", c);
				state = STATE_ERROR;
				break;
			}
			break;
		case STATE_LINE_LENGTH:
			ret = get_hex_token(fd, 2, &tmp);
			if (ret < 0) {
				state = STATE_ERROR;
			} else {
				chunk = malloc(sizeof(*chunk) + tmp);
				memset(chunk, 0x00, sizeof(*chunk) + tmp);
				chunk->len = tmp;
				state = STATE_ADDRESS;
			}
			break;
		case STATE_ADDRESS:
			ret = get_hex_token(fd, 4, &tmp);
			if (ret < 0) {
				state = STATE_ERROR;
			} else {
				chunk->addr = tmp;
				state = STATE_TYPE;
			}
			break;
		case STATE_TYPE:
			ret = get_hex_token(fd, 2, &tmp);
			if (ret < 0) {
				state = STATE_ERROR;
			} else if (tmp == 0) {
				state = STATE_DATA;
			} else if (tmp == 1) {
				state = STATE_END_OF_FILE;
			}
			break;
		case STATE_DATA:
			state = STATE_CHECKSUM;
			for (i = 0; i < chunk->len; i++) {
				ret = get_hex_token(fd, 2, &tmp);
				if (ret < 0) {
					state = STATE_ERROR;
					break;
				}
				chunk->data[i] = tmp;
			}
			break;
		case STATE_CHECKSUM:
			ret = get_hex_token(fd, 2, &tmp);
			if (ret < 0) {
				state = STATE_ERROR;
			} else {
				state = STATE_START_OF_LINE;
				chunk->checksum = tmp;
				if (file->last)
					file->last->next = chunk;
				else
					file->first = chunk;
				file->last = chunk;
				chunk = NULL;	
			}
			break;
		case STATE_END_OF_FILE:
			ret = get_hex_token(fd, 1, &tmp);
			if (ret < 0 || tmp != 0xff)
				state = STATE_ERROR;
			else
				state = STATE_DONE;
			break;
		default:
			break;
		}
	}
	if (chunk)
		free(chunk);

	if (state != STATE_DONE)
		return -1;
	return 0;
}

#define ADM_PAGE_SIZE 0x20

static int adm_eeprom_erase(int fd, unsigned int addr)
{
	unsigned char buf[2] = {0xf0, 0x00};
	int ret;

	buf[0] = (addr >> 8) & 0xff;
	buf[1] = addr & 0xff;

	ret = write(fd, buf, 2);
	if (ret != 2) {
		fprintf(stderr, "%s step 1 failed: %d, %x\n", __func__, errno, addr);
		return -errno;
	}

	buf[0] = 0xfe;

	ret = write(fd, buf, 1);
	if (ret != 1) {
		fprintf(stderr, "%s step 2 failed: %d, %x\n", __func__, errno, addr);
		return -errno;
	}

	return 0;
}

static int adm_eeprom_read(int fd, unsigned int addr, unsigned char *rbuf)
{
	unsigned char buf[33] = {0xf0, 0x00};
	struct i2c_msg msg[2];
	struct i2c_rdwr_ioctl_data xfer;
	int ret;

	buf[0] = (addr >> 8) & 0xff;
	buf[1] = addr & 0xff;
	ret = write(fd, buf, 2);
	if (ret != 2) {
		fprintf(stderr, "%s step 1 failed: %d, %x\n", __func__, errno, addr);
		return -errno;
	}

	buf[0] = 0xfd;

	msg[0].addr = 0x34;
	msg[0].flags = 0x00;
	msg[0].len = 1;
	msg[0].buf = buf;
	msg[1].addr = 0x34;
	msg[1].flags = I2C_M_RD;
	msg[1].len = ADM_PAGE_SIZE + 1;
	msg[1].buf = buf;
	xfer.msgs = msg;
	xfer.nmsgs = 2;

	ret = ioctl(fd, I2C_RDWR, &xfer);
	if (ret != 2) {
		fprintf(stderr, "%s step 2 failed: %d, %x\n", __func__, errno, addr);
		return -errno;
	}

	if (buf[0] != ADM_PAGE_SIZE) {
		fprintf(stderr, "%s step 3 failed: %d, %x\n", __func__, errno, addr);
		return -1;
	}

	memcpy(rbuf, buf + 1, ADM_PAGE_SIZE);

	return 0;
}

static int adm_eeprom_write(int fd, unsigned int addr, unsigned char *wbuf)
{
	unsigned char buf[34] = {0xf0, 0x00};
	int ret;

	buf[0] = (addr >> 8) & 0xff;
	buf[1] = addr & 0xff;

	ret = write(fd, buf, 2);
	if (ret != 2) {
		fprintf(stderr, "%s step1 failed: %d, %x\n", __func__, errno, addr);
		return -errno;
	}

	buf[0] = 0xfc;
	buf[1] = ADM_PAGE_SIZE;
	memcpy(buf+2, wbuf, ADM_PAGE_SIZE);

	ret = write(fd, buf, ADM_PAGE_SIZE + 2);
	if (ret < 0) {
		fprintf(stderr, "%s step 2 failed: %d, %x\n", __func__, errno, addr);
		return -errno;
	}

	return 0;
}

static int program_chunk(int i2c_fd, struct ihex_chunk *chunk)
{
	unsigned char rbuf[ADM_PAGE_SIZE];
	unsigned char wbuf[ADM_PAGE_SIZE];
	int ret;

	printf("Reading %4x ... ", chunk->addr);

	ret = adm_eeprom_read(i2c_fd, chunk->addr, rbuf);
	if (ret == 0) {
		printf("success\n");
	} else {
		printf("failed\n");
		return -1;
	}

	memcpy(wbuf, chunk->data, 0x10);
	memcpy(wbuf + 0x10, chunk->next->data, 0x10);

	if (memcmp(wbuf, rbuf, ADM_PAGE_SIZE) == 0) {
		printf(" ... existing memory is identical.\n");
		return 0;
	} else {
		int i;
		for (i = 0; i < 32; i++)
			printf("%.2x%c", wbuf[i], (i % 16 == 15) ? '\n' : ' ');
		printf("\n");
		for (i = 0; i < 32; i++)
			printf("%.2x%c", rbuf[i], (i % 16 == 15) ? '\n' : ' ');
		printf("\n");
	}

	printf ("Erasing %4x ... ", chunk->addr);

	ret = adm_eeprom_erase(i2c_fd, chunk->addr);
	if (ret == 0) {
		printf("success\n");
	} else {
		printf("failed\n");
		return -1;
	}
	sleep(1);

	printf("Writing %4x ...", chunk->addr);

	ret = adm_eeprom_write(i2c_fd, chunk->addr, wbuf);
	if (ret == 0) {
		printf("success\n");
	} else {
		printf("failed\n");
		return -1;
	}
	sleep(1);

	printf("Verifying %4x ...", chunk->addr);

	ret = adm_eeprom_read(i2c_fd, chunk->addr, rbuf);
	if (ret == 0) {
		int i;
		for (i = 0; i < 32; i++)
			printf("%.2x%c", wbuf[i], (i % 16 == 15) ? '\n' : ' ');
		printf("\n");
		for (i = 0; i < 32; i++)
			printf("%.2x%c", rbuf[i], (i % 16 == 15) ? '\n' : ' ');
		printf("\n");
		if (memcmp(rbuf, wbuf, ADM_PAGE_SIZE) != 0)
			ret = -1;
	}
	if (ret == 0) {
		printf("success\n");
	} else {
		printf("failed\n");
		return -1;
	}

	return 0;
}

int main(int argc, char *argv[])
{
	struct ihex_file ihex_file;
	struct ihex_chunk *chunk;
	unsigned int addr = 0xf800;
	unsigned int retry = 0;
	unsigned char buf[2];
	int fd, i2c_fd;
	int ret;

	fd = open(argv[1], 0);
	if (fd < 0) {
		perror("Failed to open file");
		exit(1);
	}
	ret = parse_ihex(fd, &ihex_file);
	close(fd);

	if (ret) {
		printf("Failed to parse ihex file \"%s\". Aborting.\n", argv[1]);
		exit(1);
	}

	for (chunk = ihex_file.first; chunk; chunk = chunk->next) {
		if (chunk->len != 0x10 || chunk->addr != addr) {
			fprintf(stderr, "Invalid file\n");
			exit(1);
		}
		if ((chunk->addr % 0x20) == 0 && chunk->next == NULL) {
			fprintf(stderr, "Invalid file\n");
			exit(1);
		}
		addr += 0x10;
	}

	i2c_fd = open("/dev/i2c-0", O_RDWR);
	if (i2c_fd < 0) {
		perror("Failed to open /dev/i2c-0");
		exit(1);
	}

	ret = ioctl(i2c_fd, I2C_SLAVE, 0x34);
	if (ret < 0) {
		perror("Failed to set I2C device address");
		exit(1);
	}

	/* Halt sequencing engine */
	buf[0] = 0x93;
	buf[1] = 0x1;
	write(i2c_fd, buf, 2);
	/* Enable EEPROM access */
	buf[0] = 0x90;
	buf[1] = 0x5;
	write(i2c_fd, buf, 2);

	printf("Starting to reprogramm the AD1166 EEPROM.\n");

	for (chunk = ihex_file.first; chunk; chunk = chunk->next->next) {
		if ((chunk->addr >= 0xF8A0 && chunk->addr < 0xF900) ||
		    (chunk->addr >= 0xF9A0 && chunk->addr < 0xFA00) ||
		    (chunk->addr >= 0xFAA0 && chunk->addr < 0xFB00) ||
		    (chunk->addr >= 0xFBA0 && chunk->addr < 0xFC00)) {
			printf("Skipping reserved page %x\n", chunk->addr);
			continue;
		}
		retry = 0;
		do {
			if (retry != 0)
				printf("Failed to program page %x, retry (%d).\n", chunk->addr,
					retry);
			retry++;
			ret = program_chunk(i2c_fd, chunk);
		} while (ret != 0 && retry < 3);

		if (ret != 0)
			break;
	}

	/* Back to normal mode */
	buf[0] = 0x90;
	buf[1] = 0x0;
	write(fd, buf, 2);

	if (ret == 0) {
		printf("Successfully reprogrammed the ADM1166 EEPROM.\n");
		printf(" ... reboot the board to load the new configuration.\n");
	} else {
		printf("!!! Re-programming the ADM1166 EEPROM failed.  !!!\n");
		printf("!!! Operation of the board may become unstable !!!\n");
		printf("!!! turn the board off immediately and         !!!\n");
		printf("!!! re-program the ADM1166 using a dedicated   !!!\n");
		printf("!!! external programmer.                       !!!\n");
	}

	return 0;
}
