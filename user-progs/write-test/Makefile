ifeq ($(strip $(OETMP)),)
	OETMP = ${OVEROTOP}/tmp
endif

TOOLDIR = $(OETMP)/sysroots/`uname -m`-linux/usr/armv7a/bin
STAGEDIR = ${OETMP}/sysroots/armv7a-angstrom-linux-gnueabi/usr

CC = ${TOOLDIR}/arm-angstrom-linux-gnueabi-gcc
CFLAGS = -Wall

LIBDIR = $(STAGEDIR)/lib
INCDIR = $(STAGEDIR)/include
DRVDIR = ../../../ecbsp

TARGET = write-test

$(TARGET) : write-test.c
	$(CC) $(CFLAGS) -I $(INCDIR) -I $(DRVDIR) -L $(LIBDIR) write-test.c -o $(TARGET)

install:
	scp $(TARGET) root@tide:/home/root


clean :
	rm -f $(TARGET)

