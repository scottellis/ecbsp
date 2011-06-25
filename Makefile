# Makefile for the ecbsp kernel module

DRIVERNAME=ecbsp

ifneq ($(KERNELRELEASE),)
    obj-m := ${DRIVERNAME}.o
else
    PWD := $(shell pwd)

default:
ifeq ($(strip $(KERNELDIR)),)
	$(error "KERNELDIR is undefined!")
else
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules 
endif


install:
	sudo cp $(DRIVERNAME).ko /exports/overo/home/root

install_scp:
	scp $(DRIVERNAME).ko root@tide:/home/root


clean:
	rm -rf *~ *.ko *.o *.mod.c modules.order Module.symvers .${DRIVERNAME}* .tmp_versions

endif

