ifneq ($(KERNELRELEASE),)
obj-m	:= can82527.o

else
#KDIR	:= /lib/modules/$(shell uname -r)/build
KDIR	:= ../../linux-2.6.7
PWD		:= $(shell pwd)

default:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules
endif
	

test : test.c
	$(CC) -o test test.c
	
clean:
	rm -f can82527.ko can82527.o can82527.mod.*
	
