ifneq ($(KERNELRELEASE),)
obj-m	:= sja1000.o

else
#KDIR	:= /lib/modules/$(shell uname -r)/build
KDIR	:= ../../linux-2.6.7
PWD		:= $(shell pwd)

default: tests
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules
	cp *.ko ../../bin
	
endif
	
tests : stats msgtest ecan test2

test2 : test2.c
	$(CC) -g -O0 -o test2 test2.c -lpthread
	
stats : stats.c
	$(CC) -g -O0 -o stats stats.c
	
msgtest : msgtest.c
	$(CC) -g -O0 -o msgtest msgtest.c
	
ecan : ecan.c
	$(CC) -g -O0 -o ecan ecan.c -lpthread
	
clean:
	rm -f sja1000.ko sja1000.o sja1000.mod.* .*.cmd test2 stats msgtest ecan
	rm -rf .tmp*
	
