ifneq ($(KERNELRELEASE),)
obj-m	:= multicomm.o

else
KDIR	:= ../../linux-2.6.7
PWD		:= $(shell pwd)

default: tests 
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules
	cp *.ko ../../bin

tests : 
	$(CC) -pthread -o mct1 test.c -lpthread
	$(CC) -o mct2 mct2.c
	$(CC) -o mct3 mct3.c
	$(CC) -pthread -g -o mct4 mct4.c -lpthread
	$(CC) -pthread -g -o mct5 mct5.c -lpthread
	
endif
	
clean:
	$(RM) -f multicomm.ko multicomm.o multicomm.mod.* .*.cmd 
	$(RM) -f mct1 mct2 mct3 mct4 mct5
	$(RM) -rf .tmp*
	
