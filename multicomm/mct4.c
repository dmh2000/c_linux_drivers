#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>

#include "conio.h"
#include "multicomm.h"

// priorities range from 1 to 99, with 99 being the highest
unsigned long createThread(void *(*f)(void *),void *arg,unsigned long stacksize) 
{
	pthread_t      thread;
	pthread_attr_t attr;
	struct sched_param    param;
	int            status;
	
	// init to defaults
	status = pthread_attr_init(&attr);
	if (status != 0) {
        printf("%s:%d\n",__FILE__,__LINE__);
		return 0;
	}
		
	// set stack size 
	status = pthread_attr_setstacksize(&attr,stacksize);
	if (status != 0) {
        printf("%s:%d\n",__FILE__,__LINE__);
		return 0;
	}
	
	status = pthread_create(&thread,&attr,f,arg);
	if (status != 0) {
        printf("%s:%d\n",__FILE__,__LINE__);
		return 0;
	}
	
	return thread;
}

unsigned long tcount = 0;
void *tx(void *arg)
{
    int fd = (int)arg;
    unsigned char line[64];
    unsigned char ch;
    int           count;
    int           i;

    for(;;) {
        ch = ' ';    
        for(i=0;i<sizeof(line);i++) {
            line[i] = ch++;
        }
        count = write(fd,line,sizeof(line));
        if (count != sizeof(line)) {
            printf("write error %d %d\n",fd,count);
        }
        tcount += count;
        usleep(1000 * 100);
    }
	
	return 0;
}

unsigned long rcount = 0;
void *rx(void *arg)
{
    int fd = (int)arg;
    unsigned char ch;
    int           count;

    rcount = 0;
    for(;;) {
        count = read(fd,&ch,1);
        if (count != 1) {
            printf("read error %d %d\n",fd,count);
        }
        rcount++;
    }
	
	return 0;
}

int main(int argc,char *argv[])
{
    int  fd;
    char dev[64];
    int  i;
    ser_stats_t stats;
    int status;
    
    if (argc <= 1) {
        printf("mct4 <device>\n");
        return -1;
    }
    
    printf("opening %s\n",argv[1]);
    fd = open(argv[1],O_RDWR);
    if (fd < 0) {
        perror("can't open port");
        exit(1);
    }
    createThread(tx,(void *)fd,65536);
    createThread(rx,(void *)fd,65536);
    
    printf("rcount tcount   isr rxchr txchr rxisr txisr erisr rxovr rxbuf txovr txbuf\n");
    for(;;) {
        usleep(1000 * 500);
        status = ioctl(fd,SER_IOCTL_STATS,&stats);
        printf("%6lu %6lu %5lu %5lu %5lu %5lu %5lu %5lu %5lu %5lu %5lu %5lu\n",
            rcount,
            tcount,
            stats.isr,
            stats.rxchr,
            stats.txchr,
            stats.rxisr,
            stats.txisr,
            stats.errisr,
            stats.rxovr,
            stats.rxbuf,
            stats.txovr,
            stats.txbuf
        );
    }
	
	return 0;
}

