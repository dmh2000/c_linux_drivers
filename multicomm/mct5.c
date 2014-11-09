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
    
    printf("  isr rxchr txchr rxisr txisr erisr rxovr rxbuf txovr txbuf\n");
    for(;;) {
        usleep(1000 * 500);
        status = ioctl(fd,SER_IOCTL_STATS,&stats);
        printf("%5lu %5lu %5lu %5lu %5lu %5lu %5lu %5lu %5lu %5lu\n",
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

