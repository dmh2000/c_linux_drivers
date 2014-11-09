#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "can.h"

const char *device = "/dev/sja10000";

void printstats(can_stats_t *stats)
{
    // print statistics
    printf("isr:%lu rxi:%lu rxp:%lu txi:%lu txp:%lu txs:%lu eri:%lu ovi:%lu txb:%lu bof:%lu ect:%lu\n",
            stats->isr,
            stats->rxisr,
            stats->rxpkt,
            stats->txisr,
            stats->txpkt,
            stats->txsnt,
            stats->txblck,
            stats->errisr,
            stats->ovrisr,
            stats->busoff,
            stats->errcnt
            );
}

int main(int argc,char *argv[])
{
	int            fd;
    ssize_t        count;
	can_stats_t    stats;
    int            status;
    
	fd = open(device,O_RDWR);
	if (fd == -1) {
		perror(device);
		exit(1);
	}
    
    status = ioctl(fd,CAN_IOCTL_STATS,&stats);
    if (status == 0) {
        printstats(&stats);
    }
    
    close(fd);

    return 0;
}

