#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "can.h"
#include "conio.h"
const char *device = "/dev/sja10000";

void printmsg(const char *hdr,can_msg_t *msg)
{
    int k;
    
    printf("%s : %08x : %08x : ",hdr,msg->id,msg->len);
    for(k=0;k<msg->len;k++) {
        printf("%02x ",msg->data[k]);
    }
    printf("\n");
}

static int kill = 0;

void *rx_thread(void *arg)
{
    can_msg_t msg;
    int       count;
    int       fd = (int)arg;
    
    while(kill == 0) {
        // read a message
        count = read(fd,&msg,sizeof(msg));
        if (count != sizeof(msg)) {
            fprintf(stderr,"bad read count %d\n",count);
        }
        printmsg("RX",&msg);
        if (msg.id == 0x80000000) {
            printf("transmit bus off error\n");
            kill = 1;
        }
    }
    
    return 0;
}

void *tx_thread(void *arg)
{
    can_msg_t msg;
    int       count;
    int       i;
    int       fd = (int)arg;
    
    i = 0;
    while(kill == 0) {
        msg.id  = 0x80;
        msg.len = 8;
        memset(msg.data,i,8);
        // write a message
        count = write(fd,&msg,sizeof(msg));
        if (count != sizeof(msg)) {
            fprintf(stderr,"bad write count %d\n",count);
        }
        printmsg("TX",&msg);
        sleep(1);
        i++;
    }
    
    return 0;
}


int main(int argc,char *argv[])
{
	int            fd;
    pthread_t      rxt;
    pthread_t      txt;
    
	fd = open(device,O_RDWR);
	if (fd == -1) {
		perror(device);
		exit(1);
	}
    
    pthread_create(&rxt,0,rx_thread,(void *)fd);
    pthread_create(&txt,0,tx_thread,(void *)fd);
    
    while(!kbhit()) {
        sleep(1);
    }
    kill = 1;
    
    pthread_join(rxt,0);
    pthread_join(txt,0);
    
    close(fd);

    return 0;
}

