#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include "conio.h"

int kill = 0;

void *echo(void *arg)
{
    int            fd = (int)arg;
	char           ch;
    int            count;
    
    while(!kill) {
        count = read(fd,&ch,1);
        if (count != 1) {
            printf("read error fd = %d\n",fd);
            perror("read");
            break;
        }
        count = write(fd,&ch,1);
        if (count != 1) {
            printf("write error fd = %d\n",fd);
            perror("write");
            break;
        }
    }
}

const int TTYS = 1;
int main(int argc,char *argv[])
{
	int           fd[TTYS];
    pthread_t     et[TTYS];
    char          dev[128];
    char          line[128];
    int           i;
    int           count;
    unsigned char ch;
	
	if (argc != 1) {
		printf("mctest\n");
		exit(0);
	}
	
    for(i=0;i<TTYS;i++) {
        sprintf(dev,"/dev/mc%d",i);
        fd[i] = open(dev,O_RDWR);
        if (fd[i] < 0) {
            sprintf(line,"can't open port %s",dev);
            perror(line);
            exit(1);
        }
    }
	
    ch = 'a';
    while(!kbhit()) {
        for(i=0;i<TTYS;i++) {
            count = write(fd[i],&ch,1);          
            if (count != 1) {
                sprintf(line,"error writing to port %s",i);
                perror(line);
                exit(1);
            }
        }
        ch++;
        if (ch > 'z') {
           ch = 'a';
        }
        sleep(1);
    }
		
	for(i=0;i<TTYS;i++) {
		close(fd[i]);
	}
	
	return 0;
}

