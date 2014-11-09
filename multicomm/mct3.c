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


int main(int argc,char *argv[])
{
	int           fd;
    char          dev[128];
    char          line[128];
    int           i;
    int           count;
    unsigned char ch;
	
	if (argc != 2) {
		printf("mct2 <port>\n");
		exit(0);
	}
	
    fd = open(argv[1],O_RDWR);
    if (fd < 0) {
        sprintf(line,"can't open port %s",argv[1]);
        perror(line);
        exit(1);
    }
	
    ch = 'a';
    while(!kbhit()) {
        count = read(fd,&ch,1);
        if (count != 1) {
            printf("read error fd = %d\n",fd);
            perror("read");
            break;
        }
        printf("%c\n",ch);
        count = write(fd,&ch,1);
        if (count != 1) {
            printf("write error fd = %d\n",fd);
            perror("write");
            break;
        }
    }
		
    close(fd);
	
	return 0;
}

