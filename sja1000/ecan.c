#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <pthread.h>
#include "conio.h"

const char *can_ver   = "D ver\r\n";
const char *can_init  = "C init 1000\r\n";
const char *can_start = "C start\r\n"; 
const char *can_filt  = "C filter disable\r\n"; 

void get_msg(int sock)
{
    char ch;
    int  status;
    char buf[128];
    int  i;
    
    memset(buf,0,sizeof(buf));
    i = 0;
    for(;;) {
        status = read(sock,&ch,1);
        if (status <= 0) {
            break;
        }
        buf[i] = ch;
        i++;
        if (ch == '\n') {
            break;
        }
    }
    printf("RX : %s",buf);
}

void snd_msg(int sock,const char *msg)
{
    int status;
    status = write(sock,msg,strlen(msg));
    if (status <= 0) {
        perror(msg);
        close(sock);
        exit(1);
    }
}

void snd_data(int sock,int id,int len,unsigned char d[8])
{
    int status;
    char buf[128];
    int i;
    int count;
    count = sprintf(buf,"M SD%d %08x",len,id);
    for(i=0;i<len;i++) {
        count += sprintf(&buf[count]," %02x",d[i]);
    }
    count += sprintf(&buf[count]," \r\n");

    printf("TX : %s",buf);

    status = write(sock,buf,strlen(buf));
    if (status <= 0) {
        perror(buf);
        close(sock);
        exit(1);
    }
}


static int kill = 0;

void *rx_thread(void *arg)
{
    int sock = (int)arg;
    
    while(kill == 0) {
        get_msg(sock);
    }
    return 0;
}

void *tx_thread(void *arg)
{
    int sock = (int)arg;
    unsigned char data[8];
    int i = 0;
    
    while(kill == 0) {
        memset(data,i,sizeof(data));
        snd_data(sock,0x100,8,data);
        sleep(1);
        i++;
    }
    return 0;
}

int main(int argc,char *argv[])
{
	int sock;
    int status;
    int i;
    int j;
    struct sockaddr_in dest;
    unsigned char data[8];
    pthread_t      rxt;
    pthread_t      txt;
	
	sock = socket(AF_INET,SOCK_STREAM,0);
	if (sock == -1) {
		perror("socket");
        return -1;
    }
    
    memset(&dest,0,sizeof(dest));
    dest.sin_addr.s_addr   = inet_addr("10.6.173.21");
    dest.sin_port          = htons(19227);
    dest.sin_family        = AF_INET;
    
    status = connect(sock,(struct sockaddr *)&dest,sizeof(dest));
    if (status != 0) {
        perror("connect");
        close(sock);
        return -1;
    }
    
    printf("connected\n");

    // perform the initialization sequence
    snd_msg(sock,can_ver);
    get_msg(sock);
    snd_msg(sock,can_init);
    get_msg(sock);
    snd_msg(sock,can_start);
    get_msg(sock);
    snd_msg(sock,can_filt);
    get_msg(sock);
    
    pthread_create(&rxt,0,rx_thread,(void *)sock);
    pthread_create(&txt,0,tx_thread,(void *)sock);
    
    while(!kbhit()) {
        sleep(1);
    }
    kill = 1;
    
    pthread_join(txt,0);
    
    close(sock);

	return 0;
}

