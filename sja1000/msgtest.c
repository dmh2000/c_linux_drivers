#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// dummy stuff to keep it happy
typedef int spinlock_t;
#define spin_lock_irqsave(lock,flags) 
#define spin_unlock_irqrestore(lock,flags)

#include "can.h"
#include "canqueue.h"


int main(int argc,char *argv[])
{
    can_msg_t       m;
    can_msg_queue_t q;
    int             status;
    int             i;
    
    can_init_queue(&q);
    
    for(i=0;i<72;i++) {
        m.id = i;
        status = can_put_msg(&q,&m,0);
        printf("put %d %d %d\n",i,m.id,status);
    }
    
    for(i=0;i<72;i++) {
        status = can_get_msg(&q,&m,0);
        printf("get %d %d %d\n",i,m.id,status);
    };

    for(i=0;i<64;i++) {
        m.id = i;
        status = can_put_msg(&q,&m,0);
        printf("put %d %d %d\n",i,m.id,status);
    }
    for(i=0;i<4;i++) {
        status = can_get_msg(&q,&m,0);
        printf("get %d %d %d\n",i,m.id,status);
    }
    for(i=0;i<8;i++) {
        m.id = i + 100;
        status = can_put_msg(&q,&m,0);
        printf("put %d %d %d\n",i,m.id,status);
    }
    do {
        status = can_get_msg(&q,&m,0);
        printf("get %d %d %d\n",i,m.id,status);
    } while(status > 0);
    
    return 0;
}

