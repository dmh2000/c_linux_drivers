#ifndef __CANQUEUE_H__
#define __CANQUEUE_H__

#include "can.h"

// max messages in can buffers
#define CAN_MSGQ_COUNT     64

/***************************************************************************************
 * io circular byte buffer functions
 ***************************************************************************************/
typedef struct can_msg_queue_t {
    can_msg_t  buf[CAN_MSGQ_COUNT];
    int        head;  // remove from here
    int        tail;  // add here
    int        count; // number of available bytes
} can_msg_queue_t;    

/**
 * initialize the queue
 */
static inline void can_init_queue(can_msg_queue_t *q)
{
    memset(q->buf,0,sizeof(q->buf));
    q->tail  = 0;
    q->head  = 0;
    q->count = 0;
}

/**
 * always get in units of 1 CAN message
 * @param q    message queue to use
 * @param msg  message buffer to fill
 * @param lock spinlock to use. if 0, skip locking 
 * @return nonzero if a message is received, 0 if no message was available
 */
static inline int can_get_msg(can_msg_queue_t *q,can_msg_t *msg,spinlock_t *lock) 
{
    int           status;
    unsigned long flags = 0;
    
    if (lock != 0) {
        // lock interrupts while checking status
        spin_lock_irqsave(lock,flags);
    }
    
    if (q->count > 0) {
        // get from head
        *msg = q->buf[q->head];
        // increment head with wrap
        q->head++;
        if (q->head >= CAN_MSGQ_COUNT) {
            q->head = 0;
        }
		
        // decrement count
        q->count--;
		
        // ok
        status = sizeof(can_msg_t);
    }
    else {
        // no data
        status = 0;
    }
    
    if (lock != 0) {
        // lock interrupts while checking status
        spin_unlock_irqrestore(lock,flags);
    }
    return status;
}

/**
 * always put 1 complete can message
 * @param q    message queue to use
 * @param msg  message to insert
 * @param lock spinlock to use. if 0, skip locking 
 * @return number of bytes if message is written, 0 if buffer was full
 */
static inline int can_put_msg(can_msg_queue_t *q,can_msg_t *msg,spinlock_t *lock) 
{
    int           status;
    unsigned long flags = 0;
    
    if (lock != 0) {
        // lock interrupts while checking status
        spin_lock_irqsave(lock,flags);
    }
    
    if (q->count < CAN_MSGQ_COUNT) {
        // add at tail
        q->buf[q->tail] = *msg;
		
        // increment tail with wrap
        q->tail++;
        if (q->tail >= CAN_MSGQ_COUNT) {
            q->tail = 0;
        }
        // increment count
        q->count++;
		
        // ok
        status = sizeof(can_msg_t);
    }
    else {
        // can't insert
        status = 0;
    }
    
    if (lock != 0) {
        // lock interrupts while checking status
        spin_unlock_irqrestore(lock,flags);
    }
    
    return status;
}

/**
 * @return number of available messages
 */
static inline int  can_has_msg(can_msg_queue_t *q)
{
    return q->count;
}

#endif // __CANQUEUE_H__

