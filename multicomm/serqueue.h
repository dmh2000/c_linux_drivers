#ifndef __SERQUEUE_H__
#define __SERQUEUE_H__

// max messages in can buffers
#define SER_BUFFER_SIZE 1024

/***************************************************************************************
 * io circular byte buffer functions
 ***************************************************************************************/
typedef struct ser_buffer_t {
    unsigned char buf[SER_BUFFER_SIZE];
    int           head;  // remove from here
    int           tail;  // add here
    int           count; // number of available bytes
} ser_buffer_t;    

/**
 * initialize the queue
 */
static inline void ser_init_queue(ser_buffer_t *q)
{
    memset(q->buf,0,sizeof(q->buf));
    q->tail  = 0;
    q->head  = 0;
    q->count = 0;
}

/**
 * always get in units of 1 CAN message
 * @param q    message queue to use
 * @param buf  buffer to fill
 * @param len  size of buffer
 * @param lock spinlock to use. if 0, skip locking 
 * @return     number of bytes retrieved
 */
static inline int ser_get(ser_buffer_t *q,unsigned char *buf,int len,spinlock_t *lock) 
{
    int           status;
    unsigned long flags = 0;
	unsigned char *p;
	int            i;
	int            count = len;
    int            done;
    
	p      = buf;
	status = 0;
	done   = 0;
	for(i=0;i<count;i++) {
		// lock around each 'get' from the circular buffer
		// this reduces the lock time span but increases the overhead
		if (lock != 0) {
			spin_lock_irqsave(lock,flags);
		}
		
		if (q->count > 0) {
			// get from head
			*p = q->buf[q->head];
			
			// next buffer position
			p++;
			
			// increment head with wrap
			q->head++;
			if (q->head >= SER_BUFFER_SIZE) {
				q->head = 0;
			}
			
			// decrement count
			q->count--;
			
			// ok
			status++;
		}
		else {
			// no more data, return what is there
			// can't read more
			done = 1;
		}
		
		// must match locks with unlocks
		if (lock != 0) {
			spin_unlock_irqrestore(lock,flags);
		}
		
		// no more data, quit
		if (done) {
			break;
		}
	}

	// return actual bytes read	
    return status;
}

/**
 * @param q    message queue to use
 * @param buf  buffer to fill
 * @param len  number of bytes to get
 * @param lock spinlock to use. if 0, skip locking 
 * @return     number of bytes actually enqueued
 */
static inline int ser_put(ser_buffer_t *q,unsigned char *buf,int len,spinlock_t *lock) 
{
    int           pcount;
    unsigned long flags = 0;
	unsigned char *p;
	int            i;
    int            done;
    
	pcount = 0;
	p      = buf;
	done   = 0;
	for(i=0;i<len;i++) {
		// lock around each insertion into the list
		// this reduces the lock time span but increases the overhead
		if (lock != 0) {
			spin_lock_irqsave(lock,flags);
		}
		
		if (q->count < SER_BUFFER_SIZE) {
			// add at tail
			q->buf[q->tail] = *p;
			
			// next buffer position
			p++;
			
			// increment tail with wrap
			q->tail++;
			if (q->tail >= SER_BUFFER_SIZE) {
				q->tail = 0;
			}
			// increment count
			q->count++;
			
			// ok
			pcount++;
		}
		else {
			// circular buffer is full, signal quit but first unlock
			done = 1;
		}
		
		// must match locks with unlocks
		if (lock != 0) {
			spin_unlock_irqrestore(lock,flags);
		}
		
		// circular buffer is full, quit
		if (done) {
			break;
		}
	}
    
	// return actual count of byte enqueued
    return pcount;
}

/**
 * @return number of available messages
 */
static inline int  ser_count(ser_buffer_t *q)
{
    return q->count;
}

#endif // __SERQUEUE_H__

