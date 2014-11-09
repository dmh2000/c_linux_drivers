#ifndef __CAN_H__
#define __CAN_H__

#pragma pack(1)
 /** 
  *  CAN message structure
  *  each CAN message is 16 bytes long. 
  *  when the user mode reads and writes to the device,
  *  it must read and write in 16 byte blocks with this format, even though it is a character
  *  device
  */
typedef struct can_msg_t  {
    long            id;          // 32 bits of ID to cover CAN 2.0A and CAN 2.0B
    long            len;         // data length (max = 8)
    unsigned char   data[8];     // max 8 bytes per message
} can_msg_t;
#pragma pack()

/** 
 * can runtime statistics data
 */
typedef struct can_stats_t {
    unsigned long isr;            // total interrupts
    unsigned long rxpkt;          // number of receive packets
    unsigned long txpkt;          // number of transmit packets
	unsigned long txsnt;          // number of transmit packets actually sent
    
    unsigned long rxisr;          // number of receive interrupts
    unsigned long txisr;          // number of transmit interrupts
    unsigned long errisr;         // number of error interrupts
    unsigned long ovrisr;         // number of overrun interrupts
    
    unsigned long txblck;         // transmit buffer locked
    unsigned long busoff;         // bus off status
    unsigned long errcnt;         // error count
} can_stats_t;


/** message id for passive error */
#define CAN_PASSIVE 0x40000000

/** 
 * message id for busoff error.
 * device should be reset
 */
#define CAN_BUSOFF  0x80000000           

// ============================
// IOCTLS
// ============================
/**
 * print statistics to kernel output.   	
 * status = ioctl(fd,CAN_IOCTL_PRINTK,0);
 * always returns 0
 */
#define CAN_IOCTL_PRINTK 0 

/**
 * copy statistics to user buffer.			
 * status = ioctl(fd,CAN_IOCTL_STATS,buffer);
 * buffer must be large enough to hold can_stats_t
 * always returns 0
 */
#define CAN_IOCTL_STATS  1

/**
 * reset hardware after error signal
 */
#define CAN_IOCTL_RESET 2

#endif /* __CAN_H__ */
