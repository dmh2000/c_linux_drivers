#ifndef __MULTICOMM_H__
#define __MULTICOMM_H__

#define MULTICOMM_MAX_WRITE 256
#define MULTICOMM_MAX_READ  256

#pragma pack(1)
 
/** 
 * can runtime statistics data
 */
typedef struct ser_stats_t {
    unsigned long isr;            // total interrupts
    unsigned long rxchr;          // number of receive packets
    unsigned long txchr;          // number of transmit packets
    
    unsigned long rxisr;          // number of receive interrupts
    unsigned long txisr;          // number of transmit interrupts
    unsigned long errisr;         // number of error interrupts
	
	unsigned long rxovr;          // receive  buffer overflow
	unsigned long rxbuf;          // receive buffer count
	unsigned long txovr;          // transmit buffer overflow
	unsigned long txbuf;          // transmit buffer count
} ser_stats_t;


// ============================
// IOCTLS
// ============================
/**
 * print statistics to kernel output.   	
 * status = ioctl(fd,SER_IOCTL_PRINTK,0);
 * always returns 0
 */
#define SER_IOCTL_PRINTK 0 

/**
 * copy statistics to user buffer.			
 * status = ioctl(fd,SER_IOCTL_STATS,buffer);
 * buffer must be large enough to hold ser_stats_t
 * always returns 0
 */
#define SER_IOCTL_STATS  1

/**
 * reset hardware after error signal
 */
#define SER_IOCTL_RESET 2

/**
 * set the baud rate
 * status = ioctl(fd,SER_IOCTL_SETBAUD,baud);
 */
#define SER_IOCTL_SETBAUD 3

/**
 * also support 
 * TIOCMGET	- get modem control status
 * TIOCMBIS - set a modem control bit
 * TIOCMBIC - clear a modem control bit 
 */

#endif /* __MULTICOMM_H__ */

