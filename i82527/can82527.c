#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/ioport.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/semaphore.h>
#include <asm/system.h>

/**
 * intel CAN 82527 driver for RTD ECANHR527-1
 * @author david howard
 * copyright (C) 2004 David Howard
 * for license information see COPYING
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

MODULE_LICENSE("Dual BSD/GPL");

#define CAN82527_IOMEM_SIZE     512
#define CAN82527_MODULE_NAME    "can82527"
#define CAN82527_DEVICE_NAME    "can82527"
#define CAN82527_MINOR_NUMBERS  1
#define CAN82527_MSG_SIZE       10
#define CAN82527_MSGQ_COUNT     64
/***************************************************************************************
 * forward references
 ***************************************************************************************/
ssize_t can82527_read(struct file *,char *,size_t, loff_t *);
ssize_t can82527_write(struct file *,const char *,size_t,loff_t *);
int     can82527_ioctl(struct inode *,struct file *,unsigned int,unsigned long);
int     can82527_open(struct inode *,struct file *);
int     can82527_release(struct inode *,struct file*);
 
/***************************************************************************************
 * module variables
 ***************************************************************************************/

/** io memory map base (input parameter) */
static int addr = 0x0;
module_param(addr,int,0);

/** irq (input parameter) */
static int irq = 0;
module_param(irq,int,0);

/**
 * memory region flag
 * 0 = iomem allocation failed
 * nonzero = succeeded
 */
static void *can82527_iomem_flag = 0;

/**
 * remapped IO memory address
 * 0 = iomem allocation failed
 * nonzero = succeeded
 */
static void *can82527_iomem_addr = 0;

/**
 * registered flag
 * 0 == registered
 * nonzero = failed
 */
static int can82527_reg_status = -EBUSY;

/**
 * device major/minor number
 */
 static dev_t can82527_dev_t = 0;

/**
 * cdev flag
 * 0 == cdev added
 * nonzero = failed
 */
static int can82527_cdev_status = -EBUSY;

/** device fileops */
static struct file_operations can82527_ops = {
        .owner   =       THIS_MODULE,
        .open    =       can82527_open,
        .read    =       can82527_read,
        .write   =       can82527_write,
        .release =       can82527_release,
        .ioctl   =       can82527_ioctl,
};

/** device cdev */
static struct cdev can82527_cdev = {
        .kobj   =       {.name = CAN82527_MODULE_NAME, },
        .owner  =       THIS_MODULE,
};

/***************************************************************************************
 * local data structures
 ***************************************************************************************/
 /** each message is 16 bytes long. when the user mode reads and writes to the device,
  *  it must read and write in 16 byte blocks with this format, even though it is a character
  *  device
  */
typedef struct can82527_msg_t  {
    unsigned long  id;          // 32 bits of ID to cover CAN 2.0A and CAN 2.0B
    unsigned long  len;         // data length (max = 8)
    unsigned char  data[8];     // max 8 bytes per message
} can82527_msg_t;

typedef struct can82527_queue_t {
    can82527_msg_t  buf[CAN82527_MSGQ_COUNT];
    int             head;  // remove from here
    int             tail;  // add here
    int             count; // number of available bytes
    spinlock_t      lock;
} can82527_queue_t;    

typedef struct can82527_statistics_t {
    unsigned long isr;            // total interrupts
    unsigned long spisr;          // number of spurious interrupts
    unsigned long rxisr;          // number of receive interrupts
    unsigned long rxpkt;          // number of receive packets
    unsigned long txisr;          // number of transmit interrupts
    unsigned long txpkt;          // number of transmit packets
    unsigned long erisr;          // number of error interrupts
} can82527_statistics_t;

/***************************************************************************************
 * HW data structures and definitions
 ***************************************************************************************/
// contents of controller message object
typedef struct can82527_mos_t {
    unsigned char ctl0;
    unsigned char ctl1;
    unsigned long arb;
    unsigned char conf;
    unsigned char data[8];
} can82527_mos_t;

 
// 82527 register map (the controller is memory mapped, no IO ports)
/** can82527 register layout */
typedef struct can82527_register_t {
    unsigned char  cr;              // control register
    unsigned char  sr;              // status register
    unsigned char  cpu;             // cpu interface register
    unsigned char  res03;           // reserved          
    unsigned short hsr;             // high speed read
    unsigned short gms;             // global mask standard
    unsigned long  gmx;             // global mask extended
    unsigned long  msk15;           // message 15 mask
    can82527_mos_t msg1;            // message 15 data
    unsigned char  clk;             // clockout register
    can82527_mos_t msg2;            // message 15 data
    unsigned char  bus;             // bus config register
    can82527_mos_t msg3;            // message 15 data
    unsigned char  bit0;            // bit timing register 0
    can82527_mos_t msg4;            // message 15 data
    unsigned char  bit1;            // bit timing register 1
    can82527_mos_t msg5;            // message 15 data
    unsigned char  intr;            // interrupt register
    can82527_mos_t msg6;            // message 15 data
    unsigned char  res6f;           // reserved
    can82527_mos_t msg7;            // message 15 data
    unsigned char  res7f;           // reserved
    can82527_mos_t msg8;            // message 15 data
    unsigned char  res8f;           // reserved
    can82527_mos_t msg9;            // message 15 data
    unsigned char  p1cf;            // P1CONF
    can82527_mos_t msg10;           // message 15 data
    unsigned char  p2cf;            // P2CONF
    can82527_mos_t msg11;           // message 15 data
    unsigned char  p1in;            // P1IN
    can82527_mos_t msg12;           // message 15 data
    unsigned char  p2in;            // P2IN
    can82527_mos_t msg13;           // message 15 data
    unsigned char  p1out;           // P1OUT
    can82527_mos_t msg14;           // message 15 data
    unsigned char  p2out;           // P2OUT
    can82527_mos_t msg15;           // message 15 data
    unsigned char  rst;             // serial reset address
} can82527_register_t;

/***************************************************************************************
 * per device private data
 ***************************************************************************************/
typedef struct can82527_pdata_t {
    struct semaphore             sema;       /** semaphore to prevent multiple process access */
    int                          is_open;    /** flag indicating driver is open or not */
    char                         data;       /** dummy read/write buffer */
    wait_queue_head_t            read_wait;  /** read wait structures */
    wait_queue_head_t            write_wait; /** write wait structures */
    can82527_queue_t             read_data;  /** read buffer for transmission of data from ISR to process */
    can82527_queue_t             write_data; /** write buffer for transmission of data from process to isr/driver */
    can82527_register_t          hw_reg;     /** shadow hardware registers CAN controller */
    void                        *hw_base;    /** base address of can controller */
    int                          hw_irq;     /** desired IRQ for CAN controller */
    can82527_statistics_t        stats;      /** tx/rx/err stats */
} can82527_pdata_t;

// one instance to start
can82527_pdata_t can82527_pdata[CAN82527_MINOR_NUMBERS];
/***************************************************************************************
 * 82527 hardware access
 ***************************************************************************************/
// register offsets
#define I82527_CR       0x00
#define I82527_SR       0x01
#define I82527_CPU      0x02
#define I82527_HREAD_LSB 0x04
#define I82527_HREAD_MSB 0x05
#define I82527_GMS      0x06
#define I82527_GMX      0x08
#define I82527_CLK      0x1f
#define I82527_BUS      0x2f
#define I82527_BIT0     0x3f
#define I82527_BIT1     0x4f
#define I82527_INTR     0x5f
#define I82527_P1CONF   0x9f
#define I82527_P1OUT    0xdf
#define I82527_P2CONF   0xaf
#define I82527_P2OUT    0xef

// control register definitions
#define I85257_CR_INIT     0x01
#define I82527_CR_CCE      0x40
#define	I82527_CPU_RST     0x80
#define	I82527_CPU_CEN	    0x01

// MESSAGE OBJECT OFFSETS
#define I82527_MSG_1  0x10
#define I82527_MSG_2  0x20
#define I82527_MSG_3  0x30
#define I82527_MSG_4  0x40
#define I82527_MSG_5  0x50
#define I82527_MSG_6  0x60
#define I82527_MSG_7  0x70
#define I82527_MSG_8  0x80
#define I82527_MSG_9  0x90
#define I82527_MSG_10 0xa0
#define I82527_MSG_11 0xb0
#define I82527_MSG_12 0xc0
#define I82527_MSG_13 0xd0
#define I82527_MSG_14 0xe0
#define I82527_MSG_15 0xf0

// MESSAGE OBJECT CONTENTS OFFSETS
#define I82527_MSG_CTL0 0x00
#define I82527_MSG_CTL1 0x01
#define I82527_MSG_ARB0 0x02
#define I82527_MSG_ARB1 0x03
#define I82527_MSG_ARB2 0x04
#define I82527_MSG_ARB3 0x05
#define I82527_MSG_CONF 0x06
#define I82527_MSG_DATA 0x07

static inline void can82527_writeb(unsigned char val,unsigned char *add)
{
    writeb(val,add);
    wmb();
}

static inline void can82527_writew(unsigned short val,unsigned short *add)
{
    writew(val,add);
    wmb();
}

static inline void can82527_writel(unsigned long val,unsigned long *add)
{
    writel(val,add);
    wmb();
}

static inline unsigned char can82527_readb(unsigned char *base,int offset)
{
    // read the actual register first, then the High Speed Read register
    // to get the real value
    readb(base + offset);
    rmb();
    return readb((base + I82527_HREAD_LSB));
}

static inline unsigned short can82527_readw(unsigned char *base,int offset)
{
    // read the actual register first, then the High Speed Read register
    // to get the real value
    readw(base + offset);
    rmb();
    return readw((base + I82527_HREAD_LSB));
}

static inline void can82527_hw_disable_msgs(void *base,can82527_register_t *reg)
{
    // ------------------------------------------
    // write CPUUD = 0, RmtPnd = 0, TxRq = 0, NewDat = 0
    // 01011001
    reg->msg1.ctl1 = 0x55;
    can82527_writeb(reg->msg1.ctl1,base + I82527_MSG_1 + I82527_MSG_CTL1);
    
    // write MSGVAL = 0, TXIE = 0, RXIE = 0, INTPND = 0
    // 01010101
    reg->msg1.ctl0 = 0x55;
    can82527_writeb(reg->msg1.ctl0,base + I82527_MSG_1 + I82527_MSG_CTL0);
    
    // ------------------------------------------
    // write CPUUD = 0, RmtPnd = 0, TxRq = 0, NewDat = 0
    // 01011001
    reg->msg2.ctl1 = 0x55;
    can82527_writeb(reg->msg2.ctl1,base + I82527_MSG_2 + I82527_MSG_CTL1);
    
    // write MSGVAL = 0, TXIE = 0, RXIE = 0, INTPND = 0
    // 01010101
    reg->msg2.ctl0 = 0x55;
    can82527_writeb(reg->msg2.ctl0,base + I82527_MSG_2 + I82527_MSG_CTL0);
    
    // ------------------------------------------
    // write CPUUD = 0, RmtPnd = 0, TxRq = 0, NewDat = 0
    // 01011001
    reg->msg3.ctl1 = 0x55;
    can82527_writeb(reg->msg3.ctl1,base + I82527_MSG_3 + I82527_MSG_CTL1);
    
    // write MSGVAL = 0, TXIE = 0, RXIE = 0, INTPND = 0
    // 01010101
    reg->msg3.ctl0 = 0x55;
    can82527_writeb(reg->msg3.ctl0,base + I82527_MSG_3 + I82527_MSG_CTL0);
    
    // ------------------------------------------
    // write CPUUD = 0, RmtPnd = 0, TxRq = 0, NewDat = 0
    // 0101101
    reg->msg4.ctl1 = 0x55;
    can82527_writeb(reg->msg4.ctl1,base + I82527_MSG_4 + I82527_MSG_CTL1);
    
    // write MSGVAL = 0, TXIE = 0, RXIE = 0, INTPND = 0
    // 01010101
    reg->msg4.ctl0 = 0x55;
    can82527_writeb(reg->msg4.ctl0,base + I82527_MSG_4 + I82527_MSG_CTL0);
    
    // ------------------------------------------
    // write CPUUD = 0, RmtPnd = 0, TxRq = 0, NewDat = 0
    // 01011001
    reg->msg5.ctl1 = 0x55;
    can82527_writeb(reg->msg5.ctl1,base + I82527_MSG_5 + I82527_MSG_CTL1);
    
    // write MSGVAL = 0, TXIE = 0, RXIE = 0, INTPND = 0
    // 01010101
    reg->msg5.ctl0 = 0x55;
    can82527_writeb(reg->msg5.ctl0,base + I82527_MSG_5 + I82527_MSG_CTL0);
    
    // ------------------------------------------
    // write CPUUD = 0, RmtPnd = 0, TxRq = 0, NewDat = 0
    // 01011001
    reg->msg6.ctl1 = 0x55;
    can82527_writeb(reg->msg1.ctl1,base + I82527_MSG_6 + I82527_MSG_CTL1);
    
    // write MSGVAL = 0, TXIE = 0, RXIE = 0, INTPND = 0
    // 01010101
    reg->msg6.ctl0 = 0x55;
    can82527_writeb(reg->msg6.ctl0,base + I82527_MSG_6 + I82527_MSG_CTL0);
    
    // ------------------------------------------
    // write CPUUD = 0, RmtPnd = 0, TxRq = 0, NewDat = 0
    // 01011001
    reg->msg7.ctl1 = 0x55;
    can82527_writeb(reg->msg7.ctl1,base + I82527_MSG_7 + I82527_MSG_CTL1);
    
    // write MSGVAL = 0, TXIE = 0, RXIE = 0, INTPND = 0
    // 01010101
    reg->msg7.ctl0 = 0x55;
    can82527_writeb(reg->msg7.ctl0,base + I82527_MSG_7 + I82527_MSG_CTL0);
    
    // ------------------------------------------
    // write CPUUD = 0, RmtPnd = 0, TxRq = 0, NewDat = 0
    // 01011001
    reg->msg8.ctl1 = 0x55;
    can82527_writeb(reg->msg1.ctl1,base + I82527_MSG_8 + I82527_MSG_CTL1);
    
    // write MSGVAL = 0, TXIE = 0, RXIE = 0, INTPND = 0
    // 01010101
    reg->msg8.ctl0 = 0x55;
    can82527_writeb(reg->msg1.ctl0,base + I82527_MSG_8 + I82527_MSG_CTL0);
    
    // ------------------------------------------
    // write CPUUD = 0, RmtPnd = 0, TxRq = 0, NewDat = 0
    // 01011001
    reg->msg9.ctl1 = 0x55;
    can82527_writeb(reg->msg1.ctl1,base + I82527_MSG_9 + I82527_MSG_CTL1);
    
    // write MSGVAL = 0, TXIE = 0, RXIE = 0, INTPND = 0
    // 01010101
    reg->msg9.ctl0 = 0x55;
    can82527_writeb(reg->msg1.ctl0,base + I82527_MSG_9 + I82527_MSG_CTL0);
    // ------------------------------------------
    // write CPUUD = 0, RmtPnd = 0, TxRq = 0, NewDat = 0
    // 01011001
    reg->msg11.ctl1 = 0x55;
    can82527_writeb(reg->msg11.ctl1,base + I82527_MSG_11 + I82527_MSG_CTL1);
    
    // write MSGVAL = 0, TXIE = 0, RXIE = 0, INTPND = 0
    // 01010101
    reg->msg11.ctl0 = 0x55;
    can82527_writeb(reg->msg11.ctl0,base + I82527_MSG_11 + I82527_MSG_CTL0);
    
    // ------------------------------------------
    // write CPUUD = 0, RmtPnd = 0, TxRq = 0, NewDat = 0
    // 01011001
    reg->msg12.ctl1 = 0x55;
    can82527_writeb(reg->msg12.ctl1,base + I82527_MSG_12 + I82527_MSG_CTL1);
    
    // write MSGVAL = 0, TXIE = 0, RXIE = 0, INTPND = 0
    // 01010101
    reg->msg12.ctl0 = 0x55;
    can82527_writeb(reg->msg12.ctl0,base + I82527_MSG_12 + I82527_MSG_CTL0);
    
    // ------------------------------------------
    // write CPUUD = 0, RmtPnd = 0, TxRq = 0, NewDat = 0
    // 01011001
    reg->msg13.ctl1 = 0x55;
    can82527_writeb(reg->msg13.ctl1,base + I82527_MSG_13 + I82527_MSG_CTL1);
    
    // write MSGVAL = 0, TXIE = 0, RXIE = 0, INTPND = 0
    // 01010101
    reg->msg13.ctl0 = 0x55;
    can82527_writeb(reg->msg13.ctl0,base + I82527_MSG_13 + I82527_MSG_CTL0);
    
    // ------------------------------------------
    // write CPUUD = 0, RmtPnd = 0, TxRq = 0, NewDat = 0
    // 0101101
    reg->msg14.ctl1 = 0x55;
    can82527_writeb(reg->msg14.ctl1,base + I82527_MSG_14 + I82527_MSG_CTL1);
    
    // write MSGVAL = 0, TXIE = 0, RXIE = 0, INTPND = 0
    // 01010101
    reg->msg14.ctl0 = 0x55;
    can82527_writeb(reg->msg14.ctl0,base + I82527_MSG_14 + I82527_MSG_CTL0);
    
    // ------------------------------------------
    // write CPUUD = 0, RmtPnd = 0, TxRq = 0, NewDat = 0
    // 01011001
    reg->msg15.ctl1 = 0x55;
    can82527_writeb(reg->msg15.ctl1,base + I82527_MSG_15 + I82527_MSG_CTL1);
    
    // write MSGVAL = 0, TXIE = 0, RXIE = 0, INTPND = 0
    // 01010101
    reg->msg15.ctl0 = 0x55;
    can82527_writeb(reg->msg15.ctl0,base + I82527_MSG_15 + I82527_MSG_CTL0);
}

// cpu register definitions
static inline void can82527_hw_init(void *base,can82527_register_t *reg)
{
    // reset the board
    
    // set to init mode. this stops all can activity and allows configuration
    // CCE  = 0x40
    // EIE  = 0x00
    // SIE  = 0x00
    // IE   = 0x00
    // INIT = 0x01
    reg->cr = 0x41;
    can82527_writeb(reg->cr,base + I82527_CR);
    
    // set up p2 bits 0 and 1 as digital outputs for the LED's
    reg->p2cf = 0xff;
    can82527_writeb(reg->p2cf,base + I82527_P2CONF);
    
    // light up the leds when the module is installed
    reg->p2out = 0x03;
    can82527_writeb(reg->p2out,base + I82527_P2OUT);
    
    // clear status register
    reg->sr = 0;
    can82527_writeb(reg->sr,base + I82527_SR);
    
    // set cpu interface register
    // CEN = 0x01
    reg->cpu = 0x01;
    can82527_writeb(reg->cpu,base + I82527_CPU);
    
    // CLKOUT = XTAL,  8 < clkout < 16
    reg->clk  = 0x20;
    can82527_writeb(reg->clk,base + I82527_CLK);
    
    // BUS = normal
    reg->bus  = 0x00;
    can82527_writeb(reg->bus,base + I82527_BUS);
    
    // bit timing per RTD document
    // 0x00-0xc6 = 1   mbs
    // 0x01-0xc6 = 500 kbs
    // sjw=0, prescaler = 0
    reg->bit0 = 0x00;
    can82527_writeb(reg->bit0,base + I82527_BIT0);
    // sampling mode = 1
    // tseg2         = 5
    // tset1         = 6
    // (11 total quanta)
    reg->bit1 = 0xc6;
    can82527_writeb(reg->bit1,base + I82527_BIT1);
    
    // ====================================
    // set global masks to sniff everything
    // ====================================
    reg->gms = 0;
    can82527_writew(reg->gms,base + I82527_GMS);
    reg->gmx = 0;
    can82527_writel(reg->gmx,base + I82527_GMX);
    
    // ====================================
    // disable all messages
    // ====================================
    can82527_hw_disable_msgs(base,reg);
    
    // ====================================
    // INIT MSG 4..12 for receive
    // ====================================
    
    
    // ====================================
    // enable interrupts 
    // ====================================
    // CCE  = 0x40
    // EIE  = 0x00
    // SIE  = 0x00
    // IE   = 0x02
    // INIT = 0x01
    reg->cr = 0x43;
    can82527_writeb(reg->cr,base + I82527_CR);
    
    // ====================================
    // go 
    // ====================================
    // CCE  = 0x00
    // EIE  = 0x00
    // SIE  = 0x00
    // IE   = 0x02
    // INIT = 0x00
    reg->cr = 0x02;
    can82527_writeb(reg->cr,base + I82527_CR);
}

// cpu register definitions
static inline void can82527_hw_close(void *base,can82527_register_t *reg)
{
    // turn off the leds
    reg->p2out &= ~0x03;
    can82527_writeb(reg->p2out,base + I82527_P2OUT);
    
    // set to init mode. this stops all can activity and allows configuration
    reg->cr = I85257_CR_INIT | I82527_CR_CCE;
    can82527_writeb(reg->cr,base + I82527_CR);
}

static inline void can82527_hw_p1out(void *base,can82527_register_t *reg,unsigned char val)
{
    // light the leds
    reg->p1out = val;
    can82527_writeb(reg->p1out,base + I82527_P1OUT);
}    

static inline void can82527_hw_p2out(void *base,can82527_register_t *reg,unsigned char val)
{
    // light the leds
    reg->p2out = val;
    can82527_writeb(reg->p2out,base + I82527_P2OUT);
}    

static inline void can82527_hw_toggle_rxled(void *base,can82527_register_t *reg)
{
    // toggle the leds
    if (reg->p2out & 0x01) {
        reg->p2out &= ~0x01;
    }
    else {
        reg->p2out |= 0x01;
    }
    can82527_writeb(reg->p2out,base + I82527_P2OUT);
}

static inline void can82527_hw_toggle_txled(void *base,can82527_register_t *reg)
{
    if (reg->p2out & 0x02) {
        reg->p2out &= ~0x02;
    }
    else {
        reg->p2out |= 0x02;
    }
    can82527_writeb(reg->p2out,base + I82527_P2OUT);
}

/**
 * Interrupt msg obj 1
 */
static inline void can82527_hw_txint(void *base,can82527_register_t *reg)
{
    // write CPUUD = 1, TxRqst = 0 leave others unchanged
    // 1101 1011
    reg->msg1.ctl1 = 0xdb;
    can82527_writeb(reg->msg1.ctl1,base + I82527_MSG_1 + I82527_MSG_CTL1);
    
    // write MSGVAL = 1, clear IntPnd
    // 1011 1101
    reg->msg1.ctl0 = 0xbd;
    can82527_writeb(reg->msg1.ctl0,base + I82527_MSG_1 + I82527_MSG_CTL0);
    
    // Clear CPUUD with other bits unchanged
    // 0x1111 0111
    reg->msg1.ctl1 = 0xf7;
    can82527_writeb(reg->msg1.ctl1,base + I82527_MSG_1 + I82527_MSG_CTL1);
}

/**
 * TRANSMIT msg obj 1
 * all transmissions go out msg object structure 1. using more than one output structure
 * would potentially speed up transmission but increases complexity
 */
static inline void can82527_hw_tx(void *base,can82527_register_t *reg,unsigned long msgid,int msglen,char msgdata[8])
{
    unsigned char xtd;
    int           i;
    
    // clear the status register
    reg->sr = 0;
    can82527_writeb(reg->sr,base + I82527_SR);
    
    // write CPUUD = 1 and NEW DATA = 1 to allow updates
    // 1111 1010
    reg->msg1.ctl1 = 0xfa;
    can82527_writeb(reg->msg1.ctl1,base + I82527_MSG_1 + I82527_MSG_CTL1);
    
    // compute and store the address
    if (msgid >= 2048) {
        // extended ID
        //          2          1         
        // 87654 32109876 54321098 76543210
        // xxxxxxxx xxxxxxxx xxxxxxxx xxxxx000
        // 10987654 32109876 54321098 76543210
        //  3          2          1         
        msgid <<= 3;           // shift left 3 to align 29 bit address with id fields in registers
        xtd = 0x04;            // extended bit
    }
    else {
        // standard id
        //          2          1         
        // 87654 32109876 54321098 76543210
        // xxxxxxxx xxx00000 00000000 00000000
        // 10987654 32109876 54321098 76543210
        //  3          2          1         
        msgid <<=21;          // shift left 21 to align 11 bit address with id fields in registers
        xtd = 0x00;           // extended bit
    }
    
    // write the data configuration register for transmit, length and extended or standard
    if (msglen > 8) msglen = 8;
    reg->msg1.conf = ((msglen << 4) & 0xf0) | 0x08 | xtd;
    can82527_writeb(reg->msg1.conf,base + I82527_MSG_1 + I82527_MSG_CONF);
    
    // write the id bytes 
    reg->msg1.arb  = msgid;
    can82527_writel(reg->msg1.arb,base + I82527_MSG_1 + I82527_MSG_ARB0);
    
    // write the data bytes
    for(i=0;i<8;i++) {
        reg->msg1.data[i] = msgdata[i];
        can82527_writeb(reg->msg1.data[i],base + I82527_MSG_1 + I82527_MSG_DATA + i);
    }
    
    // write MSGVAL = 1,TIE = 1, RXIE = 0, IntPnd = 0
    // 1010 0101
    reg->msg1.ctl0 = 0xa5;
    can82527_writeb(reg->msg1.ctl0,base + I82527_MSG_1 + I82527_MSG_CTL0);
    
    // Clear CPUUD and set NEW DATA with other bits unchanged
    // 0x1111 0110
    reg->msg1.ctl1 = 0xf6;
    can82527_writeb(reg->msg1.ctl1,base + I82527_MSG_1 + I82527_MSG_CTL1);
    
    // set transmit request leave other bits unchanged
    // 0x1110 1111
    reg->msg1.ctl1 = 0xef;
    can82527_writeb(reg->msg1.ctl1,base + I82527_MSG_1 + I82527_MSG_CTL1);
    
    // toggle the transmit led
    can82527_hw_toggle_txled(base,reg);
}

irqreturn_t can82527_int_handler(int irq,void *p,struct pt_regs *dreg)
{
    irqreturn_t          status = IRQ_HANDLED;
    can82527_pdata_t    *pdata  = (can82527_pdata_t *)p;
    void                *base   = pdata->hw_base;
    can82527_register_t *reg    = &pdata->hw_reg;
    
    pdata->stats.isr++;
    
    // get the interrupt data
    reg->intr = can82527_readb(base,I82527_INTR);
    if (reg->intr == 0x03) {
        // service msg object 1
        can82527_hw_txint(base,reg);
        pdata->stats.txisr++;
    }
    else {
        printk(KERN_ALERT "Intel CAN 82527 driver spurious interrupt : %d\n",reg->intr);
        status = IRQ_NONE;
        pdata->stats.spisr++;
    }
    
    return status;
}

/***************************************************************************************
 * io circular byte buffer functions
 ***************************************************************************************/
static inline void can82527_init_buffer(can82527_queue_t *q)
{
    memset(q->buf,0,sizeof(q->buf));
    q->tail = 0;
    q->head = 0;
    q->count = 0;
    spin_lock_init(&(q->lock));
    
}

/**
 * always get in units of 1 CAN message
 * usable from ISR so it uses a spin lock
 * @return 1 if a message is received, 0 if no message was available
 */
static inline int  can82527_get_msg(can82527_queue_t *q,can82527_msg_t *msg) 
{
    int status;
    unsigned long flags;
    
    // lock data structure interrupt safe
    spin_lock_irqsave(&q->lock,flags);
    
    if (q->count > 0) {
        // get from head
        *msg = q->buf[q->head];
        // increment head with wrap
        q->head++;
        if (q->head >= CAN82527_MSGQ_COUNT) {
            q->head = 0;
        }
        // decrement count
        q->count--;
        // ok
        status = sizeof(can82527_msg_t);
    }
    else {
        // no data
        status = 0;
    }
    
    // unlock
    spin_unlock_irqrestore(&q->lock,flags);
    
    return status;
}

/**
 * always put 1 complete can message (10 bytes)
 * usable from ISR so it uses a spin lock
 * @return number of bytes if message is written, 0 if buffer was full
 */
static inline int  can82527_put_msg(can82527_queue_t *q,can82527_msg_t *msg) 
{
    int status;
    unsigned long flags;
    
    // lock data structure interrupt safe
    spin_lock_irqsave(&q->lock,flags);
    
    if (q->count < CAN82527_MSGQ_COUNT) {
        // add at tail
        q->buf[q->tail] = *msg;
        // increment tail with wrap
        q->tail++;
        if (q->tail >= CAN82527_MSGQ_COUNT) {
            q->tail = 0;
        }
        // increment count
        q->count++;
        // ok
        status = sizeof(can82527_msg_t);
    }
    else {
        // no data
        status = 0;
    }
    
    // unlock
    spin_unlock_irqrestore(&q->lock,flags);
    
    return 0;
}

/**
 * @return number of available messages
 */
static inline int  can82527_has_msg(can82527_queue_t *q)
{
    return q->count;
}

/***************************************************************************************
 * module functions
 ***************************************************************************************/
/**
 * clean up module resources
 */
static void can82527_cleanup(void)
{
    int i;
    can82527_pdata_t *pdata;

    printk(KERN_ALERT "Intel CAN 82527 driver cleanup\n");
    
    // cleanup the device data
    for(i=0;i<CAN82527_MINOR_NUMBERS;i++) {
        // point at private data for this minor number
        pdata = &can82527_pdata[i];
        
        // de-init the actual hardware
        can82527_hw_close(pdata->hw_base,&pdata->hw_reg);
        
        // remove the interrupt handler
        free_irq(pdata->hw_irq,pdata);
    }
    
    // delete the cdev
    if (can82527_cdev_status == 0) {
        cdev_del(&can82527_cdev);
    }
    
    // release the device
    if (can82527_reg_status == 0) {
        unregister_chrdev_region(can82527_dev_t,CAN82527_MINOR_NUMBERS);
    }
    
    // unmap the iomem
    if (can82527_iomem_addr != 0) {
        iounmap(can82527_iomem_addr);
    }
    
    // release the io memory region
    if (can82527_iomem_flag != 0) {
        release_mem_region(addr,CAN82527_IOMEM_SIZE);
    }
}

/**
 * init the module
 */
static int can82527_init(void)
{
    int i;
    int j;
    can82527_pdata_t *pdata;
    int status;
    
    printk(KERN_ALERT "Intel CAN 82527 driver load addr=0x%04x irq=%d\n",addr,irq);
/*    
    // get the io address
    can82527_iomem_flag = request_mem_region(addr,CAN82527_IOMEM_SIZE,CAN82527_MODULE_NAME);
    if (can82527_iomem_flag == 0) {
        printk(KERN_ALERT "Intel CAN 82527 driver can't allocate memory at 0x%05x\n",addr);
        return -EBUSY;
    }
    
    // remap it to make it accessible
    can82527_iomem_addr = ioremap(addr,CAN82527_IOMEM_SIZE);
    printk(KERN_ALERT "Intel CAN 82527 driver ioremap %08lx %08lx\n",(unsigned long)addr,(unsigned long)can82527_iomem_addr);
*/

// ???????????????????????????????????????????????????????????????    
    // print hardware data
    for(i=0;i<0x100;i+=16) {
        printk("%08lx : ",(unsigned long)addr + i);
        for(j=0;j<16;j++) {
            printk("%02x ",isa_readb((unsigned long)(addr + i + j)));
        }
        printk("\n");
    }
    return -EBUSY;
// ???????????????????????????????????????????????????????????????

    
    // register the device
    can82527_reg_status = alloc_chrdev_region(&can82527_dev_t,0,CAN82527_MINOR_NUMBERS,CAN82527_MODULE_NAME); 
    if (can82527_reg_status != 0) {
        // failed
        printk(KERN_ALERT "Intel CAN 82527 driver can't be registered\n");
        can82527_cleanup();
        return can82527_reg_status;
    }
    
    // initialize the cdev
    cdev_init(&can82527_cdev,&can82527_ops);
    
    // add the cdev
    can82527_cdev_status = cdev_add(&can82527_cdev,can82527_dev_t,1);
    if (can82527_cdev_status != 0) {
        // failed
        printk(KERN_ALERT "Intel CAN 82527 driver can't add cdev\n");
        can82527_cleanup();
        return can82527_cdev_status;
    }
    
    // clear the device structures
    memset(can82527_pdata,0,sizeof(can82527_pdata));
    
    // initialize the device data
    for(i=0;i<CAN82527_MINOR_NUMBERS;i++) {
        // point at private data for this minor number
        pdata = &can82527_pdata[i];
        
        // init the device semaphore
        sema_init(&pdata->sema,1);
        
        // init the wait queues
        init_waitqueue_head(&pdata->read_wait);
        init_waitqueue_head(&pdata->write_wait);
        
        // init the buffers
        can82527_init_buffer(&pdata->read_data);
        can82527_init_buffer(&pdata->write_data);
        
        // init hardware parameters
        // WARNING : the input params only allow 1 board to be specified
        // they must be changed to the array style when more than 1 is to be supported
        memset(&pdata->hw_reg,0,sizeof(struct can82527_register_t));
        pdata->hw_base = can82527_iomem_addr;
        pdata->hw_irq  = irq;
        
        // set up the interrupt handler
        status = request_irq(irq,can82527_int_handler,0,CAN82527_DEVICE_NAME,pdata);
        if (status == 0) {
            // init the actual hardware
            can82527_hw_init(pdata->hw_base,&pdata->hw_reg);
        }
        else {
            printk(KERN_ALERT "Intel CAN 82527 driver IRQ busy\n");
            status = -EBUSY;
        }
    }
         
    return status;
}

/**
 * module exiting
 */
static void can82527_exit(void)
{
    can82527_cleanup();
    printk(KERN_ALERT "Intel CAN 82527 driver unload\n");
}

ssize_t can82527_read(struct file *filp,char *buffer,size_t count, loff_t *f_pos) 
{
    can82527_pdata_t *pdata;
    can82527_msg_t    msg;
    int                      status;
    
    printk(KERN_INFO "Intel CAN 82527 driver read\n");
    
    // get the private data
    pdata = (can82527_pdata_t *)filp->private_data;
    
    // buffer size must be == size of 1 can message
    if (count != sizeof(can82527_msg_t)) {
        return -EINVAL;
    }
    
    // lock the semaphore to prevent multi process access 
    if (down_interruptible(&pdata->sema)) {
        return -ERESTARTSYS;
    }
    
    // wait here until a msg is available
    status = wait_event_interruptible(pdata->read_wait,can82527_get_msg(&pdata->read_data,&msg));
    if (status > 0) {
        // got one, copy it
        copy_to_user(buffer,&msg,count);
    }
    
    // unlock the semaphore
    up(&pdata->sema);
    
    return count; 
}

ssize_t can82527_write(struct file *filp,const char *buffer,size_t count,loff_t *f_pos) 
{
    can82527_pdata_t *pdata;
    can82527_msg_t   *msg;
    size_t            rcount;
    
    printk(KERN_INFO "Intel CAN 82527 driver write\n");
    
    // buffer size must be == size of 1 can message
    if (count != sizeof(can82527_msg_t)) {
        return -EINVAL;
    }
    
    // get the private data
    pdata = (can82527_pdata_t *)filp->private_data;
    
    // lock the semaphore
    if (down_interruptible(&pdata->sema)) {
        return -ERESTARTSYS;
    }
    
    // get the user mode data which must be 16 bytes
    rcount = copy_from_user(&pdata->data,buffer,count);
    if (rcount == 0) {
        // OK
        f_pos += count;
    } else {
        count = -EFAULT;
    }
    
    // got the data, now send the frame
    msg = (can82527_msg_t *)buffer;
    can82527_hw_tx(pdata->hw_base,&pdata->hw_reg,msg->id,msg->len,msg->data);
    
    // count the packet
    pdata->stats.txpkt++;
    
    // unlock the semaphore
    up(&pdata->sema);
    
    return count;
}

int can82527_ioctl(struct inode *inode,struct file *filp,unsigned int cmd,unsigned long arg)
{
    can82527_pdata_t *pdata;
    
    printk(KERN_INFO "Intel CAN 82527 driver ioctl\n");
    
    // get the private data
    pdata = (can82527_pdata_t *)filp->private_data;
    
    // lock the semaphore
    if (down_interruptible(&pdata->sema)) {
        return -ERESTARTSYS;
    }
    
    switch(cmd) {
    default:
        break;
    }
    
    // unlock the semaphore
    up(&pdata->sema);
    
    return 0;
}


int can82527_open(struct inode *inode,struct file *filp)
{
    can82527_pdata_t *pdata;
    int minor;
    int status;
    
    printk(KERN_INFO "Intel CAN 82527 driver open\n");
    
    // get the appropriate minor device structure
    minor = MINOR(inode->i_rdev);
    if (minor < CAN82527_MINOR_NUMBERS) {
        filp->private_data = pdata = &can82527_pdata[minor];
    }
    else {
        printk(KERN_INFO "Intel CAN 82527 invalid device minor number\n");
        return -EINVAL;
    }
    
    // lock the semaphore
    if (down_interruptible(&pdata->sema)) {
        return -ERESTARTSYS;
    }
    
    // is the device already open the max number of times
    // in this case it can only be opened once
    if (pdata->is_open > 0) {
        // already open
        status = -EBUSY;
    }
    else {
        // increment reference count
        pdata->is_open++;
        status = 0;
    }
    
    // unlock the semaphore
    up(&pdata->sema);
    
    return status;
}

int can82527_release(struct inode *inode,struct file* filp)
{
    can82527_pdata_t *pdata;
    
    printk(KERN_INFO "Intel CAN 82527 driver release\n");
    
    // get the private data
    pdata = (can82527_pdata_t *)filp->private_data;

    // print statistics
    printk(KERN_INFO "Intel CAN 82527 driver statistics %lu %lu %lu %lu %lu %lu %lu\n",
            pdata->stats.isr,
            pdata->stats.spisr,
            pdata->stats.rxisr,
            pdata->stats.rxpkt,
            pdata->stats.txisr,
            pdata->stats.txpkt,
            pdata->stats.erisr
            );
    
    // lock the semaphore
    if (down_interruptible(&pdata->sema)) {
        return -ERESTARTSYS;
    }
    
    // decrement reference count
    if (pdata->is_open > 0) {
        pdata->is_open--;
    }
    
    // unlock the semaphore
    up(&pdata->sema);
    
    return 0;
}

// ==========================================================================
// module linkups
// ==========================================================================
module_init(can82527_init);
module_exit(can82527_exit);


