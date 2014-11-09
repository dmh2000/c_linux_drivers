#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/ioport.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/semaphore.h>
#include <asm/system.h>
#include "canqueue.h"
#include "can.h"

/**
 * Phillips sja1000 CAN driver for Arcom AIM_104CAN
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
MODULE_AUTHOR("Dave Howard daveh@dmh2000.com");

#define HDR                    "Philips CAN SJA1000 driver " 
#define SJA1000_MODULE_NAME    "sja1000"
#define SJA1000_DEVICE_NAME    "sja1000"
#define SJA1000_MINOR_NUMBERS  1
#define SJA1000_MSG_SIZE       10


/***************************************************************************************
 * forward references
 ***************************************************************************************/
ssize_t sja1000_read(struct file *,char *,size_t, loff_t *);
ssize_t sja1000_write(struct file *,const char *,size_t,loff_t *);
int     sja1000_ioctl(struct inode *,struct file *,unsigned int,unsigned long);
int     sja1000_open(struct inode *,struct file *);
int     sja1000_release(struct inode *,struct file*);
 
/***************************************************************************************
 * module variables
 ***************************************************************************************/

/***************************************************************************************
 * local data structures
 ***************************************************************************************/
// SJA1000 register map (the controller is memory mapped, no IO ports)
/** sja1000 register layout */
typedef struct sja1000_register_t {
    u8  cr;              // control register
    u8  cm;              // command register
    u8  sr;              // status register
    u8  ir;              // interrupt register
    u8  ac;              // acceptance code
    u8  am;              // acceptance mask
    u8  b0;              // bus timing 0
    u8  b1;              // bus timing 1
    u8  oc;              // output control
    u8  cd;              // clock divisor
    
    // transmit buffer
    u8  txid0;
    u8  txid1;
    u8  txd[8];
    
    // receive buffer
    u8  rxid0;
    u8  rxid1;
    u8  rxd[8];
} sja1000_register_t;


/***************************************************************************************
 * per device private data
 ***************************************************************************************/
typedef struct sja1000_pdata_t {
    atomic_t                     is_open;    /** flag indicating driver is open or not */
    
    wait_queue_head_t            rx_wait;    /** read wait structures */
    can_msg_queue_t              rx_dataq;   /** read buffer for transmission of data from ISR to process */
    spinlock_t                   rx_lock;    /** spin lock for receive sequence */
    struct tasklet_struct        rx_task;    /** receive tasklet */
    
    wait_queue_head_t            tx_wait;    /** write wait structures */
    can_msg_queue_t              tx_dataq;   /** write buffer for transmission of data from process to isr/driver */
    spinlock_t                   tx_lock;    /** spin lock for transmit sequence */
    int                          tx_flag;    /** flag indicating transmit in progress */
    
    sja1000_register_t           hw_reg;     /** shadow hardware registers CAN controller */
    int                          hw_addr;    /** base address of can controller */
    int                          hw_irq;     /** desired IRQ for CAN controller */
    can_stats_t         stats;          /** tx/rx/err stats */
} sja1000_pdata_t;

// one instance per controller chip
sja1000_pdata_t sja1000_pdata[SJA1000_MINOR_NUMBERS];

// ====================================================================================
// HARDWARE SPECIFIC DATA AND FUNCTIONS
// ====================================================================================
#define SJA1000_IOADDR_SIZE     34

/***************************************************************************************
 * HW data structures and definitions
 ***************************************************************************************/
 
/***************************************************************************************
 * SJA1000 hardware access
 ***************************************************************************************/
#define SJA1000_CONTROL   0
/*
CR.7 - reserved 0 0 
CR.6 - reserved X X 
CR.5 - reserved 1 1 
CR.4 OIE Overrun Interrupt Enable X X 
CR.3 EIE Error Interrupt Enable X X 
CR.2 TIE Transmit Interrupt Enable X X 
CR.1 RIE Receive Interrupt Enable X X 
CR.0 RR Reset Request 1 (reset mode) 1 (reset mode)
*/

#define SJA1000_COMMAND   1
/*
CMR.7 - reserved note 3 note 3 
CMR.6 - reserved 
CMR.5 - reserved 
CMR.4 GTS Go To Sleep 
CMR.3 CDO Clear Data Overrun 
CMR.2 RRB Release Receive Buffer 
CMR.1 AT Abort Transmission 
CMR.0 TR Transmission Request
*/

#define SJA1000_STATUS    2
/*
SR.7 BS Bus Status 0 (bus-on) X 
SR.6 ES Error Status 0 (ok) X 
SR.5 TS Transmit Status 0 (idle) 0 (idle) 
SR.4 RS Receive Status 0 (idle) 0 (idle) 
SR.3 TCS Transmission Complete Status 1 (complete) X 
SR.2 TBS Transmit Buffer Status 1 (released) 1 (released) 
SR.1 DOS Data Overrun Status 0 (absent) 0 (absent) 
SR.0 RBS Receive Buffer Status 0 (empty) 0 (empty)
*/

#define SJA1000_INTERRUPT 3
/*
IR.7 - reserved 1 1 
IR.6 - reserved 1 1 
IR.5 - reserved 1 1 
IR.4 WUI Wake-Up Interrupt 0 (reset) 0 (reset) 
IR.3 DOI Data Overrun Interrupt 0 (reset) 0 (reset) 
IR.2 EI Error Interrupt 0 (reset) X; note 4 
IR.1 TI Transmit Interrupt 0 (reset) 0 (reset) 
IR.0 RI Receive Interrupt 0 (reset) 0 (reset)
*/

#define SJA1000_ACCEPT_CODE 4
#define SJA1000_ACCEPT_MASK 5
#define SJA1000_BUS_TIMING0 6
#define SJA1000_BUS_TIMING1 7
#define SJA1000_OUTPUT_CTL  8

// =====================
// TRANSMIT BUFFER
// =====================
#define SJA1000_TX_ID0    10
/*
identifier (10 to 3)
*/

#define SJA1000_TX_ID1    11
/*
identifier (2 to 0), 
RTR and DLC identifier (2 to 0), 
*/
#define SJA1000_TX_DATA   12

// =====================
// RECEIVE BUFFER
// =====================
#define SJA1000_RX_ID0    20
/*
identifier (10 to 3)
*/
#define SJA1000_RX_ID1    21
/*
identifier (2 to 0), 
RTR and DLC identifier (2 to 0), 
*/
#define SJA1000_RX_DATA   22

#define SJA1000_CLK       31

// printk all the register data
 
static void sja1000_kreg(int ioaddr)
{
    int i;
    unsigned char b0;
    unsigned char b1;
    unsigned short id;
    
    printk("    control     = 0x%02x\n",inb(ioaddr + 0) & 0x1f);
    printk("    command     = 0x%02x\n",inb(ioaddr + 1) & 0x1f);
    printk("    status      = 0x%02x\n",inb(ioaddr + 2));
    printk("    interrupt   = 0x%02x\n",inb(ioaddr + 3) & 0x1f);
    printk("    accept code = 0x%02x\n",inb(ioaddr + 4));
    printk("    accept mask = 0x%02x\n",inb(ioaddr + 5));
    printk("    bus time 0  = 0x%02x\n",inb(ioaddr + 6));
    printk("    bus time 1  = 0x%02x\n",inb(ioaddr + 7));
    printk("    output ctl  = 0x%02x\n",inb(ioaddr + 8));
    printk("    clock div   = 0x%02x\n",inb(ioaddr + 31));
    
    printk("    transmit buffer\n");
    b0 = inb(ioaddr + 10);
    b1 = inb(ioaddr + 11);
    id = (b0 << 3) | (b1 >> 5);
    printk("        tx id       = 0x%04x\n",id);
    printk("        dlc (len)   = 0x%02x\n",b1 & 0x0f);
    printk("        rtr         = 0x%02x\n",b1 & 0x10);
    printk("        ");
    for(i=12;i<20;i++) {
        printk("%02x ",inb(ioaddr + i));
    }
    printk("\n");
    
    printk("    receive buffer\n");
    b0 = inb(ioaddr + 20);
    b1 = inb(ioaddr + 21);
    id = (b0 << 3) | (b1 >> 5);
    printk("        tx id       = 0x%04x\n",id);
    printk("        dlc (len)   = 0x%02x\n",b1 & 0x0f);
    printk("        rtr         = 0x%02x\n",b1 & 0x10);
    printk("        ");
    for(i=22;i<30;i++) {
        printk("%02x ",inb(ioaddr + i));
    }
    printk("\n");
    
}

// always run in basic mode for now
static inline void sja1000_hw_init(int ioaddr,sja1000_register_t *reg)
{
    
    // enter reset mode
    reg->cr = 0x01;
    outb(reg->cr,ioaddr + SJA1000_CONTROL);
    
    // set output control mode
    // OCMODE = 01 normal
    // OCTP1 OCTN1 OCPOL1 OCTP0 OCTN0 OCPOL0 OCMODE1 OCMODE0
    //     1     1      1     1     1      0       1       0
    reg->oc = 0xfa; // per CAN4LINUX driver
    outb(reg->oc,ioaddr + SJA1000_OUTPUT_CTL);
    
    // set clock divisor register to basic CAN mode
    reg->cd = 0x07;
    
    // set bus timing
    // 8 mhz crystal, 1 Mbs baud rate
    reg->b0 = 0x00;     // per CAN4LINUX
    outb(reg->b0,ioaddr + SJA1000_BUS_TIMING0);
    reg->b1 = 0x14;     // per CAN4LINUX
    outb(reg->b1,ioaddr + SJA1000_BUS_TIMING1);
    
    // set acceptance code and mask
    reg->ac = 0x00;  // NA unless mask is set
    outb(reg->ac,ioaddr + SJA1000_ACCEPT_CODE);
    reg->am = 0xff;  // accept all
    outb(reg->am,ioaddr + SJA1000_ACCEPT_MASK);

    // clear pending interrupts 
    reg->ir = inb(ioaddr + SJA1000_INTERRUPT);  
    
    // return to operating mode 
    reg->cr = 0x00;
    outb(reg->cr,ioaddr + SJA1000_CONTROL);
    
    // enable all interrupts 
    reg->cr = 0x1e;
    outb(reg->cr,ioaddr + SJA1000_CONTROL);
    
    // PRINT REGISTERS
    // sja1000_kreg(ioaddr);
}

// cpu register definitions
static inline void sja1000_hw_stop(int ioaddr,sja1000_register_t *reg)
{
    // disable all interrupts and put it in reset mode
    reg->cr = 0x01;
    outb(reg->cr,ioaddr + SJA1000_CONTROL);
    
    // PRINT REGISTERS
    // sja1000_kreg(ioaddr);
}

// start transmit
// MUST BE CALLED WITH SPINLOCK LOCKED
static inline int sja1000_hw_tx(sja1000_pdata_t *pdata,can_msg_t *msg)
{
    int                 ioaddr = pdata->hw_addr;
    sja1000_register_t *reg    = &pdata->hw_reg;
    int                 i;
    int                 status = 0;
    
    // count tx attempts
    pdata->stats.txpkt++;
    
    // check transmitter busy
    if (pdata->tx_flag == 1) {
        // busy, insert data into transmit queue and return
        status = can_put_msg(&pdata->tx_dataq,msg,0); 
    }
    else {
        // not busy, send it
        
        // check that transmit buffer is not locked
        reg->sr = inb(ioaddr + SJA1000_STATUS);
        if ((reg->sr & 0x04) == 0) {
            // transmit buffer locked, message lost
            pdata->stats.txblck++;
            
            // buffer locked
            status = 0;
        }
        else {
            // ok
            // count tx packets actually sent
            pdata->stats.txsnt++;
            
            // message id and length
            reg->txid0 = (msg->id >> 3);
            outb(reg->txid0,ioaddr + SJA1000_TX_ID0);
            reg->txid1 = ((msg->id & 0x07) << 5) | ((u8)msg->len & 0x0f);
            outb(reg->txid1,ioaddr + SJA1000_TX_ID1);
            
            // data bytes
            for(i=0;i<msg->len;i++) {
                reg->txd[i] = msg->data[i];
                outb(reg->txd[i],ioaddr + SJA1000_TX_DATA + i);
            }
            
            // command a transmit
            reg->cm = 0x01; // transmit command
            outb(reg->cm,ioaddr + SJA1000_COMMAND);
            
            // mark transmitter busy
            pdata->tx_flag = 1;
            
            // ok, message sent
            status = sizeof(can_msg_t);
        }
    }
    return status;
}

static inline void sja1000_hw_txint(sja1000_pdata_t *pdata)
{
    can_msg_t     msg;
    int           count;
    unsigned long flags;
    
    // lock interrupts while checking status
    spin_lock_irqsave(&pdata->tx_lock,flags);
    
    // try to get another message
    count = can_get_msg(&pdata->tx_dataq,&msg,0);
    
    // was there a message?
    if (count > 0) {
        // yes
        // mark transmitter not busy
        pdata->tx_flag = 0;
        
        // send the message (marks transmitter busy)
        sja1000_hw_tx(pdata,&msg);
    }
    else {
        // no, mark transmit interrupts done
        pdata->tx_flag = 0;
    }
    
    // lock interrupts while checking status
    spin_unlock_irqrestore(&pdata->tx_lock,flags);
}

// bottom half reader wakeup tasklet
// defers the wakeup processing to process level
static void sja1000_hw_rx_task(unsigned long p)
{
    sja1000_pdata_t *pdata = (sja1000_pdata_t *)p;
    
    wake_up_interruptible(&pdata->rx_wait);
}

// receive interrupt handler
static inline void sja1000_hw_rxint(sja1000_pdata_t *pdata)
{
    int                 ioaddr = pdata->hw_addr;
    sja1000_register_t *reg    = &pdata->hw_reg;
    can_msg_t           msg;
    int i;
    
    // disable receive interrupts
    reg->cr &= ~0x02;
    outb(reg->cr,ioaddr + SJA1000_CONTROL);
    
    // read all available messages into the queue
    for(;;) {
        reg->sr = inb(ioaddr + SJA1000_STATUS);
        if ((reg->sr & 0x01) == 0) {
            // no more messages in receive fifo
            break;
        }
    
        // get message id and length
        reg->rxid0 = inb(ioaddr + SJA1000_RX_ID0);
        reg->rxid1 = inb(ioaddr + SJA1000_RX_ID1);
        
        msg.id  = (reg->rxid0 << 3) | (reg->rxid1 >> 5) ;
        msg.len = reg->rxid1 & 0x0f; 
        
        // data bytes
        for(i=0;i<msg.len;i++) {
            msg.data[i] = reg->rxd[i] = inb(ioaddr + SJA1000_RX_DATA + i);
        }
        
        // release the receive buffer
        reg->cr = 0x04; 
        outb(reg->cr,ioaddr + SJA1000_COMMAND);
        
        // insert the message into the receive queue
        // if the queue fills then subsequent messages are discarded
        can_put_msg(&pdata->rx_dataq,&msg,&pdata->rx_lock);
        
        // count packets
        pdata->stats.rxpkt++;
    }
    
    // post a tasklet to wake up readers
    tasklet_schedule(&pdata->rx_task);
    
    // enable receive interrupts
    reg->cr |= 0x02;
    outb(reg->cr,ioaddr + SJA1000_CONTROL);
}


// count error events
static inline void sja1000_hw_errint(sja1000_pdata_t *pdata)
{
    can_msg_t  msg;
    int        i;
    
    sja1000_register_t *reg = &pdata->hw_reg;
    reg->sr = inb(pdata->hw_addr + SJA1000_STATUS);
    
    // if busoff error
    if (reg->sr & 0x80) {
        // bus off
        pdata->stats.busoff++;
        
        // send a bus off error message to readers
        // device should be reset
        msg.id  = 0x80000000;
        msg.len = 0; 
        
        // data bytes
        for(i=0;i<msg.len;i++) {
            msg.data[i] = 0;
        }
        
        // insert the message into the receive queue
        // if the queue fills then subsequent messages are discarded
        can_put_msg(&pdata->rx_dataq,&msg,&pdata->rx_lock);
        
        // post a tasklet to wake up readers
        tasklet_schedule(&pdata->rx_task);
        
        // stop the hardware
        sja1000_hw_stop(pdata->hw_addr,&pdata->hw_reg);
    }
    // if passive error
    else if (reg->sr & 0x40) {
        // max hardware error count
        pdata->stats.errcnt++;
        
        // send a passive tx error message to readers
        // device should be reset
        // send a bus off error message to readers
        // device should be reset
        msg.id  = 0x40000000;
        msg.len = 0; 
        
        // data bytes
        for(i=0;i<msg.len;i++) {
            msg.data[i] = 0;
        }
        
        // insert the message into the receive queue
        // if the queue fills then subsequent messages are discarded
        can_put_msg(&pdata->rx_dataq,&msg,&pdata->rx_lock);
        
        // post a tasklet to wake up readers
        tasklet_schedule(&pdata->rx_task);
    }
    // other error, do nothing, usually an error isr with status register indicating only transmit complete
}

static inline void sja1000_hw_ovrint(sja1000_pdata_t *pdata)
{
    // clear the overrun
    pdata->hw_reg.cr = 0x08; 
    outb(pdata->hw_reg.cr,pdata->hw_addr + SJA1000_COMMAND);
}

/**
 * interrupt handler
 */
irqreturn_t sja1000_int_handler(int irq,void *p,struct pt_regs *dreg)
{
    sja1000_pdata_t *pdata = (sja1000_pdata_t *)p;
    u8               ir;
    
    // total interrupt count
    pdata->stats.isr++;
    
    // get interrupt status register value
    ir = pdata->hw_reg.ir = inb(pdata->hw_addr + SJA1000_INTERRUPT);
    if (ir & 0x01) {
        // receive
        sja1000_hw_rxint(pdata);
        pdata->stats.rxisr++;
    }
    
    if (ir & 0x02) {
        // transmit
        sja1000_hw_txint(pdata);
        pdata->stats.txisr++;
    }
    
    if (ir & 0x04) {
        // error
        sja1000_hw_errint(pdata);
        pdata->stats.errisr++;
    }
    
    if (ir & 0x08) {
        // overrun
        sja1000_hw_ovrint(pdata);
        pdata->stats.ovrisr++;
    }
    
    return IRQ_HANDLED;
}

// ====================================================================================
// END - HARDWARE SPECIFIC DATA AND FUNCTIONS
// ====================================================================================
/**
 * memory region flag
 * 0 = iomem allocation failed
 * nonzero = succeeded
 */
static void *sja1000_ioaddr_flag = 0;

/**
 * registered flag
 * 0 == registered
 * nonzero = failed
 */
static int sja1000_reg_status = -EBUSY;

/**
 * device major/minor number
 */
 static dev_t sja1000_dev_t = 0;

/**
 * cdev flag
 * 0 == cdev added
 * nonzero = failed
 */
static int sja1000_cdev_status = -EBUSY;

/** device fileops */
static struct file_operations sja1000_ops = {
        .owner   =       THIS_MODULE,
        .open    =       sja1000_open,
        .read    =       sja1000_read,
        .write   =       sja1000_write,
        .release =       sja1000_release,
        .ioctl   =       sja1000_ioctl,
};

/** device cdev */
static struct cdev sja1000_cdev = {
        .kobj   =       {.name = SJA1000_MODULE_NAME, },
        .owner  =       THIS_MODULE,
};
/***************************************************************************************
 * module functions
 ***************************************************************************************/
/** io memory map base (input parameter) */
static int ioaddr = 0x0;
module_param(ioaddr,int,0);

/** irq (input parameter) */
static int irq = 0;
module_param(irq,int,0);


/**
 * print debug stats
 */
void sja1000_status(sja1000_pdata_t *pdata)
{
    // print statistics
    printk(KERN_INFO HDR "isr:%lu rxi:%lu rxp:%lu txi:%lu txp:%lu txs:%lu txb:%lu eri:%lu ovi:%lu bof:%lu ect:%lu\n",
            pdata->stats.isr,
            pdata->stats.rxisr,
            pdata->stats.rxpkt,
            pdata->stats.txisr,
            pdata->stats.txpkt,
            pdata->stats.txsnt,
            pdata->stats.txblck,
            pdata->stats.errisr,
            pdata->stats.ovrisr,
            pdata->stats.busoff,
            pdata->stats.errcnt
            );
}

/**
 * clean up module resources
 */
static void sja1000_cleanup(void)
{
    int i;
    sja1000_pdata_t *pdata;

    // cleanup the device data
    for(i=0;i<SJA1000_MINOR_NUMBERS;i++) {
        // point at private data for this minor number
        pdata = &sja1000_pdata[i];
        
        // print statistics
        sja1000_status(pdata);
        
        if (pdata != 0) {
            // de-init the actual hardware
            sja1000_hw_stop(pdata->hw_addr,&pdata->hw_reg);
        
            // remove the interrupt handler
            free_irq(pdata->hw_irq,pdata);
        }
        
        // kill the receive tasklet
        tasklet_kill(&pdata->rx_task);
    }
    // delete the cdev
    if (sja1000_cdev_status == 0) {
        cdev_del(&sja1000_cdev);
    }
    
    // release the device
    if (sja1000_reg_status == 0) {
        unregister_chrdev_region(sja1000_dev_t,SJA1000_MINOR_NUMBERS);
    }
    
    // release the io addr region
    if (sja1000_ioaddr_flag != 0) {
        release_region(ioaddr,SJA1000_IOADDR_SIZE);
    }
}

/**
 * module initialization
 */
static int sja1000_init(void)
{
    int i;
    sja1000_pdata_t *pdata;
    int status;
    
    if (ioaddr == 0) {
        printk(KERN_ALERT HDR "must specify ioaddr=0xnnn\n");
        return -EBUSY;
    }
        
    if (irq == 0) {
        printk(KERN_ALERT HDR "must specify irq=n\n");
        return -EBUSY;
    }
        
    printk(KERN_ALERT HDR "load ioaddr=0x%02x irq=%d\n",ioaddr,irq);
    
    // get the io port range
    sja1000_ioaddr_flag = request_region(ioaddr,SJA1000_IOADDR_SIZE,SJA1000_MODULE_NAME);
    if (sja1000_ioaddr_flag == 0) {
        printk(KERN_ALERT HDR "can't allocate ioaddr at 0x%02x\n",ioaddr);
        return -EBUSY;
    }
    
    // register the device
    sja1000_reg_status = alloc_chrdev_region(&sja1000_dev_t,0,SJA1000_MINOR_NUMBERS,SJA1000_MODULE_NAME); 
    if (sja1000_reg_status != 0) {
        // failed
        printk(KERN_ALERT HDR "can't be registered\n");
        sja1000_cleanup();
        return sja1000_reg_status;
    }
    
    // initialize the cdev
    cdev_init(&sja1000_cdev,&sja1000_ops);
    
    // add the cdev
    sja1000_cdev_status = cdev_add(&sja1000_cdev,sja1000_dev_t,1);
    if (sja1000_cdev_status != 0) {
        // failed
        printk(KERN_ALERT HDR "can't add cdev\n");
        sja1000_cleanup();
        return sja1000_cdev_status;
    }
    
    // clear the device structures
    memset(sja1000_pdata,0,sizeof(sja1000_pdata));
    
    // initialize the device data
    for(i=0;i<SJA1000_MINOR_NUMBERS;i++) {
        // point at private data for this minor number
        pdata = &sja1000_pdata[i];
        
        // set the multiprocess flag
        atomic_set(&pdata->is_open,1);
        
        // init the wait queues
        init_waitqueue_head(&pdata->rx_wait);
        init_waitqueue_head(&pdata->tx_wait);
        
        // init the buffers
        can_init_queue(&pdata->rx_dataq);
        can_init_queue(&pdata->tx_dataq);
        
        // init receive tasklet
        tasklet_init(&pdata->rx_task,sja1000_hw_rx_task,(unsigned long)pdata);
        
        // init spinlocks
        spin_lock_init(&pdata->tx_lock);
        spin_lock_init(&pdata->rx_lock);
        
        // not transmitting
        pdata->tx_flag = 0;

        // init hardware parameters
        // WARNING : the input params only allow 1 board to be specified
        // they must be changed to the array style when more than 1 is to be supported
        memset(&pdata->hw_reg,0,sizeof(struct sja1000_register_t));
        pdata->hw_addr = ioaddr;
        pdata->hw_irq  = irq;
        
        // set up the interrupt handler
        status = request_irq(irq,sja1000_int_handler,0,SJA1000_DEVICE_NAME,pdata);
        if (status == 0) {
            // init the actual hardware
            sja1000_hw_init(pdata->hw_addr,&pdata->hw_reg);
        }
        else {
            printk(KERN_ALERT HDR "IRQ busy\n");
            status = -EBUSY;
        }
    }
         
    return status;
}

/**
 * module exit
 */
static void sja1000_exit(void)
{
    sja1000_cleanup();
    printk(KERN_ALERT HDR "unload\n");
}

/**
 * read from the device
 */
ssize_t sja1000_read(struct file *filp,char *buffer,size_t count, loff_t *f_pos) 
{
    sja1000_pdata_t *pdata;
    can_msg_t    msg;
    int                      status;
    
    // get the private data
    pdata = (sja1000_pdata_t *)filp->private_data;
    
    // buffer size must be == size of 1 can message
    if (count != sizeof(can_msg_t)) {
        return -EINVAL;
    }
    
    // wait here until a msg is available
    status = wait_event_interruptible(pdata->rx_wait,can_get_msg(&pdata->rx_dataq,&msg,&pdata->rx_lock));

    // got one, copy it
    copy_to_user(buffer,&msg,count);
    
    return count; 
}

/**
 * write to the device
 */
ssize_t sja1000_write(struct file *filp,const char *buffer,size_t count,loff_t *f_pos) 
{
    sja1000_pdata_t *pdata;
    can_msg_t    msg;
    size_t           rcount;
    unsigned long    flags;
    
    // buffer size must be == size of 1 can message
    if (count != sizeof(can_msg_t)) {
        return -EINVAL;
    }
    
    // get the private data
    pdata = (sja1000_pdata_t *)filp->private_data;
    
    // get the user mode data which must be 16 bytes
    rcount = copy_from_user(&msg,buffer,count);
    if (rcount == 0) {
        // OK
        f_pos += count;
    } else {
        count = -EFAULT;
    }
    
    // send msg or queue it up for later transmit 
    spin_lock_irqsave(&pdata->tx_lock,flags);
    sja1000_hw_tx(pdata,&msg);
    spin_unlock_irqrestore(&pdata->tx_lock,flags);
    
    // let readers eval the condition
    wake_up_interruptible(&pdata->rx_wait);
    
    return count;
}

/**
 * execute IOCTL calls
 */
int sja1000_ioctl(struct inode *inode,struct file *filp,unsigned int cmd,unsigned long arg)
{
    sja1000_pdata_t *pdata;
    int              status = 0;
    
    // get the private data
    pdata = (sja1000_pdata_t *)filp->private_data;
    
    // perform the IOCTL operation    
    switch(cmd) {
    case CAN_IOCTL_PRINTK:
        // print stats
        sja1000_status(pdata);
        sja1000_kreg(pdata->hw_addr);
        status = 0;
        break;
    case CAN_IOCTL_STATS:
        // return stats (arg must point to a data area big enough for the stats
        copy_to_user((void *)arg,&pdata->stats,sizeof(can_stats_t));
        status = 0;
        break;
    case CAN_IOCTL_RESET:
        // reset the device hardware
        sja1000_hw_init(pdata->hw_addr,&pdata->hw_reg);
        status = 0;
        break;
    default:
        // not implemented
        status = -ENOSYS;
        break;
    }
    
    return status;
}

/**
 * open an instance of the device
 */
int sja1000_open(struct inode *inode,struct file *filp)
{
    sja1000_pdata_t *pdata;
    int minor;
    int status;
    
    // get the appropriate minor device structure
    minor = MINOR(inode->i_rdev);
    if (minor < SJA1000_MINOR_NUMBERS) {
        filp->private_data = pdata = &sja1000_pdata[minor];
    }
    else {
        printk(KERN_INFO "Philips CAN SJA1000 invalid device minor number\n");
        return -EINVAL;
    }
    
    // check reference count limiting one open handle
    status = atomic_dec_and_test(&pdata->is_open);
    if (status) {
        // we got the device
        status = 0;
    }
    else {
        // device is already open, decrement and return busy
        atomic_inc(&pdata->is_open);
        status = -EBUSY;
    }
    
    return status;
}

/**
 * close the instance
 */
int sja1000_release(struct inode *inode,struct file* filp)
{
    sja1000_pdata_t *pdata;
    
    // get the private data
    pdata = (sja1000_pdata_t *)filp->private_data;

    // print statistics
    sja1000_status(pdata);
    
    // increment reference count
    atomic_inc(&pdata->is_open);
    
    return 0;
}

// ==========================================================================
// module linkups
// ==========================================================================
module_init(sja1000_init);
module_exit(sja1000_exit);


