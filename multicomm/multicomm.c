#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/ioport.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/serial_reg.h>
#include <asm/ioctls.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/semaphore.h>
#include <asm/system.h>
#include <asm/termios.h>
#include "serqueue.h"
#include "multicomm.h"

/**
 * SNC Multicomm Driver
 * @author david howard
 * copyright (C) 2004 David Howard
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

#define HDR                      "SNC Serial driver for Parvus Multicomm " 
#define MULTICOMM_MODULE_NAME    "multicomm"
#define MULTICOMM_DEVICE_NAME    "multicomm"
#define MULTICOMM_MINOR_NUMBERS  8
#define C16550_FIFO_SIZE         16
/***************************************************************************************
 * forward references
 ***************************************************************************************/
ssize_t multicomm_read(struct file *,char *,size_t, loff_t *);
ssize_t multicomm_write(struct file *,const char *,size_t,loff_t *);
int     multicomm_ioctl(struct inode *,struct file *,unsigned int,unsigned long);
int     multicomm_open(struct inode *,struct file *);
int     multicomm_release(struct inode *,struct file*);
 
/***************************************************************************************
 * module variables
 ***************************************************************************************/

/***************************************************************************************
 * local data structures
 ***************************************************************************************/
 
 typedef struct c16550_t {
    /** driver parameters */
    int           hw_addr;    /** base address of 16550 uart */
    int           hw_irq;     /** desired IRQ for 16550 uart */
    
     /* device shadow registers */
    unsigned char lcr;
    unsigned char ier;
    unsigned char mcr;
    unsigned char fcr;
    unsigned char dll;
    unsigned char dlm;
 } c16550_t;


/***************************************************************************************
 * per device private data
 ***************************************************************************************/
typedef struct multicomm_pdata_t {
    int                          minor;      /** device minor number */
    int                          is_open;    /** flag indicating driver is open or not */
    spinlock_t                   lock;       /** spin lock for interrupt control */
    
    wait_queue_head_t            rx_wait;       /** read wait structures */
    ser_buffer_t                 rx_dataq;      /** circular buffer for reading */
    struct tasklet_struct        rx_task;       /** receive tasklet */
    
    wait_queue_head_t            tx_wait;     /** write wait structures */
    ser_buffer_t                 tx_dataq;    /** circular buffer for writing */
    int                          baud;        /** requested baud rate */
    
    c16550_t                     uart;       /** hardware data */
    ser_stats_t                  stats;      /** tx/rx/err stats */
    
} multicomm_pdata_t;

// board definition
typedef struct multicomm_board_t {
    // board global data
    
    // module registered
    int               reg;
    int               reg_flag;
    
    // device allocated
    dev_t             dev;
    int               dev_flag; 
    
    // cdev
    struct cdev       cdev;
    int               cdev_flag;

    // interrupt allocated
    int               irq;
    int               irq_flag;
    
    // interrupt status register io port allocated
    int               isr;                                      // interrupt status register
    void             *isr_flag;
    unsigned long     isrct; // interrupt count
    
    // base io address allocated
    int               ioaddr;
    void             *ioaddr_flag;
    
    // per unit data
    multicomm_pdata_t pdata[MULTICOMM_MINOR_NUMBERS]; // one 'port data' per port
} multicomm_board_t;

static multicomm_board_t multicomm_board;

// ====================================================================================
// HARDWARE SPECIFIC DATA AND FUNCTIONS
// ====================================================================================
#define MULTICOMM_IOADDR_SIZE     (MULTICOMM_MINOR_NUMBERS * 0x08)

/***************************************************************************************
 * QUAD 16550C MULTICOMM hardware access
 ***************************************************************************************/
 
static void multicomm_kreg(c16550_t *uart)
{
    printk(KERN_INFO HDR "ISR:%02x adr:%3x irq:%d lcr:%02x fcr:%02x mcr:%02x ier:%02x dll:%02x dlm:%02x\n",
                    inb(multicomm_board.isr),
                    uart->hw_addr,
                    uart->hw_irq,
                    uart->lcr,
                    uart->fcr,
                    uart->mcr,
                    uart->ier,
                    uart->dll,
                    uart->dlm
                    );

}

static void multicomm_hw_baud(c16550_t *uart,int baud)
{
    int div;
    
    if (baud < 50) {
        return;
    }
    
    /* Enable access to the divisor latches by setting DLAB in LCR. */
    uart->lcr |= UART_LCR_DLAB;
    outb(uart->lcr,uart->hw_addr + UART_LCR);

    // clock independent computation
    div = 115200 / baud;
    uart->dll = (div & 0x00ff) >> 0;
    uart->dlm = (div & 0xff00) >> 8;
    outb(uart->dll,uart->hw_addr + UART_DLL);
    outb(uart->dlm,uart->hw_addr + UART_DLM);
    
    /* clear access to divisor latches */
    uart->lcr &= ~UART_LCR_DLAB;
    outb(uart->lcr,uart->hw_addr + UART_LCR);
}

static void multicomm_hw_init(c16550_t *uart,int baud)
{
    int  base;

    /* init to hardcoded 8 bits per character, 1 stop bit, no parity */
    /* no modem control */
    /* fixed interrupt level */
    uart->mcr = 0;
    uart->lcr = 0;
    uart->fcr = 0;
    uart->ier = 0;
    uart->dll = 0;
    uart->dlm = 0;
    

    /* base address is set in the table */
    base = uart->hw_addr;

    /* initialize device to quiescent state */
    /* Configure Port -  Set 8 bits, 1 stop bit, no parity. */
    
    /* clear all read-registers */
	(void) inb(base + UART_LSR);
	(void) inb(base + UART_RX);
	(void) inb(base + UART_IIR);
	(void) inb(base + UART_MSR);

    /* Disable interrupts */
    uart->ier = 0;
    outb(uart->ier,base + UART_IER);

    /* Reset the FIFOs */
    uart->fcr = UART_FCR_ENABLE_FIFO; 
    outb(uart->fcr,base + UART_FCR);
    uart->fcr = UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT | UART_FCR_ENABLE_FIFO; 
    outb(uart->fcr,base + UART_FCR);

    /* Enable the FIFOs for trigger at 8 bytes */
    uart->fcr = UART_FCR_ENABLE_FIFO | UART_FCR_TRIGGER_8; 
    outb(uart->fcr,base + UART_FCR);

    /* set Line Control Register */
    uart->lcr = UART_LCR_WLEN8;
    outb(uart->lcr,base + UART_LCR);

    /* Set Modem control register */
    uart->mcr = UART_MCR_OUT2 | UART_MCR_RTS;
    outb(uart->mcr,base + UART_MCR);

    /* Set divisor latches to set baud rate */
    multicomm_hw_baud(uart,baud);

    /* initialize shadow  Interrupt Enable Register */
    uart->ier = UART_IER_RDI;

    /* clear all other registers */
	(void) inb(base + UART_LSR);
	(void) inb(base + UART_RX);
	(void) inb(base + UART_IIR);
	(void) inb(base + UART_MSR);
}

static void multicomm_hw_open(c16550_t *uart)
{
    int  base;

    base = uart->hw_addr;
    
    // clear anything pending
	(void) inb(base + UART_LSR);
	(void) inb(base + UART_RX);
	(void) inb(base + UART_IIR);
	(void) inb(base + UART_MSR);
    
    // enable preconfigured interrupts  
    uart->ier = UART_IER_RDI;
    outb(uart->ier,base + UART_IER);
}

// cpu register definitions
static inline void multicomm_hw_stop(c16550_t *uart)
{
    /* disable interrupts */
    uart->ier = 0;
    outb(uart->ier,uart->hw_addr + UART_IER);
}

// start transmit
static inline void multicomm_hw_start_tx(c16550_t *uart)
{
    // start interrupts if transmit interrupt is not enabled
    if ((uart->ier & UART_IER_THRI) == 0) {
        // enable transmit interrupts
        uart->ier |= UART_IER_THRI;
        outb(uart->ier,uart->hw_addr + UART_IER);
    }
}

// MUST BE CALLED WITH SPINLOCK LOCKED
static inline void multicomm_hw_txint(multicomm_pdata_t *pdata)
{
    unsigned char ch;
    int           count;
    c16550_t     *uart = &pdata->uart;
    int           i;

    i = 0;
    do {    
        // send the next character if there is one or disable transmit interrupts if no more data
        // let the serial queue lock around each access to the queue
        count = ser_get(&pdata->tx_dataq,&ch,1, &pdata->lock);
        if (count == 1) {
            // send the byte
            outb(ch,uart->hw_addr + UART_TX);
            
            // count output
            pdata->stats.txchr++;
        }
        else {
            if (uart->ier & UART_IER_THRI) {
                // no more data
                uart->ier &= ~UART_IER_THRI;
                outb(uart->ier,uart->hw_addr + UART_IER);
            }
            break;
        }
        
        i++;
    } while(i < C16550_FIFO_SIZE);
    
    // post a tasklet to wake up readers
    tasklet_schedule(&pdata->rx_task);
}

// bottom half reader wakeup tasklet
// defers the wakeup processing to process level
static void multicomm_hw_rx_task(unsigned long p)
{
    multicomm_pdata_t *pdata = (multicomm_pdata_t *)p;
    
    wake_up_interruptible(&pdata->rx_wait);
}

// receive interrupt handler
// MUST BE CALLED WITH SPINLOCK LOCKED
static inline void multicomm_hw_rxint(multicomm_pdata_t *pdata,unsigned char lsr)
{
    int           dr;
    int           i;
    int           count;
    int           rcount;
    unsigned char buf[16];
    c16550_t     *uart = &pdata->uart;
    
    // get all available bytes (at least one is available coming in) 
    count = 0;
    for(i=0;i<C16550_FIFO_SIZE;i++) {
        /* read character from Receive Holding Reg. */
        buf[i] = inb(uart->hw_addr + UART_RX);
        
        // count them
        count++;
        
        /* reread line status register and quit when there is no more data */
        dr = inb(uart->hw_addr + UART_LSR) & UART_LSR_DR;
        if (dr == 0) break;
    }
    
    // enqueue the bytes, drop any if buffer is full
    rcount = ser_put(&pdata->rx_dataq,buf,count,&pdata->lock);
    if (rcount != count) {
        pdata->stats.rxovr++;
    }
    
    // store current receive buffer count
    pdata->stats.rxbuf = pdata->rx_dataq.count;
    
    // count the bytes
    pdata->stats.rxchr += rcount;
    
    // post a tasklet to wake up readers
    tasklet_schedule(&pdata->rx_task);
}

// count error events
// MUST BE CALLED WITH SPINLOCK LOCKED
static inline void multicomm_hw_errint(multicomm_pdata_t *pdata)
{
    // do nothing, interrupt is cleared
}

static inline void multicomm_port_int_handler(multicomm_pdata_t *pdata)
{
    c16550_t *uart = &pdata->uart;
    unsigned char lsr;
    unsigned char iir;
    // unsigned long flags;
    
    pdata->stats.isr++;
    iir = inb(uart->hw_addr + UART_IIR) & UART_IIR_ID;

    // if an interrupt is pending (IIR bit 0 == 0)
    if ((iir & UART_IIR_NO_INT) == 0) {
        // service pending interrupts for this port
        lsr = inb(uart->hw_addr + UART_LSR);
        
        if (lsr & UART_LSR_DR) {
            // receive interrupt
            multicomm_hw_rxint(pdata,lsr);
            pdata->stats.rxisr++;
        }
        if (lsr & UART_LSR_THRE) {
            // transmit interrupt
            multicomm_hw_txint(pdata);
            pdata->stats.txisr++;
        }
    }
}

/**
 * interrupt handler
 */
irqreturn_t multicomm_int_handler(int irq,void *p,struct pt_regs *dreg)
{
    multicomm_board_t *board = (multicomm_board_t *)p;
    int isr;
    int mask;
    int port;
    int i;
    
    board->isrct++;
    
    // continuously iterate thru all 8 devices on the board until no more interrupts
    i = 0;
    for(;;) {
        // read board interrupt status register
        isr  = inb(board->isr);
        if (isr == 0xff) {
            // no more interrupts
            break;
        }
        
        // service ports that have an interrupt
        mask = 0x01;
        for(port=0;port<8;port++) {
            // process this port
            if ((isr & mask) == 0) {
                // bit is 0, service the interrupt
                multicomm_port_int_handler(&board->pdata[port]);
            }
            mask <<= 1;
        }
        
        // limit max iterations
        i++;
        if (i >= 16) {
            printk(KERN_ALERT HDR "too many interrupts\n");
            break;
        }
    }
    
    return IRQ_HANDLED;
}

// read modem control lines
static unsigned int multicomm_hw_mcr_get(c16550_t *uart)
{
    unsigned int ret;
    int          msr;
    
    // read msr
    msr = inb(uart->hw_addr + UART_MSR);
    
    // set output bits
    ret = 0;
    if (msr & UART_MSR_DCD)
        ret |= TIOCM_CAR;
    if (msr & UART_MSR_RI)
        ret |= TIOCM_RNG;
    if (msr & UART_MSR_DSR)
        ret |= TIOCM_DSR;
    if (msr & UART_MSR_CTS)
        ret |= TIOCM_CTS;
    
    // add outputs also
    if (uart->mcr & UART_MCR_RTS)
        ret |= TIOCM_RTS;
    if (uart->mcr & UART_MCR_DTR)
        ret |= TIOCM_DTR;
    
    return ret;
}

// write modem control lines (set bits)
static void multicomm_hw_mcr_set(c16550_t *uart,unsigned long mctrl)
{
    unsigned char mcr = 0;

    /* Set Modem control register */
    mcr = uart->mcr;
    
    if (mctrl & TIOCM_RTS)
        mcr |= UART_MCR_RTS;
    if (mctrl & TIOCM_DTR)
        mcr |= UART_MCR_DTR;
    if (mctrl & TIOCM_OUT1)
        mcr |= UART_MCR_OUT1;
    if (mctrl & TIOCM_OUT2)
        mcr |= UART_MCR_OUT2;
    if (mctrl & TIOCM_LOOP)
        mcr |= UART_MCR_LOOP;
    
    printk(KERN_INFO HDR "set:%08lx %02x\n",mctrl,mcr);
    
    uart->mcr = mcr;

    outb(uart->mcr,uart->hw_addr + UART_MCR);
}

// write modem control lines (clr bits)
static void multicomm_hw_mcr_clr(c16550_t *uart,unsigned long mctrl)
{
    unsigned char mcr = 0;

    /* Set Modem control register */
    mcr = uart->mcr;
    
    if (mctrl & TIOCM_RTS)
        mcr &= ~UART_MCR_RTS;
    if (mctrl & TIOCM_DTR)
        mcr &= ~UART_MCR_DTR;
    if (mctrl & TIOCM_OUT1)
        mcr &= ~UART_MCR_OUT1;
    if (mctrl & TIOCM_OUT2)
        mcr &= ~UART_MCR_OUT2;
    if (mctrl & TIOCM_LOOP)
        mcr &= ~UART_MCR_LOOP;
    
    printk(KERN_INFO HDR "clr:%08lx %02x\n",mctrl,mcr);
    
    uart->mcr = mcr;

    outb(uart->mcr,uart->hw_addr + UART_MCR);
}

// ====================================================================================
// END - HARDWARE SPECIFIC DATA AND FUNCTIONS
// ====================================================================================

/** device fileops */
static struct file_operations multicomm_ops = {
        .owner   =       THIS_MODULE,
        .open    =       multicomm_open,
        .read    =       multicomm_read,
        .write   =       multicomm_write,
        .release =       multicomm_release,
        .ioctl   =       multicomm_ioctl,
};

/***************************************************************************************
 * module parameters
 ***************************************************************************************/
/** 16550 io memory map base */
static int ioaddr = 0x0;
module_param(ioaddr,int,0);

/** irq */
static int irq = 0;
module_param(irq,int,0);

/** interrupt status register */
static int isr = 0;
module_param(isr,int,0);

// initial baud rates
static int baud[8]    = {9600,9600,9600,9600,9600,9600,9600,9600};
static int baud_len   = 8;
module_param_array(baud,int,baud_len,0);

/***************************************************************************************
 * module functions
 ***************************************************************************************/
/**
 * print debug stats
 */
void multicomm_status(multicomm_pdata_t *pdata)
{
    // print statistics
    printk(KERN_INFO HDR "minor:%d baud:%d\n",pdata->minor,pdata->baud);
    printk(KERN_INFO HDR "isr:%lu rxi:%lu rxc:%lu rxo:%lu txi:%lu txc:%lu eri:%lu\n",
            pdata->stats.isr,
            pdata->stats.rxisr,
            pdata->stats.rxchr,
            pdata->stats.rxovr,
            pdata->stats.txisr,
            pdata->stats.txchr,
            pdata->stats.errisr
            );
}

/**
 * clean up module resources
 */
static void multicomm_cleanup(multicomm_board_t *board)
{
    int i;
    multicomm_pdata_t *pdata;

    //
    // cleanup the device data
    for(i=0;i<MULTICOMM_MINOR_NUMBERS;i++) {
        // point at private data for this minor number
        pdata = &board->pdata[i];
        
        // print statistics
        multicomm_status(pdata);
        
        if (pdata != 0) {
            // de-init the actual hardware
            multicomm_hw_stop(&pdata->uart);
        }
        
        // kill the receive tasklet
        tasklet_kill(&pdata->rx_task);
    }
    
    // remove the interrupt handler
    free_irq(board->irq,board);
    
    // delete the cdev
    if (board->cdev_flag == 0) {
        cdev_del(&board->cdev);
    }
    
    // release the device
    if (board->reg_flag == 0) {
        unregister_chrdev_region(board->dev,MULTICOMM_MINOR_NUMBERS);
    }
    
    // release the ISR io addr
    if (board->isr_flag != 0) {
        release_region(board->isr,1);
    }
    
    // release the 16550 io addr region
    if (board->ioaddr_flag != 0) {
        release_region(board->ioaddr,MULTICOMM_IOADDR_SIZE);
    }
}

/**
 * module initialization
 */
static int multicomm_init(void)
{
    int i;
    multicomm_board_t *board = &multicomm_board;
    multicomm_pdata_t *pdata;
    int status;
    
    memset(board,0,sizeof(multicomm_board_t));
    
    if (ioaddr == 0) {
        printk(KERN_ALERT HDR "must specify ioaddr=0xnnn\n");
        return -EBUSY;
    }
        
    if (irq == 0) {
        printk(KERN_ALERT HDR "must specify irq=n\n");
        return -EBUSY;
    }
        
    if (isr == 0) {
        printk(KERN_ALERT HDR "must specify isr=n\n");
        return -EBUSY;
    }
    
    printk(KERN_ALERT HDR "load ioaddr=0x%03x irq=%d isr=%03x\n",ioaddr,irq,isr);
    
    board->ioaddr = ioaddr;
    board->irq    = irq;
    board->isr    = isr;
    
    // get the 16550 io port range
    board->ioaddr_flag = request_region(board->ioaddr,MULTICOMM_IOADDR_SIZE,MULTICOMM_MODULE_NAME);
    if (board->ioaddr_flag == 0) {
        printk(KERN_ALERT HDR "can't allocate ioaddr at 0x%03x\n",board->ioaddr);
        return -EBUSY;
    }

    // get the multicomm ISR io port range
    board->isr_flag = request_region(isr,1,MULTICOMM_MODULE_NAME);
    if (board->isr_flag == 0) {
        printk(KERN_ALERT HDR "can't allocate isr at 0x%03x\n",isr);
        return -EBUSY;
    }
    
    // register the device
    board->reg_flag = alloc_chrdev_region(&board->dev,0,MULTICOMM_MINOR_NUMBERS,MULTICOMM_MODULE_NAME); 
    if (board->reg_flag != 0) {
        // failed
        printk(KERN_ALERT HDR "can't be registered\n");
        multicomm_cleanup(board);
        return board->reg_flag;
    }
    
    // initialize the cdev
    cdev_init(&board->cdev,&multicomm_ops);
    kobject_set_name(&board->cdev.kobj, MULTICOMM_MODULE_NAME);
    board->cdev.owner = THIS_MODULE;
  
    // add the cdev
    board->cdev_flag = cdev_add(&board->cdev,board->dev,MULTICOMM_MINOR_NUMBERS);
    if (board->cdev_flag != 0) {
        // failed
        printk(KERN_ALERT HDR "can't add cdev\n");
        multicomm_cleanup(board);
        return board->cdev_flag;
    }
    
    // set up the interrupt handler
    status = request_irq(irq,multicomm_int_handler,0,MULTICOMM_DEVICE_NAME,(void *)board);
    if (status != 0) {
        printk(KERN_ALERT HDR "IRQ busy\n");
        multicomm_cleanup(board);
        status = -EBUSY;
    }
    
    // initialize the device data
    for(i=0;i<MULTICOMM_MINOR_NUMBERS;i++) {
        // point at private data for this minor number
        pdata = &board->pdata[i];
        
        // set pdata back index
        pdata->minor = i;
        
        // set the multiprocess flag
        pdata->is_open = 0;
        
        // init the wait queues
        init_waitqueue_head(&pdata->rx_wait);
        init_waitqueue_head(&pdata->tx_wait);
        
        // init the buffers
        ser_init_queue(&pdata->rx_dataq);
        ser_init_queue(&pdata->tx_dataq);
        
        // init receive tasklet
        tasklet_init(&pdata->rx_task,multicomm_hw_rx_task,(unsigned long)pdata);
        
        // init spinlock
        spin_lock_init(&pdata->lock);
        
        // init hardware parameters
        pdata->uart.hw_addr = ioaddr + (i * 8);
        pdata->uart.hw_irq  = irq;
        pdata->baud         = baud[i];
    }
    
    // initialize all the devices AFTER the data structures are set up
    for(i=0;i<MULTICOMM_MINOR_NUMBERS;i++) {
        // point at private data for this minor number
        pdata = &board->pdata[i];
        
        // init the hardware
        multicomm_hw_init(&pdata->uart,pdata->baud);
    }
         
    return status;
}

/**
 * module exit
 */
static void multicomm_exit(void)
{
    multicomm_cleanup(&multicomm_board);
    printk(KERN_ALERT HDR "unload\n");
}

/**
 * read from the device
 */
ssize_t multicomm_read(struct file *filp,char *ubuffer,size_t count, loff_t *f_pos) 
{
    multicomm_pdata_t *pdata;
    int                status;
    unsigned char      buf[MULTICOMM_MAX_READ];
    int                rcount;
    
    // get the private data
    pdata = (multicomm_pdata_t *)filp->private_data;
    
    // buffer size must be == size of 1 can message
    if (count >= sizeof(buf)) {
        return -EINVAL;
    }
    
    // wait here until a msg is available
    status = wait_event_interruptible(pdata->rx_wait,(rcount = ser_get(&pdata->rx_dataq,buf,count,&pdata->lock)));

    // got at least some bytes, copy them and return the actual count
    copy_to_user(ubuffer,&buf,rcount);
    
    return rcount; 
}

/**
 * write to the device
 */
ssize_t multicomm_write(struct file *filp,const char *ubuffer,size_t count,loff_t *f_pos) 
{
    multicomm_pdata_t *pdata;
    size_t             rcount;
    int                wcount;
    unsigned char      buf[MULTICOMM_MAX_WRITE];
    
    // get the private data
    pdata = (multicomm_pdata_t *)filp->private_data;
    if (pdata == 0) {
        printk(KERN_INFO HDR "no private data\n");
        return -EBUSY;
    }
    
    // buffer size must be <= size of local buffer
    if (count > sizeof(buf)) {
        printk(KERN_INFO HDR "write count too big\n");
        return -EINVAL;
    }
    
    // do nothing if no data
    if (count <= 0) {
        return 0;
    }
    
    // get the user mode data 
    rcount = copy_from_user(buf,ubuffer,count);
    if (rcount == 0) {
        // OK
        *f_pos += count;
    } else {
        printk(KERN_INFO HDR "can't read user mode data\n");
        return -EFAULT;
    }

    // copy the data into the write buffer
    wcount = ser_put(&pdata->tx_dataq,buf,count,&pdata->lock);
    
    // count transmit buffer overflows
    if (wcount != count) {
        pdata->stats.txovr++;
    }
    
    // store transmit buffer total count
    pdata->stats.txbuf = pdata->tx_dataq.count;

    // start transmitting
    multicomm_hw_start_tx(&pdata->uart);
    
    // let readers eval the condition
    wake_up_interruptible(&pdata->rx_wait);
    
    // return actual write count
    return wcount;
}

/**
 * execute IOCTL calls
 */
int multicomm_ioctl(struct inode *inode,struct file *filp,unsigned int cmd,unsigned long arg)
{
    multicomm_pdata_t *pdata;
    int                status = 0;
    unsigned long      bits;
    unsigned int       modem;
    unsigned long      flags;
    
    // get the private data
    pdata = (multicomm_pdata_t *)filp->private_data;
    
    // perform the IOCTL operation    
    switch(cmd) {
    case SER_IOCTL_SETBAUD:
        // set a new baud rate
        pdata->baud = arg;
        spin_lock_irqsave(&pdata->lock,flags);
        multicomm_hw_baud(&pdata->uart,arg);
        spin_unlock_irqrestore(&pdata->lock,flags);
        break;
    case SER_IOCTL_PRINTK:
        // print stats
        multicomm_status(pdata);
        multicomm_kreg(&pdata->uart);
        status = 0;
        break;
    case SER_IOCTL_STATS:
        // return stats (arg must point to a data area big enough for the stats
        copy_to_user((void *)arg,&pdata->stats,sizeof(ser_stats_t));
        status = 0;
        break;
    case SER_IOCTL_RESET:
        // reset the device hardware
        spin_lock_irqsave(&pdata->lock,flags);
        multicomm_hw_init(&pdata->uart,arg);
        spin_unlock_irqrestore(&pdata->lock,flags);
        status = 0;
        break;
    case TIOCMGET: // get modem control status
        modem = multicomm_hw_mcr_get(&pdata->uart);
        copy_to_user((void *)arg,&modem,sizeof(modem));
        status = 0;
        break;
    case TIOCMBIS:  //  set  modem control bit(s)
        copy_from_user(&bits,(unsigned long *)arg,sizeof(unsigned long));
        multicomm_hw_mcr_set(&pdata->uart,bits);
        status = 0;
        break;
    case TIOCMBIC:  // clear a modem control bit
        copy_from_user(&bits,(unsigned long *)arg,sizeof(unsigned long));
        multicomm_hw_mcr_clr(&pdata->uart,bits);
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
int multicomm_open(struct inode *inode,struct file *filp)
{
    multicomm_pdata_t *pdata;
    int minor;
    unsigned long flags;
    
    // get the appropriate minor device structure
    minor = MINOR(inode->i_rdev);
    if (minor < MULTICOMM_MINOR_NUMBERS) {
        filp->private_data = pdata = &multicomm_board.pdata[minor];
    }
    else {
        printk(KERN_ALERT HDR "invalid device minor number\n");
        return -EINVAL;
    }
    
    // lock interrupts
    spin_lock_irqsave(&pdata->lock,flags);
    
    // check reference count  and only initialize on the first open
    if (pdata->is_open == 0) {
        // flush the queues
        ser_init_queue(&pdata->rx_dataq);
        ser_init_queue(&pdata->tx_dataq);
        
        // enable the hardware
        multicomm_hw_open(&pdata->uart);
    }
    // increment reference count
    pdata->is_open++;
    
    // unlock interrupts
    spin_unlock_irqrestore(&pdata->lock,flags);
    
    printk(KERN_INFO HDR "open\n");
    multicomm_status(pdata);
    multicomm_kreg(&pdata->uart);    
    
    return 0;
}

/**
 * close the instance
 */
int multicomm_release(struct inode *inode,struct file* filp)
{
    multicomm_pdata_t *pdata;
    unsigned long      flags;
    
    // get the private data
    pdata = (multicomm_pdata_t *)filp->private_data;
    
    printk(KERN_INFO HDR "close\n");
    multicomm_status(pdata);
    multicomm_kreg(&pdata->uart);
    
    // lock interrupts
    spin_lock_irqsave(&pdata->lock,flags);
    // decrement and test reference count
    pdata->is_open--;
    if (pdata->is_open <= 0) {
        // last open, stop the hardware
        multicomm_hw_stop(&pdata->uart);
        
        printf(KERN_INFO HDR "stop hardware");
    }
    // unlock interrupts
    spin_unlock_irqrestore(&pdata->lock,flags);
    
    return 0;
}

// ==========================================================================
// module linkups
// ==========================================================================
module_init(multicomm_init);
module_exit(multicomm_exit);


