/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31_serial.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/serial/serial.h>

#include "arm_arch.h"
#include "arm_internal.h"

#include "registers/regsuartdbg.h"

#ifdef USE_SERIALDRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/
struct up_dev_s
{
  uint8_t  ier;       /* Saved IER value */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static inline void up_disableuartint(struct up_dev_s *priv, uint8_t *ier);
static inline void up_restoreuartint(struct up_dev_s *priv, uint8_t ier);

static int  up_setup(struct uart_dev_s *dev);
static void up_shutdown(struct uart_dev_s *dev);
static int  up_attach(struct uart_dev_s *dev);
static void up_detach(struct uart_dev_s *dev);
static int  up_interrupt(int irq, void *context, FAR void *arg);
static int  up_ioctl(struct file *filep, int cmd, unsigned long arg);
static int  up_receive(struct uart_dev_s *dev, unsigned int *status);
static void up_rxint(struct uart_dev_s *dev, bool enable);
static bool up_rxavailable(struct uart_dev_s *dev);
static void up_send(struct uart_dev_s *dev, int ch);
static void up_txint(struct uart_dev_s *dev, bool enable);
static bool up_txready(struct uart_dev_s *dev);
static bool up_txempty(struct uart_dev_s *dev);


/****************************************************************************
 * Private Data
 ****************************************************************************/






static const struct uart_ops_s g_uart_ops =
{
  .setup          = up_setup,
  .shutdown       = up_shutdown,
  .attach         = up_attach,
  .detach         = up_detach,
  .ioctl          = up_ioctl,
  .receive        = up_receive,
  .rxint          = up_rxint,
  .rxavailable    = up_rxavailable,
#ifdef CONFIG_SERIAL_IFLOWCONTROL
  .rxflowcontrol  = NULL,
#endif
  .send           = up_send,
  .txint          = up_txint,
  .txready        = up_txready,
  .txempty        = up_txempty,
};



/* I/O buffers */
#ifndef CONFIG_UART_RXBUFSIZE
#define CONFIG_UART_RXBUFSIZE 128
#endif

#ifndef CONFIG_UART_TXBUFSIZE
#define CONFIG_UART_TXBUFSIZE 128
#endif

static char g_rxbuffer[CONFIG_UART_RXBUFSIZE];
static char g_txbuffer[CONFIG_UART_TXBUFSIZE];

static struct up_dev_s g_uartpriv;
static uart_dev_t g_uartport =
{
  .recv     =
  {
    .size   = CONFIG_UART_RXBUFSIZE,
    .buffer = g_rxbuffer,
  },
  .xmit     =
  {
    .size   = CONFIG_UART_TXBUFSIZE,
    .buffer = g_txbuffer,
  },
  .ops      = &g_uart_ops,
  .priv     = &g_uartpriv,
  
};


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_disableuartint
 ****************************************************************************/

static inline void up_disableuartint(struct up_dev_s *priv, uint8_t *ier)
{
	HW_UARTDBGIMSC_CLR(0x7FF);
}

/****************************************************************************
 * Name: up_restoreuartint
 ****************************************************************************/

static inline void up_restoreuartint(struct up_dev_s *priv, uint8_t ier)
{
	HW_UARTDBGIMSC_SET(BM_UARTDBGIMSC_RTIM | BM_UARTDBGIMSC_TXIM | BM_UARTDBGIMSC_RXIM);
}




/****************************************************************************
 * Name: up_setup
 *
 * Description:
 *   Configure the UART baud, bits, parity, fifos, etc. This method is called
 *   the first time that the serial port is opened.
 *
 ****************************************************************************/

static int up_setup(struct uart_dev_s *dev)
{
//#ifndef CONFIG_SUPPRESS_UART_CONFIG

  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  uint32_t regval;
  
  
  HW_UARTDBGLCR_H_CLR( BM_UARTDBGLCR_H_SPS );
  HW_UARTDBGLCR_H_SET( BM_UARTDBGLCR_H_FEN );
  
  
  BW_UARTDBGIFLS_TXIFLSEL(BV_UARTDBGIFLS_TXIFLSEL__EMPTY);
  BW_UARTDBGIFLS_RXIFLSEL(BV_UARTDBGIFLS_RXIFLSEL__NOT_EMPTY);
				
  
  HW_UARTDBGIMSC_CLR(0x7FF);
  HW_UARTDBGIMSC_SET(BM_UARTDBGIMSC_RTIM | BM_UARTDBGIMSC_TXIM | BM_UARTDBGIMSC_RXIM);
  
  HW_UARTDBGCR_SET( BM_UARTDBGCR_RXE | BM_UARTDBGCR_TXE);
  
  HW_UARTDBGICR_SET(0x7FF);
  
  
  up_attach(dev);
  
//#endif
}

/****************************************************************************
 * Name: up_shutdown
 *
 * Description:
 *   Disable the UART.  This method is called when the serial port is closed
 *
 ****************************************************************************/

static void up_shutdown(struct uart_dev_s *dev)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  up_disableuartint(priv, NULL);

}

/****************************************************************************
 * Name: up_attach
 *
 * Description:
 *   Configure the UART to operation in interrupt driven mode.  This method
 *   is called when the serial port is opened.  Normally, this is just after
 *   the the setup() method is called, however, the serial console may operate
 *   in a non-interrupt driven mode during the boot phase.
 *
 *   RX and TX interrupts are not enabled when by the attach method (unless
 *   the hardware supports multiple levels of interrupt enabling).  The RX
 *   and TX interrupts are not enabled until the txint() and rxint() methods
 *   are called.
 *
 ****************************************************************************/

static int up_attach(struct uart_dev_s *dev)
{
  int ret;
  /* Attach and enable the IRQ */

  ret = irq_attach(STMP3770_IRQ_DEBUG_UART, up_interrupt, dev);;
  if (ret == OK)
    {
      /* Enable the interrupt (RX and TX interrupts are still disabled
       * in the UART
       */
	up_enable_irq(STMP3770_IRQ_DEBUG_UART);

    }
	//HW_UARTDBGDR_WR('A');
  return ret;
}


/****************************************************************************
 * Name: up_detach
 *
 * Description:
 *   Detach UART interrupts.  This method is called when the serial port is
 *   closed normally just before the shutdown method is called.  The
 *   exception is the serial console which is never shutdown.
 *
 ****************************************************************************/

static void up_detach(struct uart_dev_s *dev)
{ 
  up_disable_irq(STMP3770_IRQ_DEBUG_UART);
  irq_detach(STMP3770_IRQ_DEBUG_UART);
}


/****************************************************************************
 * Name: up_interrupt
 *
 * Description:
 *   This is the UART interrupt handler.  It will be invoked when an
 *   interrupt received on the UART irq.  It should call  uart_transmitchars
 *   or uart_receivechar to perform the appropriate data transfers.
 *
 ****************************************************************************/

static int up_interrupt(int irq, void *context, FAR void *arg)
{  
  struct uart_dev_s *dev = (struct uart_dev_s *)arg;
  uint8_t status;
  int passes;

  /* Loop until there are no characters to be transferred or,
   * until we have been looping for a long time.
   */

  //for (passes = 0; passes < 256; passes++)
    {
      /* Get the current UART status and check for loop
       * termination conditions
       */

       status = HW_UARTDBGMIS_RD();
		
      /* The NO INTERRUPT should be zero if there are pending
       * interrupts
       */
	   //sinfo("sint:%X\n",status);

      if ((status & 0x7FF) == 0)
        {
          /* Break out of the loop when there is no longer a pending
           * interrupt
           */
        //  break;
		  //HW_UARTDBGICR_SET(0x7FF);
		  
		  //sinfo("sint:%X\n",status);
		  //for(;;);
        }
		//else  /* Handle the interrupt by its interrupt ID field */
	  if((status & BM_UARTDBGMIS_RTMIS) != 0){
		  /* Handle incoming, receive bytes (with or without timeout) */
		  uart_recvchars(dev); 
		 // HW_UARTDBGICR_SET(BM_UARTDBGICR_RTIC);
		  
	  }
	    //else
	  if((status & BM_UARTDBGMIS_RXMIS) != 0){
		  /* Handle incoming, receive bytes (with or without timeout) */
		  uart_recvchars(dev); 
		 // HW_UARTDBGICR_SET(BM_UARTDBGICR_RXIC);
		  
	  }
	   // else
	  if((status & BM_UARTDBGMIS_TXMIS) != 0){
		  
		  uart_xmitchars(dev);
		 // HW_UARTDBGICR_SET(BM_UARTDBGICR_TXIC);
		  
	  }	  
	  
	  
	  /*
	  else{
		  serr("ERROR: Unexpected IIR: %02x\n", status);
		  HW_UARTDBGICR_SET(0x7FF);
		  //break;
	  }
*/

		
		//HW_UARTDBGDR_WR(0);
		
		
    }
	return OK;
}


/****************************************************************************
 * Name: up_ioctl
 *
 * Description:
 *   All ioctl calls will be routed through this method
 *
 ****************************************************************************/

static int up_ioctl(struct file *filep, int cmd, unsigned long arg)
{
	
#ifdef CONFIG_SERIAL_TIOCSERGSTRUCT
  struct inode      *inode = filep->f_inode;
  struct uart_dev_s *dev   = inode->i_private;
#endif
  int                ret   = OK;


	
	 return ret;
}



/****************************************************************************
 * Name: up_receive
 *
 * Description:
 *   Called (usually) from the interrupt level to receive one character from
 *   the UART.  Error bits associated with the receipt are provided in the
 *   return 'status'.
 *
 ****************************************************************************/

static int up_receive(struct uart_dev_s *dev, unsigned int *status)
{
  uint32_t rbr;
  
  rbr = HW_UARTDBGDR_RD();
  *status = rbr >> 8;
  return rbr & 0xff;
}




/****************************************************************************
 * Name: up_rxint
 *
 * Description:
 *   Call to enable or disable RX interrupts
 *
 ****************************************************************************/

static void up_rxint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  if (enable)
    {
		//sinfo("RxintEnable\n");
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
	HW_UARTDBGIMSC_SET(BM_UARTDBGIMSC_RTIM | BM_UARTDBGIMSC_RXIM);
	
#endif
    }
  else
    {
		//sinfo("RxintDisable\n");
		HW_UARTDBGIMSC_CLR(BM_UARTDBGIMSC_RTIM | BM_UARTDBGIMSC_RXIM);
    }


}

/****************************************************************************
 * Name: up_rxavailable
 *
 * Description:
 *   Return true if the receive fifo is not empty
 *
 ****************************************************************************/

static bool up_rxavailable(struct uart_dev_s *dev)
{
	//sinfo("up_rxavailable\n");
	return(!(HW_UARTDBGFR_RD() & BM_UARTDBGFR_RXFE));
	
}

/****************************************************************************
 * Name: up_send
 *
 * Description:
 *   This method will send one byte on the UART
 *
 ****************************************************************************/

static void up_send(struct uart_dev_s *dev, int ch)
{
	//sinfo("up_send\n");
	HW_UARTDBGDR_WR(ch);
	
}

/****************************************************************************
 * Name: up_txint
 *
 * Description:
 *   Call to enable or disable TX interrupts
 *
 ****************************************************************************/

static void up_txint(struct uart_dev_s *dev, bool enable)
{
  struct up_dev_s *priv = (struct up_dev_s *)dev->priv;
  if (enable)
    {
		//sinfo("txintEnable\n");
#ifndef CONFIG_SUPPRESS_SERIAL_INTS
		HW_UARTDBGIMSC_SET(BM_UARTDBGIMSC_TXIM);
#endif
    }
  else
    {
			
	HW_UARTDBGIMSC_CLR(BM_UARTDBGIMSC_TXIM);
		//sinfo("txintDisable\n");
    }

}

/****************************************************************************
 * Name: up_txready
 *
 * Description:
 *   Return true if the tranmsit fifo is not full
 *
 ****************************************************************************/

static bool up_txready(struct uart_dev_s *dev)
{
	//sinfo("up_txready\n");
	return (!(HW_UARTDBGFR_RD() & BM_UARTDBGFR_TXFF));
	//return true;
}

/****************************************************************************
 * Name: up_txempty
 *
 * Description:
 *   Return true if the transmit fifo is empty
 *
 ****************************************************************************/

static bool up_txempty(struct uart_dev_s *dev)
{
	//sinfo("up_txempty\n");00
	return (!(HW_UARTDBGFR_RD() & BM_UARTDBGFR_TXFF));
	//return true;
}


/****************************************************************************
 * Public Functions
 ****************************************************************************/


/****************************************************************************
 * Name: arm_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes that
 *   arm_earlyserialinit was called previously.
 *
 ****************************************************************************/

void arm_serialinit(void)
{

  
#if defined(CONFIG_UART_SERIAL_CONSOLE)
  g_uartport.isconsole = true;
  up_setup(&g_uartport);
  
  uart_register("/dev/console", &g_uartport);
  
#endif
  uart_register("/dev/ttyS0", &g_uartport);

}



/****************************************************************************
 * Name: up_putc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug writes
 *
 ****************************************************************************/

int up_putc(int ch)
{
	
  struct up_dev_s *priv = &g_uartpriv;
  uint8_t ier;
  
  /* Check for LF */

  //up_disable_irq(STMP3770_IRQ_DEBUG_UART);
  up_disableuartint(priv, &ier);

  if (ch == '\n')
    {
      /* Add CR */

      arm_lowputc('\r');
    }

  /* Output the character */

  arm_lowputc(ch);
  up_restoreuartint(priv, ier);
  
  //up_enable_irq(STMP3770_IRQ_DEBUG_UART);
  
  
  return ch;
}

#else /* USE_SERIALDRIVER */

int up_putc(int ch)
{
  /* Check for LF */

  if (ch == '\n')
    {
      /* Add CR */

      arm_lowputc('\r');
    }

  /* Output the character */

  arm_lowputc(ch);
  return ch;
}

#endif


