/****************************************************************************
 * arch/arm/src/stmp3770/stmp3770_irq.c
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

#include <stdint.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <arch/irq.h>

#include "arm.h"
#include "arm_arch.h"
#include "arm_internal.h"

#include "registers/regsicoll.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
 * CURRENT_REGS for portability.
 */

volatile uint32_t *g_current_regs[1];

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
	sinfo("IRQ initializing...\n");
	
	HW_ICOLL_CTRL_CLR(BM_ICOLL_CTRL_SFTRST);
	HW_ICOLL_CTRL_CLR(BM_ICOLL_CTRL_CLKGATE);
	sinfo("Interrupt Controllor Reset...\n");
	HW_ICOLL_CTRL_SET(BM_ICOLL_CTRL_SFTRST);
	
	while (!BF_RD(ICOLL_CTRL, CLKGATE)) {
        ;
    }
	HW_ICOLL_CTRL_CLR(BM_ICOLL_CTRL_SFTRST);
	HW_ICOLL_CTRL_CLR(BM_ICOLL_CTRL_CLKGATE);	
	
	
    HW_ICOLL_CTRL_CLR(BM_ICOLL_CTRL_FIQ_FINAL_ENABLE | BM_ICOLL_CTRL_IRQ_FINAL_ENABLE | 
							BM_ICOLL_CTRL_BYPASS_FSM | BM_ICOLL_CTRL_NO_NESTING | 
							BM_ICOLL_CTRL_ARM_RSE_MODE);

    HW_ICOLL_CTRL_SET( BM_ICOLL_CTRL_FIQ_FINAL_ENABLE | 
                      BM_ICOLL_CTRL_IRQ_FINAL_ENABLE |
                      BM_ICOLL_CTRL_ARM_RSE_MODE | 
                      BM_ICOLL_CTRL_NO_NESTING  );
					 
					  
	BF_CS1(ICOLL_VBASE, TABLE_ADDRESS, NULL);
    //BW_ICOLL_CTRL_VECTOR_PITCH(BV_ICOLL_CTRL_VECTOR_PITCH__BY16);
    BW_ICOLL_CTRL_VECTOR_PITCH(BV_ICOLL_CTRL_VECTOR_PITCH__BY4);
	
	sinfo("IRQ initialized.\n");
	
	CURRENT_REGS = NULL;
	
#ifndef CONFIG_SUPPRESS_INTERRUPTS
  /* And finally, enable interrupts */

  up_irq_restore(PSR_MODE_SVC | PSR_F_BIT);
#endif

}
 
/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
	unsigned int volatile *address = (unsigned int *)HW_ICOLL_PRIORITYn_ADDR((irq / 4));
	*address &= ~(0x4 << ( (irq % 4) * 8) );
}



/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
	unsigned int volatile *address = (unsigned int *)HW_ICOLL_PRIORITYn_ADDR((irq / 4));
	*address |= (0x4 << ( (irq % 4) * 8) );
}


/****************************************************************************
 * Name: arm_ack_irq
 *
 * Description:
 *   Acknowledge the interrupt
 *
 ****************************************************************************/

void arm_ack_irq(int irq)
{
	volatile uint32_t rb,rk;
	rb = BF_RD(ICOLL_VECTOR, IRQVECTOR);
	rk = 1 << ((*((volatile unsigned int *)HW_ICOLL_PRIORITYn_ADDR(irq / 4)) >> (8 * (irq % 4))) & 0x03);
	
	BF_SETV(ICOLL_VECTOR, IRQVECTOR, rb);
	
	BF_SETV(ICOLL_LEVELACK, IRQLEVELACK, rk);
}


/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
	unsigned int volatile *address = (unsigned int *)HW_ICOLL_PRIORITYn_ADDR((irq_n / 4));
	*address &= ~(0x3 << ( (irq_n % 4) * 8) );
	*address |= ((priority & 0x03) << ( (irq_n % 4) * 8) );
	return OK;
}
#endif


