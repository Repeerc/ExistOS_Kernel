/****************************************************************************
 * arch/arm/src/lpc31xx/lpc31_lowputc.c
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
#include <debug.h>

#include <nuttx/arch.h>

#include "arm_arch.h"
#include "arm_internal.h"

#include "registers/regsuartdbg.h"


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
 
 
/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/
 
 
/****************************************************************************
 * Name: arm_lowputc
 *
 * Description:
 *   Provide priority, low-level access to support OS debug  writes
 *
 ****************************************************************************/

void arm_lowputc(char ch)
{
//#ifdef HAVE_CONSOLE
 // up_waittxready();
  //putreg32((uint32_t)ch, LPC31_UART_THR);
//#endif
    while (HW_UARTDBGFR_RD() & BM_UARTDBGFR_TXFF)
		;

    if (!(HW_UARTDBGFR_RD() & BM_UARTDBGFR_TXFF))
        HW_UARTDBGDR_WR(ch);
	
}


 