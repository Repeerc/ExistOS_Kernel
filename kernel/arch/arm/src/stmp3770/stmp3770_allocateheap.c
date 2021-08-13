/****************************************************************************
 * arch/arm/src/stmp3770/stmp3770_allocateheap.c
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
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/kmalloc.h>
#include <arch/board/board.h>

#include "arm.h"
#include "chip.h"
#include "arm_arch.h"
#include "arm_internal.h"
#include "stmp3770_memorymap.h"

#ifdef CONFIG_PAGING
#  include <nuttx/page.h>
#  include "pg_macros.h"
#endif


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Terminology.  In the flat build (CONFIG_BUILD_FLAT=y), there is only a
 * single heap access with the standard allocations (malloc/free).  This
 * heap is referred to as the user heap.  In the protected build
 * (CONFIG_BUILD_PROTECTED=y) where an MPU is used to protect a region of
 * otherwise flat memory, there will be two allocators:  One that allocates
 * protected (kernel) memory and one that allocates unprotected (user)
 * memory.  These are referred to as the kernel and user heaps,
 * respectively.
 *
 * The ARM has no MPU but does have an MMU.  With this MMU, it can support
 * the kernel build (CONFIG_BUILD_KERNEL=y).  In this configuration, there
 * is one kernel heap but multiple user heaps:  One per task group.  However,
 * in this case, we need only be concerned about initializing the single
 * kernel heap here.
 */

#if defined(CONFIG_BUILD_KERNEL)
#  define MM_ADDREGION kmm_addregion
#else
#  define MM_ADDREGION umm_addregion
#endif


#ifdef CONFIG_PAGING
#    define STMP3770_HEAP_VEND (PG_LOCKED_VBASE + PG_TOTAL_VSIZE - PGTABLE_SIZE)
#else
#    define STMP3770_HEAP_VEND (STMP3770_OCSRAM_PADDR + STMP3770_OCSRAM_SIZE - PGTABLE_SIZE)
#endif



/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_allocate_heap
 *
 * Description:
 *   This function will be called to dynamically set aside the heap region.
 *
 *   For the kernel build (CONFIG_BUILD_KERNEL=y) with both kernel- and
 *   user-space heaps (CONFIG_MM_KERNEL_HEAP=y), this function provides the
 *   size of the unprotected, user-space heap.
 *
 *   If a protected kernel-space heap is provided, the kernel heap must be
 *   allocated (and protected) by an analogous up_allocate_kheap().
 *
 *   SRAM layout:
 *   Start of SRAM:   .data
 *                    .bss
 *                    IDLE thread stack
 *   End of SRAm:     heap
 *
 *   NOTE: Ignore the erroneous nomenclature DRAM and SDRAM.  That names
 *   date back to an earlier platform that had SDRAM.
 *
 ****************************************************************************/

void up_allocate_heap(FAR void **heap_start, size_t *heap_size)
{
  board_autoled_on(LED_HEAPALLOCATE);
  *heap_start = (FAR void *)g_idle_topstack;
  *heap_size  = STMP3770_HEAP_VEND - g_idle_topstack;
}

#if CONFIG_MM_REGIONS > 1
void arm_addregion(void)
{
	
}

#endif

