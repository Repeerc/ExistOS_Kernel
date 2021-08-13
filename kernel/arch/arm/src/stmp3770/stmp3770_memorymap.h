/****************************************************************************
 * arch/arm/src/stmp3770/stmp3770_memorymap.h
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

#ifndef __ARCH_ARM_SRC_STMP3770_MEMORYMAP_H
#define __ARCH_ARM_SRC_STMP3770_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "chip.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* STMP3770 Physical (unmapped) Memory Map */

#  define     STMP3770_OCSRAM_PSECTION        0x00000000
#    define      STMP3770_OCSRAM_PADDR        0x00000000

#  define     STMP3770_PERIPHERAL_PSECTION    0x80000000



/* STMP3770 Virtual Memory Map */

#  define     STMP3770_OCSRAM_VSECTION        0x00000000
#    define      STMP3770_OCSRAM_VADDR        0x00000000


#  define     STMP3770_PERIPHERAL_VSECTION    0x80000000




/* Sizes of memory regions in bytes */

#define     STMP3770_OCSRAM_SIZE         (512*1024)

#define     STMP3770_PERIPHERAL_SIZE     (128*1024)


/* Sizes of sections/regions.  The boot logic in stmp3770_boot.c, will select
 * 1Mb level 1 MMU mappings to span the entire physical address space.
 * The definitions below specify the number of 1Mb entries that are
 * required to span a particular address region.
 */
#define     STMP3770_PERIPHERAL_NSECTIONS  1
 
 /* Section MMU Flags */


#define    STMP3770_PERIPHERAL_MMUFLAG       MMU_IOFLAGS 
 

#if !defined(PGTABLE_BASE_PADDR) || !defined(PGTABLE_BASE_VADDR)

/* Sanity check.. if one is undefined, both should be undefined */

#  if defined(PGTABLE_BASE_PADDR) || defined(PGTABLE_BASE_VADDR)
#    error "Only one of PGTABLE_BASE_PADDR or PGTABLE_BASE_VADDR is defined"
#  endif

#          define PGTABLE_BASE_PADDR (STMP3770_OCSRAM_PADDR + STMP3770_OCSRAM_SIZE - 2 * PGTABLE_SIZE)
//#          define PGTABLE_BASE_VADDR (STMP3770_OCSRAM_VADDR + STMP3770_OCSRAM_SIZE - 2 * PGTABLE_SIZE)

#          define PGTABLE_BASE_VADDR  0xC0000

#endif



/* Page table start addresses:
 *
 * 16Kb of memory is reserved hold the page table for the virtual mappings.
 * A portion of this table is not accessible in the virtual address space
 * (for normal operation). We will reuse this memory for coarse page tables
 * as follows:
 *
 * NOTE: If CONFIG_PAGING is defined, pg_macros.h will re-assign the virtual
 * address of the page table.
 */

#define PGTABLE_L2_COARSE_OFFSET    (0x0000)
#define PGTABLE_L2_COARSE_PBASE     (PGTABLE_BASE_PADDR + PGTABLE_SIZE + PGTABLE_L2_COARSE_OFFSET)
#define PGTABLE_L2_COARSE_VBASE     (PGTABLE_BASE_VADDR + PGTABLE_SIZE + PGTABLE_L2_COARSE_OFFSET)

#define PGTABLE_L2_FINE_OFFSET      (0x0000)
#define PGTABLE_L2_FINE_PBASE       (PGTABLE_BASE_PADDR + PGTABLE_SIZE + PGTABLE_L2_FINE_OFFSET)
#define PGTABLE_L2_FINE_VBASE       (PGTABLE_BASE_VADDR + PGTABLE_SIZE + PGTABLE_L2_FINE_OFFSET)

/* Page table end addresses: */

#define PGTABLE_L2_END_PADDR        (PGTABLE_BASE_PADDR + 2 * PGTABLE_SIZE)
#define PGTABLE_L2_END_VADDR        (PGTABLE_BASE_VADDR + 2 * PGTABLE_SIZE)

/* Page table sizes */

#define PGTABLE_L2_COARSE_ALLOC     (PGTABLE_L2_END_VADDR - PGTABLE_L2_COARSE_VBASE)
#define PGTABLE_COARSE_TABLE_SIZE   (4*256)
#define PGTABLE_NCOARSE_TABLES      (PGTABLE_L2_COARSE_ALLOC / PGTABLE_COARSE_TABLE_SIZE)

#define PGTABLE_L2_FINE_ALLOC       (PGTABLE_L2_END_VADDR-PGTABLE_L2_FINE_VBASE)
#define PGTABLE_FINE_TABLE_SIZE     (4*1024)
#define PGTABLE_NFINE_TABLES        (PGTABLE_L2_FINE_ALLOC / PGTABLE_FINE_TABLE_SIZE)




#endif
