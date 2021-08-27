/****************************************************************************
 * arch/arm/src/stmp3770/stmp3770_boot.c
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

#include <stdio.h>

#include "chip.h"
#include "arm.h"
#include "arm_internal.h"
#include "arm_arch.h"

#include "stmp3770_clkctrl.h"

#include "registers/regspower.h"
#include "registers/regsdigctl.h"

#include "registers/regsapbx.h"
#include "registers/regsapbh.h"

#ifdef CONFIG_PAGING
#  include <nuttx/page.h>
#  include "pg_macros.h"
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct section_mapping_s
{
  uint32_t physbase;   /* Physical address of the region to be mapped */
  uint32_t virtbase;   /* Virtual address of the region to be mapped */
  uint32_t mmuflags;   /* MMU settings for the region (e.g., cache-able) */
  uint32_t nsections;  /* Number of mappings in the region */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint32_t _vector_start; /* Beginning of vector block */
extern uint32_t _vector_end;   /* End+1 of vector block */

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
static const struct section_mapping_s g_section_mapping[] =
{
	{STMP3770_PERIPHERAL_PSECTION, STMP3770_PERIPHERAL_VSECTION,
	STMP3770_PERIPHERAL_MMUFLAG, STMP3770_PERIPHERAL_NSECTIONS}
	,	{0, 0,
	STMP3770_PERIPHERAL_MMUFLAG, STMP3770_PERIPHERAL_NSECTIONS}
	
	
	
	
};
#define NMAPPINGS (sizeof(g_section_mapping) / sizeof(struct section_mapping_s))
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_setlevel1entry
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE

static inline void up_setlevel1entry(uint32_t paddr, uint32_t vaddr,
                                     uint32_t mmuflags)
{
  uint32_t *pgtable = (uint32_t *)PGTABLE_BASE_VADDR;
  uint32_t  index   = vaddr >> 20;

  /* Save the page table entry */

  pgtable[index]  = (paddr | mmuflags);
  
  
}

#endif

/****************************************************************************
 * Name: up_setlevel2coarseentry
 ****************************************************************************/

static inline void up_setlevel2coarseentry(uint32_t ctabvaddr,
                                           uint32_t paddr,
                                           uint32_t vaddr,
                                           uint32_t mmuflags)
{
  uint32_t *ctable  = (uint32_t *)ctabvaddr;
  uint32_t  index;

  /* The coarse table divides a 1Mb address space up into 256 entries, each
   * corresponding to 4Kb of address space.  The coarse page table index is
   * related to the offset from the beginning of 1Mb region.
   */

  index = (vaddr & 0x000ff000) >> 12;

  /* Save the coarse table entry */

  ctable[index] = (paddr | mmuflags);
}

/****************************************************************************
 * Name: up_setupmappings
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
static void up_setupmappings(void)
{
  int i;
  int j;

  for (i = 0; i < NMAPPINGS; i++)
    {
      uint32_t sect_paddr = g_section_mapping[i].physbase;
      uint32_t sect_vaddr = g_section_mapping[i].virtbase;
      uint32_t mmuflags   = g_section_mapping[i].mmuflags;

      for (j = 0; j < g_section_mapping[i].nsections; j++)
        {
          up_setlevel1entry(sect_paddr, sect_vaddr, mmuflags);
          sect_paddr += SECTION_SIZE;
          sect_vaddr += SECTION_SIZE;
        }
    }
	tlb_invalidate();
	
}
#endif

/****************************************************************************
 * Name: up_vectorpermissions
 *
 * Description:
 *   Set permissions on the vector mapping.
 *
 ****************************************************************************/

#if !defined(CONFIG_ARCH_ROMPGTABLE) && defined(CONFIG_ARCH_LOWVECTORS) && defined(CONFIG_PAGING)
static void  up_vectorpermissions(uint32_t mmuflags)
{
  /* The PTE for the beginning of ISRAM is at the base of the L2 page table */

  uint32_t *ptr = (uint32_t *)PG_L2_VECT_VADDR;
  uint32_t pte;

  /* The pte might be zero the first time this function is called. */

  pte = *ptr;
  if (pte == 0)
    {
      pte = PG_VECT_PBASE;
    }
  else
    {
      pte &= PG_L1_PADDRMASK;
    }

  /* Update the MMU flags and save */

  *ptr = pte | mmuflags;

  /* Invalid the TLB for this address */

  tlb_invalidate_single(PG_L2_VECT_VADDR);
}
#endif


/****************************************************************************
 * Name: up_copyvectorblock
 *
 * Description:
 *   Copy the interrupt block to its final destination.
 *
 ****************************************************************************/

static volatile void up_copyvectorblock(void)
{
  uint32_t *src;
  uint32_t *end;
  uint32_t *dest;

  /* If we are using vectors in low memory but RAM in that area has been
   * marked read only, then temporarily mark the mapping write-able
   * (non-buffered).
   */

#if !defined(CONFIG_ARCH_ROMPGTABLE) && defined(CONFIG_ARCH_LOWVECTORS) && \
     defined(CONFIG_PAGING)
  up_vectorpermissions(MMU_L2_VECTRWFLAGS);
#endif
	
  src  = (uint32_t *)&_vector_start;
  end  = (uint32_t *)&_vector_end;
  
  
  dest = (uint32_t *)0x4;
  
  src++;
  
  while (src < end)
    {
      *dest++ = *src++;
    }
/*
	src  = (uint32_t *) 0x20;
	*src = 0x40;
	*/
/*
  __asm volatile ("mov r0,#4");
  __asm volatile ("mov r1,#0");
  __asm volatile ("ldr r0,[r0]");
  __asm volatile ("str r0,[r1]");
  */
  /* Make the vectors read-only, cacheable again */

#if !defined(CONFIG_ARCH_ROMPGTABLE) && defined(CONFIG_ARCH_LOWVECTORS) && defined(CONFIG_PAGING)
  up_vectorpermissions(MMU_L2_VECTROFLAGS);
#endif


}


/****************************************************************************
 * Public Functions
 ****************************************************************************/

void arm_boot(void)
{
  /* __start provided the basic MMU mappings for SRAM.
   * Now provide mappings for all IO regions
   * (Including the vector region).
   */
   
#ifndef CONFIG_ARCH_ROMPGTABLE
    up_setupmappings();

#endif /* CONFIG_ARCH_ROMPGTABLE */

    up_copyvectorblock();
	
	
	BF_SETV(POWER_VDDDCTRL,TRG,26); // Set voltage = 1.45 V
	
	pll_enable();
	
	HCLK_set_div(0, 4);   //120 MHz
    CPUCLK_set_div(0, 2); //240 MHz
    CPUCLK_set_gating(0);
    CPUCLK_set_bypass(0);
	
	BF_CS2(APBX_CTRL0, SFTRST, 0, CLKGATE, 0); //Enable APBX DMA
	BF_CS2(APBH_CTRL0, SFTRST, 0, CLKGATE, 0); //Enable APBH DMA


}

