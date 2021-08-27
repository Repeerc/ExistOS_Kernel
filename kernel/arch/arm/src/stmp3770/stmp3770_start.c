
/****************************************************************************
 * arch/arm/src/stmp3770/stmp3770_start.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/init.h>
#include <arch/irq.h>

#include "arm_arch.h"
#include "arm_internal.h"
#include "arm.h"




#define MMU_SECTION_RAM(a) (((a)&0xfff00000) | 0xc1e)
#define MMU_SECTION_DEV(a) (((a)&0xfff00000) | 0x012 | 0x800)
#define MMU_COARSE_PAGE(a) (((a)&0xfffffc00) | 0x011)

#define MMU_SMALL_PAGE_CACHED(a) (((a)&0xfffff000) | 0xffe)
#define MMU_SMALL_PAGE_CACHED_RO(a) (((a)&0xfffff000) | 0xaae)
#define MMU_SMALL_PAGE_NONCACHED(a) (((a)&0xfffff000) | 0xff2)

#define MMU_SMALL_PAGE_NONCACHED_PRIVILEGED(a) (((a)&0xfffff000) | 0x002 | 0xAA)

#define MMU_SMALL_PAGE_UNMAP (0)

#define MMU_LEVEL1_INDEX(virt) (((virt) >> 20) & 0xfff)
#define MMU_LEVEL2_INDEX(virt) (((virt) >> 12) & 0xff)

#define MMU_UNMAP_SECTION_VIRT_RAM(virt) (tlb_base[MMU_LEVEL1_INDEX(virt)] = 0)

#define MMU_MAP_SECTION_RAM(phys, virt) (tlb_base[MMU_LEVEL1_INDEX(virt)] = MMU_SECTION_RAM(phys))
#define MMU_MAP_SECTION_DEV(phys, virt) (tlb_base[MMU_LEVEL1_INDEX(virt)] = MMU_SECTION_DEV(phys))
#define MMU_MAP_COARSE_RAM(phys, virt) (tlb_base[MMU_LEVEL1_INDEX(virt)] = MMU_COARSE_PAGE(phys))

#define MMU_MAP_SMALL_PAGE_CACHED(phys, virt) (((unsigned int *)(tlb_base[MMU_LEVEL1_INDEX(virt)] & 0xfffffc00))[MMU_LEVEL2_INDEX(virt)] = MMU_SMALL_PAGE_CACHED(phys))
#define MMU_MAP_SMALL_PAGE_CACHED_WITH_L2(phys, virt, l2virt) (((unsigned int *)(l2virt))[MMU_LEVEL2_INDEX(virt)] = MMU_SMALL_PAGE_CACHED(phys))

#define MMU_MAP_SMALL_PAGE_CACHED_RO_WITH_L2(phys, virt, l2virt) (((unsigned int *)(l2virt))[MMU_LEVEL2_INDEX(virt)] = MMU_SMALL_PAGE_CACHED_RO(phys))

#define MMU_MAP_SMALL_PAGE_NONCACHED(phys, virt) (((unsigned int *)(tlb_base[MMU_LEVEL1_INDEX(virt)] & 0xfffffc00))[MMU_LEVEL2_INDEX(virt)] = MMU_SMALL_PAGE_NONCACHED(phys))
#define MMU_MAP_SMALL_PAGE_NONCACHED_PRIVILEGED(phys, virt) (((unsigned int *)(tlb_base[MMU_LEVEL1_INDEX(virt)] & 0xfffffc00))[MMU_LEVEL2_INDEX(virt)] = MMU_SMALL_PAGE_NONCACHED_PRIVILEGED(phys))
#define MMU_MAP_SMALL_PAGE_NONCACHED_WITH_L2(phys, virt, l2virt) (((unsigned int *)(l2virt))[MMU_LEVEL2_INDEX(virt)] = MMU_SMALL_PAGE_NONCACHED(phys))

#define MMU_MAP_SMALL_PAGE_UNMAP(virt) (((unsigned int *)(tlb_base[MMU_LEVEL1_INDEX(virt)] & 0xfffffc00))[MMU_LEVEL2_INDEX(virt)] = MMU_SMALL_PAGE_UNMAP)
#define MMU_MAP_SMALL_PAGE_UNMAP_WITH_L2(virt, l2virt) (((unsigned int *)(l2virt))[MMU_LEVEL2_INDEX(virt)] = MMU_SMALL_PAGE_UNMAP)



uint32_t PAGE_TABLE_AREA[8192] __attribute__((section(".page_data"))) = {0};


//const uint32_t g_idle_topstack[CONFIG_IDLETHREAD_STACKSIZE] __attribute__((aligned(0x4))) = {0};


volatile static void __flush_Dcache(void) {
    register unsigned int counter asm("r2");
    register unsigned int cacheaddr asm("r3");

    counter = 0;
    while (counter < 512) {
        cacheaddr = ((counter >> 1) & 0xe0) | ((counter & 63) << 26);
        // CLEAN AND INVALIDATE ENTRY USING INDEX
        asm volatile("mcr p15, 0, %0, c7, c14, 2"
                     :
                     : "r"(cacheaddr));
        ++counter;
    }
}

volatile static void __flush_Icache(void) {
    // CLEAN AND INVALIDATE ENTRY USING INDEX
    register unsigned int value;
    value = 0;
    asm volatile("mcr p15, 0, %0, c7, c5, 0"
                 :
                 : "r"(value));
}

volatile static void __flush_TLB(void) {
    // CLEAN AND INVALIDATE ENTRY USING INDEX
    register unsigned int value;
    value = 0;
    asm volatile("mcr p15, 0, %0, c8, c7, 0"
                 :
                 : "r"(value));
}

void static __enable_mmu(unsigned int *base) {
    //r0 = base
    asm volatile("mcr p15,0,r0,c2,c0,0"); // WRITE MMU BASE REGISTER, ALL CACHES SHOULD'VE BEEN CLEARED BEFORE
/*
    asm volatile("mvn r0,#0");
    asm volatile("mcr p15,0,r0,c3,c0,0"); // SET R/W ACCESS PERMISSIONS FOR ALL DOMAINS

    asm volatile("mrc p15, 0, r0, c1, c0, 0");
    asm volatile("orr r0,r0,#1"); // Enable MMU

    asm volatile("orr r0,r0,#5");      // ENABLE MMU AND DATA CACHES
    asm volatile("orr r0,r0,#0x1000"); // ENABLE INSTRUCTION CACHE
*/
 

	asm volatile("mov		r0, #0x1f			");/* Domains 0, 1 = client */
	asm volatile("mcr		p15, 0, r0, c3, c0	");	/* Load domain access register */
	asm volatile("mrc		p15, 0, r0, c1, c0	");	/* Get control register */

	
	/* Clear bits (see arm.h)
	 *
	 * CR_R - ROM MMU protection
	 * CR_F - Implementation defined
	 * CR_Z - Implementation defined
	 *
	 * CR_A - Alignment abort enable
	 * CR_C - Dcache enable
	 * CR_W - Write buffer enable
	 *
	 * CR_I - Icache enable
	 */

    
    

	asm volatile("bic		r0, r0, #0x2100 ");
	asm volatile("bic		r0, r0, #0x4000 ");
	asm volatile("bic		r0, r0, #0x4 ");
	asm volatile("bic		r0, r0, #0x4000");
	asm volatile("bic		r0, r0, #0x1000");
	asm volatile("bic		r0, r0, #0x2");
	
	asm volatile("orr r0,r0,#1"); // Enable MMU
	
#ifndef CONFIG_ARCH_LOWVECTORS
	asm volatile("orr		r0, r0, #0x2100");
#else
	asm volatile("orr		r0, r0, #0x0100");
#endif


	/* CR_RR - Round Robin cache replacement */

#ifdef CPU_CACHE_ROUND_ROBIN
	asm volatile("orr		r0, r0, #0x4000");
#endif
	/* CR_C - Dcache enable */

#ifndef CPU_DCACHE_DISABLE
	asm volatile("orr		r0, r0, #(0x4)");
#endif
	/* CR_C - Icache enable */

#ifndef CPU_ICACHE_DISABLE
	asm volatile("orr		r0, r0, #(0x1000)");
#endif
	/* CR_A - Alignment abort enable */

#ifdef CPU_ALIGNMENT_TRAP
	asm volatile("orr		r0, r0, #(0x2)");
#endif

	
    asm volatile("mcr p15, 0, r0, c1, c0, 0");

    asm volatile("mov r0,r0"); // NOP INSTRUCTIONS THAT ARE FETCHED FROM PHYSICAL ADDRESS
    asm volatile("mov r0,r0");
}

void __start() __attribute__((naked));
void __start(){
	uint32_t *tlb_base, *dest;
	
	__asm volatile ("mov 	r0, r0");
	__asm volatile ("mov 	r0, r0");
	__asm volatile ("mov 	r0, r0");
	__asm volatile ("mov 	r0, r0");
	__asm volatile ("mov 	r0, r0");
	__asm volatile ("mov 	r0, r0");
	__asm volatile ("mov 	r0, r0");
	__asm volatile ("mov 	r0, r0");
	__asm volatile ("mov 	r0, r0");
	__asm volatile ("mov 	r0, r0");
	__asm volatile ("mov 	r0, r0");
	__asm volatile ("mov 	r0, r0");
	__asm volatile ("mov 	r0, r0");
	__asm volatile ("mov 	r0, r0");
	__asm volatile ("mov 	r0, r0");
	__asm volatile ("mov 	r0, r0");
	__asm volatile ("mov 	r0, r0");
	__asm volatile ("mov 	r0, r0");
	__asm volatile ("mov 	r0, r0");
	__asm volatile ("mov 	r0, r0");
	__asm volatile ("mov 	r0, r0");
	__asm volatile ("mov 	r0, r0");
	__asm volatile ("mov 	r0, r0");
	__asm volatile ("mov 	r0, r0");
	
		/* Make sure that we are in SVC mode with all IRQs disabled */
    __asm volatile ("mov	sp, #0x70000");
	
	__asm volatile ("mov	r0, #(0x00000013 | 0x00000080 | 0x00000040)");
	__asm volatile ("msr	cpsr_c, r0");
	
	
	/*
	for(dest = PAGE_TABLE_AREA; dest < (&PAGE_TABLE_AREA[8191]); )
	{
		*dest++ = 0;
	}*/
	for(int i=0;i<8192;i++){
		PAGE_TABLE_AREA[i] = 0;
	}
	
	tlb_base = PAGE_TABLE_AREA;
	
	MMU_MAP_SECTION_DEV(0, 0);
	
    __flush_Dcache();
    __flush_Icache();
    __flush_TLB();
	__enable_mmu(tlb_base);
	
    for (dest = &_sbss; dest < &_ebss; )
    {
      *dest++ = 0;
    }	
	
	
	
	arm_boot();
	
	nx_start();
	
	for(;;);
}
