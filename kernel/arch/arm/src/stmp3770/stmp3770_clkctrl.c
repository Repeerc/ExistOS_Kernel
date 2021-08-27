/****************************************************************************
 * arch/arm/src/stmp3770/stmp3770_clkctrl.c
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
 
#include "registers/regsclkctrl.h" 
#include "registers/regsdigctl.h" 
#include "stmp3770_clkctrl.h"


static void delay_us(unsigned int us) {
    unsigned int start, cur;
    start = cur = HW_DIGCTL_MICROSECONDS_RD();
    while (cur < start + us) {
        cur = HW_DIGCTL_MICROSECONDS_RD();
    }
} 

void pll_enable()
{
    BF_SETV(CLKCTRL_PLLCTRL0, POWER, 1);
    delay_us(10);
}

int pll_isenable()
{
	return (!((HW_CLKCTRL_PLLCTRL0_RD() & BM_CLKCTRL_PLLCTRL0_POWER) == 0));
}
 
void pll_set_usb_clock(int state)
{
	
	HW_CLKCTRL_PLLCTRL0_SET(BF_CLKCTRL_PLLCTRL0_EN_USB_CLKS(state));
	
}



uint8_t CPUCLK_set_div(uint8_t isFracEnabled, uint16_t divider) {
    if (HW_CLKCTRL_CPU_RD() & BM_CLKCTRL_CPU_BUSY_REF_CPU) {
        return 0;
    } else {
        BF_CS1(CLKCTRL_CPU, DIV_CPU_FRAC_EN, isFracEnabled);
        BF_SETV(CLKCTRL_CPU, DIV_CPU, divider);
        //divider = ~divider;
        //BF_CLRV(CLKCTRL_CPU, DIV_CPU, divider);
        
        if (!(divider & 1)){
            //delay_us(100);
            BF_CLRV(CLKCTRL_CPU, DIV_CPU, 1);
        }
        return 1;
    }
}

void CPUCLK_set_bypass(uint8_t bypass) {
    BF_CS1(CLKCTRL_CLKSEQ, BYPASS_CPU, bypass);
}

void CPUCLK_set_gating(uint8_t gating) {
    BF_CS1(CLKCTRL_FRAC, CLKGATECPU, gating);
}

uint8_t HCLK_set_div(uint8_t isFracEnabled, uint16_t divider) {
    if (HW_CLKCTRL_HBUS_RD() & BM_CLKCTRL_HBUS_BUSY) {
        return 0;
    } else {
        BF_CS1(CLKCTRL_HBUS, DIV_FRAC_EN, isFracEnabled);

        BF_SETV(CLKCTRL_HBUS, DIV, divider);
        //divider = ~divider;
        //BF_CLRV(CLKCTRL_HBUS, DIV, divider);
        
        if (!(divider & 1)){
            //delay_us(100);
            BF_CLRV(CLKCTRL_HBUS, DIV, 1);
            }
        return 1;
    }
}



