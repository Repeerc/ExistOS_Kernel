


#ifndef __ARCH_ARM_SRC_STMP3770_CLKCTRL_H
#define __ARCH_ARM_SRC_STMP3770_CLKCTRL_H

#include <stdint.h>

void pll_enable();
int pll_isenable();
void pll_set_usb_clock(int state);

uint8_t HCLK_set_div(uint8_t isFracEnabled, uint16_t divider);
void CPUCLK_set_gating(uint8_t gating) ;
void CPUCLK_set_bypass(uint8_t bypass);


uint8_t CPUCLK_set_div(uint8_t isFracEnabled, uint16_t divider) ;




#endif