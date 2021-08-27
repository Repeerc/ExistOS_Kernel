/****************************************************************************
 * arch/arm/include/stmp3770/irq.h
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

/* This file should never be included directly but, rather,
 * only indirectly through nuttx/irq.h
 */
 

#ifndef __ARCH_ARM_INCLUDE_STMP3770_IRQ_H
#define __ARCH_ARM_INCLUDE_STMP3770_IRQ_H


/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Prototypes
 ****************************************************************************/

/* STMP3770 Interrupts */

#define STMP3770_IRQ_DEBUG_UART          (0)
#define STMP3770_IRQ_COMMS_RX            (1)
#define STMP3770_IRQ_COMMS_TX            (1)
#define STMP3770_IRQ_SSP2_ERROR          (2)
#define STMP3770_IRQ_VDD5V               (3)
#define STMP3770_IRQ_HEADPHONE_SHORT     (4)
#define STMP3770_IRQ_DAC_DMA             (5)
#define STMP3770_IRQ_DAC_ERROR           (6)
#define STMP3770_IRQ_ADC_DMA             (7)
#define STMP3770_IRQ_ADC_ERROR           (8)
#define STMP3770_IRQ_SPDIF_DMA           (9)
#define STMP3770_IRQ_SAIF2_DMA           (9)
#define STMP3770_IRQ_SPDIF_ERROR         (10)
#define STMP3770_IRQ_SAIF1_IRQ           (10)
#define STMP3770_IRQ_SAIF2_IRQ           (10)
#define STMP3770_IRQ_USB_CTRL            (11)
#define STMP3770_IRQ_USB_WAKEUP          (12)
#define STMP3770_IRQ_GPMI_DMA            (13)
#define STMP3770_IRQ_SSP1_DMA            (14)
#define STMP3770_IRQ_SSP_ERROR           (15)
#define STMP3770_IRQ_GPIO0               (16)
#define STMP3770_IRQ_GPIO1               (17)
#define STMP3770_IRQ_GPIO2               (18)
#define STMP3770_IRQ_SAIF1_DMA           (19)
#define STMP3770_IRQ_SSP2_DMA            (20)
#define STMP3770_IRQ_ECC8_IRQ            (21)
#define STMP3770_IRQ_RTC_ALARM           (22)
#define STMP3770_IRQ_UARTAPP_TX_DMA      (23)
#define STMP3770_IRQ_UARTAPP_INTERNAL    (24)
#define STMP3770_IRQ_UARTAPP_RX_DMA      (25)
#define STMP3770_IRQ_I2C_DMA             (26)
#define STMP3770_IRQ_I2C_ERROR           (27)
#define STMP3770_IRQ_TIMER0              (28)
#define STMP3770_IRQ_TIMER1              (29)
#define STMP3770_IRQ_TIMER2              (30)
#define STMP3770_IRQ_TIMER3              (31)
#define STMP3770_IRQ_BATT_BRNOUT         (32)
#define STMP3770_IRQ_VDDD_BRNOUT         (33)
#define STMP3770_IRQ_VDDIO_BRNOUT        (34)
#define STMP3770_IRQ_VDD18_BRNOUT        (35)
#define STMP3770_IRQ_TOUCH_DETECT        (36)
#define STMP3770_IRQ_LRADC_CH0           (37)
#define STMP3770_IRQ_LRADC_CH1           (38)
#define STMP3770_IRQ_LRADC_CH2           (39)
#define STMP3770_IRQ_LRADC_CH3           (40)
#define STMP3770_IRQ_LRADC_CH4           (41)
#define STMP3770_IRQ_LRADC_CH5           (42)
#define STMP3770_IRQ_LRADC_CH6           (43)
#define STMP3770_IRQ_LRADC_CH7           (44)
#define STMP3770_IRQ_LCDIF_DMA           (45)
#define STMP3770_IRQ_LCDIF_ERROR         (46)
#define STMP3770_IRQ_DIGCTL_DEBUG_TRAP   (47)
#define STMP3770_IRQ_RTC_1MSEC           (48)
#define STMP3770_IRQ_DRI_DMA             (49)
#define STMP3770_IRQ_DRI_ATTENTION       (50)
#define STMP3770_IRQ_GPMI                (51)
#define STMP3770_IRQ_IR                  (52)
#define STMP3770_IRQ_DCP_VMI             (53)
#define STMP3770_IRQ_DCP                 (54)
#define STMP3770_IRQ_RESERVED_55         (55)

#define NR_IRQS 				STMP3770_IRQ_RESERVED_55

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif



