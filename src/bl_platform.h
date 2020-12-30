/**
 * \file bl_platform.h
 *
 * \brief This file exports the APIs used for configuring devices
 *        required during boot
 *
 */

/*
* Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#ifndef _BL_PLATFORM_H__
#define _BL_PLATFORM_H__

#include "soc_AM335x.h"
#include "beaglebone.h"


/******************************************************************************
**                        Macro Definitions
*******************************************************************************/

/* Set of config parameters for AM335x */
#define MMCSD_BASE                     SOC_MMCHS_0_REGS
#define MMCSD_DMA_BASE                 SOC_EDMA30CC_0_REGS

#define MMCSD_IN_FREQ                  96000000 /* 96MHz */
#define MMCSD_INIT_FREQ                400000   /* 400kHz */

#define MMCSD_DMA_CHA_TX               24
#define MMCSD_DMA_CHA_RX               25
#define MMCSD_DMA_QUE_NUM              0
#define MMCSD_DMA_REGION_NUM           0
#define MMCSD_BLK_SIZE                 512
#define MMCSD_OCR                      (SD_OCR_VDD_3P0_3P1 | SD_OCR_VDD_3P1_3P2)

#define IMAGE_SIZE                         50 * 1024 /* Max size */
#define DDR_START_ADDR                     0x80000000
#define UART_BASE                          SOC_UART_0_REGS

/* Set of config parameters */

/*
**Setting the CORE PLL values at OPP100:
** OSCIN = 24MHz, Fdpll = 2GHz
** HSDM4 = 200MHz, HSDM5 = 250MHz
** HSDM6 = 500MHz
*/
#define COREPLL_M                          1000
#define COREPLL_N                          23
#define COREPLL_HSD_M4                     10
#define COREPLL_HSD_M5                     8
#define COREPLL_HSD_M6                     4

/* Setting the  PER PLL values at OPP100:
** OSCIN = 24MHz, Fdpll = 960MHz
** CLKLDO = 960MHz, CLKOUT = 192MHz
*/
#define PERPLL_M                           960
#define PERPLL_N                           23
#define PERPLL_M2                          5

 /* Setting the Display CLKOUT at 300MHz independently
 ** This is required for full clock 150MHz for LCD
 ** OSCIN = 24MHz, Fdpll = 300MHz
 ** CLKOUT = 150MHz
 */
#define DISPLL_M                           25
#define DISPLL_N                           3
#define DISPLL_M2                          1

/*
**Setting the DDR2 frequency to 266MHz
*/
#define DDRPLL_M_DDR2                     (266)
#define DDRPLL_M_DDR3                     (303)
#define DDRPLL_N		           23
#define DDRPLL_M2		           1

/*
** MACROS to configure SEL bit filed in VDD1_OP_REG of PMIC.
** Refer the datasheet of PMIC for voltage values.
*/
#define     PMIC_VOLT_SEL_0950MV      DCDC_VOLT_SEL_0950MV
#define     PMIC_VOLT_SEL_1100MV      DCDC_VOLT_SEL_1100MV
#define     PMIC_VOLT_SEL_1200MV      DCDC_VOLT_SEL_1200MV
#define     PMIC_VOLT_SEL_1260MV      DCDC_VOLT_SEL_1275MV
#define     PMIC_VOLT_SEL_1325MV      (0x11u)

/*
** Structure for OPP Configuration
*/
typedef struct oppConfig
{
    unsigned int pllMult;
    unsigned int pmicVolt;
} tOPPConfig;

/******************************************************************************
**                    External Function Declarations
*******************************************************************************/

extern void BlPlatformConfigPostBoot( void );
extern void BlPlatformConfig(void);
extern void BlPlatformMMCSDSetup(void);
extern void BL_PLATFORM_MMCSDSetup(void);
extern unsigned int BlPlatformMMCSDImageCopy();
extern void TPS65217RegRead2(unsigned char regOffset, unsigned char* dest);

#endif /* _BL_PLATFORM_H__ */

