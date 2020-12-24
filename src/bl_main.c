#include "uartStdio.h"
#include "bl_platform.h"
#include "bl.h"

#include "gpio_v2.h"
#include "hw/hw_control_AM335x.h"
#include "hw/hw_types.h"
#include "hw/hw_cm_per.h"

unsigned int entryPoint = 0;
unsigned int DspEntryPoint = 0;

static void Delay(volatile unsigned int count)
{
    while(count--);
}


static void GPIO3ModuleClkConfig(void)
{

    /* Writing to MODULEMODE field of CM_PER_GPIO1_CLKCTRL register. */
    HWREG(SOC_CM_PER_REGS + CM_PER_GPIO3_CLKCTRL) |=
          CM_PER_GPIO3_CLKCTRL_MODULEMODE_ENABLE;

    /* Waiting for MODULEMODE field to reflect the written value. */
    while(CM_PER_GPIO3_CLKCTRL_MODULEMODE_ENABLE !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_GPIO3_CLKCTRL) &
           CM_PER_GPIO3_CLKCTRL_MODULEMODE));
    /*
    ** Writing to OPTFCLKEN_GPIO_3_GDBCLK bit in CM_PER_GPIO3_CLKCTRL
    ** register.
    */
    HWREG(SOC_CM_PER_REGS + CM_PER_GPIO3_CLKCTRL) |=
          CM_PER_GPIO3_CLKCTRL_OPTFCLKEN_GPIO_3_GDBCLK;

    /*
    ** Waiting for OPTFCLKEN_GPIO_3_GDBCLK bit to reflect the desired
    ** value.
    */
    while(CM_PER_GPIO3_CLKCTRL_OPTFCLKEN_GPIO_3_GDBCLK !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_GPIO3_CLKCTRL) &
           CM_PER_GPIO3_CLKCTRL_OPTFCLKEN_GPIO_3_GDBCLK));

    /*
    ** Waiting for IDLEST field in CM_PER_GPIO3_CLKCTRL register to attain the
    ** desired value.
    */
    while((CM_PER_GPIO3_CLKCTRL_IDLEST_FUNC <<
           CM_PER_GPIO3_CLKCTRL_IDLEST_SHIFT) !=
           (HWREG(SOC_CM_PER_REGS + CM_PER_GPIO3_CLKCTRL) &
            CM_PER_GPIO3_CLKCTRL_IDLEST));

    /*
    ** Waiting for CLKACTIVITY_GPIO_3_GDBCLK bit in CM_PER_L4LS_CLKSTCTRL
    ** register to attain desired value.
    */
    while(CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_GPIO_3_GDBCLK !=
          (HWREG(SOC_CM_PER_REGS + CM_PER_L4LS_CLKSTCTRL) &
           CM_PER_L4LS_CLKSTCTRL_CLKACTIVITY_GPIO_3_GDBCLK));
}

static void init_gpio()
{
    UARTPuts("Initialize GPIOs.", -1);

    GPIO0ModuleClkConfig();
    UARTPuts(".", -1);
    GPIO1ModuleClkConfig();
    UARTPuts(".", -1);
    GPIO3ModuleClkConfig();
    UARTPuts(".", -1);
    GPIOModuleEnable(SOC_GPIO_3_REGS);
    UARTPuts(".", -1);
    GPIOModuleReset(SOC_GPIO_3_REGS);
    UARTPuts(".OK.\r\n", -1);
}

static void test_green_led()
{
    UARTPuts("Test green LED.", -1);

    // GPIO 3, pin 20
    HWREG(SOC_CONTROL_REGS + CONTROL_CONF_MCASP0_AXR1) = CONTROL_CONF_MUXMODE(7);
    UARTPuts(".", -1);

    GPIODirModeSet(SOC_GPIO_3_REGS,
                   20,
                   GPIO_DIR_OUTPUT);
    UARTPuts(".", -1);

    int i;
    for (i = 0; i < 5; i++) {
        UARTPuts(".", -1);
        GPIOPinWrite(SOC_GPIO_3_REGS,
                     20,
                     GPIO_PIN_HIGH);

        Delay(0xFFFFF);

        UARTPuts("O", -1);
        GPIOPinWrite(SOC_GPIO_3_REGS,
                     20,
                     GPIO_PIN_LOW);
        Delay(0xFFFFF);
    }
    UARTPuts("OK.\r\n", -1);
}


int main(void)
{
    /* Configures PLL and DDR controller*/
    BlPlatformConfig();

    UARTPuts("OSD3358 Diagnostics\r\n\n", -1);

    init_gpio();
    test_green_led();

    UARTPuts("Done.\r\n\n", -1);

    /* Do any post-copy config before leaving boot loader */
    BlPlatformConfigPostBoot();

    UARTPuts("PostBoot done.\r\n\n", -1);
    return 0;
}

void BootAbort(void)
{
    while(1);
}

