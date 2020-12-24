#include "uartStdio.h"
#include "bl_platform.h"
#include "bl.h"

#include "gpio_v2.h"
#include "hw/hw_control_AM335x.h"
#include "hw/hw_types.h"
#include "hw/hw_cm_per.h"

#include <stdint.h>

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

        Delay(0x1FFFFF);

        UARTPuts("+", -1);
        GPIOPinWrite(SOC_GPIO_3_REGS,
                     20,
                     GPIO_PIN_LOW);
        Delay(0x1FFFFF);
    }
    UARTPuts("OK.\r\n", -1);
}

// See https://barrgroup.com/embedded-systems/how-to/memory-test-suite-c
static uint32_t ddr_test_data_bus(volatile uint32_t *address)
{
    uint32_t pattern;

    /*
     * Perform a walking 1's test at the given address.
     */
    for (pattern = 1; pattern != 0; pattern <<= 1) {
        /*
         * Write the test pattern.
         */
        *address = pattern;

        /*
         * Read it back (immediately is okay for this test).
         */
        if (*address != pattern)
            return pattern;
    }

    return 0;
}

static uint32_t *ddr_test_address_bus(volatile uint32_t *base_address, unsigned long num_bytes)
{
    uint32_t address_mask = (num_bytes/sizeof(uint32_t) - 1);
    uint32_t offset;
    uint32_t test_offset;

    uint32_t pattern     = 0xAAAAAAAA;
    uint32_t antipattern = 0x55555555;

    /*
     * Write the default pattern at each of the power-of-two offsets.
     */
    for (offset = 1; (offset & address_mask) != 0; offset <<= 1)
    {
        base_address[offset] = pattern;
    }

    /*
     * Check for address bits stuck high.
     */
    test_offset = 0;
    base_address[test_offset] = antipattern;

    for (offset = 1; (offset & address_mask) != 0; offset <<= 1)
    {
        if (base_address[offset] != pattern)
        {
            return ((uint32_t *) &base_address[offset]);
        }
    }

    base_address[test_offset] = pattern;

    /*
     * Check for address bits stuck low or shorted.
     */
    for (test_offset = 1; (test_offset & address_mask) != 0; test_offset <<= 1)
    {
        base_address[test_offset] = antipattern;

		if (base_address[0] != pattern)
		{
			return ((uint32_t *) &base_address[test_offset]);
		}

        for (offset = 1; (offset & address_mask) != 0; offset <<= 1)
        {
            if ((base_address[offset] != pattern) && (offset != test_offset))
            {
                return ((uint32_t *) &base_address[test_offset]);
            }
        }

        base_address[test_offset] = pattern;
    }

    return (NULL);

}

uint32_t *ddr_test_device(volatile uint32_t *baseAddress, uint32_t num_bytes)
{
    uint32_t offset;
    uint32_t num_words = num_bytes / sizeof(uint32_t);

    uint32_t pattern;
    uint32_t antipattern;

    /*
     * Fill memory with a known pattern.
     */
    for (pattern = 1, offset = 0; offset < num_words; pattern++, offset++)
    {
        baseAddress[offset] = pattern;
    }

    /*
     * Check each location and invert it for the second pass.
     */
    for (pattern = 1, offset = 0; offset < num_words; pattern++, offset++)
    {
        if (baseAddress[offset] != pattern)
        {
            return ((uint32_t *) &baseAddress[offset]);
        }

        antipattern = ~pattern;
        baseAddress[offset] = antipattern;
    }

    /*
     * Check each location for the inverted pattern and zero it.
     */
    for (pattern = 1, offset = 0; offset < num_words; pattern++, offset++)
    {
        antipattern = ~pattern;
        if (baseAddress[offset] != antipattern)
        {
            return ((uint32_t *) &baseAddress[offset]);
        }
    }

    return (NULL);

}

static void pass()
{
    UARTPuts("OK.\r\n", -1);
}
static void fail()
{
    UARTPuts("FAIL!\r\n", -1);
}
static void fail_if_nonzero(uint32_t result)
{
    if (result == 0)
        pass();
    else
        fail();
}

static void test_ddr()
{
    uint32_t *ddr_start = (uint32_t *) DDR_START_ADDR;
    uint32_t num_bytes = 1024 * 1024;

    UARTPuts("Testing DDR data bus...", -1);
    fail_if_nonzero(ddr_test_data_bus(ddr_start));

    UARTPuts("Testing DDR address bus...", -1);
    fail_if_nonzero((uint32_t) ddr_test_address_bus(ddr_start, num_bytes));

    UARTPuts("Testing DDR device...", -1);
    fail_if_nonzero((uint32_t) ddr_test_device(ddr_start, num_bytes));
}

int main(void)
{
    /* Configures PLL and DDR controller*/
    BlPlatformConfig();

    UARTPuts("OSD3358 Diagnostics\r\n\n", -1);

    init_gpio();
    test_green_led();
    test_ddr();
    UARTPuts("\r\n\nDone.\r\n\n", -1);

    return 0;
}

