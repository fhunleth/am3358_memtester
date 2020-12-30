#include "uartStdio.h"
#include "consoleUtils.h"
#include "bl_platform.h"
#include "bl.h"

#include "device.h"
#include "gpio_v2.h"
#include "hw/hw_control_AM335x.h"
#include "hw/hw_types.h"
#include "hw/hw_cm_per.h"

#include "memtester.h"
#include <stdint.h>
#include <string.h>

static void dump_info()
{
    UARTPuts("AM3358 Info\r\n\n", -1);
    int deviceVersion = DeviceVersionGet();
    unsigned int oppSupport = SysConfigOppDataGet();

    ConsoleUtilsPrintf("Device version: %d\r\n", deviceVersion);
    ConsoleUtilsPrintf("SysConfig OPP Data:");
    if(DEVICE_VERSION_1_0 == deviceVersion)
        ConsoleUtilsPrintf(" 720");
    else if(DEVICE_VERSION_2_0 == deviceVersion)
        ConsoleUtilsPrintf(" 800");
    else if(DEVICE_VERSION_2_1 == deviceVersion) {
        if(!(oppSupport & EFUSE_OPPNT_1000_MASK))
            ConsoleUtilsPrintf(" NT_1000");
        if(!(oppSupport & EFUSE_OPPTB_800_MASK))
            ConsoleUtilsPrintf(" TB_800");
        if(!(oppSupport & EFUSE_OPP120_720_MASK))
            ConsoleUtilsPrintf(" 120_720");
        if(!(oppSupport & EFUSE_OPP100_600_MASK))
            ConsoleUtilsPrintf(" 100_600");
        if(!(oppSupport & EFUSE_OPP100_300_MASK))
            ConsoleUtilsPrintf(" 100_300");
    }
    ConsoleUtilsPrintf("\r\n");
}

struct test {
    char *name;
    int (*fp)();
};

static struct test tests[] = {
    { "Random Value", test_random_value },
    { "Compare XOR", test_xor_comparison },
    { "Compare SUB", test_sub_comparison },
    { "Compare MUL", test_mul_comparison },
    { "Compare DIV",test_div_comparison },
    { "Compare OR", test_or_comparison },
    { "Compare AND", test_and_comparison },
    { "Sequential Increment", test_seqinc_comparison },
    { "Solid Bits", test_solidbits_comparison },
    { "Block Sequential", test_blockseq_comparison },
    { "Checkerboard", test_checkerboard_comparison },
    { "Bit Spread", test_bitspread_comparison },
    { "Bit Flip", test_bitflip_comparison },
    { "Walking Ones", test_walkbits1_comparison },
    { "Walking Zeroes", test_walkbits0_comparison },
#ifdef TEST_NARROW_WRITES
    { "8-bit Writes", test_8bit_wide_random },
    { "16-bit Writes", test_16bit_wide_random },
#endif
    { NULL, NULL }
};

static int memtester(volatile uint32_t *base_address, size_t buf_size, int loops)
{
    size_t halflen = buf_size / 2;
    size_t count = halflen / sizeof(uint32_t);
    uint32_t volatile *bufa = base_address;
    uint32_t volatile *bufb = (uint32_t volatile *) ((size_t) base_address + halflen);
    int rc = 0;

    for(int loop=1; ((!loops) || loop <= loops); loop++) {
        ConsoleUtilsPrintf("Loop %u", loop);
        if (loops) {
            ConsoleUtilsPrintf("/%u", loops);
        }
        ConsoleUtilsPrintf(":\n");
        ConsoleUtilsPrintf("  %20s: ", "Stuck Address");
        if (!test_stuck_address(base_address, buf_size / sizeof(uint32_t))) {
             ConsoleUtilsPrintf("ok\n");
        } else {
            rc |= 1;
        }
        for (int i=0;;i++) {
            if (!tests[i].name) break;

            ConsoleUtilsPrintf("  %20s: ", tests[i].name);
            if (!tests[i].fp(bufa, bufb, count)) {
                ConsoleUtilsPrintf("ok\n");
            } else {
                rc |= 2;
            }
            /* clear buffer */
            memset((void*) base_address, 255, buf_size);
        }
        ConsoleUtilsPrintf("\n");
    }
    return rc;
}

int main(void)
{
    /* Configures PLL and DDR controller*/
    BlPlatformConfig();

    dump_info();

    UARTPuts("\nFast run (128 KB)\r\n\n", -1);
    memtester((uint32_t *) DDR_START_ADDR, 128 * 1024, 1);

    UARTPuts("\nSlow runs (512 MB)\r\n\n", -1);
    memtester((uint32_t *) DDR_START_ADDR, 512 * 1024 * 1024, 1);

    return 0;
}

