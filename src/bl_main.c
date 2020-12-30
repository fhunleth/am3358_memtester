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

#define DDR_SIZE (512 * 1024 * 1024)
#define FAST_SIZE (128 * 1024)

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

    ConsoleUtilsPrintf("\nmemtester for the OSD3358\r\n\nNote: Using Octavo recommended DDR timings.\n\n");

    ConsoleUtilsPrintf("\nFast run (1st %d KB)\r\n\n", FAST_SIZE / 1024);
    memtester((uint32_t *) DDR_START_ADDR, FAST_SIZE, 1);

    ConsoleUtilsPrintf("\nFast run (Last %d KB)\r\n\n", FAST_SIZE / 1024);
    memtester((uint32_t *) (DDR_START_ADDR + DDR_SIZE - FAST_SIZE), FAST_SIZE, 1);

    ConsoleUtilsPrintf("\nSlow runs (%d MB each)\r\n\n", DDR_SIZE / 1024 / 1024);
    memtester((uint32_t *) DDR_START_ADDR, DDR_SIZE, 0);

    return 0;
}

