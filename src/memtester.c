/*
 * Very simple but very effective user-space memory tester.
 * Originally by Simon Kirby <sim@stormix.com> <sim@neato.org>
 * Version 2 by Charles Cazabon <charlesc-memtester@pyropus.ca>
 * Version 3 not publicly released.
 * Version 4 rewrite:
 * Copyright (C) 2004-2020 Charles Cazabon <charlesc-memtester@pyropus.ca>
 * Licensed under the terms of the GNU General Public License version 2 (only).
 * See the file COPYING for details.
 *
 * This file contains the functions for the actual tests, called from the
 * main routine in memtester.c.  See other comments in that file.
 *
 */

#include "memtester.h"
#include "consoleUtils.h"
#include "uartStdio.h"
#include <stdlib.h> // for rand()

#define rand32() ((unsigned int) rand() | ( (unsigned int) rand() << 16))
#define rand_ul() rand32()
#define UL_ONEBITS 0xffffffff
#define UL_LEN 32
#define CHECKERBOARD1 0x55555555
#define CHECKERBOARD2 0xaaaaaaaa
#define UL_BYTE(x) ((x | x << 8 | x << 16 | x << 24))

char progress[] = "-\\|/";
#define PROGRESSLEN 4
#define PROGRESSOFTEN 2500
#define ONE 0x00000001L

union {
    unsigned char bytes[UL_LEN/8];
    uint32_t val;
} mword8;

union {
    unsigned short u16s[UL_LEN/16];
    uint32_t val;
} mword16;

/* Function definitions. */

int compare_regions(uint32_t volatile *bufa, uint32_t volatile *bufb, size_t count) {
    int failures = 0;
    size_t i;
    uint32_t volatile *p1 = bufa;
    uint32_t volatile *p2 = bufb;

    for (i = 0; i < count; i++, p1++, p2++) {
        uint32_t a = *p1;
        uint32_t b = *p2;

        if (a != b) {
            ConsoleUtilsPrintf(
                    "FAILURE: 0x%08x != 0x%08x at offset 0x%08x. (Different bits: 0x%08x)\n",
                    a, b, (uint32_t) (i * sizeof(uint32_t)), a ^ b);

            /* ConsoleUtilsPrintf("Skipping to next test..."); */
            failures++;
            if (failures > 10) {
                ConsoleUtilsPrintf("Skipping to next test...");
                break;
            }
        }
    }

    if (failures == 0)
        return 0;
    else
        return -1;
}

int test_stuck_address(uint32_t volatile *bufa, size_t count) {
    uint32_t volatile *p1 = bufa;
    unsigned int j;
    size_t i;

    ConsoleUtilsPrintf("           ");

    int loops;
    if (count > 1024 * 1024)
        loops = 1;
    else
        loops = 16;

    for (j = 0; j < loops; j++) {
        ConsoleUtilsPrintf("\b\b\b\b\b\b\b\b\b\b\b");
        p1 = (uint32_t volatile *) bufa;
        ConsoleUtilsPrintf("setting %3u", j);

        if (j % 2) {
            for (i = 0; i < count; i += 2) {
                p1[0] = ~((uint32_t) &p1[0]);
                p1[1] = (uint32_t) &p1[1];
                p1 += 2;
            }
        } else {
            for (i = 0; i < count; i += 2) {
                p1[0] = (uint32_t) &p1[0];
                p1[1] = ~((uint32_t) &p1[1]);
                p1 += 2;
            }
        }
        ConsoleUtilsPrintf("\b\b\b\b\b\b\b\b\b\b\b");
        ConsoleUtilsPrintf("testing %3u", j);

        p1 = (uint32_t volatile *) bufa;
        for (i = 0; i < count; i++, p1++) {
            if (*p1 != (((j + i) % 2) == 0 ? (uint32_t) p1 : ~((uint32_t) p1))) {
                ConsoleUtilsPrintf(
                        "FAILURE: possible bad address line at offset "
                        "0x%08x.\n",
                        (uint32_t) (i * sizeof(uint32_t)));

                ConsoleUtilsPrintf("Skipping to next test...\n");

                return -1;
            }
        }
    }
    ConsoleUtilsPrintf("\b\b\b\b\b\b\b\b\b\b\b           \b\b\b\b\b\b\b\b\b\b\b");

    return 0;
}

int test_random_value(uint32_t volatile *bufa, uint32_t volatile *bufb, size_t count) {
    uint32_t volatile *p1 = bufa;
    uint32_t volatile *p2 = bufb;
    uint32_t j = 0;
    size_t i;

    UARTPutc(' ');

    for (i = 0; i < count; i++) {
        *p1++ = *p2++ = rand_ul();
        if (!(i % PROGRESSOFTEN)) {
            UARTPutc('\b');
            UARTPutc(progress[++j % PROGRESSLEN]);

        }
    }
    ConsoleUtilsPrintf("\b \b");

    return compare_regions(bufa, bufb, count);
}

int test_xor_comparison(uint32_t volatile *bufa, uint32_t volatile *bufb, size_t count) {
    uint32_t volatile *p1 = bufa;
    uint32_t volatile *p2 = bufb;
    size_t i;
    uint32_t q = rand_ul();

    for (i = 0; i < count; i++) {
        *p1++ ^= q;
        *p2++ ^= q;
    }
    return compare_regions(bufa, bufb, count);
}

int test_sub_comparison(uint32_t volatile *bufa, uint32_t volatile *bufb, size_t count) {
    uint32_t volatile *p1 = bufa;
    uint32_t volatile *p2 = bufb;
    size_t i;
    uint32_t q = rand_ul();

    for (i = 0; i < count; i++) {
        *p1++ -= q;
        *p2++ -= q;
    }
    return compare_regions(bufa, bufb, count);
}

int test_mul_comparison(uint32_t volatile *bufa, uint32_t volatile *bufb, size_t count) {
    uint32_t volatile *p1 = bufa;
    uint32_t volatile *p2 = bufb;
    size_t i;
    uint32_t q = rand_ul();

    for (i = 0; i < count; i++) {
        *p1++ *= q;
        *p2++ *= q;
    }
    return compare_regions(bufa, bufb, count);
}

int test_div_comparison(uint32_t volatile *bufa, uint32_t volatile *bufb, size_t count) {
    uint32_t volatile *p1 = bufa;
    uint32_t volatile *p2 = bufb;
    size_t i;
    uint32_t q = rand_ul();

    for (i = 0; i < count; i++) {
        if (!q) {
            q++;
        }
        *p1++ /= q;
        *p2++ /= q;
    }
    return compare_regions(bufa, bufb, count);
}

int test_or_comparison(uint32_t volatile *bufa, uint32_t volatile *bufb, size_t count) {
    uint32_t volatile *p1 = bufa;
    uint32_t volatile *p2 = bufb;
    size_t i;
    uint32_t q = rand_ul();

    for (i = 0; i < count; i++) {
        *p1++ |= q;
        *p2++ |= q;
    }
    return compare_regions(bufa, bufb, count);
}

int test_and_comparison(uint32_t volatile *bufa, uint32_t volatile *bufb, size_t count) {
    uint32_t volatile *p1 = bufa;
    uint32_t volatile *p2 = bufb;
    size_t i;
    uint32_t q = rand_ul();

    for (i = 0; i < count; i++) {
        *p1++ &= q;
        *p2++ &= q;
    }
    return compare_regions(bufa, bufb, count);
}

int test_seqinc_comparison(uint32_t volatile *bufa, uint32_t volatile *bufb, size_t count) {
    uint32_t volatile *p1 = bufa;
    uint32_t volatile *p2 = bufb;
    size_t i;
    uint32_t q = rand_ul();

    for (i = 0; i < count; i++) {
        *p1++ = *p2++ = (i + q);
    }
    return compare_regions(bufa, bufb, count);
}

int test_solidbits_comparison(uint32_t volatile *bufa, uint32_t volatile *bufb, size_t count) {
    uint32_t volatile *p1 = bufa;
    uint32_t volatile *p2 = bufb;
    unsigned int j;
    uint32_t q;
    size_t i;

    ConsoleUtilsPrintf("           ");

    for (j = 0; j < 64; j++) {
        ConsoleUtilsPrintf("\b\b\b\b\b\b\b\b\b\b\b");
        q = (j % 2) == 0 ? UL_ONEBITS : 0;
        ConsoleUtilsPrintf("setting %3u", j);

        p1 = (uint32_t volatile *) bufa;
        p2 = (uint32_t volatile *) bufb;
        for (i = 0; i < count; i++) {
            *p1++ = *p2++ = (i % 2) == 0 ? q : ~q;
        }
        ConsoleUtilsPrintf("\b\b\b\b\b\b\b\b\b\b\b");
        ConsoleUtilsPrintf("testing %3u", j);

        if (compare_regions(bufa, bufb, count)) {
            return -1;
        }
    }
    ConsoleUtilsPrintf("\b\b\b\b\b\b\b\b\b\b\b           \b\b\b\b\b\b\b\b\b\b\b");

    return 0;
}

int test_checkerboard_comparison(uint32_t volatile *bufa, uint32_t volatile *bufb, size_t count) {
    uint32_t volatile *p1 = bufa;
    uint32_t volatile *p2 = bufb;
    unsigned int j;
    uint32_t q;
    size_t i;

    ConsoleUtilsPrintf("           ");

    for (j = 0; j < 64; j++) {
        ConsoleUtilsPrintf("\b\b\b\b\b\b\b\b\b\b\b");
        q = (j % 2) == 0 ? CHECKERBOARD1 : CHECKERBOARD2;
        ConsoleUtilsPrintf("setting %3u", j);

        p1 = (uint32_t volatile *) bufa;
        p2 = (uint32_t volatile *) bufb;
        for (i = 0; i < count; i++) {
            *p1++ = *p2++ = (i % 2) == 0 ? q : ~q;
        }
        ConsoleUtilsPrintf("\b\b\b\b\b\b\b\b\b\b\b");
        ConsoleUtilsPrintf("testing %3u", j);

        if (compare_regions(bufa, bufb, count)) {
            return -1;
        }
    }
    ConsoleUtilsPrintf("\b\b\b\b\b\b\b\b\b\b\b           \b\b\b\b\b\b\b\b\b\b\b");

    return 0;
}

int test_blockseq_comparison(uint32_t volatile *bufa, uint32_t volatile *bufb, size_t count) {
    uint32_t volatile *p1 = bufa;
    uint32_t volatile *p2 = bufb;
    unsigned int j;
    size_t i;

    ConsoleUtilsPrintf("           ");

    for (j = 0; j < 256; j++) {
        ConsoleUtilsPrintf("\b\b\b\b\b\b\b\b\b\b\b");
        p1 = (uint32_t volatile *) bufa;
        p2 = (uint32_t volatile *) bufb;
        ConsoleUtilsPrintf("setting %3u", j);

        for (i = 0; i < count; i++) {
            *p1++ = *p2++ = (uint32_t) UL_BYTE(j);
        }
        ConsoleUtilsPrintf("\b\b\b\b\b\b\b\b\b\b\b");
        ConsoleUtilsPrintf("testing %3u", j);

        if (compare_regions(bufa, bufb, count)) {
            return -1;
        }
    }
    ConsoleUtilsPrintf("\b\b\b\b\b\b\b\b\b\b\b           \b\b\b\b\b\b\b\b\b\b\b");

    return 0;
}

int test_walkbits0_comparison(uint32_t volatile *bufa, uint32_t volatile *bufb, size_t count) {
    uint32_t volatile *p1 = bufa;
    uint32_t volatile *p2 = bufb;
    unsigned int j;
    size_t i;

    ConsoleUtilsPrintf("           ");

    for (j = 0; j < UL_LEN * 2; j++) {
        ConsoleUtilsPrintf("\b\b\b\b\b\b\b\b\b\b\b");
        p1 = (uint32_t volatile *) bufa;
        p2 = (uint32_t volatile *) bufb;
        ConsoleUtilsPrintf("setting %3u", j);

        for (i = 0; i < count; i++) {
            if (j < UL_LEN) { /* Walk it up. */
                *p1++ = *p2++ = ONE << j;
            } else { /* Walk it back down. */
                *p1++ = *p2++ = ONE << (UL_LEN * 2 - j - 1);
            }
        }
        ConsoleUtilsPrintf("\b\b\b\b\b\b\b\b\b\b\b");
        ConsoleUtilsPrintf("testing %3u", j);

        if (compare_regions(bufa, bufb, count)) {
            return -1;
        }
    }
    ConsoleUtilsPrintf("\b\b\b\b\b\b\b\b\b\b\b           \b\b\b\b\b\b\b\b\b\b\b");

    return 0;
}

int test_walkbits1_comparison(uint32_t volatile *bufa, uint32_t volatile *bufb, size_t count) {
    uint32_t volatile *p1 = bufa;
    uint32_t volatile *p2 = bufb;
    unsigned int j;
    size_t i;

    ConsoleUtilsPrintf("           ");

    for (j = 0; j < UL_LEN * 2; j++) {
        ConsoleUtilsPrintf("\b\b\b\b\b\b\b\b\b\b\b");
        p1 = (uint32_t volatile *) bufa;
        p2 = (uint32_t volatile *) bufb;
        ConsoleUtilsPrintf("setting %3u", j);

        for (i = 0; i < count; i++) {
            if (j < UL_LEN) { /* Walk it up. */
                *p1++ = *p2++ = UL_ONEBITS ^ (ONE << j);
            } else { /* Walk it back down. */
                *p1++ = *p2++ = UL_ONEBITS ^ (ONE << (UL_LEN * 2 - j - 1));
            }
        }
        ConsoleUtilsPrintf("\b\b\b\b\b\b\b\b\b\b\b");
        ConsoleUtilsPrintf("testing %3u", j);

        if (compare_regions(bufa, bufb, count)) {
            return -1;
        }
    }
    ConsoleUtilsPrintf("\b\b\b\b\b\b\b\b\b\b\b           \b\b\b\b\b\b\b\b\b\b\b");

    return 0;
}

int test_bitspread_comparison(uint32_t volatile *bufa, uint32_t volatile *bufb, size_t count) {
    uint32_t volatile *p1 = bufa;
    uint32_t volatile *p2 = bufb;
    unsigned int j;
    size_t i;

    ConsoleUtilsPrintf("           ");

    for (j = 0; j < UL_LEN * 2; j++) {
        ConsoleUtilsPrintf("\b\b\b\b\b\b\b\b\b\b\b");
        p1 = (uint32_t volatile *) bufa;
        p2 = (uint32_t volatile *) bufb;
        ConsoleUtilsPrintf("setting %3u", j);

        for (i = 0; i < count; i++) {
            if (j < UL_LEN) { /* Walk it up. */
                *p1++ = *p2++ = (i % 2 == 0)
                    ? (ONE << j) | (ONE << (j + 2))
                    : UL_ONEBITS ^ ((ONE << j)
                                    | (ONE << (j + 2)));
            } else { /* Walk it back down. */
                *p1++ = *p2++ = (i % 2 == 0)
                    ? (ONE << (UL_LEN * 2 - 1 - j)) | (ONE << (UL_LEN * 2 + 1 - j))
                    : UL_ONEBITS ^ (ONE << (UL_LEN * 2 - 1 - j)
                                    | (ONE << (UL_LEN * 2 + 1 - j)));
            }
        }
        ConsoleUtilsPrintf("\b\b\b\b\b\b\b\b\b\b\b");
        ConsoleUtilsPrintf("testing %3u", j);

        if (compare_regions(bufa, bufb, count)) {
            return -1;
        }
    }
    ConsoleUtilsPrintf("\b\b\b\b\b\b\b\b\b\b\b           \b\b\b\b\b\b\b\b\b\b\b");

    return 0;
}

int test_bitflip_comparison(uint32_t volatile *bufa, uint32_t volatile *bufb, size_t count) {
    uint32_t volatile *p1 = bufa;
    uint32_t volatile *p2 = bufb;
    unsigned int j, k;
    uint32_t q;
    size_t i;

    ConsoleUtilsPrintf("           ");

    for (k = 0; k < UL_LEN; k++) {
        q = ONE << k;
        for (j = 0; j < 8; j++) {
            ConsoleUtilsPrintf("\b\b\b\b\b\b\b\b\b\b\b");
            q = ~q;
            ConsoleUtilsPrintf("setting %3u", k * 8 + j);

            p1 = (uint32_t volatile *) bufa;
            p2 = (uint32_t volatile *) bufb;
            for (i = 0; i < count; i++) {
                *p1++ = *p2++ = (i % 2) == 0 ? q : ~q;
            }
            ConsoleUtilsPrintf("\b\b\b\b\b\b\b\b\b\b\b");
            ConsoleUtilsPrintf("testing %3u", k * 8 + j);

            if (compare_regions(bufa, bufb, count)) {
                return -1;
            }
        }
    }
    ConsoleUtilsPrintf("\b\b\b\b\b\b\b\b\b\b\b           \b\b\b\b\b\b\b\b\b\b\b");

    return 0;
}

#ifdef TEST_NARROW_WRITES
int test_8bit_wide_random(uint32_t volatile* bufa, uint32_t volatile* bufb, size_t count) {
    uint8_t volatile *p1, *t;
    uint32_t volatile *p2;
    int attempt;
    unsigned int b, j = 0;
    size_t i;

    UARTPutc(' ');

    for (attempt = 0; attempt < 2;  attempt++) {
        if (attempt & 1) {
            p1 = (uint8_t volatile *) bufa;
            p2 = bufb;
        } else {
            p1 = (uint8_t volatile *) bufb;
            p2 = bufa;
        }
        for (i = 0; i < count; i++) {
            t = mword8.bytes;
            *p2++ = mword8.val = rand_ul();
            for (b=0; b < UL_LEN/8; b++) {
                *p1++ = *t++;
            }
            if (!(i % PROGRESSOFTEN)) {
                UARTPutc('\b');
                UARTPutc(progress[++j % PROGRESSLEN]);

            }
        }
        if (compare_regions(bufa, bufb, count)) {
            return -1;
        }
    }
    ConsoleUtilsPrintf("\b \b");

    return 0;
}

int test_16bit_wide_random(uint32_t volatile* bufa, uint32_t volatile* bufb, size_t count) {
    uint16_t volatile *p1, *t;
    uint32_t volatile *p2;
    int attempt;
    unsigned int b, j = 0;
    size_t i;

    UARTPutc(' ');
    for (attempt = 0; attempt < 2; attempt++) {
        if (attempt & 1) {
            p1 = (uint16_t volatile *) bufa;
            p2 = bufb;
        } else {
            p1 = (uint16_t volatile *) bufb;
            p2 = bufa;
        }
        for (i = 0; i < count; i++) {
            t = mword16.u16s;
            *p2++ = mword16.val = rand_ul();
            for (b = 0; b < UL_LEN/16; b++) {
                *p1++ = *t++;
            }
            if (!(i % PROGRESSOFTEN)) {
                UARTPutc('\b');
                UARTPutc(progress[++j % PROGRESSLEN]);
            }
        }
        if (compare_regions(bufa, bufb, count)) {
            return -1;
        }
    }
    ConsoleUtilsPrintf("\b \b");

    return 0;
}
#endif
