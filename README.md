# AM335x memtester

This is a port of the [memtester](http://pyropus.ca/software/memtester/) memory
tester to run from on-chip RAM on AM335x-based boards and test DDR3 memory. It runs
bare metal and has very few requirements on the target device to run. This
repository contains the full source code so it's possible to modify DDR timings
and test regions as needed.

## Running

Download `memtester.bin` from the
[releases](https://github.com/fhunleth/osd3358_diagnostics/releases) or see the
following section for building from source.

Connect your computer to the console UART pins and make sure that the AM335x
tries to boot via UART. You'll know that it's right if the AM335x prints
`CCC...` repeatedly.

I use [Picocom](https://github.com/npat-efault/picocom) to connect over UART,
but any terminal emulator that supports XMODEM is fine. Picocom supports XMODEM
if you install `lrzsz`. On Ubuntu, run this:

```sh
sudo apt install lrzsz picocom
```

Here's an example run on an OSD3358 attached via /dev/ttyUSB1:

```text
$ picocom -b 115200 -s sx /dev/ttyUSB1
picocom v3.1

[TYPE CTRL-A CTRL-S and type in the path to memtester.bin]

*** file: ./memtester.bin
$ sx ./memtester.bin

[POWER UP YOUR AM335x board]

Sending ./memtester.bin, 144 blocks: Give your local XMODEM receive command now.
Bytes Sent:  18560   BPS:3847

Transfer complete

*** exit status: 0 ***

memtester for the OSD3358

Note: Using Octavo recommended DDR timings.


Fast run (1st 128 KB)

Loop 1/1:
  Stuck Address       : ok
  Random Value        : ok
  Compare XOR         : ok
  Compare SUB         : ok
  Compare MUL         : ok
  Compare DIV         : ok
  Compare OR          : ok
  Compare AND         : ok
  Sequential Increment: ok
  Solid Bits          : ok
  Block Sequential    : ok
  Checkerboard        : ok
  Bit Spread          : ok
  Bit Flip            : ok
  Walking Ones        : ok
  Walking Zeroes      : ok
  8-bit Writes        : ok
  16-bit Writes       : ok


Fast run (Last 128 KB)

Loop 1/1:
  Stuck Address       : ok
  Random Value        : ok
  Compare XOR         : ok
  Compare SUB         : ok
  Compare MUL         : ok
  Compare DIV         : ok
  Compare OR          : ok
  Compare AND         : ok
  Sequential Increment: ok
  Solid Bits          : ok
  Block Sequential    : ok
  Checkerboard        : ok
  Bit Spread          : ok
  Bit Flip            : ok
  Walking Ones        : ok
  Walking Zeroes      : ok
  8-bit Writes        : ok
  16-bit Writes       : ok


Slow runs (512 MB each)

Loop 1:
  Stuck Address       : ok
  Random Value        : ok
  Compare XOR         :
...
```

## Building from source

You'll need an ARM crosscompiler. On Ubuntu, run:

```sh
sudo apt install gcc-arm-none-eabi
```

To build, run:

```sh
cd src
make
```

If there's an error, review the gcc path in the `makedefs` file.

See `bl_platform.c` for DDR timings.

## License

`memtester` is licensed under the GNU General Public License version 2
(only). See [COPYING](http://pyropus.ca/software/memtester/COPYING).
