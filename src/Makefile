#
# Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
#
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
#
#******************************************************************************

#
# Locating the root directory
#
STARTERWARE=../starterware

#
# Device and EVM definitions
#
DEVICE=am335x
EVM=beaglebone

#
# Include the makefile definitions. This contains compiler, linker and
# archiver definitions and options
#
include ./makedefs

#
# Target Directories that need to be built
#
DIRS=${DRIVERS_BLD} ${PLATFORM_BLD} ${UTILITY_BLD}

#
# The application directory and name
#
APPDIR=bootloader
APPNAME=boot

#
# Where the bootloader will be loaded to by the ROM code
# starting 1Kbyte cannot be used due to hardware limitation
START_ADDR=0x402F0400


#
# Application Location
#
APP=${ROOT}bootloader/
APP_BIN=${ROOT}/binary/${TARGET}/${COMPILER}/${DEVICE}/${EVM}/$(APPDIR)

CFLAGS+=-D $(BOOT) -D $(DEVICE)

#
# Application source files
#
STARTERWARE_SRC=\
       ${STARTERWARE}/system_config/armv7a/cache.c\
       ${STARTERWARE}/system_config/armv7a/mmu.c\
       ${STARTERWARE}/system_config/armv7a/gcc/cp15.S\
       ${STARTERWARE}/system_config/armv7a/$(DEVICE)/device.c\
       ${STARTERWARE}/platform/beaglebone/board.c\
       ${STARTERWARE}/platform/beaglebone/cpsw.c\
       ${STARTERWARE}/platform/beaglebone/ctlregcontext.c\
       ${STARTERWARE}/platform/beaglebone/dmtimer.c\
	${STARTERWARE}/platform/beaglebone/edma.c\
	${STARTERWARE}/platform/beaglebone/eeprom.c\
	${STARTERWARE}/platform/beaglebone/gpio.c\
	${STARTERWARE}/platform/beaglebone/hsi2c.c\
        ${STARTERWARE}/platform/beaglebone/hs_mmcsd.c\
	${STARTERWARE}/platform/beaglebone/rtc.c\
	${STARTERWARE}/platform/beaglebone/sysdelay.c\
	${STARTERWARE}/platform/beaglebone/sysperf.c\
	${STARTERWARE}/platform/beaglebone/timertick.c\
	${STARTERWARE}/platform/beaglebone/uart.c\
	${STARTERWARE}/platform/beaglebone/uartConsole.c\
	${STARTERWARE}/platform/beaglebone/usb.c\
	${STARTERWARE}/platform/beaglebone/watchdog.c\
       ${STARTERWARE}/drivers/hsi2c.c\
       ${STARTERWARE}/utils/uartStdio.c

APP_SRC=${STARTERWARE_SRC}\
	bl_init.S\
	bl_platform.c\
	bl_main.c

#
# Rules for building the application and library
#
all: release

release:
	#make TARGET_MODE=release lib
	make TARGET_MODE=Release bin

lib:
	@for i in ${DIRS};				\
	do						\
		if [ -f $${i}/makefile ] ;		    \
		then					  \
			make Release -C $${i} || exit $$?; \
		fi;					   \
	done;


bin:
	$(CC)  $(CFLAGS) $(APP_SRC)
	@mkdir -p $(TARGET_MODE)_$(BOOT)/
	@mv *.o* $(TARGET_MODE)_$(BOOT)/
	$(LD) ${LDFLAGS} ${LPATH} -o $(TARGET_MODE)_$(BOOT)/$(APPNAME).out -Map $(TARGET_MODE)_$(BOOT)/$(APPNAME).map \
	      $(TARGET_MODE)_$(BOOT)/*.o*  --defsym BOOT_START_ADDR=$(START_ADDR) -T $(APPNAME).lds $(APP_LIB) -lc -lgcc $(APP_LIB) -lc -lgcc
	@mkdir -p $(APP_BIN)/$(TARGET_MODE)_$(BOOT)
	@cp $(TARGET_MODE)_$(BOOT)/$(APPNAME).out $(APP_BIN)/$(TARGET_MODE)_$(BOOT)/$(APPNAME).out
	$(BIN) $(BINFLAGS) $(APP_BIN)/$(TARGET_MODE)_$(BOOT)/$(APPNAME).out \
               $(APP_BIN)/$(TARGET_MODE)_$(BOOT)/$(APPNAME).bin


#
# Rules for cleaning
#
clean:
	@rm -rf Debug_MMCSD Debug_UART \
                Release_MMCSD Release_UART \
                $(APP_BIN)/Debug_MMCSD $(APP_BIN)/Debug_UART \
                $(APP_BIN)/Release_MMCSD $(APP_BIN)/Release_UART

clean+: clean
	@make TARGET_MODE=clean lib


