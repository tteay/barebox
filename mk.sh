#!/bin/bash
export PATH=/home/lanbo/hd/ics/src/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin:$PATH
#export PATH=/home/lanbo/hd/android4.2/prebuilts/gcc/linux-x86/arm/arm-eabi-4.8/bin:$PATH
export ARCH=arm
export CROSS_COMPILE=arm-eabi-


if [ "$1" = "distclean" ]|| [ "$1" = "clean" ]; then
        make distclean
fi

#make mrproper
make im98xxv4_wvga_A9-806MHz-AHB-div3_XM-198MHz_A7-130MHz_defconfig

# make .config
make

