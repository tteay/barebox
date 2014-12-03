#!/bin/bash
export PATH=/home/lanbo/hd/ics/src/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin:$PATH
#export PATH=/home/lanbo/hd/android4.2/prebuilts/gcc/linux-x86/arm/arm-eabi-4.8/bin:$PATH
export ARCH=arm
export CROSS_COMPILE=arm-eabi-

if [ "$1" = "h" ];then
    echo "this shell could get 3 param as follow"
    echo "c :make distclean"
    echo "s :select default config"
    echo "m :start menuconfig"
else
if [ "$1" = "c" ] || [ "$2" = "c" ]|| [ "$3" = "c" ]; then
echo "================= make distclean ====================="
        make distclean
fi

#make mrproper
if [ "$1" = "s" ] || [ "$2" = "s" ]|| [ "$3" = "s" ]; then
echo "================= make defaultconfig ================="
make im98xxv4_wvga_A9-806MHz-AHB-div3_XM-198MHz_A7-130MHz_defconfig
fi

if [ "$1" = "m" ] || [ "$2" = "m" ]|| [ "$3" = "m" ]; then
echo "================= make menuconfig ===================="
        make menuconfig
fi

# make .config
echo "================== make =============================="
make
fi
