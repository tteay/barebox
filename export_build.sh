#!/bin/bash
export PATH=/home/lanbo/hd/ics/src/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin:$PATH
#export PATH=/home/lanbo/hd/android4.2/prebuilts/gcc/linux-x86/arm/arm-eabi-4.8/bin:$PATH
#export PATH=/work/android4.2/prebuilts/gcc/linux-x86/arm/arm-eabi-4.8/bin:$PATH
export ARCH=arm
export CROSS_COMPILE=arm-eabi-

echo ' ' > include/git_commit.h
echo '#if defined(GIT_COMMIT)' >> include/git_commit.h
echo '#define GIT_RELEASE GIT_COMMIT' >> include/git_commit.h
echo '#else' >> include/git_commit.h
echo '#define GIT_RELEASE " export"' >> include/git_commit.h
echo '#endif' >> include/git_commit.h

BOARD_LIST=( iM9816 \
	     iM98xx_EVB_V1 \
	     iM98xx_EVB_V2 \
	     iM98xx_EVB_V3 \
	     iM98xx_EVB_V3_WVGA \
	     iM98xx_EVB_V4 \
	     iM98xx_EVB_V4_WVGA )

CONFIG_LIST=( im98xxv1_A9-520MHz-AHB-div2_XM-198MHz_A7-143MHz_defconfig \
	      im98xxv1_A9-520MHz-AHB-div2_XM-198MHz_A7-143MHz_defconfig \
	      im98xxv2_A9-520MHz-AHB-div2_XM-198MHz_A7-143MHz_defconfig \
	      im98xxv3_A9-624MHz-AHB-div2_XM-198MHz_A7-143MHz_defconfig \
	      im98xxv3_wvga_A9-624MHz-AHB-div2_XM-198MHz_A7-143MHz_defconfig \
	      im98xxv4_A9-806MHz-AHB-div3_XM-198MHz_A7-130MHz_defconfig \
	      im98xxv4_wvga_A9-806MHz-AHB-div3_XM-198MHz_A7-130MHz_defconfig )

IMAGE_NAME_LIST=( im98xxv1_A9-520MHz-AHB-div2_XM-198MHz_A7-143MHz \
		  im98xxv1_A9-520MHz-AHB-div2_XM-198MHz_A7-143MHz \
		  im98xxv2_A9-520MHz-AHB-div2_XM-198MHz_A7-143MHz \
		  im98xxv3_A9-624MHz-AHB-div2_XM-198MHz_A7-143MHz \
		  im98xxv3_wvga_A9-624MHz-AHB-div2_XM-198MHz_A7-143MHz \
		  im98xxv4_A9-806MHz-AHB-div3_XM-198MHz_A7-130MHz \
		  im98xxv4_wvga_A9-806MHz-AHB-div3_XM-198MHz_A7-130MHz )

if [ "$1" ] ; then
    BOARD_NUMBER=$1
    CONFIG_TYPE=${CONFIG_LIST[$BOARD_NUMBER]}
    echo "Select $CONFIG_TYPE"
    if [ "$2" ] ; then
        make distclean
    fi
    make $CONFIG_TYPE
    if [ "$3" ] ; then
        make 2>&1 | tee $3
    else
        make
    fi
    cp ./barebox.bin ../infomax_images/barebox.bin_${IMAGE_NAME_LIST[$BOARD_NUMBER]}
else
    for element in $(seq 0 $((${#BOARD_LIST[@]} - 1)))
    do
        echo $element ${BOARD_LIST[$element]}
    done
    
    read -p "Which Board do you choice?" BOARD_NUMBER

    CONFIG_TYPE=${CONFIG_LIST[$BOARD_NUMBER]}
    echo $CONFIG_TYPE

    read -p "Clean? (y/n) " isClean
    if [ "$isClean" = "y" ] || [ "$isClean" = "Y" ]; then
        echo "Cleaning"
        make distclean

        echo "Configing"
        echo ""
        echo "Select $CONFIG_TYPE"
        make $CONFIG_TYPE
        echo ""

        read -p "Menuconfig? (y/n) " isConfig
        if [ "$isConfig" = "y" ] || [ "$isConfig" = "Y" ]; then
            echo "Menuconfig"
            make menuconfig
        fi

        echo "Building"
        make

        if [ "$?" = "0" ]; then
            echo  "\n*** Barebox Build Completed Sucessfully. ***\n"
            exit 0
        else
            echo "\n*** Barebox Build Failed. ***\n"
            exit 1
        fi
    else
        read -p "Menuconfig? (y/n) " isConfig
        if [ "$isConfig" = "y" ] || [ "$isConfig" = "Y" ]; then
            echo "Menuconfig"
            make menuconfig
        fi

        echo "Building"
        make

        if [ "$?" = "0" ]; then
            cp ./barebox.bin ./../infomax_images/barebox.bin_${IMAGE_NAME_LIST[$BOARD_NUMBER]}
            echo  "\n*** Barebox Build Completed Sucessfully. ***\n"
            exit 0
        else
            echo "\n*** Barebox Build Failed. ***\n"
            exit 1
        fi
    fi

fi

exit 0
