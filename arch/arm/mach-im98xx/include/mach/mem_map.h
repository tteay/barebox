#ifndef MEM_MAP_H
#define MEM_MAP_H

/*
 *
	Hippo Memory Map
 *
 */

// define all possible ARM9 CPU operating speeds
#if defined(CONFIG_IM98XX_ARM9_CLOCK_416_CONFIG)
	#define ARM9_CPU_SPEED  416		// in MHz
#elif defined(CONFIG_IM98XX_ARM9_CLOCK_494_CONFIG)
	#define ARM9_CPU_SPEED  494		// in MHz
#elif defined(CONFIG_IM98XX_ARM9_CLOCK_520_CONFIG)
	#define ARM9_CPU_SPEED  520		// in MHz
#elif defined(CONFIG_IM98XX_ARM9_CLOCK_624_CONFIG)
	#define ARM9_CPU_SPEED  624		// in MHz
#elif defined(CONFIG_IM98XX_ARM9_CLOCK_650_CONFIG)
	#define ARM9_CPU_SPEED  650		// in MHz
#elif defined(CONFIG_IM98XX_ARM9_CLOCK_702_CONFIG)
	#define ARM9_CPU_SPEED  702		// in MHz
#elif defined(CONFIG_IM98XX_ARM9_CLOCK_728_CONFIG)
	#define ARM9_CPU_SPEED  728		// in MHz
#elif defined(CONFIG_IM98XX_ARM9_CLOCK_754_CONFIG)
	#define ARM9_CPU_SPEED  754		// in MHz
#elif defined(CONFIG_IM98XX_ARM9_CLOCK_780_CONFIG)
	#define ARM9_CPU_SPEED  780		// in MHz
#elif defined(CONFIG_IM98XX_ARM9_CLOCK_806_CONFIG)
	#define ARM9_CPU_SPEED  806		// in MHz
#elif defined(CONFIG_IM98XX_ARM9_CLOCK_910_CONFIG)
	#define ARM9_CPU_SPEED  910		// in MHz
#endif


//#define BOOT_NOR_V

//#define ARM7
#define ARM9

/*
	Interanl SRAM
 */
/* ARM9 SRAM 91~94 four bank with size 32KB */
#define ARM9_SRAM1_START	0x1FFF8000	/* SRAM91 Bank1 */
#define ARM9_SRAM2_START	0x1FFF0000	/* SRAM92 Bank2 */
#define ARM9_SRAM3_START	0x1FFE8000	/* SRAM93 Bank3 */
#define ARM9_SRAM4_START	0x1FFE0000	/* SRAM94 Bank4 */
#define ARM9_SRAM_SIZE		0x8000		/* 32KB (256Kb) */
/* ARM7 SRAM 70~74 five bank with size 64KB */
#define ARM7_SRAM0_START	0x00000000	/* SRAM70 Bank5 */
#define ARM7_SRAM1_START	0x20000000	/* SRAM71 Bank1 */
#define ARM7_SRAM2_START	0x20010000	/* SRAM72 Bank2 */
#define ARM7_SRAM3_START	0x20020000	/* SRAM73 Bank3 */
#define ARM7_SRAM4_START	0x20030000	/* SRAM74 Bank4 */
#define ARM7_SRAM_SIZE		0x10000		/* 64KB (512Kb) */
/* Asynchronous access as Data R/W */
#if (defined (ARM7) )
#define ARM7_SRAM91_START	(ARM9_SRAM1_START)
#define ARM7_SRAM92_START	(ARM9_SRAM2_START)
#define ARM7_SRAM93_START	(ARM9_SRAM3_START)
#define ARM7_SRAM94_START	(ARM9_SRAM4_START)
#endif
#if (defined (ARM9) )
#define ARM9_SRAM70_START	(ARM7_SRAM0_START + 0x28080000)
#define ARM9_SRAM71_START	(ARM7_SRAM1_START)
#define ARM9_SRAM72_START	(ARM7_SRAM2_START)
#define ARM9_SRAM73_START	(ARM7_SRAM3_START)
#define ARM9_SRAM74_START	(ARM7_SRAM4_START)
#endif
/* Synchronous access as Instruction/Data */
#if (defined (ARM7) )
#define ARM7_SRAM70_START	(ARM7_SRAM0_START)
#define ARM7_SRAM71_START	(ARM7_SRAM1_START)
#define ARM7_SRAM72_START	(ARM7_SRAM2_START)
#define ARM7_SRAM73_START	(ARM7_SRAM3_START)
#define ARM7_SRAM74_START	(ARM7_SRAM4_START)
#endif
#if (defined (ARM9) )
#define ARM9_SRAM91_START	(ARM9_SRAM1_START)
#define ARM9_SRAM92_START	(ARM9_SRAM2_START)
#define ARM9_SRAM93_START	(ARM9_SRAM3_START)
#define ARM9_SRAM94_START	(ARM9_SRAM4_START)
#endif

/*
	Exteranl Memory PS: 4+2 MB, 32+16 Mb
 */
/* External NOR Flash */
#define NOR_32M			0x400000
#define NOR_START		0x10000000
#define NOR_SIZE		(NOR_32M)
/* External pSRAM */
#define PSRAM_16M		0x200000
#define PSRAM_START		0x14000000
#define PSRAM_SIZE		(PSRAM_16M)
/* External SDRAM */
#define SDRAM_2G		0x10000000	/* (2Gb/256MB) */
#define SDRAM_1G		0x08000000	/* (1Gb/128MB) */
#define SDRAM_START		0x40000000	/* SDRAM offset */
#define SDRAM_SIZE		(SDRAM_2G)
#define TEST_SDRAM_SIZE		(0x10000)

/* SDRAM  parameters*/
/************************************************/
// SDRAM physical meaning
/************************************************/
#define	SDM_A7_LINEAR		(unsigned int) 0x00000004
#define	SDM_A9D_LINEAR		(unsigned int) 0x00000002
#define	SDM_DMA_RD_DIR		(unsigned int) 0x00000000

//---SDM_DLL_CTL_REG
#define	DLL_CAL_EN		(unsigned int) 0x00000001
#define	DLL_CAL_RETURN		(unsigned int) 0x00000002
#define	DLL_CAL_LOCK		(unsigned int) 0x00000004
#define	DLL_CAL_ERR		(unsigned int) 0x00000008
//---SDM_SIZ_CFG_REG
#define	SDM_2_BNK		(unsigned int) 0x00000000
#define	SDM_4_BNK		(unsigned int) 0x00000001
#define	SDM_ROW_11bit		(unsigned int) 0x00000000
#define	SDM_ROW_12bit		(unsigned int) 0x00000004
#define	SDM_ROW_13bit		(unsigned int) 0x00000008
#define	SDM_ROW_14bit		(unsigned int) 0x0000000C
#define	SDM_COL_08bit		(unsigned int) 0x00000000
#define	SDM_COL_09bit		(unsigned int) 0x00000020
#define	SDM_COL_10bit		(unsigned int) 0x00000040
#define	SDM_COL_11bit		(unsigned int) 0x00000060
#define	SDM_COL_12bit		(unsigned int) 0x00000080
#define	SDM_BUS_16bit		(unsigned int) 0x00000000
#define	SDM_BUS_32bit		(unsigned int) 0x00000100
#define	SDM_SDR			(unsigned int) 0x00000000
#define	SDM_DDR			(unsigned int) 0x00000200
//---SDM_RFS_CTL_REG
#define	SDM_ARF_32cyc		(unsigned int) 0x0000001F
#define	SDM_ARF_780cyc		(unsigned int) 0x0000030B
#define	SDM_ARF_1560cyc		(unsigned int) 0x00000617
#define	SDM_ARF_6240cyc		(unsigned int) 0x0000185F
#define	SDM_ARF_12480cyc	(unsigned int) 0x000030BF
#define	SDM_BRF_INI_2cyc	(unsigned int) 0x00010000
#define	SDM_BRF_INI_16cyc	(unsigned int) 0x000F0000
#define	SDM_BRF_OPR_1time	(unsigned int) 0x00000000
#define	SDM_BRF_OPR_8time	(unsigned int) 0x00700000
#define	SDM_BRF_OPR_16time	(unsigned int) 0x00F00000
#define	SDM_BRF_SR_8time	(unsigned int) 0x03000000
#define	SDM_BRF_SR_1Ktime	(unsigned int) 0x08000000
//---SDM_MOD_CTL_REG
#define	SDM_INIT		(unsigned int) 0x00000001
#define	SDM_MR			(unsigned int) 0x00000002
#define	SDM_EMR			(unsigned int) 0x00000004
#define	SDM_SR_ETR		(unsigned int) 0x00000008
#define	SDM_SR_EXT		(unsigned int) 0x00000000
#define	SDM_DPD_ETR		(unsigned int) 0x00000010
#define	SDM_DPD_EXT		(unsigned int) 0x00000000
#define	SDM_APD_NON		(unsigned int) 0x00000000
#define	SDM_APD_DYN_PRG		(unsigned int) 0x00000060
#define	SDM_APD_DYN		(unsigned int) 0x00000020
#define	SDM_APD_PRG_ONLY	(unsigned int) 0x00000040
#define	SDM_APD_PRG		(unsigned int) 0x00000040
#define	SDM_TUN_DLY		(unsigned int) 0x00000080
//#define SDM_WAT_to_APD_1cyc	(unsigned int) 0x00000000
//#define SDM_WAT_to_APD_3cyc	(unsigned int) 0x00000200
#define	SDM_WAT_to_APD_6cyc	(unsigned int) 0x00000500
#define	SDM_WAT_to_APD_7cyc	(unsigned int) 0x00000600
#define	SDM_WAT_to_APD_8cyc	(unsigned int) 0x00000700
#define	SDM_WAT_to_APD_16cyc	(unsigned int) 0x00000F00
#define	SDM_WAT_to_ASR_01	(unsigned int) 0x00000000
#define	SDM_WAT_to_ASR_02	(unsigned int) 0x00100000
#define	SDM_WAT_to_ASR_04	(unsigned int) 0x00300000
#define	SDM_WAT_to_ASR_08	(unsigned int) 0x00700000
#define	SDM_WAT_to_ASR_16	(unsigned int) 0x00F00000
#define	SDM_WAT_to_ASR_MASK	(unsigned int) 0xFFF00000
//---SDM_MOD_SET_REG
#define SDM_BL_2		(unsigned int) 0x00000001
#define SDM_BL_4		(unsigned int) 0x00000002
#define SDM_BL_8		(unsigned int) 0x00000003
#define SDM_BL_16		(unsigned int) 0x00000004
#define	SDM_CAS_2		(unsigned int) 0x00000020
#define	SDM_CAS_3		(unsigned int) 0x00000030
#define SDM_EMR_BA_0		(unsigned int) 0x00000000
#define SDM_EMR_BA_1		(unsigned int) 0x00010000
#define SDM_EMR_BA_2		(unsigned int) 0x00020000
#define SDM_EMR_BA_3		(unsigned int) 0x00030000
//---SDM_EXT_MOD_REG
#define	SDM_PASR_FUL		(unsigned int) 0x00000000
#define	SDM_PASR_HAF		(unsigned int) 0x00000001	/*** #define	SDM_TCSR	(unsigned int) 0x00000000 ***/
#define	SDM_DRI_STNG_FUL	(unsigned int) 0x00000000
#define	SDM_DRI_STNG_HAF	(unsigned int) 0x00000020
#define SDM_DRI_STNG_QUA	(unsigned int) 0x00000040
#define SDM_DRI_STNG_OCT	(unsigned int) 0x00000060
//---SDM_WRP_STA_REG
#define	SDM_FLUSH		(unsigned int) 0x00000001

//---SDM_AC_TIME_REG
#define	SDM_tRCD_2cyc		(unsigned int) 0x00000000
#define	SDM_tRCD_3cyc		(unsigned int) 0x00000001
#define	SDM_tRCD_4cyc		(unsigned int) 0x00000002
#define	SDM_tRCD_5cyc		(unsigned int) 0x00000003
#define	SDM_tRCD_6cyc		(unsigned int) 0x00000004
#define	SDM_tRCD_7cyc		(unsigned int) 0x00000005
#define	SDM_tRCD_8cyc		(unsigned int) 0x00000006
#define	SDM_tRCD_9cyc		(unsigned int) 0x00000007
#define	SDM_tRAS_1cyc		(unsigned int) 0x00000000
#define	SDM_tRAS_2cyc		(unsigned int) 0x00000010
#define	SDM_tRAS_3cyc		(unsigned int) 0x00000020
#define	SDM_tRAS_4cyc		(unsigned int) 0x00000030
#define	SDM_tRAS_5cyc		(unsigned int) 0x00000040
#define	SDM_tRAS_6cyc		(unsigned int) 0x00000050
#define	SDM_tRAS_7cyc		(unsigned int) 0x00000060
#define	SDM_tRAS_8cyc		(unsigned int) 0x00000070
#define	SDM_tRP_2cyc		(unsigned int) 0x00000000
#define	SDM_tRP_3cyc		(unsigned int) 0x00000100
#define	SDM_tRP_4cyc		(unsigned int) 0x00000200
#define	SDM_tRP_5cyc		(unsigned int) 0x00000300
#define	SDM_tRP_6cyc		(unsigned int) 0x00000400
#define	SDM_tRP_7cyc		(unsigned int) 0x00000500
#define	SDM_tRP_8cyc		(unsigned int) 0x00000600
#define	SDM_tRP_9cyc		(unsigned int) 0x00000700
#define	SDM_tRFC_2cyc		(unsigned int) 0x00000000
#define	SDM_tRFC_3cyc		(unsigned int) 0x00001000
#define	SDM_tRFC_4cyc		(unsigned int) 0x00002000
#define	SDM_tRFC_5cyc		(unsigned int) 0x00003000
#define	SDM_tRFC_6cyc		(unsigned int) 0x00004000
#define	SDM_tRFC_7cyc		(unsigned int) 0x00005000
#define	SDM_tRFC_8cyc		(unsigned int) 0x00006000
#define	SDM_tRFC_9cyc		(unsigned int) 0x00007000
#define	SDM_tRFC_10cyc		(unsigned int) 0x00008000
#define	SDM_tRFC_11cyc		(unsigned int) 0x00009000
#define	SDM_tRFC_12cyc		(unsigned int) 0x0000A000
#define	SDM_tRFC_13cyc		(unsigned int) 0x0000B000
#define	SDM_tRFC_14cyc		(unsigned int) 0x0000C000
#define	SDM_tRFC_15cyc		(unsigned int) 0x0000D000
#define	SDM_tRFC_16cyc		(unsigned int) 0x0000E000
#define	SDM_tRFC_17cyc		(unsigned int) 0x0000F000
#define	SDM_tRFC_18cyc		(unsigned int) 0x00010000
#define	SDM_tRFC_19cyc		(unsigned int) 0x00011000
#define	SDM_tRFC_20cyc		(unsigned int) 0x00012000
#define	SDM_tRFC_21cyc		(unsigned int) 0x00013000
#define	SDM_tRFC_22cyc		(unsigned int) 0x00014000
#define	SDM_tRFC_23cyc		(unsigned int) 0x00015000
#define	SDM_tRFC_24cyc		(unsigned int) 0x00016000
#define	SDM_tRFC_25cyc		(unsigned int) 0x00017000
#define	SDM_tRFC_26cyc		(unsigned int) 0x00018000
#define	SDM_tRFC_27cyc		(unsigned int) 0x00019000
#define	SDM_tRFC_28cyc		(unsigned int) 0x0001A000
#define	SDM_tRFC_29cyc		(unsigned int) 0x0001B000
#define	SDM_tRFC_30cyc		(unsigned int) 0x0001C000
#define	SDM_tRFC_31cyc		(unsigned int) 0x0001D000
#define	SDM_tRFC_32cyc		(unsigned int) 0x0001E000
#define	SDM_tRFC_33cyc		(unsigned int) 0x0001F000
#define	SDM_tXSR_2cyc		(unsigned int) 0x00000000
#define	SDM_tXSR_3cyc		(unsigned int) 0x00100000
#define	SDM_tXSR_4cyc		(unsigned int) 0x00200000
#define	SDM_tXSR_5cyc		(unsigned int) 0x00300000
#define	SDM_tXSR_6cyc		(unsigned int) 0x00400000
#define	SDM_tXSR_7cyc		(unsigned int) 0x00500000
#define	SDM_tXSR_8cyc		(unsigned int) 0x00600000
#define	SDM_tXSR_9cyc		(unsigned int) 0x00700000
#define	SDM_tXSR_10cyc		(unsigned int) 0x00800000
#define	SDM_tXSR_11cyc		(unsigned int) 0x00900000
#define	SDM_tXSR_12cyc		(unsigned int) 0x00A00000
#define	SDM_tXSR_13cyc		(unsigned int) 0x00B00000
#define	SDM_tXSR_14cyc		(unsigned int) 0x00C00000
#define	SDM_tXSR_15cyc		(unsigned int) 0x00D00000
#define	SDM_tXSR_16cyc		(unsigned int) 0x00E00000
#define	SDM_tXSR_17cyc		(unsigned int) 0x00F00000
#define	SDM_tXSR_18cyc		(unsigned int) 0x01000000
#define	SDM_tXSR_19cyc		(unsigned int) 0x01100000
#define	SDM_tXSR_20cyc		(unsigned int) 0x01200000
#define	SDM_tXSR_21cyc		(unsigned int) 0x01300000
#define	SDM_tXSR_22cyc		(unsigned int) 0x01400000
#define	SDM_tXSR_23cyc		(unsigned int) 0x01500000
#define	SDM_tXSR_24cyc		(unsigned int) 0x01600000
#define	SDM_tXSR_25cyc		(unsigned int) 0x01700000
#define	SDM_tXSR_26cyc		(unsigned int) 0x01800000
#define	SDM_tXSR_27cyc		(unsigned int) 0x01900000
#define	SDM_tXSR_28cyc		(unsigned int) 0x01A00000
#define	SDM_tXSR_29cyc		(unsigned int) 0x01B00000
#define	SDM_tXSR_30cyc		(unsigned int) 0x01C00000
#define	SDM_tXSR_31cyc		(unsigned int) 0x01D00000
#define	SDM_tXSR_32cyc		(unsigned int) 0x01E00000
#define	SDM_tXSR_33cyc		(unsigned int) 0x01F00000
#define	SDM_tXP_1cyc		(unsigned int) 0x00000000
#define	SDM_tXP_2cyc		(unsigned int) 0x10000000
#define	SDM_tXP_3cyc		(unsigned int) 0x20000000
#define	SDM_tXP_4cyc		(unsigned int) 0x30000000
#define	SDM_tWPRES_0cyc		(unsigned int) 0x00000000
#define	SDM_tWPRES_1cyc		(unsigned int) 0x80000000

/* PLL parameters */
/************************************************/
// PLL physical meaning
/************************************************/
//---
#define A9_CKSW0_FR_26M   (unsigned int) 0x00000000//1228
#define A9_CKSW0_FR_32K   (unsigned int) 0x00000001//1228
#define A9_CKSW1_FR_A9PLL (unsigned int) 0x00000000//1228
#define A9_CKSW1_FR_PKPLL (unsigned int) 0x00000002//1228
#define A9_CKSW2_FR_26_32 (unsigned int) 0x00000000//1228
#define A9_CKSW2_FR_SW1   (unsigned int) 0x00000004//1228

#define A7_CKSW1_FR_A7PLL (unsigned int) 0x00000000//1228
#define A7_CKSW1_FR_DPLL  (unsigned int) 0x00000001//1228
#define A7_CKSW2_FR_26M   (unsigned int) 0x00000000//1228
#define A7_CKSW2_FR_SW1   (unsigned int) 0x00000002//1228

#define CKSW3_FR_D4      (unsigned int) 0x00000008
#define CKSW9_FR_SW8     (unsigned int) 0x00000200
#define A9_CKSW0_CLR     (unsigned int) 0x00000001//1228
#define A9_CKSW1_CLR     (unsigned int) 0x00000002//1228
#define A9_CKSW2_CLR     (unsigned int) 0x00000004//1228
#define CKSW3_CLR        (unsigned int) 0x00000008
#define CKSW7_CLR        (unsigned int) 0x00000080
#define CKSW8_CLR        (unsigned int) 0x00000100
#define CKSW9_CLR        (unsigned int) 0x00000200
#define CKSW1_FR_SDM     (unsigned int) 0x00000000//1228
#define CKSW1_FR_PARK    (unsigned int) 0x00000001//1228
#define CKSW2_FR_26M     (unsigned int) 0x00000000//1228
#define CKSW2_FR_SW1     (unsigned int) 0x00000002//1228
#define HCLK_DIV_1       (unsigned int) 0x01000000//1228
#define HCLK_DIV_2       (unsigned int) 0x02000000//1228
#define HCLK_DIV_CLR     (unsigned int) 0x0f000000//1228
#define HCLK_DIV_UPDATE  (unsigned int) 0x00008000//1228
#define CKSW1_CLR        (unsigned int) 0x00000001//1228
#define CKSW2_CLR        (unsigned int) 0x00000002//1228
#define D5_CLR           (unsigned int) 0xff000000
#define D5_DIV_13        (unsigned int) 0x0d000000
#define D5_DIV_EN        (unsigned int) 0x80000000
#define D5_DIV_EN_CLR    (unsigned int) 0x80000000
#define D4_CLR           (unsigned int) 0x00ff0000
#define D4_DIV_2         (unsigned int) 0x00020000
#define D4_DIV_EN        (unsigned int) 0x00800000
#define D4_DIV_EN_CLR    (unsigned int) 0x00800000
#define SDMCK_SUSP       (unsigned int) 0x00000010//1228
#define SDMCK_SUSP_CLR   (unsigned int) 0x00000010//1228
#define SDMCK_RUN        (unsigned int) 0x00000000//1228
#define CLK_DIV_UPDATE   (unsigned int) 0x00008000
#define CLK_DIV_EN       (unsigned int) 0x00004000
#define USB_PLL_FSEL     (unsigned int) 0x00100000
#define USBDY_CLK_RUN    (unsigned int) 0x00000001
#define USBDY_CLK_READY  (unsigned int) 0x00000010
#define USB_DIV26        (unsigned int) 0x00010000
#define USBHY_CLK_RUN    (unsigned int) 0x00000010

//A9,  416MHz = DIVR1 + DIVF32  + DIVQ1,  means 26MHz/1 *32 /2^1
//A9,  273MHz = DIVR5 + DIVF168 + DIVQ2,  means 32.5MHz/5 *168 /2^2
//A9,  218MHz = DIVR5 + DIVF168 + DIVQ2,  means 26MHz/5 *168 /2^2
//A9,  210MHz = DIVR5 + DIVF168 + DIVQ2,  means 25MHz/5 *168 /2^2
//----- 40 ns *5 /168 *4 = 4.76   4.76*2=9.52
//A9,  208MHz = DIVR1 + DIVF27 + DIVQ2,  means 30.864MHz/1 *27 /2^2
//----- 32.4 ns /27 *4  = 4.8   4.8*2=9.6
//A9,  200MHz = DIVR1 + DIVF27 + DIVQ2,  means 29.762MHz/1 *27 /2^2
//----- 33.6 ns /27 *4  = 4.97   4.97*2=9.94
//A9,  208MHz = DIVR1 + DIVF31 + DIVQ2,  means 26.880MHz/1 *31 /2^2
//----- 37.2 ns /31 *4 = 4.8   4.8*2=9.6
//A9,  201.5MHz = DIVR1 + DIVF31 + DIVQ2,means 26.000MHz/1 *31 /2^2
//----- 38.4 ns /31 *4 = 4.9548   4.9548*2=9.9096
//A9,  175.5MHz = DIVR1 + DIVF27 + DIVQ2,means 26.000MHz/1 *27 /2^2  (175.5MHz => 5.698ns,  87.75MHz => 11.396ns)
//----- 38.4 ns /27 *4 = 5.6888   5.6888*2=11.376
//----- 38.46ns /27 *4 = 5.698    5.698 *2=11.396
//----- 39.2 ns /27 *4 ~ 5.8      5.8   *2=11.6  (25.5MHz, 86.2MHz, 172.4MHz)
//??,  104.0MHz = DIVR1 + DIVF32 + DIVQ3,  means 26MHz /1 *32 /2^8

//USB,  48.0MHz = DIVR1 + DIVF27 + DIVQ3,  means 30.864MHz /1 *27 /2^3 = 104.167MHz  => d5 (/13) = 8.013MHz
//                DIVR1 + DIVF96 + DIVQ3,  means  8.013MHz /1 *96 /2^3 =  96.154MHz  => d4 (/2)  =48.077MHz    "26M(*4)/13(*12)/2"
//                
//USB,  48.0MHz = DIVR1 + DIVF28 + DIVQ3,  means 29.762MHz /1 *28 /2^3 = 104.167MHz  => d5 (/13) = 8.013MHz
//                DIVR1 + DIVF96 + DIVQ3,  means  8.013MHz /1 *96 /2^3 =  96.154MHz  => d4 (/2)  =48.077MHz    "26M(*4)/13(*12)/2"
//                
//USB,  48.0MHz = DIVR1 + DIVF32 + DIVQ3,  means 26.000MHz /1 *32 /2^3 = 104.000MHz  => d5 (/13) = 8.000MHz
//                DIVR1 + DIVF96 + DIVQ3,  means  8.000MHz /1 *96 /2^3 =  96.000MHz  => d4 (/2)  =48.000MHz    "26M(*4)/13(*12)/2"
//                
//DIVR output 5MHz~900MHz 			(binary value + 1) --- bit[7:0]
#define PLL_DIVR1        (unsigned int) 0x00000000
#define PLL_DIVR2        (unsigned int) 0x00000001
#define PLL_DIVR5        (unsigned int) 0x00000004
//DIVF means the multiple value (500M~1000MHz)	(binary value + 1) --- bit[15:8]  =>   1e = 30+1 = 31
#define PLL_DIVF12       (unsigned int) 0x00000b00
#define PLL_DIVF16       (unsigned int) 0x00000f00
#define PLL_DIVF18       (unsigned int) 0x00001100
#define PLL_DIVF19       (unsigned int) 0x00001200
#define PLL_DIVF20       (unsigned int) 0x00001300
#define PLL_DIVF22       (unsigned int) 0x00001500
#define PLL_DIVF24       (unsigned int) 0x00001700
#define PLL_DIVF25       (unsigned int) 0x00001800
#define PLL_DIVF26       (unsigned int) 0x00001900
#define PLL_DIVF27       (unsigned int) 0x00001a00
#define PLL_DIVF28       (unsigned int) 0x00001b00
#define PLL_DIVF29       (unsigned int) 0x00001c00
#define PLL_DIVF30       (unsigned int) 0x00001d00
#define PLL_DIVF31       (unsigned int) 0x00001e00
#define PLL_DIVF32       (unsigned int) 0x00001f00
#define PLL_DIVF33       (unsigned int) 0x00002000
#define PLL_DIVF34       (unsigned int) 0x00002100
#define PLL_DIVF35       (unsigned int) 0x00002200
#define PLL_DIVF36       (unsigned int) 0x00002300
#define PLL_DIVF37       (unsigned int) 0x00002400
#define PLL_DIVF38       (unsigned int) 0x00002500
#define PLL_DIVF39       (unsigned int) 0x00002600
#define PLL_DIVF40       (unsigned int) 0x00002700
#define PLL_DIVF42       (unsigned int) 0x00002900
#define PLL_DIVF61       (unsigned int) 0x00003c00
#define PLL_DIVF96       (unsigned int) 0x00005f00
#define PLL_DIVF168      (unsigned int) 0x0000A700
//DIVQ means the multiple value 		(2^ binary value)  --- bit[18:16]
#define PLL_DIVQ0        (unsigned int) 0x00000000
#define PLL_DIVQ1        (unsigned int) 0x00010000
#define PLL_DIVQ2        (unsigned int) 0x00020000
#define PLL_DIVQ3        (unsigned int) 0x00030000
//RANGE for jitter
#define PLL_RANGE10      (unsigned int) 0x00100000
#define PLL_RANGE42      (unsigned int) 0x00400000

//VCO Band control, 0:300MHz - 600MHz, 1:500MHz - 1000MHz
#define PLL_BS0		(unsigned int) 0x00000000
#define PLL_BS1		(unsigned int) 0x00080000

//Others
#define PLL_FSEN         (unsigned int) 0x00800000
#define PLL_RESET        (unsigned int) 0x01000000
#define PLL_NO_RESET     (unsigned int) 0x00000000
#define PLL_BYPAS        (unsigned int) 0x02000000
#define PLL_NO_BYPAS     (unsigned int) 0x00000000
#define PLL_LOCK_INDCA   (unsigned int) 0x04000000
//#define PLL_NEED_CLR     (unsigned int) 0x00800000

/* PLL arguments */
/* Clock ID */
#define ARM9_ID			0
#define PARK_ID			1
#define XMEM_ID			2
#define USB_ID			3
#define ARM7_ID			4

/* CKSW ID */
#define ARM9_SW			0
#define XMEM_SW			1
#define USBH_SW			2
#define USBD_SW			3
#define EXT0_SW			4
#define EXT1_SW			5
#define EXT2_SW			6
#define EXT3_SW			7
#define EXT4_SW			8
#define EXT5_SW			9
#define EXT6_SW			10
#define ARM7_SW			11
#define DSP_SW			12
#define VDEC_SW			13
#define V3D_SW			14

/* PARK arguments */
#define ARM9_PARK		0x00000002
#define XMEM_PARK		0x00000001

/* ABB LDO */
#define V_SIM_LOW	(unsigned char) 0x00
#define V_SIM1_HIGH	(unsigned char) 0x01
#define V_SIM2_HIGH	(unsigned char) 0x02
#define V_SIM_HIGH	(unsigned char) 0x03
#define V_1_8		(unsigned char) 0x00
#define V_3_0		(unsigned char) 0x01

#define V_SIM1		(unsigned char) 0x00
#define V_SIM2		(unsigned char) 0x01
#define V_CAM_DIG	(unsigned char) 0x02
#define V_CAM_ANA	(unsigned char) 0x03
#define V_WIFI		(unsigned char) 0x04
#define V_BT		(unsigned char) 0x05
#define V_AUX2		(unsigned char) 0x06
#define V_AUX3		(unsigned char) 0x07
#define V_AUX4		(unsigned char) 0x08
#define VIO18_AUX1	(unsigned char) 0x0c
#define VIO18_AUX2	(unsigned char) 0x0d
#define LED_DRIVE	(unsigned char) 0x0e
#define VIBRATOR	(unsigned char) 0x0f

/* GPT */
#define GPT0		(unsigned char) 0x00
#define GPT1		(unsigned char) 0x01
#define GPT32K		(unsigned char) 0x20

/* GPIO Base */
#define GPIO_BASE	0xF8000300

/* GPIO Pin Number */
#define GPIO0		0
#define GPIO1		1
#define GPIO2		2
#define GPIO3		3
#define GPIO4		4
#define GPIO5		5
#define GPIO6		6
#define GPIO7		7
#define GPIO8		8
#define GPIO9		9
#define GPIO10		10
#define GPIO11		11
#define GPIO12		12
#define GPIO13		13
#define GPIO14		14
#define GPIO15		15
#define GPIO16		16
#define GPIO17		17
#define GPIO18		18
#define GPIO19		19
#define GPIO20		20
#define GPIO21		21
#define GPIO22		22
#define GPIO23		23

/* GPIO Output Enable Register */
#define GPIO_INPUT		1
#define GPIO_OUTPUT		0

/* GPIO Output Register */
#define GPIO_HIGH		1
#define GPIO_LOW		0

//--------- GPIO pin mux select --------------------//
#define GPIO_PIN_MUX_PWL	(unsigned int)0x00000001
#define GPIO_PIN_MUX_PWM0	(unsigned int)0x00000002
#define GPIO_PIN_MUX_PWM1	(unsigned int)0x00000004

/* USB Detect GPIO */ /* IM98XX_EVB_V1: GPIO1 */
#define USB_DETECT_GPIO		GPIO1
#define USB_DETECT_GPIO_REG	GPIO1_REG
#define GPIO_BL_EN		GPIO18
#define GPIO_BL_EN_REG		GPIO18_REG
#define SKIP_SD_TRUE		1
#define SKIP_SD_FALSE		0

/* USB Downloader */
#define SHARE_PARAMETER_START_ADDR	(0xFFFF8000-0x30) /* the last 16 bytes (0xFFFF7FF0~0xFFFF8000) => ROM ZI */
#define SD2BOOTLOADER_SIGNATURE		0xCAFEFEED
#define BOOTLOADER2SD_SIGNATURE		0xFEEDCAFE
#define UBOOT2SD_TERM_MODE_ADDR		((volatile unsigned int *) (SHARE_PARAMETER_START_ADDR + 0x0000))
#define UBOOT2SD_RESERVED0		((volatile unsigned int *) (SHARE_PARAMETER_START_ADDR + 0x0004))
#define UBOOT2SD_RESERVED1		((volatile unsigned int *) (SHARE_PARAMETER_START_ADDR + 0x0008))
#define UBOOT2SD_RESERVED2		((volatile unsigned int *) (SHARE_PARAMETER_START_ADDR + 0x000C))
#define UBOOT2SD_SIGNATURE_ADDR		((volatile unsigned int *) (SHARE_PARAMETER_START_ADDR + 0x0010))
#define UBOOT2SD_BOOT_MODE_ADDR		((volatile unsigned int *) (SHARE_PARAMETER_START_ADDR + 0x0014))
#define UBOOT2SD_SKIP_SD_ADDR		((volatile unsigned int *) (SHARE_PARAMETER_START_ADDR + 0x0018))
#define UBOOT2SD_TIME_OUT_ADDR		((volatile unsigned int *) (SHARE_PARAMETER_START_ADDR + 0x001C))

/* Reserved 32 bytes (0xFFFF7FD0 ~ 0xFFFF7FEF) as parameter area for the communication between barebox and SD-loader */
typedef struct bootup_share_parameter_tag {
	unsigned int terminateMode;
	unsigned int reserved0;
	unsigned int reserved1;
	unsigned int reserved2;
	unsigned int signature;
	unsigned int bootupMode;
	unsigned int skipSDloader;
	unsigned int timeoutTime; /* in mili-second */
	unsigned int reserved3;
	unsigned int reserved4;
	unsigned int reserved5;
	unsigned int reserved6;
} BOOTUP_SHARE_PARAMETER_T;

enum
{
	BT_MODE_CHARGER_IN = 1,
	BT_MODE_COMPOMPOSITE_KEY,
	BT_MODE_COMMAND,
	BT_MODE_MAX
};

enum
{
	USER_TERMINATE = 1,
	NORMAL_TERMINATE,
	TERMINATE_MODE_MAX
};
/* terminateMode = USER_TERMINATE -> terminated by user with keypad 
 * terminateMode = NORMAL_TERMINATE -> terminated by timer time up
 */

/* 32K Logic ITR FLAG */
#define KEY_32K_ITR_FLAG	(1 << 1)
#define PWM0_32K_ITR_FLAG	(1 << 2)
#define GPT_32K_ITR_FLAG	(1 << 3)
#define A9_OS_TICK_ITR_FLAG	(1 << 4)

/* Battery/Charging */
#define CHARGER_STA		(1 << 7)
#define BATTERY_MIN_PWR_OFF	3200
#define BATTERY_MIN_SHOW	3350
#define BATTERY_MIN_VOLT	3400
#define BATTERY_MIN_CHR_BOOT	3400
#define BATTERY_MIN_BOOT	3400
#define BATTERY_MAX_VOLT	4200
#define BATTERY_TERMINATION	4250
#define BAT_V42_ADC		0x37B		// (4.2 / 2 / 2.41 * 1023) = 891
#define BAT_V34_ADC		0x2D2		// (3.4 / 2 / 2.41 * 1023) = 722
#define BAT_CHARGE_CUR		0x8		/* 150 mA => 0x2; 200 mA => 0x4; 250 mA => 0x6; 300 mA => 0x8 */
#define SDELAY_USEC_BASE	(ARM9_CPU_SPEED >> 2)	// div4; ARM9 5-pipeline, sdelay loop w/ 2 instructions



#define BAT_I_ADC		0x13F
#define DBB_CHR_OP_OFF		0x0
#define DBB_CHR_OP_ON		0x1
#define ABB_CHR_ON		0x0
#define ABB_CHR_STOP		0x1
#define BAT_V42_DIR		"/env/bin/battery_4200_adc"
#define BAT_V34_DIR		"/env/bin/battery_3400_adc"
#define BAT_I_DIR		"/env/bin/battery_i_adc"

/* SAR-ADC */
#define EXT0_IN			(0)
#define EXT1_IN			(1<<8)
#define	SPKP_IN			(2<<8)
#define SPKN_IN			(3<<8)
#define XP_IN			(4<<8)
#define YP_IN			(5<<8)
#define XN_IN			(6<<8)
#define YN_IN			(7<<8)
#define BAT_VOLT_IN		(8<<8)
#define BAT_CURRENT_IN		(9<<8)
#define VREF_PMIC_IN		(10<<8)
#define AUXADIN11		(11<<8)
#define AUXADIN12		(12<<8)
#define AUXADIN13		(13<<8)
#define ADPIN			(14<<8)

/* PMU */
#define PWR_UP_BB	0x1
#define PWR_ON_CNT	0x20
#define PWR_BB		0x1

/* RTC */
#define LEAP_YEAR(year)		((!(year % 4) && (year % 100)) || !(year % 400))
#define RTC_KEY_RESET		0x0000
#define RTC_UNLOCK0		0x55AA
#define RTC_UNLOCK1		0x44BB
#define RTC_UNLOCK2		0x33CC
#define RTC_UNLOCK3		0x22DD
#define RTC_LOCK		0
#define RTC_UNLOCK		1
#define RTC_READ_ENABLE		1
#define RTC_WRITE_BUSY		(1 << 15)
#define ALARM_ITR_FLAG		1
/* 0x8000: Kernel Warm Reset, 0x4000: Barebox Warm Reset */
#define BAREBOX_WARM		(1 << 14)
#define KERNEL_WARM		(1 << 15)

struct im98xx_rtc_time {
	unsigned int tm_sec;
	unsigned int tm_minhour;
	unsigned int tm_daymonyear;
};

/* Key */
#define KEY_RELEASED_STATE		0xFFFF
#define KEY_STARUS_0_7_RESET_VALUE	0xFF
#define POWER_KEY			0xF7
#define BAT_C_KEY			0xF6
#define BAT_D_KEY			0xF3
#define HARDWARE_SCAN_ENABLE		(1 << 30)
#define HARDWARE_RSCAN			(1 << 29)
#define KEY_RELEASE_INT_ENABLE		(1 << 28)
#define KEY_ITR_FLAG			(1 << 31)

/* Vreference trimming */
#define VREF_TRIM_DIR		"/env/bin/vref_trim"

/* Boot */
#define CALIBRATION_BOOT_DIR	"/env/bin/calibration_boot"
#define DOWNLOAD_BOOT_DIR	"/env/bin/download_boot"
#define PRE_BOOT_DIR		"/env/bin/pre_boot"
#define RECOVERY_BOOT_DIR	"/env/bin/recovery_boot"
#define RTC_BOOT_DIR		"/env/bin/rtc_boot"

enum
{
	POWER_OFF = 0,
	POWER_ON,
	CALIBRATION_BOOT,
	DOWNLOAD_BOOT,
	WARM_BOOT,
	PRE_BOOT,
	RECOVERY_BOOT,
	RTC_BOOT
};

#define TRACE_BAREBOX 0
#if TRACE_BAREBOX
#define BAREBOX_P(x...)		printf(x)
#else
#define BAREBOX_P(x...)		do {} while (0)
#endif

#endif

