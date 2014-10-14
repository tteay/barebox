#include <common.h>
#include <sizes.h>
#include <asm/barebox-arm.h>
#include <asm/barebox-arm-head.h>
#include <mach/mem_map.h>

#include <asm/io.h>
#include <mach/syslib.h>

/*#include <common.h>
#include <init.h>
*/

#include <mach/lowlevel.h>
#include <mach/magic.h>

#define GPTIMER_1_INT_FLAG 1<<1

void im98xx_udelay(u32 waitTimeUs)
{	 
	 unsigned int u32_counter;

	if ((waitTimeUs * 26) > 0xFFFFFFFE) {
		u32_counter = 0xFFFFFFFE;
//		printf("\n Warning!! Out of Max. GP Timer counter!! \n");
	} else {
		u32_counter = (waitTimeUs * 26) - 1;
//		printf("\n GP Timer : %d us!! \n", u32_microsecond);
	}
#if 1 //use timer gpt0 
	 /* GPT0 free run */
	 writel(readl(A9GPT_GCLK_REG) & ~(0x1 << 0), A9GPT_GCLK_REG);
	 /* Disable GPT0 ITR */
	 writel(readl(A9GPT_CTL_STA_REG) & ~(0x1 << 4), A9GPT_CTL_STA_REG);
	 /* GPT0 ITR mode : default as one-shot to halt mode */
	 writel(readl(A9GPT_CTL_STA_REG) & ~(0x1 << 8), A9GPT_CTL_STA_REG);
	 /* Clear GPT0 Flag */
	 writel(readl(A9GPT_CTL_STA_REG) | (0x1 << 0), A9GPT_CTL_STA_REG);
	 /* GPT0 counter value */
	 writel(u32_counter, A9GPT_T0_CNT_REG);
#else //use timer gpt1
	/* GPT1 free run */
	writel(readl(A9GPT_GCLK_REG) & ~(0x1 << 1), A9GPT_GCLK_REG);
	/* Disable GPT1 ITR */
	writel(readl(A9GPT_CTL_STA_REG) & ~(0x1 << 5), A9GPT_CTL_STA_REG);
	/* GPT1 ITR mode : default as one-shot to halt mode */
	writel(readl(A9GPT_CTL_STA_REG) & ~(0x1 << 9), A9GPT_CTL_STA_REG);
	/* Clear GPT0 Flag */
	writel(readl(A9GPT_CTL_STA_REG) | (0x1 << 1), A9GPT_CTL_STA_REG);
	/* GPT1 counter value */
	writel(u32_counter, A9GPT_T1_CNT_REG);
#endif
 }

 static void pmic_pm_init(void)
{
#if defined(CONFIG_MACH_IM98XXV1)
	if (readl(ABB_PMU_LDOVS4_REG) != 0x2D9A) {
		reg_pmu_ldovs4_flag = 0;
		reg_pmu_ldovs4_value = readl(ABB_PMU_LDOVS4_REG);
		writel(0x2D9A, ABB_PMU_LDOVS4_REG);
	}
#endif

	/* Setup VIO18 DC/DC */
	writel((readl(ABB_PMU_LDOVS4_REG) & ~(0x3 << 4)) | (VIO18_VAL << 4), ABB_PMU_LDOVS4_REG);

	/* Setup VSYS DC/DC */
	writel((readl(ABB_PMU_LDOVS4_REG) & ~(0xF)) | VSYS_VAL, ABB_PMU_LDOVS4_REG);

#if defined(CONFIG_MACH_IM98XXV1)
	if (readl(ABB_PMU_A9PSC_REG) != 0xAA) {
		reg_pmu_a9psc_flag = 0;
		reg_pmu_a9psc_value = readl(ABB_PMU_A9PSC_REG);
		writel(0xAA, ABB_PMU_A9PSC_REG);
	}
#endif

	/* Setup VARM9 normal mode */
	writel((readl(ABB_PMU_A9PSC_REG) & ~(0xF)) | VARM9_NORMAL_VAL, ABB_PMU_A9PSC_REG);

	/* Setup VARM9 sleep mode */
	writel((readl(ABB_PMU_A9PSC_REG) & ~(0xF << 4)) | (VARM9_SLEEP_VAL << 4), ABB_PMU_A9PSC_REG);
}

void im98xx_pm_init(void)
{
/* ARM9 */
	pmic_pm_init();

	/* Disable interrupt */
	writel(0, A9ITR_IRQ_REG);
	writel(0, A9ITR_FIQ_REG);

/* ARM7 */
	/* switch modem power supply to communication mode */
#if defined(CONFIG_MACH_IM98XXV1)
	if (readl(ABB_PMU_MODPSC_REG) != 0x8) {
		reg_pmu_modpsc_flag = 0;
		reg_pmu_modpsc_value = readl(ABB_PMU_MODPSC_REG);
		writel(0x8, ABB_PMU_MODPSC_REG);
	}
#endif
	writel(readl(ABB_PMU_MODPSC_REG) | 0x1, ABB_PMU_MODPSC_REG);

	/* BaseBand Power Control : Disable BaseBand Power-Down */
	writel(readl(BB_PWR_CTL_REG) & ~(0x00000031), BB_PWR_CTL_REG);
}

void arch_init_lowlevel(void)
{
#if defined(CONFIG_IM98XX_BL_PM_CONFIG)
	/* Reset ABB */
	writel(0x1, ABB_RST_CTL_REG);
//	sdelay(520 * 200);
	im98xx_udelay(10);
	writel(0x0, ABB_RST_CTL_REG);
//	sdelay(520 * 200);
	im98xx_udelay(10);
#endif
#if defined(CONFIG_IM98XX_BL_PM_CONFIG)
	im98xx_pm_init();
#endif
#if defined(CONFIG_IM98XX_CLOCK_CONFIG)
	im98xx_pll_init();
#endif
	im98xx_udelay(10);
#if defined(CONFIG_IM98XX_SARADC_CONFIG)
/* SAR-ADC initate */
//	im98xx_saradc_init();//this will call in device_init or other place
#endif
}


void __naked barebox_arm_reset_vector(void)
{
    arm_cpu_lowlevel_init();
	arm_setup_stack(0x1FFFFFFC);//temply set stack at end of sram
	arch_init_lowlevel();
	board_init_lowlevel();//sdram init
	barebox_arm_entry(SDRAM_START, SDRAM_SIZE, NULL);
}
