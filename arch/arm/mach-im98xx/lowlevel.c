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


#include <mach/omap3-clock.h>
#include <mach/magic.h>
/**
 * @brief OMAP3 Architecture specific Initialization
 *
 * Does early system init of disabling the watchdog, enable
 * memory and configuring the clocks.
 *
 * prcm_init is called only if CONFIG_OMAP3_CLOCK_CONFIG is defined.
 * We depend on link time clean up to remove a_init if no caller exists.
 *
 * @warning Called path is with SRAM stack
 *
 * @return void
 */
void a_init(void)
{
#if defined(CONFIG_IM98XX_BL_PM_CONFIG)
	/* Reset ABB */
	writel(0x1, ABB_RST_CTL_REG);
//	sdelay(520 * 200);
	sdelay(104000);
	writel(0x0, ABB_RST_CTL_REG);
//	sdelay(520 * 200);
	sdelay(104000);
#endif
#if defined(CONFIG_IM98XX_BL_PM_CONFIG)
	im98xx_pm_init();
#endif
#if defined(CONFIG_IM98XX_CLOCK_CONFIG)
	im98xx_pll_init();
#endif
	sdelay(100);
#if defined(CONFIG_IM98XX_SARADC_CONFIG)
/* SAR-ADC initate */
	im98xx_saradc_init();
#endif
}


void __naked barebox_arm_reset_vector(void)
{
        arm_cpu_lowlevel_init();
#ifdef CONFIG_ARCH_HAS_LOWLEVEL_INIT
		arch_init_lowlevel();
#endif
#ifdef CONFIG_MACH_DO_LOWLEVEL_INIT
		board_init_lowlevel();
#endif

	barebox_arm_entry(SDRAM_START, SDRAM_SIZE, NULL);
}
