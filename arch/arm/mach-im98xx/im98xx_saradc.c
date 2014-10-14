/**
 * @file
 */
/*
 * (C) Copyright 2006-2008
 * Texas Instruments, <www.ti.com>
 * Richard Woodruff <r-woodruff2@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/io.h>
#include <malloc.h>
#include <mach/magic.h>
#include <mach/mem_map.h>
#include <mach/omap3-clock.h>
#include <command.h>
#include <errno.h>
#include <mach/syslib.h>
#include <linux/string.h>
#include <fcntl.h>
#include <fs.h>

extern void im98xx_rtc_write(unsigned short key);
extern void im98xx_rtc_write_busy(void);

#if defined(CONFIG_MACH_IM98XXV1)
unsigned int reg_adc_ctl_flag = 1;
unsigned int reg_adc_ctl_value = 0;
unsigned int reg_adc_str_flag = 1;
unsigned int reg_adc_str_value = 0;
#endif

void im98xx_saradc_init(void)
{
	unsigned int i;

#if defined(CONFIG_MACH_IM98XXV1)
	if (readl(ABB_ADC_CTL_REG) != 0x3F07) {
		reg_adc_ctl_flag = 0;
		reg_adc_ctl_value = readl(ABB_ADC_CTL_REG);
		writel(0x3F07, ABB_ADC_CTL_REG);
	}
	if (readl(ABB_ADC_STR_REG) != 0x0) {
		reg_adc_str_flag = 0;
		reg_adc_str_value = readl(ABB_ADC_STR_REG);
		writel(0x0, ABB_ADC_STR_REG);
	}
#endif

/* TSC mode select register configuration :
 * Transparent mode to SAR-ADC */
	writel(0x0, ABB_TSC_MODE_REG);

/* SAR-ADC control register configuration :
 * Power up SAR-ADC,
 * Power up internal reference voltage,
 * Two SAR-CLK wide,
 * To control SAR-ADC reference voltage as 2.41(V),
 * SAR-ADC Ref+/Ref- selection to AVDD/AGND.
 * Reset value : 0x3F07 */
//	writel(0xA700, ABB_ADC_CTL_REG); /* Normal mode, AVDD/AGND */
	writel(0x9704, ABB_ADC_CTL_REG); /* Normal mode, internal reference voltage/AGND, clear data register ready */
//	writel(0x9740, ABB_ADC_CTL_REG); /* Test mode, internal reference voltage/AGND */
	for (i = 0; i < 100; i++) {
		sdelay(SDELAY_USEC_BASE);
	}

/* SAR-ADC raw data capture register :
 * To read data with ready bit [15]. */
//	readl(ABB_ADC_DATA_REG);

/* SAR-ADC clock divider register configuration :
 * ADC clock freq = 1MHz = 26MHz/(2*(divisor+1))
 * => divisor = 12 = 0xC.
 * Reset value : 0x3F */
	writel(0xC, ABB_ADC_CKDIV_REG);
	for (i = 0; i < 100; i++) {
		sdelay(SDELAY_USEC_BASE);
	}

/* SAR-ADC conversion trigger register configuration :
 * Reset value : 0x0 */
	writel(0x0, ABB_ADC_STR_REG);
	for (i = 0; i < 100; i++) {
		sdelay(SDELAY_USEC_BASE);
	}
}
device_initcall(im98xx_saradc_init);

unsigned short im98xx_measure_adc_raw_data(void)
{
	unsigned short i = 0;

	readl(ABB_ADC_DATA_REG);	// clear up prior data capture, if any

	writel((readl(ABB_ADC_STR_REG) | 0x1), ABB_ADC_STR_REG); /* Enable trigger */

	sdelay(SDELAY_USEC_BASE);	// 1 usec

//	printf(" Read ABB_ADC_STR_REG=0x%x \n", readl(ABB_ADC_STR_REG));

	while ((readl(ABB_ADC_CTL_REG) & 0x8000) != 0x8000) {
		i++;
		if (i > 0x800) {
			BAREBOX_P("\n\n\n Error: SAR-ADC raw data capture fail! \n\n\n");
			BAREBOX_P("\n\n\n Please power off and then power on again. \n\n\n");
			hang();
		}
		sdelay(SDELAY_USEC_BASE);	// 1 usec
	}

/* SAR-ADC raw data capture register :
 * To read data with ready bit [15]. */

	return (readl(ABB_ADC_DATA_REG) & 0x3FF);
}

unsigned short im98xx_measure_adc_average(unsigned short u16_sample_cnt, unsigned short u16_filter_cnt)
{
	unsigned short i;
	unsigned short u16_tmp, u16_average;
	unsigned short *u16_data; 
	unsigned int u32_sum;

//	printf("\n {%d, %d} \n", u16_sample_cnt, u16_filter_cnt);

	u16_data = malloc(u16_sample_cnt*2);

	for (i = 0 ; i < (u16_filter_cnt) ; i++) {
		sdelay(SDELAY_USEC_BASE);	// 1 usec
		u16_tmp = im98xx_measure_adc_raw_data();
	}

	for (i = 0 ; i < (u16_sample_cnt) ; i++) {
		sdelay(SDELAY_USEC_BASE);	// 1 usec
//		printf("\n sample_cnt=%d  ", (i+u16_filter_cnt));
		u16_data[i] = im98xx_measure_adc_raw_data();
	}

#if 0	// no purpose to filtering out again
	for (i = 0 ; i < (u16_filter_cnt) ; i++) {
		sdelay(SDELAY_USEC_BASE);	// 1 usec
		u16_tmp = im98xx_measure_adc_raw_data();
	}
#endif

	u32_sum = (u16_sample_cnt / 2); //rounding

	for (i = 0 ; i < (u16_sample_cnt) ; i++) {
//		printf("\n u16_data[%d] = 0x%x \n", i, u16_data[i]);
		u32_sum += u16_data[i];
	}

	u16_average = (unsigned short) (u32_sum / (u16_sample_cnt));

	free(u16_data);

//	printf("\n Average=0x%x. \n", u16_average);

	return u16_average;
}

unsigned short im98xx_calculate_normal_volt(unsigned short u16_chop_ref, unsigned short u16_normal_ref)
{
	unsigned short u16_normal_volt;
	unsigned int calculate_value;

	calculate_value = ((1156 * u16_normal_ref) / (1750 * u16_chop_ref / 1000));
//	printf("\n %d \n", calculate_value); /* mV */
	u16_normal_volt = (unsigned short) (calculate_value);
//	printf("\n %d \n", u16_normal_volt);

	return u16_normal_volt; /* unit => mV */
}

unsigned short im98xx_calculate_resistor_trim(unsigned short u16_target_volt, unsigned short u16_normal_volt)
{
	unsigned short u16_volt_diff;
	unsigned short u16_trim_value;
	unsigned int calculate_value;

	u16_trim_value = 0;

	if (u16_target_volt > u16_normal_volt) {
		u16_volt_diff = u16_target_volt - u16_normal_volt;
		calculate_value = (u16_volt_diff * 1750) / 4286;
		u16_trim_value = (unsigned short) (calculate_value);
		if (u16_trim_value == 0x0) {
			u16_trim_value = 0;
		} else if ( (u16_trim_value <= 0x40) && (u16_trim_value > 0x0) ) {
			u16_trim_value = 0x80 - u16_trim_value;
		} else if (u16_trim_value > 0x40) {
			u16_trim_value = 0x40;
		}
	} else if (u16_target_volt < u16_normal_volt) {
		u16_volt_diff = u16_normal_volt - u16_target_volt;
		calculate_value = (u16_volt_diff * 1750) / 4286;
		u16_trim_value = (unsigned short) (calculate_value);
		if (u16_trim_value == 0x0) {
			u16_trim_value = 0;
		} else if ( (u16_trim_value < 0x40) && (u16_trim_value > 0x0) ) {
			u16_trim_value = 0x0 + u16_trim_value;
		} else if (u16_trim_value >= 0x40) {
			u16_trim_value = 0x3F;
		}
	} else if (u16_target_volt == u16_normal_volt) {
		u16_trim_value = 0;
	}

	return u16_trim_value;
}

void im98xx_pmic_auto_trim(unsigned short u16_sample, unsigned short u16_filter)
{
	unsigned short i;
	unsigned short u16_chop_ref, u16_normal_ref;
	unsigned short u16_normal_volt, u16_target_volt;
	unsigned short u16_trim_value;

/* BandGap select resister configuration :
 * Chop reference,
 * Disable normal reference.
 * Reset value : 0x0 */
	if ( (readl(ABB_BGSEL_REG) & 0x8) != 0x8 ) {
		BAREBOX_P("\n Warning: BandGap isn't the Chop reference!! \n");
		BAREBOX_P("\n BandGap value: 0x%x \n", readl(ABB_BGSEL_REG));
#if 0
		if ( (readl(ABB_BGSEL_REG) & 0x1) == 0x1 ) {
			printf("\n Warning: BandGap has selected the Normal reference!! \n");
		}
		if ( (readl(ABB_BGSEL_REG) & 0x2) == 0x2 ) {
			printf("\n Warning: BandGap has enabled the Normal reference!! \n");
		}
		if ( (readl(ABB_BGSEL_REG) & 0x4) == 0x4 ) {
			printf("\n Warning: BandGap status is the Normal reference!! \n");
		}
#endif
		return;
	}

	writel(0x0, ABB_BGSEL_REG);
	
	for (i = 0; i < 100; i++) {
		sdelay(SDELAY_USEC_BASE);
	}
	BAREBOX_P("\n ABB_BGSEL_REG=0x%x\n", readl(ABB_BGSEL_REG));

/* PMU reference voltage resistor-trim value configuration :
 * Reset value : 0x0 */
	writel(0x0, ABB_PMU_TRIM_REG);
	for (i = 0; i < 100; i++) {
		sdelay(SDELAY_USEC_BASE);
	}

/* SAR-ADC input select : Auxadin13 to chop reference. */
	writel((readl(ABB_ADC_STR_REG) & 0xFF) | AUXADIN13, ABB_ADC_STR_REG);
	for (i = 0; i < 200; i++) {	/* Delay 200us. */
		sdelay(SDELAY_USEC_BASE);
	}

/* Measure chop reference via SAR-ADC :
 * read chop reference voltage via SAR-ADC auxadin13 for at least (20+64+20) times,
 * ignoring first/last 20 samples;
 * take an average. */
/* Normal mode => measure chop reference over 128 samples, program will be trapped. */

	u16_chop_ref = im98xx_measure_adc_average(u16_sample, u16_filter);
	BAREBOX_P("\n chop_ref=%d \n", u16_chop_ref);

/* BandGap select : Enable normal reference. */
	writel(0x2, ABB_BGSEL_REG);
	for (i = 0; i < 100; i++) {
		sdelay(SDELAY_USEC_BASE);
	}

/* SAR-ADC input select : Auxadin11 to normal reference. */
	writel((readl(ABB_ADC_STR_REG) & 0xFF) | AUXADIN11, ABB_ADC_STR_REG);
	for (i = 0; i < 200; i++) {	/* Delay 200us. */
		sdelay(SDELAY_USEC_BASE);
	}

/* Measure normal reference via SAR-ADC :
 * read normal reference voltage via SAR-ADC auxadin11 for at least (20+64+20) times,
 * ignoring first/last 20 samples;
 * take an average. */

	u16_normal_ref = im98xx_measure_adc_average(u16_sample, u16_filter);
	BAREBOX_P("\n normal_ref=%d \n", u16_normal_ref);

/* Calculate normal reference voltage to calibrate. PS: unit => mV */
	u16_normal_volt = im98xx_calculate_normal_volt(u16_chop_ref, u16_normal_ref);
	BAREBOX_P("\n Calculated normal_volt=%d mv \n", u16_normal_volt);

/* Calculate resistor-trim value and write resistor-trim value to resistor-trim register */
	u16_target_volt = 1200; /* 1200 mV */ /* Note : 1220mV */
	u16_trim_value = im98xx_calculate_resistor_trim(u16_target_volt, u16_normal_volt);
	writel((0x7F & u16_trim_value), ABB_PMU_TRIM_REG);
	for (i = 0; i < 100; i++) {
		sdelay(SDELAY_USEC_BASE);
	}
	BAREBOX_P("\n 2504 Vref calibrated trim value = 0x%x \n", (0x7F & u16_trim_value));

/* BandGap select : Enable normal reference and switch to normal reference. */
	writel(0x3, ABB_BGSEL_REG);
	for (i = 0; i < 100; i++) {
		sdelay(SDELAY_USEC_BASE);
	}

/* SAR-ADC input select : Auxadin10 to normal reference. */
	writel((readl(ABB_ADC_STR_REG) & 0xFF) | VREF_PMIC_IN, ABB_ADC_STR_REG);
	for (i = 0; i < 100; i++) {
		sdelay(SDELAY_USEC_BASE);
	}

/* Measure Vref PMIC reference via SAR-ADC :
 * read normal reference voltage via SAR-ADC auxadin10 for at least (20+64+20) times,
 * ignoring first/last 20 samples;
 * take an average. */

	u16_normal_ref = im98xx_measure_adc_average(u16_sample, u16_filter);
	BAREBOX_P("\n Calibrated normal_ref=0x%x \n", u16_normal_ref);

/* Calculate normal reference voltage after calibrate. PS: unit => mV */
	BAREBOX_P("\n Calibrated normal_volt=%d mv \n", u16_normal_ref*2410/1024);
}

void pmic_2504b_auto_trim(unsigned short u16_sample, unsigned short u16_filter)
{
	unsigned short i;
	unsigned short u16_tmp;
	unsigned short u16_trim_value;
	unsigned short u16_PMIC;
	long chop_ref_sum = 0;
	long normal_ref_sum = 0;
	long calculate_value;

	unsigned short *u16_chop_ref   = malloc((u16_sample << 1));
	unsigned short *u16_normal_ref = malloc((u16_sample << 1));

/* BandGap select resister configuration :
 * Chop reference,
 * Disable normal reference.
 * Reset value : 0x0 */

#if 0
	writel(0x0, ABB_BGSEL_REG);
	for (i = 0; i < 2000; i++) {
		sdelay(SDELAY_USEC_BASE);
	}
#endif
	printf("\n ABB_BGSEL_REG=0x%x\n", readl(ABB_BGSEL_REG));

/* PMU reference voltage resistor-trim value configuration :
 * Reset value : 0x0 */
	writel(0x0, ABB_PMU_TRIM_REG);
	for (i = 0; i < 2000; i++) {
		sdelay(SDELAY_USEC_BASE);
	}

/* SAR-ADC input select : Auxadin13 to chop reference. */
	writel((readl(ABB_ADC_STR_REG) & 0xFF) | AUXADIN13, ABB_ADC_STR_REG);
	for (i = 0; i < 2000; i++) {
		sdelay(SDELAY_USEC_BASE);
	}

/* Measure chop reference via SAR-ADC :
 * read chop reference voltage via SAR-ADC auxadin13 for at least (20+64+20) times,
 * ignoring first/last 20 samples;
 * then, take an average. */
/* Normal mode => measure chop reference over 128 samples, program will be trapped. */

	for (i = 0 ; i < (u16_filter) ; i++) {
		u16_tmp = im98xx_measure_adc_raw_data();
	}

	for (i = 0 ; i < (u16_sample) ; i++) {
		u16_chop_ref[i] = im98xx_measure_adc_raw_data();
		chop_ref_sum += u16_chop_ref[i];
	}

	for (i = 0 ; i < (u16_filter) ; i++) {
		u16_tmp = im98xx_measure_adc_raw_data();
	}

/* BandGap select : Enable normal reference. */
	writel(0x2, ABB_BGSEL_REG);
	for (i = 0; i < 2000; i++) {
		sdelay(SDELAY_USEC_BASE);
	}

/* SAR-ADC input select : Auxadin11 to normal reference. */
	writel((readl(ABB_ADC_STR_REG) & 0xFF) | AUXADIN11, ABB_ADC_STR_REG);
	for (i = 0; i < 2000; i++) {
		sdelay(SDELAY_USEC_BASE);
	}

/* Measure normal reference via SAR-ADC :
 * read normal reference voltage via SAR-ADC auxadin11 for at least (20+64+20) times,
 * ignoring first/last 20 samples;
 * then, take an average. */

	for (i = 0 ; i < (u16_filter) ; i++) {
		u16_tmp = im98xx_measure_adc_raw_data();
	}

	for (i = 0 ; i < (u16_sample) ; i++) {
		u16_chop_ref[i] = im98xx_measure_adc_raw_data();
		normal_ref_sum += u16_chop_ref[i];
	}

	for (i = 0 ; i < (u16_filter) ; i++) {
		u16_tmp = im98xx_measure_adc_raw_data();
	}

	printf("\n 2504b chop_ref value = %d \n", chop_ref_sum/u16_sample);
	printf("\n 2504b normal_ref value = %d \n", normal_ref_sum/u16_sample);

/* Calculate resistor-trim value and write resistor-trim value to resistor-trim register */
	calculate_value = (128 + ((((((normal_ref_sum / u16_sample) * 1156) /
				     (chop_ref_sum / u16_sample)) * 1000000 - 2097857143) /
				    4285714)));

	u16_trim_value = (unsigned short)(calculate_value);

//	if (u16_trim_value > ((128) - 1)) {
	if (u16_trim_value > 127) {
		u16_trim_value -= (128);
	} else {
		u16_trim_value = (u16_trim_value - 1);
	}

	writel((0x7F & u16_trim_value), ABB_PMU_TRIM_REG);
	for (i = 0; i < 2000; i++) {
		sdelay(SDELAY_USEC_BASE);
	}
	printf("\n 2504b Vref calibrated trim = %d \n", u16_trim_value);
	printf("\n [IM98CT] Vref_trim = 0x%x \n", u16_trim_value);

/* BandGap select : Enable normal reference and switch to normal reference. */
	writel(0x3, ABB_BGSEL_REG);
	for (i = 0; i < 2000; i++) {
		sdelay(SDELAY_USEC_BASE);
	}

/* SAR-ADC input select : Auxadin10 to normal reference. */
	writel((readl(ABB_ADC_STR_REG) & 0xFF) | VREF_PMIC_IN, ABB_ADC_STR_REG);
	for (i = 0; i < 2000; i++) {
		sdelay(SDELAY_USEC_BASE);
	}

/* Measure Vref PMIC reference via SAR-ADC :
 * read normal reference voltage via SAR-ADC auxadin10 for at least (20+64+20) times,
 * ignoring first/last 20 samples;
 * take an average. */

	u16_PMIC = im98xx_measure_adc_average(u16_sample, u16_filter);
//	printf("\n Calibrated normal_ref=0x%x \n", u16_PMIC);

/* Calculate normal reference voltage after calibrate. PS: unit => mV */
	printf("\n Calibrated normal_volt=%d mv \n", ((u16_PMIC * 2410) >> 10));

	free(u16_chop_ref);
	free(u16_normal_ref);
}

void im98xx_save_auto_trim(void)
{
	int fd;
	char *src = "";
	unsigned int trim_value;

	trim_value = readl(ABB_PMU_TRIM_REG);
	trim_value &= 0x7F;

	fd = open(VREF_TRIM_DIR, O_WRONLY | O_TRUNC | O_CREAT);

	sprintf(src, "0x%x", trim_value);

	write(fd, src, strlen(src));

	close(fd);
}

/*============================================================================*/
static int do_auto_trim(int argc, char *argv[])
{
	unsigned short u16_sample, u16_filter;
	unsigned int reg_val;

#if 0 /* Mask: Not recommend to save Vtrim to NAND Falsh and read Vtrim from NAND Falsh. */
	unsigned short i, u16_PMIC;
	int fd;
	char str[6];
	unsigned short buf_size, trim_value;
#endif
	
	printf("do_auto_trim argc:%d\n",argc);

	if (argc < 1)
		return COMMAND_ERROR_USAGE;

	/* Default sample cnt => 20+64+20. (ignoring first/last 20 samples) */
	u16_sample = 64;
	u16_filter = 20;

	if ((argc == 2) || (argc == 3)) {
		u16_sample = simple_strtoul(argv[1], NULL, 10);
	}
	if (argc == 3) {
		u16_filter = simple_strtoul(argv[2], NULL, 10);
	}

#if 0 /* Mask: Not recommend to save Vtrim to NAND Falsh and read Vtrim from NAND Falsh. */
	fd = open(VREF_TRIM_DIR, O_RDONLY);

	if (fd < 0) {
		if ( (readl(ABB_BGSEL_REG) & 0x8) != 0x8 ) {
			printf("\n Warning: BandGap isn't the Chop reference!! \n");
			printf("\n BandGap value: 0x%x \n", readl(ABB_BGSEL_REG));
		} else {
			/* Vref auto-trim */
//			im98xx_pmic_auto_trim(u16_sample, u16_filter);
			pmic_2504b_auto_trim(u16_sample, u16_filter);

			/* Save to VREF_TRIM_DIR */
			im98xx_save_auto_trim();
		}
	} else {
		buf_size = read(fd, str, 6);
		trim_value = simple_strtoul(str, NULL, 16);
		printf("\n (SAR-ADC) Vreference trimming value: %d \n", trim_value);
		printf("\n [IM98CT] Trim_read = 0x%x \n", trim_value);

		if (trim_value < 0x80) {
			if ( ( (readl(ABB_BGSEL_REG) & 0xF) == 0x7) ||
			    | ( (readl(ABB_BGSEL_REG) & 0xF) == 0x4) ) {
			/* The bit#0 and bit#1 of ABB_BGSEL_REG will be reset after WDT reset. */
				/* write resistor-trim value to resistor-trim register */
				writel(trim_value, ABB_PMU_TRIM_REG);
			} else if ( (readl(ABB_BGSEL_REG) & 0xF) == 0x8) {
				/* PMU reference voltage resistor-trim value configuration : Reset value => 0x0 */
				writel(0x0, ABB_BGSEL_REG);
				/* BandGap select : Enable normal reference. */
				writel(0x2, ABB_BGSEL_REG);
				/* write resistor-trim value to resistor-trim register */
				writel(trim_value, ABB_PMU_TRIM_REG);
				/* BandGap select : Enable normal reference and switch to normal reference. */
				writel(0x3, ABB_BGSEL_REG);
				/* SAR-ADC input select : Auxadin10 to normal reference. */
				writel((readl(ABB_ADC_STR_REG) & 0xFF) | VREF_PMIC_IN, ABB_ADC_STR_REG);
				for (i = 0; i < 2000; i++) {
					sdelay(SDELAY_USEC_BASE);
				}
/* Measure Vref PMIC reference via SAR-ADC :
 * read normal reference voltage via SAR-ADC auxadin10 for at least (20+64+20) times,
 * ignoring first/last 20 samples;
 * take an average. */
				u16_PMIC = im98xx_measure_adc_average(u16_sample, u16_filter);
				/* Calculate normal reference voltage after calibrate. PS: unit => mV */
				printf("\n Calibrated normal_volt=%d mv \n", u16_PMIC*2410/1024);
			}
		} else {
			printf("\n Error format to Vreference trimming value! \n");
		}
	}

	close(fd);
#else
	reg_val = readl(ABB_BGSEL_REG);
	if ((reg_val & 0x10) == 0x10) {
		printf("\n BandGap select recommend: chop reference! \n");
	} else if ((reg_val & 0x10) == 0x0) {
		printf("\n BandGap select recommend: normal reference! \n");

		if ((reg_val & 0x8) != 0x8) {
			printf("\n BandGap value: 0x%x, Vtrim value: 0x%x \n",
					reg_val, readl(ABB_PMU_TRIM_REG));
		} else {
			/* Cold boot to Reset RTC, then auto-trim. */
			/* Clear RTC Alarm and Periodic Alarm */
			im98xx_rtc_write(RTC_UNLOCK);
			writel(0x0, ABB_RTC_CTL0_REG);
			im98xx_rtc_write_busy();
			writel(readl(ABB_RTC_CTL1_REG) | 0x3, ABB_RTC_CTL1_REG);
			im98xx_rtc_write_busy();
			im98xx_rtc_write(RTC_LOCK);

			/* Vref auto-trim */
			pmic_2504b_auto_trim(u16_sample, u16_filter);

			/* Save to VREF_TRIM_DIR */
			im98xx_save_auto_trim();
		}
	}
#endif

	return 0;
}

static const __maybe_unused char cmd_auto_trim_help[] =
"Usage: auto_trim(2504b to support) \n";

BAREBOX_CMD_START(auto_trim)
	.cmd		= do_auto_trim,
	BAREBOX_CMD_HELP(cmd_auto_trim_help)
BAREBOX_CMD_END


/*============================================================================*/
static int do_reg_read(int argc, char *argv[])
{
	unsigned int u32_address, u32_value;

	if (argc < 2)
		return COMMAND_ERROR_USAGE;

	u32_address = simple_strtoul(argv[1], NULL, 16);

	u32_value = readl(((volatile unsigned int *) (u32_address)));
	printf("\n Read data from address[0x%x] = 0x%x. \n", u32_address, u32_value);

	return 0;
}

static const __maybe_unused char cmd_reg_read_help[] =
"Usage: reg_read address(hex) \n";

BAREBOX_CMD_START(reg_read)
	.cmd		= do_reg_read,
	BAREBOX_CMD_HELP(cmd_reg_read_help)
BAREBOX_CMD_END


/*============================================================================*/
static int do_reg_write(int argc, char *argv[])
{
	unsigned int u32_address, u32_value, u32_return_value;

	if (argc < 3)
		return COMMAND_ERROR_USAGE;

	u32_address = simple_strtoul(argv[1], NULL, 16);
	u32_value = simple_strtoul(argv[2], NULL, 16);

	writel(u32_value, ((volatile unsigned int *) (u32_address)));
	BAREBOX_P("\n Write data to address[0x%x] = 0x%x. \n", u32_address, u32_value);
	u32_return_value = readl(((volatile unsigned int *) (u32_address)));
	BAREBOX_P("\n Read data from address[0x%x] = 0x%x. \n", u32_address, u32_return_value);

	return 0;
}

static const __maybe_unused char cmd_reg_write_help[] =
"Usage: reg_write address(hex) value(hex) \n";

BAREBOX_CMD_START(reg_write)
	.cmd		= do_reg_write,
	BAREBOX_CMD_HELP(cmd_reg_write_help)
BAREBOX_CMD_END


/*============================================================================*/
static int do_reg_set(int argc, char *argv[])
{
	unsigned int u32_address, u32_value, u32_return_value;

	if (argc < 3)
		return COMMAND_ERROR_USAGE;

	u32_address = simple_strtoul(argv[1], NULL, 16);
	u32_value = simple_strtoul(argv[2], NULL, 16);

	u32_return_value = u32_value;
	u32_value |= readl(((volatile unsigned int *) (u32_address)));

	writel(u32_value, ((volatile unsigned int *) (u32_address)));
	BAREBOX_P("\n Set data 0x%x to address[0x%x] = 0x%x. \n", u32_return_value, u32_address, u32_value);
	u32_return_value = readl(((volatile unsigned int *) (u32_address)));
	BAREBOX_P("\n Read data from address[0x%x] = 0x%x. \n", u32_address, u32_return_value);

	return 0;
}

static const __maybe_unused char cmd_reg_set_help[] =
"Usage: reg_set address(hex) value(hex) \n";

BAREBOX_CMD_START(reg_set)
	.cmd		= do_reg_set,
	BAREBOX_CMD_HELP(cmd_reg_set_help)
BAREBOX_CMD_END


/*============================================================================*/
static int do_reg_clear(int argc, char *argv[])
{
	unsigned int u32_address, u32_value, u32_return_value;

	if (argc < 3)
		return COMMAND_ERROR_USAGE;

	u32_address = simple_strtoul(argv[1], NULL, 16);
	u32_value = simple_strtoul(argv[2], NULL, 16);

	u32_return_value = u32_value;
	u32_value = readl(((volatile unsigned int *) (u32_address))) & (~u32_value);

	writel(u32_value, ((volatile unsigned int *) (u32_address)));
	BAREBOX_P("\n Clear data 0x%x to address[0x%x] = 0x%x. \n", u32_return_value, u32_address, u32_value);
	u32_return_value = readl(((volatile unsigned int *) (u32_address)));
	BAREBOX_P("\n Read data from address[0x%x] = 0x%x. \n", u32_address, u32_return_value);

	return 0;
}

static const __maybe_unused char cmd_reg_clear_help[] =
"Usage: reg_clear address(hex) value(hex) \n";

BAREBOX_CMD_START(reg_clear)
	.cmd		= do_reg_clear,
	BAREBOX_CMD_HELP(cmd_reg_clear_help)
BAREBOX_CMD_END

