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
#include <rtc.h>
#include <linux/string.h>
#include <fcntl.h>
#include <fs.h>

#include <mach/hippo_showbat.h>

#define FALSE	0
#define TRUE	1

#if defined(CONFIG_IM98XX_GPIO18_CTL_BACKLIGHT_CONFIG)
#define GPIO_TOGGLE_BACKLIGHT	1
#elif defined(CONFIG_IM98XX_PWL_CTL_BACKLIGHT_CONFIG)
#define GPIO_TOGGLE_BACKLIGHT	0
#endif

/* Please check the Battery Logo size/type to synchronize with Kernel Battery Logo.
 * And download the same binary file as the Battery Logo to show.
 *
 * CFG_SHOW_DEFAULT_BATTERY => Default: vertical logo. File name: BSP12_LCM_ICON.bin
 * CFG_SHOW_APPLE_BATTERY => Apple style: horizontal logo. File name: BSP12_APPLE_LCM_ICON.bin
 */
//#define CFG_SHOW_DEFAULT_BATTERY
#define CFG_SHOW_APPLE_BATTERY

bool Show_ChargingIn = TRUE;

typedef struct
{
	unsigned char start_F;
	unsigned char lock_F;
	unsigned char eoc_F;
	unsigned char error_F;
	unsigned char present_F;
} barebox_preboot_T;

typedef struct
{
	unsigned short volt_adc;
	unsigned short current_adc;
	unsigned short battery_volt;
	unsigned short bat_v42_adc;
	unsigned short bat_v34_adc;
	unsigned short bat_i_adc;
	unsigned int start_tm;
} battery_info_T;

barebox_preboot_T charging_F;
battery_info_T charging_info;

unsigned int IM98XXPowerOnType = POWER_OFF;

static unsigned long epoch = 1900;	/* year corresponding to 0x00	*/

extern unsigned short im98xx_measure_adc_average(unsigned short u16_sample_cnt, unsigned short u16_filter_cnt);
extern void pmic_2504b_auto_trim(unsigned short u16_sample, unsigned short u16_filter);
extern void im98xx_save_auto_trim(void);

unsigned int gpio_line_get_input(unsigned int line)
{
	return (readl(GPIO_IN_REG) & (1 << line));
}

void gpio_line_config(unsigned int line, unsigned int direction)
{
	unsigned int val = readl(GPIO_OEN_REG);

	if (direction == GPIO_INPUT) {
		val |= (1 << line);
	} else if (direction == GPIO_OUTPUT) {
		val &= ~(1 << line);
	} else {
		return;
	}

	writel(val, GPIO_OEN_REG);
}

void gpio_line_set_output(unsigned int line, unsigned int value)
{
	unsigned int val = readl(GPIO_OUT_REG);

	if (value == GPIO_LOW) {
		val &= ~(1 << line);
	} else if (value == GPIO_HIGH) {
		val |= (1 << line);
	} else {
		return;
	}

	writel(val, GPIO_OUT_REG);
}

void turn_backlight(bool on)
{
#if GPIO_TOGGLE_BACKLIGHT
	if (on) {
		gpio_line_set_output(GPIO_BL_EN, GPIO_HIGH);
	} else {
		gpio_line_set_output(GPIO_BL_EN, GPIO_LOW);
	}
	im98xx_sleep2itr(10000);
#else
	int value = 0x1F0;

	if (on) {
		while (value > 0x180) {
			writel(value, GPIO_PWL_REG);
			value -= 0x8;
			sdelay(100);
		}
	} else {
		writel(0x1ff, GPIO_PWL_REG);
	}
#endif
}

void im98xx_write_signature(unsigned int bt_boot_mode)
{
	/* Write a special pattern to tell SDLoader */
	writel(BOOTLOADER2SD_SIGNATURE, UBOOT2SD_SIGNATURE_ADDR);
	writel(bt_boot_mode, UBOOT2SD_BOOT_MODE_ADDR);
	writel(20000, UBOOT2SD_TIME_OUT_ADDR);	/* 20 * 1000ms -> 20s */
}

void im98xx_reset_signature(void)
{
	/* Reset signature of D-TCM */
	writel(0x0, UBOOT2SD_SIGNATURE_ADDR);
	writel(0x0, UBOOT2SD_BOOT_MODE_ADDR);
	writel(0x0, UBOOT2SD_SKIP_SD_ADDR);
	writel(0x0, UBOOT2SD_TIME_OUT_ADDR);
}

void im98xx_download_detection(void)
{
	unsigned short charger_value, charger_status, ovp_flag, ovp_status;
	bool delayF = FALSE;

	__asm__ __volatile__ (
		"ldr     r1,     =0xFFFF4015\n"
		"mcr     p15,0,r1,c9,c1,0\n"	/* write to data TCM region register */
		"ldr     r1,     =0xFFFF0015\n"
		"mcr     p15,0,r1,c9,c1,1\n"	/* write to inst TCM region register, last "1": as ITCM */
	);

	gpio_line_config(USB_DETECT_GPIO, GPIO_INPUT);

	/* Battery/Charging status : {Charger on-going, over 3600mV} to auto-download */
	/* Skip OVP mechanism to enter download mode: case(1) HW OVP, case(2) OVP Level trigger */
	charger_value = readl(ABB_BAT_CHRSTA_REG);
	charger_status = charger_value & CHARGER_STA;
	ovp_flag = charger_value & (1 << 8);
	ovp_status = charger_value & (1 << 9);
	/* Case 2: delay to read again */
	if ((charger_status != CHARGER_STA) && (!delayF)) {
		im98xx_26MHz_halt(2000, GPT1);
		delayF = TRUE;
		printf("Charger=>{%d,%d,%d}", charger_status, ovp_flag, ovp_status);
		charger_value = readl(ABB_BAT_CHRSTA_REG);
		charger_status = charger_value & CHARGER_STA;
		ovp_flag = charger_value & (1<<8);
		ovp_status = charger_value & (1<<9);
		printf("2nd Charger=>{%d,%d,%d}", charger_status, ovp_flag, ovp_status);
	}
#if 0
	im98xx_rtc_write(RTC_UNLOCK);
	printf("RTC=> CTL0:0x%x, CTL1:0x%x\n", readl(ABB_RTC_CTL0_REG), readl(ABB_RTC_CTL1_REG));
	im98xx_rtc_write(RTC_LOCK);
#endif

	/* Case 1: Skip OVP mechanism to enter download mode */
	if (!((charger_status != CHARGER_STA) && (charging_info.battery_volt <= 3600))) {
		/* Setup Charger constant current to 150mA. Because, V2 min power comsumption is 110mA. */
		writel((readl(ABB_BAT_CHRCFG_REG) & ~0x3F) | BAT_CHARGE_CUR, ABB_BAT_CHRCFG_REG);

		/* Download detection */
		BAREBOX_P("\n Charging => Go to auto-dwnload detection \n");

		if (gpio_line_get_input(USB_DETECT_GPIO) == (GPIO_HIGH << USB_DETECT_GPIO)) {	/* USB connect GPIO detection */
			BAREBOX_P("\n => Detected USB plug in/out GPIO \n");

			/* SD available pattern */
			if (readl(UBOOT2SD_SIGNATURE_ADDR) == SD2BOOTLOADER_SIGNATURE) {
				if (readl(UBOOT2SD_SKIP_SD_ADDR) == SKIP_SD_FALSE) {
					BAREBOX_P("\n USB_DETECT_GPIO = 0x%x \n", gpio_line_get_input(USB_DETECT_GPIO));

					/* Write a special pattern to tell SDLoader */
					im98xx_write_signature(BT_MODE_CHARGER_IN);

					IM98XXPowerOnType = DOWNLOAD_BOOT;
				} else {
					/* To skip SD-loader */ /* Reset signature of D-TCM */
					im98xx_reset_signature();
				}
			} else {
				BAREBOX_P("\n Cold boot USB_DETECT_GPIO = 0x%x \n", gpio_line_get_input(USB_DETECT_GPIO));

				/* Write a special pattern to tell SDLoader */
				im98xx_write_signature(BT_MODE_CHARGER_IN);

				IM98XXPowerOnType = DOWNLOAD_BOOT;
			}
		} else {
			/* Reset signature of D-TCM */
			im98xx_reset_signature();
		}

		if (IM98XXPowerOnType == DOWNLOAD_BOOT) {
			printf("\n Auto-download detection. \n");
			writel(readl(ABB_SW_PWRBB_REG) | PWR_BB, ABB_SW_PWRBB_REG);
		}
	}
}

void im98xx_rtc_write(unsigned short key)
{
	if (key == RTC_LOCK) {
		writel(RTC_KEY_RESET, ABB_RTC_KEY_REG);
	} else if (key == RTC_UNLOCK) {
		writel(RTC_KEY_RESET, ABB_RTC_KEY_REG);
		writel(RTC_UNLOCK0, ABB_RTC_KEY_REG);
		writel(RTC_UNLOCK1, ABB_RTC_KEY_REG);
		writel(RTC_UNLOCK2, ABB_RTC_KEY_REG);
		writel(RTC_UNLOCK3, ABB_RTC_KEY_REG);
	}
}

void im98xx_rtc_read_enable(void)
{
	im98xx_rtc_write(RTC_UNLOCK);
	writel(RTC_READ_ENABLE, ABB_RTC_TEN_REG);
	im98xx_rtc_write(RTC_LOCK);
}

void im98xx_rtc_write_busy(void)
{
	unsigned short busy;

	do {
		busy = readl(ABB_RTC_CTL1_REG);
	} while((busy & RTC_WRITE_BUSY) == RTC_WRITE_BUSY);
}

void im98xx_read_back_chr_flag(void)
{
	unsigned short sw0_value, start_status;

	start_status = charging_F.start_F;

	sw0_value = readl(ABB_RTC_SW0_REG);
//	sw0_value &= 0xFE00;

	if ((sw0_value & KERNEL_WARM) == KERNEL_WARM) {
	/* 1<<15: Kernel Warm Reset, 1<<14: Barebox Warm Reset */
		charging_F.start_F	= ((sw0_value >>  9) & 1);
		charging_F.lock_F	= ((sw0_value >> 10) & 1);
		charging_F.eoc_F	= ((sw0_value >> 11) & 1);
		charging_F.error_F	= ((sw0_value >> 12) & 1);
		charging_F.present_F	= ((sw0_value >> 13) & 1);

#if 0
		if (charging_F.start_F != start_status) {
			/* Restart charging status */
//			charging_F.start_F = start_status;
		}
#endif
	}
}

void im98xx_write_back_chr_flag(void)
{
	unsigned short sw0_value;

	sw0_value = readl(ABB_RTC_SW0_REG);
	sw0_value &= (~0xFE00);

	sw0_value |= (((charging_F.start_F	& 1) <<  9) |
		      ((charging_F.lock_F	& 1) << 10) |
		      ((charging_F.eoc_F	& 1) << 11) |
		      ((charging_F.error_F	& 1) << 12) |
		      ((charging_F.present_F	& 1) << 13));

	/* if Kernel Warm reset: 1<<15, else if Barebox Warm reset and booting: 1<<14 */
	sw0_value |= BAREBOX_WARM;

	im98xx_rtc_write(RTC_UNLOCK);

	do {
		writel(sw0_value, ABB_RTC_SW0_REG);
	} while (readl(ABB_RTC_SW0_REG) != sw0_value);

	im98xx_rtc_write(RTC_LOCK);
}

static const unsigned int sumofdays_in_mo[] = {
	0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334, 365
};

/*
unsigned int im98xx_update_seconds_tm(struct rtc_time *tm, unsigned int old_date_tm)
{
	unsigned int resupply_sec, yrs;

	yrs = (tm->tm_year + 1900 - 1);
	resupply_sec = (365*24*60*60 + ((!(yrs % 4) && (yrs % 100)) || !(yrs % 400))*24*60*60) - old_date_tm;

	return resupply_sec;
}
*/

static const unsigned char rtc_days_in_month[] = {
	31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31
};

static int im98xx_rtc_month_days(unsigned int month, unsigned int year)
{
	return rtc_days_in_month[month] + (LEAP_YEAR(year) && month == 1);
}

static char im98xx_rtc_valid_tm(struct rtc_time *tm)
{
	if ((tm->tm_year < 70) ||
	    (tm->tm_year > 137) ||
	    (((unsigned)tm->tm_mon) >= 12) ||
	    (tm->tm_mday < 1) ||
	    (tm->tm_mday > im98xx_rtc_month_days(tm->tm_mon, tm->tm_year + 1900)) ||
	    (((unsigned)tm->tm_hour) >= 24) ||
	    (((unsigned)tm->tm_min) >= 60) ||
	    (((unsigned)tm->tm_sec) >= 60)) {
		return -1;
	}

	return 0;
}

void im98xx_rtc_default_init(struct rtc_time *tm)
{
	tm->tm_year	= 112;
	tm->tm_mon	= 1;
	tm->tm_mday	= 1;
	tm->tm_hour	= 12;
	tm->tm_min	= 00;
	tm->tm_sec	= 00;
}

void im98xx_rtc_encodetime(struct im98xx_rtc_time *evb_time, struct rtc_time *tm)
{
	evb_time->tm_daymonyear	= ((((tm->tm_year) + 1900 - 2000) << 9) |
				   ((tm->tm_mon - 1) << 5) | (tm->tm_mday - 1));
	evb_time->tm_minhour	= ((tm->tm_hour << 8) | tm->tm_min);
	evb_time->tm_sec	= tm->tm_sec;
}

void im98xx_rtc_decodetime(struct im98xx_rtc_time *evb_time, struct rtc_time *tm)
{
	tm->tm_year = (((evb_time->tm_daymonyear & 0x7E00) >> 9) + 2000 - 1900);
	tm->tm_mon  = (((evb_time->tm_daymonyear & 0x1E0) >> 5) + 1);
	tm->tm_mday = ((evb_time->tm_daymonyear & 0x1F) + 1);
	tm->tm_hour = ((evb_time->tm_minhour & 0x1F00) >> 8);
	tm->tm_min  = (evb_time->tm_minhour & 0x3F);
	tm->tm_sec  = evb_time->tm_sec;
}

void im98xx_bcd2tm(struct rtc_time *tm)
{
/*
 * Account for differences between how the RTC uses the values
 * and how they are defined in a struct rtc_time;
 */
	if ((tm->tm_year += (epoch - 1900)) <= 69) /* base is 1970 */
		tm->tm_year += 100;

	tm->tm_mon--;
}

void im98xx_get_rtc_time(struct rtc_time *tm)
{
#if 0
	unsigned short read_F, sec_unit;
	struct im98xx_rtc_time evb_time;

	read_F = 0;

	while (!read_F) {
		im98xx_rtc_read_enable();
		evb_time.tm_daymonyear = readl(ABB_DATE_REG);
		evb_time.tm_minhour = readl(ABB_MIN_HR_REG);
		evb_time.tm_sec = readl(ABB_SEC_REG);
		evb_time.tm_sec &= 0x3F;

		if ((evb_time.tm_daymonyear == readl(ABB_DATE_REG)) &&
		    (evb_time.tm_minhour == readl(ABB_MIN_HR_REG))) {
			im98xx_rtc_read_enable();
			sec_unit = readl(ABB_SEC_REG);
			sec_unit &= 0x3F;

			if ((evb_time.tm_sec <= sec_unit) && ((sec_unit - evb_time.tm_sec) <= 1)) {
				read_F = 1;
			}
		}
	}
#endif
	struct im98xx_rtc_time evb_time;

	im98xx_rtc_read_enable();
	evb_time.tm_daymonyear = readl(ABB_DATE_REG);
	evb_time.tm_minhour = readl(ABB_MIN_HR_REG);
	evb_time.tm_sec = (readl(ABB_SEC_REG) & 0x3F);

	im98xx_rtc_decodetime(&evb_time, tm);
	im98xx_bcd2tm(tm);
}

void im98xx_rtc_valid_check (void)
{
/* Check RTC valid*/
	unsigned short valid_F, done_F, sec_unit;
	struct rtc_time tm;
	struct im98xx_rtc_time evb_time;

	valid_F = 1;
	done_F = 0;

	im98xx_rtc_write(RTC_UNLOCK);
	writel(readl(ABB_RTC_CTL0_REG) & ~0x30, ABB_RTC_CTL0_REG);
	im98xx_rtc_write(RTC_LOCK);

	im98xx_get_rtc_time(&tm);

	if ((im98xx_rtc_valid_tm(&tm)) != 0) {
		valid_F = 0;
		BAREBOX_P("\n RTC format is invalid for iM98xx! \n Go to setup iM98xx default time! \n");

		im98xx_rtc_default_init(&tm);

		im98xx_rtc_encodetime(&evb_time, &tm);

		while (!done_F) {
			im98xx_rtc_write(RTC_UNLOCK);
			writel(evb_time.tm_daymonyear, ABB_DATE_REG);
			im98xx_rtc_write_busy();
			writel(evb_time.tm_minhour, ABB_MIN_HR_REG);
			im98xx_rtc_write_busy();
			writel(evb_time.tm_sec, ABB_SEC_REG);
			im98xx_rtc_write_busy();
			im98xx_rtc_write(RTC_LOCK);

			im98xx_rtc_read_enable();

			if ((evb_time.tm_daymonyear == readl(ABB_DATE_REG)) &&
			    (evb_time.tm_minhour == readl(ABB_MIN_HR_REG))) {
				sec_unit = readl(ABB_SEC_REG);
				sec_unit &= 0x3F;

				if ((evb_time.tm_sec <= sec_unit) && ((sec_unit - evb_time.tm_sec) <= 1)) {
					done_F = 1;
				}
			}
		}
	} else {
		BAREBOX_P("\n RTC format is valid for iM98xx! \n");
	}
}

unsigned int im98xx_get_seconds_tm(struct rtc_time *tm)
{
	unsigned int sw_seconds = 0;
	unsigned int leap_yr = 0;
	unsigned int yrs = 0;
	
	yrs = tm->tm_year + 1900;

	sw_seconds = tm->tm_sec;
	sw_seconds += (tm->tm_min) * 60;
//	sw_seconds += (tm->tm_hour) * 60 * 60;
	sw_seconds += (tm->tm_hour) * 3600;
//	sw_seconds += (tm->tm_mday - 1) * 24 * 60 * 60;
	sw_seconds += (tm->tm_mday - 1) * 86400;
//	sw_seconds += (sumofdays_in_mo[(tm->tm_mon)]) * 24 * 60 * 60;
	sw_seconds += (sumofdays_in_mo[(tm->tm_mon)]) * 86400;

	if (tm->tm_mon >= 2) {
		leap_yr = ((!(yrs % 4) && (yrs % 100)) || !(yrs % 400));
	}

//	sw_seconds += leap_yr * 24 * 60 * 60;
	sw_seconds += leap_yr * 86400;

	return sw_seconds;
}

unsigned int im98xx_read_back_chr_tm(void)
{
	return ((((readl(ABB_RTC_SW0_REG) & 0x1FF) << 16) & 0xFFFF0000) |
		(readl(ABB_RTC_SW1_REG) & 0xFFFF));
}

void im98xx_write_back_chr_tm(struct rtc_time *tm)
{
	unsigned short sw0_value, sw1_value;
	unsigned int seconds;

	seconds = im98xx_get_seconds_tm(tm);

	sw0_value = (seconds & 0x1FF0000) >> 16;
	sw1_value = seconds & 0xFFFF;

	im98xx_rtc_write(RTC_UNLOCK);

	do {
		writel(sw0_value, ABB_RTC_SW0_REG);
	} while (readl(ABB_RTC_SW0_REG) != sw0_value);

	do {
		writel(sw1_value, ABB_RTC_SW1_REG);
	} while (readl(ABB_RTC_SW1_REG) != sw1_value);

	im98xx_rtc_write(RTC_LOCK);
}

void im98xx_get_battery_adc(void)
{
	unsigned short buf_size;
	int fd1, fd2, fd3;
	char str[6];

	fd1 = open(BAT_V42_DIR, O_RDONLY);

	if (fd1 < 0) {
		charging_info.bat_v42_adc = BAT_V42_ADC;
		BAREBOX_P("\n 4200mV adc: 0x%x (default BAT_V42_ADC) \n", charging_info.bat_v42_adc);
	} else {
		buf_size = read(fd1, str, 6);
		charging_info.bat_v42_adc = simple_strtoul(str, NULL, 16);
		BAREBOX_P("\n 4200mV adc: 0x%x \n", charging_info.bat_v42_adc);
	}

	close(fd1);

	fd2 = open(BAT_V34_DIR, O_RDONLY);

	if (fd2 < 0) {
		charging_info.bat_v34_adc = BAT_V34_ADC;
		BAREBOX_P("\n 3400mV adc: 0x%x (default BAT_V34_ADC) \n", charging_info.bat_v34_adc);
	} else {
		buf_size = read(fd2, str, 6);
		charging_info.bat_v34_adc = simple_strtoul(str, NULL, 16);
		BAREBOX_P("\n 3400mV adc: 0x%x \n", charging_info.bat_v34_adc);
	}

	close(fd2);

	fd3 = open(BAT_I_DIR, O_RDONLY);

	if (fd3 < 0) {
		charging_info.bat_i_adc = BAT_I_ADC;
		BAREBOX_P("\n Current adc: 0x%x (default BAT_I_ADC) \n", charging_info.bat_i_adc);
	} else {
		buf_size = read(fd3, str, 6);
		charging_info.bat_i_adc = simple_strtoul(str, NULL, 16);
		BAREBOX_P("\n Current adc: 0x%x \n", charging_info.bat_i_adc);
	}

	close(fd3);
}

void im98xx_battery_adc2volt(void)
{
	if ((charging_info.volt_adc == charging_info.bat_v42_adc) ||
	    (charging_info.volt_adc == charging_info.bat_v34_adc)) {
		if (charging_info.volt_adc == charging_info.bat_v42_adc) {
			charging_info.battery_volt = BATTERY_MAX_VOLT;
		} else if (charging_info.volt_adc == charging_info.bat_v34_adc) {
			charging_info.battery_volt = BATTERY_MIN_VOLT;
		}
	} else if ((charging_info.volt_adc < charging_info.bat_v42_adc) &&
		   (charging_info.volt_adc > charging_info.bat_v34_adc)) {
		charging_info.battery_volt = BATTERY_MIN_VOLT;
		charging_info.battery_volt += (((BATTERY_MAX_VOLT - BATTERY_MIN_VOLT) *
					(charging_info.volt_adc - charging_info.bat_v34_adc)) /
					(charging_info.bat_v42_adc - charging_info.bat_v34_adc));
		BAREBOX_P("\n Voltage map to %d mV \n", charging_info.battery_volt);
	} else if (charging_info.volt_adc > charging_info.bat_v42_adc) {
		charging_info.battery_volt = BATTERY_MAX_VOLT;
		charging_info.battery_volt += (((BATTERY_MAX_VOLT - BATTERY_MIN_VOLT) *
					(charging_info.volt_adc - charging_info.bat_v42_adc)) /
					(charging_info.bat_v42_adc - charging_info.bat_v34_adc));
	} else if (charging_info.volt_adc < charging_info.bat_v34_adc) {
		charging_info.battery_volt = BATTERY_MIN_VOLT;
		charging_info.battery_volt -= (((BATTERY_MAX_VOLT - BATTERY_MIN_VOLT) *
					(charging_info.bat_v34_adc - charging_info.volt_adc)) /
					(charging_info.bat_v42_adc - charging_info.bat_v34_adc));
	}
}

void im98xx_read_battery_adc(void)
{
	unsigned short status;

	charging_info.volt_adc = 0;
	charging_info.current_adc = 0;

	/* Battery Voltage */
	status = readl(ABB_BAT_CHRSTOP_REG);

	if (status == ABB_CHR_ON) {
		writel(ABB_CHR_STOP, ABB_BAT_CHRSTOP_REG);

		im98xx_26MHz_halt(2000, GPT1);
	}

	writel((readl(ABB_ADC_STR_REG) & 0xFF) | BAT_VOLT_IN, ABB_ADC_STR_REG);

	im98xx_26MHz_halt(2000, GPT1);

	charging_info.volt_adc = im98xx_measure_adc_average(8, 2);

	if (status == ABB_CHR_ON) {
		writel(ABB_CHR_ON, ABB_BAT_CHRSTOP_REG);
	}

	BAREBOX_P("\n volt_adc = 0x%x \n", charging_info.volt_adc);

#if 0  //uncertain about the purpose to monitor charge current
	/* Battery Current */
	writel((readl(ABB_ADC_STR_REG) & 0xFF) | BAT_CURRENT_IN, ABB_ADC_STR_REG);

	im98xx_26MHz_halt(2000, GPT1);

	charging_info.current_adc = im98xx_measure_adc_average(16, 4);

	BAREBOX_P("\n current_adc = 0x%x \n", charging_info.current_adc);
#else
	charging_info.current_adc = 0; //dummy current info
#endif

	im98xx_battery_adc2volt();

	printf("\n battery_volt = %dmV \n", charging_info.battery_volt);
}

void im98xx_sleep2itr(unsigned int u32_microsecond)
{
	/* Key interrupt */
	/* clear key interrupt flag */
	if ((readl(KEY_STA_REG) & KEY_ITR_FLAG) == KEY_ITR_FLAG) {
		writel(readl(A9_32K_LOGIC_ITR_CLR) | (0x1 << 1), A9_32K_LOGIC_ITR_CLR);
	}

	/* enable key interrupt */
	writel(readl(A9ITR_IRQ_REG) | 0x1, A9ITR_IRQ_REG);

	/* enable abb interrupt, charger plug-out */
	writel(readl(A9ITR_IRQ_REG) | (0x1 << 10), A9ITR_IRQ_REG);
	writel(readl(ABB_BAT_CHRCFG_REG) | (0x1 << 13), ABB_BAT_CHRCFG_REG);

	im98xx_26MHz_halt(u32_microsecond, GPT1);

	if ((readl(KEY_STA_REG) & KEY_ITR_FLAG) == KEY_ITR_FLAG) {
		writel(readl(A9_32K_LOGIC_ITR_CLR) | (0x1 << 1), A9_32K_LOGIC_ITR_CLR);
	}
	if ((readl(ABB_BAT_CHRSTA_REG) & (0x1 << 5)) == (0x1 << 5)) {
		writel(readl(ABB_BAT_CHRSTA_REG) | (0x1 << 5), ABB_BAT_CHRSTA_REG);
	}

	/* disable key interrupt */
	writel(readl(A9ITR_IRQ_REG) & ~(0x1), A9ITR_IRQ_REG);

	/* disable abb interrupt */
	writel(readl(A9ITR_IRQ_REG) & ~(0x1 << 10), A9ITR_IRQ_REG);
	writel(readl(ABB_BAT_CHRCFG_REG) & ~(0x1F << 10), ABB_BAT_CHRCFG_REG);
}

void im98xx_barebox_pre_boot(void)
{
	unsigned short charger_status;
#if defined(CFG_BL_SHOW_PREBOOT_BATTERY)
	struct image_grp igrp;
#endif
	/* Battery/Charging status : on-going to charge */
	charger_status = readl(ABB_BAT_CHRSTA_REG);
	charger_status &= CHARGER_STA;

#if GPIO_TOGGLE_BACKLIGHT
	gpio_line_set_output(GPIO_BL_EN, GPIO_LOW);
	gpio_line_config(GPIO_BL_EN, GPIO_OUTPUT);
	writel(readl(GPIO_PIN_MUX_REG) & (~GPIO_PIN_MUX_PWL), GPIO_PIN_MUX_REG);
#else
	writel(readl(0xF80003A4) | GPIO_PIN_MUX_PWL, GPIO_PIN_MUX_REG);
#endif

	if (charger_status == CHARGER_STA) {
		/* Setup Charger constant current to 150mA. Because, V2 min power comsumption is 110mA. */
		writel((readl(ABB_BAT_CHRCFG_REG) & ~0x3F) | BAT_CHARGE_CUR, ABB_BAT_CHRCFG_REG);

		BAREBOX_P("\n Barebox status: Charging \n");
		BAREBOX_P(" Battery Monitor flag REG=0x%x\n", readl(ABB_BAT_MOITRFLG_REG));

		/* Monitor Battery/Charging capacity */	
		charging_F.present_F = 1;

		im98xx_read_battery_adc();

#if defined(CFG_BL_SHOW_PREBOOT_BATTERY)
		/* To show battery 3.4 ~ 4.2 V: charging_info.battery_volt */
		if (Show_ChargingIn == TRUE) {
			if (charging_info.battery_volt >= BATTERY_MIN_SHOW) {
				/* Initialize LCM/LCD */
				to_initialize_lcmlcd();

				/* Prepare logo */
#if defined(CFG_SHOW_APPLE_BATTERY)
				igrp.base = 0x40000000;
				igrp.pixel_fmt = IMG_RGB565;
				igrp.which_layer = LAY_MID;
				igrp.xres = 218;
				igrp.yres = 198;
				igrp.xofft = 0;
				igrp.yofft = 0;
#endif
#if defined(CFG_SHOW_DEFAULT_BATTERY)
				igrp.base = 0x40000000;
				igrp.pixel_fmt = IMG_RGB565;
				igrp.which_layer = LAY_MID;
				igrp.xres = 76;
				igrp.yres = 132;
				igrp.xofft = 0;
				igrp.yofft = 0;
#endif
				/* Show battery icon */
				to_show_battery(&igrp);
				im98xx_sleep2itr(100000);

				turn_backlight(true);

				/* delay 2s */
				im98xx_sleep2itr(2000000);

				turn_backlight(false);

				/* turn off LCM/LCD */
				turn_lcmlcd(false);
				/* Disable to Show */
				Show_ChargingIn = FALSE;

				/* To get battery again */
				im98xx_read_battery_adc();
			} else if (charging_info.battery_volt < BATTERY_MIN_SHOW) {
				/* turn-on LED backlight */
				writel(readl(ABB_PMU_LDO_REG) | (1<<14), ABB_PMU_LDO_REG);
				writel(0xc0, ABB_PWL_REG);
				/* delay 3s */
				im98xx_sleep2itr(3000000);
				/* turn-off LED backlight */
				writel(readl(ABB_PMU_LDO_REG) & ~(1<<14), ABB_PMU_LDO_REG);
				writel(0xff, ABB_PWL_REG);

				/* Disable to Show */
				Show_ChargingIn = FALSE;

				/* To get battery again */
				im98xx_read_battery_adc();
			}
		}
#endif

		if (charging_info.battery_volt <= BATTERY_MIN_PWR_OFF) {
			BAREBOX_P("\n Battery/Charging: Battery Low! \n Battery capacity => %dmV \n",
				 charging_info.battery_volt);

			if (charging_info.battery_volt < BATTERY_MIN_PWR_OFF) {
				writel(readl(ABB_SW_PWRBB_REG) & (~PWR_BB), ABB_SW_PWRBB_REG);
			}

			/* PC to CC handler: */
			/* Add 2-minutes to sleep while setup the Charger constant current to 150mA. Because, V2 min power comsumption is 110mA. */

			/* Sleep to wait for key/charger ITR */
			im98xx_sleep2itr(120000000);
		} else if ((charging_info.battery_volt <= BATTERY_MIN_CHR_BOOT) &&
			   (charging_info.battery_volt > BATTERY_MIN_PWR_OFF)) {
			BAREBOX_P("\n Power up BB: Battery capacity => %dmV \n", charging_info.battery_volt);
			writel(readl(ABB_SW_PWRBB_REG) | PWR_BB, ABB_SW_PWRBB_REG);

			/* Sleep to wait for key/charger ITR */
			im98xx_sleep2itr(120000000);
		} else if (charging_info.battery_volt > BATTERY_MIN_CHR_BOOT) {
			BAREBOX_P("\n Boot up System: Battery capacity => %dmV \n", charging_info.battery_volt);
			writel(readl(ABB_SW_PWRBB_REG) | PWR_BB, ABB_SW_PWRBB_REG);

			/* Go to Kernel precharge */
			IM98XXPowerOnType = PRE_BOOT;
		}
	} else {
		BAREBOX_P("\n Barebox status: No charging \n");
		BAREBOX_P("\n ABB_BAT_CHRSTA_REG=0x%x \n", readl(ABB_BAT_CHRSTA_REG));
		BAREBOX_P(" Battery Monitor Flag REG=0x%x\n", readl(ABB_BAT_MOITRFLG_REG));

		/* To indicate power adapter present status */
		if ((readl(ABB_BAT_CHRSTA_REG) & (1<<6)) == (1<<6)) {
			if (readl(WDT_SW2_REG) == 0xcafefeed) {
				printf("\n Skip Download mode. Power down BB. \n");
				writel(0x9828cafe, WDT_SW2_REG);
				writel(readl(ABB_SW_PWRBB_REG) & (~PWR_BB), ABB_SW_PWRBB_REG);
				im98xx_sleep2itr(120000000);
			} else {
				/* Go to USB download */
				IM98XXPowerOnType = DOWNLOAD_BOOT;
				writel(0xcafefeed, WDT_SW2_REG);
			}
		} else {
			printf("\n Power down BB. \n");

			writel(readl(ABB_SW_PWRBB_REG) & (~PWR_BB), ABB_SW_PWRBB_REG);

			/* Sleep to wait for key/charger ITR */
			im98xx_sleep2itr(120000000);
		}
	}
}

#define SET_VIBRATOR_BIT	(0x8000)	/* Enable vibrator bit */

void iM98xx_vibrator(bool on)
{
	unsigned int val = readl(ABB_PMU_LDO_REG);
	if (on) {
		/* vibrator turn on */
		val |= SET_VIBRATOR_BIT;
	} else {
		/* vibrator turn off */
		val &= ~SET_VIBRATOR_BIT;
	}
	writel(val, ABB_PMU_LDO_REG);
}

void im98xx_key_detection(void)
{
	struct rtc_time tm;
	unsigned short polling_cnt, unknown_cnt, combination_cnt, boot_type, battery_type;
	unsigned int value_of_row ,value_of_col;
	unsigned int keycode, col, row;
	const static char col_result_map[] = {0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0xDF ,0xBF, 0x7F};
	const static char row_result_map[] = {0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0xDF, 0xBF, 0x7F, 0x00};
	unsigned short charger_status;
	int i = 0;

	polling_cnt = 0;
	unknown_cnt = 0;
	battery_type = 0;

	do {
		combination_cnt = 0;
		writel((readl(KEY_ROW_REG) | HARDWARE_SCAN_ENABLE | HARDWARE_RSCAN | KEY_RELEASE_INT_ENABLE) & ~KEY_STARUS_0_7_RESET_VALUE, KEY_ROW_REG);
		keycode = (readl(KEY_STA_REG) & KEY_RELEASED_STATE);

		battery_type = 0;

		if (keycode == KEY_RELEASED_STATE) {
			//printf("\n 0x%x \n", keycode);
			polling_cnt = 0;
			unknown_cnt = 0;
			combination_cnt = 0;
			//printf("\n barebox_pre_boot() \n");
			im98xx_barebox_pre_boot();
		} else {
			if (keycode == POWER_KEY) {
				polling_cnt++;
			} else if ((keycode == BAT_C_KEY) || (keycode == BAT_D_KEY)) {
				battery_type = 1;
			} else {
				battery_type = 0;
				printf("\nkey 0x%x\n", keycode);
				unknown_cnt++;

				/* Remove to show LCM while battery is low. And add turn-on 1s LED backlight */
				if ((Show_ChargingIn == FALSE) && (charging_info.battery_volt < BATTERY_MIN_CHR_BOOT)) {
					/* turn-on LED backlight */
					writel(readl(ABB_PMU_LDO_REG) | (1<<14), ABB_PMU_LDO_REG);
					writel(0xf0, ABB_PWL_REG);
					/* delay 1s */
					im98xx_sleep2itr(1000000);
				}

				im98xx_barebox_pre_boot();
			}
		}

		im98xx_26MHz_halt(5000, GPT1);

		value_of_col = (keycode & KEY_STARUS_0_7_RESET_VALUE);
//		printf("Column keycode = 0x%x\n", value_of_col);

		if (battery_type == 1) {
			value_of_col |= 0x8;
		}

		for (col = 0; col < 8; col++) {
			if (col_result_map[col] == value_of_col) {
				BAREBOX_P("Column keycode = 0x%x\n", value_of_col);
				printf(" Column 0x%x", value_of_col);
				value_of_row = ((keycode >> 8) & 0xff);
				//printf("Row keycode = 0x%x\n", value_of_row);
				if (battery_type == 1) {
					value_of_row = 0xFD;
				}

				if ((value_of_col == 0xF7) && (value_of_row == 0x00)) {
					polling_cnt++;
				}

				for (row = 0; row < 8; row++) {
					if ((row_result_map[row] == value_of_row) &&
					    (value_of_row == 0xFD)) {
 						BAREBOX_P("Row keycode = 0x%x\n", value_of_row);
						printf(" Row 0x%x", value_of_row);

						if ((value_of_col == 0xFE) || (value_of_col == 0xFB)) {
							do {
								writel(KEY_STARUS_0_7_RESET_VALUE, KEY_ROW_REG);
								keycode = (readl(KEY_STA_REG) & KEY_STARUS_0_7_RESET_VALUE);
								combination_cnt++;
							} while ((keycode != POWER_KEY) && (combination_cnt < 0x100));
							combination_cnt = 0;
							polling_cnt = 0;
							unknown_cnt = 0;
						}
						if (keycode == POWER_KEY) {
							if (value_of_col == 0xFE) {
								IM98XXPowerOnType = CALIBRATION_BOOT;
								writel(readl(ABB_SW_PWRBB_REG) | PWR_BB, ABB_SW_PWRBB_REG);
								BAREBOX_P("\n iM98xx A7_CMD/Calibration! \n");
							} else if (value_of_col == 0xFB) {
								/* Write a special pattern to tell SDLoader */
								im98xx_write_signature(BT_MODE_COMPOMPOSITE_KEY);

								IM98XXPowerOnType = DOWNLOAD_BOOT;

								writel(readl(ABB_SW_PWRBB_REG) | PWR_BB, ABB_SW_PWRBB_REG);
								printf("\nKey-detection download\n");
								BAREBOX_P("\n iM98xx SD/USB Download! \n");
							}
						}
					}
				}
			}
		}

		if (polling_cnt >= PWR_ON_CNT) {
			im98xx_read_battery_adc();

			if ((charging_info.bat_v42_adc == BAT_V42_ADC) &&
			    (charging_info.bat_v34_adc == BAT_V34_ADC)) {
				/* Battery boot up => 1st to calibrate */
				BAREBOX_P("\n Need to calibrate Battery! \n");
			}

			charger_status = readl(ABB_BAT_CHRSTA_REG);
			charger_status &= CHARGER_STA;

			if ((charger_status == CHARGER_STA) && (charging_info.battery_volt > BATTERY_MIN_CHR_BOOT)) {
				IM98XXPowerOnType = POWER_ON;
				BAREBOX_P("\n iM98xx Power On Kernel/Android! (Battery w/s Charger-in) \n");
			} else if ((charger_status != CHARGER_STA) && (charging_info.battery_volt > BATTERY_MIN_BOOT)) {
				IM98XXPowerOnType = POWER_ON;
				BAREBOX_P("\n iM98xx Power On Kernel/Android! (Battery w/o Charger-in) \n");
			}
			BAREBOX_P("\n Battery = %d mV \n", charging_info.battery_volt);
		}
	} while ((IM98XXPowerOnType != POWER_ON) &&
		 (IM98XXPowerOnType != PRE_BOOT) &&
		 (IM98XXPowerOnType != CALIBRATION_BOOT) &&
		 (IM98XXPowerOnType != DOWNLOAD_BOOT));

	if (IM98XXPowerOnType != POWER_OFF) {
		boot_type = IM98XXPowerOnType;
		printf("\n boot_type = %d \n", boot_type);

		im98xx_write_back_chr_flag();

		im98xx_get_rtc_time(&tm);
		im98xx_write_back_chr_tm(&tm);

		BAREBOX_P("\n Power up BB to enter Kernel mode. \n");
		writel(readl(ABB_SW_PWRBB_REG) | PWR_BB, ABB_SW_PWRBB_REG);

		/* turn-on LED backlight */
		if ((IM98XXPowerOnType == POWER_ON) || (IM98XXPowerOnType == WARM_BOOT)) {
			writel(readl(ABB_PMU_LDO_REG) | (1 << 14), ABB_PMU_LDO_REG);

			iM98xx_vibrator(true);
			for(i = 0; i < 50; i++) {
				im98xx_26MHz_halt(2000, GPT1);
			}
			iM98xx_vibrator(false);
		}
	}
}

void im98xx_develop_boot(void)
{
	int fd = 0;
	char *src = "boot";

	/* edit a file to save to boot */

	switch (IM98XXPowerOnType) {
	case PRE_BOOT:
		fd = open(PRE_BOOT_DIR, O_WRONLY | O_TRUNC | O_CREAT);
		break;

	case DOWNLOAD_BOOT:
		fd = open(DOWNLOAD_BOOT_DIR, O_WRONLY | O_TRUNC | O_CREAT);
		break;

	case CALIBRATION_BOOT:
		fd = open(CALIBRATION_BOOT_DIR, O_WRONLY | O_TRUNC | O_CREAT);
		break;

	case RECOVERY_BOOT:
		fd = open(RECOVERY_BOOT_DIR, O_WRONLY | O_TRUNC | O_CREAT);
		break;

	case RTC_BOOT:
		fd = open(RTC_BOOT_DIR, O_WRONLY | O_TRUNC | O_CREAT);
		break;

	default:
		return;
	}

	if (fd < 0) {
		BAREBOX_P("\n could not open file for writing: %s\n", errno_str());
	} else {
		write(fd, src, strlen(src));
	}

	close(fd);
}

void im98xx_battery_init(void)
{
	charging_F.start_F = 0;
	charging_F.lock_F = 0;
	charging_F.eoc_F = 0;
	charging_F.error_F = 0;
	charging_F.present_F = 0;

	charging_info.volt_adc = 0;
	charging_info.current_adc = 0;
	charging_info.battery_volt = 0;
	charging_info.bat_v34_adc = BAT_V34_ADC;
	charging_info.bat_v42_adc = BAT_V42_ADC;
	charging_info.bat_i_adc = BAT_I_ADC;
	charging_info.start_tm = 0;

	im98xx_read_back_chr_flag();

	charging_info.start_tm = im98xx_read_back_chr_tm();

	im98xx_get_battery_adc();
}

void im98xx_delete_localfile(void)
{
	int fd;

	/* To initiate boot up feature */
	/* Delete CALIBRATION_BOOT_DIR */
	fd = open(CALIBRATION_BOOT_DIR, O_RDONLY);
	if (fd >= 0) {
		close(fd);
		unlink(CALIBRATION_BOOT_DIR);
	}

	/* Delete DOWNLOAD_BOOT_DIR */
	fd = open(DOWNLOAD_BOOT_DIR, O_RDONLY);
	if (fd >= 0) {
		close(fd);
		unlink(DOWNLOAD_BOOT_DIR);
	}

	/* Delete RECOVERY_BOOT_DIR */
	fd = open(RECOVERY_BOOT_DIR, O_RDONLY);
	if (fd >= 0) {
		close(fd);
		unlink(RECOVERY_BOOT_DIR);
	}

	/* Delete PRE_BOOT_DIR */
	fd = open(PRE_BOOT_DIR, O_RDONLY);
	if (fd >= 0) {
		close(fd);
		unlink(PRE_BOOT_DIR);
	}

	/* Delete RTC_BOOT_DIR */
	fd = open(RTC_BOOT_DIR, O_RDONLY);
	if (fd >= 0) {
		close(fd);
		unlink(RTC_BOOT_DIR);
	}
}

void im98xx_save_preboot(void)
{
	unsigned int preboot_value;
	unsigned char preboot_flag, cnt;

	if (IM98XXPowerOnType == PRE_BOOT) {
		preboot_flag = 1;
	} else {
		preboot_flag = 0;
	}

	cnt = 0;
	preboot_value = ((preboot_flag << 31) |
			 (charging_info.bat_v42_adc << 0) |
			 (charging_info.bat_v34_adc << 10) |
			 (charging_info.bat_i_adc << 20));

	do {
		writel(preboot_value, A97_SW7_REG);
		cnt++;
	} while ((readl(A97_SW7_REG) != preboot_value) && (cnt <= 100));

	if (cnt >= 100) {
		BAREBOX_P("\n Error: A97_SW7_REG write failed! \n");
	}
}

void im98xx_vref_trim(void)
{
	unsigned short i, u16_PMIC, u16_sample, u16_filter;
	unsigned short buf_size, trim_value;
	int fd;
	char str[6];

	trim_value = 0;
	u16_filter = 20;
	u16_sample = 64;

	fd = open(VREF_TRIM_DIR, O_RDONLY);

	if (fd < 0) {
		//printf("\n could not open file for reading: %s\n", errno_str());
		BAREBOX_P("\n Vreference trimming value: 0 (default) \n");
	} else {
		buf_size = read(fd, str, 6);
		trim_value = simple_strtoul(str, NULL, 16);
		BAREBOX_P("\n (Battery) Vreference trimming value: 0x%x \n", trim_value);

		if (trim_value < 0x80) {
			if (((readl(ABB_BGSEL_REG) & 0xF) == 0x7) ||
			    ((readl(ABB_BGSEL_REG) & 0xF) == 0x4)) {
			/* The bit#0 and bit#1 of ABB_BGSEL_REG will be reset after WDT reset. */
				/* write resistor-trim value to resistor-trim register */
				writel(trim_value, ABB_PMU_TRIM_REG);
			} else if ((readl(ABB_BGSEL_REG) & 0xF) == 0x8) {
				/* PMU reference voltage resistor-trim value configuration : Reset value => 0x0 */
				writel(0x0, ABB_BGSEL_REG);
				/* BandGap select : Enable normal reference. */
				writel(0x2, ABB_BGSEL_REG);
				/* write resistor-trim value to resistor-trim register */
				writel(trim_value, ABB_PMU_TRIM_REG);
				/* BandGap select : Enable normal reference and switch to normal reference. */
				writel(0x3, ABB_BGSEL_REG);
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
				BAREBOX_P("\n Calibrated normal_volt=%d mv \n", u16_PMIC * 2410 / 1024);
			}
		} else {
			BAREBOX_P("\n Error format to Vreference trimming value! \n");
		}
	}

	close(fd);
}

void im98xx_boot_type_init(void)
{
	writel((readl(A97_SW7_REG) & ~(1 << 31)), A97_SW7_REG);
}

void im98xx_battery_charging(void)
{
	unsigned int rtc_itr_flag, pwr_bb, rtc_value, rtc_alarm;
	unsigned int charger_status;
	struct image_grp igrp;

	writel(7200, ABB_CHR_TM_REG); /* Setup Charge Timer to 2 hour. */

#if 0 /* Mask: Not recommend to save Vtrim to NAND Falsh and read Vtrim from NAND Falsh. */
	im98xx_vref_trim();
#else

#if GPIO_TOGGLE_BACKLIGHT
	gpio_line_set_output(GPIO_BL_EN, GPIO_LOW);
	gpio_line_config(GPIO_BL_EN, GPIO_OUTPUT);
	writel(readl(GPIO_PIN_MUX_REG) & (~GPIO_PIN_MUX_PWL), GPIO_PIN_MUX_REG);
#else
	writel(readl(0xF80003A4) | GPIO_PIN_MUX_PWL, GPIO_PIN_MUX_REG);
#endif

#endif

	im98xx_rtc_write(RTC_UNLOCK);
	rtc_itr_flag = readl(ABB_RTC_CTL1_REG);
	pwr_bb = readl(ABB_SW_PWRBB_REG);
	im98xx_rtc_write(RTC_LOCK);

	im98xx_battery_init();

	/* Nobady cares, if shotdown abnormally: Reset RTC Periodic Alarm */
	if ((rtc_itr_flag & (ALARM_ITR_FLAG << 1)) == (ALARM_ITR_FLAG << 1)) {
		printf("\n Periodic RTC Alarm! \n");
		im98xx_rtc_write(RTC_UNLOCK);
		writel(readl(ABB_RTC_CTL0_REG) & ~(0x32), ABB_RTC_CTL0_REG);
		im98xx_rtc_write_busy();
		writel(readl(ABB_RTC_CTL1_REG) | 0x2, ABB_RTC_CTL1_REG);
		im98xx_rtc_write_busy();
		im98xx_rtc_write(RTC_LOCK);
	}

	if (readl(WDT_SW2_REG) == 0x77665502) {	/* Recovery boot */
		im98xx_delete_localfile();
		printf("\n Recovery Mode\n");
		IM98XXPowerOnType = RECOVERY_BOOT;
		im98xx_develop_boot();
	} else if (readl(WDT_SW2_REG) == 0x98289828) {	/* WDT Warm boot */
		IM98XXPowerOnType = WARM_BOOT;
		printf("\n WDT SW Warm Boot\n");
		writel(PWR_UP_BB, ABB_SW_PWRBB_REG);
	} else if (((pwr_bb & PWR_UP_BB) == PWR_UP_BB) && (readl(WDT_SW2_REG) != 0x01239828)) {
		IM98XXPowerOnType = WARM_BOOT;
		printf("\n HW WDT Boot\n");
	} else if (readl(WDT_SW1_REG) != 0x1234ABCD) {	/* Cold boot */
		unsigned int keycode;

		im98xx_delete_localfile();

		/* Initialize installed panel */
		panel_main_entry();

		if (((rtc_itr_flag & ALARM_ITR_FLAG) == 0) &&
		    ((rtc_itr_flag & (ALARM_ITR_FLAG << 1)) == 0)) {
		/* Boot up due to (1) Power key or (2) Adapter/Charger in. */
			pwr_bb &= ~(PWR_BB);
		} else if ((rtc_itr_flag & ALARM_ITR_FLAG) == ALARM_ITR_FLAG) {
			im98xx_rtc_write(RTC_UNLOCK);
			writel(RTC_READ_ENABLE, ABB_RTC_TEN_REG);
			im98xx_rtc_write_busy();
			rtc_value = readl(ABB_SEC_REG);
			rtc_alarm = readl(ABB_ASEC_REG);
			im98xx_rtc_write(RTC_LOCK);

			pwr_bb &= ~(PWR_BB);

			if ((rtc_value == rtc_alarm) ||
			    ((rtc_value >= rtc_alarm) && (rtc_value <= (rtc_alarm + 3)))) {
				/* Boot up due to an alarm */
				IM98XXPowerOnType = RTC_BOOT;
			} else {
				/* Boot up due to Battery insertion */
				rtc_itr_flag |= (ALARM_ITR_FLAG);

				/* If not RTC_BOOT: clear */
				im98xx_rtc_write(RTC_UNLOCK);
				writel(readl(ABB_RTC_CTL0_REG) & ~(0x31), ABB_RTC_CTL0_REG);
				im98xx_rtc_write_busy();
				writel(readl(ABB_RTC_CTL1_REG) | 0x1, ABB_RTC_CTL1_REG);
				im98xx_rtc_write_busy();
				im98xx_rtc_write(RTC_LOCK);
			}
		}

		if (IM98XXPowerOnType == RTC_BOOT) {
			writel(PWR_BB, ABB_SW_PWRBB_REG);
			im98xx_26MHz_halt(1000, GPT1);
			/* Reset RTC Alarm */
			im98xx_rtc_write(RTC_UNLOCK);
			writel(readl(ABB_RTC_CTL0_REG) & ~(0x31), ABB_RTC_CTL0_REG);
			im98xx_rtc_write_busy();
			writel(readl(ABB_RTC_CTL1_REG) | 0x1, ABB_RTC_CTL1_REG);
			im98xx_rtc_write_busy();
			im98xx_rtc_write(RTC_LOCK);
			BAREBOX_P("\n RTC Alarm detection: done! \n\n");
		} else {
			writel(pwr_bb, ABB_SW_PWRBB_REG);
		}

		writel(DBB_CHR_OP_ON, ABB_CHG_CTL_REG);
		writel(ABB_CHR_ON, ABB_BAT_CHRSTOP_REG);

		charger_status = readl(ABB_BAT_CHRSTA_REG);
		charger_status &= CHARGER_STA;

		if (charger_status == CHARGER_STA) {
			BAREBOX_P("\n Battery/Charging detection: ongoing! \n");
			/* Setup Charger constant current to 150mA. Because, V2 min power comsumption is 110mA. */
			writel((readl(ABB_BAT_CHRCFG_REG) & ~0x3F) | BAT_CHARGE_CUR, ABB_BAT_CHRCFG_REG);
		}

		im98xx_rtc_valid_check();

		BAREBOX_P("\n Press PWR key to boot up Kernel/Android! \n");
		BAREBOX_P("\n Otherwise, go to barebox pre-boot Battery/Charging flow! \n");

//		im98xx_battery_init();

		if (IM98XXPowerOnType != RTC_BOOT) {
			writel((readl(KEY_ROW_REG) | HARDWARE_SCAN_ENABLE | HARDWARE_RSCAN | KEY_RELEASE_INT_ENABLE) & ~KEY_STARUS_0_7_RESET_VALUE, KEY_ROW_REG);
			keycode = (readl(KEY_STA_REG) & KEY_RELEASED_STATE);

			if (keycode == POWER_KEY) {
				/* Disable to Show */
				Show_ChargingIn = FALSE;
			} else {
				im98xx_download_detection();
			}

			im98xx_read_battery_adc();
			/* To show battery 3.4 ~ 4.2 V: charging_info.battery_volt */

			charger_status = readl(ABB_BAT_CHRSTA_REG);
			charger_status &= CHARGER_STA;

			if (charger_status == CHARGER_STA) {
				if (readl(UBOOT2SD_TERM_MODE_ADDR) == NORMAL_TERMINATE) {
				} else if (keycode != POWER_KEY) {
					if (charging_info.battery_volt > BATTERY_MIN_SHOW) {
						/* Initialize LCM/LCD */
						to_initialize_lcmlcd();

						/* Prepare logo */
#if defined(CFG_SHOW_APPLE_BATTERY)
						igrp.base = 0x40000000;
						igrp.pixel_fmt = IMG_RGB565;
						igrp.which_layer = LAY_MID;
						igrp.xres = 218;
						igrp.yres = 198;
						igrp.xofft = 0;
						igrp.yofft = 0;
#endif
#if defined(CFG_SHOW_DEFAULT_BATTERY)
						igrp.base = 0x40000000;
						igrp.pixel_fmt = IMG_RGB565;
						igrp.which_layer = LAY_MID;
						igrp.xres = 76;
						igrp.yres = 132;
						igrp.xofft = 0;
						igrp.yofft = 0;
#endif
						/* Show battery icon */
						to_show_battery(&igrp);
						im98xx_sleep2itr(100000);

						/* turn-on backlight */
						turn_backlight(true);

						/* delay 5s */
						im98xx_sleep2itr(5000000);//mdelay(5000) ;

						/* turn-off backlight */
						turn_backlight(false);

						writel((readl(KEY_ROW_REG) | HARDWARE_SCAN_ENABLE | HARDWARE_RSCAN | KEY_RELEASE_INT_ENABLE) & ~KEY_STARUS_0_7_RESET_VALUE, KEY_ROW_REG);
						keycode = (readl(KEY_STA_REG) & KEY_RELEASED_STATE);

						if (keycode == POWER_KEY) {
							IM98XXPowerOnType = POWER_OFF;
						}

						/* turn off LCM/LCD */
						turn_lcmlcd(false);

						/* Disable to Show */
						Show_ChargingIn = FALSE;
					} else {
						Show_ChargingIn = TRUE;
					}

					charger_status = readl(ABB_BAT_CHRSTA_REG);
					charger_status &= CHARGER_STA;

					if (charger_status != CHARGER_STA) {
						IM98XXPowerOnType = POWER_OFF;
					}

				}
			} else {
				printf("No Charging!");
			}

			/* Show Charging ICon, then determine to auto-download detection */
			if (IM98XXPowerOnType == DOWNLOAD_BOOT) {
				/* Initialize LCM/LCD */
				to_initialize_lcmlcd();

				/* Prepare logo */
#if defined(CFG_SHOW_APPLE_BATTERY)
				igrp.base = 0x40015138;
				igrp.pixel_fmt = IMG_RGB565;
				igrp.which_layer = LAY_MID;
				igrp.xres = 64;
				igrp.yres = 128;
				igrp.xofft = 0;
				igrp.yofft = 0;
#endif
#if defined(CFG_SHOW_DEFAULT_BATTERY)
				igrp.base = 0x40004E60;
				igrp.pixel_fmt = IMG_RGB565;
				igrp.which_layer = LAY_MID;
				igrp.xres = 64;
				igrp.yres = 128;
				igrp.xofft = 0;
				igrp.yofft = 0;
#endif
				/* Show download icon, RAM-Loader to enable backlight */
				to_show_battery(&igrp);
				im98xx_sleep2itr(100000);

				/* turn-off backlight */
				turn_backlight(false);
			} else {
				/* turn-off backlight */
				turn_backlight(false);

				/* Detect key event */
				im98xx_key_detection();
			}
		}

		if (IM98XXPowerOnType != DOWNLOAD_BOOT) {
			/* turn off LCM/LCD */
			turn_lcmlcd(false);
		}

		im98xx_develop_boot();
	} else {	/* Warm boot */
		IM98XXPowerOnType = WARM_BOOT;
		printf("\n Warm boot! \n");
		writel(PWR_UP_BB, ABB_SW_PWRBB_REG);
	}

	im98xx_save_preboot();

	if ((IM98XXPowerOnType == POWER_ON) ||
	    (IM98XXPowerOnType == WARM_BOOT) ||
	    (IM98XXPowerOnType == PRE_BOOT)) {
		unsigned int rtc_value, rtc_alarm;

		im98xx_rtc_write(RTC_UNLOCK);

		if ((readl(ABB_RTC_CTL0_REG) & 0x1) == 0x1) {
			writel(RTC_READ_ENABLE, ABB_RTC_TEN_REG);
			im98xx_rtc_write_busy();

			if ((readl(ABB_DATE_REG) == readl(ABB_ADATE_REG)) &&
			    (readl(ABB_MIN_HR_REG) <= readl(ABB_AMIN_HR_REG))) {
				rtc_value = readl(ABB_SEC_REG) + 60 * (readl(ABB_MIN_HR_REG) & 0x3F);
				rtc_alarm = readl(ABB_ASEC_REG) + 60 * (readl(ABB_AMIN_HR_REG) & 0x3F);

				if ((rtc_alarm - rtc_value) < 60) {
					IM98XXPowerOnType = RTC_BOOT;
					printf("\n Forecast: RTC Alarm! \n");

					/* Update the FS of barebox */
					im98xx_delete_localfile();
					im98xx_develop_boot();
				}
			}
		}

		im98xx_rtc_write(RTC_LOCK);
	}

	if (IM98XXPowerOnType == RTC_BOOT) {
		writel(PWR_BB, ABB_SW_PWRBB_REG);
		BAREBOX_P("\n RTC Alarm! \n");
		/* Reset RTC Alarm and Periodic Alarm */
		im98xx_rtc_write(RTC_UNLOCK);
		writel(readl(ABB_RTC_CTL0_REG) & ~(0x33), ABB_RTC_CTL0_REG);
		im98xx_rtc_write_busy();
		writel(readl(ABB_RTC_CTL1_REG) | 0x3, ABB_RTC_CTL1_REG);
		im98xx_rtc_write_busy();
		im98xx_rtc_write(RTC_LOCK);
		BAREBOX_P("\n RTC Alarm detection: done! \n\n");
	}

	if (IM98XXPowerOnType == DOWNLOAD_BOOT) {
		writel(0x01239828, WDT_SW2_REG);
	}
}


/*============================================================================*/
static int do_saradc_i(int argc, char *argv[])
{
	int fd;
	char *src = "";

	if (argc < 2)
		return COMMAND_ERROR_USAGE;

	charging_info.bat_i_adc = simple_strtoul(argv[1], NULL, 16);

	fd = open(BAT_I_DIR, O_WRONLY | O_TRUNC | O_CREAT);
	sprintf(src, "0x%x", charging_info.bat_i_adc);
	write(fd, src, strlen(src));
	close(fd);

	fd = open("/env/batt_cal_imax", O_WRONLY | O_TRUNC | O_CREAT);
	sprintf(src, "#!/bin/sh\nimax=0x%x", charging_info.bat_i_adc);
	write(fd, src, strlen(src));
	close(fd);

	return 0;
}

static const __maybe_unused char cmd_saradc_i_help[] =
"Usage: saradc_i value(hex) \n";

BAREBOX_CMD_START(saradc_i)
	.cmd		= do_saradc_i,
	BAREBOX_CMD_HELP(cmd_saradc_i_help)
BAREBOX_CMD_END


/*============================================================================*/
static int do_saradc_v(int argc, char *argv[])
{
	int fd;
	char *src = "";
	unsigned short voltage;

	if (argc < 3)
		return COMMAND_ERROR_USAGE;

	voltage = simple_strtoul(argv[1], NULL, 10);

	if (voltage == BATTERY_MAX_VOLT) {
		charging_info.bat_v42_adc = simple_strtoul(argv[2], NULL, 16);

		fd = open(BAT_V42_DIR, O_WRONLY | O_TRUNC | O_CREAT);
		sprintf(src, "0x%x", charging_info.bat_v42_adc);
		write(fd, src, strlen(src));
		close(fd);
		fd = open("/env/batt_cal_vmax", O_WRONLY | O_TRUNC | O_CREAT);
		sprintf(src, "#!/bin/sh\nvmax=0x%x", charging_info.bat_v42_adc);
		write(fd, src, strlen(src));
		close(fd);
	} else if (voltage == BATTERY_MIN_VOLT) {
		charging_info.bat_v34_adc = simple_strtoul(argv[2], NULL, 16);

		fd = open(BAT_V34_DIR, O_WRONLY | O_TRUNC | O_CREAT);
		sprintf(src, "0x%x", charging_info.bat_v34_adc);
		write(fd, src, strlen(src));
		close(fd);
		fd = open("/env/batt_cal_vmin", O_WRONLY | O_TRUNC | O_CREAT);
		sprintf(src, "#!/bin/sh\nvmin=0x%x", charging_info.bat_v34_adc);
		write(fd, src, strlen(src));
		close(fd);
	} else {
		printf("\n Error usage! \n");
	}

	return 0;
}

static const __maybe_unused char cmd_saradc_v_help[] =
"Usage: saradc_v volt_level(mV) value(hex) \n";

BAREBOX_CMD_START(saradc_v)
	.cmd		= do_saradc_v,
	BAREBOX_CMD_HELP(cmd_saradc_v_help)
BAREBOX_CMD_END


/*============================================================================*/
static int do_v_trim(int argc, char *argv[])
{
	int fd;
	char *src = "";
	unsigned short trim_value = 0;

	if (argc < 2)
		return COMMAND_ERROR_USAGE;

	trim_value = simple_strtoul(argv[1], NULL, 16);

	fd = open(VREF_TRIM_DIR, O_WRONLY | O_TRUNC | O_CREAT);
	sprintf(src, "0x%x", trim_value);
	write(fd, src, strlen(src));
	close(fd);

	return 0;
}

static const __maybe_unused char cmd_v_trim_help[] =
"Usage: v_trim value(hex) \n";

BAREBOX_CMD_START(v_trim)
	.cmd		= do_v_trim,
	BAREBOX_CMD_HELP(cmd_v_trim_help)
BAREBOX_CMD_END


/*============================================================================*/
static int do_bat_adc_i(int argc, char *argv[])
{
	int fd;
	char *src = "";

	if (argc < 1)
		return COMMAND_ERROR_USAGE;

	writel((readl(ABB_ADC_STR_REG) & 0xFF) | BAT_CURRENT_IN, ABB_ADC_STR_REG);

	im98xx_26MHz_halt(2000, GPT1);

	charging_info.bat_i_adc = im98xx_measure_adc_average(16, 4);
	printf("\n SAR_ADC Current {16, 4} = 0x%x \n", charging_info.bat_i_adc);

	fd = open(BAT_I_DIR, O_WRONLY | O_TRUNC | O_CREAT);
	sprintf(src, "0x%x", charging_info.bat_i_adc);
	write(fd, src, strlen(src));
	close(fd);

	fd = open("/env/batt_cal_imax", O_WRONLY | O_TRUNC | O_CREAT);
	sprintf(src, "#!/bin/sh\nimax=0x%x", charging_info.bat_i_adc);
	write(fd, src, strlen(src));
	close(fd);

	return 0;
}

static const __maybe_unused char cmd_bat_adc_i_help[] =
"Usage: bat_adc_i \n";

BAREBOX_CMD_START(bat_adc_i)
	.cmd		= do_bat_adc_i,
	BAREBOX_CMD_HELP(cmd_bat_adc_i_help)
BAREBOX_CMD_END


/*============================================================================*/
static int do_bat_adc_v(int argc, char *argv[])
{
	int fd;
	char *src = "";
	unsigned short status, voltage;

	if (argc < 2)
		return COMMAND_ERROR_USAGE;

	voltage = simple_strtoul(argv[1], NULL, 10);

	status = readl(ABB_BAT_CHRSTOP_REG);

	if (status == ABB_CHR_ON) {
		writel(ABB_CHR_STOP, ABB_BAT_CHRSTOP_REG);
	
		im98xx_26MHz_halt(2000, GPT1);
	}

	writel((readl(ABB_ADC_STR_REG) & 0xFF) | BAT_VOLT_IN, ABB_ADC_STR_REG);

	im98xx_26MHz_halt(2000, GPT1);

	if (voltage == BATTERY_MAX_VOLT) {
		charging_info.bat_v42_adc = im98xx_measure_adc_average(16, 4);
		printf("\n SAR_ADC Voltage {16, 4} = 0x%x \n", charging_info.bat_v42_adc);

		fd = open(BAT_V42_DIR, O_WRONLY | O_TRUNC | O_CREAT);
		sprintf(src, "0x%x", charging_info.bat_v42_adc);
		write(fd, src, strlen(src));
		close(fd);
		fd = open("/env/batt_cal_vmax", O_WRONLY | O_TRUNC | O_CREAT);
		sprintf(src, "#!/bin/sh\nvmax=0x%x", charging_info.bat_v42_adc);
		write(fd, src, strlen(src));
		close(fd);
	} else if (voltage == BATTERY_MIN_VOLT) {
		charging_info.bat_v34_adc = im98xx_measure_adc_average(16, 4);
		printf("\n SAR_ADC Voltage {16, 4} = 0x%x \n", charging_info.bat_v34_adc);

		fd = open(BAT_V34_DIR, O_WRONLY | O_TRUNC | O_CREAT);
		sprintf(src, "0x%x", charging_info.bat_v34_adc);
		write(fd, src, strlen(src));
		close(fd);
		fd = open("/env/batt_cal_vmin", O_WRONLY | O_TRUNC | O_CREAT);
		sprintf(src, "#!/bin/sh\nvmin=0x%x", charging_info.bat_v34_adc);
		write(fd, src, strlen(src));
		close(fd);
	} else {
		printf("\n SAR_ADC Voltage {16, 4} = 0x%x \n", im98xx_measure_adc_average(16, 4));
	}

	if (status == ABB_CHR_ON) {
		writel(ABB_CHR_ON, ABB_BAT_CHRSTOP_REG);
	}

	return 0;
}

static const __maybe_unused char cmd_bat_adc_v_help[] =
"Usage: bat_adc_v mV \n";

BAREBOX_CMD_START(bat_adc_v)
	.cmd		= do_bat_adc_v,
	BAREBOX_CMD_HELP(cmd_bat_adc_v_help)
BAREBOX_CMD_END


/*============================================================================*/
static int do_activate_calibration(int argc, char *argv[])
{
	unsigned short mode;

	if (argc < 1)
		return COMMAND_ERROR_USAGE;

	if (argc == 1) {
		mode = simple_strtoul(argv[1], NULL, 10);
	} else {
		mode = 1;
	}

	im98xx_delete_localfile();

	IM98XXPowerOnType = CALIBRATION_BOOT;

	printf("Calibration: [InfomaxCT bootloader(fastboot)]\r\n");

	im98xx_develop_boot();

	if (mode == 2) {
		run_command("boot nand");
	}

	return 0;
}

static const __maybe_unused char cmd_InfomaxCT_help[] =
"Usage: InfomaxCT mode (mode => 1:Battery+RF, 2:Only RF)\n";

BAREBOX_CMD_START(InfomaxCT)
	.cmd		= do_activate_calibration,
	BAREBOX_CMD_HELP(cmd_InfomaxCT_help)
BAREBOX_CMD_END


/*============================================================================*/
static int do_battery_charging(int argc, char *argv[])
{
	unsigned short charger_status;

	if (argc < 1)
		return COMMAND_ERROR_USAGE;

	IM98XXPowerOnType = POWER_OFF;

	im98xx_battery_charging();

	charger_status = readl(ABB_BAT_CHRSTA_REG);
	charger_status &= CHARGER_STA;

	if ((charger_status == CHARGER_STA) &&
	    (IM98XXPowerOnType == DOWNLOAD_BOOT)) {
		if (charging_info.battery_volt > BATTERY_MIN_CHR_BOOT) {
			/* Setup Charger constant current to 300mA. */
			writel((readl(ABB_BAT_CHRCFG_REG) & ~0x3F) | BAT_CHARGE_CUR, ABB_BAT_CHRCFG_REG);
		} else {
			printf("\n Warning: Charger Current => 150mA. \n");
		}
	}
	
	return 0;
}

static const __maybe_unused char cmd_battery_charging_help[] =
"Usage: battery_charging \n";

BAREBOX_CMD_START(battery_charging)
	.cmd		= do_battery_charging,
	BAREBOX_CMD_HELP(cmd_battery_charging_help)
BAREBOX_CMD_END


/*============================================================================*/
static int do_download_signature(int argc, char *argv[])
{
	if (argc < 1)
		return COMMAND_ERROR_USAGE;

	/* Write a special pattern to tell SDLoader */
	im98xx_write_signature(BT_MODE_COMMAND);

	return 0;
}

static const __maybe_unused char cmd_download_signature_help[] =
"Usage: download_signature \n";

BAREBOX_CMD_START(download_signature)
	.cmd		= do_download_signature,
	BAREBOX_CMD_HELP(cmd_download_signature_help)
BAREBOX_CMD_END

