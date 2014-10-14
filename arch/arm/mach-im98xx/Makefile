# OMAP Architecture Makefile
#
# (C) Copyright 2008
# Texas Instruments, <www.ti.com>
# Nishanth Menon <x0nishan@ti.com>
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#
lwl-$(CONFIG_ARCH_IM98XX) += syslib.o
obj-$(CONFIG_ARCH_IM98XX) += syslib.o
obj-$(CONFIG_IM98XX_CLOCK_SOURCE_S32K) += s32k_clksource.o
obj-$(CONFIG_ARCH_IM98XX) += im98xx_generic.o
lwl-$(CONFIG_ARCH_HAS_LOWLEVEL_INIT) += im98xx_core.o
lwl-y += lowlevel.o
obj-$(CONFIG_ARCH_HAS_LOWLEVEL_INIT) += im98xx_core.o
obj-y += lowlevel.o
#obj-$(CONFIG_IM98XX_CLOCK_CONFIG) += im98xx_clock.o
lwl-$(CONFIG_IM98XX_CLOCK_CONFIG) += im98xx_clock_core.o im98xx_clock.o
lwl-$(CONFIG_IM98XX_HALT_TIMER_CONFIG) += im98xx_timer.o
lwl-$(CONFIG_IM98XX_BL_PM_CONFIG) += im98xx_bl_pm.o
lwl-$(CONFIG_ARCH_VECTOR_A7_CONFIG) += im98xx_a7_download.o
lwl-$(CONFIG_IM98XX_SARADC_CONFIG) += im98xx_saradc.o im98xx_battery.o showbat/
obj-$(CONFIG_IM98XX_CLOCK_CONFIG) += im98xx_clock_core.o im98xx_clock.o
obj-$(CONFIG_IM98XX_HALT_TIMER_CONFIG) += im98xx_timer.o
obj-$(CONFIG_IM98XX_BL_PM_CONFIG) += im98xx_bl_pm.o
obj-$(CONFIG_ARCH_VECTOR_A7_CONFIG) += im98xx_a7_download.o
obj-$(CONFIG_IM98XX_SARADC_CONFIG) += im98xx_saradc.o im98xx_battery.o showbat/
