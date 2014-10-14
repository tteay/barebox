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
#include <mach/magic.h>
#include <mach/mem_map.h>

#include <command.h>
#include <errno.h>

extern unsigned int a7_vector_sdram_start;
extern unsigned int a7_vector_sdram_end;
extern unsigned int a7_vector_debug_start;
extern unsigned int a7_vector_debug_end;

void mem_cpy(unsigned int src_adr, unsigned int dst_adr, unsigned int len)
{
	unsigned int cnt = 0;

	do {
		*((volatile unsigned int *)(dst_adr + (cnt << 2))) =
		*((volatile unsigned int *)(src_adr + (cnt << 2)));
		cnt++;
	} while (cnt < len);
}

void im98xx_a7vector_init(bool debug)
{
/* ARM7 SDRAM */
	unsigned long src_addr, xfer_len, tmp;
	unsigned long des_addr = 0x28080000;

	if (debug) {
		src_addr = (unsigned long)&a7_vector_debug_start;
		xfer_len = (&a7_vector_debug_end - &a7_vector_debug_start);
	} else {
		src_addr = (unsigned long)&a7_vector_sdram_start;
		xfer_len = (&a7_vector_sdram_end - &a7_vector_sdram_start);
	}

	tmp = xfer_len % sizeof(unsigned long);

	if (tmp)
		xfer_len += (sizeof(unsigned long) - tmp);

	mem_cpy(src_addr, des_addr, xfer_len);

	if (debug) {
		BAREBOX_P("A7 Vector Debug download!\n");
	} else {
		BAREBOX_P("A7 Vector SDRAM download!\n");
	}
}

void im98xx_a7_reset(void)
{
	writel(0x0, A7_CORE_RST_REG); /* de-assert arm7 reset */

	BAREBOX_P("De-assert A7 Reset!\n");
}


/*============================================================================*/
static int do_a7_vector_debug(int argc, char *argv[])
{
	im98xx_a7vector_init(true);

	return 0;
}

static const __maybe_unused char cmd_a7_vector_debug_help[] =
"Usage: a7_vector_debug \n";

BAREBOX_CMD_START(a7_vector_debug)
	.cmd		= do_a7_vector_debug,
	BAREBOX_CMD_HELP(cmd_a7_vector_debug_help)
BAREBOX_CMD_END


/*============================================================================*/
static int do_a7_vector_sdram(int argc, char *argv[])
{
	im98xx_a7vector_init(false);

	return 0;
}

static const __maybe_unused char cmd_a7_vector_sdram_help[] =
"Usage: a7_vector_sdram \n";

BAREBOX_CMD_START(a7_vector_sdram)
	.cmd		= do_a7_vector_sdram,
	BAREBOX_CMD_HELP(cmd_a7_vector_sdram_help)
BAREBOX_CMD_END


/*============================================================================*/
static int do_a7_reset(int argc, char *argv[])
{
	im98xx_a7_reset();

	return 0;
}

static const __maybe_unused char cmd_a7_reset_help[] =
"Usage: a7_reset \n";

BAREBOX_CMD_START(a7_reset)
	.cmd		= do_a7_reset,
	BAREBOX_CMD_HELP(cmd_a7_reset_help)
BAREBOX_CMD_END

