/*
 * Copyright (C) 2009 by Marvell Semiconductors, Inc.
 * Written by Nicolas Pitre <nico at marvell.com>
 *
 * Copyright (C) 2009 by David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arm_nandio.h"
#include "armv4_5.h"


/*
 * ARM-specific bulk write from buffer to address of 8-bit wide NAND.
 * For now this only supports ARMv4 and ARMv5 cores.
 *
 * Enhancements to target_run_algorithm() could enable:
 *   - ARMv6 and ARMv7 cores in ARM mode
 *
 * Different code fragments could handle:
 *   - Thumb2 cores like Cortex-M (needs different byteswapping)
 *   - 16-bit wide data (needs different setup too)
 */
int arm_nandwrite(struct arm_nand_data *nand, uint8_t *data, int size)
{
	target_t		*target = nand->target;
	armv4_5_algorithm_t	algo;
	armv4_5_common_t	*armv4_5 = target->arch_info;
	reg_param_t		reg_params[3];
	uint32_t		target_buf;
	uint32_t		exit = 0;
	int			retval;

	/* Inputs:
	 *  r0	NAND data address (byte wide)
	 *  r1	buffer address
	 *  r2	buffer length
	 */
	static const uint32_t code[] = {
		0xe4d13001,	/* s: ldrb  r3, [r1], #1 */
		0xe5c03000,	/*    strb  r3, [r0]     */
		0xe2522001,	/*    subs  r2, r2, #1   */
		0x1afffffb,	/*    bne   s            */

		/* exit: ARMv4 needs hardware breakpoint */
		0xe1200070,	/* e: bkpt  #0           */
	};

	if (!nand->copy_area) {
		uint8_t		code_buf[sizeof(code)];
		unsigned	i;

		/* make sure we have a working area */
		if (target_alloc_working_area(target,
				sizeof(code) + nand->chunk_size,
				&nand->copy_area) != ERROR_OK) {
			LOG_DEBUG("%s: no %d byte buffer",
					__FUNCTION__,
					(int) sizeof(code) + nand->chunk_size);
			return ERROR_NAND_NO_BUFFER;
		}

		/* buffer code in target endianness */
		for (i = 0; i < sizeof(code) / 4; i++)
			target_buffer_set_u32(target, code_buf + i * 4, code[i]);

		/* copy code to work area */
                retval = target_write_memory(target,
					nand->copy_area->address,
					4, sizeof(code) / 4, code_buf);
		if (retval != ERROR_OK)
			return retval;
	}

	/* copy data to work area */
	target_buf = nand->copy_area->address + sizeof(code);
	retval = target_bulk_write_memory(target, target_buf, size / 4, data);
	if (retval == ERROR_OK && (size & 3) != 0)
		retval = target_write_memory(target,
				target_buf + (size & ~3),
				1, size & 3, data + (size & ~3));
	if (retval != ERROR_OK)
		return retval;

	/* set up algorithm and parameters */
	algo.common_magic = ARMV4_5_COMMON_MAGIC;
	algo.core_mode = ARMV4_5_MODE_SVC;
	algo.core_state = ARMV4_5_STATE_ARM;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_IN);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_IN);

	buf_set_u32(reg_params[0].value, 0, 32, nand->data);
	buf_set_u32(reg_params[1].value, 0, 32, target_buf);
	buf_set_u32(reg_params[2].value, 0, 32, size);

	/* armv4 must exit using a hardware breakpoint */
	if (armv4_5->is_armv4)
		exit = nand->copy_area->address + sizeof(code) - 4;

	/* use alg to write data from work area to NAND chip */
	retval = target_run_algorithm(target, 0, NULL, 3, reg_params,
			nand->copy_area->address, exit, 1000, &algo);
	if (retval != ERROR_OK)
		LOG_ERROR("error executing hosted NAND write");

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);

	return retval;
}

/* REVISIT do the same for bulk *read* too ... */
