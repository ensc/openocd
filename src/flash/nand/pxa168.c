/*	--*- c -*--
 * Copyright (C) 2010 Enrico Scholz <enrico.scholz@sigma-chemnitz.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 and/or 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <target/arm.h>

#include "arm_io.h"
#include "driver.h"
#include "core.h"

#define NFC_BASE		(0xD4283000u)
#define NFC_NDCR		(NFC_BASE + 0x00u)
#define NFC_NDTR0		(NFC_BASE + 0x04u)
#define NFC_NDTR1		(NFC_BASE + 0x0cu)
#define NFC_NDSR		(NFC_BASE + 0x14u)
#define NFC_NDPCR		(NFC_BASE + 0x18u)
#define NFC_NDECCCTRL		(NFC_BASE + 0x28u)
#define NFC_NDDB		(NFC_BASE + 0x40u)
#define NFC_NDCB0		(NFC_BASE + 0x48u)
#define NFC_NDCB1		(NFC_BASE + 0x4cu)
#define NFC_NDCB2		(NFC_BASE + 0x50u)
#define NFC_NDCB3		(NFC_BASE + 0x54u)

#define NFC_NDCR_SPARE_EN	(1u<<31)
#define NFC_NDCR_ECC_EN		(1u<<30)
#define NFC_NDCR_DWIDTH_C	(1u<<27)
#define NFC_NDCR_ND_RUN		(1u<<28)

#define NFC_NDSR_WRCMDREQ	(1u<< 0)
#define NFC_NDSR_RDDREQ		(1u<< 1)
#define NFC_NDSR_WRDREQ		(1u<< 2)

#define NFC_NDCB0_CMD1(_cmd)		((_cmd) <<  0)
#define NFC_NDCB0_CMD2(_cmd)		((_cmd) <<  8)
#define NFC_NDCB0_ADDR_CYC(_cnt)	((_cnt) << 16)
#define NFC_NDCB0_DBC			(1u << 19) /* Double Byte Command */
#define NFC_NDCB0_NC			(1u << 20) /* Next Command */
#define NFC_NDCB0_CMD_TYPE(_type)	((_type) << 21)
#define NFC_NDCB0_CMD_TYPE_READ		NFC_NDCB0_CMD_TYPE(0)
#define NFC_NDCB0_CMD_TYPE_WRITE	NFC_NDCB0_CMD_TYPE(1)
#define NFC_NDCB0_CMD_TYPE_ERASE	NFC_NDCB0_CMD_TYPE(2)
#define NFC_NDCB0_CMD_TYPE_READID	NFC_NDCB0_CMD_TYPE(3)
#define NFC_NDCB0_CMD_TYPE_STATUS	NFC_NDCB0_CMD_TYPE(4)
#define NFC_NDCB0_CMD_TYPE_RESET	NFC_NDCB0_CMD_TYPE(5)
#define NFC_NDCB0_CMD_TYPE_NAKED_CMD	NFC_NDCB0_CMD_TYPE(6)
#define NFC_NDCB0_CMD_TYPE_NAKED_ADDR	NFC_NDCB0_CMD_TYPE(7)
#define NFC_NDCB0_CSEL(_cs)		((_cs) << 24)
#define NFC_NDCB0_AUTO_RS		(1u << 25)
#define NFC_NDCB0_AUTO_ST_ROW_EN	(1u << 26) /* Status Row Address Enable */
#define NFC_NDCB0_AUTO_RDY_BYP		(1u << 27) /* Ready Bypass */
#define NFC_NDCB0_AUTO_LEN_OVRD		(1u << 28) /* Length Override */
#define NFC_NDCB0_CMD_XTYPE(_type)	((_type) << 29)
#define NFC_NDCB0_CMD_XTYPE_MONO_RW	NFC_NDCB0_CMD_XTYPE(0)
#define NFC_NDCB0_CMD_XTYPE_LAST_NAKED		NFC_NDCB0_CMD_XTYPE(1)
#define NFC_NDCB0_CMD_XTYPE_ILLEGAL_NAKED	NFC_NDCB0_CMD_XTYPE(2)
#define NFC_NDCB0_CMD_XTYPE_ILLEGAL_DISPATCH	NFC_NDCB0_CMD_XTYPE(3)
#define NFC_NDCB0_CMD_XTYPE_READ_DISPATCH	NFC_NDCB0_CMD_XTYPE(4)
#define NFC_NDCB0_CMD_XTYPE_NAKED_RW		NFC_NDCB0_CMD_XTYPE(5)
#define NFC_NDCB0_CMD_XTYPE_CMD_DISPATCH	NFC_NDCB0_CMD_XTYPE(6)

#define NFC_NDECCCTRL_BCH_EN		(1u << 0)

struct pxa168_nand_controller
{
	struct target		*target;
	struct arm_nand_data	io;

	uint32_t		ndcr;
	uint32_t		ndcb[5];
	uint32_t		ndeccctrl;

	size_t			addr_cnt;
	size_t			addr_idx;

	size_t			read_cnt;
	size_t			read_idx;
	unsigned char		read_buf[(2048 + 64)*2];
};

static int pxa168_nand_ready(struct nand_device *nand, int to)
{
	struct pxa168_nand_controller	*ctrl = nand->controller_priv;
	struct target			*target = ctrl->target;
	int				rc;
	uint32_t			ndsr;

	do {
		rc = target_read_u32(target, NFC_NDSR, &ndsr);
		--to;
	} while (rc >= 0 && to > 0 &&
		 (ndsr & (NFC_NDSR_RDDREQ|(1<<8))) == 0);

	if ((ndsr & (NFC_NDSR_RDDREQ|(1<<8))) == 0)
		rc = ERROR_NAND_OPERATION_TIMEOUT;

	LOG_INFO("%s -> %d/%d/%08x", __func__, rc, to, ndsr);
	return rc < 0 ? 0 : 1;
};

static int pxa168_nand_addr_complete(struct pxa168_nand_controller *ctrl)
{
	struct target			*target = ctrl->target;
	uint32_t			ndsr;
	int				rc;
	unsigned int			to = 10;
	size_t				i;

	rc = target_write_u32(target, NFC_NDSR, ~0u);
	if (rc >= 0)
		rc = target_write_u32(target, NFC_NDCR,
				      ctrl->ndcr | NFC_NDCR_ND_RUN);

	if (rc < 0)
		goto out;

	printf("NDCR=%08x\n", ctrl->ndcr | NFC_NDCR_ND_RUN);

	do {
		rc = target_read_u32(target, NFC_NDSR, &ndsr);
		--to;
	} while (rc >= 0 && to > 0 &&
		 (ndsr & NFC_NDSR_WRCMDREQ) == 0);

	if (rc >= 0 && (ndsr & NFC_NDSR_WRCMDREQ) == 0)
		rc = ERROR_NAND_OPERATION_TIMEOUT;

	if (rc >= 0 && 1)
		rc = target_write_u32(target, NFC_NDSR,
				      NFC_NDSR_WRCMDREQ);

	for (i = 0; i <= 3 && rc >= 0; ++i) {
		printf("%08x ", ctrl->ndcb[i]);
		rc = target_write_u32(target, NFC_NDCB0 + 0*i,
				      ctrl->ndcb[i]);
	}
	printf("\n");

	for (i = 0; i <= 3 && rc >= 0; ++i) {
		uint32_t	reg;
		target_read_u32(target, NFC_NDCB0 + 4*i, &reg);
		printf("%08x ", reg);
	}

	printf("\n");

out:
	LOG_INFO("%s: rc=%d", __func__, rc);
	return rc;
}

static int pxa168_nand_read_data(struct pxa168_nand_controller *ctrl,
				 void *buf, size_t cnt)
{
	struct target			*target = ctrl->target;
	uint32_t			ndsr;
	unsigned int			to = 10;
	size_t				i;
	int				rc;
	uint32_t			*buf32 = buf;

	do {
		rc = target_read_u32(target, NFC_NDSR, &ndsr);
		--to;
	} while (rc >= 0 && to > 0 &&
		 (ndsr & NFC_NDSR_RDDREQ) == 0);

	if (rc >= 0 && (ndsr & NFC_NDSR_RDDREQ) == 0)
		rc = ERROR_NAND_OPERATION_TIMEOUT;

	if (rc >= 0 && 0)
		rc = target_write_u32(target, NFC_NDSR,
				      NFC_NDSR_RDDREQ);

	for (i = 0; i < cnt && rc >= 0; i += 4, ++buf32) {
		rc = target_read_u32(target, NFC_NDDB, buf32);

		printf("%08x ", *buf32);
	}

	if (rc >= 0)
		rc = target_read_u32(target, NFC_NDSR, &ndsr);

	LOG_INFO("%s(%p,%p,%zu) -> %d/%08x", __func__, ctrl, buf, cnt, rc, ndsr);

	return rc;
}

static int pxa168_nand_write_data(struct pxa168_nand_controller *ctrl,
				  void const *buf, size_t cnt)
{
	struct target			*target = ctrl->target;
	uint32_t			ndsr;
	unsigned int			to = 10;
	size_t				i;
	int				rc;
	uint32_t const			*buf32 = buf;

	do {
		rc = target_read_u32(target, NFC_NDSR, &ndsr);
		--to;
	} while (rc >= 0 && to > 0 &&
		 (ndsr & NFC_NDSR_WRDREQ) == 0);

	if (rc >= 0 && (ndsr & NFC_NDSR_WRDREQ) == 0)
		rc = ERROR_NAND_OPERATION_TIMEOUT;

	if (rc >= 0 && 0)
		rc = target_write_u32(target, NFC_NDSR,
				      NFC_NDSR_WRDREQ);

	for (i = 0; i < cnt && rc >= 0; i += 4, ++buf32) {
		rc = target_write_u32(target, NFC_NDDB, *buf32);

		printf("%08x ", *buf32);
	}

	if (rc >= 0)
		rc = target_read_u32(target, NFC_NDSR, &ndsr);

	LOG_INFO("%s(%p,%p,%zu) -> %d/%08x", __func__, ctrl, buf, cnt, rc, ndsr);

	return rc;
}

static int pxa168_nand_command(struct nand_device *nand, uint8_t command)
{
	struct pxa168_nand_controller	*ctrl = nand->controller_priv;
	struct target			*target = ctrl->target;
	int				rc = ERROR_OK;

	switch (command) {
	case NAND_CMD_READID:
		ctrl->addr_cnt = 1;
		ctrl->addr_idx = 0;
		ctrl->read_cnt = (ctrl->ndcr >> 16) & 0x7;
		ctrl->read_idx = 0;
		ctrl->ndcb[0]  = (NFC_NDCB0_CMD1(command) |
				  NFC_NDCB0_ADDR_CYC(1) |
				  NFC_NDCB0_CMD_TYPE_READID);
		ctrl->ndcb[1]  = 0u;
		ctrl->ndcb[2]  = 0u;
		ctrl->ndcb[3]  = 0u;

		if (rc >= 0)
			rc = target_write_u32(target, NFC_NDECCCTRL,
					      ctrl->ndeccctrl &
					      ~NFC_NDECCCTRL_BCH_EN);

		break;

	case NAND_CMD_READ0:
		ctrl->addr_cnt = nand->address_cycles;
		ctrl->addr_idx = 0;
		ctrl->read_cnt = nand->page_size;
		ctrl->read_idx = 0;
		ctrl->ndcb[0]  = (NFC_NDCB0_CMD1(command + 0x0) |
				  NFC_NDCB0_CMD2(NAND_CMD_READSTART) |
				  NFC_NDCB0_ADDR_CYC(nand->address_cycles) |
				  NFC_NDCB0_DBC |
				  NFC_NDCB0_CMD_TYPE_READ |
				  NFC_NDCB0_CMD_XTYPE(0));
		ctrl->ndcb[1]  = 0u;
		ctrl->ndcb[2]  = 0u;
		ctrl->ndcb[3]  = 0u;

		ctrl->ndcr |= NFC_NDCR_ECC_EN | NFC_NDCR_SPARE_EN;

		if (rc >= 0 && 0)
			rc = target_write_u32(target, NFC_NDECCCTRL,
					      ctrl->ndeccctrl |
					      NFC_NDECCCTRL_BCH_EN);
		break;

	case NAND_CMD_RESET:
		ctrl->ndcb[0] = (NFC_NDCB0_CMD1(NAND_CMD_RESET) |
				 NFC_NDCB0_CMD_TYPE_RESET);
		ctrl->ndcb[1] = 0u;
		ctrl->ndcb[2] = 0u;
		ctrl->ndcb[3] = 0u;
		ctrl->ndcb[4] = 0u;
		break;

	case NAND_CMD_ERASE1:
		ctrl->addr_cnt = nand->address_cycles - 2;
		ctrl->addr_idx = 0;
		ctrl->read_cnt = 1;
		ctrl->read_idx = 0;
		ctrl->ndcb[0]  = (NFC_NDCB0_CMD1(0x60) |
				  NFC_NDCB0_CMD2(0xd0) |
				  NFC_NDCB0_ADDR_CYC(nand->address_cycles - 2) |
				  NFC_NDCB0_DBC |
				  0*NFC_NDCB0_AUTO_RS |
				  NFC_NDCB0_CMD_TYPE_ERASE |
				  NFC_NDCB0_CMD_XTYPE(0));
		ctrl->ndcb[1]  = 0u;
		ctrl->ndcb[2]  = 0u;
		ctrl->ndcb[3]  = 0u;

		ctrl->ndcr |= NFC_NDCR_ECC_EN | NFC_NDCR_SPARE_EN;
		break;

	case NAND_CMD_ERASE2:
		break;

	case NAND_CMD_READSTART:
		break;			/* noop */

	default:
		LOG_ERROR("NAND command %02x not supported", command);
		rc = ERROR_NAND_OPERATION_NOT_SUPPORTED;
		goto out;
	}

	if (rc >= 0)
		rc = ERROR_OK;

out:
	LOG_INFO("%s(%p, %02x) -> %d", __func__, nand, command, rc);
	return rc;
}

static int pxa168_nand_address(struct nand_device *nand, uint8_t address)
{
	struct pxa168_nand_controller	*ctrl = nand->controller_priv;
	int				rc = 0;

	switch (ctrl->addr_idx) {
	case 0:
	case 1:
	case 2:
	case 3:
		ctrl->ndcb[1] |= (address << (ctrl->addr_idx * 8));
		break;
	case 4:
		ctrl->ndcb[2] |= (address << 0);
		break;
	case 5:
	case 6:
		ctrl->ndcb[3] |= (address << ((ctrl->addr_idx - 5)*8 + 16));
		break;
	}

	++ctrl->addr_idx;
	if (ctrl->addr_idx == ctrl->addr_cnt)
		rc = pxa168_nand_addr_complete(ctrl);

	if (rc >= 0)
		rc = ERROR_OK;

	LOG_INFO("%s(%02x) -> %d", __func__, address, rc);
	return rc;
}

static int pxa168_nand_read(struct nand_device *nand, void *data)
{
	struct pxa168_nand_controller	*ctrl = nand->controller_priv;
	int				rc = ERROR_OK;

	if (ctrl->read_idx + nand->bus_width/8 > ctrl->read_cnt)
		rc = ERROR_NAND_NO_BUFFER;
	else if (ctrl->read_idx == 0u)
		rc = pxa168_nand_read_data(ctrl,
					   ctrl->read_buf,
					   ctrl->read_cnt);

	if (rc >= 0) {
		memcpy(data, &ctrl->read_buf[ctrl->read_idx],
		       nand->bus_width/8);
		ctrl->read_idx += nand->bus_width/8;
	}

	LOG_INFO("%s(%p, %p) -> %d", __func__, nand, data, rc);
	return rc;
}

static int pxa168_nand_read_block_data(struct nand_device *nand,
				       uint8_t *data, int size)
{
	struct pxa168_nand_controller	*ctrl = nand->controller_priv;
	int				rc = ERROR_NAND_NO_BUFFER;

	if (ctrl->read_idx + size != ctrl->read_cnt)
		rc = ERROR_NAND_NO_BUFFER;
	else if (ctrl->read_idx == 0u)
		rc = pxa168_nand_read_data(ctrl, data, size);

	pxa168_nand_ready(nand, 1);

	ctrl->read_idx += size;

	LOG_INFO("%s(%p,%p,%d) -> %d / %04x", __func__, nand, data, size, rc,
		 *(uint32_t *)&data[0]);
	return rc;
}

static int pxa168_nand_write(struct nand_device *nand, uint16_t data)
{
	int		rc = ERROR_OK;

	LOG_INFO("%s(%p,%02x) -> %d", __func__, nand, data, rc);
	return ERROR_OK;
}


static int pxa168_nand_reset(struct nand_device *nand)
{
	struct pxa168_nand_controller	*ctrl = nand->controller_priv;
	int		rc;

	rc = pxa168_nand_command(nand, NAND_CMD_RESET);
	if (rc >= 0)
		rc = pxa168_nand_addr_complete(ctrl);

	LOG_INFO("%s(%p) -> %d", __func__, nand, rc);
	return rc;
}

static int pxa168_nand_controller_ready(struct nand_device *nand, int timeout)
{
	int		rc = ERROR_OK;

	LOG_INFO("%s(%p, %d) -> %d", __func__, nand, timeout, rc);
	return true;
}

NAND_DEVICE_COMMAND_HANDLER(pxa168_nand_device_command)
{
	struct pxa168_nand_controller	*ctrl;

	if (CMD_ARGC != 3) {
		LOG_ERROR("arguments must be: <target_id> <ndcr>\n");
		return ERROR_NAND_DEVICE_INVALID;
	}

	ctrl = calloc(1, sizeof *ctrl);
	if (ctrl == NULL) {
		LOG_ERROR("no memory for nand controller\n");
		return ERROR_NAND_DEVICE_INVALID;
	}

	nand->controller_priv = ctrl;
	ctrl->target = get_target(CMD_ARGV[1]);
	if (ctrl->target == NULL) {
		LOG_ERROR("target '%s' not defined", CMD_ARGV[1]);
		free(ctrl);
		return ERROR_NAND_DEVICE_INVALID;
	}

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], ctrl->ndcr);

	ctrl->ndcr &= ~((1u<<31) |
			(1u<<30) |
			(1u<<29) |		/* DMA_EN */
			(1u<<28) |		/* ND_RUN */
			(1u<<23) |		/* SEQ_DIS */
			(1u<<22) |		/* reserved */
			(1u<<20) |		/* CLR_PG_CNT */
			(1u<<19));
	ctrl->ndcr |= ((1u<<11) |		/* RDYM */
		       (1u<<10) |		/* CS0_PAGEDM */
		       (1u<< 9) |		/* CS1_PAGEDM */
		       (1u<< 8) |		/* CS0_CMDDM */
		       (1u<< 7) |		/* CS1_CMDDM */
		       (1u<< 6) |		/* CS0_BBDM */
		       (1u<< 5) |		/* CS1_BBDM */
		       (1u<< 4) |		/* UNCERRM */
		       (1u<< 3) |		/* CORERRM */
		       (1u<< 2) |		/* WRDREQM */
		       (1u<< 1) |		/* RDDREQM */
		       (1u<< 0));		/* WRCMDREQM */

	//ctrl->ndcr |= (0<<23) | (1<<15) | (1<<31);

	return ERROR_OK;
}

static int pxa168_nand_init(struct nand_device *nand)
{
	struct pxa168_nand_controller	*ctrl = nand->controller_priv;
	struct target			*target = ctrl->target;
	int				rc = 0;

//	rc = target_read_u32(target, NFC_NDCR, &ctrl->ndcr);
#if 1
	if (rc >= 0)
		rc = target_read_u32(target, NFC_NDECCCTRL, &ctrl->ndeccctrl);

	if (rc < 0)
		goto err;
#endif

	nand->bus_width = (ctrl->ndcr & NFC_NDCR_DWIDTH_C) ? 16 : 8;

	if (0) {
		size_t		i;

		for (i = 0; i <= 0x7c; i += 4) {
			uint32_t	tmp;

			rc = target_read_u32(target, NFC_NDCR + i, &tmp);
			printf("%02zx -> %08x%s\n", i, tmp, rc < 0 ? "E" :"");
		}
	}

	rc = target_write_u32(target, NFC_NDCR, ctrl->ndcr);
	if (rc < 0)
		goto err;


	LOG_INFO("%s -> %08x/%08x", __func__, ctrl->ndcr, ctrl->ndeccctrl);
	return ERROR_OK;

err:
	return rc;
}

static int pxa168_nand_read_page(struct nand_device *nand,
				 uint32_t page,
				 uint8_t *data, uint32_t data_size,
				 uint8_t *oob, uint32_t oob_size)
{
	struct pxa168_nand_controller	*ctrl = nand->controller_priv;
	int				rc;

	if (data != NULL && data_size != (size_t)nand->page_size)
		return ERROR_NAND_DEVICE_INVALID;

	if (data == NULL)
		data = ctrl->read_buf;

	ctrl->ndcb[0] = (NFC_NDCB0_CMD1(NAND_CMD_READ0) |
			 NFC_NDCB0_CMD2(NAND_CMD_READSTART) |
			 NFC_NDCB0_ADDR_CYC(nand->address_cycles) |
			 NFC_NDCB0_DBC |
			 NFC_NDCB0_CMD_TYPE_READ |
			 NFC_NDCB0_CMD_XTYPE(0));
	ctrl->ndcb[1] = (page & 0xffff) << 16;
	ctrl->ndcb[2] = (page >> 16) & 0xff;
	ctrl->ndcb[3] = 0;

	ctrl->ndcr   |= NFC_NDCR_ECC_EN | NFC_NDCR_SPARE_EN;

	rc = pxa168_nand_addr_complete(ctrl);
	if (rc >= 0)
		rc = pxa168_nand_read_data(ctrl, data, nand->page_size);

	if (rc >= 0) {
		rc = pxa168_nand_read_data(ctrl, ctrl->read_buf, 40);

		if (oob)
			memcpy(oob, ctrl->read_buf, oob_size);
	}

	LOG_INFO("%s(%p,%d,%p,%u,%p,%u) -> %d", __func__,
		 nand, page, data, data_size, oob, oob_size, rc);
	return rc;
}

static int pxa168_nand_write_page(struct nand_device *nand,
				  uint32_t page,
				  uint8_t *data, uint32_t data_size,
				  uint8_t *oob, uint32_t oob_size)
{
	struct pxa168_nand_controller	*ctrl = nand->controller_priv;
	int				rc;

	if (data_size != (size_t)nand->page_size)
		return ERROR_NAND_DEVICE_INVALID;

	if (oob != NULL)
		return ERROR_NAND_OPERATION_NOT_SUPPORTED;

	ctrl->ndcb[0] = (NFC_NDCB0_CMD1(NAND_CMD_SEQIN) |
			 NFC_NDCB0_CMD2(NAND_CMD_PAGEPROG) |
			 NFC_NDCB0_DBC |
			 NFC_NDCB0_AUTO_RS |
			 NFC_NDCB0_ADDR_CYC(nand->address_cycles) |
			 NFC_NDCB0_CMD_TYPE_WRITE);
	ctrl->ndcb[1] = (page & 0xffff) << 16;
	ctrl->ndcb[2] = (page >> 16) & 0xff;
	ctrl->ndcb[3] = 0;

	ctrl->ndcr   |= NFC_NDCR_ECC_EN | NFC_NDCR_SPARE_EN;

	rc = pxa168_nand_addr_complete(ctrl);

	if (rc >= 0)
		rc = pxa168_nand_write_data(ctrl, data, data_size);

	if (oob == NULL) {
		oob = ctrl->read_buf;
		oob_size = 40;
		memset(oob, 0xff, oob_size);
	}

	if (rc >= 0)
		rc = pxa168_nand_write_data(ctrl, oob, oob_size);

	LOG_INFO("%s(%p,%d,%p,%u,%p,%u) -> %d", __func__,
		 nand, page, data, data_size, oob, oob_size, rc);

	return rc;
}


struct nand_flash_controller pxa168_nand_controller =
{
	.name			= "pxa168",
	.nand_device_command	= &pxa168_nand_device_command,
	.init			= &pxa168_nand_init,
	.reset			= &pxa168_nand_reset,
	.command		= &pxa168_nand_command,
	.address		= &pxa168_nand_address,
	.read_data		= &pxa168_nand_read,
	.nand_ready		= &pxa168_nand_ready,
	.write_data		= &pxa168_nand_write,
	.write_page		= &pxa168_nand_write_page,
	.read_page		= &pxa168_nand_read_page,
	.read_block_data	= &pxa168_nand_read_block_data,
	.controller_ready	= &pxa168_nand_controller_ready,
};
