/*
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <asm/cacheflush.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/types.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/rockchip/grf.h>
#include <linux/rockchip/pmu.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "mpp_dev_common.h"
#include "mpp_dev_rkvdec.h"
#include "mpp_service.h"

#define to_rkvdec_ctx(ctx)		\
		container_of(ctx, struct rkvdec_ctx, ictx)
#define to_rkvdec_dev(dev)		\
		container_of(dev, struct rockchip_rkvdec_dev, idev)

#define RKVDEC_REG_DEC_INT_EN		0x004
#define		RKVDEC_WR_DDR_ALIGN_EN		BIT(23)
#define		RKVDEC_FORCE_SOFT_RESET_VALID	BIT(21)
#define		RKVDEC_SOFTWARE_RESET_EN	BIT(20)
#define		RKVDEC_INT_COLMV_REF_ERROR	BIT(17)
#define		RKVDEC_INT_BUS_EMPTY		BIT(16)
#define		RKVDEC_INT_TIMEOUT		BIT(15)
#define		RKVDEC_INT_STRM_ERROR		BIT(14)
#define		RKVDEC_INT_BUS_ERROR		BIT(13)
#define		RKVDEC_DEC_INT_RAW		BIT(9)
#define		RKVDEC_DEC_INT			BIT(8)
#define		RKVDEC_DEC_TIMEOUT_EN		BIT(5)
#define		RKVDEC_DEC_IRQ_DIS		BIT(4)
#define		RKVDEC_CLOCK_GATE_EN		BIT(1)
#define		RKVDEC_DEC_START		BIT(0)

#define RKVDEC_REG_SYS_CTRL		0x008
#define		RKVDEC_GET_FORMAT(x)		(((x) >> 20) & 0x3)
#define		RKVDEC_FMT_H265D		0
#define		RKVDEC_FMT_H264D		1
#define		RKVDEC_FMT_VP9D			2

#define RKVDEC_REG_CACHE_ENABLE(i)	(0x41c + ((i) * 0x40))
#define		RKVDEC_CACHE_LINE_SIZE_64_BYTES		BIT(4)
#define		RKVDEC_CACHE_PERMIT_READ_ALLOCATE	BIT(1)
#define		RKVDEC_CACHE_PERMIT_CACHEABLE_ACCESS	BIT(0)

#define MPP_ALIGN_SIZE	0x1000

#define MHZ		(1000 * 1000)
#define DEF_ACLK	400
#define DEF_CORE	250
#define DEF_CABAC	300

/*
 * file handle translate information
 */
static const char trans_tbl_h264d[] = {
	4, 6, 7, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22,
	23, 24, 41, 42, 43, 48, 75
};

static const char trans_tbl_h265d[] = {
	4, 6, 7, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22,
	23, 24, 42, 43
};

static const char trans_tbl_vp9d[] = {
	4, 6, 7, 11, 12, 13, 14, 15, 16
};

static struct mpp_trans_info trans_rkvdec[3] = {
	[RKVDEC_FMT_H265D] = {
		.count = sizeof(trans_tbl_h265d),
		.table = trans_tbl_h265d,
	},
	[RKVDEC_FMT_H264D] = {
		.count = sizeof(trans_tbl_h264d),
		.table = trans_tbl_h264d,
	},
	[RKVDEC_FMT_VP9D] = {
		.count = sizeof(trans_tbl_vp9d),
		.table = trans_tbl_vp9d,
	},
};

static dma_addr_t rkvdec_fd_to_iova(struct mpp_ctx *ctx, int fd)
{
	struct mpp_mem_region *mem_region = NULL;

	mem_region = mpp_fd_to_mem_region(ctx->session->dma, fd);
	if (IS_ERR(mem_region))
		return PTR_ERR(mem_region);

	mpp_debug(DEBUG_IOMMU, "fd: %3d %pad\n",
		  fd, &mem_region->iova);

	INIT_LIST_HEAD(&mem_region->reg_lnk);
	list_add_tail(&mem_region->reg_lnk, &ctx->mem_region_list);

	return mem_region->iova;
}

/*
 * NOTE: rkvdec/rkhevc put scaling list address in pps buffer hardware will read
 * it by pps id in video stream data.
 *
 * So we need to translate the address in iommu case. The address data is also
 * 10bit fd + 22bit offset mode.
 * Because userspace decoder do not give the pps id in the register file sets
 * kernel driver need to translate each scaling list address in pps buffer which
 * means 256 pps for H.264, 64 pps for H.265.
 *
 * In order to optimize the performance kernel driver ask userspace decoder to
 * set all scaling list address in pps buffer to the same one which will be used
 * on current decoding task. Then kernel driver can only translate the first
 * address then copy it all pps buffer.
 */
static int fill_scaling_list_pps(struct mpp_ctx *ctx, int fd, int offset,
				 int count, int pps_info_size,
				 int sub_addr_offset)
{
	struct device *dev = NULL;
	struct dma_buf *dmabuf = NULL;
	void *vaddr = NULL;
	u8 *pps = NULL;
	u32 base = sub_addr_offset;
	u32 scaling_fd = 0;
	u32 scaling_offset;
	int ret = 0;

	dev = ctx->session->mpp->dev;
	if (!dev)
		return -EINVAL;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR_OR_NULL(dmabuf)) {
		dev_err(dev, "invliad pps buffer\n");
		return -ENOENT;
	}

	ret = dma_buf_begin_cpu_access(dmabuf, 0, dmabuf->size,
				       DMA_FROM_DEVICE);
	if (ret) {
		dev_err(dev, "can't access the pps buffer\n");
		return ret;
	}

	vaddr = dma_buf_vmap(dmabuf);
	if (!vaddr) {
		dev_err(dev, "can't access the pps buffer\n");
		return -EIO;
	}
	pps = vaddr + offset;

	memcpy(&scaling_offset, pps + base, sizeof(scaling_offset));
	scaling_offset = le32_to_cpu(scaling_offset);

	scaling_fd = scaling_offset & 0x3ff;
	scaling_offset = scaling_offset >> 10;

	if (scaling_fd > 0) {
		int i = 0;
		dma_addr_t tmp = rkvdec_fd_to_iova(ctx, scaling_fd);
		if (IS_ERR_VALUE(tmp))
			return tmp;
		tmp += scaling_offset;
		tmp = cpu_to_le32(tmp);

		/* Fill the scaling list address in each pps entries */
		for (i = 0; i < count; i++, base += pps_info_size)
			memcpy(pps + base, &tmp, sizeof(tmp));
	}

	dma_buf_vunmap(dmabuf, vaddr);
	dma_buf_end_cpu_access(dmabuf, 0, dmabuf->size, DMA_FROM_DEVICE);
	dma_buf_put(dmabuf);

	return 0;
}

#if 0
static int rkvdec_dump(struct rockchip_mpp_dev *mpp, u32 idx)
{
	struct rockchip_rkvdec_dev *dec = to_rkvdec_dev(mpp);
	struct device *dev = dec->idev.dev;
	struct mpp_ctx *ictx, *n;
	u32 *base;

	ictx = mpp_srv_get_current_ctx(mpp->srv);
	if (!IS_ERR_OR_NULL(ictx)) {
		if ((to_rkvdec_ctx(ictx))->idx == idx) {
			dev_err(dev, "stuck in current ctx\n");
			goto ret;
		}
	}

	list_for_each_entry_safe(ictx, n, &dec->idev.srv->running,
				 status_link) {
		if ((to_rkvdec_ctx(ictx))->idx == idx) {
			dev_err(dev, "stuck in pending ctx\n");
			goto ret;
		}
	}
ret:
	base = (u32 *)dec->lkt_cpu_addr + (idx % 256) * LINK_TABLE_LEN;

	mpp_dump_reg_mem(base, 128);

	return 0;
}
#endif

static struct mpp_ctx *rockchip_mpp_rkvdec_init(struct rockchip_mpp_dev *mpp,
						struct mpp_session *session,
						void __user *src, u32 size)
{
	struct rkvdec_ctx *ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	u32 reg_len;
	u32 extinf_len;
	u32 fmt = 0;
	u32 dwsize = size / sizeof(u32);
	int pps_fd;
	u32 pps_offset;
	int ret = 0;

	mpp_debug_enter();

	if (!ctx)
		return NULL;

	mpp_dev_common_ctx_init(session, &ctx->ictx);

	reg_len = dwsize > ROCKCHIP_RKVDEC_REG_NUM ?
		ROCKCHIP_RKVDEC_REG_NUM : dwsize;
	extinf_len = dwsize > reg_len ? (dwsize - reg_len) * 4 : 0;

	if (copy_from_user(ctx->reg, src, reg_len * 4)) {
		mpp_err("error: copy_from_user failed in reg_init\n");
		kfree(ctx);
		return NULL;
	}

	if (extinf_len > 0) {
		u32 ext_cpy = min_t(size_t, extinf_len, sizeof(ctx->ext_inf));

		if (copy_from_user(&ctx->ext_inf, (u8 *)src + reg_len,
				   ext_cpy)) {
			mpp_err("copy_from_user failed when extra info\n");
			kfree(ctx);
			return NULL;
		}
	}

	fmt = RKVDEC_GET_FORMAT(ctx->reg[RKVDEC_REG_SYS_CTRL / 4]);

	/*
	 * special offset scale case
	 *
	 * This translation is for fd + offset translation.
	 * One register has 32bits. We need to transfer both buffer file
	 * handle and the start address offset so we packet file handle
	 * and offset together using below format.
	 *
	 *  0~9  bit for buffer file handle range 0 ~ 1023
	 * 10~31 bit for offset range 0 ~ 4M
	 *
	 * But on 4K case the offset can be larger the 4M
	 * So on VP9 4K decoder colmv base we scale the offset by 16
	 */
	if (fmt == RKVDEC_FMT_VP9D) {
		u32 offset = ctx->reg[52] >> 10 << 4;
		int fd = ctx->reg[52] & 0x3ff;
		dma_addr_t iova = rkvdec_fd_to_iova(&ctx->ictx, fd);

		ctx->reg[52] = iova + offset;
	}

	pps_fd = ctx->reg[42] & 0x3ff;
	pps_offset = ctx->reg[42] >> 10;

	if (pps_fd > 0) {
		int pps_info_offset;
		int pps_info_count;
		int pps_info_size;
		int scaling_list_addr_offset;

		switch (fmt) {
		case RKVDEC_FMT_H264D: {
			pps_info_offset = pps_offset;
			pps_info_count = 256;
			pps_info_size = 32;
			scaling_list_addr_offset = 23;
		} break;
		case RKVDEC_FMT_H265D: {
			pps_info_offset = 0;
			pps_info_count = 64;
			pps_info_size = 80;
			scaling_list_addr_offset = 74;
		} break;
		default: {
			pps_info_offset = 0;
			pps_info_count = 0;
			pps_info_size = 0;
			scaling_list_addr_offset = 0;
		} break;
		}

		mpp_debug(DEBUG_PPS_FILL,
			  "scaling list filling parameter:\n");
		mpp_debug(DEBUG_PPS_FILL,
			  "pps_info_offset %d\n", pps_info_offset);
		mpp_debug(DEBUG_PPS_FILL,
			  "pps_info_count  %d\n", pps_info_count);
		mpp_debug(DEBUG_PPS_FILL,
			  "pps_info_size   %d\n", pps_info_size);
		mpp_debug(DEBUG_PPS_FILL,
			  "scaling_list_addr_offset %d\n",
			  scaling_list_addr_offset);

		if (pps_info_count) {
			ret = fill_scaling_list_pps(&ctx->ictx, pps_fd,
						    pps_info_offset,
						    pps_info_count,
						    pps_info_size,
						    scaling_list_addr_offset);
			if (ret) {
				mpp_err("fill pps failed\n");
				return NULL;
			}
		}
	}

	if (mpp_reg_address_translate(mpp, ctx->reg, &ctx->ictx, fmt) < 0) {
		mpp_err("error: translate reg address failed.\n");

		if (unlikely(mpp_dev_debug & DEBUG_DUMP_ERR_REG))
			mpp_dump_reg_mem(ctx->reg, ROCKCHIP_RKVDEC_REG_NUM);

		mpp_dev_common_ctx_deinit(mpp, &ctx->ictx);
		kfree(ctx);

		return NULL;
	}

	ctx->strm_base = ctx->reg[4];

	mpp_debug(DEBUG_SET_REG, "extra info cnt %u, magic %08x",
		  ctx->ext_inf.cnt, ctx->ext_inf.magic);

	mpp_translate_extra_info(&ctx->ictx, &ctx->ext_inf, ctx->reg);

	mpp_debug_leave();

	return &ctx->ictx;
}

static int rockchip_mpp_rkvdec_run(struct rockchip_mpp_dev *mpp)
{
	struct rkvdec_ctx *ctx =
		to_rkvdec_ctx(mpp_srv_get_last_running_ctx(mpp->srv));
	struct rockchip_rkvdec_dev *dec = to_rkvdec_dev(mpp);
	int i;
	u32 reg = 0;

	mpp_debug_enter();

	/*
	 * hardware bug workaround, because the write ddr align optimize need
	 * aclk and core clock using the same parent clock. so when optimization
	 * enable, we need to reset the clocks.
	 */
#if 0
	if (ctx->reg[RKVDEC_REG_DEC_INT_EN / 4] & RKVDEC_WR_DDR_ALIGN_EN) {
		if (atomic_read(&dec->cur_core) != 250) {
			atomic_set(&dec->cur_core, 250);
			mpp_debug(DEBUG_CLOCK, "set core clock to 250 MHz\n");
			clk_set_rate(dec->core, 250 * MHZ);
		}
	} else {
		if (atomic_read(&dec->cur_core) != 200) {
			atomic_set(&dec->cur_core, 200);
			mpp_debug(DEBUG_CLOCK, "set core clock to 200 MHz\n");
			clk_set_rate(dec->core, 200 * MHZ);
		}
		if (atomic_read(&dec->cur_aclk) != 300) {
			atomic_set(&dec->cur_aclk, 300);
			mpp_debug(DEBUG_CLOCK, "set core clock to 300 MHz\n");
			clk_set_rate(dec->aclk, 300 * MHZ);
		}
		if (atomic_read(&dec->cur_caback) != 200) {
			atomic_set(&dec->cur_caback, 200);
			mpp_debug(DEBUG_CLOCK, "set core clock to 200 MHz\n");
			clk_set_rate(dec->cabac, 200 * MHZ);
		}
	}
#endif
	switch (dec->state) {
	case RKVDEC_STATE_NORMAL:
		dec->current_task = ctx;

		reg = RKVDEC_CACHE_PERMIT_CACHEABLE_ACCESS
			| RKVDEC_CACHE_PERMIT_READ_ALLOCATE;
		if (!(mpp_dev_debug & DEBUG_CACHE_32B))
			reg |= RKVDEC_CACHE_LINE_SIZE_64_BYTES;

		for (i = 0; i < 2; ++i)
			mpp_write_relaxed(mpp, reg, RKVDEC_REG_CACHE_ENABLE(i));

		for (i = 2; i < ROCKCHIP_RKVDEC_REG_NUM; ++i)
			mpp_write_relaxed(mpp, ctx->reg[i], i * 4);

		mpp_write(mpp, ctx->reg[RKVDEC_REG_DEC_INT_EN / 4]
			  | RKVDEC_DEC_START
			  | RKVDEC_CLOCK_GATE_EN,
			  RKVDEC_REG_DEC_INT_EN);
		break;
	default:
		break;
	}

	mpp_debug_leave();

	return 0;
}

static int rockchip_mpp_rkvdec_done(struct rockchip_mpp_dev *mpp)
{
	struct rockchip_rkvdec_dev *dec = to_rkvdec_dev(mpp);
	int i;

	mpp_debug_enter();

	switch (dec->state) {
	case RKVDEC_STATE_NORMAL: {
		struct mpp_ctx *ictx =
			mpp_srv_get_current_ctx(mpp->srv);
		struct rkvdec_ctx *ctx = to_rkvdec_ctx(ictx);

		if (IS_ERR_OR_NULL(ictx)) {
			mpp_err("Invaidate context to save result\n");
			return -1;
		}

		for (i = 0; i < ROCKCHIP_RKVDEC_REG_NUM; i++)
			ctx->reg[i] = mpp_read(mpp, i * 4);
		ctx->reg[RKVDEC_REG_DEC_INT_EN / 4] = ctx->irq_status;
	} break;
	default:
		break;
	}

	mpp_debug_leave();

	return 0;
}

static int rockchip_mpp_rkvdec_irq(struct rockchip_mpp_dev *mpp)
{
	struct rockchip_rkvdec_dev *dec = to_rkvdec_dev(mpp);
	struct rkvdec_ctx *ctx = NULL;
	u32 err_mask;

	mpp_debug_enter();

	ctx = (struct rkvdec_ctx *)dec->current_task;
	if (!ctx)
		return -EINVAL;

	ctx->irq_status = mpp_read(mpp, RKVDEC_REG_DEC_INT_EN);

	if (ctx->irq_status & RKVDEC_DEC_INT_RAW &&
	    dec->state == RKVDEC_STATE_NORMAL) {
		mpp_debug(DEBUG_IRQ_STATUS, "irq_status: %08x\n",
			  ctx->irq_status);
		mpp_write(mpp, 0, RKVDEC_REG_DEC_INT_EN);

		err_mask = RKVDEC_INT_BUS_EMPTY
			| RKVDEC_INT_BUS_ERROR
			| RKVDEC_INT_COLMV_REF_ERROR
			| RKVDEC_INT_STRM_ERROR
			| RKVDEC_INT_TIMEOUT;

		if (err_mask & ctx->irq_status)
			atomic_set(&mpp->reset_request, 1);

		mpp_debug_leave();
		return 0;
	}

	__WARN();
	mpp_debug_leave();

	return -1;
}

static int rockchip_mpp_rkvdec_result(struct rockchip_mpp_dev *mpp,
				      struct mpp_ctx *ictx, u32 __user *dst)
{
	struct rkvdec_ctx *ctx = to_rkvdec_ctx(ictx);

	if (copy_to_user(dst, ctx->reg, ROCKCHIP_RKVDEC_REG_NUM * 4)) {
		mpp_err("copy_to_user failed\n");
		return -1;
	}

	return 0;
}

static int rockchip_mpp_rkvdec_prepare(struct rockchip_mpp_dev *mpp)
{
	struct rockchip_rkvdec_dev *dec = to_rkvdec_dev(mpp);
	struct mpp_ctx *iready;
	struct rkvdec_ctx *ready;
	struct rkvdec_ctx *last;
	bool ready_wr_align;
	bool pend_wr_align = false;

	mpp_debug_enter();

	/* service not running, run the service */
	if (!mpp_srv_is_running(mpp->srv))
		return 0;

	/* not in link table mode and service is running, don't change state */
	if (dec->state == RKVDEC_STATE_NORMAL)
		return -1;

	iready = mpp_srv_get_pending_ctx(mpp->srv);
	ready = to_rkvdec_ctx(iready);

	/*
	 * Judge if wr align is consistence between too adjcent frame, if wr
	 * align flag is difference. we need to do the clock switch, so we
	 * break the link table.
	 */
	ready_wr_align = !!(ready->reg[RKVDEC_REG_DEC_INT_EN / 4] &
		RKVDEC_WR_DDR_ALIGN_EN);

	last = to_rkvdec_ctx(mpp_srv_get_last_running_ctx(mpp->srv));

	pend_wr_align = !!(last->reg[RKVDEC_REG_DEC_INT_EN / 4] &
		RKVDEC_WR_DDR_ALIGN_EN);

	/*
	 * wr align not match, link table mode should be broken,
	 * rk3228h hw feature.
	 */
	if (ready_wr_align != pend_wr_align) {
		mpp_debug(DEBUG_CLOCK, "break link table for wr_align\n");
		//return -1;
	}

	mpp_debug_leave();

	return 0;
}

static int rockchip_mpp_rkvdec_reset_init(struct rockchip_mpp_dev *mpp)
{
	struct rockchip_rkvdec_dev *dec = to_rkvdec_dev(mpp);

	dec->rst_a = devm_reset_control_get(mpp->dev, "video_a");
	dec->rst_h = devm_reset_control_get(mpp->dev, "video_h");
	dec->rst_niu_a = devm_reset_control_get(mpp->dev, "niu_a");
	dec->rst_niu_h = devm_reset_control_get(mpp->dev, "niu_h");
	dec->rst_core = devm_reset_control_get(mpp->dev, "video_core");
	dec->rst_cabac = devm_reset_control_get(mpp->dev, "video_cabac");

	if (IS_ERR_OR_NULL(dec->rst_a)) {
		mpp_err("No aclk reset resource define\n");
		dec->rst_a = NULL;
	}

	if (IS_ERR_OR_NULL(dec->rst_h)) {
		mpp_err("No hclk reset resource define\n");
		dec->rst_h = NULL;
	}

	if (IS_ERR_OR_NULL(dec->rst_niu_a)) {
		mpp_err("No axi niu reset resource define\n");
		dec->rst_niu_a = NULL;
	}

	if (IS_ERR_OR_NULL(dec->rst_niu_h)) {
		mpp_err("No ahb niu reset resource define\n");
		dec->rst_niu_h = NULL;
	}

	if (IS_ERR_OR_NULL(dec->rst_core)) {
		mpp_err("No core reset resource define\n");
		dec->rst_core = NULL;
	}

	if (IS_ERR_OR_NULL(dec->rst_cabac)) {
		mpp_err("No cabac reset resource define\n");
		dec->rst_cabac = NULL;
	}

	return 0;
}

static inline void safe_reset(struct reset_control *rst)
{
	if (rst)
		reset_control_assert(rst);
}

static inline void safe_unreset(struct reset_control *rst)
{
	if (rst)
		reset_control_deassert(rst);
}

static int rockchip_mpp_rkvdec_reset(struct rockchip_mpp_dev *mpp)
{
	struct rockchip_rkvdec_dev *dec = to_rkvdec_dev(mpp);

	if (dec->rst_a && dec->rst_h) {
		unsigned long rate = 0;
		mpp_debug(DEBUG_RESET, "reset in\n");

		rockchip_pmu_idle_request(mpp->dev, true);
		rate = clk_get_rate(dec->aclk);
		clk_set_rate(dec->aclk, 200 * MHZ);

		safe_reset(dec->rst_niu_a);
		safe_reset(dec->rst_niu_h);
		safe_reset(dec->rst_a);
		safe_reset(dec->rst_h);
		safe_reset(dec->rst_core);
		safe_reset(dec->rst_cabac);
		udelay(2);
		safe_unreset(dec->rst_niu_h);
		safe_unreset(dec->rst_niu_a);
		safe_unreset(dec->rst_a);
		safe_unreset(dec->rst_h);
		safe_unreset(dec->rst_core);
		safe_unreset(dec->rst_cabac);

		rockchip_pmu_idle_request(mpp->dev, false);
		clk_set_rate(dec->aclk, rate);

		mpp_debug(DEBUG_RESET, "reset out\n");
	}

	return 0;
}


void rockchip_mpp_rkvdec_free_ctx(struct mpp_ctx *ictx)
{
	struct rkvdec_ctx *ctx = to_rkvdec_ctx(ictx);
	kfree(ctx);
}

struct mpp_dev_ops rkvdec_ops = {
	.init = rockchip_mpp_rkvdec_init,
	.prepare = rockchip_mpp_rkvdec_prepare,
	.run = rockchip_mpp_rkvdec_run,
	.done = rockchip_mpp_rkvdec_done,
	.irq = rockchip_mpp_rkvdec_irq,
	.result = rockchip_mpp_rkvdec_result,
};

static void rockchip_mpp_rkvdec_power_on(struct rockchip_mpp_dev *mpp)
{
	struct rockchip_rkvdec_dev *dec = to_rkvdec_dev(mpp);

	if (dec->aclk)
		clk_prepare_enable(dec->aclk);
	if (dec->hclk)
		clk_prepare_enable(dec->hclk);
	if (dec->core)
		clk_prepare_enable(dec->core);
	if (dec->cabac)
		clk_prepare_enable(dec->cabac);

#if 0
	mpp_write(mpp, RKVDEC_SOFTWARE_RESET_EN, RKVDEC_REG_DEC_INT_EN);
	mpp_write(mpp, 0, RKVDEC_LT_REG_DECODED_NUM);
	mpp_write(mpp, 0, RKVDEC_LT_REG_TOTAL_NUM);
#endif
}

static void rockchip_mpp_rkvdec_power_off(struct rockchip_mpp_dev *mpp)
{
	struct rockchip_rkvdec_dev *dec = to_rkvdec_dev(mpp);

	if (dec->cabac)
		clk_disable_unprepare(dec->cabac);
	if (dec->core)
		clk_disable_unprepare(dec->core);
	if (dec->hclk)
		clk_disable_unprepare(dec->hclk);
	if (dec->aclk)
		clk_disable_unprepare(dec->aclk);
}

static int rockchip_mpp_rkvdec_probe(struct rockchip_mpp_dev *mpp)
{
	struct rockchip_rkvdec_dev *dec = to_rkvdec_dev(mpp);

	dec->idev.ops = &rkvdec_ops;

	dec->aclk = devm_clk_get(mpp->dev, "aclk_vcodec");
	if (IS_ERR_OR_NULL(dec->aclk)) {
		dev_err(mpp->dev, "failed on clk_get aclk\n");
		goto fail;
	}

	dec->hclk = devm_clk_get(mpp->dev, "hclk_vcodec");
	if (IS_ERR_OR_NULL(dec->hclk)) {
		dev_err(mpp->dev, "failed on clk_get hclk\n");
		goto fail;
	}

	dec->core = devm_clk_get(mpp->dev, "clk_core");
	if (IS_ERR_OR_NULL(dec->core)) {
		dev_err(mpp->dev, "failed on clk_get core\n");
		goto fail;
	}

	dec->cabac = devm_clk_get(mpp->dev, "clk_cabac");
	if (IS_ERR_OR_NULL(dec->cabac)) {
		dev_err(mpp->dev, "failed on clk_get cabac\n");
		goto fail;
	}

	rockchip_mpp_rkvdec_reset_init(mpp);

	dec->state = RKVDEC_STATE_NORMAL;

	return 0;

fail:
	return -1;
}

static void rockchip_mpp_rkvdec_remove(struct rockchip_mpp_dev *mpp)
{
}

const struct rockchip_mpp_dev_variant rkvdec_variant = {
	.data_len = sizeof(struct rockchip_rkvdec_dev),
	.reg_len = 128,
	.trans_info = trans_rkvdec,
	.hw_probe = rockchip_mpp_rkvdec_probe,
	.hw_remove = rockchip_mpp_rkvdec_remove,
	.power_on = rockchip_mpp_rkvdec_power_on,
	.power_off = rockchip_mpp_rkvdec_power_off,
	.reset = rockchip_mpp_rkvdec_reset,
	.mmu_dev_dts_name = NULL,
};
