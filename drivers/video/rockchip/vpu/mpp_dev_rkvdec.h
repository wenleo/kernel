/**
 * Copyright (C) 2016 Fuzhou Rockchip Electronics Co., Ltd
 *	   Alpha Lin, alpha.lin@rock-chips.com
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

#ifndef _ROCKCHIP_MPP_DEV_RKVDEC_H_
#define _ROCKCHIP_MPP_DEV_RKVDEC_H_

/* The maxium registers number of all the version */
#define ROCKCHIP_RKVDEC_REG_NUM			109

enum RKVDEC_STATE {
	RKVDEC_STATE_NORMAL,
	RKVDEC_STATE_LT_START,
	RKVDEC_STATE_LT_RUN,
};

struct rockchip_rkvdec_dev {
	struct rockchip_mpp_dev idev;

	struct clk *aclk;
	struct clk *hclk;
	struct clk *core;
	struct clk *cabac;

	struct reset_control *rst_a;
	struct reset_control *rst_h;
	struct reset_control *rst_niu_a;
	struct reset_control *rst_niu_h;
	struct reset_control *rst_core;
	struct reset_control *rst_cabac;

	enum RKVDEC_STATE state;

	void *current_task;
};

struct rkvdec_ctx {
	struct mpp_ctx ictx;

	u32 reg[ROCKCHIP_RKVDEC_REG_NUM];
	u32 idx;
	struct extra_info_for_iommu ext_inf;

	u32 strm_base;
	u32 irq_status;
};

void rockchip_mpp_rkvdec_free_ctx(struct mpp_ctx *ictx);
#endif
