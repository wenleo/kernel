/*
 * Rockchip isp1 driver
 *
 * Copyright (C) 2017 Rockchip Electronics Co., Ltd.
 *
 * This software is available to you under a choice of one of two
 * licenses.  You may choose to be licensed under the terms of the GNU
 * General Public License (GPL) Version 2, available from the file
 * COPYING in the main directory of this source tree, or the
 * OpenIB.org BSD license below:
 *
 *     Redistribution and use in source and binary forms, with or
 *     without modification, are permitted provided that the following
 *     conditions are met:
 *
 *      - Redistributions of source code must retain the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer.
 *
 *      - Redistributions in binary form must reproduce the above
 *        copyright notice, this list of conditions and the following
 *        disclaimer in the documentation and/or other materials
 *        provided with the distribution.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-vmalloc.h>	/* for ISP params */
#include "regs.h"
#include "rkisp1.h"
#include "isp_params.h"

#define RKISP1_ISP_PARAMS_REQ_BUFS_MIN	2
#define RKISP1_ISP_PARAMS_REQ_BUFS_MAX	8

#define BLS_START_H_MAX_IS_VALID(val)	((val) < CIFISP_BLS_START_H_MAX)
#define BLS_STOP_H_MAX_IS_VALID(val)	((val) < CIFISP_BLS_STOP_H_MAX)

#define BLS_START_V_MAX_IS_VALID(val)	((val) < CIFISP_BLS_START_V_MAX)
#define BLS_STOP_V_MAX_IS_VALID(val)	((val) < CIFISP_BLS_STOP_V_MAX)

#define BLS_SAMPLE_MAX_IS_VALID(val)	((val) < CIFISP_BLS_SAMPLES_MAX)

#define BLS_FIX_SUB_IS_VALID(val)	\
	((val) > (s16)CIFISP_BLS_FIX_SUB_MIN && (val) < CIFISP_BLS_FIX_SUB_MAX)

static inline void rkisp1_iowrite32(struct rkisp1_isp_params_vdev *params_vdev,
				    u32 value, u32 addr)
{
	iowrite32(value, params_vdev->dev->base_addr + addr);
}

static inline u32 rkisp1_ioread32(struct rkisp1_isp_params_vdev *params_vdev,
				  u32 addr)
{
	return ioread32(params_vdev->dev->base_addr + addr);
}

static inline void isp_param_set_bits(struct rkisp1_isp_params_vdev
					     *params_vdev,
				      u32 reg, u32 bit_mask)
{
	u32 val;

	val = rkisp1_ioread32(params_vdev, reg);
	rkisp1_iowrite32(params_vdev, val | bit_mask, reg);
}

static inline void isp_param_clear_bits(struct rkisp1_isp_params_vdev
					       *params_vdev,
					u32 reg, u32 bit_mask)
{
	u32 val;

	val = rkisp1_ioread32(params_vdev, reg);
	rkisp1_iowrite32(params_vdev, val & ~bit_mask, reg);
}

/* ISP BP interface function */
static int dpcc_param_check(struct rkisp1_isp_params_vdev *params_vdev,
			    struct cifisp_dpcc_config *arg)
{
	unsigned int i;
	struct cifisp_dpcc_methods_config *method;

	if (arg->mode > CIF_ISP_DPCC_MODE_MAX ||
	    arg->output_mode > CIF_ISP_DPCC_OUTPUTMODE_MAX ||
	    arg->set_use > CIF_ISP_DPCC_SETUSE_MAX) {
		goto err;
	}

	if (arg->ro_limits & CIF_ISP_DPCC_RO_LIMIT_RESERVED ||
	    arg->rnd_offs & CIF_ISP_DPCC_RND_OFFS_RESERVED) {
		goto err;
	}

	method = &arg->methods[i];
	for (i = 0; i < CIFISP_DPCC_METHODS_MAX; i++) {
		if ((method->method & CIF_ISP_DPCC_METHODS_SET_RESERVED) ||
		    (method->line_thresh & CIF_ISP_DPCC_LINE_THRESH_RESERVED) ||
		    (method->line_mad_fac &
		    CIF_ISP_DPCC_LINE_MAD_FAC_RESERVED)) {
			goto err;
		}

		if ((method->pg_fac & CIF_ISP_DPCC_PG_FAC_RESERVED) ||
		    (method->rnd_thresh & CIF_ISP_DPCC_RND_THRESH_RESERVED) ||
		    (method->rg_fac & CIF_ISP_DPCC_RG_FAC_RESERVED)) {
			goto err;
		}
		method++;
	}

	return 0;
err:
	v4l2_err(params_vdev->vnode.vdev.v4l2_dev,
		 "incompatible param in function: %s\n",
		 __func__);
	return -EINVAL;
}

static void dpcc_config(struct rkisp1_isp_params_vdev *params_vdev,
			struct cifisp_dpcc_config *arg)
{
	unsigned int i;

	rkisp1_iowrite32(params_vdev, arg->mode, CIF_ISP_DPCC_MODE);
	rkisp1_iowrite32(params_vdev, arg->output_mode,
			 CIF_ISP_DPCC_OUTPUT_MODE);
	rkisp1_iowrite32(params_vdev, arg->set_use, CIF_ISP_DPCC_SET_USE);

	rkisp1_iowrite32(params_vdev, arg->methods[0].method,
			 CIF_ISP_DPCC_METHODS_SET_1);
	rkisp1_iowrite32(params_vdev, arg->methods[1].method,
			 CIF_ISP_DPCC_METHODS_SET_2);
	rkisp1_iowrite32(params_vdev, arg->methods[2].method,
			 CIF_ISP_DPCC_METHODS_SET_3);
	for (i = 0; i < CIFISP_DPCC_METHODS_MAX; i++) {
		rkisp1_iowrite32(params_vdev, arg->methods[i].line_thresh,
				 CIF_ISP_DPCC_LINE_THRESH_1 + 0x14 * i);
		rkisp1_iowrite32(params_vdev, arg->methods[i].line_mad_fac,
				 CIF_ISP_DPCC_LINE_MAD_FAC_1 + 0x14 * i);
		rkisp1_iowrite32(params_vdev, arg->methods[i].pg_fac,
				 CIF_ISP_DPCC_PG_FAC_1 + 0x14 * i);
		rkisp1_iowrite32(params_vdev, arg->methods[i].rnd_thresh,
				 CIF_ISP_DPCC_RND_THRESH_1 + 0x14 * i);
		rkisp1_iowrite32(params_vdev, arg->methods[i].rg_fac,
				 CIF_ISP_DPCC_RG_FAC_1 + 0x14 * i);
	}

	rkisp1_iowrite32(params_vdev, arg->rnd_offs, CIF_ISP_DPCC_RND_OFFS);
	rkisp1_iowrite32(params_vdev, arg->ro_limits, CIF_ISP_DPCC_RO_LIMITS);
}

/* ISP black level subtraction interface function */
static int bls_param_check(struct rkisp1_isp_params_vdev *params_vdev,
			   struct cifisp_bls_config *arg)
{
	if (!BLS_START_H_MAX_IS_VALID(arg->bls_window1.h_offs) ||
	    !BLS_STOP_H_MAX_IS_VALID(arg->bls_window1.h_size)  ||
	    !BLS_START_V_MAX_IS_VALID(arg->bls_window1.v_offs) ||
	    !BLS_STOP_V_MAX_IS_VALID(arg->bls_window1.v_size))
		goto err;

	if (!BLS_START_H_MAX_IS_VALID(arg->bls_window2.h_offs) ||
	    !BLS_STOP_H_MAX_IS_VALID(arg->bls_window2.h_size) ||
	    !BLS_START_V_MAX_IS_VALID(arg->bls_window2.v_offs) ||
	    !BLS_STOP_V_MAX_IS_VALID(arg->bls_window2.v_size))
		goto err;

	if (!BLS_SAMPLE_MAX_IS_VALID(arg->bls_samples))
		goto err;

	if (!BLS_FIX_SUB_IS_VALID(arg->fixed_val.r) ||
	    !BLS_FIX_SUB_IS_VALID(arg->fixed_val.gr) ||
	    !BLS_FIX_SUB_IS_VALID(arg->fixed_val.gb) ||
	    !BLS_FIX_SUB_IS_VALID(arg->fixed_val.b))
		goto err;

	return 0;
err:
	v4l2_err(params_vdev->vnode.vdev.v4l2_dev,
		 "incompatible param in function: %s\n", __func__);
	return -EINVAL;
}

static void bls_config(struct rkisp1_isp_params_vdev *params_vdev,
		       struct cifisp_bls_config *arg)
{
	u32 new_control = 0;

	/* fixed subtraction values */
	if (!arg->enable_auto) {
		const struct cifisp_bls_fixed_val *pval = &arg->fixed_val;

		switch (params_vdev->raw_type) {
		case RAW_BGGR:
			rkisp1_iowrite32(params_vdev,
					 pval->r, CIF_ISP_BLS_D_FIXED);
			rkisp1_iowrite32(params_vdev,
					 pval->gr, CIF_ISP_BLS_C_FIXED);
			rkisp1_iowrite32(params_vdev,
					 pval->gb, CIF_ISP_BLS_B_FIXED);
			rkisp1_iowrite32(params_vdev,
					 pval->b, CIF_ISP_BLS_A_FIXED);
			break;
		case RAW_GBRG:
			rkisp1_iowrite32(params_vdev,
					 pval->r, CIF_ISP_BLS_C_FIXED);
			rkisp1_iowrite32(params_vdev,
					 pval->gr, CIF_ISP_BLS_D_FIXED);
			rkisp1_iowrite32(params_vdev,
					 pval->gb, CIF_ISP_BLS_A_FIXED);
			rkisp1_iowrite32(params_vdev,
					 pval->b, CIF_ISP_BLS_B_FIXED);
			break;
		case RAW_GRBG:
			rkisp1_iowrite32(params_vdev,
					 pval->r, CIF_ISP_BLS_B_FIXED);
			rkisp1_iowrite32(params_vdev,
					 pval->gr, CIF_ISP_BLS_A_FIXED);
			rkisp1_iowrite32(params_vdev,
					 pval->gb, CIF_ISP_BLS_D_FIXED);
			rkisp1_iowrite32(params_vdev,
					 pval->b, CIF_ISP_BLS_C_FIXED);
			break;
		case RAW_RGGB:
			rkisp1_iowrite32(params_vdev,
					 pval->r, CIF_ISP_BLS_A_FIXED);
			rkisp1_iowrite32(params_vdev,
					 pval->gr, CIF_ISP_BLS_B_FIXED);
			rkisp1_iowrite32(params_vdev,
					 pval->gb, CIF_ISP_BLS_C_FIXED);
			rkisp1_iowrite32(params_vdev,
					 pval->b, CIF_ISP_BLS_D_FIXED);
			break;
		default:
			break;
		}

		new_control = CIF_ISP_BLS_MODE_FIXED;
		rkisp1_iowrite32(params_vdev, new_control, CIF_ISP_BLS_CTRL);
	} else {
		if (arg->en_windows & 2) {
			rkisp1_iowrite32(params_vdev, arg->bls_window2.h_offs,
					 CIF_ISP_BLS_H2_START);
			rkisp1_iowrite32(params_vdev, arg->bls_window2.h_size,
					 CIF_ISP_BLS_H2_STOP);
			rkisp1_iowrite32(params_vdev, arg->bls_window2.v_offs,
					 CIF_ISP_BLS_V2_START);
			rkisp1_iowrite32(params_vdev, arg->bls_window2.v_size,
					 CIF_ISP_BLS_V2_STOP);
			new_control |= CIF_ISP_BLS_WINDOW_2;
		}

		if (arg->en_windows & 1) {
			rkisp1_iowrite32(params_vdev, arg->bls_window1.h_offs,
					 CIF_ISP_BLS_H1_START);
			rkisp1_iowrite32(params_vdev, arg->bls_window1.h_size,
					 CIF_ISP_BLS_H1_STOP);
			rkisp1_iowrite32(params_vdev, arg->bls_window1.v_offs,
					 CIF_ISP_BLS_V1_START);
			rkisp1_iowrite32(params_vdev, arg->bls_window1.v_size,
					 CIF_ISP_BLS_V1_STOP);
			new_control |= CIF_ISP_BLS_WINDOW_1;
		}

		rkisp1_iowrite32(params_vdev, arg->bls_samples,
				 CIF_ISP_BLS_SAMPLES);

		new_control |= CIF_ISP_BLS_MODE_MEASURED;

		rkisp1_iowrite32(params_vdev, new_control, CIF_ISP_BLS_CTRL);
	}
}

/* ISP LS correction interface function */
static void
__lsc_correct_matrix_config(struct rkisp1_isp_params_vdev *params_vdev,
			    struct cifisp_lsc_config *pconfig)
{
	int i, j;
	unsigned int isp_lsc_status, sram_addr, isp_lsc_table_sel;
	unsigned int data;

	isp_lsc_status = rkisp1_ioread32(params_vdev, CIF_ISP_LSC_STATUS);

	/* CIF_ISP_LSC_TABLE_ADDRESS_153 = ( 17 * 18 ) >> 1 */
	sram_addr = (isp_lsc_status & CIF_ISP_LSC_ACTIVE_TABLE) ?
		     CIF_ISP_LSC_TABLE_ADDRESS_0 :
		     CIF_ISP_LSC_TABLE_ADDRESS_153;
	rkisp1_iowrite32(params_vdev, sram_addr, CIF_ISP_LSC_R_TABLE_ADDR);
	rkisp1_iowrite32(params_vdev, sram_addr, CIF_ISP_LSC_GR_TABLE_ADDR);
	rkisp1_iowrite32(params_vdev, sram_addr, CIF_ISP_LSC_GB_TABLE_ADDR);
	rkisp1_iowrite32(params_vdev, sram_addr, CIF_ISP_LSC_B_TABLE_ADDR);

	/* program data tables (table size is 9 * 17 = 153) */
	for (i = 0; i < ((CIF_ISP_LSC_SECTORS_MAX + 1) *
			 (CIF_ISP_LSC_SECTORS_MAX + 1));
	     i += CIF_ISP_LSC_SECTORS_MAX + 1) {
		/*
		 * 17 sectors with 2 values in one DWORD = 9
		 * DWORDs (8 steps + 1 outside loop)
		 */
		for (j = 0; j < (CIF_ISP_LSC_SECTORS_MAX); j += 2) {
			data = CIF_ISP_LSC_TABLE_DATA(
					pconfig->r_data_tbl[i + j],
					pconfig->r_data_tbl[i + j + 1]);
			rkisp1_iowrite32(params_vdev, data,
					 CIF_ISP_LSC_R_TABLE_DATA);

			data = CIF_ISP_LSC_TABLE_DATA(
					pconfig->gr_data_tbl[i + j],
					pconfig->gr_data_tbl[i + j + 1]);
			rkisp1_iowrite32(params_vdev, data,
					 CIF_ISP_LSC_GR_TABLE_DATA);

			data = CIF_ISP_LSC_TABLE_DATA(
					pconfig->gb_data_tbl[i + j],
					pconfig->gb_data_tbl[i + j + 1]);
			rkisp1_iowrite32(params_vdev, data,
					 CIF_ISP_LSC_GB_TABLE_DATA);

			data = CIF_ISP_LSC_TABLE_DATA(
					pconfig->b_data_tbl[i + j],
					pconfig->b_data_tbl[i + j + 1]);
			rkisp1_iowrite32(params_vdev, data,
					 CIF_ISP_LSC_B_TABLE_DATA);
		}

		data = CIF_ISP_LSC_TABLE_DATA(
			pconfig->r_data_tbl[i + CIF_ISP_LSC_SECTORS_MAX],
			/* isp_dev->lsc_config.r_data_tbl[n + i] */
			0);
		rkisp1_iowrite32(params_vdev, data, CIF_ISP_LSC_R_TABLE_DATA);

		data = CIF_ISP_LSC_TABLE_DATA(
			pconfig->gr_data_tbl[i + CIF_ISP_LSC_SECTORS_MAX],
			/* isp_dev->lsc_config.gr_data_tbl[n + i] */
			0);
		rkisp1_iowrite32(params_vdev, data, CIF_ISP_LSC_GR_TABLE_DATA);

		data = CIF_ISP_LSC_TABLE_DATA(
			pconfig->gb_data_tbl[i + CIF_ISP_LSC_SECTORS_MAX],
			/* isp_dev->lsc_config.gr_data_tbl[n + i] */
			0);
		rkisp1_iowrite32(params_vdev, data, CIF_ISP_LSC_GB_TABLE_DATA);

		data = CIF_ISP_LSC_TABLE_DATA(
			pconfig->b_data_tbl[i + CIF_ISP_LSC_SECTORS_MAX],
			/* isp_dev->lsc_config.b_data_tbl[n + i] */
			0);
		rkisp1_iowrite32(params_vdev, data, CIF_ISP_LSC_B_TABLE_DATA);
	}

	isp_lsc_table_sel = (isp_lsc_status & CIF_ISP_LSC_ACTIVE_TABLE) ?
				CIF_ISP_LSC_TABLE_0 : CIF_ISP_LSC_TABLE_1;
	rkisp1_iowrite32(params_vdev, isp_lsc_table_sel, CIF_ISP_LSC_TABLE_SEL);
}

static int lsc_param_check(struct rkisp1_isp_params_vdev *params_vdev,
			   struct cifisp_lsc_config *arg)
{
	int i;

	if (arg->config_width != params_vdev->isp_acq_width ||
	    arg->config_height != params_vdev->isp_acq_height) {
		v4l2_err(params_vdev->vnode.vdev.v4l2_dev,
			 "LSC config: lsc_w %d lsc_h %d act_w %d act_h %d\n",
			 arg->config_width,
			 arg->config_height,
			 params_vdev->isp_acq_width,
			 params_vdev->isp_acq_height);
		return -EINVAL;
	}

	for (i = 0; i < CIFISP_LSC_SIZE_TBL_SIZE; i++) {
		if ((arg->x_size_tbl[i] & CIF_ISP_LSC_SECT_SIZE_RESERVED) ||
		    (arg->y_size_tbl[i] & CIF_ISP_LSC_SECT_SIZE_RESERVED)) {
			v4l2_err(params_vdev->vnode.vdev.v4l2_dev,
				 "incompatible sect size x 0x%x y 0x%x in function: %s\n",
				  arg->x_size_tbl[i],
				  arg->y_size_tbl[i], __func__);
			return -EINVAL;
		}
	}

	for (i = 0; i < CIFISP_LSC_GRAD_TBL_SIZE; i++) {
		if ((arg->x_grad_tbl[i] & CIF_ISP_LSC_GRAD_RESERVED) ||
		    (arg->y_grad_tbl[i] & CIF_ISP_LSC_GRAD_RESERVED)) {
			v4l2_err(params_vdev->vnode.vdev.v4l2_dev,
				      "incompatible grad x 0x%x y 0x%x in function: %s\n",
				      arg->x_grad_tbl[i],
				      arg->y_grad_tbl[i], __func__);
			return -EINVAL;
		}
	}

	for (i = 0; i < CIFISP_LSC_DATA_TBL_SIZE; i++) {
		if ((arg->r_data_tbl[i] &
		     CIF_ISP_LSC_SAMPLE_RESERVED) ||
		    (arg->gr_data_tbl[i] &
		     CIF_ISP_LSC_SAMPLE_RESERVED) ||
		    (arg->gb_data_tbl[i] &
		     CIF_ISP_LSC_SAMPLE_RESERVED) ||
		    (arg->b_data_tbl[i] & CIF_ISP_LSC_SAMPLE_RESERVED)) {
			v4l2_err(params_vdev->vnode.vdev.v4l2_dev,
				 "incompatible sample r 0x%x gr 0x%x gb 0x%x b 0x%x in function: %s\n",
				 arg->r_data_tbl[i],
				 arg->gr_data_tbl[i],
				 arg->gb_data_tbl[i],
				 arg->b_data_tbl[i], __func__);
			return -EINVAL;
		}
	}

	return 0;
}

static void lsc_config(struct rkisp1_isp_params_vdev *params_vdev,
		       struct cifisp_lsc_config *arg)
{
	int i;
	unsigned int data;
	u32 lsc_ctrl;

	/* To config must be off , store the current status firstly */
	lsc_ctrl = rkisp1_ioread32(params_vdev, CIF_ISP_LSC_CTRL);
	isp_param_clear_bits(params_vdev, CIF_ISP_LSC_CTRL,
			     CIF_ISP_LSC_CTRL_ENA);
	__lsc_correct_matrix_config(params_vdev, arg);

	if (params_vdev->active_lsc_width !=
	    arg->config_width ||
	    params_vdev->active_lsc_height != arg->config_height) {
		for (i = 0; i < 4; i++) {
			/* program x size tables */
			data = CIF_ISP_LSC_SECT_SIZE(arg->x_size_tbl[i * 2],
						arg->x_size_tbl[i * 2 + 1]);
			rkisp1_iowrite32(params_vdev, data,
					 CIF_ISP_LSC_XSIZE_01 + i * 4);

			/* program x grad tables */
			data = CIF_ISP_LSC_SECT_SIZE(arg->x_grad_tbl[i * 2],
						arg->x_grad_tbl[i * 2 + 1]);
			rkisp1_iowrite32(params_vdev, data,
					 CIF_ISP_LSC_XGRAD_01 + i * 4);

			/* program y size tables */
			data = CIF_ISP_LSC_SECT_SIZE(arg->y_size_tbl[i * 2],
						arg->y_size_tbl[i * 2 + 1]);
			rkisp1_iowrite32(params_vdev, data,
					 CIF_ISP_LSC_YSIZE_01 + i * 4);

			/* program y grad tables */
			data = CIF_ISP_LSC_SECT_SIZE(arg->y_grad_tbl[i * 2],
						arg->y_grad_tbl[i * 2 + 1]);
			rkisp1_iowrite32(params_vdev, data,
					 CIF_ISP_LSC_YGRAD_01 + i * 4);
		}

		params_vdev->active_lsc_width = arg->config_width;
		params_vdev->active_lsc_height = arg->config_height;
	}

	/* restore the bls ctrl status */
	if (lsc_ctrl & CIF_ISP_LSC_CTRL_ENA) {
		isp_param_set_bits(params_vdev,
				   CIF_ISP_LSC_CTRL,
				   CIF_ISP_LSC_CTRL_ENA);
	} else {
		isp_param_clear_bits(params_vdev,
				     CIF_ISP_LSC_CTRL,
				     CIF_ISP_LSC_CTRL_ENA);
	}
}

/* ISP Filtering function */
static int flt_param_check(struct rkisp1_isp_params_vdev *params_vdev,
			   struct cifisp_flt_config *arg)
{
	if (arg->mode > CIF_ISP_FLT_MODE_MAX ||
	    arg->grn_stage1 > CIF_ISP_FLT_GREEN_STAGE1_MAX ||
	    arg->chr_v_mode > CIF_ISP_FLT_CHROMA_MODE_MAX ||
	    arg->chr_h_mode > CIF_ISP_FLT_CHROMA_MODE_MAX ||
	    arg->thresh_sh0 & CIF_ISP_FLT_THREAD_RESERVED ||
	    arg->thresh_sh1 & CIF_ISP_FLT_THREAD_RESERVED ||
	    arg->thresh_bl0 & CIF_ISP_FLT_THREAD_RESERVED ||
	    arg->thresh_bl1 & CIF_ISP_FLT_THREAD_RESERVED ||
	    arg->fac_bl0 & CIF_ISP_FLT_FAC_RESERVED ||
	    arg->fac_bl1 & CIF_ISP_FLT_FAC_RESERVED ||
	    arg->fac_sh0 & CIF_ISP_FLT_FAC_RESERVED ||
	    arg->fac_sh1 & CIF_ISP_FLT_FAC_RESERVED ||
	    arg->fac_mid & CIF_ISP_FLT_FAC_RESERVED ||
	    arg->lum_weight & CIF_ISP_FLT_LUM_WEIGHT_RESERVED) {
		v4l2_err(params_vdev->vnode.vdev.v4l2_dev,
			 "incompatible param in function: %s\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static void flt_config(struct rkisp1_isp_params_vdev *params_vdev,
		       struct cifisp_flt_config *arg)
{
	rkisp1_iowrite32(params_vdev, arg->thresh_bl0, CIF_ISP_FILT_THRESH_BL0);
	rkisp1_iowrite32(params_vdev, arg->thresh_bl1, CIF_ISP_FILT_THRESH_BL1);
	rkisp1_iowrite32(params_vdev, arg->thresh_sh0, CIF_ISP_FILT_THRESH_SH0);
	rkisp1_iowrite32(params_vdev, arg->thresh_sh1, CIF_ISP_FILT_THRESH_SH1);
	rkisp1_iowrite32(params_vdev, arg->fac_bl0, CIF_ISP_FILT_FAC_BL0);
	rkisp1_iowrite32(params_vdev, arg->fac_bl1, CIF_ISP_FILT_FAC_BL1);
	rkisp1_iowrite32(params_vdev, arg->fac_mid, CIF_ISP_FILT_FAC_MID);
	rkisp1_iowrite32(params_vdev, arg->fac_sh0, CIF_ISP_FILT_FAC_SH0);
	rkisp1_iowrite32(params_vdev, arg->fac_sh1, CIF_ISP_FILT_FAC_SH1);
	rkisp1_iowrite32(params_vdev, arg->lum_weight, CIF_ISP_FILT_LUM_WEIGHT);

	rkisp1_iowrite32(params_vdev, (arg->mode ? CIF_ISP_FLT_MODE_DNR : 0) |
			 CIF_ISP_FLT_CHROMA_V_MODE(arg->chr_v_mode) |
			 CIF_ISP_FLT_CHROMA_H_MODE(arg->chr_h_mode) |
			 CIF_ISP_FLT_GREEN_STAGE1(arg->grn_stage1),
			 CIF_ISP_FILT_MODE);
}

/* ISP demosaic interface function */
static int bdm_config(struct rkisp1_isp_params_vdev *params_vdev,
		      struct cifisp_bdm_config *arg)
{
	/*set demosaic threshold */
	rkisp1_iowrite32(params_vdev, arg->demosaic_th, CIF_ISP_DEMOSAIC);
	return 0;
}

/* ISP GAMMA correction interface function */
static int sdg_param_check(struct rkisp1_isp_params_vdev *params_vdev,
			   struct cifisp_sdg_config *arg)
{
	int i;

	if (arg->xa_pnts.gamma_dx0 & CIFISP_DEGAMMA_X_RESERVED ||
	    arg->xa_pnts.gamma_dx1 & CIFISP_DEGAMMA_X_RESERVED) {
		v4l2_err(params_vdev->vnode.vdev.v4l2_dev,
			 "incompatible param in function: %s\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < CIFISP_DEGAMMA_CURVE_SIZE; i++) {
		if ((arg->curve_b.gamma_y[i] & CIFISP_DEGAMMA_Y_RESERVED) ||
		    (arg->curve_r.gamma_y[i] & CIFISP_DEGAMMA_Y_RESERVED) ||
		    (arg->curve_g.gamma_y[i] & CIFISP_DEGAMMA_Y_RESERVED)) {
			v4l2_err(params_vdev->vnode.vdev.v4l2_dev,
				 "incompatible param in function: %s\n",
				 __func__);
			return -EINVAL;
		}
	}

	return 0;
}

static void sdg_config(struct rkisp1_isp_params_vdev *params_vdev,
		       struct cifisp_sdg_config *arg)
{
	int i;

	rkisp1_iowrite32(params_vdev,
			 arg->xa_pnts.gamma_dx0, CIF_ISP_GAMMA_DX_LO);
	rkisp1_iowrite32(params_vdev,
			 arg->xa_pnts.gamma_dx1, CIF_ISP_GAMMA_DX_HI);

	for (i = 0; i < CIFISP_DEGAMMA_CURVE_SIZE; i++) {
		rkisp1_iowrite32(params_vdev, arg->curve_r.gamma_y[i],
				 CIF_ISP_GAMMA_R_Y0 + i * 4);
		rkisp1_iowrite32(params_vdev, arg->curve_g.gamma_y[i],
				 CIF_ISP_GAMMA_G_Y0 + i * 4);
		rkisp1_iowrite32(params_vdev, arg->curve_b.gamma_y[i],
				 CIF_ISP_GAMMA_B_Y0 + i * 4);
	}
}

/* ISP GAMMA correction interface function */
static int goc_param_check(struct rkisp1_isp_params_vdev *params_vdev,
			   struct cifisp_goc_config *arg)
{
	if (arg->mode > CIF_ISP_GOC_MODE_MAX) {
		v4l2_err(params_vdev->vnode.vdev.v4l2_dev,
			 "incompatible param 0x%x in  function: %s\n",
			 arg->mode, __func__);
		return -EINVAL;
	}

	return 0;
}

static void goc_config(struct rkisp1_isp_params_vdev *params_vdev,
		       struct cifisp_goc_config *arg)
{
	int i;

	isp_param_clear_bits(params_vdev, CIF_ISP_CTRL,
			     CIF_ISP_CTRL_ISP_GAMMA_OUT_ENA);
	rkisp1_iowrite32(params_vdev, arg->mode, CIF_ISP_GAMMA_OUT_MODE);

	for (i = 0; i < CIFISP_GAMMA_OUT_MAX_SAMPLES; i++)
		rkisp1_iowrite32(params_vdev, arg->gamma_y[i],
				 CIF_ISP_GAMMA_OUT_Y_0 + i * 4);
}

/* ISP Cross Talk */
static int ctk_param_check(struct rkisp1_isp_params_vdev *params_vdev,
			   struct cifisp_ctk_config *arg)
{
	if (arg->coeff0 & CIF_ISP_CTK_COEFF_RESERVED ||
	    arg->coeff1 & CIF_ISP_CTK_COEFF_RESERVED ||
	    arg->coeff2 & CIF_ISP_CTK_COEFF_RESERVED ||
	    arg->coeff3 & CIF_ISP_CTK_COEFF_RESERVED ||
	    arg->coeff4 & CIF_ISP_CTK_COEFF_RESERVED ||
	    arg->coeff5 & CIF_ISP_CTK_COEFF_RESERVED ||
	    arg->coeff6 & CIF_ISP_CTK_COEFF_RESERVED ||
	    arg->coeff7 & CIF_ISP_CTK_COEFF_RESERVED ||
	    arg->coeff8 & CIF_ISP_CTK_COEFF_RESERVED ||
	    arg->ct_offset_r & CIF_ISP_XTALK_OFFSET_RESERVED ||
	    arg->ct_offset_g & CIF_ISP_XTALK_OFFSET_RESERVED ||
	    arg->ct_offset_b & CIF_ISP_XTALK_OFFSET_RESERVED) {
		v4l2_err(params_vdev->vnode.vdev.v4l2_dev,
			 "incompatible param in function: %s\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static void ctk_config(struct rkisp1_isp_params_vdev *params_vdev,
		       struct cifisp_ctk_config *arg)
{
	rkisp1_iowrite32(params_vdev, arg->coeff0, CIF_ISP_CT_COEFF_0);
	rkisp1_iowrite32(params_vdev, arg->coeff1, CIF_ISP_CT_COEFF_1);
	rkisp1_iowrite32(params_vdev, arg->coeff2, CIF_ISP_CT_COEFF_2);
	rkisp1_iowrite32(params_vdev, arg->coeff3, CIF_ISP_CT_COEFF_3);
	rkisp1_iowrite32(params_vdev, arg->coeff4, CIF_ISP_CT_COEFF_4);
	rkisp1_iowrite32(params_vdev, arg->coeff5, CIF_ISP_CT_COEFF_5);
	rkisp1_iowrite32(params_vdev, arg->coeff6, CIF_ISP_CT_COEFF_6);
	rkisp1_iowrite32(params_vdev, arg->coeff7, CIF_ISP_CT_COEFF_7);
	rkisp1_iowrite32(params_vdev, arg->coeff8, CIF_ISP_CT_COEFF_8);
	rkisp1_iowrite32(params_vdev, arg->ct_offset_r, CIF_ISP_CT_OFFSET_R);
	rkisp1_iowrite32(params_vdev, arg->ct_offset_g, CIF_ISP_CT_OFFSET_G);
	rkisp1_iowrite32(params_vdev, arg->ct_offset_b, CIF_ISP_CT_OFFSET_B);
}

static void ctk_enable(struct rkisp1_isp_params_vdev *params_vdev, bool en)
{
	if (en)
		return;

	/* Write back the default values. */
	rkisp1_iowrite32(params_vdev, 0x80, CIF_ISP_CT_COEFF_0);
	rkisp1_iowrite32(params_vdev, 0, CIF_ISP_CT_COEFF_1);
	rkisp1_iowrite32(params_vdev, 0, CIF_ISP_CT_COEFF_2);
	rkisp1_iowrite32(params_vdev, 0, CIF_ISP_CT_COEFF_3);
	rkisp1_iowrite32(params_vdev, 0x80, CIF_ISP_CT_COEFF_4);
	rkisp1_iowrite32(params_vdev, 0, CIF_ISP_CT_COEFF_5);
	rkisp1_iowrite32(params_vdev, 0, CIF_ISP_CT_COEFF_6);
	rkisp1_iowrite32(params_vdev, 0, CIF_ISP_CT_COEFF_7);
	rkisp1_iowrite32(params_vdev, 0x80, CIF_ISP_CT_COEFF_8);

	rkisp1_iowrite32(params_vdev, 0, CIF_ISP_CT_OFFSET_R);
	rkisp1_iowrite32(params_vdev, 0, CIF_ISP_CT_OFFSET_G);
	rkisp1_iowrite32(params_vdev, 0, CIF_ISP_CT_OFFSET_B);
}

/* ISP White Balance Mode */
static int awb_meas_param_check(struct rkisp1_isp_params_vdev *params_vdev,
				struct cifisp_awb_meas_config *arg)
{
	if (arg->awb_mode > CIFISP_AWB_MODE_YCBCR ||
	    arg->awb_wnd.h_offs > CIF_ISP_AWB_WINDOW_OFFSET_MAX ||
	    arg->awb_wnd.v_offs > CIF_ISP_AWB_WINDOW_OFFSET_MAX ||
	    arg->awb_wnd.h_size > CIF_ISP_AWB_WINDOW_MAX_SIZE ||
	    arg->awb_wnd.v_size > CIF_ISP_AWB_WINDOW_MAX_SIZE ||
	    arg->frames > CIFISP_AWB_MAX_FRAMES) {
		v4l2_err(params_vdev->vnode.vdev.v4l2_dev,
			 "incompatible param in function: %s\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static void awb_meas_config(struct rkisp1_isp_params_vdev *params_vdev,
			    struct cifisp_awb_meas_config *arg)
{
	/* based on the mode,configure the awb module */
	if (arg->awb_mode == CIFISP_AWB_MODE_YCBCR) {
		/* Reference Cb and Cr */
		rkisp1_iowrite32(params_vdev,
				 CIF_ISP_AWB_REF_CR_SET(arg->awb_ref_cr) |
				 arg->awb_ref_cb, CIF_ISP_AWB_REF);
		/* Yc Threshold */
		rkisp1_iowrite32(params_vdev,
				 CIF_ISP_AWB_MAX_Y_SET(arg->max_y) |
				 CIF_ISP_AWB_MIN_Y_SET(arg->min_y) |
				 CIF_ISP_AWB_MAX_CS_SET(arg->max_csum) |
				 arg->min_c, CIF_ISP_AWB_THRESH);
	}

	/* window offset */
	rkisp1_iowrite32(params_vdev,
			 arg->awb_wnd.v_offs, CIF_ISP_AWB_WND_V_OFFS);
	rkisp1_iowrite32(params_vdev,
			 arg->awb_wnd.h_offs, CIF_ISP_AWB_WND_H_OFFS);
	/* AWB window size */
	rkisp1_iowrite32(params_vdev,
			 arg->awb_wnd.v_size, CIF_ISP_AWB_WND_V_SIZE);
	rkisp1_iowrite32(params_vdev,
			 arg->awb_wnd.h_size, CIF_ISP_AWB_WND_H_SIZE);
	/* Number of frames */
	rkisp1_iowrite32(params_vdev,
			 arg->frames, CIF_ISP_AWB_FRAMES);
}

static void awb_meas_enable(struct rkisp1_isp_params_vdev *params_vdev,
			    struct cifisp_awb_meas_config *arg, bool en)
{
	u32 reg_val = rkisp1_ioread32(params_vdev, CIF_ISP_AWB_PROP);

	/* switch off */
	reg_val &= CIF_ISP_AWB_MODE_MASK_NONE;

	if (en) {
		if (arg->awb_mode == CIFISP_AWB_MODE_RGB)
			reg_val |= CIF_ISP_AWB_MODE_RGB_EN;
		else
			reg_val |= CIF_ISP_AWB_MODE_YCBCR_EN;

		rkisp1_iowrite32(params_vdev, reg_val, CIF_ISP_AWB_PROP);

		/* Measurements require AWB block be active. */
		/* TODO: need to enable here ? awb_gain_enable has done this */
		isp_param_set_bits(params_vdev, CIF_ISP_CTRL,
				   CIF_ISP_CTRL_ISP_AWB_ENA);
	} else {
		rkisp1_iowrite32(params_vdev,
				 reg_val, CIF_ISP_AWB_PROP);
		isp_param_clear_bits(params_vdev, CIF_ISP_CTRL,
				     CIF_ISP_CTRL_ISP_AWB_ENA);
	}
}

static int awb_gain_param_check(struct rkisp1_isp_params_vdev *params_vdev,
				struct cifisp_awb_gain_config *arg)
{
	if (arg->gain_red > CIF_ISP_AWB_GAINS_MAX_VAL ||
	    arg->gain_green_r > CIF_ISP_AWB_GAINS_MAX_VAL ||
	    arg->gain_green_b > CIF_ISP_AWB_GAINS_MAX_VAL ||
	    arg->gain_blue > CIF_ISP_AWB_GAINS_MAX_VAL) {
		v4l2_err(params_vdev->vnode.vdev.v4l2_dev,
			 "incompatible param in function: %s\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static void awb_gain_config(struct rkisp1_isp_params_vdev *params_vdev,
			    struct cifisp_awb_gain_config *arg)
{
	rkisp1_iowrite32(params_vdev,
			 CIF_ISP_AWB_GAIN_R_SET(arg->gain_green_r) |
			 arg->gain_green_b, CIF_ISP_AWB_GAIN_G);

	rkisp1_iowrite32(params_vdev, CIF_ISP_AWB_GAIN_R_SET(arg->gain_red) |
			 arg->gain_blue, CIF_ISP_AWB_GAIN_RB);
}

static int aec_param_check(struct rkisp1_isp_params_vdev *params_vdev,
			   struct cifisp_aec_config *arg)
{
	if (arg->meas_window.h_offs > CIF_ISP_EXP_MAX_HOFFS ||
	    arg->meas_window.h_size > CIF_ISP_EXP_MAX_HSIZE ||
	    arg->meas_window.h_size < CIF_ISP_EXP_MIN_HSIZE ||
	    arg->meas_window.v_offs > CIF_ISP_EXP_MAX_VOFFS ||
	    arg->meas_window.v_size > CIF_ISP_EXP_MAX_VSIZE ||
	    arg->meas_window.v_size < CIF_ISP_EXP_MIN_VSIZE ||
	    arg->mode > CIFISP_EXP_MEASURING_MODE_1) {
		v4l2_err(params_vdev->vnode.vdev.v4l2_dev,
			 "incompatible param in function: %s\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static void aec_config(struct rkisp1_isp_params_vdev *params_vdev,
		       struct cifisp_aec_config *arg)
{
	unsigned int block_hsize, block_vsize;

	rkisp1_iowrite32(params_vdev,
			 ((arg->autostop) ? CIF_ISP_EXP_CTRL_AUTOSTOP : 0) |
			 ((arg->mode == CIFISP_EXP_MEASURING_MODE_1) ?
			 CIF_ISP_EXP_CTRL_MEASMODE_1 : 0), CIF_ISP_EXP_CTRL);

	rkisp1_iowrite32(params_vdev,
			 arg->meas_window.h_offs, CIF_ISP_EXP_H_OFFSET);
	rkisp1_iowrite32(params_vdev,
			 arg->meas_window.v_offs, CIF_ISP_EXP_V_OFFSET);

	block_hsize = arg->meas_window.h_size / CIF_ISP_EXP_COLUMN_NUM - 1;
	block_vsize = arg->meas_window.v_size / CIF_ISP_EXP_ROW_NUM - 1;

	rkisp1_iowrite32(params_vdev, CIF_ISP_EXP_H_SIZE_SET(block_hsize),
			 CIF_ISP_EXP_H_SIZE);
	rkisp1_iowrite32(params_vdev, CIF_ISP_EXP_V_SIZE_SET(block_vsize),
			 CIF_ISP_EXP_V_SIZE);
}

static int cproc_param_check(struct rkisp1_isp_params_vdev *params_vdev,
			     struct cifisp_cproc_config *arg)
{
	if (arg->c_out_range & CIF_C_PROC_CTRL_RESERVED ||
	    arg->y_out_range & CIF_C_PROC_CTRL_RESERVED ||
	    arg->y_in_range & CIF_C_PROC_CTRL_RESERVED ||
	    arg->contrast & CIF_C_PROC_CONTRAST_RESERVED ||
	    arg->brightness & CIF_C_PROC_BRIGHTNESS_RESERVED ||
	    arg->sat & CIF_C_PROC_SATURATION_RESERVED ||
	    arg->hue & CIF_C_PROC_HUE_RESERVED) {
		v4l2_err(params_vdev->vnode.vdev.v4l2_dev,
			 "incompatible param in function: %s\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static void cproc_config(struct rkisp1_isp_params_vdev *params_vdev,
			 struct cifisp_cproc_config *arg)
{
	/* TODO: get quantization, ie effect*/
	u32 quantization = V4L2_QUANTIZATION_FULL_RANGE;
	u32 effect = V4L2_COLORFX_NONE;

	rkisp1_iowrite32(params_vdev, arg->contrast, CIF_C_PROC_CONTRAST);
	rkisp1_iowrite32(params_vdev, arg->hue, CIF_C_PROC_HUE);
	rkisp1_iowrite32(params_vdev, arg->sat, CIF_C_PROC_SATURATION);
	rkisp1_iowrite32(params_vdev, arg->brightness, CIF_C_PROC_BRIGHTNESS);

	if (quantization != V4L2_QUANTIZATION_FULL_RANGE ||
	    effect != V4L2_COLORFX_NONE) {
		rkisp1_iowrite32(params_vdev, ~(u32)(CIF_C_PROC_YOUT_FULL |
						CIF_C_PROC_YIN_FULL |
						CIF_C_PROC_COUT_FULL),
						CIF_C_PROC_CTRL);
	} else {
		isp_param_set_bits(params_vdev, CIF_C_PROC_CTRL,
				   (CIF_C_PROC_YOUT_FULL |
				    CIF_C_PROC_YIN_FULL |
				    CIF_C_PROC_COUT_FULL));
	}
}

static int hst_param_check(struct rkisp1_isp_params_vdev *params_vdev,
			   struct cifisp_hst_config *arg)
{
	int i;

	if (arg->mode > CIFISP_HISTOGRAM_MODE_Y_HISTOGRAM ||
	    arg->histogram_predivider > CIF_ISP_MAX_HIST_PREDIVIDER ||
	    arg->meas_window.v_offs & CIF_ISP_HIST_WINDOW_OFFSET_RESERVED ||
	    arg->meas_window.h_offs & CIF_ISP_HIST_WINDOW_OFFSET_RESERVED ||
	    (arg->meas_window.v_size / (CIF_ISP_HIST_ROW_NUM - 1)) &
	    CIF_ISP_HIST_WINDOW_SIZE_RESERVED ||
	    (arg->meas_window.h_size / (CIF_ISP_HIST_COLUMN_NUM - 1)) &
	    CIF_ISP_HIST_WINDOW_SIZE_RESERVED) {
		v4l2_err(params_vdev->vnode.vdev.v4l2_dev,
			 "incompatible param in function: %s\n",
			 __func__);
		return -EINVAL;
	}

	for (i = 0; i < CIFISP_HISTOGRAM_WEIGHT_GRIDS_SIZE; i++) {
		if (arg->hist_weight[i] & CIF_ISP_HIST_WEIGHT_RESERVED) {
			v4l2_err(params_vdev->vnode.vdev.v4l2_dev,
				 "incompatible param in function: %s\n",
				 __func__);
			return -EINVAL;
		}
	}

	return 0;
}

static void hst_config(struct rkisp1_isp_params_vdev *params_vdev,
		       struct cifisp_hst_config *arg)
{
	unsigned int block_hsize, block_vsize;

	rkisp1_iowrite32(params_vdev,
			 CIF_ISP_HIST_PREDIV_SET(arg->histogram_predivider),
			 CIF_ISP_HIST_PROP);
	rkisp1_iowrite32(params_vdev,
			 arg->meas_window.h_offs,
			 CIF_ISP_HIST_H_OFFS);
	rkisp1_iowrite32(params_vdev,
			 arg->meas_window.v_offs,
			 CIF_ISP_HIST_V_OFFS);

	block_hsize = arg->meas_window.h_size / CIF_ISP_HIST_COLUMN_NUM - 1;
	block_vsize = arg->meas_window.v_size / CIF_ISP_HIST_ROW_NUM - 1;

	rkisp1_iowrite32(params_vdev, block_hsize, CIF_ISP_HIST_H_SIZE);
	rkisp1_iowrite32(params_vdev, block_vsize, CIF_ISP_HIST_V_SIZE);

	rkisp1_iowrite32(params_vdev, CIF_ISP_HIST_WEIGHT_SET
			 (arg->hist_weight[0], arg->hist_weight[1],
			  arg->hist_weight[2], arg->hist_weight[3]),
			 CIF_ISP_HIST_WEIGHT_00TO30);
	rkisp1_iowrite32(params_vdev, CIF_ISP_HIST_WEIGHT_SET
			 (arg->hist_weight[4], arg->hist_weight[5],
			  arg->hist_weight[6], arg->hist_weight[7]),
			 CIF_ISP_HIST_WEIGHT_40TO21);
	rkisp1_iowrite32(params_vdev, CIF_ISP_HIST_WEIGHT_SET
			 (arg->hist_weight[8], arg->hist_weight[9],
			  arg->hist_weight[10], arg->hist_weight[11]),
			 CIF_ISP_HIST_WEIGHT_31TO12);
	rkisp1_iowrite32(params_vdev, CIF_ISP_HIST_WEIGHT_SET
			 (arg->hist_weight[12], arg->hist_weight[13],
			  arg->hist_weight[14], arg->hist_weight[15]),
			 CIF_ISP_HIST_WEIGHT_22TO03);
	rkisp1_iowrite32(params_vdev, CIF_ISP_HIST_WEIGHT_SET
			 (arg->hist_weight[16], arg->hist_weight[17],
			  arg->hist_weight[18], arg->hist_weight[19]),
			 CIF_ISP_HIST_WEIGHT_13TO43);
	rkisp1_iowrite32(params_vdev, CIF_ISP_HIST_WEIGHT_SET
			 (arg->hist_weight[20], arg->hist_weight[21],
			  arg->hist_weight[22], arg->hist_weight[23]),
			 CIF_ISP_HIST_WEIGHT_04TO34);
	rkisp1_iowrite32(params_vdev, CIF_ISP_HIST_WEIGHT_SET
			 (arg->hist_weight[24], 0, 0, 0),
			 CIF_ISP_HIST_WEIGHT_44);
}

static void hst_enable(struct rkisp1_isp_params_vdev *params_vdev,
		       struct cifisp_hst_config *arg, bool en)
{
	if (en)	{
		u32 hist_prop = rkisp1_ioread32(params_vdev, CIF_ISP_HIST_PROP);

		hist_prop &= ~CIF_ISP_HIST_PROP_MODE_MASK | arg->mode;
		isp_param_set_bits(params_vdev, CIF_ISP_HIST_PROP, hist_prop);
	} else {
		isp_param_clear_bits(params_vdev, CIF_ISP_HIST_PROP,
				     CIF_ISP_HIST_PROP_MODE_MASK);
	}
}

static inline unsigned int afm_calc_luminanceshift(const unsigned int
							maxpixelcnt)
{
	unsigned int lgrad = maxpixelcnt;
	unsigned int lshift = 0U;

	while (lgrad > 65793) {
		++lshift;
		lgrad >>= 1;
	}

	return lshift;
}

static inline unsigned int afm_calc_tennenggradshift(const unsigned int
							maxpixelcnt)
{
	unsigned int tgrad = maxpixelcnt;
	unsigned int tshift = 0U;

	while (tgrad > (128 * 128)) {
		++tshift;
		tgrad >>= 1;
	}

	return tshift;
}

static int afm_param_check(struct rkisp1_isp_params_vdev *params_vdev,
			   struct cifisp_afc_config *arg)
{
	int i;

	if (arg->num_afm_win > CIFISP_AFM_MAX_WINDOWS ||
	    arg->thres & CIF_ISP_AFM_THRES_RESERVED ||
	    arg->var_shift & CIF_ISP_AFM_VAR_SHIFT_RESERVED) {
		v4l2_err(params_vdev->vnode.vdev.v4l2_dev,
			 "incompatible param in function: %s\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < arg->num_afm_win; i++) {
		if (arg->afm_win[i].h_offs & CIF_ISP_AFM_WINDOW_X_RESERVED ||
		    arg->afm_win[i].h_offs < CIF_ISP_AFM_WINDOW_X_MIN ||
		    arg->afm_win[i].v_offs & CIF_ISP_AFM_WINDOW_Y_RESERVED ||
		    arg->afm_win[i].v_offs < CIF_ISP_AFM_WINDOW_Y_MIN ||
		    arg->afm_win[i].h_size & CIF_ISP_AFM_WINDOW_X_RESERVED ||
		    arg->afm_win[i].v_size & CIF_ISP_AFM_WINDOW_Y_RESERVED) {
			v4l2_err(params_vdev->vnode.vdev.v4l2_dev,
				 "incompatible param in function: %s\n",
				 __func__);
			return -EINVAL;
		}
	}

	return 0;
}

static void afm_config(struct rkisp1_isp_params_vdev *params_vdev,
		       struct cifisp_afc_config *arg)
{
	int num_of_win = arg->num_afm_win, i;

	/* Switch off to configure. Enabled during normal flow in frame isr. */
	rkisp1_iowrite32(params_vdev, ~(u32)CIF_ISP_AFM_ENA, CIF_ISP_AFM_CTRL);

	for (i = 0; i < num_of_win; i++) {
		rkisp1_iowrite32(params_vdev,
				 CIF_ISP_AFM_WINDOW_X(arg->afm_win[0].h_offs) |
				 CIF_ISP_AFM_WINDOW_Y(arg->afm_win[0].v_offs),
				 CIF_ISP_AFM_LT_A + i * 8);
		rkisp1_iowrite32(params_vdev,
				 CIF_ISP_AFM_WINDOW_X(arg->afm_win[0].h_size +
						      arg->afm_win[0].h_offs) |
				 CIF_ISP_AFM_WINDOW_Y(arg->afm_win[0].v_size +
						      arg->afm_win[0].v_offs),
				 CIF_ISP_AFM_RB_A + i * 8);
	}
	rkisp1_iowrite32(params_vdev, arg->thres, CIF_ISP_AFM_THRES);
	rkisp1_iowrite32(params_vdev, arg->var_shift, CIF_ISP_AFM_VAR_SHIFT);
}

static int ie_param_check(struct rkisp1_isp_params_vdev *params_vdev,
			  struct cifisp_ie_config *arg)
{
	switch (arg->effect) {
	case V4L2_COLORFX_NONE:
	case V4L2_COLORFX_BW:
	case V4L2_COLORFX_SEPIA:
	case V4L2_COLORFX_NEGATIVE:
	case V4L2_COLORFX_EMBOSS:
	case V4L2_COLORFX_SKETCH:
	case V4L2_COLORFX_AQUA:
	case V4L2_COLORFX_SET_CBCR:
		return 0;
	default:
		v4l2_err(params_vdev->vnode.vdev.v4l2_dev,
			 "incompatible param in function:%s\n", __func__);
		return -EINVAL;
	}
}

static void ie_config(struct rkisp1_isp_params_vdev *params_vdev,
		      struct cifisp_ie_config *arg)
{
	u32 eff_ctrl;

	eff_ctrl = rkisp1_ioread32(params_vdev, CIF_IMG_EFF_CTRL);
	eff_ctrl &= ~CIF_IMG_EFF_CTRL_MODE_MASK;

	switch (arg->effect) {
	case V4L2_COLORFX_SEPIA:
		eff_ctrl |= CIF_IMG_EFF_CTRL_MODE_SEPIA;
		break;
	case V4L2_COLORFX_SET_CBCR:
		rkisp1_iowrite32(params_vdev, arg->eff_tint, CIF_IMG_EFF_TINT);
		eff_ctrl |= CIF_IMG_EFF_CTRL_MODE_SEPIA;
		break;
		/*
		 * Color selection is similar to water color(AQUA):
		 * grayscale + selected color w threshold
		 */
	case V4L2_COLORFX_AQUA:
		eff_ctrl |= CIF_IMG_EFF_CTRL_MODE_COLOR_SEL;
		rkisp1_iowrite32(params_vdev, arg->color_sel,
				 CIF_IMG_EFF_COLOR_SEL);
		break;
	case V4L2_COLORFX_EMBOSS:
		eff_ctrl |= CIF_IMG_EFF_CTRL_MODE_EMBOSS;
		rkisp1_iowrite32(params_vdev, arg->eff_mat_1,
				 CIF_IMG_EFF_MAT_1);
		rkisp1_iowrite32(params_vdev, arg->eff_mat_2,
				 CIF_IMG_EFF_MAT_2);
		rkisp1_iowrite32(params_vdev, arg->eff_mat_3,
				 CIF_IMG_EFF_MAT_3);
		break;
	case V4L2_COLORFX_SKETCH:
		eff_ctrl |= CIF_IMG_EFF_CTRL_MODE_SKETCH;
		rkisp1_iowrite32(params_vdev, arg->eff_mat_3,
				 CIF_IMG_EFF_MAT_3);
		rkisp1_iowrite32(params_vdev, arg->eff_mat_4,
				 CIF_IMG_EFF_MAT_4);
		rkisp1_iowrite32(params_vdev, arg->eff_mat_5,
				 CIF_IMG_EFF_MAT_5);
		break;
	case V4L2_COLORFX_BW:
		eff_ctrl |= CIF_IMG_EFF_CTRL_MODE_BLACKWHITE;
		break;
	case V4L2_COLORFX_NEGATIVE:
		eff_ctrl |= CIF_IMG_EFF_CTRL_MODE_NEGATIVE;
		break;
	default:
		break;
	}

	rkisp1_iowrite32(params_vdev, eff_ctrl, CIF_IMG_EFF_CTRL);
}

static void ie_enable(struct rkisp1_isp_params_vdev *params_vdev, bool en)
{
	if (en) {
		isp_param_set_bits(params_vdev, CIF_ICCL, CIF_ICCL_IE_CLK);
		rkisp1_iowrite32(params_vdev, CIF_IMG_EFF_CTRL_ENABLE,
				 CIF_IMG_EFF_CTRL);
		isp_param_set_bits(params_vdev, CIF_IMG_EFF_CTRL,
				   CIF_IMG_EFF_CTRL_CFG_UPD);
	} else {
		/* Disable measurement */
		isp_param_clear_bits(params_vdev, CIF_IMG_EFF_CTRL,
				     CIF_IMG_EFF_CTRL_ENABLE);
		isp_param_clear_bits(params_vdev, CIF_ICCL, CIF_ICCL_IE_CLK);
	}
}

static const u16 full_range_coeff_array[] = {
	0x0026, 0x004b, 0x000f,
	0x01ea, 0x01d6, 0x0040,
	0x0040, 0x01ca, 0x01f6
};

static const u16 limited_range_coeff_array[] = {
	0x0021, 0x0040, 0x000d,
	0x01ed, 0x01db, 0x0038,
	0x0038, 0x01d1, 0x01f7,
};

static void csm_config(struct rkisp1_isp_params_vdev *params_vdev,
		       bool full_range)
{
	u8 i;
	const u16 *val;

	if (!full_range) {
		val = limited_range_coeff_array;

		for (i = 0; i < 8; i++)
			rkisp1_iowrite32(params_vdev, val[i],
					 CIF_ISP_CC_COEFF_0 + i * 4);

		isp_param_clear_bits(params_vdev, CIF_ISP_CTRL,
				     CIF_ISP_CTRL_ISP_CSM_Y_FULL_ENA |
				     CIF_ISP_CTRL_ISP_CSM_C_FULL_ENA);

	} else {
		val = full_range_coeff_array;

		for (i = 0; i < 8; i++)
			rkisp1_iowrite32(params_vdev, val[i],
					 CIF_ISP_CC_COEFF_0 + i * 4);

		isp_param_set_bits(params_vdev, CIF_ISP_CTRL,
				   CIF_ISP_CTRL_ISP_CSM_Y_FULL_ENA |
				   CIF_ISP_CTRL_ISP_CSM_C_FULL_ENA);
	}
}

/* ISP De-noise Pre-Filter(DPF) function */
static int dpf_param_check(struct rkisp1_isp_params_vdev *params_vdev,
			   struct cifisp_dpf_config *arg)
{
	int retval = 0;
	int i;

	/* Parameter check */
	if (arg->gain.mode >= CIFISP_DPF_GAIN_USAGE_MAX ||
	    arg->gain.mode < CIFISP_DPF_GAIN_USAGE_DISABLED ||
	    arg->gain.nf_b_gain & CIF_ISP_DPF_NF_GAIN_RESERVED ||
	    arg->gain.nf_r_gain & CIF_ISP_DPF_NF_GAIN_RESERVED ||
	    arg->gain.nf_gr_gain & CIF_ISP_DPF_NF_GAIN_RESERVED ||
	    arg->gain.nf_gb_gain & CIF_ISP_DPF_NF_GAIN_RESERVED) {
		v4l2_err(params_vdev->vnode.vdev.v4l2_dev,
			 "incompatible DPF GAIN param\n");
		retval = -EINVAL;
		goto end;
	}

	for (i = 0; i < CIFISP_DPF_MAX_SPATIAL_COEFFS; i++) {
		if (arg->g_flt.spatial_coeff[i] >
		    CIF_ISP_DPF_SPATIAL_COEFF_MAX) {
			v4l2_err(params_vdev->vnode.vdev.v4l2_dev,
				 "incompatible DPF G Spatial param\n");
			retval = -EINVAL;
			goto end;
		}

		if (arg->rb_flt.spatial_coeff[i] >
		    CIF_ISP_DPF_SPATIAL_COEFF_MAX) {
			v4l2_err(params_vdev->vnode.vdev.v4l2_dev,
				 "incompatible DPF RB Spatial param\n");
			retval = -EINVAL;
			goto end;
		}
	}

	if (arg->rb_flt.fltsize != CIFISP_DPF_RB_FILTERSIZE_9x9 &&
	    arg->rb_flt.fltsize != CIFISP_DPF_RB_FILTERSIZE_13x9) {
		v4l2_err(params_vdev->vnode.vdev.v4l2_dev,
			 "incompatible DPF RB filter size param\n");
		retval = -EINVAL;
		goto end;
	}

	for (i = 0; i < CIFISP_DPF_MAX_NLF_COEFFS; i++) {
		if (arg->nll.coeff[i] > CIF_ISP_DPF_NLL_COEFF_N_MAX) {
			v4l2_err(params_vdev->vnode.vdev.v4l2_dev,
				 "incompatible DPF NLL coeff param\n");
			retval = -EINVAL;
			goto end;
		}
	}

	if (arg->nll.scale_mode != CIFISP_NLL_SCALE_LINEAR &&
	    arg->nll.scale_mode != CIFISP_NLL_SCALE_LOGARITHMIC) {
		v4l2_err(params_vdev->vnode.vdev.v4l2_dev,
			 "incompatible DPF NLL scale mode param\n");
		retval = -EINVAL;
		goto end;
	}
end:
	return retval;
}

static void dpf_config(struct rkisp1_isp_params_vdev *params_vdev,
		       struct cifisp_dpf_config *arg)
{
	unsigned int isp_dpf_mode;
	unsigned int spatial_coeff;
	unsigned int i;

	switch (arg->gain.mode) {
	case CIFISP_DPF_GAIN_USAGE_NF_GAINS:
		isp_dpf_mode = CIF_ISP_DPF_MODE_USE_NF_GAIN |
				CIF_ISP_DPF_MODE_AWB_GAIN_COMP;
		break;
	case CIFISP_DPF_GAIN_USAGE_LSC_GAINS:
		isp_dpf_mode = CIF_ISP_DPF_MODE_LSC_GAIN_COMP;
		break;
	case CIFISP_DPF_GAIN_USAGE_NF_LSC_GAINS:
		isp_dpf_mode = CIF_ISP_DPF_MODE_USE_NF_GAIN |
				CIF_ISP_DPF_MODE_AWB_GAIN_COMP |
				CIF_ISP_DPF_MODE_LSC_GAIN_COMP;
		break;
	case CIFISP_DPF_GAIN_USAGE_AWB_GAINS:
		isp_dpf_mode = CIF_ISP_DPF_MODE_AWB_GAIN_COMP;
		break;
	case CIFISP_DPF_GAIN_USAGE_AWB_LSC_GAINS:
		isp_dpf_mode = CIF_ISP_DPF_MODE_LSC_GAIN_COMP |
				CIF_ISP_DPF_MODE_AWB_GAIN_COMP;
		break;
	case CIFISP_DPF_GAIN_USAGE_DISABLED:
	default:
		isp_dpf_mode = 0;
		break;
	}

	if (arg->nll.scale_mode == CIFISP_NLL_SCALE_LOGARITHMIC)
		isp_dpf_mode |= CIF_ISP_DPF_MODE_NLL_SEGMENTATION;
	if (arg->rb_flt.fltsize == CIFISP_DPF_RB_FILTERSIZE_9x9)
		isp_dpf_mode |= CIF_ISP_DPF_MODE_RB_FLTSIZE_9x9;
	if (!arg->rb_flt.r_enable)
		isp_dpf_mode |= CIF_ISP_DPF_MODE_R_FLT_DIS;
	if (!arg->rb_flt.b_enable)
		isp_dpf_mode |= CIF_ISP_DPF_MODE_B_FLT_DIS;
	if (!arg->g_flt.gb_enable)
		isp_dpf_mode |= CIF_ISP_DPF_MODE_GB_FLT_DIS;
	if (!arg->g_flt.gr_enable)
		isp_dpf_mode |= CIF_ISP_DPF_MODE_GR_FLT_DIS;

	isp_param_set_bits(params_vdev, CIF_ISP_DPF_MODE, isp_dpf_mode);
	rkisp1_iowrite32(params_vdev, arg->gain.nf_b_gain,
			 CIF_ISP_DPF_NF_GAIN_B);
	rkisp1_iowrite32(params_vdev, arg->gain.nf_r_gain,
			 CIF_ISP_DPF_NF_GAIN_R);
	rkisp1_iowrite32(params_vdev, arg->gain.nf_gb_gain,
			 CIF_ISP_DPF_NF_GAIN_GB);
	rkisp1_iowrite32(params_vdev, arg->gain.nf_gr_gain,
			 CIF_ISP_DPF_NF_GAIN_GR);

	for (i = 0; i < CIFISP_DPF_MAX_NLF_COEFFS; i++) {
		rkisp1_iowrite32(params_vdev, arg->nll.coeff[i],
				 CIF_ISP_DPF_NULL_COEFF_0 + i * 4);
	}

	spatial_coeff = arg->g_flt.spatial_coeff[0] |
			(arg->g_flt.spatial_coeff[1] << 8) |
			(arg->g_flt.spatial_coeff[2] << 16) |
			(arg->g_flt.spatial_coeff[3] << 24);
	rkisp1_iowrite32(params_vdev, spatial_coeff,
			 CIF_ISP_DPF_S_WEIGHT_G_1_4);

	spatial_coeff = arg->g_flt.spatial_coeff[4] |
			(arg->g_flt.spatial_coeff[5] << 8);
	rkisp1_iowrite32(params_vdev, spatial_coeff,
			 CIF_ISP_DPF_S_WEIGHT_G_5_6);

	spatial_coeff = arg->rb_flt.spatial_coeff[0] |
			(arg->rb_flt.spatial_coeff[1] << 8) |
			(arg->rb_flt.spatial_coeff[2] << 16) |
			(arg->rb_flt.spatial_coeff[3] << 24);
	rkisp1_iowrite32(params_vdev, spatial_coeff,
			 CIF_ISP_DPF_S_WEIGHT_RB_1_4);

	spatial_coeff = arg->rb_flt.spatial_coeff[4] |
			(arg->rb_flt.spatial_coeff[5] << 8);
	rkisp1_iowrite32(params_vdev, spatial_coeff,
			CIF_ISP_DPF_S_WEIGHT_RB_5_6);
}

static void dpf_strength_config(struct rkisp1_isp_params_vdev *params_vdev,
				struct cifisp_dpf_strength_config *arg)
{
	rkisp1_iowrite32(params_vdev, arg->b, CIF_ISP_DPF_STRENGTH_B);
	rkisp1_iowrite32(params_vdev, arg->g, CIF_ISP_DPF_STRENGTH_G);
	rkisp1_iowrite32(params_vdev, arg->r, CIF_ISP_DPF_STRENGTH_R);
}

static __maybe_unused
void __isp_isr_other_config(struct rkisp1_isp_params_vdev *params_vdev,
			    struct  rkisp1_isp_params_cfg *new_params)
{
	unsigned int module_en_update, module_cfg_update, module_ens;

	module_en_update = new_params->module_en_update;
	module_cfg_update = new_params->module_cfg_update;
	module_ens = new_params->module_ens;

	if ((module_en_update & CIFISP_MODULE_DPCC) ||
	    (module_cfg_update & CIFISP_MODULE_DPCC)) {
		/*update dpc config */
		if ((module_cfg_update & CIFISP_MODULE_DPCC))
			dpcc_config(params_vdev,
				    &new_params->others.dpcc_config);

		if (module_en_update & CIFISP_MODULE_DPCC) {
			if (!!(module_ens & CIFISP_MODULE_DPCC))
				isp_param_set_bits(params_vdev,
						   CIF_ISP_DPCC_MODE,
						   CIF_ISP_DPCC_ENA);
			else
				isp_param_clear_bits(params_vdev,
						     CIF_ISP_DPCC_MODE,
						     CIF_ISP_DPCC_ENA);
		}
	}

	if ((module_en_update & CIFISP_MODULE_BLS) ||
	    (module_cfg_update & CIFISP_MODULE_BLS)) {
		/* update bls config */
		if ((module_cfg_update & CIFISP_MODULE_BLS))
			bls_config(params_vdev, &new_params->others.bls_config);

		if (module_en_update & CIFISP_MODULE_BLS) {
			if (!!(module_ens & CIFISP_MODULE_BLS))
				isp_param_set_bits(params_vdev,
						   CIF_ISP_BLS_CTRL,
						   CIF_ISP_BLS_ENA);
			else
				isp_param_clear_bits(params_vdev,
						     CIF_ISP_BLS_CTRL,
						     CIF_ISP_BLS_ENA);
		}
	}

	if ((module_en_update & CIFISP_MODULE_SDG) ||
	    (module_cfg_update & CIFISP_MODULE_SDG)) {
		/* update sdg config */
		if ((module_cfg_update & CIFISP_MODULE_SDG))
			sdg_config(params_vdev, &new_params->others.sdg_config);

		if (module_en_update & CIFISP_MODULE_SDG) {
			if (!!(module_ens & CIFISP_MODULE_SDG))
				isp_param_set_bits(params_vdev,
						CIF_ISP_CTRL,
						CIF_ISP_CTRL_ISP_GAMMA_IN_ENA);
			else
				isp_param_clear_bits(params_vdev,
						CIF_ISP_CTRL,
						CIF_ISP_CTRL_ISP_GAMMA_IN_ENA);
		}
	}

	if ((module_en_update & CIFISP_MODULE_LSC) ||
	    (module_cfg_update & CIFISP_MODULE_LSC)) {
		/* update lsc config */
		if ((module_cfg_update & CIFISP_MODULE_LSC))
			lsc_config(params_vdev, &new_params->others.lsc_config);

		if (module_en_update & CIFISP_MODULE_LSC) {
			if (!!(module_ens & CIFISP_MODULE_LSC))
				isp_param_set_bits(params_vdev,
						   CIF_ISP_LSC_CTRL,
						   CIF_ISP_LSC_CTRL_ENA);
			else
				isp_param_clear_bits(params_vdev,
						     CIF_ISP_LSC_CTRL,
						     CIF_ISP_LSC_CTRL_ENA);
		}
	}

	if ((module_en_update & CIFISP_MODULE_AWB_GAIN) ||
	    (module_cfg_update & CIFISP_MODULE_AWB_GAIN)) {
		/* update awb gains */
		if ((module_cfg_update & CIFISP_MODULE_AWB_GAIN))
			awb_gain_config(params_vdev,
					&new_params->others.awb_gain_config);

		if (module_en_update & CIFISP_MODULE_AWB_GAIN) {
			if (!!(module_ens & CIFISP_MODULE_AWB_GAIN))
				isp_param_set_bits(params_vdev,
						   CIF_ISP_CTRL,
						   CIF_ISP_CTRL_ISP_AWB_ENA);
			else
				isp_param_clear_bits(params_vdev,
						     CIF_ISP_CTRL,
						     CIF_ISP_CTRL_ISP_AWB_ENA);
		}
	}

	if ((module_en_update & CIFISP_MODULE_BDM) ||
	    (module_cfg_update & CIFISP_MODULE_BDM)) {
		/* update bdm config */
		if ((module_cfg_update & CIFISP_MODULE_BDM))
			bdm_config(params_vdev, &new_params->others.bdm_config);

		if (module_en_update & CIFISP_MODULE_BDM) {
			if (!!(module_ens & CIFISP_MODULE_BDM))
				isp_param_set_bits(params_vdev,
						   CIF_ISP_DEMOSAIC,
						   CIF_ISP_DEMOSAIC_BYPASS);
			else
				isp_param_clear_bits(params_vdev,
						     CIF_ISP_DEMOSAIC,
						     CIF_ISP_DEMOSAIC_BYPASS);
		}
	}

	if ((module_en_update & CIFISP_MODULE_FLT) ||
	    (module_cfg_update & CIFISP_MODULE_FLT)) {
		/* update filter config */
		if ((module_cfg_update & CIFISP_MODULE_FLT))
			flt_config(params_vdev, &new_params->others.flt_config);

		if (module_en_update & CIFISP_MODULE_FLT) {
			if (!!(module_ens & CIFISP_MODULE_FLT))
				isp_param_set_bits(params_vdev,
						   CIF_ISP_FILT_MODE,
						   CIF_ISP_FLT_ENA);
			else
				isp_param_clear_bits(params_vdev,
						     CIF_ISP_FILT_MODE,
						     CIF_ISP_FLT_ENA);
		}
	}

	if ((module_en_update & CIFISP_MODULE_CTK) ||
	    (module_cfg_update & CIFISP_MODULE_CTK)) {
		/* update ctk config */
		if ((module_cfg_update & CIFISP_MODULE_CTK))
			ctk_config(params_vdev, &new_params->others.ctk_config);

		if (module_en_update & CIFISP_MODULE_CTK)
			ctk_enable(params_vdev,
				   !!(module_ens & CIFISP_MODULE_CTK));
	}

	if ((module_en_update & CIFISP_MODULE_GOC) ||
	    (module_cfg_update & CIFISP_MODULE_GOC)) {
		/* update goc config */
		if ((module_cfg_update & CIFISP_MODULE_CTK))
			goc_config(params_vdev, &new_params->others.goc_config);

		if (module_en_update & CIFISP_MODULE_GOC) {
			if (!!(module_ens & CIFISP_MODULE_GOC))
				isp_param_set_bits(params_vdev,
						CIF_ISP_CTRL,
						CIF_ISP_CTRL_ISP_GAMMA_OUT_ENA);
			else
				isp_param_clear_bits(params_vdev,
						CIF_ISP_CTRL,
						CIF_ISP_CTRL_ISP_GAMMA_OUT_ENA);
		}
	}

	if ((module_en_update & CIFISP_MODULE_CPROC) ||
	    (module_cfg_update & CIFISP_MODULE_CPROC)) {
		/* update cprc config */
		if ((module_cfg_update & CIFISP_MODULE_CPROC)) {
			cproc_config(params_vdev,
				     &new_params->others.cproc_config);
			/* TODO: get range */
			csm_config(params_vdev, true);
		}

		if (module_en_update & CIFISP_MODULE_CPROC) {
			if (!!(module_ens & CIFISP_MODULE_CPROC))
				isp_param_set_bits(params_vdev,
						   CIF_C_PROC_CTRL,
						   CIF_C_PROC_CTR_ENABLE);
			else
				isp_param_clear_bits(params_vdev,
						   CIF_C_PROC_CTRL,
						   CIF_C_PROC_CTR_ENABLE);
		}
	}

	if ((module_en_update & CIFISP_MODULE_IE) ||
	    (module_cfg_update & CIFISP_MODULE_IE)) {
		/* update ie config */
		if ((module_cfg_update & CIFISP_MODULE_IE))
			ie_config(params_vdev, &new_params->others.ie_config);

		if (module_en_update & CIFISP_MODULE_IE)
			ie_enable(params_vdev,
				   !!(module_ens & CIFISP_MODULE_IE));
	}

	if ((module_en_update & CIFISP_MODULE_DPF) ||
	    (module_cfg_update & CIFISP_MODULE_DPF)) {
		/* update dpf  config */
		if ((module_cfg_update & CIFISP_MODULE_DPF))
			dpf_config(params_vdev, &new_params->others.dpf_config);

		if (module_en_update & CIFISP_MODULE_DPF) {
			if (!!(module_ens & CIFISP_MODULE_DPF))
				isp_param_set_bits(params_vdev,
						   CIF_ISP_DPF_MODE,
						   CIF_ISP_DPF_MODE_EN);
			else
				isp_param_clear_bits(params_vdev,
						     CIF_ISP_DPF_MODE,
						     CIF_ISP_DPF_MODE_EN);
		}
	}

	if ((module_en_update & CIFISP_MODULE_DPF_STRENGTH) ||
	    (module_cfg_update & CIFISP_MODULE_DPF_STRENGTH)) {
		/* update dpf strength config */
		dpf_strength_config(params_vdev,
				    &new_params->others.dpf_strength_config);
	}
}

static __maybe_unused
void __isp_isr_meas_config(struct rkisp1_isp_params_vdev *params_vdev,
			   struct  rkisp1_isp_params_cfg *new_params)
{
	unsigned int module_en_update, module_cfg_update, module_ens;

	module_en_update = new_params->module_en_update;
	module_cfg_update = new_params->module_cfg_update;
	module_ens = new_params->module_ens;

	if ((module_en_update & CIFISP_MODULE_AWB) ||
	    (module_cfg_update & CIFISP_MODULE_AWB)) {
		/* update awb config */
		if ((module_cfg_update & CIFISP_MODULE_AWB))
			awb_meas_config(params_vdev,
					&new_params->meas.awb_meas_config);

		if (module_en_update & CIFISP_MODULE_AWB)
			awb_meas_enable(params_vdev,
					&new_params->meas.awb_meas_config,
					!!(module_ens & CIFISP_MODULE_AWB));
	}

	if ((module_en_update & CIFISP_MODULE_AFC) ||
	    (module_cfg_update & CIFISP_MODULE_AFC)) {
		/* update afc config */
		if ((module_cfg_update & CIFISP_MODULE_AFC))
			afm_config(params_vdev, &new_params->meas.afc_config);

		if (module_en_update & CIFISP_MODULE_AFC) {
			if (!!(module_ens & CIFISP_MODULE_AFC))
				isp_param_set_bits(params_vdev,
						   CIF_ISP_AFM_CTRL,
						   CIF_ISP_AFM_ENA);
			else
				isp_param_clear_bits(params_vdev,
						     CIF_ISP_AFM_CTRL,
						     CIF_ISP_AFM_ENA);
		}
	}

	if ((module_en_update & CIFISP_MODULE_HST) ||
	    (module_cfg_update & CIFISP_MODULE_HST)) {
		/* update hst config */
		if ((module_cfg_update & CIFISP_MODULE_HST))
			hst_config(params_vdev, &new_params->meas.hst_config);

		if (module_en_update & CIFISP_MODULE_HST)
			hst_enable(params_vdev,
				   &new_params->meas.hst_config,
				   !!(module_ens & CIFISP_MODULE_HST));
	}

	if ((module_en_update & CIFISP_MODULE_AEC) ||
	    (module_cfg_update & CIFISP_MODULE_AEC)) {
		/* update aec config */
		if ((module_cfg_update & CIFISP_MODULE_AEC))
			aec_config(params_vdev, &new_params->meas.aec_config);

		if (module_en_update & CIFISP_MODULE_AEC) {
			if (!!(module_ens & CIFISP_MODULE_AEC))
				isp_param_set_bits(params_vdev,
						   CIF_ISP_EXP_CTRL,
						   CIF_ISP_EXP_ENA);
			else
				isp_param_clear_bits(params_vdev,
						     CIF_ISP_EXP_CTRL,
						     CIF_ISP_EXP_ENA);
		}
	}
}

void rkisp1_params_isr(struct rkisp1_isp_params_vdev *params_vdev, u32 isp_mis)
{
	struct rkisp1_isp_params_cfg *new_params;
	struct rkisp1_buffer *cur_buf = NULL;

	spin_lock(&params_vdev->config_lock);
	if (!params_vdev->streamon) {
		spin_unlock(&params_vdev->config_lock);
		return;
	}

	/* get one empty buffer */
	if (!list_empty(&params_vdev->params))
		cur_buf = list_first_entry(&params_vdev->params,
					   struct rkisp1_buffer, queue);
	spin_unlock(&params_vdev->config_lock);

	if (!cur_buf)
		return;

	new_params = (struct rkisp1_isp_params_cfg *)(cur_buf->vaddr[0]);

	if (isp_mis & CIF_ISP_FRAME) {
		/* __isp_isr_other_config(params_vdev, new_params); */
		/* __isp_isr_meas_config(params_vdev, new_params); */
		spin_lock(&params_vdev->config_lock);
		list_del(&cur_buf->queue);
		spin_unlock(&params_vdev->config_lock);
		vb2_buffer_done(&cur_buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
	}
}

static struct cifisp_awb_meas_config awb_params_default_config = {
		{0, 0, RKISP1_DEFAULT_WIDTH, RKISP1_DEFAULT_HEIGHT},
		CIFISP_AWB_MODE_YCBCR, 200, 30, 20, 20, 0, 128, 128
};

static struct cifisp_aec_config aec_params_default_config = {
		CIFISP_EXP_MEASURING_MODE_0,
		CIFISP_EXP_CTRL_AUTOSTOP_0,
		{
			RKISP1_DEFAULT_WIDTH >> 2, RKISP1_DEFAULT_HEIGHT >> 2,
			RKISP1_DEFAULT_WIDTH >> 1, RKISP1_DEFAULT_HEIGHT >> 1
		}
};

static struct cifisp_hst_config hst_params_default_config = {
		CIFISP_HISTOGRAM_MODE_RGB_COMBINED,
		3,
		{
			RKISP1_DEFAULT_WIDTH >> 2, RKISP1_DEFAULT_HEIGHT >> 2,
			RKISP1_DEFAULT_WIDTH >> 1, RKISP1_DEFAULT_HEIGHT >> 1
		}
};

static struct cifisp_afc_config afc_params_default_config = {
	1,
	{
		{
			300, 225, 200, 150
		}
	},
	4,
	14
};

static
void rkisp1_params_config_parameter(struct rkisp1_isp_params_vdev *params_vdev)
{
	spin_lock(&params_vdev->config_lock);

	awb_meas_config(params_vdev, &awb_params_default_config);
	awb_meas_enable(params_vdev, &awb_params_default_config, true);

	aec_config(params_vdev, &aec_params_default_config);
	isp_param_set_bits(params_vdev, CIF_ISP_EXP_CTRL, CIF_ISP_EXP_ENA);

	afm_config(params_vdev, &afc_params_default_config);
	isp_param_set_bits(params_vdev, CIF_ISP_AFM_CTRL, CIF_ISP_AFM_ENA);

	memset(hst_params_default_config.hist_weight, 0x01,
	       sizeof(hst_params_default_config.hist_weight));
	hst_config(params_vdev, &hst_params_default_config);
	isp_param_set_bits(params_vdev, CIF_ISP_HIST_PROP,
			   ~CIF_ISP_HIST_PROP_MODE_MASK |
			   hst_params_default_config.mode);
	spin_unlock(&params_vdev->config_lock);
}

/* Not called when the camera active, thus not isr protection. */
void rkisp1_configure_isp(struct rkisp1_isp_params_vdev *params_vdev,
			  struct ispsd_in_fmt *in_fmt,
			  enum v4l2_quantization quantization)
{
	/*TODO: default config ?  */
	rkisp1_params_config_parameter(params_vdev);
}

/* Not called when the camera active, thus not isr protection. */
void rkisp1_disable_isp(struct rkisp1_isp_params_vdev *params_vdev)
{
	isp_param_clear_bits(params_vdev, CIF_ISP_DPCC_MODE, CIF_ISP_DPCC_ENA);
	isp_param_clear_bits(params_vdev, CIF_ISP_LSC_CTRL,
			     CIF_ISP_LSC_CTRL_ENA);
	isp_param_clear_bits(params_vdev, CIF_ISP_BLS_CTRL, CIF_ISP_BLS_ENA);
	isp_param_clear_bits(params_vdev, CIF_ISP_CTRL,
			     CIF_ISP_CTRL_ISP_GAMMA_IN_ENA);
	isp_param_clear_bits(params_vdev, CIF_ISP_CTRL,
			     CIF_ISP_CTRL_ISP_GAMMA_OUT_ENA);
	isp_param_clear_bits(params_vdev, CIF_ISP_DEMOSAIC,
			     CIF_ISP_DEMOSAIC_BYPASS);
	isp_param_clear_bits(params_vdev, CIF_ISP_FILT_MODE, CIF_ISP_FLT_ENA);
	awb_meas_enable(params_vdev, NULL, false);
	isp_param_clear_bits(params_vdev, CIF_ISP_CTRL,
			     CIF_ISP_CTRL_ISP_AWB_ENA);
	isp_param_clear_bits(params_vdev, CIF_ISP_EXP_CTRL, CIF_ISP_EXP_ENA);
	ctk_enable(params_vdev, false);
	isp_param_clear_bits(params_vdev, CIF_C_PROC_CTRL,
			     CIF_C_PROC_CTR_ENABLE);
	hst_enable(params_vdev, NULL, false);
	isp_param_clear_bits(params_vdev, CIF_ISP_AFM_CTRL, CIF_ISP_AFM_ENA);
	ie_enable(params_vdev, false);
	isp_param_clear_bits(params_vdev, CIF_ISP_DPF_MODE,
			     CIF_ISP_DPF_MODE_EN);
}

static int
__check_params(struct rkisp1_isp_params_vdev *params_vdev,
	       struct rkisp1_isp_params_cfg *configs)
{
	int ret;

	if (configs->module_cfg_update & CIFISP_MODULE_DPCC) {
		ret = dpcc_param_check(params_vdev,
				       &configs->others.dpcc_config);
		if (ret < 0)
			return ret;
	}

	if (configs->module_cfg_update & CIFISP_MODULE_BLS) {
		ret = bls_param_check(params_vdev,
				      &configs->others.bls_config);
		if (ret < 0)
			return ret;
	}

	if (configs->module_cfg_update & CIFISP_MODULE_SDG) {
		ret = sdg_param_check(params_vdev,
				      &configs->others.sdg_config);
		if (ret < 0)
			return ret;
	}

	if (configs->module_cfg_update & CIFISP_MODULE_LSC) {
		ret = lsc_param_check(params_vdev,
				      &configs->others.lsc_config);
		if (ret < 0)
			return ret;
	}

	if (configs->module_cfg_update & CIFISP_MODULE_AWB_GAIN) {
		ret = awb_gain_param_check(params_vdev,
					   &configs->others.awb_gain_config);
		if (ret < 0)
			return ret;

	}

	if (configs->module_cfg_update & CIFISP_MODULE_FLT) {
		ret = flt_param_check(params_vdev,
				      &configs->others.flt_config);
		if (ret < 0)
			return ret;
	}

	if (configs->module_cfg_update & CIFISP_MODULE_CTK) {
		ret = ctk_param_check(params_vdev,
				      &configs->others.ctk_config);
		if (ret < 0)
			return ret;
	}

	if (configs->module_cfg_update & CIFISP_MODULE_GOC) {
		ret = goc_param_check(params_vdev,
				      &configs->others.goc_config);
		if (ret < 0)
			return ret;
	}

	if (configs->module_cfg_update & CIFISP_MODULE_CPROC) {
		ret = cproc_param_check(params_vdev,
					&configs->others.cproc_config);
		if (ret < 0)
			return ret;
	}

	if (configs->module_cfg_update & CIFISP_MODULE_IE) {
		ret = ie_param_check(params_vdev,
				     &configs->others.ie_config);
		if (ret < 0)
			return ret;
	}

	if (configs->module_cfg_update & CIFISP_MODULE_DPF) {
		ret = dpf_param_check(params_vdev,
				      &configs->others.dpf_config);
		if (ret < 0)
			return ret;
	}

	if (configs->module_cfg_update & CIFISP_MODULE_AWB) {
		ret = awb_meas_param_check(params_vdev,
					   &configs->meas.awb_meas_config);
		if (ret < 0)
			return ret;
	}

	if (configs->module_cfg_update & CIFISP_MODULE_AFC) {
		ret = afm_param_check(params_vdev,
				      &configs->meas.afc_config);
		if (ret < 0)
			return ret;
	}

	if (configs->module_cfg_update & CIFISP_MODULE_HST) {
		ret = hst_param_check(params_vdev,
				      &configs->meas.hst_config);
		if (ret < 0)
			return ret;
	}

	if (configs->module_cfg_update & CIFISP_MODULE_AEC) {
		ret = aec_param_check(params_vdev,
				      &configs->meas.aec_config);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/* TODO: should add RKISP1_PARAMS_FMT pixel format  to V4L2 framework ? */
static int rkisp1_params_enum_fmt_meta_out(struct file *file, void *priv,
					   struct v4l2_fmtdesc *f)
{
	struct video_device *video = video_devdata(file);
	struct rkisp1_isp_params_vdev *params_vdev = video_get_drvdata(video);

	if (f->index > 0 || f->type != video->queue->type)
		return -EINVAL;

	f->pixelformat = params_vdev->vdev_fmt.fmt.meta.dataformat;

	return 0;
}

static int rkisp1_params_g_fmt_meta_out(struct file *file, void *fh,
					struct v4l2_format *f)
{
	struct video_device *video = video_devdata(file);
	struct rkisp1_isp_params_vdev *params_vdev = video_get_drvdata(video);
	struct v4l2_meta_format *meta = &f->fmt.meta;

	if (f->type != video->queue->type)
		return -EINVAL;

	memset(meta, 0, sizeof(*meta));
	meta->dataformat = params_vdev->vdev_fmt.fmt.meta.dataformat;
	meta->buffersize = params_vdev->vdev_fmt.fmt.meta.buffersize;

	return 0;
}

static int rkisp1_params_querycap(struct file *file,
				  void *priv, struct v4l2_capability *cap)
{
	struct video_device *vdev = video_devdata(file);

	strcpy(cap->driver, DRIVER_NAME);
	strlcpy(cap->card, vdev->name, sizeof(cap->card));
	strlcpy(cap->bus_info, "platform: " DRIVER_NAME, sizeof(cap->bus_info));

	return 0;
}

/* ISP params video device IOCTLs */
static const struct v4l2_ioctl_ops rkisp1_params_ioctl = {
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_enum_fmt_meta_out = rkisp1_params_enum_fmt_meta_out,
	.vidioc_g_fmt_meta_out = rkisp1_params_g_fmt_meta_out,
	.vidioc_s_fmt_meta_out = rkisp1_params_g_fmt_meta_out,
	.vidioc_try_fmt_meta_out = rkisp1_params_g_fmt_meta_out,
	.vidioc_querycap = rkisp1_params_querycap
};

static int rkisp1_params_vb2_queue_setup(struct vb2_queue *vq,
					 const void *parg,
					 unsigned int *num_buffers,
					 unsigned int *num_planes,
					 unsigned int sizes[],
					 void *alloc_ctxs[])
{
	struct rkisp1_isp_params_vdev *params_vdev = vq->drv_priv;

	*num_buffers = clamp_t(u32, *num_buffers,
			       RKISP1_ISP_PARAMS_REQ_BUFS_MIN,
			       RKISP1_ISP_PARAMS_REQ_BUFS_MAX);

	*num_planes = 1;

	sizes[0] = sizeof(struct rkisp1_isp_params_cfg);

	INIT_LIST_HEAD(&params_vdev->params);
	params_vdev->first_params = true;

	return 0;
}

static void rkisp1_params_vb2_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct rkisp1_buffer *params_buf = to_rkisp1_buffer(vbuf);
	struct vb2_queue *vq = vb->vb2_queue;
	struct rkisp1_isp_params_vdev *params_vdev = vq->drv_priv;
	struct rkisp1_isp_params_cfg *new_params;
	unsigned long flags;

	if (__check_params(params_vdev, vb2_plane_vaddr(vb, 0))) {
		v4l2_err(params_vdev->vnode.vdev.v4l2_dev, "check new param error!\n");
		vb2_buffer_done(&params_buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		return;
	}

	if (params_vdev->first_params) {
		new_params = (struct rkisp1_isp_params_cfg *)
			(vb2_plane_vaddr(vb, 0));
		spin_lock_irqsave(&params_vdev->config_lock, flags);
		/* __isp_isr_other_config(params_vdev, new_params); */
		/* __isp_isr_meas_config(params_vdev, new_params); */
		spin_unlock_irqrestore(&params_vdev->config_lock, flags);
		vb2_buffer_done(&params_buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
		params_vdev->first_params = false;
		params_vdev->cur_params = *new_params;
		return;
	}

	params_buf->vaddr[0] = vb2_plane_vaddr(vb, 0);
	spin_lock_irqsave(&params_vdev->config_lock, flags);
	list_add_tail(&params_buf->queue, &params_vdev->params);
	spin_unlock_irqrestore(&params_vdev->config_lock, flags);
}

static void rkisp1_params_vb2_stop_streaming(struct vb2_queue *vq)
{
	struct rkisp1_isp_params_vdev *params_vdev = vq->drv_priv;
	struct rkisp1_buffer *buf;
	unsigned long flags;
	int i;

	/* stop params input firstly */
	spin_lock_irqsave(&params_vdev->config_lock, flags);
	params_vdev->streamon = false;
	spin_unlock_irqrestore(&params_vdev->config_lock, flags);

	for (i = 0; i < RKISP1_ISP_PARAMS_REQ_BUFS_MAX; i++) {
		spin_lock_irqsave(&params_vdev->config_lock, flags);
		if (!list_empty(&params_vdev->params)) {
			buf = list_first_entry(&params_vdev->params,
					       struct rkisp1_buffer, queue);
			list_del(&buf->queue);
			spin_unlock_irqrestore(&params_vdev->config_lock,
					       flags);
		} else {
			spin_unlock_irqrestore(&params_vdev->config_lock,
					       flags);
			break;
		}

		if (buf)
			vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
		buf = NULL;
	}
}

static int
rkisp1_params_vb2_start_streaming(struct vb2_queue *queue, unsigned int count)
{
	struct rkisp1_isp_params_vdev *params_vdev = queue->drv_priv;
	unsigned long flags;

	spin_lock_irqsave(&params_vdev->config_lock, flags);
	params_vdev->streamon = true;
	spin_unlock_irqrestore(&params_vdev->config_lock, flags);

	return 0;
}

static struct vb2_ops rkisp1_params_vb2_ops = {
	.queue_setup = rkisp1_params_vb2_queue_setup,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
	.buf_queue = rkisp1_params_vb2_buf_queue,
	.start_streaming = rkisp1_params_vb2_start_streaming,
	.stop_streaming = rkisp1_params_vb2_stop_streaming,

};

struct v4l2_file_operations rkisp1_params_fops = {
	.mmap = vb2_fop_mmap,
	.unlocked_ioctl = video_ioctl2,
	.poll = vb2_fop_poll,
	.open = v4l2_fh_open,
	.release = vb2_fop_release
};

static int
rkisp1_params_init_vb2_queue(struct vb2_queue *q,
			     struct rkisp1_isp_params_vdev *params_vdev)
{
	struct rkisp1_vdev_node *node;

	node = queue_to_node(q);

	q->type = V4L2_BUF_TYPE_META_OUTPUT;
	q->io_modes = VB2_MMAP | VB2_USERPTR;
	q->drv_priv = params_vdev;
	q->ops = &rkisp1_params_vb2_ops;
	q->mem_ops = &vb2_vmalloc_memops;
	q->buf_struct_size = sizeof(struct rkisp1_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &node->vlock;

	return vb2_queue_init(q);
}

static void rkisp1_init_params_vdev(struct rkisp1_isp_params_vdev *params_vdev)
{
	params_vdev->vdev_fmt.fmt.meta.dataformat =
		V4L2_META_FMT_RK_ISP1_PARAMS;
	params_vdev->vdev_fmt.fmt.meta.buffersize =
		sizeof(struct rkisp1_isp_params_cfg);
}

int rkisp1_register_params_vdev(struct rkisp1_isp_params_vdev *params_vdev,
				struct v4l2_device *v4l2_dev,
				struct rkisp1_device *dev)
{
	int ret;
	struct rkisp1_vdev_node *node = &params_vdev->vnode;
	struct video_device *vdev = &node->vdev;

	params_vdev->dev = dev;
	mutex_init(&node->vlock);
	spin_lock_init(&params_vdev->config_lock);

	strlcpy(vdev->name, "rkisp1-input-params", sizeof(vdev->name));

	video_set_drvdata(vdev, params_vdev);
	vdev->ioctl_ops = &rkisp1_params_ioctl;
	vdev->fops = &rkisp1_params_fops;
	vdev->release = video_device_release_empty;
	/*
	 * Provide a mutex to v4l2 core. It will be used
	 * to protect all fops and v4l2 ioctls.
	 */
	vdev->lock = &node->vlock;
	vdev->v4l2_dev = v4l2_dev;
	vdev->queue = &node->buf_queue;
	vdev->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_META_OUTPUT;
	vdev->vfl_dir = VFL_DIR_TX;
	rkisp1_params_init_vb2_queue(vdev->queue, params_vdev);
	rkisp1_init_params_vdev(params_vdev);
	video_set_drvdata(vdev, params_vdev);

	node->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_init(&vdev->entity, 1, &node->pad, 0);
	if (ret < 0)
		goto err_release_queue;
	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret < 0) {
		dev_err(&vdev->dev,
			"could not register Video for Linux device\n");
		goto err_cleanup_media_entity;
	}
	return 0;
err_cleanup_media_entity:
	media_entity_cleanup(&vdev->entity);
err_release_queue:
	vb2_queue_release(vdev->queue);
	return ret;
}

void rkisp1_unregister_params_vdev(struct rkisp1_isp_params_vdev *params_vdev)
{
	struct rkisp1_vdev_node *node = &params_vdev->vnode;
	struct video_device *vdev = &node->vdev;

	video_unregister_device(vdev);
	media_entity_cleanup(&vdev->entity);
	vb2_queue_release(vdev->queue);
}
