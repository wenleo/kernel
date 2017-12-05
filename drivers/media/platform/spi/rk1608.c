/*
* Rockchip rk1608 driver
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
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-of.h>
#include "rk1608.h"

struct rk1608_state {
	struct v4l2_subdev		sd;
	struct v4l2_subdev		*cam_sd;
	struct spi_device		*spi;
	struct device			*dev;
	struct media_pad		pad;
	struct clk			*mclk;
	struct mutex			lock;		/* protect resource */
	struct mutex			send_msg_lock;	/* protect msg */
	int power_count;
	int reset_gpio;
	int reset_active;
	int irq_gpio;
	int irq;
	int sleepst_gpio;
	int sleepst_irq;
	int wakeup_gpio;
	int wakeup_active;
	int powerdown_gpio;
	int powerdown_active;
	int msg_num;
	u32 sensor_cnt;
	u32 cam_nums;
	u32 max_speed_hz;
	u32 min_speed_hz;
	atomic_t			msg_done[8];
	wait_queue_head_t		msg_wait;
	struct v4l2_ctrl_handler	ctrl_handler;
	struct v4l2_ctrl		*link_freq;
};

static const s64 link_freq_menu_items[] = {
	1000000000
};

static inline struct rk1608_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct rk1608_state, sd);
}

/**
 * rk1608_operation_query - RK1608_2_APB last operation state query
 *
 * @spi: device from which data will be read
 * @state: last operation state [out]
 * Context: can sleep
 *
 * It returns zero on success, else a negative error code.
 */
int rk1608_operation_query(struct spi_device *spi, s32 *state)
{
	s32 query_cmd = APB_CMD_QUERY;
	struct spi_transfer query_cmd_packet = {
		.tx_buf = &query_cmd,
		.len    = sizeof(query_cmd),
	};
	struct spi_transfer state_packet = {
		.rx_buf = state,
		.len    = sizeof(*state),
	};
	struct spi_message  m;

	spi_message_init(&m);
	spi_message_add_tail(&query_cmd_packet, &m);
	spi_message_add_tail(&state_packet, &m);
	spi_sync(spi, &m);

	return ((*state & APB_OP_STATE_ID_MASK) == APB_OP_STATE_ID) ? 0 : -1;
}

int rk1608_write(struct spi_device *spi,
		 s32 addr, const s32 *data, size_t data_len)
{
	s32 write_cmd = APB_CMD_WRITE;
	struct spi_transfer write_cmd_packet = {
		.tx_buf = &write_cmd,
		.len    = sizeof(write_cmd),
	};
	struct spi_transfer addr_packet = {
		.tx_buf = &addr,
		.len    = sizeof(addr),
	};
	struct spi_transfer data_packet = {
		.tx_buf = data,
		.len    = data_len,
	};
	struct spi_message  m;

	spi_message_init(&m);
	spi_message_add_tail(&write_cmd_packet, &m);
	spi_message_add_tail(&addr_packet, &m);
	spi_message_add_tail(&data_packet, &m);
	return spi_sync(spi, &m);
}

/**
 * _rk1608_safe_write - RK1608_2_APB synchronous write with state check
 *
 * @spi: spi device
 * @addr: apb resource address
 * @data: data buffer
 * @data_len: data buffer size, in bytes
 * Context: can sleep
 *
 * It returns zero on success, else operation state code.
 */
static int _rk1608_safe_write(struct spi_device *spi,
			      s32 addr, const s32 *data, size_t data_len)
{
	s32 state = 0;
	s32 try = 0;

	do {
		rk1608_write(spi, addr, data, data_len);
		if (rk1608_operation_query(spi, &state) != 0) {
			dev_err(&spi->dev,
				"rk1608_operation_query_state =%d\n", state);
			return -1;
		} else if ((state & APB_OP_STATE_MASK) == 0) {
			break;
		}

		if (try++ == APB_SAFE_OPERATION_TRY_MAX)
			break;
		udelay(APB_SAFE_OPERATION_TRY_DELAY_US);
	} while (1);

	return (state & APB_OP_STATE_MASK);
}

/**
 * rk1608_safe_write - RK1608_2_APB synchronous write with state check
 *
 * @spi: spi device
 * @addr: apb resource address
 * @data: data buffer
 * @data_len: data buffer size, in bytes
 * Context: can sleep
 *
 * It returns zero on success, else operation state code.
 */
int rk1608_safe_write(struct spi_device *spi,
		      s32 addr, const s32 *data, size_t data_len)
{
	int ret = 0;

	while (data_len > 0) {
		size_t slen = MIN(data_len, APB_MAX_OP_BYTES);

		ret = _rk1608_safe_write(spi, addr, data, slen);
		if (ret) {
			dev_err(&spi->dev, "rk1608_safe_write err =%d\n", ret);
			break;
		}

		data_len = data_len - slen;
		data = (s32 *)((s8 *)data + slen);
		addr += slen;
	}
	return ret;
}

/**
 * rk1608_safe_w32 - RK1608_2_APB synchronous 32-bit write with state check
 *
 * @spi: spi device
 * @addr: apb resource address
 * @data: 32-bit data
 * Context: can sleep
 *
 * It returns zero on success, else operation state code.
 */
int rk1608_safe_w32(struct spi_device *spi,
		    s32 addr, s32 data)
{
	return rk1608_safe_write(spi, addr, &data, 4);
}

void rk1608_hw_init(struct spi_device *spi)
{
	/* modify rk1608 spi slave clk to 300M */
	rk1608_safe_w32(spi, CRUPMU_CLKSEL14_CON, SPI0_PLL_SEL_APLL);

	/* modify rk1608 spi io driver strength to 8mA */
	rk1608_safe_w32(spi, PMUGRF_GPIO1A_E, BIT7_6_SEL_8MA);
	rk1608_safe_w32(spi, PMUGRF_GPIO1B_E, BIT1_0_SEL_8MA);
}

/**
 * rk1608_read - RK1608_2_APB synchronous read
 *
 * @spi: spi device
 * @addr: apb resource address
 * @data: data buffer [out]
 * @data_len: data buffer size, in bytes
 * Context: can sleep
 *
 * It returns zero on success, else a negative error code.
 */
int rk1608_read(struct spi_device *spi,
		s32 addr, s32 *data, size_t data_len)
{
	s32 real_len = MIN(data_len, APB_MAX_OP_BYTES);
	s32 read_cmd = APB_CMD_READ | (real_len << 14 & APB_OP_STATE_ID_MASK);
	s32 read_begin_cmd = APB_CMD_READ_BEGIN;
	s32 dummy = 0;
	struct spi_transfer read_cmd_packet = {
		.tx_buf = &read_cmd,
		.len    = sizeof(read_cmd),
	};
	struct spi_transfer addr_packet = {
		.tx_buf = &addr,
		.len    = sizeof(addr),
	};
	struct spi_transfer read_dummy_packet = {
		.tx_buf = &dummy,
		.len    = sizeof(dummy),
	};
	struct spi_transfer read_begin_cmd_packet = {
		.tx_buf = &read_begin_cmd,
		.len    = sizeof(read_begin_cmd),
	};
	struct spi_transfer data_packet = {
		.rx_buf = data,
		.len    = data_len,
	};
	struct spi_message  m;

	spi_message_init(&m);
	spi_message_add_tail(&read_cmd_packet, &m);
	spi_message_add_tail(&addr_packet, &m);
	spi_message_add_tail(&read_dummy_packet, &m);
	spi_message_add_tail(&read_begin_cmd_packet, &m);
	spi_message_add_tail(&data_packet, &m);
	return spi_sync(spi, &m);
}

/**
 * rk1608_safe_read - RK1608_2_APB synchronous read with state check
 *
 * @spi: spi device
 * @addr: apb resource address
 * @data: data buffer [out]
 * @data_len: data buffer size, in bytes
 * Context: can sleep
 *
 * It returns zero on success, else operation state code.
 */
int rk1608_safe_read(struct spi_device *spi,
		     s32 addr, s32 *data, size_t data_len)
{
	s32 state = 0;
	s32 try = 0;

	do {
		rk1608_read(spi, addr, data, data_len);
		if (rk1608_operation_query(spi, &state) != 0)
			return -1;
		else if ((state & APB_OP_STATE_MASK) == 0)
			break;

		if (try++ == APB_SAFE_OPERATION_TRY_MAX)
			break;
		udelay(APB_SAFE_OPERATION_TRY_DELAY_US);
	} while (1);

	return (state & APB_OP_STATE_MASK);
}

/**
 * rk1608_safe_r32 - RK1608_2_APB synchronous 32-bit read with state check
 *
 * @spi: spi device
 * @addr: apb resource address
 * @data: 32-bit data buffer [out]
 * Context: can sleep
 *
 * It returns zero on success, else operation state code.
 */
int rk1608_safe_r32(struct spi_device *spi,
		    s32 addr, s32 *data)
{
	return rk1608_safe_read(spi, addr, data, 4);
}

static int rk1608_read_wait(struct spi_device *spi,
			    const struct rkl_section *sec)
{
	s32 value = 0;
	int try = 0;
	int ret = 0;

	do {
		ret = rk1608_safe_r32(spi, sec->wait_addr, &value);
		if (!ret && value == sec->wait_value)
			break;

		if (try++ == sec->timeout) {
			ret = -1;
			dev_err(&spi->dev, "read 0x%x is %x != %x timeout\n",
				sec->wait_addr, value, sec->wait_value);
			break;
		}
		mdelay(sec->wait_time);
	} while (1);

	return ret;
}

static int rk1608_boot_request(struct spi_device *spi,
			       const struct rkl_section *sec)
{
	struct rkl_boot_request boot_req;
	int try = 0;
	int ret = 0;

    /*send boot request to rk1608 for ddr init*/
	boot_req.flag = sec->flag;
	boot_req.load_addr = sec->load_addr;
	boot_req.boot_len = sec->size;
	boot_req.status = 1;
	boot_req.cmd = 2;

	ret = rk1608_safe_write(spi, BOOT_REQUEST_ADDR,
				(s32 *)&boot_req, sizeof(boot_req));
	if (ret)
		return ret;

	if (sec->flag & BOOT_FLAG_READ_WAIT) {
	/*waitting for rk1608 init ddr done*/
		do {
			ret = rk1608_safe_read(spi, BOOT_REQUEST_ADDR,
					       (s32 *)&boot_req,
					       sizeof(boot_req));

			if (!ret && boot_req.status == 0)
				break;

			if (try++ == sec->timeout) {
				ret = -1;
				dev_err(&spi->dev, "boot_request timeout\n");
				break;
		}
		mdelay(sec->wait_time);
		} while (1);
	}

	return ret;
}

static int rk1608_download_section(struct spi_device *spi, const u8 *data,
				   const struct rkl_section *sec)
{
	int ret = 0;

	dev_info(&spi->dev, "offset:%x,size:%x,addr:%x,wait_time:%x",
		 sec->offset, sec->size, sec->load_addr, sec->wait_time);
	dev_info(&spi->dev, "timeout:%x,crc:%x,flag:%x,type:%x",
		 sec->timeout, sec->crc_16, sec->flag, sec->type);

	if (sec->size > 0) {
		ret = rk1608_safe_write(spi, sec->load_addr,
					(s32 *)(data + sec->offset),
					sec->size);
		if (ret) {
			dev_err(&spi->dev, "rk1608_safe_write err =%d\n", ret);
			return ret;
		}
	}

	if (sec->flag & BOOT_FLAG_BOOT_REQUEST)
		ret = rk1608_boot_request(spi, sec);
	else if (sec->flag & BOOT_FLAG_READ_WAIT)
		ret = rk1608_read_wait(spi, sec);

	return ret;
}

/**
 * rk1608_download_fw: - rk1608 firmware download through spi
 *
 * @spi: spi device
 * @fw_name: name of firmware file, NULL for default firmware name
 * Context: can sleep
 *
 * It returns zero on success, else a negative error code.
 **/
int rk1608_download_fw(struct spi_device *spi, const char *fw_name)
{
	const struct rkl_header *head;
	const struct firmware *fw;
	int i = 0;
	int ret = 0;

	if (!fw_name)
		fw_name = RKL_DEFAULT_FW_NAME;

	dev_info(&spi->dev, "before request firmware");
	ret = request_firmware(&fw, fw_name, &spi->dev);
	if (ret) {
		dev_err(&spi->dev, "request firmware %s failed!", fw_name);
		return ret;
	}

	head = (const struct rkl_header *)fw->data;

	dev_info(&spi->dev, "request firmware %s (version:%s) success!",
		 fw_name, head->version);

	for (i = 0; i < head->section_count; i++) {
		ret = rk1608_download_section(spi, fw->data,
					      &head->sections[i]);
		if (ret)
			break;
	}

	release_firmware(fw);
	return ret;
}

int rk1608_lsb_w32(struct spi_device *spi, s32 addr, s32 data)
{
	s32 write_cmd = APB_CMD_WRITE;
	struct spi_transfer write_cmd_packet = {
		.tx_buf = &write_cmd,
		.len    = sizeof(write_cmd),
	};
	struct spi_transfer addr_packet = {
		.tx_buf = &addr,
		.len    = sizeof(addr),
	};
	struct spi_transfer data_packet = {
		.tx_buf = &data,
		.len    = sizeof(data),
	};
	struct spi_message  m;

	write_cmd = MSB2LSB32(write_cmd);
	addr = MSB2LSB32(addr);
	data = MSB2LSB32(data);

	spi_message_init(&m);
	spi_message_add_tail(&write_cmd_packet, &m);
	spi_message_add_tail(&addr_packet, &m);
	spi_message_add_tail(&data_packet, &m);
	return spi_sync(spi, &m);
}

void rk1608_cs_set_value(struct rk1608_state *pdata, int value)
{
	s8 null_cmd = 0;
	struct spi_transfer null_cmd_packet = {
		.tx_buf = &null_cmd,
		.len    = sizeof(null_cmd),
		.cs_change = !value,
	};
	struct spi_message  m;

	spi_message_init(&m);
	spi_message_add_tail(&null_cmd_packet, &m);
	spi_sync(pdata->spi, &m);
}

void rk1608_set_spi_speed(struct rk1608_state *pdata, u32 hz)
{
	pdata->spi->max_speed_hz = hz;
}

static int rk1608_sensor_power(struct v4l2_subdev *sd, int on)
{
	int ret = 0;
	struct rk1608_state *pdata = to_state(sd);
	struct spi_device *spi = pdata->spi;

	mutex_lock(&pdata->lock);
	/*start camera power on/off*/
	if (pdata->cam_sd)
		pdata->cam_sd->ops->core->s_power(pdata->cam_sd, on);
	if (on && !pdata->power_count)	{
		clk_prepare_enable(pdata->mclk);
		clk_set_rate(pdata->mclk, RK1608_MCLK_RATE);
		/*request rk1608 enter slave mode*/
		rk1608_cs_set_value(pdata, 0);
		if (pdata->powerdown_gpio > 0) {
			gpio_set_value(pdata->powerdown_gpio,
				       pdata->powerdown_active);
		}
		if (pdata->wakeup_gpio > 0) {
			gpio_set_value(pdata->wakeup_gpio,
				       pdata->wakeup_active);
		}
		mdelay(3);
		if (pdata->reset_gpio > 0)
			gpio_set_value(pdata->reset_gpio, pdata->reset_active);
		mdelay(5);
		rk1608_cs_set_value(pdata, 1);
		rk1608_set_spi_speed(pdata, pdata->min_speed_hz);
		rk1608_lsb_w32(spi, SPI_ENR, 0);
		rk1608_lsb_w32(spi, SPI_CTRL0,
			       OPM_SLAVE_MODE | RSD_SEL_2CYC | DFS_SEL_16BIT);
		rk1608_hw_init(pdata->spi);
		rk1608_set_spi_speed(pdata, pdata->max_speed_hz);
		/*download system firmware*/
		ret = rk1608_download_fw(pdata->spi, NULL);
		if (ret)
			dev_err(pdata->dev, "Download firmware failed!");
		else
			dev_info(pdata->dev, "Download firmware success!");
		enable_irq(pdata->irq);
		if (pdata->sleepst_irq > 0)
			enable_irq(pdata->sleepst_irq);

	} else if (!on && pdata->power_count == 1) {
		disable_irq(pdata->irq);
		if (pdata->sleepst_irq > 0)
			disable_irq(pdata->sleepst_irq);
		if (pdata->powerdown_gpio > 0)
			gpio_set_value(pdata->powerdown_gpio,
				       !pdata->powerdown_active);

		if (pdata->wakeup_gpio > 0)
			gpio_set_value(pdata->wakeup_gpio,
				       !pdata->wakeup_active);

		if (pdata->reset_gpio > 0)
			gpio_set_value(pdata->reset_gpio, !pdata->reset_active);

		rk1608_cs_set_value(pdata, 0);
		clk_disable_unprepare(pdata->mclk);
	}
	/* Update the power count. */
	pdata->power_count += on ? 1 : -1;
	WARN_ON(pdata->power_count < 0);
	mutex_unlock(&pdata->lock);

	return ret;
}

static int rk1608_stream_on(struct rk1608_state *pdata)
{
	int  cnt = 0;
	/*Waiting for the sensor to be ready*/
	while (pdata->sensor_cnt < pdata->cam_nums) {
		/* TIMEOUT 10s break*/
		if (cnt++ > SENSOR_TIMEOUT) {
			dev_err(pdata->dev, "Sensor%d is ready to timeout!",
				pdata->sensor_cnt);
			break;
		}
	mdelay(10);
	}

	if (pdata->cam_nums) {
		if (pdata->sensor_cnt == pdata->cam_nums)
			dev_info(pdata->dev, "Sensor(num %d) is ready!",
				 pdata->sensor_cnt);
	} else {
		dev_warn(pdata->dev, "No sensor is found!");
	}

	return 0;
}

static int rk1608_stream_off(struct rk1608_state *pdata)
{
	pdata->sensor_cnt = 0;

	return 0;
}

static int rk1608_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct rk1608_state *pdata = to_state(sd);

	if (enable)
		return rk1608_stream_on(pdata);
	else
		return rk1608_stream_off(pdata);
}

static int rk1608_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SGRBG8_1X8;

	return 0;
}

static int rk1608_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *mf = &fmt->format;

	mf->code = MEDIA_BUS_FMT_SGRBG8_1X8;
	mf->width = 640;
	mf->height = 480;
	mf->field = V4L2_FIELD_NONE;
	mf->colorspace = V4L2_COLORSPACE_SRGB;

	return 0;
}

static const struct v4l2_subdev_internal_ops rk1608_subdev_internal_ops = {
	.open	= NULL,
};

static const struct v4l2_subdev_video_ops rk1608_subdev_video_ops = {
	.s_stream	= rk1608_s_stream,
};

static const struct v4l2_subdev_pad_ops rk1608_subdev_pad_ops = {
	.enum_mbus_code	= rk1608_enum_mbus_code,
	.get_fmt	= rk1608_get_fmt,
};

static const struct v4l2_subdev_core_ops rk1608_core_ops = {
	.s_power	= rk1608_sensor_power,
};

static const struct v4l2_subdev_ops rk1608_subdev_ops = {
	.core	= &rk1608_core_ops,
	.video	= &rk1608_subdev_video_ops,
	.pad	= &rk1608_subdev_pad_ops,
};

/**
 * rk1608_msq_read_head - read rk1608 msg queue head
 *
 * @spi: spi device
 * @addr: msg queue head addr
 * @m: msg queue pointer
 *
 * It returns zero on success, else a negative error code.
 */
int rk1608_msq_read_head(struct spi_device *spi,
			 u32 addr, struct rk1608_msg_queue *q)
{
	int err = 0;
	s32 reg;

	err = rk1608_safe_r32(spi, RK1608_PMU_SYS_REG0, &reg);

	if (err || ((reg & RK1608_MSG_QUEUE_OK_MASK) !=
			 RK1608_MSG_QUEUE_OK_TAG))
	/*dev_warn(&spi->dev, "rk1608 msg queue head not init!\n");*/
		return -1;

	err = rk1608_safe_read(spi, addr, (s32 *)q, sizeof(*q));
	return err;
}

/**
 * rk1608_msq_recv_msg - receive a msg from RK1608 -> AP msg queue
 *
 * @q: msg queue
 * @m: a msg pointer buf [out]
 *
 * need call rk1608_msq_recv_msg_free to free msg after msg use done
 *
 * It returns zero on success, else a negative error code.
 */
int rk1608_msq_recv_msg(struct spi_device *spi, struct msg **m)
{
	struct rk1608_msg_queue queue;
	struct rk1608_msg_queue *q = &queue;
	u32 size = 0, msg_size = 0;
	u32 recv_addr = 0;
	u32 next_recv_addr = 0;
	int err = 0;

	*m = NULL;
	err = rk1608_msq_read_head(spi, RK1608_S_MSG_QUEUE_ADDR, q);
	if (err)
		return err;

	if (q->cur_send == q->cur_recv)
		return -1;
	/*skip to head when size is 0*/
	err = rk1608_safe_r32(spi, (s32)q->cur_recv, (s32 *)&size);
	if (err)
		return err;
	if (size == 0) {
		err = rk1608_safe_r32(spi, (s32)q->buf_head,
				      (s32 *)&size);
		if (err)
			return err;

		msg_size = size * sizeof(u32);
		recv_addr = q->buf_head;
		next_recv_addr = q->buf_head + msg_size;
	} else {
		msg_size = size * sizeof(u32);
		recv_addr = q->cur_recv;
		next_recv_addr = q->cur_recv + msg_size;
		if (next_recv_addr == q->buf_tail)
			next_recv_addr = q->buf_head;
	}

	if (msg_size > (q->buf_tail - q->buf_head))
		return -2;

	*m = kmalloc(msg_size, GFP_KERNEL);
	err = rk1608_safe_read(spi, recv_addr, (s32 *)*m, msg_size);
	if (err == 0) {
		err = rk1608_safe_w32(spi, RK1608_S_MSG_QUEUE_ADDR +
				       (u8 *)&q->cur_recv - (u8 *)q,
				       next_recv_addr);
	}
	if (err)
		kfree(*m);

	return err;
}

static void print_rk1608_log(struct rk1608_state *pdata,
			     struct msg_rk1608_log_t *log)
{
	char *str = (char *)(log);

	str[log->size * sizeof(s32) - 1] = 0;
	str += sizeof(struct msg_rk1608_log_t);
	dev_info(pdata->dev, "RK1608%d: %s", log->core_id, str);
}

static void dispatch_received_msg(struct rk1608_state *pdata,
				  struct msg *msg)
{
	#if DEBUG_DUMP_ALL_SEND_RECV_MSG == 1
	int32_hexdump("recv msg:", (s32 *)msg, msg->size * 4);
	#endif

	if (msg->type == id_msg_set_stream_out_on_ret_t)
		pdata->sensor_cnt++;

	if (msg->type == id_msg_rk1608_log_t)
		print_rk1608_log(pdata, (struct msg_rk1608_log_t *)msg);
}

static irqreturn_t rk1608_threaded_isr(int irq, void *dev_id)
{
	struct rk1608_state *pdata = dev_id;
	struct msg *msg;

	WARN_ON(irq != pdata->irq);
	while (!rk1608_msq_recv_msg(pdata->spi, &msg) && NULL != msg) {
		dispatch_received_msg(pdata, msg);
		/* for kernel msg sync */
		if (pdata->msg_num != 0 && msg->sync) {
			dev_info(pdata->dev, "rk1608 kernel sync\n");
			mutex_lock(&pdata->send_msg_lock);
			pdata->msg_num--;
			atomic_set(&pdata->msg_done[pdata->msg_num], 1);
			mutex_unlock(&pdata->send_msg_lock);
			wake_up(&pdata->msg_wait);
		}
		kfree(msg);
	}

	return IRQ_HANDLED;
}

static irqreturn_t rk1608_sleep_isr(int irq, void *dev_id)
{
	struct rk1608_state *pdata = dev_id;

	WARN_ON(irq != pdata->sleepst_irq);
	if (pdata->powerdown_gpio > 0)
		gpio_set_value(pdata->powerdown_gpio, !pdata->powerdown_active);

	return IRQ_HANDLED;
}

static int rk1608_parse_dt_property(struct rk1608_state *pdata)
{
	int ret = 0;
	int i;
	struct device *dev = pdata->dev;
	struct device_node *node = dev->of_node;
	enum of_gpio_flags flags;

	if (!node)
		return 1;

	ret = of_property_read_u32(node, "spi-max-frequency",
				   &pdata->max_speed_hz);
	if (ret <= 0) {
		dev_warn(dev, "can not get spi-max-frequency!");
		pdata->max_speed_hz = RK1608_MCLK_RATE;
	}

	ret = of_property_read_u32(node, "spi-min-frequency",
				   &pdata->min_speed_hz);
	if (ret <= 0) {
		dev_warn(dev, "can not get spi-min-frequency!");
		pdata->min_speed_hz = pdata->max_speed_hz / 2;
	}

	pdata->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(pdata->mclk)) {
		dev_err(dev, "can not get mclk, error %ld\n",
			PTR_ERR(pdata->mclk));
		pdata->mclk = NULL;
		return -1;
	}

	ret = of_get_named_gpio_flags(node, "reset-gpio", 0, &flags);
	if (ret <= 0) {
		dev_warn(dev, "can not find reset-gpio, error %d\n", ret);
		return ret;
	}
	pdata->reset_gpio = ret;
	pdata->reset_active = 1;
	if (flags == OF_GPIO_ACTIVE_LOW)
		pdata->reset_active = 0;

	if (pdata->reset_gpio > 0) {
		ret = devm_gpio_request(dev, pdata->reset_gpio, "rk1608-reset");
		if (ret) {
			dev_err(dev, "gpio %d request error %d\n",
				pdata->reset_gpio, ret);
			return ret;
		}

		ret = gpio_direction_output(pdata->reset_gpio,
					    !pdata->reset_active);
		if (ret) {
			dev_err(dev, "gpio %d direction output error %d\n",
				pdata->reset_gpio, ret);
			return ret;
		}
	}

	ret = of_get_named_gpio_flags(node, "irq-gpio", 0, NULL);
	if (ret <= 0) {
		dev_warn(dev, "can not find irq-gpio, error %d\n", ret);
		return ret;
	}

	pdata->irq_gpio = ret;

	ret = devm_gpio_request(dev, pdata->irq_gpio, "rk1608-irq");
	if (ret) {
		dev_err(dev, "gpio %d request error %d\n", pdata->irq_gpio,
			ret);
		return ret;
	}

	ret = gpio_direction_input(pdata->irq_gpio);
	if (ret) {
		dev_err(dev, "gpio %d direction input error %d\n",
			pdata->irq_gpio, ret);
		return ret;
	}

	ret = gpio_to_irq(pdata->irq_gpio);
	if (ret < 0) {
		dev_err(dev, "Unable to get irq number for GPIO %d, error %d\n",
			pdata->irq_gpio, ret);
		return ret;
	}
	pdata->irq = ret;
	ret = request_threaded_irq(pdata->irq, NULL, rk1608_threaded_isr,
				   IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				   "rk1608-irq", pdata);
	if (ret) {
		dev_err(dev, "cannot request thread irq: %d\n", ret);
		return ret;
	}

	disable_irq(pdata->irq);

	ret = of_get_named_gpio_flags(node, "powerdown-gpio", 0, &flags);
	if (ret <= 0)
		dev_warn(dev, "can not find  powerdown-gpio, error %d\n", ret);

	pdata->powerdown_gpio = ret;
	pdata->powerdown_active = 1;
	if (flags == OF_GPIO_ACTIVE_LOW)
		pdata->powerdown_active = 0;

	if (pdata->powerdown_gpio > 0) {
		ret = devm_gpio_request(dev, pdata->powerdown_gpio,
					"rk1608-powerdown");
		if (ret) {
			dev_err(dev, "gpio %d request error %d\n",
				pdata->powerdown_gpio, ret);
			return ret;
		}

		ret = gpio_direction_output(pdata->powerdown_gpio,
					    !pdata->powerdown_active);
		if (ret) {
			dev_err(dev, "gpio %d direction output error %d\n",
				pdata->powerdown_gpio, ret);
			return ret;
		}
	}

	pdata->sleepst_gpio = -1;
	pdata->sleepst_irq = -1;
	pdata->wakeup_gpio = -1;

	ret = of_get_named_gpio_flags(node, "sleepst-gpio", 0, NULL);
	if (ret <= 0) {
		dev_warn(dev, "can not find property sleepst-gpio, error %d\n",
			 ret);
		return ret;
	}

	pdata->sleepst_gpio = ret;

	ret = devm_gpio_request(dev, pdata->sleepst_gpio, "rk1608-sleep-irq");
	if (ret) {
		dev_err(dev, "gpio %d request error %d\n",
			pdata->sleepst_gpio, ret);
		return ret;
	}

	ret = gpio_direction_input(pdata->sleepst_gpio);
	if (ret) {
		dev_err(dev, "gpio %d direction input error %d\n",
			pdata->sleepst_gpio, ret);
		return ret;
	}

	ret = gpio_to_irq(pdata->sleepst_gpio);
	if (ret < 0) {
		dev_err(dev, "Unable to get irq number for GPIO %d, error %d\n",
			pdata->sleepst_gpio, ret);
		return ret;
	}
	pdata->sleepst_irq = ret;
	ret = request_any_context_irq(pdata->sleepst_irq,
				      rk1608_sleep_isr,
				      IRQF_TRIGGER_RISING,
				      "rk1608-sleepst", pdata);
	disable_irq(pdata->sleepst_irq);

	ret = of_get_named_gpio_flags(node, "wakeup-gpio", 0, &flags);
	if (ret <= 0)
		dev_warn(dev, "can not find wakeup-gpio error %d\n", ret);

	pdata->wakeup_gpio = ret;
	pdata->wakeup_active = 1;
	if (flags == OF_GPIO_ACTIVE_LOW)
		pdata->wakeup_active = 0;

	if (pdata->wakeup_gpio > 0) {
		ret = devm_gpio_request(dev, pdata->wakeup_gpio,
					"rk1608-wakeup");
		if (ret) {
			dev_err(dev, "gpio %d request error %d\n",
				pdata->wakeup_gpio, ret);
			return ret;
		}

		ret = gpio_direction_output(pdata->wakeup_gpio,
					    !pdata->wakeup_active);
		if (ret) {
			dev_err(dev, "gpio %d direction output error %d\n",
				pdata->wakeup_gpio, ret);
			return ret;
		}
	}
	pdata->msg_num = 0;
	init_waitqueue_head(&pdata->msg_wait);
	for (i = 0; i < 8; i++)
		atomic_set(&pdata->msg_done[i], 0);

	return ret;
}

static int get_remote_node_dev(struct rk1608_state *pdev)
{
	struct platform_device *cam_pdev = NULL;
	struct device *dev = pdev->dev;
	struct device_node *parent = dev->of_node;
	struct device_node *node, *pre_node = NULL;
	struct device_node  *remote = NULL;
	int ret, cam_nums = 0;

	node = of_graph_get_next_endpoint(parent, pre_node);
	if (node) {
		of_node_put(pre_node);
		pre_node = node;
	} else {
		dev_err(dev, "fieled to get endpoint\n");
		return -EINVAL;
	}
	while ((node = of_graph_get_next_endpoint(parent, pre_node)) != NULL) {
		cam_nums++;
		of_node_put(pre_node);
		pre_node = node;
		remote = of_graph_get_remote_port_parent(node);
		if (!remote) {
			dev_err(dev, "%s: no valid device\n", __func__);
			of_node_put(remote);
		}

		cam_pdev = of_find_device_by_node(remote);
		of_node_put(remote);

		if (!cam_pdev) {
			dev_err(dev, "fieled to get camera device\n");
			ret = -EINVAL;
		} else {
			pdev->cam_sd = platform_get_drvdata(cam_pdev);
			ret = 0;
		}
	}

	if (cam_nums)
		dev_info(dev, "get camera (nums=%d) dev is OK!\n", cam_nums);
	else
		dev_info(dev, "fieled to get camera dev\n");
	pdev->cam_nums = cam_nums;

	return ret;
}

static int rk1608_probe(struct spi_device *spi)
{
	struct rk1608_state *rk1608;
	struct v4l2_subdev *sd;
	struct v4l2_ctrl_handler *handler;
	int ret;

	rk1608 = devm_kzalloc(&spi->dev, sizeof(*rk1608), GFP_KERNEL);
	if (!rk1608)
		return -ENOMEM;
	rk1608->dev = &spi->dev;
	rk1608->spi = spi;
	dev_set_drvdata(&spi->dev, rk1608);

	ret = rk1608_parse_dt_property(rk1608);
	if (ret) {
		dev_err(rk1608->dev, "rk1608 parse dt property err(%d)\n", ret);
		goto parse_err;
	}
	ret = get_remote_node_dev(rk1608);
	if (ret)
		dev_warn(rk1608->dev, "get remote node dev err(%d)\n", ret);
	rk1608->sensor_cnt = 0;
	mutex_init(&rk1608->send_msg_lock);
	mutex_init(&rk1608->lock);
	sd = &rk1608->sd;
	v4l2_spi_subdev_init(sd, spi, &rk1608_subdev_ops);

	handler = &rk1608->ctrl_handler;
	ret = v4l2_ctrl_handler_init(handler, 1);
	if (ret)
		goto handler_init_err;

	rk1608->link_freq = v4l2_ctrl_new_int_menu(handler, NULL,
						   V4L2_CID_LINK_FREQ,
						   0, 0, link_freq_menu_items);
	if (rk1608->link_freq)
		rk1608->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	if (handler->error)
		goto handler_err;

	sd->ctrl_handler = handler;
	sd->internal_ops = &rk1608_subdev_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	rk1608->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;

	ret = media_entity_init(&sd->entity, 1, &rk1608->pad, 0);
	if (ret < 0)
		goto handler_err;

	ret = v4l2_async_register_subdev(sd);
	if (ret < 0)
		goto register_err;
	dev_info(rk1608->dev, "DSP rk1608 Driver probe is OK!\n");

	return 0;
register_err:
	media_entity_cleanup(&sd->entity);
handler_err:
	v4l2_ctrl_handler_free(handler);
handler_init_err:
	v4l2_device_unregister_subdev(&rk1608->sd);
	mutex_destroy(&rk1608->lock);
	mutex_destroy(&rk1608->send_msg_lock);
parse_err:
	kfree(rk1608);
	return ret;
}

static int rk1608_remove(struct spi_device *spi)
{
	struct rk1608_state *rk1608 = dev_get_drvdata(&spi->dev);

	v4l2_async_unregister_subdev(&rk1608->sd);
	media_entity_cleanup(&rk1608->sd.entity);
	v4l2_ctrl_handler_free(&rk1608->ctrl_handler);
	v4l2_device_unregister_subdev(&rk1608->sd);
	mutex_destroy(&rk1608->lock);
	mutex_destroy(&rk1608->send_msg_lock);
	kfree(rk1608);

	return 0;
}

static const struct spi_device_id rk1608_id[] = {
	{ "RK1608", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, rk1608_id);

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id rk1608_of_match[] = {
	{ .compatible = "rockchip,rk1608" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, rk1608_of_match);
#endif

static struct spi_driver rk1608_driver = {
	.driver = {
		.of_match_table = of_match_ptr(rk1608_of_match),
		.name	= "RK1608",
	},
	.probe		= rk1608_probe,
	.remove		= rk1608_remove,
	.id_table	= rk1608_id,
};

module_spi_driver(rk1608_driver);

MODULE_AUTHOR("Rockchip Camera/ISP team");
MODULE_DESCRIPTION("A DSP driver for rk1608 chip");
MODULE_LICENSE("Dual BSD/GPL");
