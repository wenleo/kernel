/**
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

/**
 * Rk1608 is used as the Pre-ISP to link on Soc, which mainly has two
 * functions. One is to download the firmware of RK1608, and the other
 * is to match the extra sensor such as camera and enable sensor by
 * calling sensor's s_power.
 *	|-----------------------|
 *	|     Sensor Camera     |
 *	|-----------------------|
 *	|-----------||----------|
 *	|-----------||----------|
 *	|-----------\/----------|
 *	|     Pre-ISP RK1608	|
 *	|-----------------------|
 *	|-----------||----------|
 *	|-----------||----------|
 *	|-----------\/----------|
 *	|      Rockchip Soc     |
 *	|-----------------------|
 * Data Transfer As shown above. In RK1608, the data received from the
 * extra sensor,and it is passed to the Soc through ISP.
 */

static inline struct rk1608_state *to_state(struct v4l2_subdev *sd)
{
	return container_of(sd, struct rk1608_state, sd);
}

/**
 * rk1608_operation_query - RK1608 last operation state query
 *
 * @spi: device from which data will be read
 * @state: last operation state [out]
 * Context: can sleep
 *
 * It returns zero on success, else a negative error code.
 */
int rk1608_operation_query(struct spi_device *spi, s32 *state)
{
	s32 query_cmd = RK1608_CMD_QUERY;
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

	return ((*state & RK1608_STATE_ID_MASK) == RK1608_STATE_ID) ? 0 : -1;
}

int rk1608_write(struct spi_device *spi,
		 s32 addr, const s32 *data, size_t data_len)
{
	s32 write_cmd = RK1608_CMD_WRITE;
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
 * rk1608_safe_write - RK1608 synchronous write with state check
 *
 * @spi: spi device
 * @addr: resource address
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
	s32 state, retry = 0;

	while (data_len > 0) {
		size_t slen = MIN(data_len, RK1608_MAX_OP_BYTES);

		do {
			rk1608_write(spi, addr, data, data_len);
			if (rk1608_operation_query(spi, &state) != 0)
				return -1;
			if ((state & RK1608_STATE_MASK) == 0)
				break;

			udelay(RK1608_OP_TRY_DELAY);
		} while (retry++ != RK1608_OP_TRY_MAX);

		data_len = data_len - slen;
		data = (s32 *)((s8 *)data + slen);
		addr += slen;
	}
	return ret;
}

void rk1608_hw_init(struct spi_device *spi)
{
	s32 write_data = SPI0_PLL_SEL_APLL;

	/* modify rk1608 spi slave clk to 300M */
	rk1608_safe_write(spi, CRUPMU_CLKSEL14_CON, &write_data, 4);

	/* modify rk1608 spi io driver strength to 8mA */
	write_data = BIT7_6_SEL_8MA;
	rk1608_safe_write(spi, PMUGRF_GPIO1A_E, &write_data, 4);
	write_data = BIT1_0_SEL_8MA;
	rk1608_safe_write(spi, PMUGRF_GPIO1B_E, &write_data, 4);
}

/**
 * rk1608_read - RK1608 synchronous read
 *
 * @spi: spi device
 * @addr: resource address
 * @data: data buffer [out]
 * @data_len: data buffer size, in bytes
 * Context: can sleep
 *
 * It returns zero on success, else a negative error code.
 */
int rk1608_read(struct spi_device *spi,
		s32 addr, s32 *data, size_t data_len)
{
	s32 real_len = MIN(data_len, RK1608_MAX_OP_BYTES);
	s32 read_cmd = RK1608_CMD_READ | (real_len << 14 &
					   RK1608_STATE_ID_MASK);
	s32 read_begin_cmd = RK1608_CMD_READ_BEGIN;
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
 * rk1608_safe_read - RK1608 synchronous read with state check
 *
 * @spi: spi device
 * @addr: resource address
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
	s32 retry = 0;

	do {
		rk1608_read(spi, addr, data, data_len);
		if (rk1608_operation_query(spi, &state) != 0)
			return -1;
		if ((state & RK1608_STATE_MASK) == 0)
			break;
		udelay(RK1608_OP_TRY_DELAY);
	} while (retry++ != RK1608_OP_TRY_MAX);

	return (state & RK1608_STATE_MASK);
}

static int rk1608_read_wait(struct spi_device *spi,
			    const struct rk1608_section *sec)
{
	s32 value = 0;
	int retry = 0;
	int ret = 0;

	do {
		ret = rk1608_safe_read(spi, sec->wait_addr, &value, 4);
		if (!ret && value == sec->wait_value)
			break;

		if (retry++ == sec->timeout) {
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
			       const struct rk1608_section *sec)
{
	struct rk1608_boot_req boot_req;
	int retry = 0;
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

			if (retry++ == sec->timeout) {
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
				   const struct rk1608_section *sec)
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
	const struct rk1608_header *head;
	const struct firmware *fw;
	int i = 0;
	int ret = 0;

	if (!fw_name)
		fw_name = RK1608_FW_NAME;

	dev_info(&spi->dev, "before request firmware");
	ret = request_firmware(&fw, fw_name, &spi->dev);
	if (ret) {
		dev_err(&spi->dev, "request firmware %s failed!", fw_name);
		return ret;
	}

	head = (const struct rk1608_header *)fw->data;

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
	s32 write_cmd = RK1608_CMD_WRITE;
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

int rk1608_msg_init_sensor(struct rk1608_state *pdata)
{
	struct msg_init *msg;

	msg = (struct msg_init *)devm_kzalloc(pdata->dev,
					      sizeof(struct msg_init),
					      GFP_KERNEL);
	if (!msg)
		return -ENOMEM;
	msg->size = sizeof(struct msg_init);
	msg->type = id_msg_init_sensor_t;
	msg->camera_id = pdata->cd->ci.cam_id;
	msg->sync = 1;
	msg->in_mipi_phy = pdata->cd->ci.in_mipi;
	msg->out_mipi_phy = pdata->cd->ci.out_mipi;
	msg->mipi_lane = pdata->cd->ci.mipi_lane;
	msg->bayer = 0;
	memcpy((void *)msg->sensor_name, pdata->cd->sd.name, 32);

	msg->i2c_slave_addr = (pdata->cd->client->addr) << 1;
	msg->i2c_bus = pdata->cd->ci.i2c_bus;

	return  rk1608_send_msg_to_dsp(pdata, (struct msg *)msg);
}

int rk1608_msg_set_input_size(struct rk1608_state *pdata)
{
	struct msg_in_size *msg;

	msg = (struct msg_in_size *)devm_kzalloc(pdata->dev,
						 sizeof(struct msg_in_size),
						 GFP_KERNEL);
	if (!msg)
		return -ENOMEM;
	msg->msg_head.size = sizeof(struct msg_in_size) / sizeof(int);
	msg->msg_head.type = id_msg_set_input_size_t;
	msg->msg_head.id.camera_id = pdata->cd->ci.cam_id;
	msg->msg_head.mux.sync = 1;
	msg->channel.data_type =  pdata->cd->ci.data_type;
	msg->channel.decode_format =  pdata->cd->ci.data_type;
	msg->channel.width = pdata->cd->ci.width;
	msg->channel.height = pdata->cd->ci.height;
	msg->channel.flag = 1;

	return rk1608_send_msg_to_dsp(pdata, (struct msg *)msg);
}

int rk1608_msg_set_output_size(struct rk1608_state *pdata)
{
	struct msg_out_size *msg;

	msg = (struct msg_out_size *)devm_kzalloc(pdata->dev,
						  sizeof(struct msg_out_size),
						  GFP_KERNEL);
	if (!msg)
		return -ENOMEM;
	msg->size = sizeof(struct msg_out_size);
	msg->type = id_msg_set_output_size_t;
	msg->camera_id = pdata->cd->ci.cam_id;
	msg->sync = 1;
	msg->width = pdata->cd->ci.width;
	msg->height = pdata->cd->ci.height;
	msg->mipi_clk = pdata->cd->ci.mipi_clock;
	msg->line_length_pclk = pdata->cd->ci.htotal;
	msg->frame_length_lines = pdata->cd->ci.vtotal;

	return rk1608_send_msg_to_dsp(pdata, (struct msg *)msg);
}

int rk1608_msg_set_stream_in_on(struct rk1608_state *pdata)
{
	struct msg_t *msg;

	msg = (struct msg_t *)devm_kzalloc(pdata->dev, sizeof(struct msg_t),
					   GFP_KERNEL);
	if (!msg)
		return -ENOMEM;
	msg->size = sizeof(struct msg_t);
	msg->type = id_msg_set_stream_in_on_t;
	msg->id.camera_id = pdata->cd->ci.cam_id;
	msg->mux.sync = 1;

	return rk1608_send_msg_to_dsp(pdata, (struct msg *)msg);
}

int rk1608_msg_set_stream_in_off(struct rk1608_state *pdata)
{
	struct msg_t *msg;

	msg = (struct msg_t *)devm_kzalloc(pdata->dev, sizeof(struct msg_t),
					   GFP_KERNEL);
	if (!msg)
		return -ENOMEM;
	msg->size = sizeof(struct msg_t);
	msg->type = id_msg_set_stream_in_off_t;
	msg->id.camera_id = pdata->cd->ci.cam_id;
	msg->mux.sync = 1;

	return rk1608_send_msg_to_dsp(pdata, (struct msg *)msg);
}

int rk1608_msg_set_stream_out_on(struct rk1608_state *pdata)
{
	struct msg_t *msg;

	msg = (struct msg_t *)devm_kzalloc(pdata->dev, sizeof(struct msg_t),
					   GFP_KERNEL);
	if (!msg)
		return -ENOMEM;
	msg->size = sizeof(struct msg_t);
	msg->type = id_msg_set_stream_out_on_t;
	msg->id.camera_id = pdata->cd->ci.cam_id;
	msg->mux.sync = 1;

	return rk1608_send_msg_to_dsp(pdata, (struct msg *)msg);
}

int rk1608_msg_set_stream_out_off(struct rk1608_state *pdata)
{
	struct msg_t *msg;

	msg = (struct msg_t *)devm_kzalloc(pdata->dev, sizeof(struct msg_t),
					   GFP_KERNEL);
	if (!msg)
		return -ENOMEM;
	msg->size = sizeof(struct msg_t);
	msg->type = id_msg_set_stream_out_off_t;
	msg->id.camera_id = pdata->cd->ci.cam_id;
	msg->mux.sync = 1;

	return rk1608_send_msg_to_dsp(pdata, (struct msg *)msg);
}

int rk1608_set_log_level(struct rk1608_state *pdata, int level)
{
	struct msg_t *msg;

	msg = (struct msg_t *)devm_kzalloc(pdata->dev, sizeof(struct msg_t),
					   GFP_KERNEL);
	if (!msg)
		return -ENOMEM;
	msg->size = sizeof(struct msg_t);
	msg->type = id_msg_set_log_level_t;
	msg->mux.log_level = level;

	return  rk1608_send_msg_to_dsp(pdata, (struct msg *)msg);
}

int rk1608_init_sensor(struct rk1608_state *pdata)
{
	int ret;

	if (!pdata->cd) {
		dev_err(pdata->dev, "No find camera device!\n");
		return -1;
	}

	ret = rk1608_msg_init_sensor(pdata);
	mdelay(20);
	ret |= rk1608_msg_set_input_size(pdata);
	ret |= rk1608_msg_set_output_size(pdata);
	ret |= rk1608_msg_set_stream_in_on(pdata);
	mdelay(30);
	ret |= rk1608_msg_set_stream_out_on(pdata);

	return ret;
}

int rk1608_deinit(struct rk1608_state *pdata)
{
	int ret;

	ret = rk1608_msg_set_stream_out_off(pdata);
	mdelay(30);
	ret |= rk1608_msg_set_stream_in_off(pdata);

	return ret;
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

static int rk1608_pin_power(struct rk1608_state *pdata, int on)
{
	unsigned int grf_val = 0;

	if (on) {
		clk_prepare_enable(pdata->pd_cif);
		clk_prepare_enable(pdata->aclk_cif);
		clk_prepare_enable(pdata->hclk_cif);
		clk_prepare_enable(pdata->cif_clk_in);
		clk_prepare_enable(pdata->cif_clk_out);
		clk_prepare_enable(pdata->clk_mipi_24m);
		clk_prepare_enable(pdata->hclk_mipiphy);
		clk_set_rate(pdata->cif_clk_out, RK1608_MCLK_RATE);

		clk_prepare_enable(pdata->mipi_clk);
		clk_set_rate(pdata->mipi_clk, RK1608_MCLK_RATE);

		gpio_direction_output(pdata->rst1, 0);
		mdelay(3);
		gpio_direction_output(pdata->rst1, 1);
		mdelay(5);
		gpio_direction_output(pdata->powerdown1, 1);

		gpio_direction_output(pdata->rst0, 0);
		mdelay(3);
		gpio_direction_output(pdata->rst0, 1);
		mdelay(5);
		gpio_direction_output(pdata->powerdown0, 1);

		pdata->grf_gpio2b_iomux = ioremap((resource_size_t)
						  (GRF_BASE_ADDR +
						   GRF_GPIO2B_IOMUX), 4);
		grf_val = __raw_readl(pdata->grf_gpio2b_iomux);
		__raw_writel(((grf_val) | (1 << 6) | (1 << (6 + 16))),
			     pdata->grf_gpio2b_iomux);

		pdata->grf_io_vsel = ioremap((resource_size_t)
					      (GRF_BASE_ADDR + GRF_IO_VSEL), 4);
		grf_val = __raw_readl(pdata->grf_io_vsel);
		__raw_writel(((grf_val) | (1 << 1) | (1 << (1 + 16))),
			     pdata->grf_io_vsel);
	} else if (!on) {
		grf_val = __raw_readl(pdata->grf_gpio2b_iomux);
		grf_val |= (1 << (6 + 16));
		grf_val &= (~(1 << 6));
		__raw_writel(grf_val, pdata->grf_gpio2b_iomux);
		iounmap(pdata->grf_gpio2b_iomux);

		grf_val = 0;
		grf_val = __raw_readl(pdata->grf_io_vsel);
		grf_val |= (1 << (1 + 16));
		grf_val &= (~(1 << 1));
		__raw_writel(grf_val, pdata->grf_io_vsel);
		iounmap(pdata->grf_io_vsel);

		gpio_direction_input(pdata->rst1);
		mdelay(3);
		gpio_direction_input(pdata->powerdown1);
		gpio_direction_input(pdata->rst0);
		mdelay(5);
		gpio_direction_input(pdata->powerdown0);

		clk_disable_unprepare(pdata->pd_cif);
		clk_disable_unprepare(pdata->aclk_cif);
		clk_disable_unprepare(pdata->hclk_cif);
		clk_disable_unprepare(pdata->cif_clk_in);
		clk_disable_unprepare(pdata->cif_clk_out);
		clk_disable_unprepare(pdata->clk_mipi_24m);
		clk_disable_unprepare(pdata->hclk_mipiphy);
		clk_disable_unprepare(pdata->mipi_clk);
	}
	return 0;
}

static int rk1608_sensor_power(struct v4l2_subdev *sd, int on)
{
	int ret = 0;
	struct rk1608_state *pdata = to_state(sd);
	struct spi_device *spi = pdata->spi;

	mutex_lock(&pdata->lock);
	rk1608_pin_power(pdata, on);
	/*start Sensor power on/off*/
	if (pdata->cd)
		pdata->cd->sd.ops->core->s_power(&pdata->cd->sd, on);
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

		ret = rk1608_set_log_level(pdata, 2);
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
	int  cnt, ret = 0;

	ret = rk1608_init_sensor(pdata);
	if (ret)
		dev_err(pdata->dev, "rk1608 init sensor is err!(ret %d)", ret);
	/*Waiting for the sensor to be ready*/
	while (pdata->sensor_cnt < pdata->sensor_nums) {
		/* TIMEOUT 10s break*/
		if (cnt++ > SENSOR_TIMEOUT) {
			dev_err(pdata->dev, "Sensor%d is ready to timeout!",
				pdata->sensor_cnt);
			break;
		}
	mdelay(10);
	}

	if (pdata->sensor_nums) {
		if (pdata->sensor_cnt == pdata->sensor_nums)
			dev_info(pdata->dev, "Sensor(num %d) is ready!",
				 pdata->sensor_cnt);
	} else {
		dev_warn(pdata->dev, "No sensor is found!");
	}

	return 0;
}

static int rk1608_stream_off(struct rk1608_state *pdata)
{
	mutex_lock(&pdata->sensor_lock);
	pdata->sensor_cnt = 0;
	mutex_unlock(&pdata->sensor_lock);

	return rk1608_deinit(pdata);
}

static int rk1608_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret;
	struct rk1608_state *pdata = to_state(sd);

	if (enable)
		ret = rk1608_stream_on(pdata);
	else
		ret = rk1608_stream_off(pdata);

	if (pdata->cd)
		ret |= pdata->cd->sd.ops->video->s_stream(&pdata->cd->sd,
							  enable);
	return ret;
}

static int rk1608_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct rk1608_state *pdata = to_state(sd);
	struct camera_dev *cd = pdata->cd;

	if (code->index > 0)
		return -EINVAL;

	code->code = cd->ci.code;

	return 0;
}

static int rk1608_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	struct rk1608_state *pdata = to_state(sd);
	struct camera_dev *cd = pdata->cd;

	if (cd) {
		mf->code = cd->ci.code;
		mf->width = cd->ci.width;
		mf->height = cd->ci.height;
		mf->field = cd->ci.field;
		mf->colorspace = cd->ci.colorspace;
	}

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

	err = rk1608_safe_read(spi, RK1608_PMU_SYS_REG0, &reg, 4);

	if (err || ((reg & RK1608_MSG_QUEUE_OK_MASK) !=
			 RK1608_MSG_QUEUE_OK_TAG))
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
	err = rk1608_safe_read(spi, (s32)q->cur_recv, (s32 *)&size, 4);
	if (err)
		return err;
	if (size == 0) {
		err = rk1608_safe_read(spi, (s32)q->buf_head, (s32 *)&size, 4);
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
		err = rk1608_safe_write(spi, RK1608_S_MSG_QUEUE_ADDR +
				       (u8 *)&q->cur_recv - (u8 *)q,
				       &next_recv_addr, 4);
	}
	if (err)
		kfree(*m);

	return err;
}

/**
 * rk1608_msq_tail_free_size - get msg queue tail unused buf size
 *
 * @q: msg queue
 *
 * It returns size of msg queue tail unused buf size, unit byte
 */
u32 rk1608_msq_tail_free_size(const struct rk1608_msg_queue *q)
{
	if (q->cur_send >= q->cur_recv)
		return (q->buf_tail - q->cur_send);

	return q->cur_recv - q->cur_send;
}

/**
 * spi2apb_interrupt_request - SPI2APB request a dsp interrupt
 *
 * @spi: spi device
 * @interrupt_num: interrupt identification
 * Context: can sleep
 *
 * It returns zero on success, else a negative error code.
 */
int spi2apb_interrupt_request(struct spi_device *spi,
			      s32 interrupt_num)
{
	s32 write_reg1_cmd = APB_CMD_WRITE_REG1;
	struct spi_transfer write_reg1_cmd_packet = {
		.tx_buf = &write_reg1_cmd,
		.len    = sizeof(write_reg1_cmd),
	};
	struct spi_transfer reg1_packet = {
		.tx_buf = &interrupt_num,
		.len    = sizeof(interrupt_num),
	};
	struct spi_message  m;

	spi_message_init(&m);
	spi_message_add_tail(&write_reg1_cmd_packet, &m);
	spi_message_add_tail(&reg1_packet, &m);
	return spi_sync(spi, &m);
}

/**
 * dsp_msq_head_free_size - get msg queue head unused buf size
 *
 * @q: msg queue
 *
 * It returns size of msg queue head unused buf size, unit byte
 */
u32 rk1608_msq_head_free_size(const struct rk1608_msg_queue *q)
{
	if (q->cur_send >= q->cur_recv)
		return (q->cur_recv - q->buf_head);

	return 0;
}

/**
 * rk1608_msq_send_msg - send a msg to Soc -> DSP msg queue
 *
 * @spi: spi device
 * @m: a msg to send
 *
 * It returns zero on success, else a negative error code.
 */
int rk1608_msq_send_msg(struct spi_device *spi, const struct msg *m)
{
	int err = 0;
	struct rk1608_msg_queue queue;
	struct rk1608_msg_queue *q = &queue;
	u32 msg_size = m->size * sizeof(u32);

	err = rk1608_msq_read_head(spi, RK1608_R_MSG_QUEUE_ADDR, q);
	if (err)
		return err;

	if (rk1608_msq_tail_free_size(q) > msg_size) {
		u32 next_send;

		err = rk1608_safe_write(spi, q->cur_send, (s32 *)m, msg_size);
		next_send = q->cur_send + msg_size;
		if (next_send == q->buf_tail)
			next_send = q->buf_head;
		q->cur_send = next_send;
	} else if (rk1608_msq_head_free_size(q) > msg_size) {
	/* set size to 0 for skip to head mark */
		err = rk1608_safe_write(spi, q->cur_send,
					(s32 *)NULL, 4);
		if (err)
			return err;
		err = rk1608_safe_write(spi, q->buf_head,
					(s32 *)m, msg_size);
		q->cur_send = q->buf_head + msg_size;
	} else {
		return -1;
	}

	if (err)
		return err;

	err = rk1608_safe_write(spi, RK1608_R_MSG_QUEUE_ADDR +
				(u8 *)&q->cur_send - (u8 *)q, &q->cur_send, 4);
	spi2apb_interrupt_request(spi, RK1608_IRQ_TYPE_MSG);
	return err;
}

int rk1608_send_msg_to_dsp(struct rk1608_state *pdata, const struct msg *m)
{
	int ret = 0;
	int msg_num;
	int timeout;
	int readval;

	/* for msg sync */
	if (pdata->msg_num >= 8) {
		dev_err(pdata->dev, "msg sync queue full\n!");
		ret = -EINVAL;
	} else if (m->sync == 1) {
		mutex_lock(&pdata->send_msg_lock);
		msg_num = pdata->msg_num;
		atomic_set(&pdata->msg_done[pdata->msg_num++], 0);
		mutex_unlock(&pdata->send_msg_lock);
	}

	mutex_lock(&pdata->send_msg_lock);
	ret = rk1608_msq_send_msg(pdata->spi, m);
	mutex_unlock(&pdata->send_msg_lock);

	/* for msg sync */
	if (m->sync == 1) {
		readval = atomic_read(&pdata->msg_done[msg_num]);
		timeout = wait_event_timeout(pdata->msg_wait, readval,
					     MSG_SYNC_TIMEOUT);
		if (unlikely(timeout <= 0)) {
			dev_info(pdata->dev, "msg_wait timeout %d msg_num:%d\n",
				 timeout, pdata->msg_num);
			mutex_lock(&pdata->send_msg_lock);
			atomic_set(&pdata->msg_done[msg_num], 0);
			mutex_unlock(&pdata->send_msg_lock);
		}
	}

	return ret;
}

static void print_rk1608_log(struct rk1608_state *pdata,
			     struct msg_rk1608_log_t *log)
{
	char *str = (char *)(log);

	str[log->size * sizeof(s32) - 1] = 0;
	str += sizeof(struct msg_rk1608_log_t);
	dev_info(pdata->dev, "RK1608(%d): %s", log->core_id, str);
}

void int32_hexdump(const char *prefix, s32 *data, int len)
{
	pr_err("%s\n", prefix);
	print_hex_dump(KERN_ERR, "offset ", DUMP_PREFIX_OFFSET,
		       16, 4, data, len, false);
	pr_err("\n");
}

static void dispatch_received_msg(struct rk1608_state *pdata,
				  struct msg *msg)
{
	#if DEBUG_DUMP_ALL_SEND_RECV_MSG == 1
	int32_hexdump("recv msg:", (s32 *)msg, msg->size * 4);
	#endif
	if (msg->type == id_msg_set_stream_out_on_ret_t) {
		mutex_lock(&pdata->sensor_lock);
		pdata->sensor_cnt++;
		mutex_unlock(&pdata->sensor_lock);
	}

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
	if (ret) {
		dev_warn(dev, "can not get spi-max-frequency!");
		pdata->max_speed_hz = RK1608_MCLK_RATE;
	}
	ret = of_property_read_u64(node, "link-freqs",
				   &pdata->link_freqs);
	if (ret)
		dev_warn(dev, "can not get link_freqs!");
	pdata->mipi_clk = devm_clk_get(dev, "mipi_clk");
	if (IS_ERR(pdata->mipi_clk)) {
		dev_err(dev, "can not get mipi_clk, error %ld\n",
			PTR_ERR(pdata->mipi_clk));
		pdata->mipi_clk = NULL;
	}
	pdata->pd_cif = devm_clk_get(dev, "pd_cif");
	if (IS_ERR(pdata->pd_cif)) {
		dev_err(dev, "can not get pd_cif, error %ld\n",
			PTR_ERR(pdata->pd_cif));
		pdata->pd_cif = NULL;
	}
	pdata->aclk_cif = devm_clk_get(dev, "aclk_cif");
	if (IS_ERR(pdata->aclk_cif)) {
		dev_err(dev, "can not get aclk_cif, error %ld\n",
			PTR_ERR(pdata->aclk_cif));
		pdata->aclk_cif = NULL;
	}
	pdata->hclk_cif = devm_clk_get(dev, "hclk_cif");
	if (IS_ERR(pdata->hclk_cif)) {
		dev_warn(dev, "can not get hclk_cif, error %ld\n",
			 PTR_ERR(pdata->hclk_cif));
		pdata->hclk_cif = NULL;
	}
	pdata->cif_clk_in = devm_clk_get(dev, "cif0_in");
	if (IS_ERR(pdata->cif_clk_in)) {
		dev_err(dev, "can not get cif_clk_in, error %ld\n",
			PTR_ERR(pdata->cif_clk_in));
		pdata->cif_clk_in = NULL;
	}
	pdata->cif_clk_out = devm_clk_get(dev, "cif0_out");
	if (IS_ERR(pdata->cif_clk_out)) {
		dev_err(dev, "can not get cif_clk_out, error %ld\n",
			PTR_ERR(pdata->cif_clk_out));
		pdata->cif_clk_out = NULL;
	}
	pdata->clk_mipi_24m = devm_clk_get(dev, "clk_mipi_24m");
	if (IS_ERR(pdata->clk_mipi_24m)) {
		dev_err(dev, "can not get clk_mipi_24m, error %ld\n",
			PTR_ERR(pdata->clk_mipi_24m));
			pdata->clk_mipi_24m = NULL;
	}
	pdata->hclk_mipiphy = devm_clk_get(dev, "hclk_mipiphy");
	if (IS_ERR(pdata->hclk_mipiphy)) {
		dev_err(dev, "can not get hclk_mipiphy, error %ld\n",
			PTR_ERR(pdata->hclk_mipiphy));
		pdata->hclk_mipiphy = NULL;
	}
	pdata->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(pdata->mclk)) {
		dev_err(dev, "can not get mclk, error %ld\n",
			PTR_ERR(pdata->mclk));
		pdata->mclk = NULL;
	}
	ret = of_get_named_gpio_flags(node, "rockchip,reset1", 0, &flags);
	if (ret <= 0)
		dev_err(dev, "can not find reset1 error %d\n", ret);
	pdata->rst1 = ret;

	ret = devm_gpio_request(dev, pdata->rst1, "rockchip-reset1");
	if (ret) {
		dev_err(dev, "gpio pdata->rst1 %d request error %d\n",
			pdata->rst1, ret);
		return ret;
	}
	gpio_set_value(pdata->rst1, 0);
	ret = gpio_direction_output(pdata->rst1, 0);
	if (ret) {
		dev_err(dev, "gpio %d direction output error %d\n",
			pdata->rst1, ret);
		return ret;
	}

	ret = of_get_named_gpio_flags(node, "rockchip,powerdown1", 0, &flags);
	if (ret <= 0)
		dev_warn(dev, "can not find powerdown1, error %d\n", ret);

	pdata->powerdown1 = ret;
	ret = devm_gpio_request(dev, pdata->powerdown1, "rockchip-powerdown1");
	if (ret) {
		dev_err(dev, "gpio pdata->powerdown1 %d request error %d\n",
			pdata->powerdown1, ret);
		return ret;
	}

	ret = gpio_direction_output(pdata->powerdown1, 0);
	if (ret) {
		dev_err(dev, "gpio %d direction output error %d\n",
			pdata->powerdown1, ret);
		return ret;
	}

	ret = of_get_named_gpio_flags(node, "rockchip,reset0", 0, &flags);
	if (ret <= 0)
		dev_warn(dev,
			 "can not find property rockchip,reset0, error %d\n",
			 ret);
	pdata->rst0 = ret;
	ret = devm_gpio_request(dev, pdata->rst0, "rockchip-reset0");
	if (ret) {
		dev_err(dev, "gpio pdata->rst0 %d request error %d\n",
			pdata->rst0, ret);
		return ret;
	}
	gpio_set_value(pdata->rst0, 0);
	ret = gpio_direction_output(pdata->rst0, 0);
	if (ret) {
		dev_err(dev, "gpio0 %d direction output error %d\n",
			pdata->rst0, ret);
		return ret;
	}

	ret = of_get_named_gpio_flags(node, "rockchip,powerdown0", 0, &flags);
	if (ret <= 0)
		dev_warn(dev, "can not find powerdown0,error %d\n", ret);

	pdata->powerdown0 = ret;

	ret = devm_gpio_request(dev, pdata->powerdown0, "rockchip-powerdown0");
	if (ret) {
		dev_err(dev, "gpio pdata->powerdown0 %d request error %d\n",
			pdata->powerdown0, ret);
		return ret;
	}

	ret = gpio_direction_output(pdata->powerdown0, 0);
	if (ret) {
		dev_err(dev, "gpio0 %d direction output error %d\n",
			pdata->powerdown0, ret);
		return ret;
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
	struct i2c_client *sensor_pdev = NULL;
	struct device *dev = pdev->dev;
	struct device_node *parent = dev->of_node;
	struct device_node *node, *pre_node = NULL;
	struct device_node  *remote = NULL;
	int ret, sensor_nums = 0;

	node = of_graph_get_next_endpoint(parent, pre_node);
	if (node) {
		of_node_put(pre_node);
		pre_node = node;
	} else {
		dev_err(dev, "fieled to get endpoint\n");
		return -EINVAL;
	}
	while ((node = of_graph_get_next_endpoint(parent, pre_node)) != NULL) {
		of_node_put(pre_node);
		pre_node = node;
		remote = of_graph_get_remote_port_parent(node);
		if (!remote) {
			dev_err(dev, "%s: no valid device\n", __func__);
			of_node_put(remote);
			ret = -EINVAL;
		}

		sensor_pdev = of_find_i2c_device_by_node(remote);
		of_node_put(remote);

		if (!sensor_pdev) {
			dev_err(dev, "fieled to get Sensor device\n");
			ret = -EINVAL;
		} else {
			pdev->cd = i2c_get_clientdata(sensor_pdev);
			if (pdev->cd)
				sensor_nums++;
			else
				dev_err(dev, "fieled to get Sensor drvdata\n");
			ret = 0;
		}
	}
	pdev->sensor_nums = sensor_nums;
	if (pdev->sensor_nums)
		dev_info(dev, "get Sensor (nums=%d) dev is OK!\n",
			 pdev->sensor_nums);

	return ret;
}

static int rk1608_probe(struct spi_device *spi)
{
	struct rk1608_state		*rk1608;
	struct v4l2_subdev		*sd;
	struct v4l2_ctrl_handler	*handler;

	int ret;

	rk1608 = devm_kzalloc(&spi->dev, sizeof(*rk1608), GFP_KERNEL);
	if (!rk1608)
		return -ENOMEM;
	rk1608->dev = &spi->dev;
	rk1608->spi = spi;
	spi_set_drvdata(spi, rk1608);
	ret = rk1608_parse_dt_property(rk1608);
	if (ret) {
		dev_err(rk1608->dev, "rk1608 parse dt property err(%x)\n", ret);
		goto parse_err;
	}
	ret = get_remote_node_dev(rk1608);
	if (ret)
		dev_warn(rk1608->dev, "get remote node dev err(%x)\n", ret);
	rk1608->sensor_cnt = 0;
	mutex_init(&rk1608->sensor_lock);
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
						   0, 0, &rk1608->link_freqs);
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
	mutex_destroy(&rk1608->sensor_lock);
parse_err:
	kfree(rk1608);
	return ret;
}

static int rk1608_remove(struct spi_device *spi)
{
	struct rk1608_state *rk1608 = spi_get_drvdata(spi);

	v4l2_async_unregister_subdev(&rk1608->sd);
	media_entity_cleanup(&rk1608->sd.entity);
	v4l2_ctrl_handler_free(&rk1608->ctrl_handler);
	v4l2_device_unregister_subdev(&rk1608->sd);
	mutex_destroy(&rk1608->lock);
	mutex_destroy(&rk1608->send_msg_lock);
	mutex_destroy(&rk1608->sensor_lock);
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
