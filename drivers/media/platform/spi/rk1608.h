/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author: Tusson <dusong@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __SPI_RK_PREISP_H__
#define __SPI_RK_PREISP_H__

#include <linux/types.h>
#include <linux/string.h>
#include <linux/spi/spi.h>
#include "linux/i2c.h"

#define SENSOR_TIMEOUT			1000
#define GRF_BASE_ADDR			0xff770000
#define GRF_GPIO2B_IOMUX		0x0014
#define GRF_IO_VSEL			0x0380

#define APB_SAFE_OPERATION_TRY_MAX	3
#define APB_SAFE_OPERATION_TRY_DELAY_US	10
#define APB_CMD_WRITE			0x00000011
#define APB_CMD_WRITE_REG0		0X00010011
#define APB_CMD_WRITE_REG1		0X00020011
#define APB_CMD_READ			0x00000077
#define APB_CMD_READ_BEGIN		0x000000AA
#define APB_CMD_QUERY			0x000000FF
#define APB_CMD_QUERY_REG2		0x000001FF

#define APB_OP_STATE_ID_MASK		(0xffff0000)
#define APB_OP_STATE_ID			(0X16080000)

#define APB_OP_STATE_MASK		(0x0000ffff)
#define APB_OP_STATE_WRITE_ERROR	(0x01 << 0)
#define APB_OP_STATE_WRITE_OVERFLOW	(0x01 << 1)
#define APB_OP_STATE_WRITE_UNFINISHED	(0x01 << 2)
#define APB_OP_STATE_READ_ERROR		(0x01 << 8)
#define APB_OP_STATE_READ_UNDERFLOW	(0x01 << 9)
#define APB_OP_STATE_PRE_READ_ERROR	(0x01 << 10)

#define APB_MAX_OP_BYTES		60000

#define RK1608_ROW_START_DEF		54
#define RK1608_COLUMN_START_DEF		16
#define RK1608_WINDOW_HEIGHT_DEF	480
#define RK1608_WINDOW_WIDTH_DEF		640
#define DEBUG_DUMP_ALL_SEND_RECV_MSG	0
#define RK1608_MCLK_RATE		(24 * 1000 * 1000ul)
#define MSG_SYNC_TIMEOUT		100

#define	OPM_SLAVE_MODE			0X100000
#define	RSD_SEL_2CYC			0X008000
#define	DFS_SEL_16BIT			0X000002
#define SPI_CTRL0			0x11060000
#define SPI_ENR				0x11060008
#define CRUPMU_CLKSEL14_CON		0x12008098
#define PMUGRF_GPIO1A_E			0x12030040
#define PMUGRF_GPIO1B_E			0x12030044
#define BIT7_6_SEL_8MA			0xf000a000
#define BIT1_0_SEL_8MA			0x000f000a
#define SPI0_PLL_SEL_APLL		0xff004000
#define INVALID_ID			-1

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef MSB2LSB8
#define MSB2LSB8(x) ((((u8)x & 0x80) >> 7) | \
			(((u8)x & 0x40) >> 5) | \
			(((u8)x & 0x20) >> 3) | \
			(((u8)x & 0x10) >> 1) | \
			(((u8)x & 0x08) << 1) | \
			(((u8)x & 0x04) << 3) | \
			(((u8)x & 0x02) << 5) | \
			(((u8)x & 0x01) << 7))
#endif

#ifndef MSB2LSB32
#define MSB2LSB32(x) (((u32)(MSB2LSB8(((u32)(x) >> 24) & 0xff)) << 24) | \
			((u32)(MSB2LSB8(((u32)(x) >> 16) & 0xff)) << 16) | \
			((u32)(MSB2LSB8(((u32)(x) >> 8) & 0xff)) << 8) | \
			(u32)(MSB2LSB8((u32)(x) & 0xff)))
#endif

/**
 * rk1608_write - RK1608_2_APB synchronous write
 *
 * @spi: spi device
 * @addr: apb resource address
 * @data: data buffer
 * @data_len: data buffer size, in bytes
 * Context: can sleep
 *
 * It returns zero on success, else a negative error code.
 */
int rk1608_write(struct spi_device *spi, s32 addr,
		 const s32 *data, size_t data_len);

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
			      s32 addr, const s32 *data, size_t data_len);
/**
 * rk1608_w32 - RK1608_2_APB synchronous 32-bit write
 *
 * @spi: spi device
 * @addr: apb resource address
 * @data: 32-bit data
 * Context: can sleep
 *
 * It returns zero on success, else a negative error code.
 */
int rk1608_safe_w32(struct spi_device *spi,
		    s32 addr, s32 data);

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
		    s32 addr, s32 data);

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
int rk1608_read(struct spi_device *spi, s32 addr,
		s32 *data, size_t data_len);

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
		     s32 addr, s32 *data, size_t data_len);

/**
 * rk1608_r32 - RK1608_2_APB synchronous 32-bit read
 *
 * @spi: spi device
 * @addr: apb resource address
 * @data: 32-bit data buffer [out]
 * Context: can sleep
 *
 * It returns zero on success, else a negative error code.
 */
int rk1608_safe_r32(struct spi_device *spi, s32 addr, s32 *data);

/**
 * rk1608_operation_query - RK1608_2_APB last operation state query
 *
 * @spi: spi device
 * @state: last operation state [out]
 * Context: can sleep
 *
 * It returns zero on success, else a negative error code.
 */
int rk1608_operation_query(struct spi_device *spi, s32 *state);

/**
 * rk1608_interrupt_request - RK1608_2_APB request a rk1608 interrupt
 *
 * @spi: spi device
 * @interrupt_num: interrupt identification
 * Context: can sleep
 *
 * It returns zero on success, else a negative error code.
 */
int rk1608_interrupt_request(struct spi_device *spi,
			     s32 interrupt_num);

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
		      s32 addr, const s32 *data, size_t data_len);

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
int rk1608_safe_w32(struct spi_device *spi, s32 addr, s32 data);

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
		     s32 addr, s32 *data, size_t data_len);

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
int rk1608_safe_r32(struct spi_device *spi, s32 addr, s32 *data);

struct rkl_section {
	union {
		u32	offset;
		u32	wait_value;
	};
	u32 size;
	union {
		u32	load_addr;
		u32	wait_addr;
	};
	u16	wait_time;
	u16	timeout;
	u16	crc_16;
	u8	flag;
	u8	type;
};

#define RKL_MAX_SECTION_NUM 10
struct rkl_header {
	char version[32];
	u32 header_size;
	u32 section_count;
	struct rkl_section sections[RKL_MAX_SECTION_NUM];
};

#define BOOT_FLAG_CRC		(1 << 0)
#define BOOT_FLAG_EXE		(1 << 1)
#define BOOT_FLAG_LOAD_PMEM	(1 << 2)
#define BOOT_FLAG_ACK		(1 << 3)
#define BOOT_FLAG_READ_WAIT	(1 << 4)
#define BOOT_FLAG_BOOT_REQUEST	(1 << 5)

struct rkl_boot_request {
	u32 flag;
	u32 load_addr;
	u32 boot_len;
	u8 status;
	u8 dummy[2];
	u8 cmd;
};

#define BOOT_REQUEST_ADDR		0x18000010
#define BOOT_REQUEST_WAIT_DELAY_MS	1
#define BOOT_REQUEST_TIMEOUT		10
#define RK1608_HEAD_ADDR		0x60000000
#define RKL_DEFAULT_FW_NAME		"rk1608.rkl"
#define RKPREISP_VERSION		"1.0.6"
#define PREISP_FW_NAME_LEN		128
#define RK1608_R_MSG_QUEUE_ADDR		0x60050000
#define RK1608_S_MSG_QUEUE_ADDR		0x60050010
#define MSG_QUEUE_DEFAULT_SIZE		(8 * 1024)

struct msg_queue {
	u32 *buf_head; /* msg buffer head */
	u32 *buf_tail; /* msg buffer tail */
	u32 *cur_send; /* current msg send postition */
	u32 *cur_recv; /* current msg receive position */
};

#define RK1608_PMU_SYS_REG0		0x120000f0
#define RK1608_MSG_QUEUE_OK_MASK	0xffff0001
#define RK1608_MSG_QUEUE_OK_TAG		0x16080001

struct rk1608_msg_queue {
	u32 buf_head; /* msg buffer head */
	u32 buf_tail; /* msg buffer tail */
	u32 cur_send; /* current msg send postition */
	u32 cur_recv; /* current msg receive position */
};

struct msg {
	u32 size; /* unit 4 bytes */
	u16 type; /* msg identification */
	s8  camera_id;
	s8  sync;
};

enum {
	/* AP -> RK1608
	*   1 msg of sensor
	*/
	id_msg_init_sensor_t =		0x0001,
	id_msg_set_input_size_t,
	id_msg_set_output_size_t,
	id_msg_set_stream_in_on_t,
	id_msg_set_stream_in_off_t,
	id_msg_set_stream_out_on_t,
	id_msg_set_stream_out_off_t,

	/* AP -> RK1608
	*   2 msg of take picture
	*/
	id_msg_take_picture_t =		0x0021,
	id_msg_take_picture_done_t,

	/* AP -> RK1608
	*   3 msg of realtime parameter
	*/
	id_msg_rt_args_t =		0x0031,

	/* AP -> RK1608
	*   4 msg of power manager
	*/
	id_msg_set_sys_mode_bypass_t =	0x0200,
	id_msg_set_sys_mode_standby_t,
	id_msg_set_sys_mode_idle_enable_t,
	id_msg_set_sys_mode_idle_disable_t,
	id_msg_set_sys_mode_slave_rk1608_on_t,
	id_msg_set_sys_mode_slave_rk1608_off_t,

	/* AP -> RK1608
	*   5 msg of debug config
	*/
	id_msg_set_log_level_t =	0x0250,

	/* RK1608 -> AP
	*   6 response of sensor msg
	*/
	id_msg_init_sensor_ret_t =	0x0301,
	id_msg_set_input_size_ret_t,
	id_msg_set_output_size_ret_t,
	id_msg_set_stream_in_on_ret_t,
	id_msg_set_stream_in_off_ret_t,
	id_msg_set_stream_out_on_ret_t,
	id_msg_set_stream_out_off_ret_t,

	/* RK1608 -> AP
	*   7 response of take picture msg
	*/
	id_msg_take_picture_ret_t =	0x0320,
	id_msg_take_picture_done_ret_t,

	/* RK1608 -> AP
	*   8 response of realtime parameter msg
	*/
	id_msg_rt_args_ret_t =		0x0330,

	/*rk1608 -> ap*/
	id_msg_do_i2c_t =		0x0390,
	/*ap -> rk1608*/
	id_msg_do_i2c_ret_t,

	/* RK1608 -> AP
	*   9 msg of print log
	*/
	id_msg_rk1608_log_t =		0x0400,

	/* dsi2csi dump */
	id_msg_dsi2sci_rgb_dump_t =	0x6000,
	id_msg_dsi2sci_nv12_dump_t =	0x6001,

	/* RK1608 -> AP
	*	10  msg of xfile
	*/
	id_msg_xfile_import_t =		0x8000 + 0x0600,
	id_msg_xfile_export_t,
	id_msg_xfile_mkdir_t
};

struct msg_rk1608_log_t {
	u32	size;
	u16	type;
	s8	core_id;
	s8	log_level;
};

static int rk1608_read_wait(struct spi_device *spi,
			    const struct rkl_section *sec);

static int rk1608_boot_request(struct spi_device *spi,
			       const struct rkl_section *sec);

static int rk1608_download_section(struct spi_device *spi, const u8 *data,
				   const struct rkl_section *sec);
/**
 * rk1608_download_fw: - rk1608 firmware download through spi
 *
 * @spi: spi device
 * @fw_name: name of firmware file, NULL for default firmware name
 * Context: can sleep
 *
 * It returns zero on success, else a negative error code.
 **/
int rk1608_download_fw(struct spi_device *spi, const char *fw_name);

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
			 u32 addr, struct rk1608_msg_queue *q);

/**
 * rk1608_msq_recv_msg - receive a msg from RK1608 -> AP msg queue
 *
 * @q: msg queue
 * @m: a msg pointer buf [out]
 *
 * need call rk1608_msq_free_received_msg to free msg after msg use done
 *
 * It returns zero on success, else a negative error code.
 */
int rk1608_msq_recv_msg(struct spi_device *spi, struct msg **m);
#endif
