/*
* Copyright (C) 2011-2016 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * Copyright 2018-2019 NXP
 * Copyright 2026 Bruetsch Technology AG
 *
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

#include "oah0428_mode_tbls.h"
#include "vvsensor.h"

#define MIN_FPS 30
#define MAX_FPS 30
#define DEFAULT_FPS 30

#define OAH0428_VVS_SW_RESET_ADDR 0x0103
#define OAH0428_VVS_BRIDGE_ID_ADDR_MSB 0x302A
#define OAH0428_VVS_BRIDGE_ID_ADDR_LSB 0x302B
#define OAH0428_VVS_CHIP_ID_ADDR_MSB 0x300A
#define OAH0428_VVS_CHIP_ID_ADDR_LSB 0x300B
#define OAH0428_VVS_REG_MODE_SELECT 0x0100
#define OAH0428_VVS_MODE_STANDBY 0x00
#define OAH0428_VVS_MODE_STREAMING 0x01
#define OAH0428_VVS_REG_EXPOSURE_H 0x3501
#define OAH0428_VVS_REG_EXPOSURE_L 0x3502
#define OAH0428_VVS_REG_GAIN_H 0x3508
#define OAH0428_VVS_REG_GAIN_L 0x3509

#define OAH0428_VVS_DEFAULT_FPS_Q10 (30 * (1 << SENSOR_FIX_FRACBITS))
#define OAH0428_VVS_DEFAULT_MCLK 24000000UL
#define OAH0428_VVS_CSI_MAX_PIXEL_CLK 100000000UL
#define OAH0428_VVS_400_MAX_EXPOSURE_LINES 0x01ca
#define OAH0428_VVS_720_MAX_EXPOSURE_LINES 0x02d6
#define OAH0428_VVS_MIN_EXPOSURE_LINES 4
#define OAH0428_VVS_DEFAULT_GAIN_Q8 0x0180
#define OAH0428_VVS_MIN_GAIN_Q8 0x0100
#define OAH0428_VVS_MAX_GAIN_Q8 0x0300

enum oah0428_vvs_mode {
	oah0428_vvs_mode_MIN = 0,
	oah0428_vvs_mode_1_1_400 = 0,
	oah0428_vvs_mode_1_1_720 = 1,
	oah0428_vvs_mode_MAX = 1,
	oah0428_vvs_mode_INIT = 0xff, /*only for sensor init*/
};

enum oah0428_vvs_frame_rate { oah0428_vvs_30_fps };

static int oah0428_vvs_framerates[] = {
	[oah0428_vvs_30_fps] = 30,
};

struct oah0428_vvs_datafmt {
	u32 code;
	enum v4l2_colorspace colorspace;
};

struct oah0428 {
	struct i2c_client *i2c_client;
	struct regmap *regmap;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */

	struct v4l2_captureparm streamcap;
	bool on;

	/* lock to protect all members below */
	struct mutex lock;

	struct v4l2_mbus_framefmt fmt;
	const struct oah0428_vvs_datafmt *datafmt;
	struct clk *sensor_clk;
	unsigned long xclk_freq;
	struct vvcam_mode_info_s vvcam_mode;
	u32 cur_mode;
	u32 fps_q10;
	u16 exposure_lines;
	u16 gain_q8;
	bool streaming;
};

struct oah0428_vvs_res {
	int width;
	int height;
};

struct oah0428_vvs_res oah0428_vvs_valid_res[] = {
	[oah0428_vvs_mode_1_1_400] = { 400, 400 },
	[oah0428_vvs_mode_1_1_720] = { 720, 720 },
};

static const struct oah0428_vvs_datafmt oah0428_vvs_colour_fmts[] = {
	{ MEDIA_BUS_FMT_SGRBG10_1X10, V4L2_COLORSPACE_RAW },
};

static struct vvcam_mode_info_s oah0428_vvs_modes[] = {
	{
		.index = oah0428_vvs_mode_1_1_400,
		.size = {
			.bounds_width = 400,
			.bounds_height = 400,
			.top = 0,
			.left = 0,
			.width = 400,
			.height = 400,
		},
		.hdr_mode = SENSOR_MODE_LINEAR,
		.bit_width = 10,
		.data_compress = {
			.enable = 0,
		},
		.bayer_pattern = BAYER_GRBG,
		.ae_info = {
			.def_frm_len_lines = 0x02dc,
			.curr_frm_len_lines = 0x02dc,
			.one_line_exp_time_ns = 45537,
			.max_integration_line = OAH0428_VVS_400_MAX_EXPOSURE_LINES,
			.min_integration_line = OAH0428_VVS_MIN_EXPOSURE_LINES,
			.max_again = 3 * (1 << SENSOR_FIX_FRACBITS),
			.min_again = 1 * (1 << SENSOR_FIX_FRACBITS),
			.max_dgain = 1 * (1 << SENSOR_FIX_FRACBITS),
			.min_dgain = 1 * (1 << SENSOR_FIX_FRACBITS),
			.start_exposure = 22 * (1 << SENSOR_FIX_FRACBITS),
			.gain_step = 1,
			.cur_fps = OAH0428_VVS_DEFAULT_FPS_Q10,
			.max_fps = OAH0428_VVS_DEFAULT_FPS_Q10,
			.min_fps = OAH0428_VVS_DEFAULT_FPS_Q10,
			.min_afps = OAH0428_VVS_DEFAULT_FPS_Q10,
			.int_update_delay_frm = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 1,
		},
		.preg_data = NULL,
		.reg_data_count = 0,
	},
	{
		.index = oah0428_vvs_mode_1_1_720,
		.size = {
			.bounds_width = 720,
			.bounds_height = 720,
			.top = 0,
			.left = 0,
			.width = 720,
			.height = 720,
		},
		.hdr_mode = SENSOR_MODE_LINEAR,
		.bit_width = 10,
		.data_compress = {
			.enable = 0,
		},
		.bayer_pattern = BAYER_BGGR,
		.ae_info = {
			.def_frm_len_lines = 0x02dc,
			.curr_frm_len_lines = 0x02dc,
			.one_line_exp_time_ns = 45537,
			.max_integration_line = OAH0428_VVS_720_MAX_EXPOSURE_LINES,
			.min_integration_line = OAH0428_VVS_MIN_EXPOSURE_LINES,
			.max_again = 3 * (1 << SENSOR_FIX_FRACBITS),
			.min_again = 1 * (1 << SENSOR_FIX_FRACBITS),
			.max_dgain = 1 * (1 << SENSOR_FIX_FRACBITS),
			.min_dgain = 1 * (1 << SENSOR_FIX_FRACBITS),
			.start_exposure = 32 * (1 << SENSOR_FIX_FRACBITS),
			.gain_step = 1,
			.cur_fps = OAH0428_VVS_DEFAULT_FPS_Q10,
			.max_fps = OAH0428_VVS_DEFAULT_FPS_Q10,
			.min_fps = OAH0428_VVS_DEFAULT_FPS_Q10,
			.min_afps = OAH0428_VVS_DEFAULT_FPS_Q10,
			.int_update_delay_frm = 1,
			.gain_update_delay_frm = 1,
		},
		.mipi_info = {
			.mipi_lane = 1,
		},
		.preg_data = NULL,
		.reg_data_count = 0,
	},
};

static const struct regmap_config oah0428_vvs_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0xffff,
};

static int get_capturemode(int width, int height)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(oah0428_vvs_valid_res); i++) {
		if ((oah0428_vvs_valid_res[i].width == width) &&
		    (oah0428_vvs_valid_res[i].height == height))
			return i;
	}

	return -1;
}

static struct oah0428 *to_oah0428(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct oah0428, sd);
}

/* Find a data format by a pixel code in an array */
static const struct oah0428_vvs_datafmt *oah0428_vvs_find_datafmt(u32 code)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(oah0428_vvs_colour_fmts); i++)
		if (oah0428_vvs_colour_fmts[i].code == code)
			return oah0428_vvs_colour_fmts + i;

	return NULL;
}

static int oah0428_vvs_write_reg(struct oah0428 *sensor, u16 reg, u8 val)
{
	return regmap_write(sensor->regmap, reg, val);
}

static int oah0428_vvs_read_reg(struct oah0428 *sensor, u16 reg, u8 *val)
{
	unsigned int reg_val;
	int ret;

	ret = regmap_read(sensor->regmap, reg, &reg_val);
	if (ret)
		return ret;

	*val = reg_val;
	return 0;
}

static inline struct oah0428 *sd_to_oah0428(struct v4l2_subdev *sd)
{
	return container_of(sd, struct oah0428, sd);
}

static int oah0428_vvs_write_reg16(struct oah0428 *sensor, u16 reg_h, u16 reg_l,
				   u16 val)
{
	int ret;

	ret = oah0428_vvs_write_reg(sensor, reg_h, val >> 8);
	if (ret)
		return ret;

	return oah0428_vvs_write_reg(sensor, reg_l, val & 0xff);
}

static int oah0428_vvs_read_chip_id(struct oah0428 *sensor, u32 *id)
{
	u8 msb, lsb;
	int ret;

	ret = oah0428_vvs_read_reg(sensor, OAH0428_VVS_CHIP_ID_ADDR_MSB, &msb);
	if (ret)
		return ret;
	ret = oah0428_vvs_read_reg(sensor, OAH0428_VVS_CHIP_ID_ADDR_LSB, &lsb);
	if (ret)
		return ret;

	*id = ((u32)msb << 8) | lsb;
	return 0;
}

static u32 oah0428_vvs_mode_max_exposure_lines(struct oah0428 *sensor)
{
	if (sensor->cur_mode == oah0428_vvs_mode_1_1_720)
		return OAH0428_VVS_720_MAX_EXPOSURE_LINES;

	return OAH0428_VVS_400_MAX_EXPOSURE_LINES;
}

static int oah0428_vvs_set_exposure_lines(struct oah0428 *sensor, u32 lines)
{
	lines = clamp_t(u32, lines, OAH0428_VVS_MIN_EXPOSURE_LINES,
			oah0428_vvs_mode_max_exposure_lines(sensor));
	sensor->exposure_lines = lines;

	return oah0428_vvs_write_reg16(sensor, OAH0428_VVS_REG_EXPOSURE_H,
				       OAH0428_VVS_REG_EXPOSURE_L, lines);
}

static int oah0428_vvs_set_gain_q8(struct oah0428 *sensor, u32 gain_q8)
{
	gain_q8 = clamp_t(u32, gain_q8, OAH0428_VVS_MIN_GAIN_Q8,
			  OAH0428_VVS_MAX_GAIN_Q8);
	sensor->gain_q8 = gain_q8;

	return oah0428_vvs_write_reg16(sensor, OAH0428_VVS_REG_GAIN_H,
				       OAH0428_VVS_REG_GAIN_L, gain_q8);
}

static void __maybe_unused oah0428_vvs_dump_mipi_state(struct oah0428 *sensor,
						       const char *ctx)
{
	struct device *dev = &sensor->i2c_client->dev;
	u8 mipi_ctrl_00 = 0, mipi_ctrl_01 = 0, mipi_ctrl_04 = 0;
	u8 mipi_ctrl_0b = 0, mipi_ctrl_0c = 0, mipi_ctrl_0e = 0;
	u8 mipi_ctrl_13 = 0, mipi_ctrl_27 = 0;
	u8 mipi_pclk = 0, mipi_lp_gpio = 0;
	u8 pll_01 = 0, pll_04 = 0, pll_05 = 0, pll_06 = 0, pll_07 = 0;
	u8 pll_24 = 0, pll_25 = 0, pll_2a = 0, pll_2b = 0;

	/* MIPI TX registers */
	oah0428_vvs_read_reg(sensor, 0x4800, &mipi_ctrl_00);
	oah0428_vvs_read_reg(sensor, 0x4801, &mipi_ctrl_01);
	oah0428_vvs_read_reg(sensor, 0x4804, &mipi_ctrl_04);
	oah0428_vvs_read_reg(sensor, 0x480b, &mipi_ctrl_0b);
	oah0428_vvs_read_reg(sensor, 0x480c, &mipi_ctrl_0c);
	oah0428_vvs_read_reg(sensor, 0x480e, &mipi_ctrl_0e);
	oah0428_vvs_read_reg(sensor, 0x4813, &mipi_ctrl_13);
	oah0428_vvs_read_reg(sensor, 0x4827, &mipi_ctrl_27);
	oah0428_vvs_read_reg(sensor, 0x4837, &mipi_pclk);
	oah0428_vvs_read_reg(sensor, 0x4803, &mipi_lp_gpio);

	/* PLL registers */
	oah0428_vvs_read_reg(sensor, 0x0301, &pll_01);
	oah0428_vvs_read_reg(sensor, 0x0304, &pll_04);
	oah0428_vvs_read_reg(sensor, 0x0305, &pll_05);
	oah0428_vvs_read_reg(sensor, 0x0306, &pll_06);
	oah0428_vvs_read_reg(sensor, 0x0307, &pll_07);
	oah0428_vvs_read_reg(sensor, 0x0324, &pll_24);
	oah0428_vvs_read_reg(sensor, 0x0325, &pll_25);
	oah0428_vvs_read_reg(sensor, 0x032a, &pll_2a);
	oah0428_vvs_read_reg(sensor, 0x032b, &pll_2b);

	dev_info(
		dev,
		"[%s] MIPI: 4800=%02x 4801=%02x 4803=%02x 4804=%02x 480b=%02x 480c=%02x 480e=%02x 4813=%02x 4827=%02x 4837=%02x\n",
		ctx, mipi_ctrl_00, mipi_ctrl_01, mipi_lp_gpio, mipi_ctrl_04,
		mipi_ctrl_0b, mipi_ctrl_0c, mipi_ctrl_0e, mipi_ctrl_13,
		mipi_ctrl_27, mipi_pclk);
	dev_info(
		dev,
		"[%s] PLL: 0301=%02x 0304=%02x 0305=%02x 0306=%02x 0307=%02x 0324=%02x 0325=%02x 032a=%02x 032b=%02x\n",
		ctx, pll_01, pll_04, pll_05, pll_06, pll_07, pll_24, pll_25,
		pll_2a, pll_2b);
}

static void __maybe_unused oah0428_vvs_dump_sensor_state(struct oah0428 *sensor,
							 const char *ctx)
{
	struct device *dev = &sensor->i2c_client->dev;
	u8 stream = 0, sensor_reset = 0, img_fmt = 0;
	u8 hts_hi = 0, hts_lo = 0, vts_hi = 0, vts_lo = 0;
	u8 x_out_hi = 0, x_out_lo = 0, y_out_hi = 0, y_out_lo = 0;
	u8 exp_hi = 0, exp_lo = 0, gain_hi = 0, gain_lo = 0;
	u8 isp_ctrl = 0, awb_ctrl = 0, blc_ctrl = 0;
	u8 aec_ctrl = 0;
	u8 spi_3200 = 0, spi_3201 = 0, spi_3202 = 0, spi_3204 = 0;
	u8 spi_3206 = 0, spi_3207 = 0, spi_3209 = 0;
	u8 clk_300d = 0, clk_300e = 0, clk_300f = 0;
	u8 clk_3013 = 0, clk_3107 = 0;

	oah0428_vvs_read_reg(sensor, 0x0100, &stream);
	oah0428_vvs_read_reg(sensor, 0x3205, &sensor_reset);
	oah0428_vvs_read_reg(sensor, 0x3208, &img_fmt);
	oah0428_vvs_read_reg(sensor, 0x380e, &hts_hi);
	oah0428_vvs_read_reg(sensor, 0x380f, &hts_lo);
	oah0428_vvs_read_reg(sensor, 0x3812, &vts_hi);
	oah0428_vvs_read_reg(sensor, 0x3813, &vts_lo);
	oah0428_vvs_read_reg(sensor, 0x3828, &x_out_hi);
	oah0428_vvs_read_reg(sensor, 0x3829, &x_out_lo);
	oah0428_vvs_read_reg(sensor, 0x382a, &y_out_hi);
	oah0428_vvs_read_reg(sensor, 0x382b, &y_out_lo);
	oah0428_vvs_read_reg(sensor, 0x3501, &exp_hi);
	oah0428_vvs_read_reg(sensor, 0x3502, &exp_lo);
	oah0428_vvs_read_reg(sensor, 0x3508, &gain_hi);
	oah0428_vvs_read_reg(sensor, 0x3509, &gain_lo);
	oah0428_vvs_read_reg(sensor, 0x5005, &isp_ctrl);
	oah0428_vvs_read_reg(sensor, 0x5004, &awb_ctrl);
	oah0428_vvs_read_reg(sensor, 0x4004, &blc_ctrl);
	oah0428_vvs_read_reg(sensor, 0x3503, &aec_ctrl);

	/* SPI control registers — bridge-to-sensor communication */
	oah0428_vvs_read_reg(sensor, 0x3200, &spi_3200);
	oah0428_vvs_read_reg(sensor, 0x3201, &spi_3201);
	oah0428_vvs_read_reg(sensor, 0x3202, &spi_3202);
	oah0428_vvs_read_reg(sensor, 0x3204, &spi_3204);
	oah0428_vvs_read_reg(sensor, 0x3206, &spi_3206);
	oah0428_vvs_read_reg(sensor, 0x3207, &spi_3207);
	oah0428_vvs_read_reg(sensor, 0x3209, &spi_3209);

	/* Internal clock config */
	oah0428_vvs_read_reg(sensor, 0x300d, &clk_300d);
	oah0428_vvs_read_reg(sensor, 0x300e, &clk_300e);
	oah0428_vvs_read_reg(sensor, 0x300f, &clk_300f);
	oah0428_vvs_read_reg(sensor, 0x3013, &clk_3013);
	oah0428_vvs_read_reg(sensor, 0x3107, &clk_3107);

	dev_info(
		dev,
		"[%s] SENSOR: stream=%02x reset=%02x imgfmt=%02x HTS=%02x%02x VTS=%02x%02x output=%02x%02x x %02x%02x\n",
		ctx, stream, sensor_reset, img_fmt, hts_hi, hts_lo, vts_hi,
		vts_lo, x_out_hi, x_out_lo, y_out_hi, y_out_lo);
	dev_info(
		dev,
		"[%s] EXPOSURE: exp=%02x%02x gain=%02x%02x aec_ctrl=%02x ISP: 5005=%02x 5004=%02x BLC: 4004=%02x\n",
		ctx, exp_hi, exp_lo, gain_hi, gain_lo, aec_ctrl, isp_ctrl,
		awb_ctrl, blc_ctrl);
	dev_info(
		dev,
		"[%s] SPI_CTRL: 3200=%02x 3201=%02x 3202=%02x 3204=%02x 3206=%02x 3207=%02x 3209=%02x\n",
		ctx, spi_3200, spi_3201, spi_3202, spi_3204, spi_3206, spi_3207,
		spi_3209);
	dev_info(
		dev,
		"[%s] CLOCKS: 300d=%02x 300e=%02x 300f=%02x 3013=%02x 3107=%02x\n",
		ctx, clk_300d, clk_300e, clk_300f, clk_3013, clk_3107);
}

static void __maybe_unused
oah0428_vvs_check_frame_counter(struct oah0428 *sensor, const char *ctx)
{
	struct device *dev = &sensor->i2c_client->dev;
	u8 cnt1 = 0, cnt2 = 0;
	int ret;

	ret = oah0428_vvs_read_reg(sensor, 0x3206, &cnt1);
	if (ret) {
		dev_err(dev, "[%s] frame counter read failed: %d\n", ctx, ret);
		return;
	}

	msleep(150); /* wait ~4-5 frames at 30fps */

	ret = oah0428_vvs_read_reg(sensor, 0x3206, &cnt2);
	if (ret) {
		dev_err(dev, "[%s] frame counter read 2 failed: %d\n", ctx,
			ret);
		return;
	}

	if (cnt1 == cnt2)
		dev_err(dev,
			"[%s] FRAME COUNTER STUCK: 0x3206=%02x -> %02x (sensor NOT generating frames!)\n",
			ctx, cnt1, cnt2);
	else
		dev_info(
			dev,
			"[%s] frame counter OK: 0x3206=%02x -> %02x (delta=%d, sensor is producing frames)\n",
			ctx, cnt1, cnt2, (u8)(cnt2 - cnt1));
}

static int __maybe_unused oah0428_vvs_sensor_start(struct oah0428 *sensor)
{
	struct device *dev = &sensor->i2c_client->dev;
	u8 bridge_id_msb;
	u8 bridge_id_lsb;
	u16 bridge_id;
	int ret;

	dev_info(dev, "sensor_start: issuing software reset\n");

	ret = oah0428_vvs_write_reg(sensor, OAH0428_VVS_SW_RESET_ADDR, 0x01);
	if (ret) {
		dev_err(dev, "software reset failed (I2C NACK?): %d\n", ret);
		return ret;
	}

	usleep_range(1000, 2000);

	ret = oah0428_vvs_read_reg(sensor, OAH0428_VVS_BRIDGE_ID_ADDR_MSB,
				   &bridge_id_msb);
	if (ret) {
		dev_err(dev, "bridge ID MSB read failed: %d\n", ret);
		return ret;
	}

	ret = oah0428_vvs_read_reg(sensor, OAH0428_VVS_BRIDGE_ID_ADDR_LSB,
				   &bridge_id_lsb);
	if (ret) {
		dev_err(dev, "bridge ID LSB read failed: %d\n", ret);
		return ret;
	}

	bridge_id = ((u16)bridge_id_msb << 8) | bridge_id_lsb;

	dev_info(dev, "bridge id: 0x%04x\n", bridge_id);

	if (bridge_id == 0x0000 || bridge_id == 0xffff)
		dev_err(dev,
			"WARNING: bridge id 0x%04x looks invalid — bridge not responding?\n",
			bridge_id);

	return 0;
}

static int __maybe_unused oah0428_vvs_get_chip_id(struct oah0428 *sensor)
{
	struct device *dev = &sensor->i2c_client->dev;
	u8 chip_id_msb;
	u8 chip_id_lsb;
	u16 chip_id;
	int ret;

	ret = oah0428_vvs_read_reg(sensor, OAH0428_VVS_CHIP_ID_ADDR_MSB,
				   &chip_id_msb);
	if (ret) {
		dev_err(dev, "chip ID MSB read failed: %d\n", ret);
		return ret;
	}

	ret = oah0428_vvs_read_reg(sensor, OAH0428_VVS_CHIP_ID_ADDR_LSB,
				   &chip_id_lsb);
	if (ret) {
		dev_err(dev, "chip ID LSB read failed: %d\n", ret);
		return ret;
	}

	chip_id = ((u16)chip_id_msb << 8) | chip_id_lsb;

	dev_info(dev, "chip id: 0x%04x\n", chip_id);

	if (chip_id == 0x0000 || chip_id == 0xffff) {
		dev_err(dev,
			"WARNING: chip id 0x%04x looks invalid — I2C communication problem?\n",
			chip_id);
	}

	return 0;
}

static void __maybe_unused oah0428_vvs_log_stream_debug(struct oah0428 *sensor)
{
	struct device *dev = &sensor->i2c_client->dev;
	u8 stream = 0;
	u8 frame_cnt_1 = 0;
	u8 frame_cnt_2 = 0;
	u8 chip_hi = 0;
	u8 chip_lo = 0;
	u8 cbar_sel = 0;
	u8 pre_isp = 0;
	u8 pre_isp_mode = 0;
	u8 mipi_cbar = 0;
	u8 mipi_phy_test = 0;
	u8 sensor_reset = 0;
	u8 img_fmt = 0;
	u8 hts_hi = 0;
	u8 hts_lo = 0;
	u8 vts_hi = 0;
	u8 vts_lo = 0;
	u8 pclk_div = 0;

	if (!oah0428_vvs_read_reg(sensor, 0x0100, &stream) &&
	    !oah0428_vvs_read_reg(sensor, 0x3206, &frame_cnt_1) &&
	    !oah0428_vvs_read_reg(sensor, 0x300a, &chip_hi) &&
	    !oah0428_vvs_read_reg(sensor, 0x300b, &chip_lo) &&
	    !oah0428_vvs_read_reg(sensor, 0x3205, &sensor_reset) &&
	    !oah0428_vvs_read_reg(sensor, 0x3208, &img_fmt) &&
	    !oah0428_vvs_read_reg(sensor, 0x380e, &hts_hi) &&
	    !oah0428_vvs_read_reg(sensor, 0x380f, &hts_lo) &&
	    !oah0428_vvs_read_reg(sensor, 0x3812, &vts_hi) &&
	    !oah0428_vvs_read_reg(sensor, 0x3813, &vts_lo) &&
	    !oah0428_vvs_read_reg(sensor, 0x4837, &pclk_div) &&
	    !oah0428_vvs_read_reg(sensor, 0x3660, &cbar_sel) &&
	    !oah0428_vvs_read_reg(sensor, 0x5005, &pre_isp) &&
	    !oah0428_vvs_read_reg(sensor, 0x5140, &pre_isp_mode) &&
	    !oah0428_vvs_read_reg(sensor, 0x48c0, &mipi_cbar) &&
	    !oah0428_vvs_read_reg(sensor, 0x4803, &mipi_phy_test)) {
		msleep(120);
		if (oah0428_vvs_read_reg(sensor, 0x3206, &frame_cnt_2))
			frame_cnt_2 = 0xff;

		dev_info(
			dev,
			"stream dbg: 0100=%02x 3206=%02x->%02x chip=%02x%02x 3205=%02x 3208=%02x 380e=%02x 380f=%02x 3812=%02x 3813=%02x 4837=%02x 3660=%02x 5005=%02x 5140=%02x 48c0=%02x 4803=%02x\n",
			stream, frame_cnt_1, frame_cnt_2, chip_hi, chip_lo,
			sensor_reset, img_fmt, hts_hi, hts_lo, vts_hi, vts_lo,
			pclk_div, cbar_sel, pre_isp, pre_isp_mode, mipi_cbar,
			mipi_phy_test);
	}
}

int oah0428_vvs_write_mode_table(struct device *dev, struct regmap *regmap,
				 const oah0428_reg table[])
{
	int i;
	int ret;

	if (!table)
		return -EINVAL;

	if (!regmap)
		return -ENODEV;

	for (i = 0;; i++) {
		u16 addr = table[i].addr;
		u8 val = table[i].val;

		if (addr == OAH0428_TABLE_END)
			break;

		if (addr == OAH0428_TABLE_WAIT_MS) {
			if (val)
				msleep(val);
			continue;
		}

		ret = regmap_write(regmap, addr, val);
		if (ret) {
			dev_err(dev,
				"mode table write failed at 0x%04x = 0x%02x: %d\n",
				addr, val, ret);
			return ret;
		}
	}

	return 0;
}

static int oah0428_vvs_init_mode(struct oah0428 *sensor,
				 enum oah0428_vvs_mode mode)
{
	struct device *dev = &sensor->i2c_client->dev;
	int retval = 0;
	const oah0428_reg *table;

	if ((mode > oah0428_vvs_mode_MAX || mode < oah0428_vvs_mode_MIN) &&
	    (mode != oah0428_vvs_mode_INIT)) {
		dev_err(dev, "Wrong oah0428 mode detected!\n");
		return -1;
	}

	if (mode == oah0428_vvs_mode_INIT) {
		sensor->fmt.width = oah0428_vvs_valid_res[0].width;
		sensor->fmt.height = oah0428_vvs_valid_res[0].height;
		table = mode_table[OAH0428_MODE_400X400_30FPS];
	} else {
		sensor->fmt.width = oah0428_vvs_valid_res[mode].width;
		sensor->fmt.height = oah0428_vvs_valid_res[mode].height;
		table = mode_table[mode];
	}
	dev_info(dev, "Initializing oah0428 mode %d\n", mode);
	retval = oah0428_vvs_write_mode_table(dev, sensor->regmap, table);
	if (retval)
		goto err;

err:
	return retval;
}

/*!
 * oah0428_vvs_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int oah0428_vvs_s_power(struct v4l2_subdev *sd, int on)
{
	struct oah0428 *sensor = sd_to_oah0428(sd);
	int ret = 0;

	mutex_lock(&sensor->lock);
	if (on && !sensor->on) {
		ret = clk_prepare_enable(sensor->sensor_clk);
		if (!ret)
			sensor->on = true;
	} else if (!on && sensor->on) {
		if (sensor->streaming)
			oah0428_vvs_write_mode_table(
				&sensor->i2c_client->dev, sensor->regmap,
				mode_table[OAH0428_MODE_STOP_STREAM]);
		clk_disable_unprepare(sensor->sensor_clk);
		sensor->on = false;
		sensor->streaming = false;
	}
	mutex_unlock(&sensor->lock);

	return ret;
}

/*!
 * oah0428_vvs_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 sub device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int oah0428_vvs_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct oah0428 *sensor = to_oah0428(client);
	struct device *dev = &sensor->i2c_client->dev;
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		dev_warn(dev, "Type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * oah0428_vvs_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 sub device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int oah0428_vvs_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct oah0428 *sensor = to_oah0428(client);
	struct device *dev = &sensor->i2c_client->dev;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 tgt_fps; /* target frames per secound */
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		/* Check that the new frame rate is allowed. */
		if ((timeperframe->numerator == 0) ||
		    (timeperframe->denominator == 0)) {
			timeperframe->denominator = DEFAULT_FPS;
			timeperframe->numerator = 1;
		}

		tgt_fps = timeperframe->denominator / timeperframe->numerator;

		if (tgt_fps > MAX_FPS) {
			timeperframe->denominator = MAX_FPS;
			timeperframe->numerator = 1;
		} else if (tgt_fps < MIN_FPS) {
			timeperframe->denominator = MIN_FPS;
			timeperframe->numerator = 1;
		}

		/* Actual frame rate we use */
		tgt_fps = timeperframe->denominator / timeperframe->numerator;

		if (tgt_fps != 30) {
			dev_warn(dev,
				 "The camera frame rate is not supported!\n");
			return -EINVAL;
		}

		// ret = oah0428_vvs_init_mode(
		// 	sensor, oah0428_vvs_mode_1_1_400);
		// if (ret < 0)
		// 	return ret;

		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode =
			(u32)a->parm.capture.capturemode;

		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		dev_warn(dev,
			 "Type is not V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			 a->type);
		ret = -EINVAL;
		break;

	default:
		dev_warn(dev, "Type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int oah0428_vvs_set_fmt(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *sd_state,
			       struct v4l2_subdev_format *format)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct oah0428 *sensor = to_oah0428(client);

	struct v4l2_mbus_framefmt *fmt = &format->format;
	const struct oah0428_vvs_datafmt *datafmt =
		oah0428_vvs_find_datafmt(fmt->code);
	int capturemode;
	int ret = 0;

	mutex_lock(&sensor->lock);

	if (!datafmt) {
		fmt->code = oah0428_vvs_colour_fmts[0].code;
		fmt->colorspace = oah0428_vvs_colour_fmts[0].colorspace;
		datafmt = &oah0428_vvs_colour_fmts[0];
	}

	fmt->field = V4L2_FIELD_NONE;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_state_get_format(sd_state, 0) = *fmt;
		goto out;
	}

	sensor->datafmt = datafmt;

	capturemode = get_capturemode(fmt->width, fmt->height);
	if (capturemode >= 0) {
		sensor->streamcap.capturemode = capturemode;
		sensor->fmt.width = fmt->width;
		sensor->fmt.height = fmt->height;
		goto out;
	}

	dev_err(&client->dev, "Set format failed %d, %d\n", datafmt->code,
		datafmt->colorspace);
	ret = -EINVAL;
out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int oah0428_vvs_get_fmt(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *sd_state,
			       struct v4l2_subdev_format *format)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct oah0428 *sensor = to_oah0428(client);

	struct v4l2_mbus_framefmt *fmt;

	if (format->pad)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		fmt = v4l2_subdev_state_get_format(sd_state, format->pad);
	} else {
		fmt = &sensor->fmt;
	}

	format->format = *fmt;

	mutex_unlock(&sensor->lock);

	return 0;
}

static int oah0428_vvs_enum_mbus_code(struct v4l2_subdev *sd,
				      struct v4l2_subdev_state *sd_state,
				      struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= ARRAY_SIZE(oah0428_vvs_colour_fmts))
		return -EINVAL;

	code->code = oah0428_vvs_colour_fmts[code->index].code;
	return 0;
}

/*!
 * oah0428_vvs_enum_frame_size - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int oah0428_vvs_enum_frame_size(struct v4l2_subdev *sd,
				       struct v4l2_subdev_state *sd_state,
				       struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > oah0428_vvs_mode_MAX)
		return -EINVAL;

	fse->max_width = oah0428_vvs_valid_res[fse->index].width;
	fse->min_width = oah0428_vvs_valid_res[fse->index].width;
	fse->max_height = oah0428_vvs_valid_res[fse->index].height;
	fse->min_height = oah0428_vvs_valid_res[fse->index].height;
	return 0;
}

/*!
 * oah0428_vvs_enum_frame_interval - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int
oah0428_vvs_enum_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_frame_interval_enum *fie)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *dev = &client->dev;
	int j, count = 0;

	if (fie->index < 0 || fie->index > oah0428_vvs_mode_MAX)
		return -EINVAL;

	if (fie->width == 0 || fie->height == 0 || fie->code == 0) {
		dev_warn(dev, "Please assign pixel format, width and height\n");
		return -EINVAL;
	}

	fie->interval.numerator = 1;

	count = 0;
	for (j = 0; j < (oah0428_vvs_mode_MAX + 1); j++) {
		if (fie->width == oah0428_vvs_valid_res[j].width &&
		    fie->height == oah0428_vvs_valid_res[j].height) {
			count++;
		}
		if (fie->index == (count - 1)) {
			fie->interval.denominator = oah0428_vvs_framerates[0];
			return 0;
		}
	}

	return -EINVAL;
}

/*!
 * dev_init - V4L2 sensor init
 * @s: pointer to standard V4L2 device structure
 *
 */
static int init_device(struct oah0428 *sensor)
{
	struct device *dev = &sensor->i2c_client->dev;
	u32 tgt_fps; /* target frames per secound */

	sensor->on = true;

	/* Default camera frame rate is set in probe */
	tgt_fps = sensor->streamcap.timeperframe.denominator /
		  sensor->streamcap.timeperframe.numerator;

	dev_info(dev, "init_device: target fps=%u, capturemode=%u\n", tgt_fps,
		 sensor->streamcap.capturemode);

	if (tgt_fps != 30)
		return -EINVAL; /* Only 30fps now. */

	return 0;
}

static int oah0428_vvs_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct oah0428 *sensor = to_oah0428(client);
	struct device *dev = &sensor->i2c_client->dev;
	int ret = 0;

	dev_info(dev,
		 "s_stream: enable=%d capturemode=%u fmt=%ux%u code=0x%04x\n",
		 enable, sensor->streamcap.capturemode, sensor->fmt.width,
		 sensor->fmt.height, sensor->fmt.code);

	mutex_lock(&sensor->lock);

	// if (enable) {
	// 	if (!sensor->on) {
	// 		ret = clk_prepare_enable(sensor->sensor_clk);
	// 		if (ret)
	// 			goto out;
	// 		sensor->on = true;
	// 	}
	//
	// 	ret = oah0428_vvs_init_mode(sensor,
	// 				    sensor->streamcap.capturemode);
	// 	if (ret) {
	// 		dev_err(dev, "s_stream: init_mode failed: %d\n", ret);
	// 		goto out;
	// 	}
	//
	// 	ret = oah0428_vvs_write_mode_table(
	// 		dev, sensor->regmap,
	// 		mode_table[OAH0428_MODE_START_STREAM]);
	// 	if (ret) {
	// 		dev_err(dev, "s_stream: start stream failed: %d\n",
	// 			ret);
	// 		goto out;
	// 	}
	//
	// 	sensor->streaming = true;
	// } else {
	// 	ret = oah0428_vvs_write_mode_table(
	// 		dev, sensor->regmap,
	// 		mode_table[OAH0428_MODE_STOP_STREAM]);
	// 	if (ret)
	// 		goto out;
	//
	// 	sensor->streaming = false;
	// }

out:
	mutex_unlock(&sensor->lock);
	return ret;
}

static int oah0428_vvs_vv_querycap(struct oah0428 *sensor, void *args)
{
	struct device *dev = &sensor->i2c_client->dev;
	struct v4l2_capability *cap = (struct v4l2_capability *)args;
	const char *csi_id;

	strscpy(cap->driver, "oah0428-vvs", sizeof(cap->driver));

	if (!of_property_read_string(dev->of_node, "isp-bus-info", &csi_id))
		strscpy(cap->bus_info, csi_id, sizeof(cap->bus_info));
	else
		strscpy(cap->bus_info, "csi0", sizeof(cap->bus_info));

	if (sensor->i2c_client->adapter)
		cap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] =
			(__u8)sensor->i2c_client->adapter->nr;
	else
		cap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] = 0xff;

	return 0;
}

static int oah0428_vvs_vv_querymode(struct oah0428 *sensor, void *args)
{
	struct vvcam_mode_info_array_s array = { 0 };

	array.count = ARRAY_SIZE(oah0428_vvs_modes);
	memcpy(array.modes, oah0428_vvs_modes, sizeof(oah0428_vvs_modes));

	return copy_to_user(args, &array, sizeof(array)) ? -EIO : 0;
}

static int oah0428_vvs_vv_get_sensormode(struct oah0428 *sensor, void *args)
{
	struct vvcam_mode_info_s mode;

	mutex_lock(&sensor->lock);
	mode = sensor->vvcam_mode;
	mode.ae_info.cur_fps = sensor->fps_q10;
	mutex_unlock(&sensor->lock);

	return copy_to_user(args, &mode, sizeof(mode)) ? -EIO : 0;
}

static int oah0428_vvs_vv_set_sensormode(struct oah0428 *sensor, void *args)
{
	struct vvcam_mode_info_s mode;

	if (copy_from_user(&mode, args, sizeof(mode)))
		return -EIO;

	if (mode.index >= ARRAY_SIZE(oah0428_vvs_modes))
		return -EINVAL;

	mutex_lock(&sensor->lock);
	sensor->cur_mode = mode.index;
	sensor->vvcam_mode = oah0428_vvs_modes[mode.index];
	sensor->streamcap.capturemode = mode.index;
	sensor->fmt.width = sensor->vvcam_mode.size.width;
	sensor->fmt.height = sensor->vvcam_mode.size.height;
	sensor->fmt.code = oah0428_vvs_colour_fmts[0].code;
	mutex_unlock(&sensor->lock);

	return 0;
}

static int oah0428_vvs_vv_s_stream(struct oah0428 *sensor, void *args)
{
	u32 enable;

	if (copy_from_user(&enable, args, sizeof(enable)))
		return -EIO;

	return oah0428_vvs_s_stream(&sensor->sd, enable);
}
//
// static int oah0428_vvs_vv_s_exp(struct oah0428 *sensor, void *args)
// {
// 	u32 exp_q10, exposure_us, lines;
// 	u32 line_ns;
// 	int ret;
//
// 	if (copy_from_user(&exp_q10, args, sizeof(exp_q10)))
// 		return -EIO;
//
// 	exposure_us = exp_q10 / 1024;
// 	line_ns = sensor->vvcam_mode.ae_info.one_line_exp_time_ns;
// 	if (!line_ns)
// 		line_ns = 45537;
// 	lines = DIV_ROUND_CLOSEST(exposure_us * 1000, line_ns);
//
// 	mutex_lock(&sensor->lock);
// 	ret = oah0428_vvs_set_exposure_lines(sensor, lines);
// 	mutex_unlock(&sensor->lock);
//
// 	return ret;
// }
//
// static int oah0428_vvs_vv_s_gain(struct oah0428 *sensor, void *args)
// {
// 	u32 gain_q10, gain_q8;
// 	int ret;
//
// 	if (copy_from_user(&gain_q10, args, sizeof(gain_q10)))
// 		return -EIO;
//
// 	gain_q8 = DIV_ROUND_CLOSEST(gain_q10 * 256, 1024);
//
// 	mutex_lock(&sensor->lock);
// 	ret = oah0428_vvs_set_gain_q8(sensor, gain_q8);
// 	mutex_unlock(&sensor->lock);
//
// 	return ret;
// }

static int oah0428_vvs_vv_g_fps(struct oah0428 *sensor, void *args)
{
	u32 fps_q10 = sensor->fps_q10;

	return copy_to_user(args, &fps_q10, sizeof(fps_q10)) ? -EIO : 0;
}

static int oah0428_vvs_vv_s_fps(struct oah0428 *sensor, void *args)
{
	u32 fps_q10;

	if (copy_from_user(&fps_q10, args, sizeof(fps_q10)))
		return -EIO;

	return fps_q10 == sensor->fps_q10 ? 0 : -EINVAL;
}

static int oah0428_vvs_vv_read_reg(struct oah0428 *sensor, void *args)
{
	struct vvcam_sccb_data_s reg;
	u8 val;
	int ret;

	if (copy_from_user(&reg, args, sizeof(reg)))
		return -EIO;

	ret = oah0428_vvs_read_reg(sensor, reg.addr, &val);
	if (ret)
		return ret;

	reg.data = val;
	return copy_to_user(args, &reg, sizeof(reg)) ? -EIO : 0;
}

static int oah0428_vvs_vv_write_reg(struct oah0428 *sensor, void *args)
{
	struct vvcam_sccb_data_s reg;

	if (copy_from_user(&reg, args, sizeof(reg)))
		return -EIO;

	return oah0428_vvs_write_reg(sensor, reg.addr, reg.data & 0xff);
}

static int oah0428_vvs_vv_get_clk(struct oah0428 *sensor, void *args)
{
	struct vvcam_clk_s clk = {
		.status = sensor->on,
		.sensor_mclk = sensor->xclk_freq,
		.csi_max_pixel_clk = OAH0428_VVS_CSI_MAX_PIXEL_CLK,
	};

	return copy_to_user(args, &clk, sizeof(clk)) ? -EIO : 0;
}

static int oah0428_vvs_vv_get_chip_id(struct oah0428 *sensor, void *args)
{
	u32 chip_id;
	int ret;

	mutex_lock(&sensor->lock);
	ret = oah0428_vvs_read_chip_id(sensor, &chip_id);
	mutex_unlock(&sensor->lock);
	if (ret)
		return ret;

	return copy_to_user(args, &chip_id, sizeof(chip_id)) ? -EIO : 0;
}

static long oah0428_vvs_priv_ioctl(struct v4l2_subdev *sd, unsigned int cmd,
				   void *arg)
{
	struct oah0428 *sensor = sd_to_oah0428(sd);
	int ret = 0;

	switch (cmd) {
	case VIDIOC_QUERYCAP:
		ret = oah0428_vvs_vv_querycap(sensor, arg);
		break;
	case VVSENSORIOC_G_CLK:
		ret = oah0428_vvs_vv_get_clk(sensor, arg);
		break;
	case VVSENSORIOC_S_CLK:
		break;
	case VVSENSORIOC_QUERY:
		ret = oah0428_vvs_vv_querymode(sensor, arg);
		break;
	case VVSENSORIOC_G_SENSOR_MODE:
		ret = oah0428_vvs_vv_get_sensormode(sensor, arg);
		break;
	case VVSENSORIOC_S_SENSOR_MODE:
		ret = oah0428_vvs_vv_set_sensormode(sensor, arg);
		break;
	case VVSENSORIOC_G_CHIP_ID:
		ret = oah0428_vvs_vv_get_chip_id(sensor, arg);
		break;
	case VVSENSORIOC_G_RESERVE_ID: {
		u32 reserve_id = 0;

		ret = copy_to_user(arg, &reserve_id, sizeof(reserve_id)) ?
			      -EIO :
			      0;
		break;
	}
	case VVSENSORIOC_S_STREAM:
		ret = oah0428_vvs_vv_s_stream(sensor, arg);
		break;
	// case VVSENSORIOC_S_EXP:
	// 	ret = oah0428_vvs_vv_s_exp(sensor, arg);
	// 	break;
	// case VVSENSORIOC_S_GAIN:
	// case VVSENSORIOC_S_LONG_GAIN:
	// 	ret = oah0428_vvs_vv_s_gain(sensor, arg);
	// 	break;
	case VVSENSORIOC_G_FPS:
		ret = oah0428_vvs_vv_g_fps(sensor, arg);
		break;
	case VVSENSORIOC_S_FPS:
		ret = oah0428_vvs_vv_s_fps(sensor, arg);
		break;
	case VVSENSORIOC_READ_REG:
		ret = oah0428_vvs_vv_read_reg(sensor, arg);
		break;
	case VVSENSORIOC_WRITE_REG:
		ret = oah0428_vvs_vv_write_reg(sensor, arg);
		break;
	case VVSENSORIOC_S_INIT:
		mutex_lock(&sensor->lock);
		// ret = oah0428_vvs_init_mode(sensor,
		// 			    sensor->streamcap.capturemode);
		mutex_unlock(&sensor->lock);
		break;
	case VVSENSORIOC_S_POWER: {
		u32 on;

		if (copy_from_user(&on, arg, sizeof(on)))
			ret = -EIO;
		else
			ret = oah0428_vvs_s_power(sd, on);
		break;
	}
	case VVSENSORIOC_G_POWER: {
		u32 on = sensor->on;

		ret = copy_to_user(arg, &on, sizeof(on)) ? -EIO : 0;
		break;
	}
	case VVSENSORIOC_S_WB:
	case VVSENSORIOC_S_BLC:
	case VVSENSORIOC_RESET:
	case VVSENSORIOC_S_LONG_EXP:
	case VVSENSORIOC_S_VSEXP:
	case VVSENSORIOC_S_VSGAIN:
	case VVSENSORIOC_S_HDR_RADIO:
	case VVSENSORIOC_G_EXPAND_CURVE:
	case VVSENSORIOC_S_TEST_PATTERN:
	case VVSENSORIOC_G_LENS:
		break;
	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

static struct v4l2_subdev_video_ops oah0428_vvs_subdev_video_ops = {
	.g_parm = oah0428_vvs_g_parm,
	.s_parm = oah0428_vvs_s_parm,
	.s_stream = oah0428_vvs_s_stream,
};

static const struct v4l2_subdev_pad_ops oah0428_vvs_subdev_pad_ops = {
	.enum_frame_size = oah0428_vvs_enum_frame_size,
	.enum_frame_interval = oah0428_vvs_enum_frame_interval,
	.enum_mbus_code = oah0428_vvs_enum_mbus_code,
	.set_fmt = oah0428_vvs_set_fmt,
	.get_fmt = oah0428_vvs_get_fmt,
};

static struct v4l2_subdev_core_ops oah0428_vvs_subdev_core_ops = {
	.s_power = oah0428_vvs_s_power,
	.ioctl = oah0428_vvs_priv_ioctl,
};

static struct v4l2_subdev_ops oah0428_vvs_subdev_ops = {
	.core = &oah0428_vvs_subdev_core_ops,
	.video = &oah0428_vvs_subdev_video_ops,
	.pad = &oah0428_vvs_subdev_pad_ops,
};

static int oah0428_vvs_link_setup(struct media_entity *entity,
				  const struct media_pad *local,
				  const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct media_entity_operations oah0428_vvs_media_ops = {
	.link_setup = oah0428_vvs_link_setup,
};

/*!
 * oah0428 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int oah0428_vvs_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct oah0428 *sensor;
	struct v4l2_mbus_framefmt *fmt;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(sensor->sensor_clk)) {
		/* assuming clock enabled by default */
		ret = PTR_ERR(sensor->sensor_clk);
		sensor->sensor_clk = NULL;
		dev_err(dev, "clock-frequency missing or invalid\n");
		return ret;
	}

	ret = clk_prepare_enable(sensor->sensor_clk);
	if (ret)
		return ret;
	sensor->xclk_freq = clk_get_rate(sensor->sensor_clk);
	if (!sensor->xclk_freq)
		sensor->xclk_freq = OAH0428_VVS_DEFAULT_MCLK;

	endpoint =
		fwnode_graph_get_next_endpoint(dev_fwnode(&client->dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		ret = -EINVAL;
		goto err_clk;
	}

	ret = v4l2_fwnode_endpoint_parse(endpoint, &sensor->ep);
	fwnode_handle_put(endpoint);
	if (ret) {
		dev_err(dev, "Could not parse endpoint\n");
		goto err_clk;
	}

	dev_info(dev,
		 "probe: endpoint parsed: bus_type=%u, num_data_lanes=%u\n",
		 sensor->ep.bus_type, sensor->ep.bus.mipi_csi2.num_data_lanes);
	if (sensor->ep.bus.mipi_csi2.num_data_lanes > 0) {
		int i;

		for (i = 0; i < sensor->ep.bus.mipi_csi2.num_data_lanes; i++)
			dev_info(dev, "probe:   data lane[%d] = %u\n", i,
				 sensor->ep.bus.mipi_csi2.data_lanes[i]);
		oah0428_vvs_modes[0].mipi_info.mipi_lane =
			sensor->ep.bus.mipi_csi2.num_data_lanes;
		oah0428_vvs_modes[1].mipi_info.mipi_lane =
			sensor->ep.bus.mipi_csi2.num_data_lanes;
	}

	dev_info(dev, "probe: MCLK rate = %lu Hz\n",
		 sensor->sensor_clk ? clk_get_rate(sensor->sensor_clk) : 0);

	sensor->i2c_client = client;
	sensor->regmap =
		devm_regmap_init_i2c(client, &oah0428_vvs_regmap_config);
	if (IS_ERR(sensor->regmap)) {
		ret = PTR_ERR(sensor->regmap);
		dev_err(dev, "Failed to initialize regmap: %d\n", ret);
		goto err_clk;
	}

	/*
	 * default init sequence initialize sensor to
	 * YUV422 UYVY VGA@30fps
	 */
	fmt = &sensor->fmt;
	fmt->code = oah0428_vvs_colour_fmts[0].code;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
	fmt->width = 400;
	fmt->height = 400;
	fmt->field = V4L2_FIELD_NONE;

	sensor->datafmt = &oah0428_vvs_colour_fmts[0];
	sensor->cur_mode = oah0428_vvs_mode_1_1_400;
	sensor->vvcam_mode = oah0428_vvs_modes[sensor->cur_mode];
	sensor->fps_q10 = OAH0428_VVS_DEFAULT_FPS_Q10;
	sensor->exposure_lines = OAH0428_VVS_400_MAX_EXPOSURE_LINES;
	sensor->gain_q8 = OAH0428_VVS_DEFAULT_GAIN_Q8;
	sensor->streamcap.capability = V4L2_MODE_HIGHQUALITY |
				       V4L2_CAP_TIMEPERFRAME;
	sensor->streamcap.capturemode = oah0428_vvs_mode_1_1_400;
	sensor->streamcap.timeperframe.denominator = DEFAULT_FPS;
	sensor->streamcap.timeperframe.numerator = 1;

	ret = init_device(sensor);
	if (ret < 0) {
		clk_disable_unprepare(sensor->sensor_clk);
		dev_warn(dev, "Camera init failed\n");
		return ret;
	}

	v4l2_i2c_subdev_init(&sensor->sd, client, &oah0428_vvs_subdev_ops);

	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.ops = &oah0428_vvs_media_ops;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	mutex_init(&sensor->lock);

	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		goto out_media;

	ret = v4l2_async_register_subdev_sensor(&sensor->sd);
	if (ret) {
		dev_err(&client->dev, "Async register failed, ret=%d\n", ret);
		goto out_media;
	}

	dev_info(dev, "Camera is found\n");
	return ret;

out_media:
	media_entity_cleanup(&sensor->sd.entity);
	mutex_destroy(&sensor->lock);
err_clk:
	clk_disable_unprepare(sensor->sensor_clk);
	return ret;
}

/*!
 * oah0428 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static void oah0428_vvs_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct oah0428 *sensor = to_oah0428(client);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sensor->sd.entity);
	clk_disable_unprepare(sensor->sensor_clk);
	//v4l2_ctrl_handler_free(&sensor->ctrls.handler);
	mutex_destroy(&sensor->lock);
}

static const struct i2c_device_id oah0428_vvs_id[] = {
	{ "oah0428-vvs", 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, oah0428_vvs_id);

static const struct of_device_id oah0428_vvs_dt_ids[] = {
	{ .compatible = "btech,oah0428-vvs" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, oah0428_vvs_dt_ids);

static struct i2c_driver oah0428_vvs_i2c_driver = {
	.driver = {
		  .name  = "oah0428-vvs",
		  .of_match_table = oah0428_vvs_dt_ids,
		  },
	.id_table = oah0428_vvs_id,
	.probe  = oah0428_vvs_probe,
	.remove = oah0428_vvs_remove,
};

module_i2c_driver(oah0428_vvs_i2c_driver);

MODULE_AUTHOR("Bruetsch Technology AG.");
MODULE_DESCRIPTION("oah0428 Camera Subdev Driver ISP");
MODULE_LICENSE("GPL");
