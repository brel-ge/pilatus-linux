/*
 * Copyright (C) 2011-2016 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * Copyright 2018-2019 NXP
 * Copyright 2022 Bruetsch Elektronik AG
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
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#define OCHSA10_VOLTAGE_ANALOG 2800000
#define OCHSA10_VOLTAGE_DIGITAL_CORE 1500000
#define OCHSA10_VOLTAGE_DIGITAL_IO 1800000

#define MIN_FPS 30
#define MAX_FPS 30
#define DEFAULT_FPS 30

#define OCHSA10_XCLK_MIN 6000000
#define OCHSA10_XCLK_MAX 24000000

#define OCHSA10_CHIP_ID_HIGH_BYTE 0x300A
#define OCHSA10_CHIP_ID_MID_BYTE 0x300B
#define OCHSA10_CHIP_ID_LOW_BYTE 0x300C

#define REG_SC_CTRL_MODE 0x0100
#define SC_CTRL_MODE_STANDBY 0x0
#define SC_CTRL_MODE_STREAMING BIT(0)

enum ochsa10_mode {
	ochsa10_mode_MIN = 0,
	ochsa10_mode_800_800 = 0,
	ochsa10_mode_MAX = 0,
	ochsa10_mode_INIT = 0xff, /*only for sensor init*/
};

enum ochsa10_frame_rate { ochsa10_30_fps };

static int ochsa10_framerates[] = {
	[ochsa10_30_fps] = 30,
};

struct ochsa10_datafmt {
	u32 code;
	enum v4l2_colorspace colorspace;
};

struct reg_value {
	u16 u16RegAddr;
	u8 u8Val;
	u8 u8Mask;
	u32 u32Delay_ms;
};

struct ochsa10_mode_info {
	enum ochsa10_mode mode;
	u32 width;
	u32 height;
	struct reg_value *init_data_ptr;
	u32 init_data_size;
};

struct ochsa10 {
	struct v4l2_subdev subdev;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	const struct ochsa10_datafmt *fmt;
	struct v4l2_captureparm streamcap;
	bool on;

	/* control settings */
	int brightness;
	int hue;
	int contrast;
	int saturation;
	int red;
	int green;
	int blue;
	int ae_mode;

	u32 mclk;
	u8 mclk_source;
	struct clk *sensor_clk;

	void (*io_init)(struct ochsa10 *);
	int pwn_gpio, rst_gpio;
};

struct ochsa10_res {
	int width;
	int height;
};

/*!
 * Maintains the information on the current state of the sesor.
 */
static s32 ochsa10_read_reg(struct ochsa10 *sensor, u16 reg, u8 *val);
static s32 ochsa10_write_reg(struct ochsa10 *sensor, u16 reg, u8 val);

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ochsa10_get_register(struct v4l2_subdev *sd,
				struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ochsa10 *sensor = to_ochsa10(client);
	int ret;
	u8 val;

	if (reg->reg & ~0xffff)
		return -EINVAL;

	reg->size = 1;

	ret = ochsa10_read_reg(sensor, reg->reg, &val);
	if (!ret)
		reg->val = (__u64)val;

	return ret;
}

static int ochsa10_set_register(struct v4l2_subdev *sd,
				const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ochsa10 *sensor = to_ochsa10(client);
	if (reg->reg & ~0xffff || reg->val & ~0xff)
		return -EINVAL;

	return ochsa10_write_reg(sensor, reg->reg, reg->val);
}
#endif

struct ochsa10_res ochsa10_valid_res[] = {
	[0] = { 800, 800 },
};

static struct reg_value ochsa10_setting_init[] = {
	{ 0x0100, 0x00, 0 , 0},
	{ 0x0103, 0x01, 0 , 0},
	// PLL
	{ 0x0302, 0x00, 0 , 0},
	{ 0x0303, 0x06, 0 , 0},
	{ 0x0304, 0x01, 0 , 0},
	{ 0x0305, 0x90, 0 , 0},
	//{ 0x0306, 0x00, 0 , 0},
	{ 0x0306, 0x01, 0 , 0},
	{ 0x0308, 0x01, 0 , 0},
	//{ 0x0309, 0x00, 0 , 0},
	{ 0x0309, 0x00, 0 , 0},
	{ 0x030c, 0x01, 0 , 0},

	// PLL2
	{ 0x0322, 0x01, 0 , 0},
	{ 0x0323, 0x06, 0 , 0},
	{ 0x0324, 0x01, 0 , 0},
	{ 0x0325, 0x68, 0 , 0},

	{ 0x3002, 0xa1, 0 , 0},
	{ 0x301e, 0xf0, 0 , 0},
	{ 0x3022, 0x41, 0 , 0}, // 8-bit mode
	// { 0x3501, 0x03, 0 , 0},
	
	{ 0x3501, 0x03, 0 , 0},
	{ 0x3502, 0x00, 0 , 0},

	{ 0x3504, 0x0c, 0 , 0},
	{ 0x3508, 0x06, 0 , 0},
	{ 0x3509, 0x00, 0 , 0},

	{ 0x3505, 0x01, 0 , 0}, // Manual exposure

	{ 0x3601, 0xc0, 0 , 0},
	{ 0x3603, 0x71, 0 , 0},
	{ 0x3610, 0x68, 0 , 0},
	{ 0x3611, 0x86, 0 , 0},
	{ 0x3640, 0x10, 0 , 0},
	{ 0x3641, 0x80, 0 , 0},
	{ 0x3642, 0xdc, 0 , 0},
	{ 0x3646, 0x55, 0 , 0},
	{ 0x3647, 0x57, 0 , 0},
	{ 0x364b, 0x00, 0 , 0},
	{ 0x3653, 0x10, 0 , 0},
	{ 0x3655, 0x00, 0 , 0},
	{ 0x3656, 0x00, 0 , 0},
	{ 0x365f, 0x0f, 0 , 0},
	{ 0x3661, 0x45, 0 , 0},
	{ 0x3662, 0x24, 0 , 0},
	{ 0x3663, 0x11, 0 , 0},
	{ 0x3664, 0x07, 0 , 0},
	{ 0x3709, 0x34, 0 , 0},
	{ 0x370b, 0x6f, 0 , 0},
	{ 0x3714, 0x22, 0 , 0},
	{ 0x371b, 0x27, 0 , 0},
	{ 0x371c, 0x67, 0 , 0},
	{ 0x371d, 0xa7, 0 , 0},
	{ 0x371e, 0xe7, 0 , 0},
	{ 0x3730, 0x81, 0 , 0},
	{ 0x3733, 0x10, 0 , 0},
	{ 0x3734, 0x40, 0 , 0},
	{ 0x3737, 0x04, 0 , 0},
	{ 0x3739, 0x1c, 0 , 0},
	{ 0x3767, 0x00, 0 , 0},
	{ 0x376c, 0x81, 0 , 0},
	{ 0x3772, 0x14, 0 , 0},
	{ 0x37c2, 0x04, 0 , 0},
	{ 0x37d8, 0x03, 0 , 0},
	{ 0x37d9, 0x0c, 0 , 0},
	{ 0x37e0, 0x00, 0 , 0},
	{ 0x37e1, 0x08, 0 , 0},
	{ 0x37e2, 0x10, 0 , 0},
	{ 0x37e3, 0x04, 0 , 0},
	{ 0x37e4, 0x04, 0 , 0},
	{ 0x37e5, 0x03, 0 , 0},
	{ 0x37e6, 0x04, 0 , 0},
	{ 0x3800, 0x00, 0 , 0},
	{ 0x3801, 0x00, 0 , 0},
	{ 0x3802, 0x00, 0 , 0},
	{ 0x3803, 0x00, 0 , 0},
	// { 0x3804, 0x03, 0 , 0}, // x addr end
	// { 0x3805, 0x27, 0 , 0},
	{ 0x3804, 0x05, 0 , 0}, // x addr end
	{ 0x3805, 0x0f, 0 , 0},
	{ 0x3806, 0x03, 0 , 0}, // y addr end
	{ 0x3807, 0x2f, 0 , 0},
	
	//{ 0x3808, 0x05, 0 , 0}, // 0x500 = 1280
	//{ 0x3809, 0x00, 0 , 0},
	{ 0x3808, 0x03, 0 , 0}, // 0x320 = 800
	{ 0x3809, 0x20, 0 , 0},

	{ 0x380a, 0x03, 0 , 0},
	{ 0x380b, 0x20, 0 , 0},

	{ 0x380c, 0x03, 0 , 0}, // HTS 896
	{ 0x380d, 0x80, 0 , 0},
	{ 0x380e, 0x03, 0 , 0}, // VTS 896
	{ 0x380f, 0x80, 0 , 0},

	{ 0x3810, 0x00, 0 , 0},
	{ 0x3811, 0x09, 0 , 0},
	{ 0x3812, 0x00, 0 , 0},
	{ 0x3813, 0x08, 0 , 0},
	{ 0x3814, 0x01, 0 , 0},
	{ 0x3815, 0x01, 0 , 0},
	{ 0x3816, 0x01, 0 , 0},
	{ 0x3817, 0x01, 0 , 0},
	{ 0x3820, 0xa8, 0 , 0},
	{ 0x3822, 0x13, 0 , 0},
	{ 0x3832, 0x28, 0 , 0},
	{ 0x3833, 0x10, 0 , 0},
	{ 0x3b00, 0x00, 0 , 0},
	{ 0x3c80, 0x00, 0 , 0},
	{ 0x3c88, 0x02, 0 , 0},
	{ 0x3c8c, 0x07, 0 , 0},
	{ 0x3c8d, 0x40, 0 , 0},
	{ 0x3cc7, 0x80, 0 , 0},
	{ 0x4000, 0xc3, 0 , 0},
	{ 0x4001, 0xe0, 0 , 0},
	{ 0x4003, 0x40, 0 , 0},
	{ 0x4008, 0x02, 0 , 0},
	{ 0x4009, 0x19, 0 , 0},
	{ 0x400a, 0x01, 0 , 0},
	{ 0x400b, 0x6c, 0 , 0},
	{ 0x4011, 0x00, 0 , 0},
	{ 0x4041, 0x00, 0 , 0},
	{ 0x4300, 0xff, 0 , 0},
	{ 0x4301, 0x00, 0 , 0},
	{ 0x4302, 0x0f, 0 , 0},
	{ 0x4503, 0x00, 0 , 0},
	{ 0x4601, 0x50, 0 , 0},
	{ 0x4800, 0x64, 0 , 0},
	{ 0x481f, 0x34, 0 , 0},
	{ 0x4825, 0x33, 0 , 0},
	{ 0x4837, 0x14, 0 , 0},
	{ 0x4881, 0x40, 0 , 0},
	{ 0x4883, 0x01, 0 , 0},
	{ 0x4890, 0x00, 0 , 0},
	{ 0x4901, 0x00, 0 , 0},
	{ 0x4902, 0x00, 0 , 0},
	{ 0x4b00, 0x2a, 0 , 0},
	{ 0x4b0d, 0x00, 0 , 0},
	{ 0x450a, 0x04, 0 , 0},
	{ 0x450b, 0x00, 0 , 0},
	{ 0x5000, 0x75, 0 , 0},
	{ 0x5004, 0x00, 0 , 0},
	{ 0x5080, 0x40, 0 , 0},
	{ 0x4800, 0x64, 0 , 0},
	{ 0x3808, 0x03, 0 , 0},
	{ 0x3809, 0x20, 0 , 0},
	{ 0x380a, 0x03, 0 , 0},
	{ 0x380b, 0x20, 0 , 0},
	{ 0x3810, 0x00, 0 , 0},
	{ 0x3811, 0xf9, 0 , 0},
	{ 0x3812, 0x00, 0 , 0},
	{ 0x3813, 0x08, 0 , 0},
	{ 0x4837, 0x10, 0 , 0},
};

static struct reg_value ochsa10_setting_uyvy[] = {
};

static struct ochsa10_mode_info ochsa10_mode_info_data[ochsa10_mode_MAX + 1] = {
	{ ochsa10_mode_800_800, 640, 480, ochsa10_setting_uyvy,
	  ARRAY_SIZE(ochsa10_setting_uyvy) },
};

static struct regulator *io_regulator;
static struct regulator *core_regulator;
static struct regulator *analog_regulator;
static struct regulator *gpo_regulator;
static DEFINE_MUTEX(ochsa10_mutex);

static int ochsa10_probe(struct i2c_client *adapter,
			 const struct i2c_device_id *device_id);
static int ochsa10_remove(struct i2c_client *client);

#ifdef CONFIG_OF
static const struct of_device_id ochsa10_mipi_v2_of_match[] = {
	{
		.compatible = "ovti,ochsa10_mipi",
	},
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, ochsa10_mipi_v2_of_match);
#endif

static const struct i2c_device_id ochsa10_id[] = {
	{ "ochsa10_mipi", 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, ochsa10_id);

static struct i2c_driver ochsa10_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "ochsa10_mipi",
#ifdef CONFIG_OF
		  .of_match_table = of_match_ptr(ochsa10_mipi_v2_of_match),
#endif
		  },
	.probe  = ochsa10_probe,
	.remove = ochsa10_remove,
	.id_table = ochsa10_id,
};

static const struct ochsa10_datafmt ochsa10_colour_fmts[] = {
	{ MEDIA_BUS_FMT_SBGGR8_1X8, V4L2_COLORSPACE_SRGB },
};

static int get_capturemode(int width, int height)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ochsa10_valid_res); i++) {
		if ((ochsa10_valid_res[i].width == width) &&
		    (ochsa10_valid_res[i].height == height))
			return i;
	}

	return -1;
}

static struct ochsa10 *to_ochsa10(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ochsa10, subdev);
}

/* Find a data format by a pixel code in an array */
static const struct ochsa10_datafmt *ochsa10_find_datafmt(u32 code)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(ochsa10_colour_fmts); i++)
		if (ochsa10_colour_fmts[i].code == code)
			return ochsa10_colour_fmts + i;

	return NULL;
}

static inline void ochsa10_power_down(struct ochsa10 *sensor, int enable)
{
	if (sensor->pwn_gpio < 0)
		return;

	if (!enable)
		gpio_set_value_cansleep(sensor->pwn_gpio, 0);
	else
		gpio_set_value_cansleep(sensor->pwn_gpio, 1);

	msleep(2);
}

static void ochsa10_reset(struct ochsa10 *sensor)
{
	if (sensor->rst_gpio < 0 || sensor->pwn_gpio < 0)
		return;

	/* camera reset */
	gpio_set_value_cansleep(sensor->rst_gpio, 1);

	/* camera power dowmn */
	gpio_set_value_cansleep(sensor->pwn_gpio, 1);
	msleep(5);

	gpio_set_value_cansleep(sensor->rst_gpio, 0);
	msleep(1);

	gpio_set_value_cansleep(sensor->pwn_gpio, 0);
	msleep(5);

	gpio_set_value_cansleep(sensor->rst_gpio, 1);
	msleep(5);
}

static int ochsa10_regulator_enable(struct device *dev)
{
	int ret = 0;

	io_regulator = devm_regulator_get(dev, "DOVDD");
	if (!IS_ERR(io_regulator)) {
		regulator_set_voltage(io_regulator, OCHSA10_VOLTAGE_DIGITAL_IO,
				      OCHSA10_VOLTAGE_DIGITAL_IO);
		ret = regulator_enable(io_regulator);
		if (ret) {
			pr_err("%s:io set voltage error\n", __func__);
			return ret;
		} else {
			dev_dbg(dev, "%s:io set voltage ok\n", __func__);
		}
	} else {
		pr_err("%s: cannot get io voltage error\n", __func__);
		io_regulator = NULL;
	}

	core_regulator = devm_regulator_get(dev, "DVDD");
	if (!IS_ERR(core_regulator)) {
		regulator_set_voltage(core_regulator,
				      OCHSA10_VOLTAGE_DIGITAL_CORE,
				      OCHSA10_VOLTAGE_DIGITAL_CORE);
		ret = regulator_enable(core_regulator);
		if (ret) {
			pr_err("%s:core set voltage error\n", __func__);
			return ret;
		} else {
			dev_dbg(dev, "%s:core set voltage ok\n", __func__);
		}
	} else {
		core_regulator = NULL;
		pr_err("%s: cannot get core voltage error\n", __func__);
	}

	analog_regulator = devm_regulator_get(dev, "AVDD");
	if (!IS_ERR(analog_regulator)) {
		regulator_set_voltage(analog_regulator, OCHSA10_VOLTAGE_ANALOG,
				      OCHSA10_VOLTAGE_ANALOG);
		ret = regulator_enable(analog_regulator);
		if (ret) {
			pr_err("%s:analog set voltage error\n", __func__);
			return ret;
		} else {
			dev_dbg(dev, "%s:analog set voltage ok\n", __func__);
		}
	} else {
		analog_regulator = NULL;
		pr_err("%s: cannot get analog voltage error\n", __func__);
	}

	return ret;
}

static void ochsa10_regualtor_disable(void)
{
	if (analog_regulator)
		regulator_disable(analog_regulator);

	if (core_regulator)
		regulator_disable(core_regulator);

	if (io_regulator)
		regulator_disable(io_regulator);

	if (gpo_regulator)
		regulator_disable(gpo_regulator);
}

static s32 ochsa10_write_reg(struct ochsa10 *sensor, u16 reg, u8 val)
{
	struct device *dev = &sensor->i2c_client->dev;
	u8 au8Buf[3] = { 0 };

	au8Buf[0] = reg >> 8;
	au8Buf[1] = reg & 0xff;
	au8Buf[2] = val;

	if (i2c_master_send(sensor->i2c_client, au8Buf, 3) < 0) {
		dev_err(dev, "Write reg error: reg=%x, val=%x\n", reg, val);
		return -1;
	}

	return 0;
}

static s32 ochsa10_read_reg(struct ochsa10 *sensor, u16 reg, u8 *val)
{
	struct device *dev = &sensor->i2c_client->dev;
	u8 au8RegBuf[2] = { 0 };
	u8 u8RdVal = 0;

	au8RegBuf[0] = reg >> 8;
	au8RegBuf[1] = reg & 0xff;

	if (i2c_master_send(sensor->i2c_client, au8RegBuf, 2) != 2) {
		dev_err(dev, "Read reg error: reg=%x\n", reg);
		return -1;
	}

	if (i2c_master_recv(sensor->i2c_client, &u8RdVal, 1) != 1) {
		dev_err(dev, "Read reg error: reg=%x, val=%x\n", reg, u8RdVal);
		return -1;
	}

	*val = u8RdVal;

	return u8RdVal;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ochsa10_get_register(struct v4l2_subdev *sd,
				struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ochsa10 *sensor = to_ochsa10(client);
	int ret;
	u8 val;

	if (reg->reg & ~0xffff)
		return -EINVAL;

	reg->size = 1;

	ret = ochsa10_read_reg(sensor, reg->reg, &val);
	if (!ret)
		reg->val = (__u64)val;

	return ret;
}

static int ochsa10_set_register(struct v4l2_subdev *sd,
				const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ochsa10 *sensor = to_ochsa10(client);

	if (reg->reg & ~0xffff || reg->val & ~0xff)
		return -EINVAL;

	return ochsa10_write_reg(sensor, reg->reg, reg->val);

}
#endif

static void OCHSA10_stream_on(struct ochsa10 *sensor)
{
	ochsa10_write_reg(sensor, REG_SC_CTRL_MODE, SC_CTRL_MODE_STREAMING);
	ochsa10_write_reg(sensor, REG_SC_CTRL_MODE, SC_CTRL_MODE_STREAMING);
	ochsa10_write_reg(sensor, REG_SC_CTRL_MODE, SC_CTRL_MODE_STREAMING);
	ochsa10_write_reg(sensor, REG_SC_CTRL_MODE, SC_CTRL_MODE_STREAMING);
	ochsa10_write_reg(sensor, REG_SC_CTRL_MODE, SC_CTRL_MODE_STREAMING);
}

static void OCHSA10_stream_off(struct ochsa10 *sensor)
{
	ochsa10_write_reg(sensor, REG_SC_CTRL_MODE, SC_CTRL_MODE_STANDBY);
}

/* download ochsa10 settings to sensor through i2c */
static int ochsa10_download_firmware(struct ochsa10 *sensor,
				     struct reg_value *pModeSetting,
				     s32 ArySize)
{
	u32 Delay_ms = 0;
	u16 RegAddr = 0;
	u8 Mask = 0;
	u8 Val = 0;
	u8 RegVal = 0;
	int i, retval = 0;

	for (i = 0; i < ArySize; ++i, ++pModeSetting) {
		Delay_ms = pModeSetting->u32Delay_ms;
		RegAddr = pModeSetting->u16RegAddr;
		Val = pModeSetting->u8Val;
		Mask = pModeSetting->u8Mask;

		if (Mask) {
			retval = ochsa10_read_reg(sensor, RegAddr, &RegVal);
			if (retval < 0) {
				pr_err("Error reading OCHSA10 register\n");
				goto err;
			}

			RegVal &= ~(u8)Mask;
			Val &= Mask;
			Val |= RegVal;
		}

		retval = ochsa10_write_reg(sensor, RegAddr, Val);
		if (retval < 0)
			goto err;

		if (Delay_ms)
			msleep(Delay_ms);
	}
err:
	return retval;
}

/* if sensor changes inside scaling or subsampling
 * change mode directly
 * */
static int ochsa10_change_mode_direct(struct ochsa10 *sensor,
				      enum ochsa10_mode mode)
{
	struct reg_value *pModeSetting = NULL;
	s32 ArySize = 0;
	int retval = 0;

	/* check if the input mode and frame rate is valid */
	pModeSetting = ochsa10_mode_info_data[mode].init_data_ptr;
	ArySize = ochsa10_mode_info_data[mode].init_data_size;

	sensor->pix.width = ochsa10_mode_info_data[mode].width;
	sensor->pix.height = ochsa10_mode_info_data[mode].height;

	if (sensor->pix.width == 0 || sensor->pix.height == 0 ||
	    pModeSetting == NULL || ArySize == 0)
		return -EINVAL;

	OCHSA10_stream_off(sensor);

	/* Write capture setting */
	retval = ochsa10_download_firmware(sensor, pModeSetting, ArySize);
	if (retval < 0)
		goto err;

err:
	return retval;
}

static int ochsa10_init_mode(struct ochsa10 *sensor, enum ochsa10_mode mode,
			     enum ochsa10_mode orig_mode)
{
	struct device *dev = &sensor->i2c_client->dev;
	struct reg_value *pModeSetting = NULL;
	s32 ArySize = 0;
	int retval = 0;
	u32 msec_wait4stable = 0;

	if ((mode > ochsa10_mode_MAX || mode < ochsa10_mode_MIN) &&
	    (mode != ochsa10_mode_INIT)) {
		dev_err(dev, "Wrong ochsa10 mode detected!\n");
		return -1;
	}

	if (mode == ochsa10_mode_INIT) {
		pModeSetting = ochsa10_setting_init;
		ArySize = ARRAY_SIZE(ochsa10_setting_init);

		sensor->pix.width = ochsa10_valid_res[0].width;
		sensor->pix.height = ochsa10_valid_res[0].height;
		retval = ochsa10_download_firmware(sensor, pModeSetting,
						   ArySize);
		if (retval < 0)
			goto err;

		pModeSetting = ochsa10_setting_uyvy;
		ArySize = ARRAY_SIZE(ochsa10_setting_uyvy);
		retval = ochsa10_download_firmware(sensor, pModeSetting,
						   ArySize);
	} else {
		retval = ochsa10_change_mode_direct(sensor, mode);
	}

	if (retval < 0)
		goto err;

	/* dump the first eighteen frames: 1/30*18 */
	msec_wait4stable = 600;
	msleep(msec_wait4stable);

err:
	return retval;
}

/*!
 * ochsa10_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int ochsa10_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ochsa10 *sensor = to_ochsa10(client);

	if (on && !sensor->on) {
		if (io_regulator)
			if (regulator_enable(io_regulator) != 0)
				return -EIO;
		if (core_regulator)
			if (regulator_enable(core_regulator) != 0)
				return -EIO;
		if (gpo_regulator)
			if (regulator_enable(gpo_regulator) != 0)
				return -EIO;
		if (analog_regulator)
			if (regulator_enable(analog_regulator) != 0)
				return -EIO;
	} else if (!on && sensor->on) {
		ochsa10_regualtor_disable();
	}

	sensor->on = on;

	return 0;
}

/*!
 * ochsa10_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 sub device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ochsa10_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ochsa10 *sensor = to_ochsa10(client);
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
 * ochsa10_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 sub device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ochsa10_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ochsa10 *sensor = to_ochsa10(client);
	struct device *dev = &sensor->i2c_client->dev;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 tgt_fps; /* target frames per secound */
	enum ochsa10_frame_rate frame_rate;
	enum ochsa10_mode orig_mode;
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

		if (tgt_fps == 30)
			frame_rate = ochsa10_30_fps;
		else {
			dev_warn(dev,
				 "The camera frame rate is not supported!\n");
			return -EINVAL;
		}

		orig_mode = sensor->streamcap.capturemode;
		ret = ochsa10_init_mode(
			sensor, (u32)a->parm.capture.capturemode, orig_mode);
		if (ret < 0)
			return ret;

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

static int ochsa10_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	const struct ochsa10_datafmt *fmt = ochsa10_find_datafmt(mf->code);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ochsa10 *sensor = to_ochsa10(client);
	int capturemode;

	if (!fmt) {
		mf->code = ochsa10_colour_fmts[0].code;
		mf->colorspace = ochsa10_colour_fmts[0].colorspace;
		fmt = &ochsa10_colour_fmts[0];
	}

	mf->field = V4L2_FIELD_NONE;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	sensor->fmt = fmt;

	capturemode = get_capturemode(mf->width, mf->height);
	if (capturemode >= 0) {
		sensor->streamcap.capturemode = capturemode;
		sensor->pix.width = mf->width;
		sensor->pix.height = mf->height;
		return 0;
	}
 
	dev_err(&client->dev, "Set format failed %d, %d\n", fmt->code,
		fmt->colorspace);
	return -EINVAL;
}

static int ochsa10_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ochsa10 *sensor = to_ochsa10(client);
	const struct ochsa10_datafmt *fmt = sensor->fmt;

	if (format->pad)
		return -EINVAL;

	mf->code = fmt->code;
	mf->colorspace = fmt->colorspace;
	mf->field = V4L2_FIELD_NONE;

	mf->width = sensor->pix.width;
	mf->height = sensor->pix.height;

	return 0;
}

static int ochsa10_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= ARRAY_SIZE(ochsa10_colour_fmts))
		return -EINVAL;

	code->code = ochsa10_colour_fmts[code->index].code;
	return 0;
}

/*!
 * ochsa10_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ochsa10_enum_framesizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > ochsa10_mode_MAX)
		return -EINVAL;
	fse->code = fse->max_width = ochsa10_valid_res[0].width;
	fse->min_width = ochsa10_valid_res[0].width;
	fse->max_height = ochsa10_valid_res[0].height;
	fse->min_height = ochsa10_valid_res[0].height;
	return 0;
}

/*!
 * ochsa10_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int
ochsa10_enum_frameintervals(struct v4l2_subdev *sd,
			    struct v4l2_subdev_pad_config *cfg,
			    struct v4l2_subdev_frame_interval_enum *fie)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *dev = &client->dev;
	int j, count = 0;

	if (fie->index < 0 || fie->index > ochsa10_mode_MAX)
		return -EINVAL;

	if (fie->width == 0 || fie->height == 0 || fie->code == 0) {
		dev_warn(dev, "Please assign pixel format, width and height\n");
		return -EINVAL;
	}

	fie->interval.numerator = 1;

	count = 0;
	for (j = 0; j < (ochsa10_mode_MAX + 1); j++) {
		if (fie->width == ochsa10_mode_info_data[j].width &&
		    fie->height == ochsa10_mode_info_data[j].height &&
		    ochsa10_mode_info_data[j].init_data_ptr != NULL) {
			count++;
		}
		if (fie->index == (count - 1)) {
			fie->interval.denominator = ochsa10_framerates[0];
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
static int init_device(struct ochsa10 *sensor)
{
	struct device *dev = &sensor->i2c_client->dev;
	u32 tgt_xclk; /* target xclk */
	u32 tgt_fps; /* target frames per secound */
	int ret;

	sensor->on = true;

	/* mclk */
	tgt_xclk = sensor->mclk;
	tgt_xclk = min(tgt_xclk, (u32)OCHSA10_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)OCHSA10_XCLK_MIN);
	sensor->mclk = tgt_xclk;

	dev_dbg(dev, "Setting mclk to %d MHz\n", tgt_xclk / 1000000);

	/* Default camera frame rate is set in probe */
	tgt_fps = sensor->streamcap.timeperframe.denominator /
		  sensor->streamcap.timeperframe.numerator;

	if (tgt_fps != 30)
		return -EINVAL; /* Only 30fps now. */

	ret = ochsa10_init_mode(sensor, ochsa10_mode_INIT, ochsa10_mode_INIT);

	return ret;
}

static int ochsa10_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ochsa10 *sensor = to_ochsa10(client);
	struct device *dev = &sensor->i2c_client->dev;

	dev_info(dev, "s_stream: %d\n", enable);
	if (enable)
		OCHSA10_stream_on(sensor);
	else
		OCHSA10_stream_off(sensor);
	return 0;
}

static struct v4l2_subdev_video_ops ochsa10_subdev_video_ops = {
	.g_parm = ochsa10_g_parm,
	.s_parm = ochsa10_s_parm,
	.s_stream = ochsa10_s_stream,
};

static const struct v4l2_subdev_pad_ops ochsa10_subdev_pad_ops = {
	.enum_frame_size = ochsa10_enum_framesizes,
	.enum_frame_interval = ochsa10_enum_frameintervals,
	.enum_mbus_code = ochsa10_enum_mbus_code,
	.set_fmt = ochsa10_set_fmt,
	.get_fmt = ochsa10_get_fmt,
};

static struct v4l2_subdev_core_ops ochsa10_subdev_core_ops = {
	.s_power = ochsa10_s_power,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ochsa10_get_register,
	.s_register = ochsa10_set_register,
#endif
};

static struct v4l2_subdev_ops ochsa10_subdev_ops = {
	.core = &ochsa10_subdev_core_ops,
	.video = &ochsa10_subdev_video_ops,
	.pad = &ochsa10_subdev_pad_ops,
};

/*!
 * ochsa10 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int ochsa10_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct pinctrl *pinctrl;
	struct device *dev = &client->dev;
	int retval;
	u8 chip_id_high, chip_id_mid, chip_id_low;
	struct ochsa10 *sensor;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	/* ochsa10 pinctrl */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl))
		dev_warn(dev, "No pin available\n");

	/* request power down pin */
	sensor->pwn_gpio = of_get_named_gpio(dev->of_node, "pwn-gpios", 0);
	if (!gpio_is_valid(sensor->pwn_gpio)) {
		if (sensor->pwn_gpio == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_warn(dev, "No sensor pwdn pin available");
	} else {
		retval = devm_gpio_request_one(dev, sensor->pwn_gpio,
					       GPIOF_OUT_INIT_HIGH,
					       "ochsa10_mipi_pwdn");
		if (retval < 0) {
			dev_warn(dev, "Failed to set power pin\n");
			dev_warn(dev, "retval=%d\n", retval);
			return retval;
		}
	}

	/* request reset pin */
	sensor->rst_gpio = of_get_named_gpio(dev->of_node, "rst-gpios", 0);
	if (!gpio_is_valid(sensor->rst_gpio)) {
		if (sensor->rst_gpio == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		dev_warn(dev, "No sensor reset pin available");
	} else {
		retval = devm_gpio_request_one(dev, sensor->rst_gpio,
					       GPIOF_OUT_INIT_HIGH,
					       "ochsa10_mipi_reset");
		if (retval < 0) {
			dev_warn(dev, "Failed to set reset pin\n");
			return retval;
		}
	}

	sensor->sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(sensor->sensor_clk)) {
		/* assuming clock enabled by default */
		sensor->sensor_clk = NULL;
		dev_err(dev, "clock-frequency missing or invalid\n");
		return PTR_ERR(sensor->sensor_clk);
	}

	retval = of_property_read_u32(dev->of_node, "mclk", &(sensor->mclk));
	if (retval) {
		dev_err(dev, "mclk missing or invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "mclk_source",
				      (u32 *)&(sensor->mclk_source));
	if (retval) {
		dev_err(dev, "mclk_source missing or invalid\n");
		return retval;
	}

	clk_prepare_enable(sensor->sensor_clk);

	sensor->io_init = ochsa10_reset;
	sensor->i2c_client = client;
	sensor->pix.pixelformat = V4L2_PIX_FMT_YUYV;
	sensor->pix.width = ochsa10_valid_res[0].width;
	sensor->pix.height = ochsa10_valid_res[0].height;
	sensor->streamcap.capability =
		V4L2_MODE_HIGHQUALITY | V4L2_CAP_TIMEPERFRAME;
	sensor->streamcap.capturemode = 0;
	sensor->streamcap.timeperframe.denominator = DEFAULT_FPS;
	sensor->streamcap.timeperframe.numerator = 1;

	ochsa10_regulator_enable(&client->dev);

	mutex_lock(&ochsa10_mutex);
	{
		ochsa10_reset(sensor);
		ochsa10_power_down(sensor, 0);
	}
	mutex_unlock(&ochsa10_mutex);
	if (retval < 0) {
		clk_disable_unprepare(sensor->sensor_clk);
		return -ENODEV;
	}

	retval = ochsa10_read_reg(sensor, OCHSA10_CHIP_ID_HIGH_BYTE,
				  &chip_id_high);
	dev_warn(dev, "CHIP ID HIGH %d\n", chip_id_high);
	// if (retval < 0 || chip_id_high != 0x76) {
	// 	dev_warn(dev, "Camera is not found\n");
	// 	ochsa10_regualtor_disable();
	// 	clk_disable_unprepare(sensor->sensor_clk);
	// 	return -ENODEV;
	// }
	retval = ochsa10_read_reg(sensor, OCHSA10_CHIP_ID_MID_BYTE,
				  &chip_id_mid);
	dev_warn(dev, "CHIP ID MID %d\n", chip_id_mid);

	retval = ochsa10_read_reg(sensor, OCHSA10_CHIP_ID_LOW_BYTE,
				  &chip_id_low);
	dev_warn(dev, "CHIP ID LOW %d\n", chip_id_low);
	// if (retval < 0 || chip_id_low != 0x95) {
	// 	dev_warn(dev, "Camera is not found\n");
	// 	ochsa10_regualtor_disable();
	// 	clk_disable_unprepare(sensor->sensor_clk);
	// 	return -ENODEV;
	// }

	retval = init_device(sensor);
	if (retval < 0) {
		ochsa10_regualtor_disable();
		clk_disable_unprepare(sensor->sensor_clk);
		dev_warn(dev, "Camera init failed\n");
		ochsa10_power_down(sensor, 1);
		return retval;
	}

	v4l2_i2c_subdev_init(&sensor->subdev, client, &ochsa10_subdev_ops);

	sensor->subdev.grp_id = 678;
	retval = v4l2_async_register_subdev(&sensor->subdev);
	if (retval < 0)
		dev_err(&client->dev, "Async register failed, ret=%d\n",
			retval);

	OCHSA10_stream_off(sensor);
	dev_info(dev, "Camera is found\n");
	return retval;
}

/*!
 * ochsa10 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ochsa10_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ochsa10 *sensor = to_ochsa10(client);

	v4l2_async_unregister_subdev(sd);

	clk_disable_unprepare(sensor->sensor_clk);

	ochsa10_power_down(sensor, 1);

	ochsa10_regualtor_disable();

	return 0;
}

module_i2c_driver(ochsa10_i2c_driver);

MODULE_AUTHOR("Bruetsch Elektronik AG.");
MODULE_DESCRIPTION("OCHSA10 MIPI Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
