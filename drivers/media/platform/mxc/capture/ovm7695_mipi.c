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

#define OVM7695_VOLTAGE_ANALOG 2800000
#define OVM7695_VOLTAGE_DIGITAL_CORE 1500000
#define OVM7695_VOLTAGE_DIGITAL_IO 1800000

#define MIN_FPS 30
#define MAX_FPS 30
#define DEFAULT_FPS 30

#define OVM7695_XCLK_MIN 6000000
#define OVM7695_XCLK_MAX 24000000

#define OVM7695_CHIP_ID_HIGH_BYTE 0x300A
#define OVM7695_CHIP_ID_LOW_BYTE 0x300B

#define REG_SC_CTRL_MODE 0x0100
#define SC_CTRL_MODE_STANDBY 0x0
#define SC_CTRL_MODE_STREAMING BIT(0)

enum ovm7695_mode {
	ovm7695_mode_MIN = 0,
	ovm7695_mode_VGA_640_480 = 0,
	ovm7695_mode_MAX = 0,
	ovm7695_mode_INIT = 0xff, /*only for sensor init*/
};

enum ovm7695_frame_rate { ovm7695_30_fps };

static int ovm7695_framerates[] = {
	[ovm7695_30_fps] = 30,
};

struct ovm7695_datafmt {
	u32 code;
	enum v4l2_colorspace colorspace;
};

struct reg_value {
	u16 u16RegAddr;
	u8 u8Val;
	u8 u8Mask;
	u32 u32Delay_ms;
};

struct ovm7695_mode_info {
	enum ovm7695_mode mode;
	u32 width;
	u32 height;
	struct reg_value *init_data_ptr;
	u32 init_data_size;
};

struct ovm7695 {
	struct v4l2_subdev subdev;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	const struct ovm7695_datafmt *fmt;
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

	void (*io_init)(struct ovm7695 *);
	int pwn_gpio, rst_gpio;
};

struct ovm7695_res {
	int width;
	int height;
};

/*!
 * Maintains the information on the current state of the sesor.
 */
static s32 ovm7695_read_reg(struct ovm7695 *sensor, u16 reg, u8 *val);
static s32 ovm7695_write_reg(struct ovm7695 *sensor, u16 reg, u8 val);

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ovm7695_get_register(struct v4l2_subdev *sd,
				struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ovm7695 *sensor = to_ovm7695(client);
	int ret;
	u8 val;

	if (reg->reg & ~0xffff)
		return -EINVAL;

	reg->size = 1;

	ret = ovm7695_read_reg(sensor, reg->reg, &val);
	if (!ret)
		reg->val = (__u64)val;

	return ret;
}

static int ovm7695_set_register(struct v4l2_subdev *sd,
				const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ovm7695 *sensor = to_ovm7695(client);
	if (reg->reg & ~0xffff || reg->val & ~0xff)
		return -EINVAL;

	return ovm7695_write_reg(sensor, reg->reg, reg->val);
}
#endif

struct ovm7695_res ovm7695_valid_res[] = {
	[0] = { 640, 480 },
};

static struct reg_value ovm7695_setting_init[] = {
	{ 0x0103, 0x01, 0, 0},
	{ 0x3620, 0x2f, 0, 0},
	{ 0x3623, 0x12, 0, 0},
	{ 0x3718, 0x88, 0, 0},
	//42 3703 80
	//42 3712 40
	//42 3706 40
	{ 0x3631, 0x44, 0, 0},
	{ 0x3632, 0x05, 0, 0},
	{ 0x3013, 0xd0, 0, 0},
	{ 0x3705, 0x1d, 0, 0},
	{ 0x3713, 0x0e, 0, 0},
	{ 0x3012, 0x0a, 0, 0},
	{ 0x3717, 0x18, 0, 0},// 19
	{ 0x3621, 0x47, 0, 0},// 44

	{ 0x0309, 0x24, 0, 0},
	{ 0x3820, 0x90, 0, 0},
	{ 0x4803, 0x08, 0, 0},
	{ 0x0101, 0x01, 0, 0},
	{ 0x5100, 0x01, 0, 0},

	{ 0x5301, 0x05, 0, 0},
	{ 0x5302, 0x0c, 0, 0},
	{ 0x5303, 0x1c, 0, 0},
	{ 0x5304, 0x2a, 0, 0},
	{ 0x5305, 0x39, 0, 0},
	{ 0x5306, 0x45, 0, 0},
	{ 0x5307, 0x52, 0, 0},
	{ 0x5308, 0x5d, 0, 0},
	{ 0x5309, 0x68, 0, 0},
	{ 0x530a, 0x7f, 0, 0},
	{ 0x530b, 0x91, 0, 0},
	{ 0x530c, 0xa5, 0, 0},
	{ 0x530d, 0xc6, 0, 0},
	{ 0x530e, 0xde, 0, 0},
	{ 0x530f, 0xef, 0, 0},
	{ 0x5310, 0x16, 0, 0},
	{ 0x520a, 0xf4, 0, 0},
	{ 0x520b, 0xf4, 0, 0},
	{ 0x520c, 0xf4, 0, 0},
	{ 0x5504, 0x08, 0, 0},
	{ 0x5505, 0x48, 0, 0},
	{ 0x5506, 0x07, 0, 0},
	{ 0x5507, 0x0b, 0, 0},
	{ 0x3a18, 0x01, 0, 0},
	{ 0x3a19, 0x00, 0, 0},
	{ 0x3503, 0x03, 0, 0},
	{ 0x3500, 0x00, 0, 0},
	{ 0x3501, 0x21, 0, 0},
	{ 0x3502, 0x00, 0, 0},
	{ 0x350a, 0x00, 0, 0},
	{ 0x350b, 0x00, 0, 0},
	{ 0x4008, 0x02, 0, 0},
	{ 0x4009, 0x09, 0, 0},
	{ 0x3002, 0x09, 0, 0},
	{ 0x3024, 0x00, 0, 0},
	{ 0x3503, 0x00, 0, 0},

	//@@ OV7695_ISP

	{ 0x0101, 0x01, 0, 0}, //mirror_on

	{ 0x5002, 0x48, 0, 0}, //[7:6] Y source select// [3]LENC bias plus
	{ 0x5910, 0x00, 0, 0}, //Y formula
	{ 0x3a0f, 0x58, 0, 0}, 
	{ 0x3a10, 0x50, 0, 0}, //38 //AEC target 
	{ 0x3a1b, 0x5a, 0, 0}, //40 
	{ 0x3a1e, 0x4e, 0, 0}, //36 
	{ 0x3a11, 0xa0, 0, 0}, //80 
	{ 0x3a1f, 0x28, 0, 0}, //18 
	{ 0x3a18, 0x00, 0, 0}, //
	{ 0x3a19, 0xf8, 0, 0}, //max gain 15.5x
	{ 0x3503, 0x00, 0, 0}, //aec/agc

	{ 0x5000, 0xff, 0, 0}, //lcd,gma,awb,awbg,bc,wc,lenc,isp
	{ 0x5001, 0x3f, 0, 0}, //avg, blc,sde,uv_avg,cmx, cip

	//lens
	{ 0x5100, 0x01, 0, 0},
	{ 0x5101, 0xbf, 0, 0},
	{ 0x5102, 0x00, 0, 0},
	{ 0x5103, 0xaa, 0, 0},
	{ 0x5104, 0x3f, 0, 0},
	{ 0x5105, 0x05, 0, 0},
	{ 0x5106, 0xff, 0, 0},
	{ 0x5107, 0x0f, 0, 0},
	{ 0x5108, 0x01, 0, 0},
	{ 0x5109, 0xff, 0, 0},
	{ 0x510a, 0x00, 0, 0},
	{ 0x510b, 0x72, 0, 0},
	{ 0x510c, 0x45, 0, 0},
	{ 0x510d, 0x06, 0, 0},
	{ 0x510e, 0xff, 0, 0},
	{ 0x510f, 0x0f, 0, 0},
	{ 0x5110, 0x01, 0, 0},
	{ 0x5111, 0xfe, 0, 0},
	{ 0x5112, 0x00, 0, 0},
	{ 0x5113, 0x70, 0, 0},
	{ 0x5114, 0x21, 0, 0},
	{ 0x5115, 0x05, 0, 0},
	{ 0x5116, 0xff, 0, 0},
	{ 0x5117, 0x0f, 0, 0},

	//AWB
	{ 0x520a, 0x74, 0, 0},
	{ 0x520b, 0x64, 0, 0},
	{ 0x520c, 0xd4, 0, 0},

	//@@ Gamma
	{ 0x5301, 0x05, 0, 0},
	{ 0x5302, 0x0c, 0, 0},
	{ 0x5303, 0x1c, 0, 0},
	{ 0x5304, 0x2a, 0, 0},
	{ 0x5305, 0x39, 0, 0},
	{ 0x5306, 0x45, 0, 0},
	{ 0x5307, 0x53, 0, 0},
	{ 0x5308, 0x5d, 0, 0},
	{ 0x5309, 0x68, 0, 0},
	{ 0x530a, 0x7f, 0, 0},
	{ 0x530b, 0x91, 0, 0},
	{ 0x530c, 0xa5, 0, 0},
	{ 0x530d, 0xc6, 0, 0},
	{ 0x530e, 0xde, 0, 0},
	{ 0x530f, 0xef, 0, 0},
	{ 0x5310, 0x16, 0, 0},

	//sharpen/denoise
	{ 0x5003, 0x80, 0, 0}, //enable bit7 to avoid even/odd pattern at bright area
	{ 0x5500, 0x08, 0, 0}, //sharp th1 8x
	{ 0x5501, 0x48, 0, 0}, //sharp th2 8x
	{ 0x5502, 0x18, 0, 0}, //sharp mt offset1
	{ 0x5503, 0x04, 0, 0}, //sharp mt offset2
	{ 0x5504, 0x08, 0, 0}, //dns th1 8x
	{ 0x5505, 0x48, 0, 0}, //dns th2 8x
	{ 0x5506, 0x02, 0, 0}, //dns offset1
	{ 0x5507, 0x16, 0, 0}, //dns offset2
	{ 0x5508, 0x2d, 0, 0}, //disable bit7 (interlace) to avoid even/odd pattern at other area
	{ 0x5509, 0x08, 0, 0}, //sharpth th1 8x
	{ 0x550a, 0x48, 0, 0}, //sharpth th2 8x
	{ 0x550b, 0x06, 0, 0}, //sharpth offset1
	{ 0x550c, 0x04, 0, 0}, //sharpth offset2
	{ 0x550d, 0x01, 0, 0}, //recursive_en

	//SDE, for saturation 120% under D65
	{ 0x5800, 0x02, 0, 0},
	{ 0x5803, 0x2e, 0, 0},//40
	{ 0x5804, 0x20, 0, 0},//34

	//@@ CMX QE
	{ 0x5600, 0x00, 0, 0},
	{ 0x5601, 0x2c, 0, 0},
	{ 0x5602, 0x5a, 0, 0},
	{ 0x5603, 0x06, 0, 0},
	{ 0x5604, 0x1c, 0, 0},
	{ 0x5605, 0x65, 0, 0},
	{ 0x5606, 0x81, 0, 0},
	{ 0x5607, 0x9f, 0, 0},
	{ 0x5608, 0x8a, 0, 0},
	{ 0x5609, 0x15, 0, 0},
	{ 0x560a, 0x01, 0, 0},
	{ 0x560b, 0x9c, 0, 0},

	{ 0x3811, 0x07, 0, 0},// Tradeoff position to make YUV/RAW x VGA/QVGA x Mirror/Flip all work
	{ 0x3813, 0x06, 0, 0},

	{ 0x3630, 0x69, 0, 0},// ADC6

	//{ 0x100, 0x01, 0, 0},
	{ 0x300a, 0x76, 0, 0}, // make SCCB as sensor, not FPGA-c8 or others-102, otherwise, DB "Control Tool" might fail

};

static struct reg_value ovm7695_setting_uyvy[] = {
	// PLL
	{ 0x4300, 0x3F, 0, 0},
	{ 0x030B, 0x02, 0, 0},
	{ 0x3106, 0x92, 0, 0 }, // YUV422
};

static struct reg_value ovm7695_setting_raw[] = {
	{ 0x4300, 0xf8, 0, 0},
	{ 0x030B, 0x04, 0, 0},
	{ 0x3106, 0x91, 0, 0 }, // RAW
	{ 0x301e, 0x60, 0, 0},
};

static struct ovm7695_mode_info ovm7695_mode_info_data[ovm7695_mode_MAX + 1] = {
	{ ovm7695_mode_VGA_640_480, 640, 480, ovm7695_setting_uyvy,
	  ARRAY_SIZE(ovm7695_setting_uyvy) },
};

static struct regulator *io_regulator;
static struct regulator *core_regulator;
static struct regulator *analog_regulator;
static struct regulator *gpo_regulator;
static DEFINE_MUTEX(ovm7695_mutex);

static int ovm7695_probe(struct i2c_client *adapter,
			 const struct i2c_device_id *device_id);
static int ovm7695_remove(struct i2c_client *client);

#ifdef CONFIG_OF
static const struct of_device_id ovm7695_mipi_v2_of_match[] = {
	{
		.compatible = "ovti,ovm7695_mipi",
	},
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, ovm7695_mipi_v2_of_match);
#endif

static const struct i2c_device_id ovm7695_id[] = {
	{ "ovm7695_mipi", 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, ovm7695_id);

static struct i2c_driver ovm7695_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "ovm7695_mipi",
#ifdef CONFIG_OF
		  .of_match_table = of_match_ptr(ovm7695_mipi_v2_of_match),
#endif
		  },
	.probe  = ovm7695_probe,
	.remove = ovm7695_remove,
	.id_table = ovm7695_id,
};

static const struct ovm7695_datafmt ovm7695_colour_fmts[] = {
	{ MEDIA_BUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_JPEG },
//	{ MEDIA_BUS_FMT_SBGGR8_1X8, V4L2_COLORSPACE_SRGB },
};

static int get_capturemode(int width, int height)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ovm7695_valid_res); i++) {
		if ((ovm7695_valid_res[i].width == width) &&
		    (ovm7695_valid_res[i].height == height))
			return i;
	}

	return -1;
}

static struct ovm7695 *to_ovm7695(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ovm7695, subdev);
}

/* Find a data format by a pixel code in an array */
static const struct ovm7695_datafmt *ovm7695_find_datafmt(u32 code)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(ovm7695_colour_fmts); i++)
		if (ovm7695_colour_fmts[i].code == code)
			return ovm7695_colour_fmts + i;

	return NULL;
}

static inline void ovm7695_power_down(struct ovm7695 *sensor, int enable)
{
	if (sensor->pwn_gpio < 0)
		return;

	if (!enable)
		gpio_set_value_cansleep(sensor->pwn_gpio, 0);
	else
		gpio_set_value_cansleep(sensor->pwn_gpio, 1);

	msleep(2);
}

static void ovm7695_reset(struct ovm7695 *sensor)
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

static int ovm7695_regulator_enable(struct device *dev)
{
	int ret = 0;

	io_regulator = devm_regulator_get(dev, "DOVDD");
	if (!IS_ERR(io_regulator)) {
		regulator_set_voltage(io_regulator, OVM7695_VOLTAGE_DIGITAL_IO,
				      OVM7695_VOLTAGE_DIGITAL_IO);
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
				      OVM7695_VOLTAGE_DIGITAL_CORE,
				      OVM7695_VOLTAGE_DIGITAL_CORE);
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
		regulator_set_voltage(analog_regulator, OVM7695_VOLTAGE_ANALOG,
				      OVM7695_VOLTAGE_ANALOG);
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

static void ovm7695_regualtor_disable(void)
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

static s32 ovm7695_write_reg(struct ovm7695 *sensor, u16 reg, u8 val)
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

static s32 ovm7695_read_reg(struct ovm7695 *sensor, u16 reg, u8 *val)
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
static int ovm7695_get_register(struct v4l2_subdev *sd,
				struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ovm7695 *sensor = to_ovm7695(client);
	int ret;
	u8 val;

	if (reg->reg & ~0xffff)
		return -EINVAL;

	reg->size = 1;

	ret = ovm7695_read_reg(sensor, reg->reg, &val);
	if (!ret)
		reg->val = (__u64)val;

	return ret;
}

static int ovm7695_set_register(struct v4l2_subdev *sd,
				const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ovm7695 *sensor = to_ovm7695(client);

	if (reg->reg & ~0xffff || reg->val & ~0xff)
		return -EINVAL;

	return ovm7695_write_reg(sensor, reg->reg, reg->val);

}
#endif

static void OVM7695_stream_on(struct ovm7695 *sensor)
{
	ovm7695_write_reg(sensor, REG_SC_CTRL_MODE, SC_CTRL_MODE_STREAMING);
}

static void OVM7695_stream_off(struct ovm7695 *sensor)
{
	ovm7695_write_reg(sensor, REG_SC_CTRL_MODE, SC_CTRL_MODE_STANDBY);
}

/* download ovm7695 settings to sensor through i2c */
static int ovm7695_download_firmware(struct ovm7695 *sensor,
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
			retval = ovm7695_read_reg(sensor, RegAddr, &RegVal);
			if (retval < 0) {
				pr_err("Error reading OVM7695 register\n");
				goto err;
			}

			RegVal &= ~(u8)Mask;
			Val &= Mask;
			Val |= RegVal;
		}

		retval = ovm7695_write_reg(sensor, RegAddr, Val);
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
static int ovm7695_change_mode_direct(struct ovm7695 *sensor,
				      enum ovm7695_mode mode)
{
	struct reg_value *pModeSetting = NULL;
	s32 ArySize = 0;
	int retval = 0;

	/* check if the input mode and frame rate is valid */
	pModeSetting = ovm7695_mode_info_data[mode].init_data_ptr;
	ArySize = ovm7695_mode_info_data[mode].init_data_size;

	sensor->pix.width = ovm7695_mode_info_data[mode].width;
	sensor->pix.height = ovm7695_mode_info_data[mode].height;

	if (sensor->pix.width == 0 || sensor->pix.height == 0 ||
	    pModeSetting == NULL || ArySize == 0)
		return -EINVAL;

	OVM7695_stream_off(sensor);

	/* Write capture setting */
	retval = ovm7695_download_firmware(sensor, pModeSetting, ArySize);
	if (retval < 0)
		goto err;

err:
	return retval;
}

static int ovm7695_init_mode(struct ovm7695 *sensor, enum ovm7695_mode mode,
			     enum ovm7695_mode orig_mode)
{
	struct device *dev = &sensor->i2c_client->dev;
	struct reg_value *pModeSetting = NULL;
	s32 ArySize = 0;
	int retval = 0;
	u32 msec_wait4stable = 0;

	if ((mode > ovm7695_mode_MAX || mode < ovm7695_mode_MIN) &&
	    (mode != ovm7695_mode_INIT)) {
		dev_err(dev, "Wrong ovm7695 mode detected!\n");
		return -1;
	}

	if (mode == ovm7695_mode_INIT) {
		pModeSetting = ovm7695_setting_init;
		ArySize = ARRAY_SIZE(ovm7695_setting_init);

		sensor->pix.width = ovm7695_valid_res[0].width;
		sensor->pix.height = ovm7695_valid_res[0].height;
		retval = ovm7695_download_firmware(sensor, pModeSetting,
						   ArySize);
		if (retval < 0)
			goto err;

		pModeSetting = ovm7695_setting_uyvy;
		ArySize = ARRAY_SIZE(ovm7695_setting_uyvy);
		retval = ovm7695_download_firmware(sensor, pModeSetting,
						   ArySize);
	} else {
		retval = ovm7695_change_mode_direct(sensor, mode);
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
 * ovm7695_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int ovm7695_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ovm7695 *sensor = to_ovm7695(client);

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
		ovm7695_regualtor_disable();
	}

	sensor->on = on;

	return 0;
}

/*!
 * ovm7695_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 sub device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ovm7695_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ovm7695 *sensor = to_ovm7695(client);
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
 * ovm7695_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 sub device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ovm7695_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ovm7695 *sensor = to_ovm7695(client);
	struct device *dev = &sensor->i2c_client->dev;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 tgt_fps; /* target frames per secound */
	enum ovm7695_frame_rate frame_rate;
	enum ovm7695_mode orig_mode;
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
			frame_rate = ovm7695_30_fps;
		else {
			dev_warn(dev,
				 "The camera frame rate is not supported!\n");
			return -EINVAL;
		}

		orig_mode = sensor->streamcap.capturemode;
		ret = ovm7695_init_mode(
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

static int ovm7695_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	const struct ovm7695_datafmt *fmt = ovm7695_find_datafmt(mf->code);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ovm7695 *sensor = to_ovm7695(client);
	struct device *dev = &sensor->i2c_client->dev;
	int capturemode;
	u8 sys_clk_div = 0x02; 
	u8 yuv_select = 0x92; 
	u8 fmt_ctrl = 0x3f;
	u8 no_idea = 0x61;

	dev_info(dev, "set_fmt");
	if (!fmt) {
		mf->code = ovm7695_colour_fmts[0].code;
		mf->colorspace = ovm7695_colour_fmts[0].colorspace;
		fmt = &ovm7695_colour_fmts[0];
	}

	mf->field = V4L2_FIELD_NONE;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	dev_info(dev, "set_fmt_1");
	sensor->fmt = fmt;

	capturemode = get_capturemode(mf->width, mf->height);
	if (capturemode >= 0) {
		u8 val;
		sensor->streamcap.capturemode = capturemode;
		sensor->pix.width = mf->width;
		sensor->pix.height = mf->height;
		dev_info(dev, "set_fmt_2");

	        ovm7695_read_reg(sensor, 0x301e, &val);
		dev_info(dev, "reg 0x301e: %x", val);
	 
		switch(fmt->code){
			default:
			case MEDIA_BUS_FMT_YUYV8_2X8:
				dev_info(dev, "set yuv");
				sys_clk_div = 0x02;
				yuv_select = 0x92;
				fmt_ctrl = 0x3f;
				no_idea = 0x61;
				break;
			case MEDIA_BUS_FMT_SBGGR8_1X8:
				dev_info(dev, "set raw");
				sys_clk_div = 0x04;
				yuv_select = 0x91;
				fmt_ctrl = 0xf8;
				no_idea = 0x60;
				break;
		};
				
		ovm7695_write_reg(sensor, 0x4300, fmt_ctrl);
		ovm7695_write_reg(sensor, 0x030b, sys_clk_div);
		ovm7695_write_reg(sensor, 0x3106, yuv_select);
		ovm7695_write_reg(sensor, 0x301e, no_idea);
		return 0;
	}

	dev_err(&client->dev, "Set format failed %d, %d\n", fmt->code,
		fmt->colorspace);
	return -EINVAL;
}

static int ovm7695_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ovm7695 *sensor = to_ovm7695(client);
	const struct ovm7695_datafmt *fmt = sensor->fmt;

	if (format->pad)
		return -EINVAL;

	mf->code = fmt->code;
	mf->colorspace = fmt->colorspace;
	mf->field = V4L2_FIELD_NONE;

	mf->width = sensor->pix.width;
	mf->height = sensor->pix.height;

	return 0;
}

static int ovm7695_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= ARRAY_SIZE(ovm7695_colour_fmts))
		return -EINVAL;

	code->code = ovm7695_colour_fmts[code->index].code;
	return 0;
}

/*!
 * ovm7695_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ovm7695_enum_framesizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > ovm7695_mode_MAX)
		return -EINVAL;
	fse->code = fse->max_width = ovm7695_valid_res[0].width;
	fse->min_width = ovm7695_valid_res[0].width;
	fse->max_height = ovm7695_valid_res[0].height;
	fse->min_height = ovm7695_valid_res[0].height;
	return 0;
}

/*!
 * ovm7695_enum_frameintervals - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int
ovm7695_enum_frameintervals(struct v4l2_subdev *sd,
			    struct v4l2_subdev_pad_config *cfg,
			    struct v4l2_subdev_frame_interval_enum *fie)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *dev = &client->dev;
	int j, count = 0;

	if (fie->index < 0 || fie->index > ovm7695_mode_MAX)
		return -EINVAL;

	if (fie->width == 0 || fie->height == 0 || fie->code == 0) {
		dev_warn(dev, "Please assign pixel format, width and height\n");
		return -EINVAL;
	}

	fie->interval.numerator = 1;

	count = 0;
	for (j = 0; j < (ovm7695_mode_MAX + 1); j++) {
		if (fie->width == ovm7695_mode_info_data[j].width &&
		    fie->height == ovm7695_mode_info_data[j].height &&
		    ovm7695_mode_info_data[j].init_data_ptr != NULL) {
			count++;
		}
		if (fie->index == (count - 1)) {
			fie->interval.denominator = ovm7695_framerates[0];
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
static int init_device(struct ovm7695 *sensor)
{
	struct device *dev = &sensor->i2c_client->dev;
	u32 tgt_xclk; /* target xclk */
	u32 tgt_fps; /* target frames per secound */
	int ret;

	sensor->on = true;

	/* mclk */
	tgt_xclk = sensor->mclk;
	tgt_xclk = min(tgt_xclk, (u32)OVM7695_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)OVM7695_XCLK_MIN);
	sensor->mclk = tgt_xclk;

	dev_dbg(dev, "Setting mclk to %d MHz\n", tgt_xclk / 1000000);

	/* Default camera frame rate is set in probe */
	tgt_fps = sensor->streamcap.timeperframe.denominator /
		  sensor->streamcap.timeperframe.numerator;

	if (tgt_fps != 30)
		return -EINVAL; /* Only 30fps now. */

	ret = ovm7695_init_mode(sensor, ovm7695_mode_INIT, ovm7695_mode_INIT);

	return ret;
}

static int ovm7695_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ovm7695 *sensor = to_ovm7695(client);
	struct device *dev = &sensor->i2c_client->dev;

	dev_info(dev, "s_stream: %d\n", enable);
	if (enable)
		OVM7695_stream_on(sensor);
	else
		OVM7695_stream_off(sensor);
	return 0;
}

static struct v4l2_subdev_video_ops ovm7695_subdev_video_ops = {
	.g_parm = ovm7695_g_parm,
	.s_parm = ovm7695_s_parm,
	.s_stream = ovm7695_s_stream,
};

static const struct v4l2_subdev_pad_ops ovm7695_subdev_pad_ops = {
	.enum_frame_size = ovm7695_enum_framesizes,
	.enum_frame_interval = ovm7695_enum_frameintervals,
	.enum_mbus_code = ovm7695_enum_mbus_code,
	.set_fmt = ovm7695_set_fmt,
	.get_fmt = ovm7695_get_fmt,
};

static struct v4l2_subdev_core_ops ovm7695_subdev_core_ops = {
	.s_power = ovm7695_s_power,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ovm7695_get_register,
	.s_register = ovm7695_set_register,
#endif
};

static struct v4l2_subdev_ops ovm7695_subdev_ops = {
	.core = &ovm7695_subdev_core_ops,
	.video = &ovm7695_subdev_video_ops,
	.pad = &ovm7695_subdev_pad_ops,
};

/*!
 * ovm7695 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int ovm7695_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct pinctrl *pinctrl;
	struct device *dev = &client->dev;
	int retval;
	u8 chip_id_high, chip_id_low;
	struct ovm7695 *sensor;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	/* ovm7695 pinctrl */
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
					       "ovm7695_mipi_pwdn");
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
					       "ovm7695_mipi_reset");
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

	sensor->io_init = ovm7695_reset;
	sensor->i2c_client = client;
	sensor->pix.pixelformat = V4L2_PIX_FMT_YUYV;
	sensor->pix.width = ovm7695_valid_res[0].width;
	sensor->pix.height = ovm7695_valid_res[0].height;
	sensor->streamcap.capability =
		V4L2_MODE_HIGHQUALITY | V4L2_CAP_TIMEPERFRAME;
	sensor->streamcap.capturemode = 0;
	sensor->streamcap.timeperframe.denominator = DEFAULT_FPS;
	sensor->streamcap.timeperframe.numerator = 1;

	ovm7695_regulator_enable(&client->dev);

	mutex_lock(&ovm7695_mutex);
	{
		ovm7695_reset(sensor);
		ovm7695_power_down(sensor, 0);
	}
	mutex_unlock(&ovm7695_mutex);
	if (retval < 0) {
		clk_disable_unprepare(sensor->sensor_clk);
		return -ENODEV;
	}

	retval = ovm7695_read_reg(sensor, OVM7695_CHIP_ID_HIGH_BYTE,
				  &chip_id_high);
	if (retval < 0 || chip_id_high != 0x76) {
		dev_warn(dev, "Camera is not found\n");
		ovm7695_regualtor_disable();
		clk_disable_unprepare(sensor->sensor_clk);
		return -ENODEV;
	}
	retval = ovm7695_read_reg(sensor, OVM7695_CHIP_ID_LOW_BYTE,
				  &chip_id_low);
	if (retval < 0 || chip_id_low != 0x95) {
		dev_warn(dev, "Camera is not found\n");
		ovm7695_regualtor_disable();
		clk_disable_unprepare(sensor->sensor_clk);
		return -ENODEV;
	}

	retval = init_device(sensor);
	if (retval < 0) {
		ovm7695_regualtor_disable();
		clk_disable_unprepare(sensor->sensor_clk);
		dev_warn(dev, "Camera init failed\n");
		ovm7695_power_down(sensor, 1);
		return retval;
	}

	v4l2_i2c_subdev_init(&sensor->subdev, client, &ovm7695_subdev_ops);

	sensor->subdev.grp_id = 678;
	retval = v4l2_async_register_subdev(&sensor->subdev);
	if (retval < 0)
		dev_err(&client->dev, "Async register failed, ret=%d\n",
			retval);

	OVM7695_stream_off(sensor);
	dev_info(dev, "Camera is found\n");
	return retval;
}

/*!
 * ovm7695 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ovm7695_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ovm7695 *sensor = to_ovm7695(client);

	v4l2_async_unregister_subdev(sd);

	clk_disable_unprepare(sensor->sensor_clk);

	ovm7695_power_down(sensor, 1);

	ovm7695_regualtor_disable();

	return 0;
}

module_i2c_driver(ovm7695_i2c_driver);

MODULE_AUTHOR("Bruetsch Elektronik AG.");
MODULE_DESCRIPTION("OVM7695 MIPI Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
