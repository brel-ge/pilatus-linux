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
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>


#define MIN_FPS 30
#define MAX_FPS 30
#define DEFAULT_FPS 30


enum eiger_stub_mode {
	eiger_stub_mode_MIN = 0,
	eiger_stub_mode_4_3_640 = 0,
	eiger_stub_mode_1_1_800 = 1,
	eiger_stub_mode_16_9_1920 = 2,
	eiger_stub_mode_16_10_1280 = 3,
	eiger_stub_mode_4_3_648 = 4,
	eiger_stub_mode_MAX = 4,
	eiger_stub_mode_INIT = 0xff, /*only for sensor init*/
};

enum eiger_stub_frame_rate { eiger_stub_30_fps };

static int eiger_stub_framerates[] = {
	[eiger_stub_30_fps] = 30,
};

struct eiger_stub_datafmt {
	u32 code;
	enum v4l2_colorspace colorspace;
};


struct eiger_stub {
	struct i2c_client *i2c_client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_fwnode_endpoint ep; /* the parsed DT endpoint info */

        struct v4l2_captureparm streamcap;
	bool on;

	/* lock to protect all members below */
	struct mutex lock;

	struct v4l2_mbus_framefmt fmt;
	const struct eiger_stub_datafmt *datafmt;
	struct clk *sensor_clk;
};


struct eiger_stub_res {
	int width;
	int height;
};



struct eiger_stub_res eiger_stub_valid_res[] = {
	[eiger_stub_mode_4_3_640] = { 640, 480 },
	[eiger_stub_mode_1_1_800] = { 800, 800 },
	[eiger_stub_mode_16_9_1920] = { 1920, 1080 },
	[eiger_stub_mode_16_10_1280] = { 1280, 800 },
	[eiger_stub_mode_4_3_648] = { 648, 488 },
};


static const struct eiger_stub_datafmt eiger_stub_colour_fmts[] = {
	{ MEDIA_BUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_SRGB },
	{ MEDIA_BUS_FMT_SBGGR8_1X8, V4L2_COLORSPACE_SRGB },
};

static int get_capturemode(int width, int height)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(eiger_stub_valid_res); i++) {
		if ((eiger_stub_valid_res[i].width == width) &&
		    (eiger_stub_valid_res[i].height == height))
			return i;
	}

	return -1;
}

static struct eiger_stub *to_eiger_stub(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct eiger_stub, sd);
}

/* Find a data format by a pixel code in an array */
static const struct eiger_stub_datafmt *eiger_stub_find_datafmt(u32 code)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(eiger_stub_colour_fmts); i++)
		if (eiger_stub_colour_fmts[i].code == code)
			return eiger_stub_colour_fmts + i;

	return NULL;
}

static int eiger_stub_init_mode(struct eiger_stub *sensor, enum eiger_stub_mode mode,
			     enum eiger_stub_mode orig_mode)
{
	struct device *dev = &sensor->i2c_client->dev;
	int retval = 0;

	if ((mode > eiger_stub_mode_MAX || mode < eiger_stub_mode_MIN) &&
	    (mode != eiger_stub_mode_INIT)) {
		dev_err(dev, "Wrong eiger_stub mode detected!\n");
		return -1;
	}

	if (mode == eiger_stub_mode_INIT) {
		sensor->fmt.width = eiger_stub_valid_res[0].width;
		sensor->fmt.height = eiger_stub_valid_res[0].height;
	}

	if (retval < 0)
		goto err;

err:
	return retval;
}

/*!
 * eiger_stub_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int eiger_stub_s_power(struct v4l2_subdev *sd, int on)
{

	return 0;
}

/*!
 * eiger_stub_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 sub device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int eiger_stub_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct eiger_stub *sensor = to_eiger_stub(client);
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
 * eiger_stub_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 sub device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int eiger_stub_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct eiger_stub *sensor = to_eiger_stub(client);
	struct device *dev = &sensor->i2c_client->dev;
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 tgt_fps; /* target frames per secound */
	enum eiger_stub_frame_rate frame_rate;
	enum eiger_stub_mode orig_mode;
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
			frame_rate = eiger_stub_30_fps;
		else {
			dev_warn(dev,
				 "The camera frame rate is not supported!\n");
			return -EINVAL;
		}

		orig_mode = sensor->streamcap.capturemode;
		ret = eiger_stub_init_mode(
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



static int eiger_stub_set_fmt(struct v4l2_subdev *sd,
		           struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_format *format)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct eiger_stub *sensor = to_eiger_stub(client);

	struct v4l2_mbus_framefmt *fmt = &format->format;
	const struct eiger_stub_datafmt *datafmt = eiger_stub_find_datafmt(fmt->code);
	int capturemode;
	int ret = 0;

	mutex_lock(&sensor->lock);

	if (!datafmt) {
		fmt->code = eiger_stub_colour_fmts[0].code;
		fmt->colorspace = eiger_stub_colour_fmts[0].colorspace;
		datafmt = &eiger_stub_colour_fmts[0];
	}

	fmt->field = V4L2_FIELD_NONE;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		*v4l2_subdev_get_try_format(sd, sd_state, 0) = *fmt;
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

static int eiger_stub_get_fmt(struct v4l2_subdev *sd,
		           struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_format *format)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct eiger_stub *sensor = to_eiger_stub(client);
	struct device *dev = &client->dev;

	struct v4l2_mbus_framefmt *fmt;

	if (format->pad)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
		fmt = v4l2_subdev_get_try_format(&sensor->sd, sd_state,
						 format->pad);
	} else {
		fmt = &sensor->fmt;
	}

	format->format = *fmt;

	mutex_unlock(&sensor->lock);

	return 0;
}

static int eiger_stub_enum_mbus_code(struct v4l2_subdev *sd,
			          struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index >= ARRAY_SIZE(eiger_stub_colour_fmts))
		return -EINVAL;

	code->code = eiger_stub_colour_fmts[code->index].code;
	return 0;
}

/*!
 * eiger_stub_enum_frame_size - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int eiger_stub_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index > eiger_stub_mode_MAX)
		return -EINVAL;

	fse->max_width = eiger_stub_valid_res[fse->index].width;
	fse->min_width = eiger_stub_valid_res[fse->index].width;
	fse->max_height = eiger_stub_valid_res[fse->index].height;
	fse->min_height = eiger_stub_valid_res[fse->index].height;
	return 0;
}

/*!
 * eiger_stub_enum_frame_interval - V4L2 sensor interface handler for
 *			       VIDIOC_ENUM_FRAMEINTERVALS ioctl
 * @s: pointer to standard V4L2 device structure
 * @fival: standard V4L2 VIDIOC_ENUM_FRAMEINTERVALS ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int
eiger_stub_enum_frame_interval(struct v4l2_subdev *sd,
			    struct v4l2_subdev_state *sd_state,
			    struct v4l2_subdev_frame_interval_enum *fie)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct device *dev = &client->dev;
	int j, count = 0;

	if (fie->index < 0 || fie->index > eiger_stub_mode_MAX)
		return -EINVAL;

	if (fie->width == 0 || fie->height == 0 || fie->code == 0) {
		dev_warn(dev, "Please assign pixel format, width and height\n");
		return -EINVAL;
	}

	fie->interval.numerator = 1;

	count = 0;
	for (j = 0; j < (eiger_stub_mode_MAX + 1); j++) {
		if (fie->width == eiger_stub_valid_res[j].width &&
		    fie->height == eiger_stub_valid_res[j].height) {
			count++;
		}
		if (fie->index == (count - 1)) {
			fie->interval.denominator = eiger_stub_framerates[0];
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
static int init_device(struct eiger_stub *sensor)
{
	u32 tgt_fps; /* target frames per secound */
	int ret;

	sensor->on = true;

	/* Default camera frame rate is set in probe */
	tgt_fps = sensor->streamcap.timeperframe.denominator /
		  sensor->streamcap.timeperframe.numerator;

	if (tgt_fps != 30)
		return -EINVAL; /* Only 30fps now. */


	ret = eiger_stub_init_mode(sensor, eiger_stub_mode_INIT, eiger_stub_mode_INIT);

	return ret;
}

static int eiger_stub_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct eiger_stub *sensor = to_eiger_stub(client);
	struct device *dev = &sensor->i2c_client->dev;

	dev_info(dev, "s_stream: %d\n", enable);
	return 0;
}

static struct v4l2_subdev_video_ops eiger_stub_subdev_video_ops = {
	.g_parm = eiger_stub_g_parm,
	.s_parm = eiger_stub_s_parm,
	.s_stream = eiger_stub_s_stream,
};

static const struct v4l2_subdev_pad_ops eiger_stub_subdev_pad_ops = {
	.enum_frame_size = eiger_stub_enum_frame_size,
	.enum_frame_interval = eiger_stub_enum_frame_interval,
	.enum_mbus_code = eiger_stub_enum_mbus_code,
	.set_fmt = eiger_stub_set_fmt,
	.get_fmt = eiger_stub_get_fmt,
};

static struct v4l2_subdev_core_ops eiger_stub_subdev_core_ops = {
	.s_power = eiger_stub_s_power,
};

static struct v4l2_subdev_ops eiger_stub_subdev_ops = {
	.core = &eiger_stub_subdev_core_ops,
	.video = &eiger_stub_subdev_video_ops,
	.pad = &eiger_stub_subdev_pad_ops,
};

static int eiger_stub_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct media_entity_operations eiger_stub_media_ops = {
	.link_setup = eiger_stub_link_setup,
};


/*!
 * eiger_stub I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int eiger_stub_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct fwnode_handle *endpoint;
	struct eiger_stub *sensor;
	struct v4l2_mbus_framefmt *fmt;
	int ret;

	sensor = devm_kzalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(sensor->sensor_clk)) {
		/* assuming clock enabled by default */
		sensor->sensor_clk = NULL;
		dev_err(dev, "clock-frequency missing or invalid\n");
		return PTR_ERR(sensor->sensor_clk);
	}

	clk_prepare_enable(sensor->sensor_clk);

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(&client->dev),
						  NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	ret = v4l2_fwnode_endpoint_parse(endpoint, &sensor->ep);
	fwnode_handle_put(endpoint);
	if (ret) {
		dev_err(dev, "Could not parse endpoint\n");
		return ret;
	}



	sensor->i2c_client = client;

	/*
	 * default init sequence initialize sensor to
	 * YUV422 UYVY VGA@30fps
	 */
	fmt = &sensor->fmt;
	fmt->code = MEDIA_BUS_FMT_YUYV8_2X8;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width = 640;
	fmt->height = 480;
	fmt->field = V4L2_FIELD_NONE;

	sensor->streamcap.capability = 
		V4L2_MODE_HIGHQUALITY | V4L2_CAP_TIMEPERFRAME;
	sensor->streamcap.capturemode = 0;
	sensor->streamcap.timeperframe.denominator = DEFAULT_FPS;
	sensor->streamcap.timeperframe.numerator = 1;


	ret = init_device(sensor);
	if (ret < 0) {
		clk_disable_unprepare(sensor->sensor_clk);
		dev_warn(dev, "Camera init failed\n");
		return ret;
	}

	v4l2_i2c_subdev_init(&sensor->sd, client, &eiger_stub_subdev_ops);

	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_EVENTS;
	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.entity.ops = &eiger_stub_media_ops;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		return ret;

        mutex_init(&sensor->lock);

	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret)
		goto out_media;

        ret = v4l2_async_register_subdev_sensor(&sensor->sd);
	if (ret < 0)
		dev_err(&client->dev, "Async register failed, ret=%d\n",
			ret);

	dev_info(dev, "Camera is found\n");
	return ret;

out_media:
	media_entity_cleanup(&sensor->sd.entity);
	return ret;
}

/*!
 * eiger_stub I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static void eiger_stub_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct eiger_stub *sensor = to_eiger_stub(client);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sensor->sd.entity);
	//v4l2_ctrl_handler_free(&sensor->ctrls.handler);
	mutex_destroy(&sensor->lock);
}


static const struct i2c_device_id eiger_stub_id[] = {
	{ "eiger_stub", 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, eiger_stub_id);

static const struct of_device_id eiger_stub_dt_ids[] = {
	{ .compatible = "brel,eiger_stub" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, eiger_stub_mipi_v2_of_match);

static struct i2c_driver eiger_stub_i2c_driver = {
	.driver = {
		  .name  = "eiger_stub",
		  .of_match_table = eiger_stub_dt_ids,
		  },
	.id_table = eiger_stub_id,
	.probe_new  = eiger_stub_probe,
	.remove = eiger_stub_remove,
};

module_i2c_driver(eiger_stub_i2c_driver);

MODULE_AUTHOR("Bruetsch Elektronik AG.");
MODULE_DESCRIPTION("Eiger_stub Camera Subdev Driver");
MODULE_LICENSE("GPL");
