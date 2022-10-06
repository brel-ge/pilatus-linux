// SPDX-License-Identifier: GPL-2.0
/*
 * Raydium pilatus_panel MIPI-DSI panel driver
 *
 * Copyright 2019 NXP
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

/* Panel specific color-format bits */
#define COL_FMT_16BPP 0x55
#define COL_FMT_18BPP 0x66
#define COL_FMT_24BPP 0x77

/* Write Manufacture Command Set Control */
#define WRMAUCCTR 0xFE

#define REGFLAG_DELAY             							(0XFFFE)
#define REGFLAG_END_OF_TABLE      							(0xF100)	// END OF REGISTERS MARKER

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

#define dsi_generic_write_seq(dsi, cmd, seq...) do {				\
		static const u8 d[] = {cmd , seq };				\
		int ret;						\
		ret = mipi_dsi_generic_write(dsi, d, ARRAY_SIZE(d));	\
		if (ret < 0)						\
			return ret;					\
	} while (0)



static int init_sequence(struct mipi_dsi_device *dsi)
{

dsi_generic_write_seq(dsi,0xE0,0x00);
dsi_generic_write_seq(dsi,0xE1,0x93);
dsi_generic_write_seq(dsi,0xE2,0x65);
dsi_generic_write_seq(dsi,0xE3,0xF8);
dsi_generic_write_seq(dsi,0x80,0x03);


dsi_generic_write_seq(dsi,0xE0,0x01);
dsi_generic_write_seq(dsi,0x00,0x00);
dsi_generic_write_seq(dsi,0x01,0x3B);

dsi_generic_write_seq(dsi,0x0C,0x74);

dsi_generic_write_seq(dsi,0x17,0x00);
dsi_generic_write_seq(dsi,0x18,0xAF);//VGMP=4.8V
dsi_generic_write_seq(dsi,0x19,0x00);//VGSP=0.3V
dsi_generic_write_seq(dsi,0x1A,0x00);
dsi_generic_write_seq(dsi,0x1B,0xAF);
dsi_generic_write_seq(dsi,0x1C,0x00);


dsi_generic_write_seq(dsi,0x35,0x26);	//ASP=0110

dsi_generic_write_seq(dsi,0x37,0x09);	//SS=1,BGR=1

dsi_generic_write_seq(dsi,0x38,0x04);	//JDT=100 column inversion
dsi_generic_write_seq(dsi,0x39,0x00);
dsi_generic_write_seq(dsi,0x3A,0x01);
dsi_generic_write_seq(dsi,0x3C,0x78);
dsi_generic_write_seq(dsi,0x3D,0xFF);
dsi_generic_write_seq(dsi,0x3E,0xFF);
dsi_generic_write_seq(dsi,0x3F,0x7F);

dsi_generic_write_seq(dsi,0x40,0x06);
dsi_generic_write_seq(dsi,0x41,0xA0);
dsi_generic_write_seq(dsi,0x42,0x81);
dsi_generic_write_seq(dsi,0x43,0x14);
dsi_generic_write_seq(dsi,0x44,0x23);
dsi_generic_write_seq(dsi,0x45,0x28);

dsi_generic_write_seq(dsi,0x55,0x02);
dsi_generic_write_seq(dsi,0x57,0x69);
dsi_generic_write_seq(dsi,0x59,0x0A);
dsi_generic_write_seq(dsi,0x5A,0x2A);
dsi_generic_write_seq(dsi,0x5B,0x17);

dsi_generic_write_seq(dsi,0x5D,0x7F);
dsi_generic_write_seq(dsi,0x5E,0x6B);
dsi_generic_write_seq(dsi,0x5F,0x5C);
dsi_generic_write_seq(dsi,0x60,0x4F);
dsi_generic_write_seq(dsi,0x61,0x4D);
dsi_generic_write_seq(dsi,0x62,0x3F);
dsi_generic_write_seq(dsi,0x63,0x42);
dsi_generic_write_seq(dsi,0x64,0x2B);
dsi_generic_write_seq(dsi,0x65,0x44);
dsi_generic_write_seq(dsi,0x66,0x43);
dsi_generic_write_seq(dsi,0x67,0x43);
dsi_generic_write_seq(dsi,0x68,0x63);
dsi_generic_write_seq(dsi,0x69,0x52);
dsi_generic_write_seq(dsi,0x6A,0x5A);
dsi_generic_write_seq(dsi,0x6B,0x4F);
dsi_generic_write_seq(dsi,0x6C,0x4E);
dsi_generic_write_seq(dsi,0x6D,0x20);
dsi_generic_write_seq(dsi,0x6E,0x0F);
dsi_generic_write_seq(dsi,0x6F,0x00);

dsi_generic_write_seq(dsi,0x70,0x7F);
dsi_generic_write_seq(dsi,0x71,0x6B);
dsi_generic_write_seq(dsi,0x72,0x5C);
dsi_generic_write_seq(dsi,0x73,0x4F);
dsi_generic_write_seq(dsi,0x74,0x4D);
dsi_generic_write_seq(dsi,0x75,0x3F);
dsi_generic_write_seq(dsi,0x76,0x42);
dsi_generic_write_seq(dsi,0x77,0x2B);
dsi_generic_write_seq(dsi,0x78,0x44);
dsi_generic_write_seq(dsi,0x79,0x43);
dsi_generic_write_seq(dsi,0x7A,0x43);
dsi_generic_write_seq(dsi,0x7B,0x63);
dsi_generic_write_seq(dsi,0x7C,0x52);
dsi_generic_write_seq(dsi,0x7D,0x5A);
dsi_generic_write_seq(dsi,0x7E,0x4F);
dsi_generic_write_seq(dsi,0x7F,0x4E);
dsi_generic_write_seq(dsi,0x80,0x20);
dsi_generic_write_seq(dsi,0x81,0x0F);
dsi_generic_write_seq(dsi,0x82,0x00);



dsi_generic_write_seq(dsi,0xE0,0x02);
dsi_generic_write_seq(dsi,0x00,0x02);
dsi_generic_write_seq(dsi,0x01,0x02);
dsi_generic_write_seq(dsi,0x02,0x00);
dsi_generic_write_seq(dsi,0x03,0x00);
dsi_generic_write_seq(dsi,0x04,0x1E);
dsi_generic_write_seq(dsi,0x05,0x1E);
dsi_generic_write_seq(dsi,0x06,0x1F);
dsi_generic_write_seq(dsi,0x07,0x1F);
dsi_generic_write_seq(dsi,0x08,0x1F);
dsi_generic_write_seq(dsi,0x09,0x17);
dsi_generic_write_seq(dsi,0x0A,0x17);
dsi_generic_write_seq(dsi,0x0B,0x37);
dsi_generic_write_seq(dsi,0x0C,0x37);
dsi_generic_write_seq(dsi,0x0D,0x47);
dsi_generic_write_seq(dsi,0x0E,0x47);
dsi_generic_write_seq(dsi,0x0F,0x45);
dsi_generic_write_seq(dsi,0x10,0x45);
dsi_generic_write_seq(dsi,0x11,0x4B);
dsi_generic_write_seq(dsi,0x12,0x4B);
dsi_generic_write_seq(dsi,0x13,0x49);
dsi_generic_write_seq(dsi,0x14,0x49);
dsi_generic_write_seq(dsi,0x15,0x1F);

dsi_generic_write_seq(dsi,0x16,0x01);
dsi_generic_write_seq(dsi,0x17,0x01);
dsi_generic_write_seq(dsi,0x18,0x00);
dsi_generic_write_seq(dsi,0x19,0x00);
dsi_generic_write_seq(dsi,0x1A,0x1E);
dsi_generic_write_seq(dsi,0x1B,0x1E);
dsi_generic_write_seq(dsi,0x1C,0x1F);
dsi_generic_write_seq(dsi,0x1D,0x1F);
dsi_generic_write_seq(dsi,0x1E,0x1F);
dsi_generic_write_seq(dsi,0x1F,0x17);
dsi_generic_write_seq(dsi,0x20,0x17);
dsi_generic_write_seq(dsi,0x21,0x37);
dsi_generic_write_seq(dsi,0x22,0x37);
dsi_generic_write_seq(dsi,0x23,0x46);
dsi_generic_write_seq(dsi,0x24,0x46);
dsi_generic_write_seq(dsi,0x25,0x44);
dsi_generic_write_seq(dsi,0x26,0x44);
dsi_generic_write_seq(dsi,0x27,0x4A);
dsi_generic_write_seq(dsi,0x28,0x4A);
dsi_generic_write_seq(dsi,0x29,0x48);
dsi_generic_write_seq(dsi,0x2A,0x48);
dsi_generic_write_seq(dsi,0x2B,0x1F);

dsi_generic_write_seq(dsi,0x2C,0x01);
dsi_generic_write_seq(dsi,0x2D,0x01);
dsi_generic_write_seq(dsi,0x2E,0x00);
dsi_generic_write_seq(dsi,0x2F,0x00);
dsi_generic_write_seq(dsi,0x30,0x1F);
dsi_generic_write_seq(dsi,0x31,0x1F);
dsi_generic_write_seq(dsi,0x32,0x1E);
dsi_generic_write_seq(dsi,0x33,0x1E);
dsi_generic_write_seq(dsi,0x34,0x1F);
dsi_generic_write_seq(dsi,0x35,0x17);
dsi_generic_write_seq(dsi,0x36,0x17);
dsi_generic_write_seq(dsi,0x37,0x37);
dsi_generic_write_seq(dsi,0x38,0x37);
dsi_generic_write_seq(dsi,0x39,0x08);
dsi_generic_write_seq(dsi,0x3A,0x08);
dsi_generic_write_seq(dsi,0x3B,0x0A);
dsi_generic_write_seq(dsi,0x3C,0x0A);
dsi_generic_write_seq(dsi,0x3D,0x04);
dsi_generic_write_seq(dsi,0x3E,0x04);
dsi_generic_write_seq(dsi,0x3F,0x06);
dsi_generic_write_seq(dsi,0x40,0x06);
dsi_generic_write_seq(dsi,0x41,0x1F);

dsi_generic_write_seq(dsi,0x42,0x02);
dsi_generic_write_seq(dsi,0x43,0x02);
dsi_generic_write_seq(dsi,0x44,0x00);
dsi_generic_write_seq(dsi,0x45,0x00);
dsi_generic_write_seq(dsi,0x46,0x1F);
dsi_generic_write_seq(dsi,0x47,0x1F);
dsi_generic_write_seq(dsi,0x48,0x1E);
dsi_generic_write_seq(dsi,0x49,0x1E);
dsi_generic_write_seq(dsi,0x4A,0x1F);
dsi_generic_write_seq(dsi,0x4B,0x17);
dsi_generic_write_seq(dsi,0x4C,0x17);
dsi_generic_write_seq(dsi,0x4D,0x37);
dsi_generic_write_seq(dsi,0x4E,0x37);
dsi_generic_write_seq(dsi,0x4F,0x09);
dsi_generic_write_seq(dsi,0x50,0x09);
dsi_generic_write_seq(dsi,0x51,0x0B);
dsi_generic_write_seq(dsi,0x52,0x0B);
dsi_generic_write_seq(dsi,0x53,0x05);
dsi_generic_write_seq(dsi,0x54,0x05);
dsi_generic_write_seq(dsi,0x55,0x07);
dsi_generic_write_seq(dsi,0x56,0x07);
dsi_generic_write_seq(dsi,0x57,0x1F);

dsi_generic_write_seq(dsi,0x58,0x40);
dsi_generic_write_seq(dsi,0x5B,0x30);
dsi_generic_write_seq(dsi,0x5C,0x16);
dsi_generic_write_seq(dsi,0x5D,0x34);
dsi_generic_write_seq(dsi,0x5E,0x05);
dsi_generic_write_seq(dsi,0x5F,0x02);
dsi_generic_write_seq(dsi,0x63,0x00);
dsi_generic_write_seq(dsi,0x64,0x6A);
dsi_generic_write_seq(dsi,0x67,0x73);
dsi_generic_write_seq(dsi,0x68,0x1D);
dsi_generic_write_seq(dsi,0x69,0x08);
dsi_generic_write_seq(dsi,0x6A,0x6A);
dsi_generic_write_seq(dsi,0x6B,0x08);

dsi_generic_write_seq(dsi,0x6C,0x00);
dsi_generic_write_seq(dsi,0x6D,0x00);
dsi_generic_write_seq(dsi,0x6E,0x00);
dsi_generic_write_seq(dsi,0x6F,0x88);

dsi_generic_write_seq(dsi,0x75,0xFF);
dsi_generic_write_seq(dsi,0x77,0xDD);
dsi_generic_write_seq(dsi,0x78,0x3F);
dsi_generic_write_seq(dsi,0x79,0x15);
dsi_generic_write_seq(dsi,0x7A,0x17);
dsi_generic_write_seq(dsi,0x7D,0x14);
dsi_generic_write_seq(dsi,0x7E,0x82);


dsi_generic_write_seq(dsi,0xE0,0x04);
dsi_generic_write_seq(dsi,0x00,0x0E);
dsi_generic_write_seq(dsi,0x02,0xB3);
dsi_generic_write_seq(dsi,0x09,0x61);
dsi_generic_write_seq(dsi,0x0E,0x48);

dsi_generic_write_seq(dsi,0xE0,0x00);

dsi_generic_write_seq(dsi,0xE6,0x02);
dsi_generic_write_seq(dsi,0xE7,0x0C);
return 0;
}

static int init_sequence1(struct mipi_dsi_device *dsi)
{

dsi_generic_write_seq(dsi,0xE0,0x03);

dsi_generic_write_seq(dsi,0x2B,0x01);
dsi_generic_write_seq(dsi,0x2C,0x00);

dsi_generic_write_seq(dsi,0x30,0x40);
dsi_generic_write_seq(dsi,0x31,0x1F);
dsi_generic_write_seq(dsi,0x32,0xFF);
dsi_generic_write_seq(dsi,0x33,0xF0);
dsi_generic_write_seq(dsi,0x34,0xF1);
dsi_generic_write_seq(dsi,0x35,0xF2);
dsi_generic_write_seq(dsi,0x36,0x10);
dsi_generic_write_seq(dsi,0x37,0xF3);
dsi_generic_write_seq(dsi,0x38,0x21);
dsi_generic_write_seq(dsi,0x39,0x0F);
dsi_generic_write_seq(dsi,0x3A,0xEF);
dsi_generic_write_seq(dsi,0x3B,0xFF);
dsi_generic_write_seq(dsi,0x3C,0xFF);
dsi_generic_write_seq(dsi,0x3D,0xFF);
dsi_generic_write_seq(dsi,0x3E,0xFF);
dsi_generic_write_seq(dsi,0x3F,0xFF);
dsi_generic_write_seq(dsi,0x40,0xFF);
dsi_generic_write_seq(dsi,0x41,0xFF);
dsi_generic_write_seq(dsi,0x42,0x00);
dsi_generic_write_seq(dsi,0x43,0x00);
dsi_generic_write_seq(dsi,0x44,0x00);
dsi_generic_write_seq(dsi,0x45,0x0F);
dsi_generic_write_seq(dsi,0x46,0x00);
dsi_generic_write_seq(dsi,0x47,0x00);
dsi_generic_write_seq(dsi,0x48,0x00);
dsi_generic_write_seq(dsi,0x49,0x1F);
dsi_generic_write_seq(dsi,0x4A,0xFE);
dsi_generic_write_seq(dsi,0x4B,0xDD);
dsi_generic_write_seq(dsi,0x4C,0xFE);
dsi_generic_write_seq(dsi,0x4D,0xED);
dsi_generic_write_seq(dsi,0x4E,0xFD);
dsi_generic_write_seq(dsi,0x4F,0xFF);
dsi_generic_write_seq(dsi,0x50,0xFE);
dsi_generic_write_seq(dsi,0x51,0xFF);
dsi_generic_write_seq(dsi,0x52,0xFF);
dsi_generic_write_seq(dsi,0x53,0x00);


dsi_generic_write_seq(dsi,0x54,0x3D);
dsi_generic_write_seq(dsi,0x55,0x3D);
dsi_generic_write_seq(dsi,0x56,0xDE);
dsi_generic_write_seq(dsi,0x57,0xEF);
dsi_generic_write_seq(dsi,0x58,0xF1);
dsi_generic_write_seq(dsi,0x59,0xF0);
dsi_generic_write_seq(dsi,0x5A,0xFF);
dsi_generic_write_seq(dsi,0x5B,0xFF);
dsi_generic_write_seq(dsi,0x5C,0x00);
dsi_generic_write_seq(dsi,0x5D,0x0F);
dsi_generic_write_seq(dsi,0x5E,0xEF);
dsi_generic_write_seq(dsi,0x5F,0x00);
dsi_generic_write_seq(dsi,0x60,0x01);
dsi_generic_write_seq(dsi,0x61,0x11);
dsi_generic_write_seq(dsi,0x62,0x11);
dsi_generic_write_seq(dsi,0x63,0x22);
dsi_generic_write_seq(dsi,0x64,0x22);
dsi_generic_write_seq(dsi,0x65,0x32);
dsi_generic_write_seq(dsi,0x66,0x44);
dsi_generic_write_seq(dsi,0x67,0x43);
dsi_generic_write_seq(dsi,0x68,0x33);
dsi_generic_write_seq(dsi,0x69,0x33);
dsi_generic_write_seq(dsi,0x6A,0x44);
dsi_generic_write_seq(dsi,0x6B,0x44);
dsi_generic_write_seq(dsi,0x6C,0x44);
dsi_generic_write_seq(dsi,0x6D,0x43);
dsi_generic_write_seq(dsi,0x6E,0x55);
dsi_generic_write_seq(dsi,0x6F,0x43);
dsi_generic_write_seq(dsi,0x70,0x22);
dsi_generic_write_seq(dsi,0x71,0x1F);
dsi_generic_write_seq(dsi,0x72,0xFE);
dsi_generic_write_seq(dsi,0x73,0xFD);
dsi_generic_write_seq(dsi,0x74,0xDD);
dsi_generic_write_seq(dsi,0x75,0xCC);
dsi_generic_write_seq(dsi,0x76,0xCC);
dsi_generic_write_seq(dsi,0x77,0x05);

dsi_generic_write_seq(dsi,0xE0,0x00);
	return 0;
}

static int init_sequence2(struct mipi_dsi_device *dsi)
{

dsi_generic_write_seq(dsi,0x35,0x00);
	return 0;
}

static const u32 rad_bus_formats[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_RGB666_1X18,
	MEDIA_BUS_FMT_RGB565_1X16,
};

static const u32 rad_bus_flags = DRM_BUS_FLAG_DE_LOW |
				 DRM_BUS_FLAG_PIXDATA_SAMPLE_POSEDGE;

struct pilatus_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;

	struct gpio_desc *reset;
	struct backlight_device *backlight;

	struct regulator_bulk_data *supplies;
	unsigned int num_supplies;

	bool prepared;
	bool enabled;

	const struct rad_platform_data *pdata;
};

struct rad_platform_data {
	int (*enable)(struct pilatus_panel *panel);
};

static const struct drm_display_mode default_mode = {
	.hdisplay    = 800,
	.hsync_start = 800 + 20,
	.hsync_end   = 800 + 20 + 20,
	.htotal	     = 800 + 20 + 20 + 40,
	.vdisplay    = 1280,
	.vsync_start = 1280 + 20,
	.vsync_end   = 1280 + 20 + 4,
	.vtotal	     = 1280 + 20 + 4 + 20,
	.clock	     = 70000,
	.width_mm    = 135,
	.height_mm   = 217,
	.flags = DRM_MODE_FLAG_NHSYNC |
		 DRM_MODE_FLAG_NVSYNC,
};

static inline struct pilatus_panel *to_pilatus_panel(struct drm_panel *panel)
{
	return container_of(panel, struct pilatus_panel, panel);
}

static int color_format_from_dsi_format(enum mipi_dsi_pixel_format format)
{
	switch (format) {
	case MIPI_DSI_FMT_RGB565:
		return COL_FMT_16BPP;
	case MIPI_DSI_FMT_RGB666:
	case MIPI_DSI_FMT_RGB666_PACKED:
		return COL_FMT_18BPP;
	case MIPI_DSI_FMT_RGB888:
		return COL_FMT_24BPP;
	default:
		return COL_FMT_24BPP; /* for backward compatibility */
	}
};

static int pilatus_panel_prepare(struct drm_panel *panel)
{
	struct pilatus_panel *rad = to_pilatus_panel(panel);
	int ret;

	if (rad->prepared)
		return 0;

	ret = regulator_bulk_enable(rad->num_supplies, rad->supplies);
	if (ret)
		return ret;

	/* At lest 10ms needed between power-on and reset-out as RM specifies */
	usleep_range(10000, 12000);

	if (rad->reset) {
		gpiod_set_value_cansleep(rad->reset, 0);
		/*
		 * 50ms delay after reset-out, as per manufacturer initalization
		 * sequence.
		 */
		msleep(50);
	}

	rad->prepared = true;

	return 0;
}

static int pilatus_panel_unprepare(struct drm_panel *panel)
{
	struct pilatus_panel *rad = to_pilatus_panel(panel);
	int ret;

	if (!rad->prepared)
		return 0;

	/*
	 * Right after asserting the reset, we need to release it, so that the
	 * touch driver can have an active connection with the touch controller
	 * even after the display is turned off.
	 */
	if (rad->reset) {
		gpiod_set_value_cansleep(rad->reset, 1);
		usleep_range(15000, 17000);
		gpiod_set_value_cansleep(rad->reset, 0);
	}

	ret = regulator_bulk_disable(rad->num_supplies, rad->supplies);
	if (ret)
		return ret;

	rad->prepared = false;

	return 0;
}

static int pilatus_real_panel_enable(struct pilatus_panel *panel)
{
	struct mipi_dsi_device *dsi = panel->dsi;
	struct device *dev = &dsi->dev;
	u8 dsi_mode;
	int color_format = color_format_from_dsi_format(dsi->format);
	int ret;

	if (panel->enabled)
		return 0;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

        ret = init_sequence(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to send init sequence (%d)\n", ret);
		goto fail;
	}

	/* Select User Command Set table (CMD1) */
	ret = mipi_dsi_generic_write(dsi, (u8[]){ WRMAUCCTR, 0x00 }, 2);
	if (ret < 0)
		goto fail;

	/* Software reset */
	ret = mipi_dsi_dcs_soft_reset(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to do Software Reset (%d)\n", ret);
		goto fail;
	}

	usleep_range(15000, 17000);

	/* Set DSI mode */
	dsi_mode = (dsi->mode_flags & MIPI_DSI_MODE_VIDEO) ? 0x0B : 0x00;
	ret = mipi_dsi_generic_write(dsi, (u8[]){ 0xC2, dsi_mode }, 2);
	if (ret < 0) {
		dev_err(dev, "Failed to set DSI mode (%d)\n", ret);
		goto fail;
	}
	/* Set tear ON */
	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0) {
		dev_err(dev, "Failed to set tear ON (%d)\n", ret);
		goto fail;
	}
	/* Set tear scanline */
	ret = mipi_dsi_dcs_set_tear_scanline(dsi, 0x380);
	if (ret < 0) {
		dev_err(dev, "Failed to set tear scanline (%d)\n", ret);
		goto fail;
	}
	/* Set pixel format */
	ret = mipi_dsi_dcs_set_pixel_format(dsi, color_format);
	dev_dbg(dev, "Interface color format set to 0x%x\n", color_format);
	if (ret < 0) {
		dev_err(dev, "Failed to set pixel format (%d)\n", ret);
		goto fail;
	}
	/* Exit sleep mode */
	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to exit sleep mode (%d)\n", ret);
		goto fail;
	}

	//usleep_range(5000, 7000);
	msleep(120);
        ret = init_sequence1(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to send init sequence 1 (%d)\n", ret);
		goto fail;
	}

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display ON (%d)\n", ret);
		goto fail;
	}

	usleep_range(5000, 7000);
        ret = init_sequence2(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to send init sequence 2 (%d)\n", ret);
		goto fail;
	}

	backlight_enable(panel->backlight);

	panel->enabled = true;
	dev_info(dev, "Pilatus panel enable end");

	return 0;

fail:
	gpiod_set_value_cansleep(panel->reset, 1);

	return ret;
}


static int pilatus_panel_enable(struct drm_panel *panel)
{
	struct pilatus_panel *rad = to_pilatus_panel(panel);

	return rad->pdata->enable(rad);
}

static int pilatus_panel_disable(struct drm_panel *panel)
{
	struct pilatus_panel *rad = to_pilatus_panel(panel);
	struct mipi_dsi_device *dsi = rad->dsi;
	struct device *dev = &dsi->dev;
	int ret;

	if (!rad->enabled)
		return 0;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	backlight_disable(rad->backlight);

	usleep_range(10000, 12000);

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to set display OFF (%d)\n", ret);
		return ret;
	}

	usleep_range(5000, 10000);

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0) {
		dev_err(dev, "Failed to enter sleep mode (%d)\n", ret);
		return ret;
	}

	rad->enabled = false;

	return 0;
}

static int pilatus_panel_get_modes(struct drm_panel *panel,
			       struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		dev_err(panel->dev, "failed to add mode %ux%u@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	connector->display_info.bus_flags = rad_bus_flags;

	drm_display_info_set_bus_formats(&connector->display_info,
					 rad_bus_formats,
					 ARRAY_SIZE(rad_bus_formats));
	return 1;
}

static int rad_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct pilatus_panel *rad = mipi_dsi_get_drvdata(dsi);
	u16 brightness;
	int ret;

	if (!rad->prepared)
		return 0;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_get_display_brightness(dsi, &brightness);
	if (ret < 0)
		return ret;

	bl->props.brightness = brightness;

	return brightness & 0xff;
}

static int rad_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct pilatus_panel *rad = mipi_dsi_get_drvdata(dsi);
	int ret = 0;

	if (!rad->prepared)
		return 0;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_brightness(dsi, bl->props.brightness);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct backlight_ops rad_bl_ops = {
	.update_status = rad_bl_update_status,
	.get_brightness = rad_bl_get_brightness,
};

static const struct drm_panel_funcs pilatus_panel_funcs = {
	.prepare = pilatus_panel_prepare,
	.unprepare = pilatus_panel_unprepare,
	.enable = pilatus_panel_enable,
	.disable = pilatus_panel_disable,
	.get_modes = pilatus_panel_get_modes,
};


static const struct rad_platform_data rad_pilatus_panel = {
	.enable = &pilatus_real_panel_enable,
};

static const struct of_device_id rad_of_match[] = {
	{ .compatible = "marelcom,pilatus", .data = &rad_pilatus_panel },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, rad_of_match);

static int pilatus_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct of_device_id *of_id = of_match_device(rad_of_match, dev);
	struct device_node *np = dev->of_node;
	struct pilatus_panel *panel;
	struct backlight_properties bl_props;
	int ret;
	u32 video_mode;
		
	dev_info(dev, "Pilatus panel probe 1");

	if (!of_id || !of_id->data)
		return -ENODEV;

	dev_info(dev, "Pilatus panel probe 2");

	panel = devm_kzalloc(&dsi->dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	dev_info(dev, "Pilatus panel probe 3");

	mipi_dsi_set_drvdata(dsi, panel);

	panel->dsi = dsi;
	panel->pdata = of_id->data;

	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_NO_EOT_PACKET;

	ret = of_property_read_u32(np, "video-mode", &video_mode);
	if (!ret) {
		switch (video_mode) {
		case 0:
			/* burst mode */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_BURST |
					   MIPI_DSI_MODE_VIDEO;
			break;
		case 1:
			/* non-burst mode with sync event */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO;
			break;
		case 2:
			/* non-burst mode with sync pulse */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
					   MIPI_DSI_MODE_VIDEO;
			break;
		case 3:
			/* command mode */
			dsi->mode_flags |= MIPI_DSI_CLOCK_NON_CONTINUOUS |
					   MIPI_DSI_MODE_VSYNC_FLUSH;
			break;
		default:
			dev_warn(dev, "invalid video mode %d\n", video_mode);
			break;
		}
	}

	ret = of_property_read_u32(np, "dsi-lanes", &dsi->lanes);
	if (ret) {
		dev_err(dev, "Failed to get dsi-lanes property (%d)\n", ret);
		return ret;
	}

	panel->reset = devm_gpiod_get_optional(dev, "reset",
					       GPIOD_OUT_LOW |
					       GPIOD_FLAGS_BIT_NONEXCLUSIVE);
	if (IS_ERR(panel->reset)) {
		ret = PTR_ERR(panel->reset);
		dev_err(dev, "Failed to get reset gpio (%d)\n", ret);
		return ret;
	}
	gpiod_set_value_cansleep(panel->reset, 1);

	memset(&bl_props, 0, sizeof(bl_props));
	bl_props.type = BACKLIGHT_RAW;
	bl_props.brightness = 255;
	bl_props.max_brightness = 255;

	panel->backlight = devm_backlight_device_register(dev, dev_name(dev),
							  dev, dsi, &rad_bl_ops,
							  &bl_props);
	if (IS_ERR(panel->backlight)) {
		ret = PTR_ERR(panel->backlight);
		dev_err(dev, "Failed to register backlight (%d)\n", ret);
		return ret;
	}

	drm_panel_init(&panel->panel, dev, &pilatus_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	dev_set_drvdata(dev, panel);

	drm_panel_add(&panel->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret)
		drm_panel_remove(&panel->panel);
	else
		dev_info(dev, "Pilatus panel probe end");

	return ret;
}

static int pilatus_panel_remove(struct mipi_dsi_device *dsi)
{
	struct pilatus_panel *rad = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret)
		dev_err(dev, "Failed to detach from host (%d)\n", ret);

	drm_panel_remove(&rad->panel);

	return 0;
}

static void pilatus_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct pilatus_panel *rad = mipi_dsi_get_drvdata(dsi);

	pilatus_panel_disable(&rad->panel);
	pilatus_panel_unprepare(&rad->panel);
}

static struct mipi_dsi_driver pilatus_panel_driver = {
	.driver = {
		.name = "panel-pilatus-marelcom",
		.of_match_table = rad_of_match,
	},
	.probe = pilatus_panel_probe,
	.remove = pilatus_panel_remove,
	.shutdown = pilatus_panel_shutdown,
};
module_mipi_dsi_driver(pilatus_panel_driver);

MODULE_AUTHOR("Robert Chiras <robert.chiras@nxp.com>");
MODULE_DESCRIPTION("DRM Driver for Raydium pilatus_panel MIPI DSI panel");
MODULE_LICENSE("GPL v2");
