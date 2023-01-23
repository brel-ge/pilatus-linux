/*
 * IIO driver for PAC193x series chips
 *
 * Copyright (C) 2022 Microchip Technology Inc.
 *
 * Author: Bogdan Bolocan http://www.microchip.com/support
 * Author: Victor Tudose http://www.microchip.com/support
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA02110-1301, USA.
 *
 */

#include <asm/div64.h>
#include <linux/acpi.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/sysfs.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/timer.h>
#include <linux/util_macros.h>
#define PAC193X_MAX_RFSH_LIMIT 60000
/*(17 * 60 * 1000) //around 17 minutes@1024 sps */
#define PAC193X_MIN_POLLING_TIME 50
/* 50msec is the timeout for validity of the cached registers */

#define SHUNT_UOHMS_DEFAULT 100000

#define PAC193X_VOLTAGE_MILLIVOLTS_MAX 32000
/* 32000mV */
#define PAC193X_VOLTAGE_U_RES 16
/* voltage bits resolution when set for unsigned values */
#define PAC193X_VOLTAGE_S_RES 15
/* voltage bits resolution when set for signed values */

#define PAC193X_VSENSE_MILLIVOLTS_MAX 100
/* 100mV maximum for current shunts */
#define PAC193X_CURRENT_U_RES 16
/* voltage bits resolution when set for unsigned values */
#define PAC193X_CURRENT_S_RES 15
/* voltage bits resolution when set for signed values */

#define PAC193X_POWER_U_RES 28
/* power resolution is 28 bits when unsigned */
#define PAC193X_POWER_S_RES 27
/* power resolution is 27 bits when signed */

#define PAC193X_ENERGY_U_RES 48
/* energy accumulation is 48 bits long */
#define PAC193X_ENERGY_S_RES 47
#define PAC193X_ENERGY_SHIFT_MAIN_VAL 32

#define BIT_INDEX_31 31

#define PAC_193X_MAX_POWER_ACC 0x7fffffffffffffffLL
#define PAC_193X_MIN_POWER_ACC 0x8000000000000000LL

/* maximum value that device can measure - 32 V * 0.1 V */
#define PAC193X_PRODUCT_VOLTAGE_PV_FSR 3200000000000UL
#define PAC193X_PRODUCT_VOLTAGE_NV_FSR 3200000000UL

#define PAC193X_MAX_NUM_CHANNELS 4
#define PAC1931_NUM_CHANNELS 1
#define PAC1932_NUM_CHANNELS 2
#define PAC1933_NUM_CHANNELS 3
#define PAC1934_NUM_CHANNELS 4
#define PAC193X_MEAS_REG_SNAPSHOT_LEN 76
#define PAC193X_CTRL_REG_SNAPSHOT_LEN 12

#define PAC193x_MIN_UPDATE_WAIT_TIME 1000
/*
 * 1000usec is the minimum wait time for normal conversions when sample
 * rate doesn't change
 */
#define PAC193x_DEFAULT_CHIP_SAMP_SPEED 1024

/* I2C address map */
#define PAC193X_REFRESH_REG 0x00
#define PAC193X_CTRL_REG 0x01
#define PAC193X_REFRESH_V_REG 0x1F
#define PAC193X_ACC_COUNT_REG 0x02
#define PAC193X_CTRL_STAT_REGS_ADDR 0x1C
#define PAC193X_PID_REG_ADDR 0xFD

#define PAC193X_ACPI_ARG_COUNT 4
#define PAC193X_ACPI_GET_NAMES_AND_MOHMS_VALS 1
#define PAC193X_ACPI_GET_UOHMS_VALS 2
#define PAC193X_ACPI_GET_BIPOLAR_SETTINGS 4
#define PAC193X_ACPI_GET_SAMP 5

#define PAC193X_VPOWER_ACC_0_ADDR 0x03
#define PAC193X_VPOWER_ACC_1_ADDR 0x04
#define PAC193X_VPOWER_ACC_2_ADDR 0x05
#define PAC193X_VPOWER_ACC_3_ADDR 0x06
#define PAC193X_VBUS_0_ADDR 0x07
#define PAC193X_VBUS_1_ADDR 0x08
#define PAC193X_VBUS_2_ADDR 0x09
#define PAC193X_VBUS_3_ADDR 0x0A
#define PAC193X_VSENSE_0_ADDR 0x0B
#define PAC193X_VSENSE_1_ADDR 0x0C
#define PAC193X_VSENSE_2_ADDR 0x0D
#define PAC193X_VSENSE_3_ADDR 0x0E
#define PAC193X_VBUS_AVG_0_ADDR 0x0F
#define PAC193X_VBUS_AVG_1_ADDR 0x10
#define PAC193X_VBUS_AVG_2_ADDR 0x11
#define PAC193X_VBUS_AVG_3_ADDR 0x12
#define PAC193X_VSENSE_AVG_0_ADDR 0x13
#define PAC193X_VSENSE_AVG_1_ADDR 0x14
#define PAC193X_VSENSE_AVG_2_ADDR 0x15
#define PAC193X_VSENSE_AVG_3_ADDR 0x16
#define PAC193X_VPOWER_0_ADDR 0x17
#define PAC193X_VPOWER_1_ADDR 0x18
#define PAC193X_VPOWER_2_ADDR 0x19
#define PAC193X_VPOWER_3_ADDR 0x1A

#define PAC193X_MAX_REGISTER_LENGTH 6

/*
 * these indexes are exactly describing the element order within a single
 * PAC193x phys channel IIO channel descriptor; see the static const struct
 * iio_chan_spec pac193x_single_channel[] declaration
 */
#define IIO_EN 0
#define IIO_POW 1
#define IIO_VOLT 2
#define IIO_CRT 3
#define IIO_VOLTAVG 4
#define IIO_CRTAVG 5

#define PAC193X_ACC_REG_LEN 3
#define PAC193X_VPOWER_ACC_REG_LEN 6
#define PAC193X_VBUS_SENSE_REG_LEN 2
#define PAC193X_VPOWER_REG_LEN 4

#define PAC193X_CUSTOM_ATTR_FOR_CHANNEL 3

/* 
 * relative offsets when using multi-byte reads/writes even though these
 * bytes are read one after the other, they are not at adjacent memory
 * locations within the I2C memory map. The chip can skip some addresses
 */
#define PAC193X_CHANNEL_DIS_REG_OFF 0
#define PAC193X_NEG_PWR_REG_OFF 1
/*
 * when reading/writing multiple bytes from offset PAC193X_CHANNEL_DIS_REG_OFF,
 * the chip jumps over the 0x1E (REFRESH_G) and 0x1F (REFRESH_V) offsets
 */
#define PAC193X_SLOW_REG_OFF 2
#define PAC193X_CTRL_ACT_REG_OFF 3
#define PAC193X_CHANNEL_DIS_ACT_REG_OFF 4
#define PAC193X_NEG_PWR_ACT_REG_OFF 5
#define PAC193X_CTRL_LAT_REG_OFF 6
#define PAC193X_CHANNEL_DIS_LAT_REG_OFF 7
#define PAC193X_NEG_PWR_LAT_REG_OFF 8
#define PAC193X_PID_REG_OFF 9
#define PAC193X_MID_REG_OFF 10
#define PAC193X_REV_REG_OFF 11
#define PAC193X_CTRL_STATUS_INFO_LEN 12

#define PAC193X_CH_DIS_NOSKIP_VAL 0x02

#define PAC193X_MID 0x5D
#define PAC1934_PID 0x5B
#define PAC1933_PID 0x5A
#define PAC1932_PID 0x59
#define PAC1931_PID 0x58

#define PAC193x_DEV_ATTR(name) (&iio_dev_attr_##name.dev_attr.attr)

#define CTRL_REG(samp, sleep, sing, al_p, al_cc, ovf_al)                       \
	((((u8)samp & 0x03) << 6) | (((u8)sleep & 0x01) << 5) |                \
	 (((u8)sing & 0x01) << 4) | (((u8)al_p & 0x01) << 3) |                 \
	 (((u8)al_cc & 0x01) << 2) | (((u8)ovf_al & 0x01) << 1))

#define CHANNEL_DIS_REG(ch1_on, ch2_on, ch3_on, ch4_on, smb_tout, bycount,     \
			skip)                                                  \
	((ch1_on ? 0 : 0x80) | (ch2_on ? 0 : 0x40) | (ch3_on ? 0 : 0x20) |     \
	 (ch4_on ? 0 : 0x10) | (((u8)smb_tout & 0x01) << 3) |                  \
	 (((u8)bycount & 0x01) << 2) | (skip ? 0 : 0x02))

#define NEG_PWR_REG(ch1_bidi, ch2_bidi, ch3_bidi, ch4_bidi, ch1_bidv,          \
		    ch2_bidv, ch3_bidv, ch4_bidv)                              \
	((ch1_bidi ? 0x80 : 0) | (ch2_bidi ? 0x40 : 0) |                       \
	 (ch3_bidi ? 0x20 : 0) | (ch4_bidi ? 0x10 : 0) |                       \
	 (ch1_bidv ? 0x08 : 0) | (ch2_bidv ? 0x04 : 0) |                       \
	 (ch3_bidv ? 0x02 : 0) | (ch4_bidv ? 0x01 : 0))

enum pac193x_ids { pac1934, pac1933, pac1932, pac1931 };

enum pac193x_samps {
	pac193X_samp_1024sps,
	pac193X_samp_256sps,
	pac193X_samp_64sps,
	pac193X_samp_8sps
};

/**
 * struct pac193x_features - features of a pac193x instance
 * @phys_channels: number of physical channels supported by the chip
 * @prod_id: product ID
 */
struct pac193x_features {
	u8 phys_channels;
	u8 prod_id;
};

struct samp_rate_mapping {
	u16 samp_rate;
	u8 shift2value;
};

static const unsigned int samp_rate_map_tbl[] = {
	[pac193X_samp_1024sps] = 1024,
	[pac193X_samp_256sps] = 256,
	[pac193X_samp_64sps] = 64,
	[pac193X_samp_8sps] = 8,
};

static const struct pac193x_features pac193x_chip_config[] = {
    [pac1934] = {
        .phys_channels = PAC193X_MAX_NUM_CHANNELS,
        .prod_id = PAC1934_PID,
    },
    [pac1933] = {
        .phys_channels = PAC193X_MAX_NUM_CHANNELS - 1,
        .prod_id = PAC1933_PID,
    },
    [pac1932] = {
        .phys_channels = PAC193X_MAX_NUM_CHANNELS - 2,
        .prod_id = PAC1932_PID,
    },
    [pac1931] = {
        .phys_channels = PAC193X_MAX_NUM_CHANNELS - 3,
        .prod_id = PAC1931_PID,
    },
};

/**
 * struct reg_data - data from the registers
 * @active_channels: array of values, true means that channel is active 
 * @bi_dir: array of bools, true means that channel is bidirectional
 * @meas_regs: snapshot of raw measurements registers
 * @ctrl_regs: snapshot of control registers
 * @acc_count: snapshot of the accumulator register
 * @energy_sec_acc: snapshot of energy values
 * @vpower_acc: accumulated vpower values
 * @vpower: snapshot of vpower registers
 * @vbus: snapshot of vbus registers
 * @vbus_avg: averages of vbus registers
 * @vsense: snapshot of vsense registers
 * @vsense_avg: averages of vsense registers
 * @jiffies_tstamp: chip's uptime
 * @crt_samp_spd_bitfield:the current sampling speed
 * @num_enabled_channels: count of how many chip channels are currently enabled
 */
struct reg_data {
	bool active_channels[PAC193X_MAX_NUM_CHANNELS];
	bool bi_dir[PAC193X_MAX_NUM_CHANNELS];
	u8 meas_regs[PAC193X_MEAS_REG_SNAPSHOT_LEN];
	u8 ctrl_regs[PAC193X_CTRL_REG_SNAPSHOT_LEN];
	u32 acc_count;
	s64 energy_sec_acc[PAC193X_MAX_NUM_CHANNELS];
	s64 vpower_acc[PAC193X_MAX_NUM_CHANNELS];
	s32 vpower[PAC193X_MAX_NUM_CHANNELS];
	s32 vbus[PAC193X_MAX_NUM_CHANNELS];
	s32 vbus_avg[PAC193X_MAX_NUM_CHANNELS];
	s32 vsense[PAC193X_MAX_NUM_CHANNELS];
	s32 vsense_avg[PAC193X_MAX_NUM_CHANNELS];
	unsigned long jiffies_tstamp;
	u8 crt_samp_spd_bitfield;
	u8 num_enabled_channels;
};
/**
 * struct pac193x_chip_info - information about the chip
 * @channels: array of values, true means that channel is active 
 * @indio_info: array of bools, true means that channel is bidirectional
 * @client: 
 * @ctrl_regs: snapshot of control values
 * @acc_count: snapshot of the accumulator register
 * @energy_sec_acc: snapshot of energy values
 * @vpower_acc: accumulated vpower values
 * @vpower: snapshot of vpower registers
 * @vbus: snapshot of vbus registers
 * @vbus_avg: averages of vbus registers
 * @vsense: snapshot of vsense registers
 * @vsense_avg: averages of vsense registers
 * @jiffies_tstamp: chip's uptime
 * @crt_samp_spd_bitfield:the current sampling speed
 * @num_enabled_channels: count of how many chip channels are currently enabled
 */
struct pac193x_chip_info {
	const struct iio_chan_spec *channels;
	const struct iio_info *indio_info;
	struct i2c_client *client;
	struct mutex lock;

	struct timer_list tmr_forced_update;
	/* to be used to now when will be the chip read timeout */
	u32 forced_reads_triggered;
	u32 rearm_force_read;

	/* workqueue for periodic chip readings to prevent saturation */
	struct workqueue_struct *wq_chip;
	struct work_struct work_chip_rfsh;

	u8 phys_channels;
	u8 chip_variant;
	u8 chip_revision;

	u32 shunts[PAC193X_MAX_NUM_CHANNELS];
	struct reg_data chip_reg_data;
	unsigned int avg_num;
	u32 sample_rate_value;
	char *channel_names[4];
	struct iio_info pac193x_info;
};

struct __attribute__((__packed__)) pac193x_uuid_format {
	u32 data1;
	u16 data2;
	u16 data3;
	u8 data4[8];
};

#define to_pac193x_chip_info(d)                                                \
	container_of(d, struct pac193x_chip_info, work_chip_rfsh)
/* macros to extract the parameters */
#define mACC_COUNT(addr)                                                       \
	(((u32)(*(u8 *)(addr + 0)) << 16) | ((u32)(*(u8 *)(addr + 1)) << 8) |  \
	 ((u32)(*(u8 *)(addr + 2)) << 0))

#define mVPOWER_ACCu(addr)                                                     \
	(((u64)(*(u8 *)(addr + 0)) << 40) | ((u64)(*(u8 *)(addr + 1)) << 32) | \
	 ((u64)(*(u8 *)(addr + 2)) << 24) | ((u64)(*(u8 *)(addr + 3)) << 16) | \
	 ((u64)(*(u8 *)(addr + 4)) << 8) | ((u64)(*(u8 *)(addr + 5)) << 0))

#define mVPOWER_ACCs(addr) sign_extend64(mVPOWER_ACCu(addr), 47)

#define mVPOWERu(addr)                                                         \
	(((u32)(*(u8 *)(addr + 0)) << 20) | ((u32)(*(u8 *)(addr + 1)) << 12) | \
	 ((u32)(*(u8 *)(addr + 2)) << 4) | ((u32)(*(u8 *)(addr + 3)) >> 4))

#define mVPOWERs(addr) sign_extend32(mVPOWERu(addr), 27)

#define mVBUS_SENSEu(addr)                                                     \
	(((u16)(*(u8 *)(addr + 0)) << 8) | ((u16)(*(u8 *)(addr + 1)) << 0))

#define mVBUS_SENSEs(addr) ((__s16)mVBUS_SENSEu(addr))

static int pac193x_retrieve_data(struct pac193x_chip_info *chip_info,
				 u32 wait_time);
static int pac193x_reg_snapshot(struct pac193x_chip_info *chip_info,
				bool do_rfsh, bool refresh_v, u32 wait_time);
static int pac193x_remove(struct i2c_client *client);
static const char *pac193x_get_of_match_entry(struct i2c_client *client);

#define PAC193x_VPOWER_ACC_CHANNEL(_index, _address)                           \
	{                                                                      \
		.type = IIO_ENERGY, .address = (_address), .indexed = 1,       \
		.channel = (_index),                                           \
		.info_mask_separate = BIT(IIO_CHAN_INFO_AVERAGE_RAW) |         \
				      BIT(IIO_CHAN_INFO_SCALE),                \
		.info_mask_shared_by_dir = BIT(IIO_CHAN_INFO_SAMP_FREQ),       \
		.info_mask_shared_by_all_available =                           \
			BIT(IIO_CHAN_INFO_SAMP_FREQ),                          \
		.scan_index = (_index), .scan_type = {                         \
			.sign = 'u',                                           \
			.realbits = PAC193X_ENERGY_U_RES,                      \
			.storagebits = PAC193X_ENERGY_U_RES,                   \
			.shift = 0,                                            \
			.endianness = IIO_CPU,                                 \
		}                                                              \
	}

#define PAC193x_VBUS_CHANNEL(_index, _address)                                 \
	{                                                                      \
		.type = IIO_VOLTAGE, .address = (_address), .indexed = 1,      \
		.channel = (_index),                                           \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),                  \
		.info_mask_shared_by_dir = BIT(IIO_CHAN_INFO_SAMP_FREQ),       \
		.info_mask_shared_by_all_available =                           \
			BIT(IIO_CHAN_INFO_SAMP_FREQ),                          \
		.scan_index = (_index), .scan_type = {                         \
			.sign = 'u',                                           \
			.realbits = PAC193X_VOLTAGE_U_RES,                     \
			.storagebits = PAC193X_VOLTAGE_U_RES,                  \
			.shift = 0,                                            \
			.endianness = IIO_CPU,                                 \
		}                                                              \
	}

#define PAC193x_VBUS_AVG_CHANNEL(_index, _address)                             \
	{                                                                      \
		.type = IIO_VOLTAGE, .address = (_address), .indexed = 1,      \
		.channel = (_index),                                           \
		.info_mask_separate = BIT(IIO_CHAN_INFO_AVERAGE_RAW) |         \
				      BIT(IIO_CHAN_INFO_SCALE),                \
		.info_mask_shared_by_type =                                    \
			BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),                 \
		.info_mask_shared_by_dir = BIT(IIO_CHAN_INFO_SAMP_FREQ),       \
		.info_mask_shared_by_all_available =                           \
			BIT(IIO_CHAN_INFO_SAMP_FREQ),                          \
		.scan_index = (_index), .scan_type = {                         \
			.sign = 'u',                                           \
			.realbits = PAC193X_VOLTAGE_U_RES,                     \
			.storagebits = PAC193X_VOLTAGE_U_RES,                  \
			.shift = 0,                                            \
			.endianness = IIO_CPU,                                 \
		}                                                              \
	}

#define PAC193x_VSENSE_CHANNEL(_index, _address)                               \
	{                                                                      \
		.type = IIO_CURRENT, .address = (_address), .indexed = 1,      \
		.channel = (_index),                                           \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),                  \
		.info_mask_shared_by_dir = BIT(IIO_CHAN_INFO_SAMP_FREQ),       \
		.info_mask_shared_by_all_available =                           \
			BIT(IIO_CHAN_INFO_SAMP_FREQ),                          \
		.scan_index = (_index), .scan_type = {                         \
			.sign = 'u',                                           \
			.realbits = PAC193X_CURRENT_U_RES,                     \
			.storagebits = PAC193X_CURRENT_U_RES,                  \
			.shift = 0,                                            \
			.endianness = IIO_CPU,                                 \
		}                                                              \
	}

#define PAC193x_VSENSE_AVG_CHANNEL(_index, _address)                           \
	{                                                                      \
		.type = IIO_CURRENT, .address = (_address), .indexed = 1,      \
		.channel = (_index),                                           \
		.info_mask_separate = BIT(IIO_CHAN_INFO_AVERAGE_RAW) |         \
				      BIT(IIO_CHAN_INFO_SCALE),                \
		.info_mask_shared_by_type =                                    \
			BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),                 \
		.info_mask_shared_by_dir = BIT(IIO_CHAN_INFO_SAMP_FREQ),       \
		.info_mask_shared_by_all_available =                           \
			BIT(IIO_CHAN_INFO_SAMP_FREQ),                          \
		.scan_index = (_index), .scan_type = {                         \
			.sign = 'u',                                           \
			.realbits = PAC193X_CURRENT_U_RES,                     \
			.storagebits = PAC193X_CURRENT_U_RES,                  \
			.shift = 0,                                            \
			.endianness = IIO_CPU,                                 \
		}                                                              \
	}

#define PAC193x_VPOWER_CHANNEL(_index, _address)                               \
	{                                                                      \
		.type = IIO_POWER, .address = (_address), .indexed = 1,        \
		.channel = (_index),                                           \
		.info_mask_separate =                                          \
			BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),     \
		.info_mask_shared_by_dir = BIT(IIO_CHAN_INFO_SAMP_FREQ),       \
		.info_mask_shared_by_all_available =                           \
			BIT(IIO_CHAN_INFO_SAMP_FREQ),                          \
		.scan_index = (_index), .scan_type = {                         \
			.sign = 'u',                                           \
			.realbits = PAC193X_POWER_U_RES,                       \
			.storagebits = 32,                                     \
			.shift = 4,                                            \
			.endianness = IIO_CPU,                                 \
		}                                                              \
	}

#define PAC193x_SOFT_TIMESTAMP(_index)                                         \
	{                                                                      \
		.type = IIO_TIMESTAMP, .channel = -1, .scan_index = (_index),  \
	}

static const struct iio_chan_spec pac193x_single_channel[] = {
	PAC193x_VPOWER_ACC_CHANNEL(0, PAC193X_VPOWER_ACC_0_ADDR),
	PAC193x_VPOWER_CHANNEL(0, PAC193X_VPOWER_0_ADDR),
	PAC193x_VBUS_CHANNEL(0, PAC193X_VBUS_0_ADDR),
	PAC193x_VSENSE_CHANNEL(0, PAC193X_VSENSE_0_ADDR),
	PAC193x_VBUS_AVG_CHANNEL(0, PAC193X_VBUS_AVG_0_ADDR),
	PAC193x_VSENSE_AVG_CHANNEL(0, PAC193X_VSENSE_AVG_0_ADDR),
};

static const struct iio_chan_spec pac193x_ts[] = {
	PAC193x_SOFT_TIMESTAMP(0),
};
/* Low-level I2c functions */
static int pac193x_i2c_read(struct i2c_client *client, u8 reg_addr,
			    void *databuf, u8 len)
{
	int ret;
	struct i2c_msg msgs[2] = { { .addr = client->addr,
				     .len = 1,
				     .buf = (u8 *)&reg_addr,
				     .flags = 0 },
				   { .addr = client->addr,
				     .len = len,
				     .buf = databuf,
				     .flags = I2C_M_RD } };

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		dev_err(&client->dev,
			"failed reading data from register 0x%02X\n", reg_addr);
		return ret;
	}
	return 0;
}

static int pac193x_i2c_write_byte(struct i2c_client *client, u8 reg_addr,
				  u8 val)
{
	int ret;
	u8 buf[2];
	struct i2c_msg msg = { .addr = client->addr,
			       .len = sizeof(buf),
			       .buf = (u8 *)&buf,
			       .flags = 0 };
	buf[0] = reg_addr;
	buf[1] = val;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "failed writing register 0x%02X\n",
			reg_addr);
		return ret;
	}
	return 0;
}

static int pac193x_i2c_send_byte(struct i2c_client *client, u8 reg_addr)
{
	int ret;
	u8 buf;
	struct i2c_msg msg = { .addr = client->addr,
			       .len = sizeof(buf),
			       .buf = (u8 *)&buf,
			       .flags = 0 };
	buf = reg_addr;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev,
			"failed sending byte to register 0x%02X\n", reg_addr);
		return ret;
	}
	return 0;
}

static int pac193x_i2c_write(struct i2c_client *client, u8 reg_addr, u8 *data,
			     int len)
{
	int ret;
	u8 send[PAC193X_MAX_REGISTER_LENGTH + 1];
	struct i2c_msg msg = { .addr = client->addr,
			       .len = len + 1,
			       .flags = 0 };

	send[0] = reg_addr;
	memcpy(&send[1], data, len * sizeof(u8));
	msg.buf = send;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		dev_err(&client->dev,
			"failed writing data from register 0x%02X\n", reg_addr);
		return ret;
	}
	return 0;
}

static int pac193x_match_samp_rate(struct pac193x_chip_info *chip_info,
				   u32 new_samp_rate)
{
	int cnt;

	for (cnt = 0; cnt < ARRAY_SIZE(samp_rate_map_tbl); cnt++) {
		if (new_samp_rate == samp_rate_map_tbl[cnt]) {
			chip_info->chip_reg_data.crt_samp_spd_bitfield = cnt;
			break;
		}
	}
	if (cnt == ARRAY_SIZE(samp_rate_map_tbl)) {
		/* not a valid sample rate value */
		return cnt;
	}
	return 0;
}

static ssize_t rst_en_regs_wo_param_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct pac193x_chip_info *chip_info = iio_priv(indio_dev);
	int val;
	int cnt;
	if (kstrtoint(buf, 10, &val))
		return -EINVAL;
	if (!(val == 0 || val == 1))
		return -EINVAL;

	mutex_lock(&chip_info->lock);
	for (cnt = 0; cnt < chip_info->phys_channels; cnt++)
		chip_info->chip_reg_data.energy_sec_acc[cnt] = 0;
	mutex_unlock(&chip_info->lock);
	return count;
}

static ssize_t shunt_value_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct pac193x_chip_info *chip_info = iio_priv(indio_dev);
	int len = 0;
	int target = (int)(attr->attr.name[strlen(attr->attr.name) - 1] - '0');
	len += sprintf(buf, "%d\n", chip_info->shunts[target]);
	return len;
}

static ssize_t channel_name_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct pac193x_chip_info *chip_info = iio_priv(indio_dev);
	int len = 0;
	int target = (int)(attr->attr.name[strlen(attr->attr.name) - 1] - '0');
	len += sprintf(buf, "%s\n", chip_info->channel_names[target]);
	return len;
}

static ssize_t shunt_value_store(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct pac193x_chip_info *chip_info = iio_priv(indio_dev);
	int sh_val;
	int target;

	target = (int)(attr->attr.name[strlen(attr->attr.name) - 1] - '0');
	if (kstrtoint(buf, 10, &sh_val)) {
		dev_err(dev, "%s: Shunt value is not a number\n",
			"shunt_value");
		return -EINVAL;
	}
	if (sh_val < 0) {
		dev_err(dev, "%s: Negative shunt values not allowed\n",
			"shunt_value");
		return -EINVAL;
	}
	mutex_lock(&chip_info->lock);
	chip_info->shunts[target] = sh_val;
	mutex_unlock(&chip_info->lock);
	return count;
}

static int pac193x_read_avail(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *channel,
			      const int **vals, int *type, int *length,
			      long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*type = IIO_VAL_INT;
		*vals = samp_rate_map_tbl;
		*length = ARRAY_SIZE(samp_rate_map_tbl);
		return IIO_AVAIL_LIST;
	}
	return -EINVAL;
}

/*
 * pac193x_read_raw() - data read function.
 * @indio_dev:    the struct iio_dev associated with this device instance
 * @chan:    the channel whose data is to be read
 * @val:    first element of returned value (typically INT)
 * @val2:    second element of returned value (typically MICRO)
 * @mask:    what we actually want to read as per the info_mask_*
 *        in iio_chan_spec.
 */
static int pac193x_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int *val,
			    int *val2, long mask)
{
	struct pac193x_chip_info *chip_info = iio_priv(indio_dev);
	int ret = -EINVAL;
	u64 tmp;
	s64 curr_energy;
	int msb_index = 0; /* first signifiant bit index */
	int shift_basis = 0;

	ret = pac193x_retrieve_data(chip_info, PAC193x_MIN_UPDATE_WAIT_TIME);
	if (ret < 0)
		return ret;
	/* check what data is requested from us */
	ret = -EINVAL;

	switch (mask) {
	/* Raw data requested */
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		/* Voltages */
		case IIO_VOLTAGE:
			switch (chan->address) {
			case PAC193X_VBUS_0_ADDR:
			case PAC193X_VBUS_1_ADDR:
			case PAC193X_VBUS_2_ADDR:
			case PAC193X_VBUS_3_ADDR:
				*val = chip_info->chip_reg_data
					       .vbus[chan->channel];
				return IIO_VAL_INT;

			default:
				return -EINVAL;
			}
			break;
		/* Currents */
		case IIO_CURRENT:
			switch (chan->address) {
			case PAC193X_VSENSE_0_ADDR:
			case PAC193X_VSENSE_1_ADDR:
			case PAC193X_VSENSE_2_ADDR:
			case PAC193X_VSENSE_3_ADDR:
				*val = chip_info->chip_reg_data
					       .vsense[chan->channel];
				return IIO_VAL_INT;
			default:
				return -EINVAL;
			}
			break;
		/* Power */
		case IIO_POWER:
			switch (chan->address) {
			case PAC193X_VPOWER_0_ADDR:
			case PAC193X_VPOWER_1_ADDR:
			case PAC193X_VPOWER_2_ADDR:
			case PAC193X_VPOWER_3_ADDR:
				*val = chip_info->chip_reg_data
					       .vpower[chan->channel];
				return IIO_VAL_INT;
			default:
				return -EINVAL;
			}
			break;
		default:
			return -EINVAL;
		}
		break;
	/* Average raw data */
	case IIO_CHAN_INFO_AVERAGE_RAW:
		switch (chan->type) {
		/* Voltages */
		case IIO_VOLTAGE:
			switch (chan->address) {
			case PAC193X_VBUS_AVG_0_ADDR:
			case PAC193X_VBUS_AVG_1_ADDR:
			case PAC193X_VBUS_AVG_2_ADDR:
			case PAC193X_VBUS_AVG_3_ADDR:
				*val = chip_info->chip_reg_data
					       .vbus_avg[chan->channel];
				return IIO_VAL_INT;

			default:
				return -EINVAL;
			}
			break;
		/* Currents */
		case IIO_CURRENT:
			switch (chan->address) {
			case PAC193X_VSENSE_AVG_0_ADDR:
			case PAC193X_VSENSE_AVG_1_ADDR:
			case PAC193X_VSENSE_AVG_2_ADDR:
			case PAC193X_VSENSE_AVG_3_ADDR:
				*val = chip_info->chip_reg_data
					       .vsense_avg[chan->channel];
				return IIO_VAL_INT;

			default:
				return -EINVAL;
			}
			break;
		/* Energy */
		case IIO_ENERGY:
			switch (chan->address) {
			case PAC193X_VPOWER_ACC_0_ADDR:
			case PAC193X_VPOWER_ACC_1_ADDR:
			case PAC193X_VPOWER_ACC_2_ADDR:
			case PAC193X_VPOWER_ACC_3_ADDR:
				/*
     * expresses the 64 bit energy value as a 32 bit value and a 32 bit scale
     * here reports the 32 bit energy value
     */
				curr_energy =
					chip_info->chip_reg_data
						.energy_sec_acc[chan->channel];
				if (curr_energy < 0)
					curr_energy = ~curr_energy;
				msb_index = fls64(curr_energy);
				if (msb_index > BIT_INDEX_31)
					/* right-shifts the value in order to put first signifiant bit on the 31st position */
					*val = chip_info->chip_reg_data
						       .energy_sec_acc
							       [chan->channel] >>
					       (msb_index - BIT_INDEX_31);
				else
					*val = chip_info->chip_reg_data
						       .energy_sec_acc
							       [chan->channel];
				return IIO_VAL_INT;
			default:
				return -EINVAL;
			}
			break;
		default:
			return -EINVAL;
		}
		break;
	case IIO_CHAN_INFO_SCALE:
		switch (chan->address) {
		/* Voltages - scale for millivolts */
		case PAC193X_VBUS_0_ADDR:
		case PAC193X_VBUS_1_ADDR:
		case PAC193X_VBUS_2_ADDR:
		case PAC193X_VBUS_3_ADDR:
		case PAC193X_VBUS_AVG_0_ADDR:
		case PAC193X_VBUS_AVG_1_ADDR:
		case PAC193X_VBUS_AVG_2_ADDR:
		case PAC193X_VBUS_AVG_3_ADDR:
			*val = PAC193X_VOLTAGE_MILLIVOLTS_MAX;
			if (chan->scan_type.sign == 'u')
				*val2 = PAC193X_VOLTAGE_U_RES;
			else
				*val2 = PAC193X_VOLTAGE_S_RES;
			return IIO_VAL_FRACTIONAL_LOG2;
		/*
         * Currents - scale for mA - depends on the
         * channel's shunt value
         * (100mV * 1000000) / (2^16 * shunt(uohm))
         */
		case PAC193X_VSENSE_0_ADDR:
		case PAC193X_VSENSE_1_ADDR:
		case PAC193X_VSENSE_2_ADDR:
		case PAC193X_VSENSE_3_ADDR:
		case PAC193X_VSENSE_AVG_0_ADDR:
		case PAC193X_VSENSE_AVG_1_ADDR:
		case PAC193X_VSENSE_AVG_2_ADDR:
		case PAC193X_VSENSE_AVG_3_ADDR:
			tmp = (PAC193X_VSENSE_MILLIVOLTS_MAX * 1000000) /
			      chip_info->shunts[chan->channel];
			*val = (int)tmp;
			if (chan->scan_type.sign == 'u')
				*val2 = PAC193X_CURRENT_U_RES;
			else
				*val2 = PAC193X_CURRENT_S_RES;
			return IIO_VAL_FRACTIONAL_LOG2;
		/*
         * Power - mW - it will use the combined scale
         * for current and voltage
         * current(mA) * voltage(mV) = power (uW)
         */
		case PAC193X_VPOWER_0_ADDR:
		case PAC193X_VPOWER_1_ADDR:
		case PAC193X_VPOWER_2_ADDR:
		case PAC193X_VPOWER_3_ADDR:
			tmp = PAC193X_PRODUCT_VOLTAGE_PV_FSR;
			do_div(tmp, chip_info->shunts[chan->channel]);
			*val = (int)tmp;
			if (chan->scan_type.sign == 'u')
				*val2 = PAC193X_POWER_U_RES;
			else
				*val2 = PAC193X_POWER_S_RES;
			return IIO_VAL_FRACTIONAL_LOG2;
		case PAC193X_VPOWER_ACC_0_ADDR:
		case PAC193X_VPOWER_ACC_1_ADDR:
		case PAC193X_VPOWER_ACC_2_ADDR:
		case PAC193X_VPOWER_ACC_3_ADDR:
			/*
         * expresses the 64 bit energy value as a 32 bit value and a 32 bit scale
         * here compute the scale for energy (Watt-second or Joule)
         * scale is adjusted with 2^(energy value shift)
         */
			tmp = PAC193X_PRODUCT_VOLTAGE_NV_FSR;
			do_div(tmp, chip_info->shunts[chan->channel]);
			curr_energy = chip_info->chip_reg_data
					      .energy_sec_acc[chan->channel];
			if (curr_energy < 0)
				curr_energy = ~curr_energy;
			msb_index = fls64(curr_energy);
			if (chan->scan_type.sign == 'u')
				shift_basis = PAC193X_POWER_U_RES;
			else
				shift_basis = PAC193X_POWER_S_RES;

			if (msb_index > BIT_INDEX_31) {
				int diff = shift_basis -
					   (msb_index - BIT_INDEX_31);

				if (diff < 0) {
					tmp <<= (-diff);
					*val = (int)tmp;
					return IIO_VAL_INT;
				}
				*val = (int)tmp;
				*val2 = diff;
				return IIO_VAL_FRACTIONAL_LOG2;
			}
			*val = (int)tmp;
			*val2 = shift_basis;
			return IIO_VAL_FRACTIONAL_LOG2;
		default:
			return -EINVAL;
		}
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = (int)chip_info->sample_rate_value;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
	return ret;
}
/*
 * pac193x_write_raw() - data write function.
 * @indio_dev:    the struct iio_dev associated with this device instance
 * @chan:    the channel whose data is to be written
 * @val:    first element of value to set (typically INT)
 * @val2:    second element of value to set (typically MICRO)
 * @mask:    what we actually want to write as per the info_mask_*
 *        in iio_chan_spec.
 *
 * Note that all raw writes are assumed IIO_VAL_INT and info mask elements
 * are assumed to be IIO_INT_PLUS_MICRO unless the callback write_raw_get_fmt
 * in struct iio_info is provided by the driver.
 */
static int pac193x_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int val,
			     int val2, long mask)
{
	struct pac193x_chip_info *chip_info = iio_priv(indio_dev);
	struct i2c_client *client = chip_info->client;
	int ret = -EINVAL;
	u32 old_samp_rate;

	if (iio_buffer_enabled(indio_dev))
		return -EBUSY;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (pac193x_match_samp_rate(chip_info, (u16)val))
			return -EINVAL;
		/* store the old sampling rate */
		old_samp_rate = chip_info->sample_rate_value;
		/* we have a valid sample rate */
		chip_info->sample_rate_value = (u16)val;
		/*
         * now lock the access to the chip, write the new
         * sampling value and trigger a snapshot(incl refresh)
         */
		mutex_lock(&chip_info->lock);
		/* enable ALERT pin */
		ret = pac193x_i2c_write_byte(
			chip_info->client, PAC193X_CTRL_REG,
			CTRL_REG(chip_info->chip_reg_data.crt_samp_spd_bitfield,
				 0, 0, 1, 0, 0));
		if (ret < 0) {
			dev_err(&client->dev,
				"%s - cannot write PAC193x ctrl reg at 0x%02X\n",
				__func__, PAC193X_CTRL_REG);
			chip_info->sample_rate_value = old_samp_rate;
			mutex_unlock(&chip_info->lock);
			return ret;
		}
		/*
         * unlock the access towards the chip - register
         * snapshot includes its own access lock
         */
		mutex_unlock(&chip_info->lock);
		/*
         * now, force a snapshot with refresh - call retrieve
         * data in order to update the refresh timer
         * alter the timestamp in order to force trigger a
         * register snapshot and a timestamp update
         */
		chip_info->chip_reg_data.jiffies_tstamp -=
			msecs_to_jiffies(PAC193X_MIN_POLLING_TIME);
		ret = pac193x_retrieve_data(chip_info,
					    (1024 / old_samp_rate) * 1000);
		if (ret < 0) {
			dev_err(&client->dev,
				"%s - cannot snapshot PAC193x ctrl and measurement regs\n",
				__func__);
			return ret;
		}
		ret = 0;
		break;
	}
	return ret;
}

static int pac193x_send_refresh(struct pac193x_chip_info *chip_info,
				bool refresh_v, u32 wait_time)
{
	/* this function only sends REFRESH or REFRESH_V */
	struct i2c_client *client = chip_info->client;
	int ret;
	u8 refresh_cmd;
	u8 bidir_reg;
	bool revision_bug = false;

	if ((chip_info->chip_revision == 2) ||
	    (chip_info->chip_revision == 3)) {
		/*rev 2 and 3 bug workaround*/
		revision_bug = true;

		bidir_reg = NEG_PWR_REG(chip_info->chip_reg_data.bi_dir[0],
					chip_info->chip_reg_data.bi_dir[1],
					chip_info->chip_reg_data.bi_dir[2],
					chip_info->chip_reg_data.bi_dir[3],
					chip_info->chip_reg_data.bi_dir[0],
					chip_info->chip_reg_data.bi_dir[1],
					chip_info->chip_reg_data.bi_dir[2],
					chip_info->chip_reg_data.bi_dir[3]);
		/* write the updated registers back */
		ret = pac193x_i2c_write_byte(chip_info->client,
					     PAC193X_CTRL_STAT_REGS_ADDR +
						     PAC193X_NEG_PWR_REG_OFF,
					     bidir_reg);
	}

	/*
     * if refresh_v is not false, send a REFRESH_V instead
     * (doesn't reset the accumulators)
     */
	refresh_cmd = PAC193X_REFRESH_REG;
	if (refresh_v)
		refresh_cmd = PAC193X_REFRESH_V_REG;
	/* now write a REFRESH or a REFRESH_V command */
	ret = pac193x_i2c_send_byte(chip_info->client, refresh_cmd);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s - cannot send byte to PAC193x 0x%02X reg\n",
			__func__, refresh_cmd);
		return ret;
	}
	if (revision_bug) {
		/*rev 3 bug workaround - write again the same register */
		/* write the updated registers back */
		ret = pac193x_i2c_write_byte(chip_info->client,
					     PAC193X_CTRL_STAT_REGS_ADDR +
						     PAC193X_NEG_PWR_REG_OFF,
					     bidir_reg);
	}
	/* register data retrieval timestamp */
	chip_info->chip_reg_data.jiffies_tstamp = jiffies;
	/* wait till the data is available */
	usleep_range(wait_time, wait_time + 100);
	return ret;
}

static int pac193x_reg_snapshot(struct pac193x_chip_info *chip_info,
				bool do_refresh, bool refresh_v, u32 wait_time)
{
	int ret;
	struct i2c_client *client = chip_info->client;
	u8 offset_reg_data, samp_shift;
	int cnt;
	s64 curr_energy;
	s64 inc;

	/* protect the access to the chip */
	mutex_lock(&chip_info->lock);

	if (do_refresh) {
		ret = pac193x_send_refresh(chip_info, refresh_v, wait_time);
		if (ret < 0) {
			dev_err(&client->dev,
				"%s - cannot send refresh towards PAC193x\n",
				__func__);
			goto reg_snapshot_err;
		}
	}
	/* read the ctrl/status registers for this snapshot */
	ret = pac193x_i2c_read(chip_info->client, PAC193X_CTRL_STAT_REGS_ADDR,
			       (u8 *)chip_info->chip_reg_data.ctrl_regs,
			       PAC193X_CTRL_REG_SNAPSHOT_LEN);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s - cannot read PAC193x regs from 0x%02X\n", __func__,
			PAC193X_CTRL_STAT_REGS_ADDR);
		goto reg_snapshot_err;
	}
	/* read the data registers */
	ret = pac193x_i2c_read(chip_info->client, PAC193X_ACC_COUNT_REG,
			       (u8 *)chip_info->chip_reg_data.meas_regs,
			       PAC193X_MEAS_REG_SNAPSHOT_LEN);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s - cannot read PAC193x regs from 0x%02X\n", __func__,
			PAC193X_ACC_COUNT_REG);
		goto reg_snapshot_err;
	}
	offset_reg_data = 0;
	chip_info->chip_reg_data.acc_count = mACC_COUNT(
		&chip_info->chip_reg_data.meas_regs[offset_reg_data]);
	/* move the register offset */
	offset_reg_data += PAC193X_ACC_REG_LEN;
	/* start with VPOWER_ACC */
	for (cnt = 0; cnt < chip_info->phys_channels; cnt++) {
		/* check if the channel is active(within the data read from
         * the chip), skip all fields if disabled
         */
		if (((chip_info->chip_reg_data
			      .ctrl_regs[PAC193X_CHANNEL_DIS_LAT_REG_OFF]
		      << cnt) &
		     0x80) == 0) {
			/* add the power_acc field */
			if (chip_info->chip_reg_data.bi_dir[cnt]) {
				/* bi-directional channel */
				chip_info->chip_reg_data
					.vpower_acc[cnt] = mVPOWER_ACCs(
					&chip_info->chip_reg_data
						 .meas_regs[offset_reg_data]);
			} else {
				/* uni-directional channel */
				chip_info->chip_reg_data
					.vpower_acc[cnt] = mVPOWER_ACCu(
					&chip_info->chip_reg_data
						 .meas_regs[offset_reg_data]);
			}
			offset_reg_data += PAC193X_VPOWER_ACC_REG_LEN;
			/* now compute the scaled to 1 second
             * accumulated energy value; see how much shift
             * is required by the sample rate
             */
			samp_shift = get_count_order(samp_rate_map_tbl[(
				(chip_info->chip_reg_data
					 .ctrl_regs[PAC193X_CTRL_LAT_REG_OFF]) >>
				6)]);
			/* energy accumulator scaled to 1sec = VPOWER_ACC/2^samp_shift */
			/* the chip's sampling rate is 2^samp_shift samples/sec */
			curr_energy =
				chip_info->chip_reg_data.energy_sec_acc[cnt];
			inc = (chip_info->chip_reg_data.vpower_acc[cnt] >>
			       samp_shift);
			if (curr_energy >= 0) {
				if (inc >= (((s64)PAC_193X_MAX_POWER_ACC) -
					    curr_energy))
					curr_energy = PAC_193X_MAX_POWER_ACC;
				else
					curr_energy += inc;
			} else {
				if (inc < (((s64)PAC_193X_MIN_POWER_ACC) -
					   curr_energy))
					curr_energy = PAC_193X_MIN_POWER_ACC;
				else
					curr_energy += inc;
			}
			chip_info->chip_reg_data.energy_sec_acc[cnt] =
				curr_energy;
		}
	}
	/* continue with VBUS */
	for (cnt = 0; cnt < chip_info->phys_channels; cnt++) {
		/* check if the channel is active, skip all fields if disabled */
		if (((chip_info->chip_reg_data
			      .ctrl_regs[PAC193X_CHANNEL_DIS_LAT_REG_OFF]
		      << cnt) &
		     0x80) == 0) {
			/* read the VBUS channels */
			chip_info->chip_reg_data.vbus[cnt] = mVBUS_SENSEu(
				&chip_info->chip_reg_data
					 .meas_regs[offset_reg_data]);
			offset_reg_data += PAC193X_VBUS_SENSE_REG_LEN;
		} else
			printk("this is dissabled\n");
	}
	/* VSENSE */
	for (cnt = 0; cnt < chip_info->phys_channels; cnt++) {
		/* check if the channel is active, skip all fields if disabled */
		if (((chip_info->chip_reg_data
			      .ctrl_regs[PAC193X_CHANNEL_DIS_LAT_REG_OFF]
		      << cnt) &
		     0x80) == 0) {
			/* read the VSENSE registers */
			if (chip_info->chip_reg_data.bi_dir[cnt]) {
				/* bi-directional channel */
				chip_info->chip_reg_data
					.vsense[cnt] = mVBUS_SENSEs(
					&chip_info->chip_reg_data
						 .meas_regs[offset_reg_data]);
			} else {
				/* uni-directional channel */
				chip_info->chip_reg_data
					.vsense[cnt] = mVBUS_SENSEu(
					&chip_info->chip_reg_data
						 .meas_regs[offset_reg_data]);
			}
			offset_reg_data += PAC193X_VBUS_SENSE_REG_LEN;
		}
		// TODO same, and for the ones that follow
	}
	/* VBUS_AVG */
	for (cnt = 0; cnt < chip_info->phys_channels; cnt++) {
		/* check if the channel is active, skip all fields if disabled */
		if (((chip_info->chip_reg_data
			      .ctrl_regs[PAC193X_CHANNEL_DIS_LAT_REG_OFF]
		      << cnt) &
		     0x80) == 0) {
			/* read the VBUS_AVG registers */
			chip_info->chip_reg_data.vbus_avg[cnt] = mVBUS_SENSEu(
				&chip_info->chip_reg_data
					 .meas_regs[offset_reg_data]);
			offset_reg_data += PAC193X_VBUS_SENSE_REG_LEN;
		}
	}
	/* VSENSE_AVG */
	for (cnt = 0; cnt < chip_info->phys_channels; cnt++) {
		/* check if the channel is active, skip all fields if disabled */
		if (((chip_info->chip_reg_data
			      .ctrl_regs[PAC193X_CHANNEL_DIS_LAT_REG_OFF]
		      << cnt) &
		     0x80) == 0) {
			/* read the VSENSE_AVG registers */
			if (chip_info->chip_reg_data.bi_dir[cnt]) {
				/* bi-directional channel */
				chip_info->chip_reg_data
					.vsense_avg[cnt] = mVBUS_SENSEs(
					&chip_info->chip_reg_data
						 .meas_regs[offset_reg_data]);
			} else {
				/* uni-directional channel */
				chip_info->chip_reg_data
					.vsense_avg[cnt] = mVBUS_SENSEu(
					&chip_info->chip_reg_data
						 .meas_regs[offset_reg_data]);
			}
			offset_reg_data += PAC193X_VBUS_SENSE_REG_LEN;
		}
	}
	/* VPOWER */
	for (cnt = 0; cnt < chip_info->phys_channels; cnt++) {
		/* check if the channel is active, skip all fields if disabled */
		if (((chip_info->chip_reg_data
			      .ctrl_regs[PAC193X_CHANNEL_DIS_LAT_REG_OFF]
		      << cnt) &
		     0x80) == 0) {
			/* read the VPOWER fields */
			if (chip_info->chip_reg_data.bi_dir[cnt]) {
				/* bi-directional channel */
				chip_info->chip_reg_data.vpower[cnt] = mVPOWERs(
					&chip_info->chip_reg_data
						 .meas_regs[offset_reg_data]);
			} else {
				/* uni-directional channel */
				chip_info->chip_reg_data.vpower[cnt] = mVPOWERu(
					&chip_info->chip_reg_data
						 .meas_regs[offset_reg_data]);
			}
			offset_reg_data += PAC193X_VPOWER_REG_LEN;
		}
	}
reg_snapshot_err:
	mutex_unlock(&chip_info->lock);
	return ret;
}

static void pac193x_work_periodic_rfsh(struct work_struct *work)
{
	struct pac193x_chip_info *chip_info = to_pac193x_chip_info(work);
	/* do a REFRESH, then read */
	pac193x_reg_snapshot(chip_info, true, false,
			     PAC193x_MIN_UPDATE_WAIT_TIME);
}

static void pac193x_read_reg_timeout(struct timer_list *t)
{
	int ret;
	struct pac193x_chip_info *chip_info =
		from_timer(chip_info, t, tmr_forced_update);
	struct i2c_client *client = chip_info->client;

	ret = mod_timer(&chip_info->tmr_forced_update,
			jiffies + msecs_to_jiffies(PAC193X_MAX_RFSH_LIMIT));
	if (ret < 0)
		dev_err(&client->dev,
			"forced read timer cannot be modified!\n");
	/* schedule the periodic reading from the chip */
	queue_work(chip_info->wq_chip, &chip_info->work_chip_rfsh);
}

static int pac193x_chip_identify(struct pac193x_chip_info *chip_info)
{
	int ret = 0;
	struct i2c_client *client = chip_info->client;
	u8 chip_rev_info[3];
	/*try to identify the chip variant
     * read the chip ID values
     */
	ret = pac193x_i2c_read(chip_info->client, PAC193X_PID_REG_ADDR,
			       (u8 *)chip_rev_info, 3);
	if (ret < 0) {
		dev_err(&client->dev, "cannot read PAC193x revision\n");
		goto chip_identify_err;
	}
	if (chip_rev_info[0] !=
	    pac193x_chip_config[chip_info->chip_variant].prod_id) {
		ret = -EINVAL;
		dev_err(&client->dev,
			"chip's product ID doesn't match the exact one for this part\n");
		goto chip_identify_err;
	}
	dev_info(&client->dev, "Chip revision: 0x%02X\n", chip_rev_info[2]);
	chip_info->chip_revision = chip_rev_info[2];
chip_identify_err:
	return ret;
}

static u8 *pac193x_read_revision(struct pac193x_chip_info *chip_info)
{
	int ret = 0;
	struct i2c_client *client = chip_info->client;
	u8 *chip_rev_info = kmalloc(3 * sizeof(u8), GFP_KERNEL);
	/*
     * try to identify the chip variant
     * read the chip ID values
     */
	ret = pac193x_i2c_read(chip_info->client, PAC193X_PID_REG_ADDR,
			       (u8 *)chip_rev_info, 3);
	if (ret < 0) {
		dev_err(&client->dev, "cannot read PAC193x revision\n");
		return NULL;
	}
	return chip_rev_info;
}

static int pac193x_get_revision(struct pac193x_chip_info *chip_info)
{
	u8 *chip_rev_info = pac193x_read_revision(chip_info);
	if (chip_rev_info != NULL) {
		chip_info->chip_variant = chip_rev_info[0];
		chip_info->chip_revision = chip_rev_info[2];
		switch (chip_rev_info[0]) {
		case PAC1934_PID:
			chip_info->phys_channels = PAC1934_NUM_CHANNELS;
			break;
		case PAC1933_PID:
			chip_info->phys_channels = PAC1933_NUM_CHANNELS;
			break;
		case PAC1932_PID:
			chip_info->phys_channels = PAC1932_NUM_CHANNELS;
			break;
		case PAC1931_PID:
			chip_info->phys_channels = PAC1931_NUM_CHANNELS;
			break;
		default:
			return -EINVAL;
		}
	}
	kfree(chip_rev_info);
	return 0;
}

static int pac193x_setup_periodic_refresh(struct pac193x_chip_info *chip_info)
{
	int ret = 0;

	chip_info->wq_chip = create_workqueue("wq_pac193x");
	INIT_WORK(&chip_info->work_chip_rfsh, pac193x_work_periodic_rfsh);

	/* setup the latest moment for reading the regs before saturation */
	timer_setup(&chip_info->tmr_forced_update, pac193x_read_reg_timeout, 0);
	/* register the timer */
	mod_timer(&chip_info->tmr_forced_update,
		  jiffies + msecs_to_jiffies(PAC193X_MAX_RFSH_LIMIT));

	return ret;
}

static union acpi_object *pac193x_acpi_eval_function(acpi_handle handle,
						     int revision, int function)
{
	acpi_status status;
	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	union acpi_object args[PAC193X_ACPI_ARG_COUNT];
	struct acpi_object_list args_list;
	struct pac193x_uuid_format *uuid;

	uuid = kmalloc(sizeof(struct pac193x_uuid_format), GFP_KERNEL);

	uuid->data1 = 0x33771e0;
	uuid->data2 = 0x1705;
	uuid->data3 = 0x47b4;

	uuid->data4[0] = 0x95;
	uuid->data4[1] = 0x35;
	uuid->data4[2] = 0xd1;
	uuid->data4[3] = 0xbb;
	uuid->data4[4] = 0xe1;
	uuid->data4[5] = 0x4d;
	uuid->data4[6] = 0x9a;
	uuid->data4[7] = 0x09;

	args[0].type = ACPI_TYPE_BUFFER;
	args[0].buffer.length = sizeof(struct pac193x_uuid_format);
	args[0].buffer.pointer = (void *)uuid;

	args[1].type = ACPI_TYPE_INTEGER;
	args[1].integer.value = revision;

	args[2].type = ACPI_TYPE_INTEGER;
	args[2].integer.value = function;

	args[3].type = ACPI_TYPE_PACKAGE;
	args[3].package.count = 0;

	args_list.count = PAC193X_ACPI_ARG_COUNT;
	args_list.pointer = &args[0];

	status = acpi_evaluate_object(handle, "_DSM", &args_list, &buffer);
	kfree(uuid);
	if (ACPI_FAILURE(status)) {
		kfree(buffer.pointer);
		return NULL;
	}
	return buffer.pointer;
}

static char *pac193x_acpi_get_acpi_match_entry(acpi_handle handle)
{
	acpi_status status;
	union acpi_object *name_object;

	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	status = acpi_evaluate_object(handle, "_HID", NULL, &buffer);
	name_object = buffer.pointer;
	return name_object->string.pointer;
}

static const char *
pac193x_match_acpi_device(struct i2c_client *client,
			  struct pac193x_chip_info *chip_info)
{
	char *ptr_name;
	acpi_handle handle;
	union acpi_object *rez;
	unsigned short bi_dir_mask;
	int channel_index;
	int i;
	handle = ACPI_HANDLE(&client->dev);
	ptr_name = pac193x_acpi_get_acpi_match_entry(handle);
	if (ptr_name == NULL)
		return NULL;
	rez = pac193x_acpi_eval_function(handle, 0,
					 PAC193X_ACPI_GET_NAMES_AND_MOHMS_VALS);
	if (rez == NULL)
		return NULL;

	// the received results has the following format
	// channel_name_0 , channel_value_0
	// channel_name_1 , channel_value_1
	// channel_name_2 , channel_value_2
	// channel_name_3 , channel_value_3
	for (i = 0; i < rez->package.count; i += 2) {
		channel_index = i / 2;
		chip_info->channel_names[channel_index] = kmemdup(
			rez->package.elements[i].string.pointer,
			(size_t)rez->package.elements[i].string.length + 1,
			GFP_KERNEL);
		chip_info->channel_names[channel_index]
					[rez->package.elements[i].string.length] =
			'\0';
		chip_info->shunts[channel_index] =
			rez->package.elements[i + 1].integer.value * 1000;
		chip_info->chip_reg_data.active_channels[channel_index] =
			(chip_info->shunts[channel_index] != 0);
	}

	kfree(rez);
	rez = pac193x_acpi_eval_function(handle, 1,
					 PAC193X_ACPI_GET_UOHMS_VALS);
	if (rez == NULL) {
		// initialisng with default values
		// we assume all channels are unidectional(the mask is zero)
		// and assign the default sampling rate
		chip_info->sample_rate_value = PAC193x_DEFAULT_CHIP_SAMP_SPEED;
		return ptr_name;
	}
	for (i = 0; i < rez->package.count; i++) {
		channel_index = i;
		chip_info->shunts[channel_index] =
			rez->package.elements[i].integer.value;
		chip_info->chip_reg_data.active_channels[channel_index] =
			(chip_info->shunts[channel_index] != 0);
	}
	kfree(rez);
	rez = pac193x_acpi_eval_function(handle, 1,
					 PAC193X_ACPI_GET_BIPOLAR_SETTINGS);
	if (rez == NULL)
		return NULL;
	bi_dir_mask = rez->package.elements[0].integer.value;
	chip_info->chip_reg_data.bi_dir[0] = (bi_dir_mask & (1 << 0)) << 0;
	chip_info->chip_reg_data.bi_dir[0] = (bi_dir_mask & (1 << 1)) << 1;
	chip_info->chip_reg_data.bi_dir[0] = (bi_dir_mask & (1 << 2)) << 2;
	chip_info->chip_reg_data.bi_dir[0] = (bi_dir_mask & (1 << 3)) << 3;
	kfree(rez);
	rez = pac193x_acpi_eval_function(handle, 1, PAC193X_ACPI_GET_SAMP);
	if (rez == NULL)
		return NULL;
	chip_info->sample_rate_value = rez->package.elements[0].integer.value;
	kfree(rez);
	return ptr_name;
}

static const char *pac193x_match_of_device(struct i2c_client *client,
					   struct pac193x_chip_info *chip_info)
{
	struct device_node *node;
	unsigned int current_channel;
	const char *ptr_name;
	int channel_index;

	ptr_name = pac193x_get_of_match_entry(client);

	if (of_property_read_u32(client->dev.of_node, "microchip,samp-rate",
				 &chip_info->sample_rate_value)) {
		dev_err(&client->dev, "Cannot read sample rate value ...\n");
		return NULL;
	}
	if (pac193x_match_samp_rate(chip_info, chip_info->sample_rate_value)) {
		dev_err(&client->dev,
			"The given sample rate value is not supported: %d\n",
			chip_info->sample_rate_value);
		return NULL;
	}
	current_channel = 0;
	for_each_child_of_node (client->dev.of_node, node) {
		if (of_property_read_u32(node, "microchip,channel-index",
					 &channel_index)) {
			dev_err(&client->dev,
				"invalid channel_index %d value on %s\n",
				channel_index, node->full_name);
			return NULL;
		}
		channel_index--;
		if (current_channel >= chip_info->phys_channels ||
		    channel_index >= chip_info->phys_channels ||
		    channel_index < 0) {
			dev_err(&client->dev,
				"invalid channel_index %d value on %s\n",
				channel_index, node->full_name);

			return NULL;
		}
		/* check if the channel is enabled or not */
		chip_info->chip_reg_data.active_channels[channel_index] =
			of_property_read_bool(node,
					      "microchip,channel-enabled");
		if (!chip_info->chip_reg_data.active_channels[channel_index]) {
			/* set the chunt value to 0 for the disabled channels */
			chip_info->shunts[channel_index] = 0;
			current_channel++;
			continue;
		}
		if (of_property_read_u32(node, "microchip,uohms-shunt-res",
					 &chip_info->shunts[channel_index])) {
			dev_err(&client->dev,
				"invalid shunt-resistor value on %s\n",
				node->full_name);
			return NULL;
		}
		if (of_property_read_string(
			    node, "microchip,rail-name",
			    (const char **)&chip_info
				    ->channel_names[channel_index])) {
			dev_err(&client->dev, "invalid rail-name value on %s\n",
				node->full_name);
			return NULL;
		}
		chip_info->chip_reg_data.bi_dir[channel_index] =
			of_property_read_bool(node, "microchip,bi-directional");

		current_channel++;
	}
	return ptr_name;
}

static int pac193x_chip_configure(struct pac193x_chip_info *chip_info)
{
	int cnt, ret = 0;
	struct i2c_client *client = chip_info->client;
	u8 regs[PAC193X_CTRL_STATUS_INFO_LEN];
	u32 wait_time;

	/*
     * count how many channels are enabled and store
     * this information within the driver data
     */
	cnt = 0;
	chip_info->chip_reg_data.num_enabled_channels = 0;
	while (cnt < chip_info->phys_channels) {
		if (chip_info->chip_reg_data.active_channels[cnt])
			chip_info->chip_reg_data.num_enabled_channels++;
		cnt++;
	}
	/*
     * read whatever information was gathered before the driver was loaded
     * establish which channels are enabled/disabled and then establish the
     * information retrieval mode (using SKIP or no).
     * Read the chip ID values
     */
	ret = pac193x_i2c_read(chip_info->client, PAC193X_CTRL_STAT_REGS_ADDR,
			       (u8 *)regs, PAC193X_CTRL_STATUS_INFO_LEN);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s - cannot read PAC193x regs from 0x%02X\n", __func__,
			PAC193X_CTRL_STAT_REGS_ADDR);
		return ret;
	}
	/* write the CHANNEL_DIS and the NEG_PWR registers */
	regs[PAC193X_CHANNEL_DIS_REG_OFF] =
		CHANNEL_DIS_REG(chip_info->chip_reg_data.active_channels[0],
				chip_info->chip_reg_data.active_channels[1],
				chip_info->chip_reg_data.active_channels[2],
				chip_info->chip_reg_data.active_channels[3], 0,
				0, 1);

	regs[PAC193X_NEG_PWR_REG_OFF] =
		NEG_PWR_REG(chip_info->chip_reg_data.bi_dir[0],
			    chip_info->chip_reg_data.bi_dir[1],
			    chip_info->chip_reg_data.bi_dir[2],
			    chip_info->chip_reg_data.bi_dir[3],
			    chip_info->chip_reg_data.bi_dir[0],
			    chip_info->chip_reg_data.bi_dir[1],
			    chip_info->chip_reg_data.bi_dir[2],
			    chip_info->chip_reg_data.bi_dir[3]);
	/*
     * the current can be measured uni or bi-dir, but voltages are set only
     * for uni-directional operation
     * no SLOW triggered REFRESH, clear POR
     */
	regs[PAC193X_SLOW_REG_OFF] = 0;
	/* write the updated registers back */
	ret = pac193x_i2c_write(chip_info->client, PAC193X_CTRL_STAT_REGS_ADDR,
				(u8 *)regs, 3);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s - cannot write PAC193x regs from 0x%02X\n",
			__func__, PAC193X_CHANNEL_DIS_REG_OFF);
		return ret;
	}
	/* enable the ALERT pin functionality */
	ret = pac193x_i2c_write_byte(
		chip_info->client, PAC193X_CTRL_REG,
		CTRL_REG(chip_info->chip_reg_data.crt_samp_spd_bitfield, 0, 0,
			 1, 0, 0));
	if (ret < 0) {
		dev_err(&client->dev,
			"%s - cannot write PAC193x ctrl reg at 0x%02X\n",
			__func__, PAC193X_CTRL_REG);
		return ret;
	}
	/*
     * send a REFRESH to the chip, so the new settings take place
     * as well as reseting the accumulators
     */
	ret = pac193x_i2c_send_byte(chip_info->client, PAC193X_REFRESH_REG);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s - cannot send byte to PAC193x 0x%02X reg\n",
			__func__, PAC193X_REFRESH_REG);
		return ret;
	}

	/*
     * get the current(in the chip) sampling speed and compute the
     * required timeout based on its value
     * the timeout is 1/sampling_speed
     */
	wait_time = (1024 /
		     samp_rate_map_tbl[(regs[PAC193X_CTRL_ACT_REG_OFF] >> 6)]) *
		    1000;
	/*
     * wait the maximum amount of time to be on the safe side - the
     * maximum wait time is for 8sps
     */
	usleep_range(wait_time, wait_time + 100);
	/* setup the refresh timeout */
	ret = pac193x_setup_periodic_refresh(chip_info);
	return ret;
}

static int pac193x_retrieve_data(struct pac193x_chip_info *chip_info,
				 u32 wait_time)
{
	int ret = 0;
	struct i2c_client *client = chip_info->client;
	/* check if the minimal elapsed time has passed and if so,
     * re-read the chip, otherwise the cached info is just fine
     */
	if (time_after(jiffies,
		       chip_info->chip_reg_data.jiffies_tstamp +
			       msecs_to_jiffies(PAC193X_MIN_POLLING_TIME))) {
		/* we need to re-read the chip values
         * call the pac193x_reg_snapshot
         */
		ret = pac193x_reg_snapshot(chip_info, true, false, wait_time);
		/* re-schedule the work for the read registers timeout
         * (to prevent chip regs saturation)
         */
		ret = mod_timer(
			&chip_info->tmr_forced_update,
			chip_info->chip_reg_data.jiffies_tstamp +
				msecs_to_jiffies(PAC193X_MAX_RFSH_LIMIT));
		if (ret < 0)
			dev_err(&client->dev,
				"forced read timer cannot be modified!\n");
	}
	return ret;
}

static int pac193x_prep_iio_channels(struct pac193x_chip_info *chip_info,
				     struct iio_dev *indio_dev)
{
	struct i2c_client *client;
	struct iio_chan_spec *ch_sp;
	int channel_size, channel_attribute_count, attribute_count;
	int cnt;
	void *dyn_ch_struct, *tmp_data;

	client = chip_info->client;
	/* find out dynamically how many IIO channels we need */
	channel_attribute_count = 0;
	channel_size = 0;
	for (cnt = 0; cnt < chip_info->phys_channels; cnt++) {
		if (chip_info->chip_reg_data.active_channels[cnt]) {
			/* add the size of the properties of one chip physical channel */
			channel_size += sizeof(pac193x_single_channel);
			/* count how many enabled channels we have */
			channel_attribute_count +=
				ARRAY_SIZE(pac193x_single_channel);
			dev_info(&client->dev, ":%s: Channel %d active\n",
				 __func__, cnt);
		}
	}
	/* now add the timestamp channel size */
	channel_size += sizeof(pac193x_ts);
	/* add one more channel which is the timestamp */
	attribute_count = channel_attribute_count + 1;

	dev_info(&client->dev, ":%s: Active chip attributes: %d\n", __func__,
		 attribute_count);
	dyn_ch_struct = kzalloc(channel_size, GFP_KERNEL);
	if (!dyn_ch_struct)
		return -EINVAL;

	tmp_data = dyn_ch_struct;
	/* populate the dynamic channels and make all the adjustments */
	for (cnt = 0; cnt < chip_info->phys_channels; cnt++) {
		if (chip_info->chip_reg_data.active_channels[cnt]) {
			memcpy(tmp_data, pac193x_single_channel,
			       sizeof(pac193x_single_channel));
			ch_sp = (struct iio_chan_spec *)tmp_data;
			ch_sp[IIO_EN].channel = cnt;
			ch_sp[IIO_EN].scan_index = cnt;
			ch_sp[IIO_EN].address = cnt + PAC193X_VPOWER_ACC_0_ADDR;
			ch_sp[IIO_POW].channel = cnt;
			ch_sp[IIO_POW].scan_index = cnt;
			ch_sp[IIO_POW].address = cnt + PAC193X_VPOWER_0_ADDR;
			ch_sp[IIO_VOLT].channel = cnt;
			ch_sp[IIO_VOLT].scan_index = cnt;
			ch_sp[IIO_VOLT].address = cnt + PAC193X_VBUS_0_ADDR;
			ch_sp[IIO_CRT].channel = cnt;
			ch_sp[IIO_CRT].scan_index = cnt;
			ch_sp[IIO_CRT].address = cnt + PAC193X_VSENSE_0_ADDR;
			ch_sp[IIO_VOLTAVG].channel = cnt;
			ch_sp[IIO_VOLTAVG].scan_index = cnt;
			ch_sp[IIO_VOLTAVG].address =
				cnt + PAC193X_VBUS_AVG_0_ADDR;
			ch_sp[IIO_CRTAVG].channel = cnt;
			ch_sp[IIO_CRTAVG].scan_index = cnt;
			ch_sp[IIO_CRTAVG].address =
				cnt + PAC193X_VSENSE_AVG_0_ADDR;
			/* now modify the parameters in all channels if the
             * whole chip rail(channel) is bi-directional
             */
			if (chip_info->chip_reg_data.bi_dir[cnt]) {
				ch_sp[IIO_EN].scan_type.sign = 's';
				ch_sp[IIO_EN].scan_type.realbits =
					PAC193X_ENERGY_S_RES;
				ch_sp[IIO_POW].scan_type.sign = 's';
				ch_sp[IIO_POW].scan_type.realbits =
					PAC193X_POWER_S_RES;
				ch_sp[IIO_VOLT].scan_type.sign = 's';
				ch_sp[IIO_VOLT].scan_type.realbits =
					PAC193X_VOLTAGE_S_RES;
				ch_sp[IIO_CRT].scan_type.sign = 's';
				ch_sp[IIO_CRT].scan_type.realbits =
					PAC193X_CURRENT_S_RES;
				ch_sp[IIO_VOLTAVG].scan_type.sign = 's';
				ch_sp[IIO_VOLTAVG].scan_type.realbits =
					PAC193X_VOLTAGE_S_RES;
				ch_sp[IIO_CRTAVG].scan_type.sign = 's';
				ch_sp[IIO_CRTAVG].scan_type.realbits =
					PAC193X_CURRENT_S_RES;
			}
			/* advance the pointer */
			tmp_data += sizeof(pac193x_single_channel);
		}
	}
	/* now copy the timestamp channel */
	memcpy(tmp_data, pac193x_ts, sizeof(pac193x_ts));
	ch_sp = (struct iio_chan_spec *)tmp_data;
	ch_sp[0].scan_index = attribute_count - 1;

	/*
     * send the updated dynamic channel structure information towards IIO
     * prepare the required field for IIO class registration
     */
	indio_dev->num_channels = attribute_count;
	indio_dev->channels =
		kmemdup((const struct iio_chan_spec *)dyn_ch_struct,
			channel_size, GFP_KERNEL);
	if (!indio_dev->channels) {
		dev_err(&client->dev, "failed to duplicate channels\n");
		return -EINVAL;
	}
	/* free the dynamic channels attributes memory */
	kfree(dyn_ch_struct);

	return 0;
}

static IIO_DEVICE_ATTR(rst_en_regs_wo_param_0, 0200, NULL,
		       rst_en_regs_wo_param_store, 0);

static IIO_DEVICE_ATTR(rst_en_regs_wo_param_1, 0200, NULL,
		       rst_en_regs_wo_param_store, 0);

static IIO_DEVICE_ATTR(rst_en_regs_wo_param_2, 0200, NULL,
		       rst_en_regs_wo_param_store, 0);

static IIO_DEVICE_ATTR(rst_en_regs_wo_param_3, 0200, NULL,
		       rst_en_regs_wo_param_store, 0);

static IIO_DEVICE_ATTR(shunt_value_0, 0644, shunt_value_show, shunt_value_store,
		       0);

static IIO_DEVICE_ATTR(shunt_value_1, 0644, shunt_value_show, shunt_value_store,
		       0);

static IIO_DEVICE_ATTR(shunt_value_2, 0644, shunt_value_show, shunt_value_store,
		       0);

static IIO_DEVICE_ATTR(shunt_value_3, 0644, shunt_value_show, shunt_value_store,
		       0);

static IIO_DEVICE_ATTR(channel_name_0, 0444, channel_name_show, NULL, 0);

static IIO_DEVICE_ATTR(channel_name_1, 0444, channel_name_show, NULL, 0);

static IIO_DEVICE_ATTR(channel_name_2, 0444, channel_name_show, NULL, 0);

static IIO_DEVICE_ATTR(channel_name_3, 0444, channel_name_show, NULL, 0);

static struct attribute *pac193x_all_attributes[] = {
	PAC193x_DEV_ATTR(rst_en_regs_wo_param_0),
	PAC193x_DEV_ATTR(shunt_value_0),
	PAC193x_DEV_ATTR(channel_name_0),
	PAC193x_DEV_ATTR(rst_en_regs_wo_param_1),
	PAC193x_DEV_ATTR(shunt_value_1),
	PAC193x_DEV_ATTR(channel_name_1),
	PAC193x_DEV_ATTR(rst_en_regs_wo_param_2),
	PAC193x_DEV_ATTR(shunt_value_2),
	PAC193x_DEV_ATTR(channel_name_2),
	PAC193x_DEV_ATTR(rst_en_regs_wo_param_3),
	PAC193x_DEV_ATTR(shunt_value_3),
	PAC193x_DEV_ATTR(channel_name_3),
	NULL
};

static int pac193x_prep_custom_attributes(struct pac193x_chip_info *chip_info,
					  struct iio_dev *indio_dev)
{
	int i, j;
	int active_channels_count = 0;
	struct attribute **pac193x_custom_attributes;
	struct attribute_group *pac193x_group;

	for (i = 0; i < chip_info->phys_channels; i++)
		if (chip_info->chip_reg_data.active_channels[i])
			active_channels_count++;

	pac193x_group = kzalloc(sizeof(struct attribute_group), GFP_KERNEL);

	pac193x_custom_attributes = kzalloc(
		PAC193X_CUSTOM_ATTR_FOR_CHANNEL * active_channels_count *
				sizeof(struct attribute *) +
			1,
		GFP_KERNEL);
	j = 0;

	for (i = 0; i < chip_info->phys_channels; i++) {
		if (chip_info->chip_reg_data.active_channels[i]) {
			pac193x_custom_attributes[PAC193X_CUSTOM_ATTR_FOR_CHANNEL *
						  j] = pac193x_all_attributes
				[PAC193X_CUSTOM_ATTR_FOR_CHANNEL * i];
			pac193x_custom_attributes[PAC193X_CUSTOM_ATTR_FOR_CHANNEL *
							  j +
						  1] = pac193x_all_attributes
				[PAC193X_CUSTOM_ATTR_FOR_CHANNEL * i + 1];
			pac193x_custom_attributes[PAC193X_CUSTOM_ATTR_FOR_CHANNEL *
							  j +
						  2] = pac193x_all_attributes
				[PAC193X_CUSTOM_ATTR_FOR_CHANNEL * i + 2];
			j++;
		}
	}
	pac193x_group->attrs = pac193x_custom_attributes;
	chip_info->pac193x_info.attrs = pac193x_group;
	return 0;
}

static int pac193x_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct pac193x_chip_info *chip_info;
	struct iio_dev *indio_dev;
	const char *name = NULL;
	int cnt, ret = 0;
	int dev_id = 0;

	/*
     * allocate the memory for our private structure
     * related to the chip info structure
     */
	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*chip_info));
	if (!indio_dev)
		return -ENOMEM;
	/*
     * point our chip info structure towards
     * the address freshly allocated
     */
	chip_info = iio_priv(indio_dev);
	/* make the link between the I2C slave client driver and the IIO */
	i2c_set_clientdata(client, indio_dev);
	chip_info->client = client;

	/* clear the chip-related structure */
	memset(&chip_info->chip_reg_data, 0, sizeof(chip_info->chip_reg_data));
	/*
     * load default settings - all channels enabled,
     * uni directional flow, default shunt values
     */

	if (ACPI_HANDLE(&client->dev)) {
		pac193x_get_revision(chip_info);
	} else {
		/* get the name and the dev_id */
		name = id->name;
		dev_id = id->driver_data;
		/* store the type of chip */
		chip_info->chip_variant = dev_id;
		/* get the maximum number of channels for the given chip id */
		chip_info->phys_channels =
			pac193x_chip_config[dev_id].phys_channels;
	}

	for (cnt = 0; cnt < chip_info->phys_channels; cnt++) {
		chip_info->chip_reg_data.active_channels[cnt] = true;
		chip_info->chip_reg_data.bi_dir[cnt] = false;
		chip_info->shunts[cnt] = SHUNT_UOHMS_DEFAULT;
	}
	chip_info->chip_reg_data.crt_samp_spd_bitfield = pac193X_samp_1024sps;

	if (ACPI_HANDLE(&client->dev)) {
		switch (chip_info->chip_variant) {
		case PAC1934_PID:
			client->dev.init_name = "pac1934";
			break;
		case PAC1933_PID:
			client->dev.init_name = "pac1933";
			break;
		case PAC1932_PID:
			client->dev.init_name = "pac1932";
			break;
		case PAC1931_PID:
			client->dev.init_name = "pac1931";
			break;
		default:
			return -EINVAL;
		}
		name = pac193x_match_acpi_device(client, chip_info);
	} else {
		/* identify the chip we have to deal with */
		ret = pac193x_chip_identify(chip_info);
		if (ret < 0)
			return -EINVAL;
		/* check if we find the device within DT */
		if (!client->dev.of_node ||
		    (!of_get_next_child(client->dev.of_node, NULL)))
			return -EINVAL;

		name = pac193x_match_of_device(client, chip_info);
	}
	if (!name) {
		dev_err(&client->dev,
			"DT parameter parsing returned an error\n");
		return -EINVAL;
	}

	/* initialize the chip access mutex */
	mutex_init(&chip_info->lock);
	/*
     * do now any chip specific initialization (e.g. read/write
     * some registers), enable/disable certain channels, change the sampling
     * rate to the requested value
     */
	ret = pac193x_chip_configure(chip_info);
	/* prepare the channel information */
	ret = pac193x_prep_iio_channels(chip_info, indio_dev);
	if (ret < 0)
		goto free_chan_attr_mem;
	/* configure the IIO related fields and register this device with IIO */
	ret = pac193x_prep_custom_attributes(chip_info, indio_dev);
	if (ret < 0)
		goto free_chan_attr_mem;

	chip_info->pac193x_info.read_raw = pac193x_read_raw;
	chip_info->pac193x_info.read_avail = pac193x_read_avail;
	chip_info->pac193x_info.write_raw = pac193x_write_raw;

	indio_dev->info = &chip_info->pac193x_info;
	indio_dev->name = name;
	indio_dev->dev.parent = &client->dev;
	indio_dev->modes = INDIO_DIRECT_MODE;
	/*
     * read whatever it has been accumulated in the chip so far
     * and reset the accumulators
     */
	ret = pac193x_reg_snapshot(chip_info, true, false,
				   PAC193x_MIN_UPDATE_WAIT_TIME);
	/* register with IIO */
	ret = devm_iio_device_register(&client->dev, indio_dev);
	if (ret < 0) {
	free_chan_attr_mem:
		pac193x_remove(client);
	}
	return ret;
}

static int pac193x_remove(struct i2c_client *client)
{
	int ret = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(&client->dev);
	struct pac193x_chip_info *chip_info = iio_priv(indio_dev);
	/* free the channel attributes memory */
	kfree(indio_dev->channels);
	ret = try_to_del_timer_sync(&chip_info->tmr_forced_update);
	if (ret < 0) {
		dev_err(&client->dev,
			"%s - cannot delete the forced readout timer\n",
			__func__);
		return ret;
	}
	if (chip_info->wq_chip != NULL) {
		cancel_work_sync(&chip_info->work_chip_rfsh);
		flush_workqueue(chip_info->wq_chip);
		destroy_workqueue(chip_info->wq_chip);
	}
	kfree(chip_info->pac193x_info.attrs->attrs);
	kfree(chip_info->pac193x_info.attrs);
	return ret;
}

static const struct i2c_device_id pac193x_id[] = { { "pac1934", pac1934 },
						   { "pac1933", pac1933 },
						   { "pac1932", pac1932 },
						   { "pac1931", pac1931 },
						   {} };
MODULE_DEVICE_TABLE(i2c, pac193x_id);

static const struct of_device_id pac193x_of_match[] = {
	{ .compatible = "microchip,pac1934",
	  .data = (void *)&pac193x_chip_config[pac1934] },
	{ .compatible = "microchip,pac1933",
	  .data = (void *)&pac193x_chip_config[pac1933] },
	{ .compatible = "microchip,pac1932",
	  .data = (void *)&pac193x_chip_config[pac1932] },
	{ .compatible = "microchip,pac1931",
	  .data = (void *)&pac193x_chip_config[pac1931] },
	{}
};
MODULE_DEVICE_TABLE(of, pac193x_of_match);

static const char *pac193x_get_of_match_entry(struct i2c_client *client)
{
	const struct of_device_id *match;

	match = of_match_node(pac193x_of_match, client->dev.of_node);
	return match->compatible;
}

#ifdef CONFIG_ACPI
static const struct acpi_device_id pac193x_acpi_match[] = {
	{ "MCHP1930", 0 },
	{},
};
MODULE_DEVICE_TABLE(acpi, pac193x_acpi_match);
#endif

static struct i2c_driver pac193x_driver = {
	.driver = { .name = "pac193x",
		    .of_match_table = pac193x_of_match,
		    .acpi_match_table = ACPI_PTR(pac193x_acpi_match) },
	.probe = pac193x_probe,
	.remove = pac193x_remove,
	.id_table = pac193x_id,
};

module_i2c_driver(pac193x_driver);

MODULE_AUTHOR("Bogdan Bolocan");
MODULE_AUTHOR("Victor Tudose");
MODULE_DESCRIPTION("PAC193x");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("0.2.0");
