// SPDX-License-Identifier: GPL-2.0
/*
 * ov2740 driver
 *
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 *
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <linux/rk-preisp.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>
#include <linux/interrupt.h>
#include <linux/nvmem-provider.h>
#include <linux/regmap.h>

#define DRIVER_VERSION			KERNEL_VERSION(0, 0x01, 0x01)
#define REG_NULL			0xFFFF
#define OV2740_VTS_MAX			0xffff

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif

#define OV2740_FETCH_LSB_GAIN(VAL) (VAL & 0x00FF)       /* gain[7:0] */
#define OV2740_FETCH_MSB_GAIN(VAL) ((VAL >> 8) & 0x1F)	/* gain[10:8] */
#define OV2740_REG_ANALOG_GAIN 0x3508

#define OV2740_REG_EXPOSURE 0x3500
#define OV2740_REG_EXPOSURE_AUTO 0x3503

#define OV2740_AEC_GROUP_UPDATE_ADDRESS 0x3208
#define OV2740_AEC_GROUP_UPDATE_START_DATA 0x00
#define OV2740_AEC_GROUP_UPDATE_END_DATA 0x10
#define OV2740_AEC_GROUP_UPDATE_END_LAUNCH 0xA0	//delay launch  ?

#define OV2740_FETCH_3RD_BYTE_EXP(VAL) (((VAL) >> 12) & 0xF)	/* 4 Bits */	//?
#define OV2740_FETCH_2ND_BYTE_EXP(VAL) (((VAL) >> 4) & 0xFF)	/* 8 Bits */	//?
#define OV2740_FETCH_1ST_BYTE_EXP(VAL) (((VAL) & 0x0F) << 4)	/* 4 Bits */	//?

#define OV2740_PIDH_ADDR     0x300A
#define OV2740_PIDM_ADDR     0x300B
#define OV2740_PIDL_ADDR     0x300C

/* High byte of product ID */
#define OV2740_PIDH_MAGIC 0x00
/* Middle byte of product ID  */
#define OV2740_PIDM_MAGIC 0x27
/* Low byte of product ID  */
#define OV2740_PIDL_MAGIC 0x40

#define OV2740_XVCLK_FREQ 24000000
#define OV2740_PLL_PREDIV0_REG 0x3088
#define OV2740_PLL_PREDIV_REG  0x3080
#define OV2740_PLL_MUL_HIGH_REG 0x3081
#define OV2740_PLL_MUL_LOW_REG 0x3082
#define OV2740_PLL_SPDIV_REG 0x3086
#define OV2740_PLL_DIVSYS_REG 0x3084

#define OV2740_REG_VTS 0x380e
#define OV2740_FINE_INTG_TIME_MIN 0
#define OV2740_FINE_INTG_TIME_MAX_MARGIN 0
#define OV2740_COARSE_INTG_TIME_MIN 1
#define OV2740_COARSE_INTG_TIME_MAX_MARGIN 4
#define OV2740_TIMING_X_INC		0x3814
#define OV2740_TIMING_Y_INC		0x3815
#define OV2740_HORIZONTAL_START_HIGH_REG 0x3800
#define OV2740_HORIZONTAL_START_LOW_REG 0x3801
#define OV2740_VERTICAL_START_HIGH_REG 0x3802
#define OV2740_VERTICAL_START_LOW_REG 0x3803
#define OV2740_HORIZONTAL_END_HIGH_REG 0x3804
#define OV2740_HORIZONTAL_END_LOW_REG 0x3805
#define OV2740_VERTICAL_END_HIGH_REG 0x3806
#define OV2740_VERTICAL_END_LOW_REG 0x3807
#define OV2740_HORIZONTAL_OUTPUT_SIZE_HIGH_REG 0x3808
#define OV2740_HORIZONTAL_OUTPUT_SIZE_LOW_REG 0x3809
#define OV2740_VERTICAL_OUTPUT_SIZE_HIGH_REG 0x380a
#define OV2740_VERTICAL_OUTPUT_SIZE_LOW_REG 0x380b
#define OV2740_H_WIN_OFF_HIGH_REG 0x3810
#define OV2740_H_WIN_OFF_LOW_REG 0x3811
#define OV2740_V_WIN_OFF_HIGH_REG 0x3812
#define OV2740_V_WIN_OFF_LOW_REG 0x3813


#define OV2740_REG_TEST_PATTERN 0x5040
#define OV2740_TEST_PATTERN_ENABLE BIT(7)
#define OV2740_TEST_PATTERN_DISABLE	    0x0
#define OV2740_TEST_PATTERN_BAR_SHIFT	2


#define OV2740_REG_FLIP_MIRROR 0x58fd
#define OV2740_REG_MIRROR_MASK BIT(4)
#define OV2740_REG_FLIP_MASK BIT(5)

#define OV2740_EXPOSURE_MIN 1
#define OV2740_EXPOSURE_STEP 1
#define	ANALOG_GAIN_MIN			0x80
#define	ANALOG_GAIN_MAX			0x7C0
#define	ANALOG_GAIN_STEP		1
#define	ANALOG_GAIN_DEFAULT		0x80
#define OV2740_REG_MWB_R_GAIN			0x500a
#define OV2740_REG_MWB_G_GAIN			0x500c
#define OV2740_REG_MWB_B_GAIN			0x500e

#define OV2740_ANA_ARRAR01 0x3823					//binning
#define OV2740_TIMING_CONCTROL_VH 0x3803	//flip
#define OV2740_TIMING_CONCTROL18 0x3820		//mirror
#define OV2740_NAME			"ov2740"
#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"

/* ISP CTRL00 */
#define OV2740_REG_ISP_CTRL00		0x5000
/* ISP CTRL01 */
#define OV2740_REG_ISP_CTRL01		0x5001
/* Customer Addresses: 0x7010 - 0x710F */
#define CUSTOMER_USE_OTP_SIZE		0x100
/* OTP registers from sensor */
#define OV2740_REG_OTP_CUSTOMER		0x7010

#define OV2740_REG_MODE_SELECT		0x0100
#define OV2740_MODE_STANDBY		0x00
#define OV2740_MODE_STREAMING		0x01
#define OV2740_OTP_PROGRAM_CTRL 0x3d80
#define OV2740_OTP_PROGRAM_ENABLE 0x01
#define OV2740_OTP_PROGRAM_DISABLE 0x00
#define OV2740_OTP_MODE_CTL 0x3d84
#define OV2740_OTP_REG85  0x3d85
#define OV2740_OTP_START_ADDRESS_HIGH 0x3d88
#define OV2740_OTP_START_ADDRESS_LOW 0x3d89
#define OV2740_OTP_END_ADDRESS_HIGH 0x3d8a
#define OV2740_OTP_END_ADDRESS_LOW 0x3d8b
#define OV2740_OTP_R_WAIT (30) //ms
#define OV2740_OTP_W_WAIT (600) //ms

#define CENTER_X_MIN (655)
#define CENTER_X_MID (967)
#define CENTER_X_MAX (1279)

#define CENTER_Y_MIN (487)
#define CENTER_Y_MID (547)
#define CENTER_Y_MAX (607)

struct nvm_data {
	struct i2c_client *client;
	struct nvmem_device *nvmem;
	struct regmap *regmap;
	char *nvm_buffer;
};

enum ov2740_max_pad {
	PAD0, /* link to isp */
	PAD1, /* link to csi wr0 | hdr x2:L x3:M */
	PAD2, /* link to csi wr1 | hdr      x3:L */
	PAD3, /* link to csi wr2 | hdr x2:M x3:S */
	PAD_MAX,
};
static const char * const ov2740_supply_names[] = {
	"avdd",		/* Analog power */
	"dovdd",	/* Digital I/O power */
	"dvdd",		/* Digital core power */
};

#define OV2740_NUM_SUPPLIES ARRAY_SIZE(ov2740_supply_names)

struct regval {
	u16 addr;
	u8 val;
};

struct ov2740_mode {
	u32 bus_fmt;
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
	u32 hdr_mode;
	u32 vc[PAD_MAX];
};

struct ov2740 {
	struct i2c_client	*client;
	struct clk		*xvclk;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*vsync_gpio;
	struct gpio_desc	*pwdn_gpio;
	struct gpio_desc	*power_gpio;
	struct regulator_bulk_data supplies[OV2740_NUM_SUPPLIES];
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pins_default;

	struct v4l2_subdev	subdev;
	struct media_pad	pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl	*exposure;
	struct v4l2_ctrl	*anal_gain;
	struct v4l2_ctrl	*digi_gain;
	struct v4l2_ctrl	*hblank;
	struct v4l2_ctrl	*vblank;
	struct v4l2_ctrl	*test_pattern;
	struct v4l2_ctrl	*h_flip;
	struct v4l2_ctrl	*v_flip;
	struct v4l2_ctrl	*auto_exp;
	struct mutex		mutex;
	bool			streaming;
	bool			power_on;
	const struct ov2740_mode *cur_mode;
	u32			module_index;
	u32			flip;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
	struct nvm_data *nvm;
	bool online;
	uint32_t fmt_mode;
	uint32_t center_x;
	uint32_t center_y;
};

#define to_ov2740(sd) container_of(sd, struct ov2740, subdev)

static const struct regval ov2740_global_regs[] = {
	{REG_NULL, 0x00},
};

/*
 * Xclk 24Mhz
 * max_framerate 60fps
 * mipi_datarate per lane 720Mbps, 2 lane
 */
static struct regval ov2740_1920_1080_60fps[] = {
{0x0103, 0x01},
{0x0302, 0x1e},
{0x030d, 0x1e},
{0x030e, 0x02},
{0x0312, 0x01},
{0x3000, 0x20},
{0x3018, 0x32},
{0x3031, 0x0a},
{0x3080, 0x08},
{0x3083, 0xB4},
{0x3103, 0x00},
{0x3104, 0x01},
{0x3106, 0x01},
{0x3500, 0x00},
{0x3501, 0x44},
{0x3502, 0x40},
{0x3503, 0x88},
{0x3507, 0x00},
{0x3508, 0x00},
{0x3509, 0x80},
{0x350c, 0x00},
{0x350d, 0x80},
{0x3510, 0x00},
{0x3511, 0x00},
{0x3512, 0x20},
{0x3632, 0x00},
{0x3633, 0x10},
{0x3634, 0x10},
{0x3635, 0x10},
{0x3645, 0x13},
{0x3646, 0x81},
{0x3636, 0x10},
{0x3651, 0x0a},
{0x3656, 0x02},
{0x3659, 0x04},
{0x365a, 0xda},
{0x365b, 0xa2},
{0x365c, 0x04},
{0x365d, 0x1d},
{0x365e, 0x1a},
{0x3662, 0xd7},
{0x3667, 0x78},
{0x3669, 0x0a},
{0x366a, 0x92},
{0x3700, 0x54},
{0x3702, 0x10},
{0x3706, 0x42},
{0x3709, 0x30},
{0x370b, 0xc2},
{0x3714, 0x63},
{0x3715, 0x01},
{0x3716, 0x00},
{0x371a, 0x3e},
{0x3732, 0x0e},
{0x3733, 0x10},
{0x375f, 0x0e},
{0x3768, 0x30},
{0x3769, 0x44},
{0x376a, 0x22},
{0x377b, 0x20},
{0x377c, 0x00},
{0x377d, 0x0c},
{0x3798, 0x00},
{0x37a1, 0x55},
{0x37a8, 0x6d},
{0x37c2, 0x04},
{0x37c5, 0x00},
{0x37c8, 0x00},
{0x3800, 0x00},
{0x3801, 0x00},
{0x3802, 0x00},
{0x3803, 0x00},
{0x3804, 0x07},
{0x3805, 0x8f},
{0x3806, 0x04},
{0x3807, 0x47},
{0x3808, 0x07},
{0x3809, 0x80},
{0x380a, 0x04},
{0x380b, 0x38},
{0x380c, 0x04},
{0x380d, 0x38},
{0x380e, 0x04},
{0x380f, 0x60},
{0x3810, 0x00},
{0x3811, 0x04},
{0x3812, 0x00},
{0x3813, 0x04},
{0x3814, 0x01},
{0x3815, 0x01},
{0x3820, 0x80},
{0x3821, 0x46},
{0x3822, 0x84},
{0x3829, 0x00},
{0x382a, 0x01},
{0x382b, 0x01},
{0x3830, 0x04},
{0x3836, 0x01},
{0x3837, 0x08},
{0x3839, 0x01},
{0x383a, 0x00},
{0x383b, 0x08},
{0x383c, 0x00},
{0x3f0b, 0x00},
{0x4001, 0x20},
{0x4009, 0x07},
{0x4003, 0x10},
{0x4010, 0xe0},
{0x4016, 0x00},
{0x4017, 0x10},
{0x4044, 0x02},
{0x4304, 0x08},
{0x4307, 0x30},
{0x4320, 0x80},
{0x4322, 0x00},
{0x4323, 0x00},
{0x4324, 0x00},
{0x4325, 0x00},
{0x4326, 0x00},
{0x4327, 0x00},
{0x4328, 0x00},
{0x4329, 0x00},
{0x432c, 0x03},
{0x432d, 0x81},
{0x4501, 0x84},
{0x4502, 0x40},
{0x4503, 0x18},
{0x4504, 0x04},
{0x4508, 0x02},
{0x4601, 0x10},
{0x4800, 0x00},
{0x4816, 0x52},
{0x4837, 0x16},
{0x5000, 0x43},
{0x5001, 0x00},
{0x5005, 0x38},
{0x501e, 0x0d},
{0x5040, 0x00},
{0x5901, 0x00},
{REG_NULL, 0x00},
};
/*
 * Xclk 24Mhz
 * max_framerate 60fps
 * mipi_datarate per lane 216Mbps, 2 lane
 */
static struct regval ov2740_640_480_60fps[] = {
{0x0103, 0x01},
{0x0302, 0x12},
{0x0303, 0x01},
{0x030d, 0x1e},
{0x030e, 0x02},
{0x0312, 0x01},
{0x3000, 0x20},
{0x3018, 0x32},
{0x3031, 0x0a},
{0x3080, 0x08},
{0x3083, 0xB4},
{0x3103, 0x00},
{0x3104, 0x01},
{0x3106, 0x01},
{0x3500, 0x00},
{0x3501, 0x22},
{0x3502, 0x00},
{0x3503, 0x88},
{0x3507, 0x00},
{0x3508, 0x00},
{0x3509, 0x80},
{0x350c, 0x00},
{0x350d, 0x80},
{0x3510, 0x00},
{0x3511, 0x00},
{0x3512, 0x20},
{0x3632, 0x00},
{0x3633, 0x10},
{0x3634, 0x10},
{0x3635, 0x10},
{0x3645, 0x13},
{0x3646, 0x81},
{0x3636, 0x10},
{0x3651, 0x0a},
{0x3656, 0x02},
{0x3659, 0x04},
{0x365a, 0xda},
{0x365b, 0xa2},
{0x365c, 0x04},
{0x365d, 0x1d},
{0x365e, 0x1a},
{0x3662, 0xd7},
{0x3667, 0x78},
{0x3669, 0x0a},
{0x366a, 0x92},
{0x3700, 0x54},
{0x3702, 0x10},
{0x3706, 0x42},
{0x3709, 0x30},
{0x370b, 0xc2},
{0x3714, 0x26},
{0x3715, 0x01},
{0x3716, 0x00},
{0x371a, 0x3e},
{0x3732, 0x0e},
{0x3733, 0x10},
{0x375f, 0x0e},
{0x3768, 0x20},
{0x3769, 0x44},
{0x376a, 0x22},
{0x377b, 0x20},
{0x377c, 0x00},
{0x377d, 0x0c},
{0x3798, 0x00},
{0x37a1, 0x55},
{0x37a8, 0x6d},
{0x37c2, 0x14},
{0x37c5, 0x00},
{0x37c8, 0x00},
{0x3800, 0x01},
{0x3801, 0x38},
{0x3802, 0x00},
{0x3803, 0x3c},
{0x3804, 0x06},
{0x3805, 0x57},
{0x3806, 0x04},
{0x3807, 0x0b},
{0x3808, 0x02},
{0x3809, 0x80},
{0x380a, 0x01},
{0x380b, 0xe0},
{0x380c, 0x08},
{0x380d, 0x70},
{0x380e, 0x02},
{0x380f, 0x34},
{0x3810, 0x00},
{0x3811, 0x04},
{0x3812, 0x00},
{0x3813, 0x02},
{0x3814, 0x03},
{0x3815, 0x01},
{0x3820, 0x80},
{0x3821, 0x67},
{0x3822, 0x84},
{0x3829, 0x00},
{0x382a, 0x03},
{0x382b, 0x01},
{0x3830, 0x04},
{0x3836, 0x02},
{0x3837, 0x08},
{0x3839, 0x01},
{0x383a, 0x00},
{0x383b, 0x08},
{0x383c, 0x00},
{0x3f0b, 0x00},
{0x4001, 0x20},
{0x4009, 0x03},
{0x4003, 0x10},
{0x4010, 0xe0},
{0x4016, 0x00},
{0x4017, 0x10},
{0x4044, 0x02},
{0x4304, 0x08},
{0x4307, 0x30},
{0x4320, 0x80},
{0x4322, 0x00},
{0x4323, 0x00},
{0x4324, 0x00},
{0x4325, 0x00},
{0x4326, 0x00},
{0x4327, 0x00},
{0x4328, 0x00},
{0x4329, 0x00},
{0x432c, 0x03},
{0x432d, 0x81},
{0x4501, 0x84},
{0x4502, 0x40},
{0x4503, 0x18},
{0x4504, 0x04},
{0x4508, 0x02},
{0x4601, 0x50},
{0x4800, 0x00},
{0x4816, 0x52},
{0x4837, 0x16},
{0x5000, 0x7f},
{0x5001, 0x00},
{0x5005, 0x38},
{0x501e, 0x0d},
{0x5040, 0x00},
{0x5901, 0x00},
{REG_NULL, 0x00},
};

static struct regval ov2740_1920_1080_30fps[] = {
{0x0103, 0x01},
{0x0302, 0x1e},
{0x030d, 0x1e},
{0x030e, 0x02},
{0x0312, 0x01},
{0x3000, 0x20},
{0x3018, 0x12},
{0x3031, 0x0a},
{0x3080, 0x08},
{0x3083, 0xB4},
{0x3103, 0x00},
{0x3104, 0x01},
{0x3106, 0x01},
{0x3500, 0x00},
{0x3501, 0x44},
{0x3502, 0x40},
{0x3503, 0x88},
{0x3507, 0x00},
{0x3508, 0x00},
{0x3509, 0x80},
{0x350c, 0x00},
{0x350d, 0x80},
{0x3510, 0x00},
{0x3511, 0x00},
{0x3512, 0x20},
{0x3632, 0x00},
{0x3633, 0x10},
{0x3634, 0x10},
{0x3635, 0x10},
{0x3645, 0x13},
{0x3646, 0x81},
{0x3636, 0x10},
{0x3651, 0x0a},
{0x3656, 0x02},
{0x3659, 0x04},
{0x365a, 0xda},
{0x365b, 0xa2},
{0x365c, 0x04},
{0x365d, 0x1d},
{0x365e, 0x1a},
{0x3662, 0xd7},
{0x3667, 0x78},
{0x3669, 0x0a},
{0x366a, 0x92},
{0x3700, 0x54},
{0x3702, 0x10},
{0x3706, 0x42},
{0x3709, 0x30},
{0x370b, 0xc2},
{0x3714, 0x63},
{0x3715, 0x01},
{0x3716, 0x00},
{0x371a, 0x3e},
{0x3732, 0x0e},
{0x3733, 0x10},
{0x375f, 0x0e},
{0x3768, 0x30},
{0x3769, 0x44},
{0x376a, 0x22},
{0x377b, 0x20},
{0x377c, 0x00},
{0x377d, 0x0c},
{0x3798, 0x00},
{0x37a1, 0x55},
{0x37a8, 0x6d},
{0x37c2, 0x04},
{0x37c5, 0x00},
{0x37c8, 0x00},
{0x3800, 0x00},
{0x3801, 0x00},
{0x3802, 0x00},
{0x3803, 0x00},
{0x3804, 0x07},
{0x3805, 0x8f},
{0x3806, 0x04},
{0x3807, 0x47},
{0x3808, 0x07},
{0x3809, 0x88},
{0x380a, 0x04},
{0x380b, 0x40},
{0x380c, 0x08},
{0x380d, 0x70},
{0x380e, 0x04},
{0x380f, 0x60},
{0x3810, 0x00},
{0x3811, 0x04},
{0x3812, 0x00},
{0x3813, 0x04},
{0x3814, 0x01},
{0x3815, 0x01},
{0x3820, 0x80},
{0x3821, 0x46},
{0x3822, 0x84},
{0x3829, 0x00},
{0x382a, 0x01},
{0x382b, 0x01},
{0x3830, 0x04},
{0x3836, 0x01},
{0x3837, 0x08},
{0x3839, 0x01},
{0x383a, 0x00},
{0x383b, 0x08},
{0x383c, 0x00},
{0x3f0b, 0x00},
{0x4001, 0x20},
{0x4009, 0x07},
{0x4003, 0x10},
{0x4010, 0xe0},
{0x4016, 0x00},
{0x4017, 0x10},
{0x4044, 0x02},
{0x4304, 0x08},
{0x4307, 0x30},
{0x4320, 0x80},
{0x4322, 0x00},
{0x4323, 0x00},
{0x4324, 0x00},
{0x4325, 0x00},
{0x4326, 0x00},
{0x4327, 0x00},
{0x4328, 0x00},
{0x4329, 0x00},
{0x432c, 0x03},
{0x432d, 0x81},
{0x4501, 0x84},
{0x4502, 0x40},
{0x4503, 0x18},
{0x4504, 0x04},
{0x4508, 0x02},
{0x4601, 0x10},
{0x4800, 0x00},
{0x4816, 0x52},
{0x4837, 0x16},
{0x5000, 0x43},
{0x5001, 0x00},
{0x5005, 0x38},
{0x501e, 0x0d},
{0x5040, 0x00},
{0x5901, 0x00},
{REG_NULL, 0x00},
};

static const struct ov2740_mode supported_modes[] = {
	{
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.exp_def = 0x18f,
		.hts_def = 0x438 * 2,
		.vts_def = 0x460,
		.hdr_mode = NO_HDR,
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.reg_list = ov2740_1920_1080_60fps,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
	{
		.width = 640,
		.height = 480,
		.max_fps = {
			.numerator = 10000,
			.denominator = 600000,
		},
		.exp_def = 0x18f,
		.hts_def = 0x870 * 2,
		.vts_def = 0x234,
		.hdr_mode = NO_HDR,
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.reg_list = ov2740_640_480_60fps,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	},
	{
		.width = 1920,
		.height = 1080,
		.max_fps = {
			.numerator = 10000,
			.denominator = 300000,
		},
		.exp_def = 0x18f,
		.hts_def = 0x438 * 2,
		.vts_def = 0x460,
		.hdr_mode = NO_HDR,
		.bus_fmt = MEDIA_BUS_FMT_SBGGR10_1X10,
		.reg_list = ov2740_1920_1080_30fps,
		.vc[PAD0] = V4L2_MBUS_CSI2_CHANNEL_0,
	}
};

#define OV2740_LANES			2
#define OV2740_BITS_PER_SAMPLE		10
#define OV2740_LINK_FREQ		360000000	// 742.5Mbps
#define OV2740_PIXEL_RATE		(OV2740_LINK_FREQ * 2 * \
					OV2740_LANES / OV2740_BITS_PER_SAMPLE)

static const s64 link_freq_menu_items[] = {
	OV2740_LINK_FREQ
};
#define OV2740_REG_VALUE_08BIT		1
#define OV2740_REG_VALUE_16BIT		2
#define OV2740_REG_VALUE_24BIT		3

static const char * const ov2740_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

u64 vsync_time = 0;

u64 get_ov2740_vsync_time(void) {
	return vsync_time;
}
EXPORT_SYMBOL_GPL(get_ov2740_vsync_time);

/* Write registers up to 4 at a time */
static int ov2740_write_reg(struct i2c_client *client, u16 reg,
			    u32 len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

static void center_fixme(struct ov2740 *ov2740) {
	u8 x_start_h = 0, x_start_l = 0, y_start_h = 0, y_start_l = 0;
	u8 x_end_h = 0, x_end_l = 0, y_end_h = 0, y_end_l = 0;
	u16 x_start, y_start, x_end, y_end;
	u32 i;
	const struct regval *regs;

	if (IS_ERR_OR_NULL(ov2740)) {
		pr_err("center fixme ov2740 ptr err\n");
		return;
	}

	if (ov2740->center_x == CENTER_X_MID && \
			ov2740->center_y == CENTER_Y_MID) {
		pr_info("no need center fix");
		return;
	}

	regs = ov2740->cur_mode->reg_list;

	for (i = 0; regs[i].addr != REG_NULL; i++) {
		if (regs[i].addr == OV2740_HORIZONTAL_START_HIGH_REG)
			x_start_h = regs[i].val;
		if (regs[i].addr == OV2740_HORIZONTAL_START_LOW_REG)
			x_start_l = regs[i].val;
		if (regs[i].addr == OV2740_VERTICAL_START_HIGH_REG)
			y_start_h = regs[i].val;
		if (regs[i].addr == OV2740_VERTICAL_START_LOW_REG)
			y_start_l = regs[i].val;
		if (regs[i].addr == OV2740_HORIZONTAL_END_HIGH_REG)
			x_end_h = regs[i].val;
		if (regs[i].addr == OV2740_HORIZONTAL_END_LOW_REG)
			x_end_l = regs[i].val;
		if (regs[i].addr == OV2740_VERTICAL_END_HIGH_REG)
			y_end_h = regs[i].val;
		if (regs[i].addr == OV2740_VERTICAL_END_LOW_REG)
			y_end_l = regs[i].val;
	}

	x_start = (x_start_h << 8) | x_start_l;
	y_start = (y_start_h << 8) | y_start_l;
	x_end = (x_end_h << 8) | x_end_l;
	y_end = (y_end_h << 8) | y_end_l;

	x_start += ov2740->center_x - CENTER_X_MID;
	y_start += ov2740->center_y - CENTER_Y_MID;
	x_end += ov2740->center_x - CENTER_X_MID;
	y_end += ov2740->center_y - CENTER_Y_MID;

	x_start_h = (x_start >> 8) & 0xff;
	x_start_l = x_start & 0x00ff;

	y_start_h = (y_start >> 8) & 0xff;
	y_start_l = y_start & 0x00ff;

	x_end_h = (x_end >> 8) & 0xff;
	x_end_l = x_end & 0x00ff;

	y_end_h = (y_end >> 8) & 0xff;
	y_end_l = y_end & 0x00ff;

	ov2740_write_reg(ov2740->client, OV2740_HORIZONTAL_START_HIGH_REG,
			OV2740_REG_VALUE_08BIT, x_start_h);

	ov2740_write_reg(ov2740->client, OV2740_HORIZONTAL_START_LOW_REG,
			OV2740_REG_VALUE_08BIT, x_start_l);

	ov2740_write_reg(ov2740->client, OV2740_VERTICAL_START_HIGH_REG,
			OV2740_REG_VALUE_08BIT, y_start_h);

	ov2740_write_reg(ov2740->client, OV2740_VERTICAL_START_LOW_REG,
			OV2740_REG_VALUE_08BIT, y_start_l);

	ov2740_write_reg(ov2740->client, OV2740_HORIZONTAL_END_HIGH_REG,
			OV2740_REG_VALUE_08BIT, x_end_h);

	ov2740_write_reg(ov2740->client, OV2740_HORIZONTAL_END_LOW_REG,
			OV2740_REG_VALUE_08BIT, x_end_l);

	ov2740_write_reg(ov2740->client, OV2740_VERTICAL_END_HIGH_REG,
			OV2740_REG_VALUE_08BIT, y_end_h);

	ov2740_write_reg(ov2740->client, OV2740_VERTICAL_END_LOW_REG,
			OV2740_REG_VALUE_08BIT, y_end_l);

	pr_info("sensor center fix: [xsh: %#x], [xsl: %#x], [ysh %#x]," \
			" [ysl: %#x], [xeh %#x], [xel %#x], [yeh %#x], [yel %#x]\n", \
			x_start_h, x_start_l, y_start_h, y_start_l, x_end_h, \
			x_end_l, y_end_h, y_end_l);
}

static int ov2740_write_array(struct i2c_client *client,
			      const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
		ret = ov2740_write_reg(client, regs[i].addr,
					OV2740_REG_VALUE_08BIT, regs[i].val);

	return ret;
}

/* Read registers up to 4 at a time */
/* sensor register read */
static int ov2740_read_reg(struct i2c_client *client, u16 reg, unsigned int len,
			    u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs)) {
		printk("ov2740 ret(%d) size is not right\n",ret );
		return -EIO;
		}
	*val = be32_to_cpu(data_be);

	return 0;
}

static int ov2740_get_reso_dist(const struct ov2740_mode *mode,
				struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct ov2740_mode *
ov2740_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = ov2740_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int ov2740_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ov2740 *ov2740 = to_ov2740(sd);
	const struct ov2740_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&ov2740->mutex);

	mode = ov2740_find_best_fit(fmt);
	fmt->format.code = MEDIA_BUS_FMT_SBGGR10_1X10;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&ov2740->mutex);
		return -ENOTTY;
#endif
	} else {
		ov2740->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(ov2740->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(ov2740->vblank, vblank_def,
					 OV2740_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&ov2740->mutex);

	return 0;
}

static int ov2740_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ov2740 *ov2740 = to_ov2740(sd);
	const struct ov2740_mode *mode = ov2740->cur_mode;

	mutex_lock(&ov2740->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&ov2740->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = MEDIA_BUS_FMT_SBGGR10_1X10;
		fmt->format.field = V4L2_FIELD_NONE;
		/* format info: width/height/data type/virctual channel */
		if (fmt->pad < PAD_MAX && mode->hdr_mode != NO_HDR)
			fmt->reserved[0] = mode->vc[fmt->pad];
		else
			fmt->reserved[0] = mode->vc[PAD0];
	}
	mutex_unlock(&ov2740->mutex);

	return 0;
}

static int ov2740_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_SBGGR10_1X10;

	return 0;
}

static int ov2740_enum_frame_sizes(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SBGGR10_1X10)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

static int ov2740_enable_test_pattern(struct ov2740 *ov2740, u32 pattern)
{
	u32 val = 0;

	if (pattern)
		val = (pattern - 1) << OV2740_TEST_PATTERN_BAR_SHIFT |
			  OV2740_TEST_PATTERN_ENABLE;

	return ov2740_write_reg(ov2740->client,
				 OV2740_REG_TEST_PATTERN,
				 OV2740_REG_VALUE_08BIT,
				 val);
}

static void ov2740_get_module_inf(struct ov2740 *ov2740,
				  struct rkmodule_inf *inf)
{
	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, OV2740_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, ov2740->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, ov2740->len_name, sizeof(inf->base.lens));
}

static int ov2740_set_hdrae(struct ov2740 *ov2740,
			    struct preisp_hdrae_exp_s *ae)
{
#if 1
	return 0;
#else
	int ret = 0;
	u32 l_exp_time, m_exp_time, s_exp_time;
	u32 l_a_gain, m_a_gain, s_a_gain;

	if (!ov2740->has_init_exp && !ov2740->streaming) {
		ov2740->init_hdrae_exp = *ae;
		ov2740->has_init_exp = true;
		dev_info(&ov2740->client->dev, "sc200ai don't stream, record exp for hdr!\n");
		return ret;
	}
	l_exp_time = ae->long_exp_reg;
	m_exp_time = ae->middle_exp_reg;
	s_exp_time = ae->short_exp_reg;
	l_a_gain = ae->long_gain_reg;
	m_a_gain = ae->middle_gain_reg;
	s_a_gain = ae->short_gain_reg;

	dev_dbg(&ov2740->client->dev,
		"rev exp req: L_exp: 0x%x, 0x%x, M_exp: 0x%x, 0x%x S_exp: 0x%x, 0x%x\n",
		l_exp_time, m_exp_time, s_exp_time,
		l_a_gain, m_a_gain, s_a_gain);

	if (ov2740->cur_mode->hdr_mode == HDR_X2) {
		//2 stagger
		l_a_gain = m_a_gain;
		l_exp_time = m_exp_time;
	}

	//set exposure
	l_exp_time = l_exp_time * 2;
	s_exp_time = s_exp_time * 2;
	if (l_exp_time > 4362)                  //(2250 - 64 - 5) * 2
		l_exp_time = 4362;
	if (s_exp_time > 118)                //(64 - 5) * 2
		s_exp_time = 118;

	ret = ov2740_write_reg(ov2740->client,
				SC200AI_REG_EXPOSURE_H,
				OV2740_REG_VALUE_08BIT,
				SC200AI_FETCH_EXP_H(l_exp_time));
	ret |= ov2740_write_reg(sc200ai->client,
				 SC200AI_REG_EXPOSURE_M,
				 OV2740_REG_VALUE_08BIT,
				 SC200AI_FETCH_EXP_M(l_exp_time));
	ret |= ov2740_write_reg(sc200ai->client,
				 SC200AI_REG_EXPOSURE_L,
				 OV2740_REG_VALUE_08BIT,
				 SC200AI_FETCH_EXP_L(l_exp_time));
	ret |= ov2740_write_reg(sc200ai->client,
				 SC200AI_REG_SEXPOSURE_M,
				 OV2740_REG_VALUE_08BIT,
				 SC200AI_FETCH_EXP_M(s_exp_time));
	ret |= ov2740_write_reg(sc200ai->client,
				 SC200AI_REG_SEXPOSURE_L,
				 OV2740_REG_VALUE_08BIT,
				 SC200AI_FETCH_EXP_L(s_exp_time));


	ret |= sc200ai_set_gain_reg(sc200ai, l_a_gain, SC200AI_LGAIN);
	ret |= sc200ai_set_gain_reg(sc200ai, s_a_gain, SC200AI_SGAIN);
	return ret;
#endif
}

static long ov2740_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct ov2740 *ov2740 = to_ov2740(sd);
	struct rkmodule_hdr_cfg *hdr;
	u32 i, h, w;
	long ret = 0;
	u32  stream;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		ov2740_get_module_inf(ov2740, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		hdr->esp.mode = HDR_NORMAL_VC;
		hdr->hdr_mode = ov2740->cur_mode->hdr_mode;
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = (struct rkmodule_hdr_cfg *)arg;
		w = ov2740->cur_mode->width;
		h = ov2740->cur_mode->height;
		for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
			if (w == supported_modes[i].width &&
			    h == supported_modes[i].height &&
			    supported_modes[i].hdr_mode == hdr->hdr_mode) {
				ov2740->cur_mode = &supported_modes[i];
				break;
			}
		}
		if (i == ARRAY_SIZE(supported_modes)) {
			dev_err(&ov2740->client->dev,
				"not find hdr mode:%d %dx%d config\n",
				hdr->hdr_mode, w, h);
			ret = -EINVAL;
		} else {
			w = ov2740->cur_mode->hts_def - ov2740->cur_mode->width;
			h = ov2740->cur_mode->vts_def - ov2740->cur_mode->height;
			__v4l2_ctrl_modify_range(ov2740->hblank, w, w, 1, w);
			__v4l2_ctrl_modify_range(ov2740->vblank, h,
						 OV2740_VTS_MAX - ov2740->cur_mode->height, 1, h);
		}
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		ov2740_set_hdrae(ov2740, arg);
		break;
	case RKMODULE_SET_QUICK_STREAM:

		stream = *((u32 *)arg);

		if (stream)
			ret = ov2740_write_reg(ov2740->client, 0x0100,OV2740_REG_VALUE_08BIT,0x1);
		else
			ret = ov2740_write_reg(ov2740->client, 0x0100,OV2740_REG_VALUE_08BIT,0x0);
		break;
	case RKMODULE_GET_VSYNC_TIME:
		*(u64*)arg = vsync_time;
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;

	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long ov2740_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	long ret;
	u32 stream = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = ov2740_ioctl(sd, cmd, inf);
		if (!ret)
			ret = copy_to_user(up, inf, sizeof(*inf));
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
		if (!cfg) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(cfg, up, sizeof(*cfg));
		if (!ret)
			ret = ov2740_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	case RKMODULE_GET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = ov2740_ioctl(sd, cmd, hdr);
		if (!ret)
			ret = copy_to_user(up, hdr, sizeof(*hdr));
		kfree(hdr);
		break;
	case RKMODULE_SET_HDR_CFG:
		hdr = kzalloc(sizeof(*hdr), GFP_KERNEL);
		if (!hdr) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(hdr, up, sizeof(*hdr));
		if (!ret)
			ret = ov2740_ioctl(sd, cmd, hdr);
		kfree(hdr);
		break;
	case PREISP_CMD_SET_HDRAE_EXP:
		hdrae = kzalloc(sizeof(*hdrae), GFP_KERNEL);
		if (!hdrae) {
			ret = -ENOMEM;
			return ret;
		}

		ret = copy_from_user(hdrae, up, sizeof(*hdrae));
		if (!ret)
			ret = ov2740_ioctl(sd, cmd, hdrae);
		kfree(hdrae);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif

static int ov2740_load_otp_data(struct nvm_data *nvm)
{
	struct i2c_client *client;
	struct ov2740 *ov2740;
	u32 isp_ctrl00 = 0;
	u32 isp_ctrl01 = 0;
	int ret;

	if (!nvm)
		return -EINVAL;

	if (nvm->nvm_buffer)
		return 0;

	client = nvm->client;
	ov2740 = to_ov2740(i2c_get_clientdata(client));

	nvm->nvm_buffer = kzalloc(CUSTOMER_USE_OTP_SIZE, GFP_KERNEL);
	if (!nvm->nvm_buffer)
		return -ENOMEM;

	ret = ov2740_read_reg(ov2740->client, OV2740_REG_ISP_CTRL00, 1, &isp_ctrl00);
	if (ret) {
		dev_err(&client->dev, "failed to read ISP CTRL00\n");
		goto err;
	}

	ret = ov2740_read_reg(ov2740->client, OV2740_REG_ISP_CTRL01, 1, &isp_ctrl01);
	if (ret) {
		dev_err(&client->dev, "failed to read ISP CTRL01\n");
		goto err;
	}

	/* Clear bit 5 of ISP CTRL00 */
	ret = ov2740_write_reg(ov2740->client, OV2740_REG_ISP_CTRL00, 1,
			isp_ctrl00 & ~BIT(5));
	if (ret) {
		dev_err(&client->dev, "failed to set ISP CTRL00\n");
		goto err;
	}

	/* Clear bit 7 of ISP CTRL01 */
	ret = ov2740_write_reg(ov2740->client, OV2740_REG_ISP_CTRL01, 1,
			isp_ctrl01 & ~BIT(7));
	if (ret) {
		dev_err(&client->dev, "failed to set ISP CTRL01\n");
		goto err;
	}

	ret = ov2740_write_reg(ov2740->client, OV2740_REG_MODE_SELECT, 1,
			OV2740_MODE_STREAMING);
	if (ret) {
		dev_err(&client->dev, "failed to set streaming mode\n");
		goto err;
	}

	/*
	 * Users are not allowed to access OTP-related registers and memory
	 * during the 20 ms period after streaming starts (0x100 = 0x01).
	 */
	msleep(OV2740_OTP_R_WAIT);

	ret = regmap_bulk_read(nvm->regmap, OV2740_REG_OTP_CUSTOMER,
			nvm->nvm_buffer, CUSTOMER_USE_OTP_SIZE);
	if (ret) {
		dev_err(&client->dev, "failed to read OTP data, ret %d\n", ret);
		goto err;
	}

	ret = ov2740_write_reg(ov2740->client, OV2740_REG_MODE_SELECT, 1,
			OV2740_MODE_STANDBY);
	if (ret) {
		dev_err(&client->dev, "failed to set streaming mode\n");
		goto err;
	}

	ret = ov2740_write_reg(ov2740->client, OV2740_REG_ISP_CTRL01, 1, isp_ctrl01);
	if (ret) {
		dev_err(&client->dev, "failed to set ISP CTRL01\n");
		goto err;
	}

	ret = ov2740_write_reg(ov2740->client, OV2740_REG_ISP_CTRL00, 1, isp_ctrl00);
	if (ret) {
		dev_err(&client->dev, "failed to set ISP CTRL00\n");
		goto err;
	}

	return 0;
err:
	kfree(nvm->nvm_buffer);
	nvm->nvm_buffer = NULL;

	return ret;
}

static int ov2740_store_otp_data(struct nvm_data *nvm ,unsigned int off,
				   void *val, size_t count)
{
	struct i2c_client *client;
	struct ov2740 *ov2740;
	u32 isp_ctrl00 = 0;
	u32 isp_ctrl01 = 0;
	int ret;

	if (!nvm) {
		pr_err("nvm invalid\n");
		return -EINVAL;
	}

	if (!val || count == 0) {
		pr_err("para invalid\n");
		return -EINVAL;
	}

	client = nvm->client;
	ov2740 = to_ov2740(i2c_get_clientdata(client));

	ret = ov2740_read_reg(ov2740->client, OV2740_REG_ISP_CTRL00, 1, &isp_ctrl00);
	if (ret) {
		dev_err(&client->dev, "failed to read ISP CTRL00, ret %d\n", ret);
		goto err;
	}

	ret = ov2740_read_reg(ov2740->client, OV2740_REG_ISP_CTRL01, 1, &isp_ctrl01);
	if (ret) {
		dev_err(&client->dev, "failed to read ISP CTRL01, ret %d\n", ret);
		goto err;
	}

	/* Clear bit 5 of ISP CTRL00 */
	ret = ov2740_write_reg(ov2740->client, OV2740_REG_ISP_CTRL00, 1,
			isp_ctrl00 & ~BIT(5));
	if (ret) {
		dev_err(&client->dev, "failed to set ISP CTRL00, ret %d\n", ret);
		goto err;
	}

	/* Clear bit 7 of ISP CTRL01 */
	ret = ov2740_write_reg(ov2740->client, OV2740_REG_ISP_CTRL01, 1,
			isp_ctrl01 & ~BIT(7));
	if (ret) {
		dev_err(&client->dev, "failed OV2740_REG_ISP_CTRL01, ret %d\n", ret);
		goto err;
	}

	ret = ov2740_write_reg(ov2740->client, OV2740_REG_MODE_SELECT, 1,
			OV2740_MODE_STREAMING);
	if (ret) {
		dev_err(&client->dev, "failed OV2740_REG_MODE_SELECT, ret %d\n", ret);
		goto err;
	}

	ret = ov2740_write_reg(ov2740->client, OV2740_OTP_MODE_CTL, 1, 0x40);
	if (ret) {
		dev_err(&client->dev, "failed OV2740_OTP_MODE_CTL, ret %d\n", ret);
		goto err;
	}

	ret = ov2740_write_reg(ov2740->client, OV2740_OTP_REG85, 1, 0x00);
	if (ret) {
		dev_err(&client->dev, "failed OV2740_OTP_REG85, ret %d\n", ret);
		goto err;
	}

	ret = ov2740_write_reg(ov2740->client, OV2740_OTP_START_ADDRESS_HIGH, 1,\
			(OV2740_REG_OTP_CUSTOMER + off) >> 16);
	if (ret) {
		dev_err(&client->dev, "failed OV2740_OTP_START_ADDRESS_HIGH, ret %d\n", ret);
		goto err;
	}

	ret = ov2740_write_reg(ov2740->client, OV2740_OTP_START_ADDRESS_LOW, 1,\
			(OV2740_REG_OTP_CUSTOMER + off) & 0xff);
	if (ret) {
		dev_err(&client->dev, "failed OV2740_OTP_START_ADDRESS_LOW, ret %d\n", ret);
		goto err;
	}

	ret = ov2740_write_reg(ov2740->client, OV2740_OTP_END_ADDRESS_HIGH, 1,\
			(OV2740_REG_OTP_CUSTOMER + off + count) >> 16);
	if (ret) {
		dev_err(&client->dev, "failed OV2740_OTP_END_ADDRESS_HIGH, ret %d\n", ret);
		goto err;
	}

	ret = ov2740_write_reg(ov2740->client, OV2740_OTP_END_ADDRESS_LOW, 1,\
			(OV2740_REG_OTP_CUSTOMER + off + count) & 0xff);
	if (ret) {
		dev_err(&client->dev, "failed OV2740_OTP_END_ADDRESS_LOW, ret %d\n", ret);
		goto err;
	}

	ret = ov2740_write_reg(ov2740->client, OV2740_REG_MODE_SELECT, 1,\
			OV2740_MODE_STREAMING);
	if (ret) {
		dev_err(&client->dev, "failed OV2740_REG_MODE_SELECT, ret %d\n", ret);
		goto err;
	}

	msleep(OV2740_OTP_R_WAIT);

	ret = regmap_bulk_write(nvm->regmap, OV2740_REG_OTP_CUSTOMER + off, val, count);
	if (ret) {
		dev_err(&client->dev, "regmap bulk write, ret %d\n", ret);
		goto err;
	}

	ret = ov2740_write_reg(ov2740->client, OV2740_OTP_PROGRAM_CTRL, 1,\
			OV2740_OTP_PROGRAM_ENABLE);
	if (ret) {
		dev_err(&client->dev, "failed OV2740_OTP_PROGRAM_CTRL, ret %d\n", ret);
		goto err;
	}

	msleep(OV2740_OTP_W_WAIT);
	ret = ov2740_write_reg(ov2740->client, OV2740_OTP_PROGRAM_CTRL, 1,\
			OV2740_OTP_PROGRAM_DISABLE);
	if (ret) {
		dev_err(&client->dev, "failed OV2740_OTP_PROGRAM_CTRL, ret %d\n", ret);
		goto err;
	}

	ret = ov2740_write_reg(ov2740->client, OV2740_REG_ISP_CTRL01, 1, isp_ctrl01);
	if (ret) {
		dev_err(&client->dev, "failed to set ISP CTRL01\n");
		goto err;
	}

	ret = ov2740_write_reg(ov2740->client, OV2740_REG_ISP_CTRL00, 1, isp_ctrl00);
	if (ret) {
		dev_err(&client->dev, "failed to set ISP CTRL00\n");
		goto err;
	}

	msleep(OV2740_OTP_W_WAIT);
err:
	/* free read buffer whether write success or not,
	 * otp read will alloc buffer again */
	if (nvm->nvm_buffer) {
		kfree(nvm->nvm_buffer);
		nvm->nvm_buffer = NULL;
	}

	return ret;
}

static int __ov2740_start_stream(struct ov2740 *ov2740)
{
	int ret;

	ret = ov2740_write_array(ov2740->client, ov2740->cur_mode->reg_list);
	if (ret) {
		pr_err("[%s][%d]: error ret is %d", __FILE__, __LINE__, ret);
		return ret;
	}

	center_fixme(ov2740);

	/* In case these controls are set before streaming */
	mutex_unlock(&ov2740->mutex);
	ret = v4l2_ctrl_handler_setup(&ov2740->ctrl_handler);
	mutex_lock(&ov2740->mutex);
	if (ret) {
		pr_err("[%s][%d]: error ret is %d", __FILE__, __LINE__, ret);
		return ret;
	}

	ret |= ov2740_write_reg(ov2740->client, 0x0100,1, 0x01);

	return ret;
}

static int __ov2740_stop_stream(struct ov2740 *ov2740)
{
	return ov2740_write_reg(ov2740->client, 0x0100, 1,0x00);
}

static int ov2740_s_stream(struct v4l2_subdev *sd, int on)
{
	struct ov2740 *ov2740 = to_ov2740(sd);
	struct i2c_client *client = ov2740->client;
	int ret = 0;
	dev_info(&client->dev, "%s: on: %d, %dx%d@%d\n", __func__, on,
				ov2740->cur_mode->width,
				ov2740->cur_mode->height,
		DIV_ROUND_CLOSEST(ov2740->cur_mode->max_fps.denominator,
				  ov2740->cur_mode->max_fps.numerator));

	mutex_lock(&ov2740->mutex);
	on = !!on;
	if (on == ov2740->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __ov2740_start_stream(ov2740);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs, ret = %d\n", ret);
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__ov2740_stop_stream(ov2740);
		pm_runtime_put(&client->dev);
	}

	ov2740->streaming = on;

unlock_and_return:
	mutex_unlock(&ov2740->mutex);

	return ret;
}

static int ov2740_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct ov2740 *ov2740 = to_ov2740(sd);
	const struct ov2740_mode *mode = ov2740->cur_mode;

	mutex_lock(&ov2740->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&ov2740->mutex);

	return 0;
}
static int ov2740_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov2740 *ov2740 = to_ov2740(sd);
	struct i2c_client *client = ov2740->client;
	int ret = 0;

	mutex_lock(&ov2740->mutex);

	/* If the power state is not modified - no work to do. */
	if (ov2740->power_on == !!on)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = ov2740_write_array(ov2740->client, ov2740_global_regs);
		if (ret) {
			v4l2_err(sd, "could not set init registers\n");
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ov2740->power_on = true;
	} else {
		pm_runtime_put(&client->dev);
		ov2740->power_on = false;
	}

unlock_and_return:
	mutex_unlock(&ov2740->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 ov2740_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, OV2740_XVCLK_FREQ / 1000 / 1000);
}

static int __ov2740_power_on(struct ov2740 *ov2740)
{
	int ret;
	u32 delay_us;
	struct device *dev = &ov2740->client->dev;

	if (!IS_ERR_OR_NULL(ov2740->pins_default)) {
		ret = pinctrl_select_state(ov2740->pinctrl,
					   ov2740->pins_default);
		if (ret < 0)
			dev_err(dev, "could not set pins\n");
	}

	ret = clk_set_rate(ov2740->xvclk, OV2740_XVCLK_FREQ);
	if (ret < 0)
		dev_warn(dev, "Failed to set xvclk rate (24MHz)\n");
	if (clk_get_rate(ov2740->xvclk) != OV2740_XVCLK_FREQ)
		dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");
	ret = clk_prepare_enable(ov2740->xvclk);
	if (ret < 0) {
		dev_info(dev, "Failed to enable xvclk\n");
		return ret;
	}

	if (!IS_ERR(ov2740->pwdn_gpio)) {
		gpiod_set_value_cansleep(ov2740->pwdn_gpio, 1);
		usleep_range(2000, 5000);
	}
	if (!IS_ERR(ov2740->reset_gpio)) {
		gpiod_set_value_cansleep(ov2740->reset_gpio, 1);
	}

	ret = regulator_bulk_enable(OV2740_NUM_SUPPLIES, ov2740->supplies);
	usleep_range(20000, 50000);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto disable_clk;
	}

	if (!IS_ERR(ov2740->pwdn_gpio)) {
		gpiod_set_value_cansleep(ov2740->pwdn_gpio, 0);
		usleep_range(2000, 5000);
	}

	if (!IS_ERR(ov2740->reset_gpio)) {
		gpiod_set_value_cansleep(ov2740->reset_gpio, 0);
		usleep_range(2000, 5000);
	}

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = ov2740_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

	return 0;

disable_clk:
	clk_disable_unprepare(ov2740->xvclk);

	return ret;
}

static void __ov2740_power_off(struct ov2740 *ov2740)
{
	if (!IS_ERR(ov2740->pwdn_gpio))
		gpiod_set_value_cansleep(ov2740->pwdn_gpio, 0);
	clk_disable_unprepare(ov2740->xvclk);
	if (!IS_ERR(ov2740->reset_gpio))
		gpiod_set_value_cansleep(ov2740->reset_gpio, 1);
	regulator_bulk_disable(OV2740_NUM_SUPPLIES, ov2740->supplies);
}

static int ov2740_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov2740 *ov2740 = to_ov2740(sd);

	return __ov2740_power_on(ov2740);
}

static int ov2740_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov2740 *ov2740 = to_ov2740(sd);

	__ov2740_power_off(ov2740);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int ov2740_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct ov2740 *ov2740 = to_ov2740(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct ov2740_mode *def_mode = &supported_modes[ov2740->fmt_mode];

	mutex_lock(&ov2740->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = MEDIA_BUS_FMT_SBGGR10_1X10;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&ov2740->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static int ov2740_g_mbus_config(struct v4l2_subdev *sd,
				 struct v4l2_mbus_config *config)
{
	u32 val = 1 << (OV2740_LANES - 1) |
		V4L2_MBUS_CSI2_CHANNEL_0 |
		V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;

	config->type = V4L2_MBUS_CSI2;
	config->flags = val;

	return 0;
}
static int ov2740_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	if (fie->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;
#if 0
	if (fie->code != MEDIA_BUS_FMT_SBGGR10_1X10)
		return -EINVAL;
#endif
	fie->width = supported_modes[fie->index].width;
	fie->height = supported_modes[fie->index].height;
	fie->interval = supported_modes[fie->index].max_fps;
	return 0;
}

static const struct dev_pm_ops ov2740_pm_ops = {
	SET_RUNTIME_PM_OPS(ov2740_runtime_suspend,
			   ov2740_runtime_resume, NULL)
};

static irqreturn_t camera_vsync_irq_func(int irq, void *dev)
{
	vsync_time = ktime_get_ns();

	return IRQ_HANDLED;
}

static int init_vsync_interrupt(struct ov2740 *ov2740)
{
	struct gpio_desc *vsync_gpio;
	struct device *dev = &ov2740->client->dev;
	int vsync_irq;
	int ret;

	vsync_gpio = ov2740->vsync_gpio;

	ret = gpiod_direction_input(vsync_gpio);
	if (ret) {
		dev_err(dev, "Failed to set gpio input.\n");
		return -EINVAL;
	}

	vsync_irq = gpiod_to_irq(vsync_gpio);
	if (vsync_irq < 0) {
		dev_err(dev, "Failed to set gpio to irq.\n");
		return -ENODEV;
	}

	ret = devm_request_threaded_irq(dev, vsync_irq,
			NULL, camera_vsync_irq_func,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"vsync_irq_in", NULL);

	if (ret < 0) {
		dev_err(dev, "Failed to request vsync irq.\n");
		return -EINVAL;
	}

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops ov2740_internal_ops = {
	.open = ov2740_open,
};
#endif

static const struct v4l2_subdev_core_ops ov2740_core_ops = {
	.s_power = ov2740_s_power,
	.ioctl = ov2740_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = ov2740_compat_ioctl32,
#endif
};

static int ov2740_get_selection(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_selection *sel)
{
	struct ov2740 *ov2740 = to_ov2740(sd);

	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EINVAL;

	switch (sel->target) {
		case V4L2_SEL_TGT_CROP_BOUNDS:
		case V4L2_SEL_TGT_CROP_DEFAULT:
		case V4L2_SEL_TGT_CROP:
			if (ov2740->cur_mode->width == 1920) {
				sel->r.left = 0;
				sel->r.top = 0;
				sel->r.width = 1920;
				sel->r.height = 1080;
			} else {
				sel->r.left = 0;
				sel->r.top = 0;
				sel->r.width = 640;
				sel->r.height = 480;
			}
			return 0;
		default:
			return -EINVAL;
	}
}

static const struct v4l2_subdev_video_ops ov2740_video_ops = {
	.s_stream = ov2740_s_stream,
	.g_frame_interval = ov2740_g_frame_interval,
	.g_mbus_config = ov2740_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops ov2740_pad_ops = {
	.enum_mbus_code = ov2740_enum_mbus_code,
	.enum_frame_size = ov2740_enum_frame_sizes,
	.enum_frame_interval = ov2740_enum_frame_interval,
	.get_selection	= ov2740_get_selection,
	.get_fmt = ov2740_get_fmt,
	.set_fmt = ov2740_set_fmt,
};

static const struct v4l2_subdev_ops ov2740_subdev_ops = {
	.core	= &ov2740_core_ops,
	.video	= &ov2740_video_ops,
	.pad	= &ov2740_pad_ops,
};

static int ov2740_update_digital_gain(struct ov2740 *ov2740, u32 d_gain)
{
	int ret = 0;

	ret = ov2740_write_reg(ov2740->client, OV2740_REG_MWB_R_GAIN,
						OV2740_REG_VALUE_16BIT, d_gain);
	if (ret)
		return ret;

	ret = ov2740_write_reg(ov2740->client, OV2740_REG_MWB_G_GAIN, 
						OV2740_REG_VALUE_16BIT, d_gain);
	if (ret)
		return ret;

	return ov2740_write_reg(ov2740->client, OV2740_REG_MWB_B_GAIN,
						OV2740_REG_VALUE_16BIT, d_gain);
}

static int ov2740_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov2740 *ov2740 = container_of(ctrl->handler,
					     struct ov2740, ctrl_handler);
	struct i2c_client *client = ov2740->client;
	s64 max;
	int ret = 0;

	dev_dbg(&client->dev, "ctrl->id(0x%x) val 0x%x\n",
		ctrl->id, ctrl->val);
	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = ov2740->cur_mode->height + ctrl->val - 8;
		__v4l2_ctrl_modify_range(ov2740->exposure,
					 ov2740->exposure->minimum, max,
					 ov2740->exposure->step,
					 ov2740->exposure->default_value);
		break;
	}
	if (pm_runtime_get(&client->dev) <= 0)
		return 0;

	switch (ctrl->id) {
		case V4L2_CID_EXPOSURE:
			{
				u32 val;

				dev_info(&client->dev, "set exposure val is %#x\n", ctrl->val);
				ret = ov2740_write_reg(ov2740->client, OV2740_REG_EXPOSURE, 3,
						ctrl->val << 4);

				ov2740_read_reg(ov2740->client, OV2740_REG_EXPOSURE_AUTO, 1, &val);
				if (val & 0x00000008) {
					dev_info(&client->dev, "get exposure mode is menual\n");
				} else {
					dev_info(&client->dev, "get exposure mode is auto\n");
				}
			}
			break;

	case V4L2_CID_EXPOSURE_AUTO:
		{
			u32 val;

			dev_info(&client->dev, "set exposure mode val is %#x\n", ctrl->val);
			ov2740_read_reg(ov2740->client, OV2740_REG_EXPOSURE_AUTO, 1, &val);

			if (ctrl->val == V4L2_EXPOSURE_MANUAL) {
				val |= 0x08;
			} else {
				val &= 0xf7;
			}

			ov2740_write_reg(ov2740->client, OV2740_REG_EXPOSURE_AUTO, 1, val);
		}
		break;

	case V4L2_CID_ANALOGUE_GAIN:
		ret = ov2740_write_reg(ov2740->client, OV2740_REG_ANALOG_GAIN, 2,
				       ctrl->val);
		break;
    case V4L2_CID_DIGITAL_GAIN:
		ret = ov2740_update_digital_gain(ov2740, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = ov2740_write_reg(ov2740->client, OV2740_REG_VTS, 2,
				       ov2740->cur_mode->height + ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = ov2740_enable_test_pattern(ov2740, ctrl->val);

		break;
	case V4L2_CID_HFLIP: {
		int mirror;
		u32 val;

		ret = ov2740_read_reg(ov2740->client, OV2740_REG_FLIP_MIRROR, 1,&val);
		mirror = ctrl->val ? (val |OV2740_REG_MIRROR_MASK) : (val & (~OV2740_REG_MIRROR_MASK));
		ret |= ov2740_write_reg(ov2740->client, OV2740_REG_FLIP_MIRROR, 1,mirror);
		}
		break;
	case V4L2_CID_VFLIP:{
		int flip;
		u32 val;

		ret = ov2740_read_reg(ov2740->client, OV2740_REG_FLIP_MIRROR, 1, &val);
		flip = ctrl->val ? (val |OV2740_REG_FLIP_MASK) : (val & (~OV2740_REG_FLIP_MASK));
		ret |= ov2740_write_reg(ov2740->client, OV2740_REG_FLIP_MIRROR, 1,flip);
		}
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ov2740_ctrl_ops = {
	.s_ctrl = ov2740_set_ctrl,
};

static int ov2740_initialize_controls(struct ov2740 *ov2740)
{
	const struct ov2740_mode *mode;
	struct v4l2_ctrl_handler *handler;
	struct v4l2_ctrl *ctrl;
	s64 exposure_max, vblank_def;
	u32 h_blank;
	int ret;

	handler = &ov2740->ctrl_handler;
	mode = ov2740->cur_mode;
	ret = v4l2_ctrl_handler_init(handler, 7);
	if (ret)
		return ret;
	handler->lock = &ov2740->mutex;

	ctrl = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, OV2740_PIXEL_RATE, 1, OV2740_PIXEL_RATE);

	h_blank = mode->hts_def - mode->width;
	ov2740->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (ov2740->hblank)
		ov2740->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts_def - mode->height;
	ov2740->vblank = v4l2_ctrl_new_std(handler, &ov2740_ctrl_ops,
				V4L2_CID_VBLANK, vblank_def,
				OV2740_VTS_MAX - mode->height,
				1, vblank_def);
	/* Auto/manual exposure */
	ov2740->auto_exp =
		v4l2_ctrl_new_std_menu(handler, &ov2740_ctrl_ops,
				       V4L2_CID_EXPOSURE_AUTO,
				       V4L2_EXPOSURE_MANUAL, 0,
				       V4L2_EXPOSURE_MANUAL);

	//max is 1112 for ov2740
	exposure_max = mode->vts_def - 4;
	ov2740->exposure = v4l2_ctrl_new_std(handler, &ov2740_ctrl_ops,
				V4L2_CID_EXPOSURE, OV2740_EXPOSURE_MIN,
				exposure_max, OV2740_EXPOSURE_STEP,
				mode->exp_def);

	ov2740->anal_gain = v4l2_ctrl_new_std(handler, &ov2740_ctrl_ops,
				V4L2_CID_ANALOGUE_GAIN, ANALOG_GAIN_MIN,
				ANALOG_GAIN_MAX, ANALOG_GAIN_STEP,
				ANALOG_GAIN_DEFAULT);

	ov2740->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
				&ov2740_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(ov2740_test_pattern_menu) - 1,
				0, 0, ov2740_test_pattern_menu);
	ov2740->h_flip = v4l2_ctrl_new_std(handler, &ov2740_ctrl_ops,
					   V4L2_CID_HFLIP, 0, 1, 1, 0);

	ov2740->v_flip = v4l2_ctrl_new_std(handler, &ov2740_ctrl_ops,
					   V4L2_CID_VFLIP, 0, 1, 1, 0);
	ov2740->flip = 0;

	if (handler->error) {
		ret = handler->error;
		dev_err(&ov2740->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}

	ov2740->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int ov2740_check_sensor_id(struct ov2740 *ov2740,
				  struct i2c_client *client)
{
	struct device *dev = &ov2740->client->dev;
	int ret = 0;
	u32 pidh = 0;
	u32 pidm = 0;
	u32 pidl = 0;

	ret |= ov2740_read_reg(ov2740->client, OV2740_PIDH_ADDR ,OV2740_REG_VALUE_08BIT, &pidh);
	ret |= ov2740_read_reg(ov2740->client, OV2740_PIDM_ADDR, OV2740_REG_VALUE_08BIT, &pidm);
	ret |= ov2740_read_reg(ov2740->client, OV2740_PIDL_ADDR,OV2740_REG_VALUE_08BIT, &pidl);
	if (ret) {
		dev_err(dev,
			"register read failed, camera module powered off?\n");
		dev_info(dev,
			"Found cameraID 0x%02x%02x%02x\n", pidh, pidm, pidl);
		goto err;
	}

	if ((pidh == OV2740_PIDH_MAGIC) && (pidm == OV2740_PIDM_MAGIC) && (pidl == OV2740_PIDL_MAGIC)) {
		dev_info(dev,
			"Found cameraID 0x%02x%02x%02x\n", pidh, pidm, pidl);
	} else {
		dev_err(dev,
			"wrong camera ID, expected 0x%02x%02x%02x, detected 0x%02x%02x%02x\n",
			OV2740_PIDH_MAGIC, OV2740_PIDM_MAGIC, OV2740_PIDL_MAGIC, pidh, pidm, pidl);
		ret = -EINVAL;
		goto err;
	}

	return 0;
err:
	dev_err(dev, "failed with error (%d)\n", ret);
	return ret;
}

static int ov2740_configure_regulators(struct ov2740 *ov2740)
{
	size_t i;

	for (i = 0; i < OV2740_NUM_SUPPLIES; i++)
		ov2740->supplies[i].supply = ov2740_supply_names[i];

	return devm_regulator_bulk_get(&ov2740->client->dev,
				       OV2740_NUM_SUPPLIES,
				       ov2740->supplies);
}

static ssize_t sensor_online_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = (struct v4l2_subdev *)dev_get_drvdata(dev);
	struct ov2740 *ov2740;

	if (IS_ERR_OR_NULL(sd))
		return sprintf(buf, "%d", false);
	else
		ov2740 = to_ov2740(sd);

	return sprintf(buf, "%d", ov2740->online);
}

static DEVICE_ATTR(sensor_online, S_IRUSR, sensor_online_get, NULL);

static int sensor_online_sysfs_register(struct device *dev)
{
	return device_create_file(dev, &dev_attr_sensor_online);
}

static ssize_t fmt_mode_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = (struct v4l2_subdev *)dev_get_drvdata(dev);
	struct ov2740 *ov2740;

	if (IS_ERR_OR_NULL(sd))
		return sprintf(buf, "%d", false);
	else
		ov2740 = to_ov2740(sd);

	return sprintf(buf, "%d", ov2740->fmt_mode);
}

static ssize_t fmt_mode_set(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	uint32_t val = 0;
	int ret;
	struct v4l2_subdev *sd = (struct v4l2_subdev *)dev_get_drvdata(dev);
	struct ov2740 *ov2740;

	ret = kstrtouint(buf, 10, &val);
	if (ret || val > 3)
		return -EINVAL;

	if (IS_ERR_OR_NULL(sd))
		return -EINVAL;
	else
		ov2740 = to_ov2740(sd);

	ov2740->fmt_mode = val;
	ov2740->cur_mode = &supported_modes[ov2740->fmt_mode];
	pr_info("set fmt mode %d\n", val);

	return count;
}

static DEVICE_ATTR(fmt_mode, S_IRUSR | S_IWUSR, fmt_mode_get, fmt_mode_set);

static int fmt_mode_sysfs_register(struct device *dev)
{
	return device_create_file(dev, &dev_attr_fmt_mode);
}

static ssize_t center_x_get(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = (struct v4l2_subdev *)dev_get_drvdata(dev);
	struct ov2740 *ov2740;

	if (IS_ERR_OR_NULL(sd))
		return sprintf(buf, "%d", false);
	else
		ov2740 = to_ov2740(sd);

	pr_info("get center x %d\n", ov2740->center_x);

	return sprintf(buf, "%d\n", ov2740->center_x);
}

static ssize_t center_x_set(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	uint32_t val = 0;
	int ret;
	struct v4l2_subdev *sd = (struct v4l2_subdev *)dev_get_drvdata(dev);
	struct ov2740 *ov2740;

	ret = kstrtouint(buf, 10, &val);
	if (ret || val > CENTER_X_MAX || val < CENTER_X_MIN) {
		pr_info("set centor x val %d invalid\n", val);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(sd))
		return -EINVAL;
	else
		ov2740 = to_ov2740(sd);

	ov2740->center_x = val;
	pr_info("set center x %d\n", val);

	return count;
}

static ssize_t center_y_get(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = (struct v4l2_subdev *)dev_get_drvdata(dev);
	struct ov2740 *ov2740;

	if (IS_ERR_OR_NULL(sd))
		return sprintf(buf, "%d", false);
	else
		ov2740 = to_ov2740(sd);

	pr_info("get center y %d\n", ov2740->center_y);

	return sprintf(buf, "%d\n", ov2740->center_y);
}

static ssize_t center_y_set(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	uint32_t val = 0;
	int ret;
	struct v4l2_subdev *sd = (struct v4l2_subdev *)dev_get_drvdata(dev);
	struct ov2740 *ov2740;

	ret = kstrtouint(buf, 10, &val);
	if (ret || val > CENTER_Y_MAX || val < CENTER_Y_MIN) {
		pr_info("set centor y val %d invalid\n", val);
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(sd))
		return -EINVAL;
	else
		ov2740 = to_ov2740(sd);

	ov2740->center_y = val;
	pr_info("set center y %d\n", val);

	return count;
}

static DEVICE_ATTR(center_x, S_IRUSR | S_IWUSR, center_x_get, center_x_set);
static DEVICE_ATTR(center_y, S_IRUSR | S_IWUSR, center_y_get, center_y_set);
static int center_sysfs_register(struct device *dev)
{
	int ret;

	ret = device_create_file(dev, &dev_attr_center_x);
	if (ret) {
		dev_err(dev, "center x register fail\n");
		goto end;
	}

	ret = device_create_file(dev, &dev_attr_center_y);
	if (ret) {
		dev_err(dev, "center y register fail\n");
		goto end;
	}

end:
	return ret;
}

static int ov2740_nvmem_read(void *priv, unsigned int off, void *val,
			     size_t count)
{
	struct nvm_data *nvm = priv;
	struct v4l2_subdev *sd = i2c_get_clientdata(nvm->client);
	struct device *dev = &nvm->client->dev;
	struct ov2740 *ov2740 = to_ov2740(sd);
	int ret = 0;

	mutex_lock(&ov2740->mutex);

	if (nvm->nvm_buffer) {
		memcpy(val, nvm->nvm_buffer + off, count);
		goto exit;
	}

	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		pm_runtime_put_noidle(dev);
		goto exit;
	}

	ret = ov2740_load_otp_data(nvm);
	if (!ret)
		memcpy(val, nvm->nvm_buffer + off, count);

	pm_runtime_put(dev);
exit:
	mutex_unlock(&ov2740->mutex);
	return ret;
}

static void dump_otp_write_data(void *priv, \
		const unsigned int off, const void *val, \
		const size_t count) {
	struct nvm_data *nvm = priv;
	struct device *dev = &nvm->client->dev;
	int i;

	if (val == NULL) {
		dev_err(dev, "err, dump ptr is null");
		return;
	}

	pr_info("write otp data offset is %d, length is %d\n", off, count);

	for (i = 0; i < count; i++) {
		pr_info(KERN_CONT "%#x ", *(u8 *)(val + i));
	}

	pr_info(KERN_CONT "\n");
}

static int ov2740_nvmem_write(void *priv, unsigned int off,
				   void *val, size_t count)
{
	struct nvm_data *nvm = priv;
	struct v4l2_subdev *sd = i2c_get_clientdata(nvm->client);
	struct device *dev = &nvm->client->dev;
	struct ov2740 *ov2740 = to_ov2740(sd);
	int ret = 0;

	if (unlikely(off >= CUSTOMER_USE_OTP_SIZE)) {
		dev_err(dev, "offset err : %d\n", off);
		return -EFBIG;
	}

	if ((off + count) > CUSTOMER_USE_OTP_SIZE) {
		dev_err(dev, "err, offset is %d, count is %d\n", \
				off, count);
		count = CUSTOMER_USE_OTP_SIZE - off;
	}

	if (unlikely(!count)) {
		dev_err(dev, "err: count is %d\n", count);
		return count;
	}

	mutex_lock(&ov2740->mutex);

	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		pm_runtime_put_noidle(dev);
		goto exit;
	}

	dump_otp_write_data(priv, off, val, count);
	ret = ov2740_store_otp_data(nvm, off, val, count);

	pm_runtime_put(dev);
exit:
	mutex_unlock(&ov2740->mutex);
	return ret;
}

static int ov2740_register_nvmem(struct i2c_client *client,
				 struct ov2740 *ov2740)
{
	struct nvm_data *nvm;
	struct regmap_config regmap_config = { };
	struct nvmem_config nvmem_config = { };
	struct regmap *regmap;
	struct device *dev = &client->dev;
	int ret;

	nvm = devm_kzalloc(dev, sizeof(*nvm), GFP_KERNEL);
	if (!nvm)
		return -ENOMEM;

	regmap_config.val_bits = 8;
	regmap_config.reg_bits = 16;
	regmap_config.disable_locking = true;
	regmap = devm_regmap_init_i2c(client, &regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	nvm->regmap = regmap;
	nvm->client = client;

	nvmem_config.name = dev_name(dev);
	nvmem_config.dev = dev;
	nvmem_config.read_only = false;//todo
	nvmem_config.root_only = true;
	nvmem_config.owner = THIS_MODULE;
	nvmem_config.compat = true;
	nvmem_config.base_dev = dev;
	nvmem_config.reg_read = ov2740_nvmem_read;
	nvmem_config.reg_write = ov2740_nvmem_write;
	nvmem_config.priv = nvm;
	nvmem_config.stride = 1;
	nvmem_config.word_size = 1;
	nvmem_config.size = CUSTOMER_USE_OTP_SIZE;

	nvm->nvmem = devm_nvmem_register(dev, &nvmem_config);

	ret = PTR_ERR_OR_ZERO(nvm->nvmem);
	if (!ret)
		ov2740->nvm = nvm;

	return ret;
}

static int ov2740_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct ov2740 *ov2740;
	struct v4l2_subdev *sd;
	char facing[2];
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);

	ov2740 = devm_kzalloc(dev, sizeof(*ov2740), GFP_KERNEL);
	if (!ov2740)
		return -ENOMEM;

	ret = sensor_online_sysfs_register(dev);
	if (ret) {
		dev_err(dev, "sensor online failed to register\n");
		return ret;
	}

	ret = fmt_mode_sysfs_register(dev);
	if (ret) {
		dev_err(dev, "fmt_mode failed to register\n");
		return ret;
	}

	ret = center_sysfs_register(dev);
	if (ret) {
		dev_err(dev, "center failed to register\n");
		return ret;
	}

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &ov2740->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &ov2740->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &ov2740->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &ov2740->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	ov2740->client = client;
	ov2740->fmt_mode = 0;
	ov2740->center_x = CENTER_X_MID;
	ov2740->center_y = CENTER_Y_MID;
	ov2740->cur_mode = &supported_modes[ov2740->fmt_mode];

	ov2740->xvclk = devm_clk_get(dev, "xvclk");
	if (IS_ERR(ov2740->xvclk)) {
		dev_err(dev, "Failed to get xvclk\n");
		return -EINVAL;
	}
	ov2740->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(ov2740->pinctrl)) {
		ov2740->pins_default =
			pinctrl_lookup_state(ov2740->pinctrl,
					     OF_CAMERA_PINCTRL_STATE_DEFAULT);
		if (IS_ERR(ov2740->pins_default))
			dev_err(dev, "could not get default pinstate\n");

	} else {
		dev_err(dev, "no pinctrl\n");
	}
	
	ov2740->power_gpio = devm_gpiod_get(dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(ov2740->power_gpio))
		dev_warn(dev, "Failed to get power-gpios\n");
	else
		gpiod_set_value_cansleep(ov2740->power_gpio, 1);

	ov2740->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ov2740->reset_gpio))
		dev_warn(dev, "Failed to get reset-gpios\n");

	ov2740->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(ov2740->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");

	ov2740->vsync_gpio = devm_gpiod_get(dev, "vsync", GPIOD_OUT_LOW);
	if (IS_ERR(ov2740->vsync_gpio))
		dev_warn(dev, "can not find vsync-gpios, error %ld\n",
			 PTR_ERR(ov2740->vsync_gpio));

	ret = ov2740_configure_regulators(ov2740);
	if (ret) {
		dev_err(dev, "Failed to get power regulators\n");
		return ret;
	}

	init_vsync_interrupt(ov2740);

	mutex_init(&ov2740->mutex);

	sd = &ov2740->subdev;
	v4l2_i2c_subdev_init(sd, client, &ov2740_subdev_ops);
	ret = ov2740_initialize_controls(ov2740);
	if (ret)
		goto err_destroy_mutex;

	ret = __ov2740_power_on(ov2740);
	if (ret)
		goto err_free_handler;

	ret = ov2740_check_sensor_id(ov2740, client);
	if (ret) {
		ov2740->online = false;
		goto err_power_off;
	} else
		ov2740->online = true;

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &ov2740_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;
#endif

#if defined(CONFIG_MEDIA_CONTROLLER)
	ov2740->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&sd->entity, 1, &ov2740->pad);
	if (ret < 0)
		goto err_power_off;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(ov2740->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(sd->name, sizeof(sd->name), "m%02d_%s_%s %s",
		 ov2740->module_index, facing,
		 OV2740_NAME, dev_name(sd->dev));
	ret = v4l2_async_register_subdev_sensor_common(sd);
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	ret = ov2740_register_nvmem(client, ov2740);
	if (ret)
		dev_err(&client->dev, "register nvmem failed, ret %d\n", ret);

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__ov2740_power_off(ov2740);
err_free_handler:
	v4l2_ctrl_handler_free(&ov2740->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&ov2740->mutex);

	return ret;
}

static int ov2740_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov2740 *ov2740 = to_ov2740(sd);

	device_remove_file(&client->dev, &dev_attr_sensor_online);
	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&ov2740->ctrl_handler);
	mutex_destroy(&ov2740->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__ov2740_power_off(ov2740);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ov2740_of_match[] = {
	{ .compatible = "ovti,ov2740" },
	{},
};
MODULE_DEVICE_TABLE(of, ov2740_of_match);
#endif

static const struct i2c_device_id ov2740_match_id[] = {
	{ "ovti,ov2740", 0 },
	{ },
};

static struct i2c_driver ov2740_i2c_driver = {
	.driver = {
		.name = OV2740_NAME,
		.pm = &ov2740_pm_ops,
		.of_match_table = of_match_ptr(ov2740_of_match),
	},
	.probe		= &ov2740_probe,
	.remove		= &ov2740_remove,
	.id_table	= ov2740_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&ov2740_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&ov2740_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("OmniVision ov2740 sensor driver");
MODULE_LICENSE("GPL v2");
