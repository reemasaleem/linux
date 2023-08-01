// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD405x ADC driver
 *
 * Copyright (c) 2023 Analog Devices Inc.
 *
 * Jorge Marques <jorge.marques@analog.com>
 */

#ifndef _AD405X_H_
#define _AD405X_H_

#include <linux/iio/iio.h>

#define AD405X_I3C  0
#define AD405X_SPI  1

/* AD405X register definitions */
#define AD405X_REG_ENTER_ADC_MODE	0x20 // SPI only
#define AD405X_REG_ADC_MODES		0x21
#define AD405X_REG_ADC_CONFIG		0x22
#define AD405X_REG_AVG_CONFIG		0x23
#define AD405X_REG_GPIO_CONFIG		0x24
#define AD405X_REG_INTR_CONFIG		0x25
#define AD405X_REG_CHOP_CONFIG		0x26
#define AD405X_REG_TIMER_CONFIG		0x27
#define AD405X_REG_MAX_THRESH(i)	(0x28 + i) // index 0-1
#define AD405X_REG_MIN_THRESH(i)	(0x2a + i) // index 0-1
#define AD405X_REG_MAX_HYST		0x2c
#define AD405X_REG_MIN_HYST		0x2d
#define AD405X_REG_MON_VAL(i)		(0x2e + i) // index 0-1
#define AD405X_REG_INTERFACE_IBI_EN	0x30 // I3C only
#define AD405X_REG_ADC_IBI_EN		0x31 // I3C only
#define AD405X_REG_FUSE_CRC		0x40
#define AD405X_REG_STATUS		0x41
#define AD405X_REG_MAX_INTR(i)		(0x42 + i) // index 0-1
#define AD405X_REG_MIN_INTR(i)		(0x44 + i) // index 0-1
#define AD405X_REG_TGT_ADDR		0x46 // I3C only
#define AD405X_REG_GRP_ADDR		0x47 // I3C only
#define AD405X_REG_IBI_STATUS		0x48 // I3C only
#define AD405X_REG_CONV_READ(i)		(0x50 + i) // I3C only, index 0-3
#define AD405X_REG_CONV_TRIGGER(i)	(0x54 + i) // I3C only, index 0-3

#define AD405X_ENTER_ADC_MODE		BIT(0) // SPI only
#define AD405X_DATA_FORMAT_SIGNED	BIT(7)
#define AD405X_SIGN_EXT			BIT(6) // Don't care I3C
// ADC_FRAME_LEN
// 0 = Full Result Mode
// 1 = 1-Byte Mode
// 2 = 2-Byte Mode
// 3 = INVALID (Full Result Mode)
#define AD405X_ADC_FRAME_LEN(i)		(GENMASK(5,4) & i << 4) // Don't care I3C
#define AD405X_AUTO_MODE		BIT(3)
// ADC_MODE
// 0 = Normal
// 1 = Burst conversion
// 2 = Averaging mode, SPI only
// 3 = (if AUTO_MODE == 1: non-)persistent autonomous mode, SPI only
#define AD405X_ADC_MODE(m)		(GENMASK(1,0) & m) // ([1] Don't care I3C)
// AD405X_REG_ENTER_ADC_MODE =
// AD405X_DATA_FORMAT_SIGNED | AD405X_SIGN_EXT | AD405X_ADC_FRAME_LEN(f_len) |
// AD405X_AUTO_MODE | AD405X_ADC_MODE

#define AD405X_REF_SEL_VDD		BIT(5)
#define AD405X_SCALE_EN			BIT(4)
// INP_MUX_SEL
// 0 = IN+, IN-,
// 1 = REFIN,
// 2 = VDD/2,
// 3 = CLDO
#define AD405X_INP_MUX_SEL(m)		(GENMASK(1,0) & m) // ([1] Don't care I3C)
// AD405X_REG_ADC_CONFIG = AD405X_REF_SEL_VDD | AD405X_SCALE_EN | AD405X_INP_MUX_SEL(m)

// AD405X_AVG_WIN_LEN
// Value = 2^(a+1), max 256 (a = 7) for 12,14-bits and 4096 (a = 11) for 16-bits
// Anything higher for 16-bits is 4096
#define AD405X_AVG_WIN_LEN(l)		(GENMASK(3,0) & l)  // ([3] Don't care for 12,14-bits
// AD405X_REG_AVG_CONFIG = AD405X_AVG_WIN_LEN(a)

#define AD405X_DEV_EN_POL		BIT(7)
// AD405X_GPIO_1_MODE
// 0 = Disabled/High-Z (default)
// 1 = GPIO_0_INTR
// 2 = RDYb
// 3 = DEV_EN
// 4 = CHOP
// 5 = GPIO_STAT_GND
// 6 = GPIO_STAT_VDIO
// 7 = SYNC_DEMOD
#define AD405X_GPIO_1_MODE(m)		(GENMASK(6,4) & (m << 4))
#define AD405X_INVERT_ON_CHOP		BIT(3)
// AD405X_GPIO_0_MODE
// 0 = Disabled/High-Z
// 1 = GPIO_1_INTR
// 2 = RDYb
// 3 = DEV_EN
// 4 = CHOP
// 5 = GPIO_STAT_GND
// 6 = GPIO_STAT_VDIO
// 7 = DEV_RDY (default)
#define AD405X_GPIO_0_MODE(m)		(GENMASK(2,0) & m)
// AD405X_REG_GPIO_CONFIG =
// AD405X_DEV_EN_POL | AD405X_GPIO_1_MODE | AD405X_INVERT_ON_CHOP | AD405X_GPIO_0_MODE

#define AD405X_GPIO_1_INTR_SEL(m)	(GENMASK(5,4) & (m << 4))
#define AD405X_GPIO_0_INTR_SEL(m)	(GENMASK(2,0) & m)
// AD405X_REG_INTR_CONFIG = AD405X_GPIO_1_INTR_SEL | AD405X_GPIO_0_INTR_SEL

// AD405X_AVG_WIN_LEN
// Chop window size = 2^(l)
#define AD405X_CHOP_WIN_LEN(l)		(GENMASK(3,0) & l) // ([3] Don't care for 12,14-bits
// AD405X_REG_CHOP_CONFIG = AD405X_CHOP_WIN_LEN(l)

// AD405X_FS_BURST_AUTO
//  0 = 2 MSPS (DEFAULT)
//  1 = 1 MSPS
//  2 = 303 kSPS
//  3 = 100 kSPS
//  4 = 33.3 kSPS
//  5 = 10 kSPS
//  6 = 3.03 kSPS
//  7 = 1 kSPS
//  8 = 0.5 kSPS
//  9 = 0.33 kSPS
// 10 = 0.25 kSPS
// 11 = 0.2 kSPS
// 12 = 0.166 kSPS
// 13 = 0.14 kSPS
// 14 = 0.125 kSPS,
// 15 = 0.111 kSPS
#define AD405X_FS_BURST_AUTO(s)		(GENMASK(7,4) & (s << 4))
// AD405X_TIMER_PWR_ON
//  0 = 500 ns (Default),
//  1 = 1 us,
//  2 = 3.3 us,
//  3 = 10 us,
//  4 = 30 us,
//  5 = 100 us,
//  6 = 330 us,
//  7 = 1000 us,
//  8 = 2000 us,
//  9 = 3000 us,
// 10 = 4000 us,
// 11 = 5000 us,
// 12 = 6000 us,
// 13 = 7000 us,
// 14 = 8000 us,
// 15 = 9000 us
#define AD405X_TIMER_PWR_ON(s)		(GENMASK(3,0) & s)
// AD405X_REG_TIMER_CONFIG = AD405X_FS_BURST_AUTO | AD405X_TIMER_PWR_ON

// MAX/MIN_THRESH_VAL are 12-bits values, 2-complementary (DATA_FORMAT_SIGNED) or straight binary, further handle sign in method
#define AD405X_THRESH_VAL_0(v)		v
#define AD405X_THRESH_VAL_1(v)		(GENMASK(3,0) & (v >> 8))
// AD405X_REG_*_THRESH(0) = AD405X_THRESH_VAL_0(v)
// AD405X_REG_*_THRESH(1) = AD405X_THRESH_VAL_1(v)

// *_THERSH_HYST are 7-bits values, straight binary
#define AD405X_THRESH_HYST(v)		(GENMASK(6,0) & v)
// AD405X_REG_*_HYST = AD405X_THRESH_HYST(v)

// AD405X_MON_VAL (RO)
// Measured source value = {0,2^24-1} = v
// Measured value in volts = 2 * VREF * AD405X_MON_VAL
#define AD405X_MON_VAL(v)		v
// AD405X_REG_MON_VAL = AD405X_MON_VAL

#define AD405X_PAR_ERR_IBI_EN			BIT(5)
#define AD405X_CLOCK_COUNT_ERR_IBI_EN		BIT(4)
#define AD405X_CRC_ERR_IBI_EN			BIT(3)
#define AD405X_WR_TO_RD_ONLY_ERR_IBI_EN		BIT(2)
#define AD405X_PARTIAL_ACCESS_ERR_IBI_EN	BIT(1)
#define AD405X_ADDR_INVALID_ERR_IBI_EN		BIT(0)
// AD405X_REG_INTERFACE_IBI_EN = AD405X_PAR_ERR_IBI_EN | AD405X_CLOCK_COUNT_ERR_IBI_EN	AD405X_CRC_ERR_IBI_EN | AD405X_WR_TO_RD_ONLY_ERR_IBI_EN | AD405X_PARTIAL_ACCESS_ERR_IBI_EN | AD405X_ADDR_INVALID_ERR_IBI_EN

#define AD4050X_CONV_TRIGGER_IBI_EN		BIT(2)
#define AD4050X_MAX_IBI_EN			BIT(1)
#define AD4050X_MIN_IBI_EN			BIT(0)
// AD405X_REG_ADC_IBI_EN = CONV_TRIGGER_IBI_EN | MAX_IBI_EN | MIN_IBI_EN

// W AD405X_FUSE_CRC_EN to trigger CRC check, R it to see status (0 == finished CRC,AD405X_FUSE_CRC_ERR field updated)
// RO AD405X_FUSE_CRC_ERR
#define AD405X_FUSE_CRC_ERR			BIT(1)
#define AD405X_FUSE_CRC_EN			BIT(0)
// AD405X_REG_FUSE_CRC = AD405X_FUSE_CRC_EN

// AD405X_INTERFACE_ERR is | of * interface error, that's why it is RO
#define AD405X_DEVICE_READY(v)			((v >> 7) & BIT(0))
#define AD405X_DEVICE_RESET(v)			((v >> 6) & BIT(0)) // W1C
#define AD405X_INTERFACE_ERR(v)			((v >> 5) & BIT(0))
#define AD405X_THRESH_OVERRUN(v)		((v >> 4) & BIT(0)) // W1C
#define AD405X_MAX_THRESH_INTR(v)		((v >> 3) & BIT(0)) // W1C
#define AD405X_MIN_THRESH_INTR(v)		((v >> 2) & BIT(0)) // W1C
#define AD405X_OVER_RNG_ERR(v)			((v >> 1) & BIT(0)) // W1C
#define AD405X_UNDER_RNG_ERR(v)			((v >> 0) & BIT(0)) // W1C
#define AD405X_STATUS_CLR_ALL			(BIT(6) & GENMASK(4,0))
// AD405X_REG_STATUS

// MAX and MIN Threshold Events, only for Non-Persistent Autonomous Mode
// u16 *_intr_val = readl(AD405X_REG_*_INTR(1)) << 8 & readl(AD405X_REG_*_INTR(0))

#define AD405X_TGT_ADDR(v)		(GENMASK(7,0) & v)
#define AD405X_GRP_NOT_ASSIGNED(v)	(BIT(0) & (v >> 7))
#define AD405X_GRP_ADDR(v)		(GENMASK(6,0) & v)
#define AD405X_IBI_PENDING(v)		(BIT(0) & (v >> 1))
#define AD405X_IBI_EN(v)		(BIT(0) & v)

// For CONV_READ and CONV_AUTO
// "Users shall be allowed to set the Address Pointer to one of the “middle” bytes of the CONV_READ register,
// in order to determine the number of data and/or sign extension bytes they want to read out.
// For example, if the CONV_READ register exists at address 0x49 to 0x4C, the user can set the Address Pointer to 0x4A,
// in which case the following Target Read Request will still trigger a convert-start, and the first byte read out will
// correspond to DATA[15:8]. This is expected to be used with address descending. It is acceptable that this
// would trigger a strict entity access error if strict entity access is enabled.

//extern const struct regmap_access_table ad405x_spi_readable_regs_table;
extern const struct regmap_access_table ad405x_i3c_readable_regs_table;

//extern const struct regmap_access_table ad405x_spi_writable_regs_table;
extern const struct regmap_access_table ad405x_i3c_writable_regs_table;

enum ad405x_device_type {
	// SPI
	//AD4050,
	//AD4051,
	//AD4052,
	// I3C
	AD4053,
	AD4054,
	AD4055,
};

struct ad405x_data {
	struct regmap	*regmap;
	const struct	ad405x_chip_info *chip_info;
	struct mutex	lock; /* lock to protect transf_buf */
	int samp_freq;
	__le16		transf_buf ____cacheline_aligned;
};

struct ad405x_chip_info {
	const char		*name;
	enum ad405x_device_type	type;
	bool			soft_reset;
	int interface;
};

extern const struct ad405x_chip_info ad405x_chip_info[];

int ad405x_core_probe(struct device *dev,
		       struct regmap *regmap,
		       const struct ad405x_chip_info *chip_info,
		       int (*setup)(struct device *, struct regmap *));

#endif /* _AD405X_H_ */
