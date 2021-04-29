/*
 * aw_init.c   aw882xx codec module
 *
 *
 * Copyright (c) 2019 AWINIC Technology CO., LTD
 * Copyright (C) 2021 XiaoMi, Inc.
 *
 *  Author: Nick Li <liweilei@awinic.com.cn>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

//#define DEBUG
#include <linux/module.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/version.h>
#include <linux/workqueue.h>
#include <linux/syscalls.h>
#include <sound/control.h>
#include <linux/uaccess.h>

#include "aw882xx.h"
#include "aw_pid_1852_reg.h"
#include "aw_pid_2032_reg.h"
#include "aw_log.h"


int aw882xx_dev_i2c_write_bits(struct aw_device *aw_dev,
	unsigned char reg_addr, unsigned int mask, unsigned int reg_data)
{
	struct aw882xx *aw882xx = (struct aw882xx *)aw_dev->private_data;

	return aw882xx_i2c_write_bits(aw882xx, reg_addr, mask, reg_data);
}

int aw882xx_dev_i2c_write(struct aw_device *aw_dev,
	unsigned char reg_addr, unsigned int reg_data)
{
	struct aw882xx *aw882xx = (struct aw882xx *)aw_dev->private_data;

	return aw882xx_i2c_write(aw882xx, reg_addr, reg_data);
}

int aw882xx_dev_i2c_read(struct aw_device *aw_dev,
	unsigned char reg_addr, unsigned int *reg_data)
{
	struct aw882xx *aw882xx = (struct aw882xx *)aw_dev->private_data;

	return aw882xx_i2c_read(aw882xx, reg_addr, reg_data);
}

void aw882xx_dev_set_algo_en(struct aw_device *aw_dev)
{
	struct aw882xx *aw882xx = (struct aw882xx *)aw_dev->private_data;

	aw882xx_kcontorl_set(aw882xx);
}

//[7 : 4]: -6DB ; [3 : 0]: 0.5DB  real_value = value * 2 : 0.5db --> 1
static unsigned int aw_pid_1852_reg_val_to_db(unsigned int value)
{
	return ((value >> 4) * AW_PID_1852_VOL_STEP_DB + (value & 0x0f));
}

//[7 : 4]: -6DB ; [3 : 0]: -0.5DB reg_value = value / step << 4 + value % step ; step = 6 * 2
static unsigned int aw_pid_1852_db_val_to_reg(unsigned int value)
{
	return (((value / AW_PID_1852_VOL_STEP_DB) << 4) + (value % AW_PID_1852_VOL_STEP_DB));
}

static int aw_pid_1852_set_volume(struct aw_device *aw_dev, unsigned int value)
{
	struct aw882xx *aw882xx = (struct aw882xx *)aw_dev->private_data;
	unsigned int reg_value = 0;
	unsigned int real_value = aw_pid_1852_db_val_to_reg(value);

	/* cal real value */
	aw882xx_i2c_read(aw882xx, AW_PID_1852_HAGCCFG4_REG, &reg_value);

	aw_dev_dbg(aw882xx->dev, "value %d , 0x%x", value, real_value);

	//15 : 8] volume
	real_value = (real_value << 8) | (reg_value & 0x00ff);

	/* write value */
	aw882xx_i2c_write(aw882xx, AW_PID_1852_HAGCCFG4_REG, real_value);
	return 0;
}

static int aw_pid_1852_get_volume(struct aw_device *aw_dev, unsigned int* value)
{
	unsigned int reg_value = 0;
	unsigned int real_value = 0;
	struct aw882xx *aw882xx = (struct aw882xx *)aw_dev->private_data;

	/* read value */
	aw882xx_i2c_read(aw882xx, AW_PID_1852_HAGCCFG4_REG, &reg_value);

	//[15 : 8] volume
	real_value = reg_value >> 8;

	real_value = aw_pid_1852_reg_val_to_db(real_value);
	*value = real_value;

	return 0;
}

static void aw_pid_1852_i2s_enable(struct aw_device *aw_dev, bool flag)
{
	struct aw882xx *aw882xx = (struct aw882xx *)aw_dev->private_data;
	aw_dev_dbg(aw882xx->dev, "enter");

	if (flag) {
		aw882xx_i2c_write_bits(aw882xx, AW_PID_1852_I2SCFG1_REG,
				AW_PID_1852_I2STXEN_MASK,
				AW_PID_1852_I2STXEN_ENABLE_VALUE);
	} else {
		aw882xx_i2c_write_bits(aw882xx, AW_PID_1852_I2SCFG1_REG,
				AW_PID_1852_I2STXEN_MASK,
				AW_PID_1852_I2STXEN_DISABLE_VALUE);
	}
}

static void aw_pid_1852_update_rate(struct aw_device *aw_dev, unsigned int rate, int width)
{
	uint32_t cco_mux_value;
	int reg_value = 0;

	struct aw882xx *aw882xx = (struct aw882xx *)aw_dev->private_data;

	cco_mux_value = AW_PID_1852_I2S_CCO_MUX_EXC_8_16_32KHZ_VALUE;

	/* match rate */
	switch (rate) {
	case 8000:
		reg_value = AW_PID_1852_I2SSR_8KHZ_VALUE;
		cco_mux_value = AW_PID_1852_I2S_CCO_MUX_8_16_32KHZ_VALUE;
		break;
	case 16000:
		reg_value = AW_PID_1852_I2SSR_16KHZ_VALUE;
		cco_mux_value = AW_PID_1852_I2S_CCO_MUX_8_16_32KHZ_VALUE;
		break;
	case 32000:
		reg_value = AW_PID_1852_I2SSR_32KHZ_VALUE;
		cco_mux_value = AW_PID_1852_I2S_CCO_MUX_8_16_32KHZ_VALUE;
		break;
	case 44100:
		reg_value = AW_PID_1852_I2SSR_44P1KHZ_VALUE;
		break;
	case 48000:
		reg_value = AW_PID_1852_I2SSR_48KHZ_VALUE;
		break;
	case 96000:
		reg_value = AW_PID_1852_I2SSR_96KHZ_VALUE;
		break;
	case 192000:
		reg_value = AW_PID_1852_I2SSR_192KHZ_VALUE;
		break;
	default:
		reg_value = AW_PID_1852_I2SSR_48KHZ_VALUE;
		aw_dev_err(aw882xx->dev, "rate can not support");
		break;
	}

	aw882xx_i2c_write_bits(aw882xx, AW_PID_1852_PLLCTRL1_REG,
				AW_PID_1852_I2S_CCO_MUX_MASK, cco_mux_value);

	/* set chip rate */
	if (-1 != reg_value) {
		aw882xx_i2c_write_bits(aw882xx, AW_PID_1852_I2SCTRL_REG,
				AW_PID_1852_I2SSR_MASK, reg_value);
	}


	/* get bit width */
	aw_dev_dbg(aw882xx->dev, "width = %d", width);
	switch (width) {
	case 16:
		reg_value = AW_PID_1852_I2SFS_16_BITS_VALUE;
		break;
	case 20:
		reg_value = AW_PID_1852_I2SFS_20_BITS_VALUE;
		break;
	case 24:
		reg_value = AW_PID_1852_I2SFS_24_BITS_VALUE;
		break;
	case 32:
		reg_value = AW_PID_1852_I2SFS_32_BITS_VALUE;
		break;
	default:
		reg_value = AW_PID_1852_I2SFS_16_BITS_VALUE;
		aw_dev_err(aw882xx->dev, "width can not support");
		break;
	}

	/* get width */
	if (-1 != reg_value) {
		aw882xx_i2c_write_bits(aw882xx, AW_PID_1852_I2SCTRL_REG,
				AW_PID_1852_I2SFS_MASK, reg_value);
	}
}

static bool aw_pid_1852_check_rd_access(int reg)
{
	if (reg >= AW_PID_1852_REG_MAX) {
		return false;
	}

	if (aw_pid_1852_reg_access[reg] & AW_PID_1852_REG_RD_ACCESS) {
		return true;
	} else {
		return false;
	}
}

static bool aw_pid_1852_check_wr_access(int reg)
{
	if (reg >= AW_PID_1852_REG_MAX) {
		return false;
	}

	if (aw_pid_1852_reg_access[reg] & AW_PID_1852_REG_WR_ACCESS) {
		return true;
	} else {
		return false;
	}
}

static int aw_pid_1852_get_reg_num(void)
{
	return AW_PID_1852_REG_MAX;
}

static int aw_pid_1852_dev_init(struct aw882xx *aw882xx,  int index)
{
	struct aw_device *aw_pa = NULL;
	aw_pa = devm_kzalloc(aw882xx->dev, sizeof(struct aw_device), GFP_KERNEL);
	if (aw_pa == NULL) {
		aw_dev_err(aw882xx->dev, "dev kalloc failed");
		return -ENOMEM;
	}

	//call aw device init func
	memset(aw_pa->acf_name, 0, AW_NAME_MAX);
	memcpy(aw_pa->acf_name, AW_PID_1852_ACF_FILE, strlen(AW_PID_1852_ACF_FILE));
	memset(aw_pa->monitor_name, 0, AW_NAME_MAX);
	memcpy(aw_pa->monitor_name, AW_PID_1852_MONITOR_FILE, strlen(AW_PID_1852_MONITOR_FILE));

	aw_pa->prof_info.prof_desc = NULL;
	aw_pa->prof_info.count = 0;
	aw_pa->channel = 0;

	aw_pa->index = index;
	aw_pa->private_data = (void *)aw882xx;
	aw_pa->dev = aw882xx->dev;
	aw_pa->i2c = aw882xx->i2c;
	aw_pa->ops.aw_get_version = aw882xx_get_version;
	aw_pa->ops.aw_set_algo = aw882xx_dev_set_algo_en;
	aw_pa->ops.aw_i2c_read = aw882xx_dev_i2c_read;
	aw_pa->ops.aw_i2c_write = aw882xx_dev_i2c_write;
	aw_pa->ops.aw_i2c_write_bits = aw882xx_dev_i2c_write_bits;
	aw_pa->ops.aw_get_volume = aw_pid_1852_get_volume;
	aw_pa->ops.aw_set_volume = aw_pid_1852_set_volume;
	aw_pa->ops.aw_reg_val_to_db = aw_pid_1852_reg_val_to_db;
	aw_pa->ops.aw_i2s_enable = aw_pid_1852_i2s_enable;
	aw_pa->ops.aw_update_rate = aw_pid_1852_update_rate;
	aw_pa->ops.aw_check_rd_access = aw_pid_1852_check_rd_access;
	aw_pa->ops.aw_check_wr_access = aw_pid_1852_check_wr_access;
	aw_pa->ops.aw_get_reg_num = aw_pid_1852_get_reg_num;

	aw_pa->int_desc.reg = AW_PID_1852_SYSINTM_REG;
	aw_pa->int_desc.reg_default = AW_PID_1852_SYSINTM_DEFAULT;

	aw_pa->pwd_desc.reg = AW_PID_1852_SYSCTRL_REG;
	aw_pa->pwd_desc.mask = AW_PID_1852_PWDN_MASK;
	aw_pa->pwd_desc.enable = AW_PID_1852_PWDN_POWER_DOWN_VALUE;
	aw_pa->pwd_desc.disable = AW_PID_1852_PWDN_NORMAL_WORKING_VALUE;

	aw_pa->amppd_desc.reg = AW_PID_1852_SYSCTRL_REG;
	aw_pa->amppd_desc.mask = AW_PID_1852_AMPPD_MASK;
	aw_pa->amppd_desc.enable = AW_PID_1852_AMPPD_POWER_DOWN_VALUE;
	aw_pa->amppd_desc.disable = AW_PID_1852_AMPPD_NORMAL_WORKING_VALUE;

	aw_pa->mute_desc.reg = AW_PID_1852_SYSCTRL2_REG;
	aw_pa->mute_desc.mask = AW_PID_1852_HMUTE_MASK;
	aw_pa->mute_desc.enable = AW_PID_1852_HMUTE_ENABLE_VALUE;
	aw_pa->mute_desc.disable = AW_PID_1852_HMUTE_DISABLE_VALUE;

	aw_pa->vcalb_desc.vcalb_reg = AW_PID_1852_VTMCTRL3_REG;
	aw_pa->vcalb_desc.vcal_factor = AW_PID_1852_VCAL_FACTOR;
	aw_pa->vcalb_desc.cabl_base_value = AW_PID_1852_CABL_BASE_VALUE;

	aw_pa->vcalb_desc.icalk_value_factor = AW_PID_1852_ICABLK_FACTOR;
	aw_pa->vcalb_desc.icalk_reg = AW_PID_1852_EFRM1_REG;
	aw_pa->vcalb_desc.icalk_reg_mask = AW_PID_1852_EF_ISN_GESLP_MASK;
	aw_pa->vcalb_desc.icalk_sign_mask = AW_PID_1852_EF_ISN_GESLP_SIGN_MASK;
	aw_pa->vcalb_desc.icalk_neg_mask = AW_PID_1852_EF_ISN_GESLP_NEG;

	aw_pa->vcalb_desc.vcalk_reg = AW_PID_1852_EFRH_REG;
	aw_pa->vcalb_desc.vcalk_reg_mask = AW_PID_1852_EF_VSN_GESLP_MASK;
	aw_pa->vcalb_desc.vcalk_sign_mask = AW_PID_1852_EF_VSN_GESLP_SIGN_MASK;
	aw_pa->vcalb_desc.vcalk_neg_mask = AW_PID_1852_EF_VSN_GESLP_NEG;
	aw_pa->vcalb_desc.vcalk_value_factor = AW_PID_1852_VCABLK_FACTOR;


	aw_pa->sysst_desc.reg = AW_PID_1852_SYSST_REG;
	aw_pa->sysst_desc.mask = AW_PID_1852_SYSST_CHECK_MASK;
	aw_pa->sysst_desc.check = AW_PID_1852_SYSST_CHECK;


	aw_pa->voltage_desc.reg = AW_PID_1852_VBAT_REG;
	aw_pa->voltage_desc.int_bit = AW_PID_1852_MONITOR_INT_10BIT;
	aw_pa->voltage_desc.vbat_range = AW_PID_1852_MONITOR_VBAT_RANGE;

	aw_pa->temp_desc.reg = AW_PID_1852_TEMP_REG;
	aw_pa->temp_desc.neg_mask = AW_PID_1852_MONITOR_TEMP_NEG_MASK;
	aw_pa->temp_desc.sign_mask = AW_PID_1852_MONITOR_TEMP_SIGN_MASK;

	aw_pa->ipeak_desc.reg = AW_PID_1852_SYSCTRL2_REG;
	aw_pa->ipeak_desc.mask = AW_PID_1852_BST_IPEAK_MASK;

	aw_pa->volume_desc.reg = AW_PID_1852_HAGCCFG4_REG;
	aw_pa->volume_desc.mask = AW_PID_1852_VOL_MASK;
	aw_pa->volume_desc.shift = AW_PID_1852_VOL_START_BIT;
	aw_pa->volume_desc.mute_volume = AW_PID_1852_MUTE_VOL;

	aw_pa->soft_rst.reg = AW_PID_1852_ID_REG;
	aw_pa->soft_rst.reg_value = AW882XX_SOFT_RESET_VALUE;

	aw_device_probe(aw_pa);

	aw882xx->aw_pa = aw_pa;
	return 0;
}


//[7 : 4]: -6DB ; [3 : 0]: 0.5DB  real_value = value * 2 : 0.5db --> 1
static unsigned int aw_pid_2032_reg_val_to_db(unsigned int value)
{
	return ((value >> 6) * AW_PID_2032_VOL_STEP_DB + (value & 0x3f));
}

//[7 : 4]: -6DB ; [3 : 0]: -0.5DB reg_value = value / step << 4 + value % step ; step = 6 * 2
static unsigned int aw_pid_2032_db_val_to_reg(unsigned int value)
{
	return (((value / AW_PID_2032_VOL_STEP_DB) << 6) + (value % AW_PID_2032_VOL_STEP_DB));
}


static int aw_pid_2032_set_volume(struct aw_device *aw_dev, unsigned int value)
{
	struct aw882xx *aw882xx = (struct aw882xx *)aw_dev->private_data;
	unsigned int reg_value = 0;
	unsigned int real_value = aw_pid_2032_db_val_to_reg(value);

	/* cal real value */
	aw882xx_i2c_read(aw882xx, AW_PID_2032_SYSCTRL2_REG, &reg_value);

	aw_dev_dbg(aw882xx->dev, "value %d , 0x%x", value, real_value);

	//15 : 6] volume
	real_value = (real_value << 6) | (reg_value & 0x003f);

	/* write value */
	aw882xx_i2c_write(aw882xx, AW_PID_2032_SYSCTRL2_REG, real_value);
	return 0;
}

static int aw_pid_2032_get_volume(struct aw_device *aw_dev, unsigned int* value)
{
	unsigned int reg_value = 0;
	unsigned int real_value = 0;
	struct aw882xx *aw882xx = (struct aw882xx *)aw_dev->private_data;

	/* read value */
	aw882xx_i2c_read(aw882xx, AW_PID_2032_SYSCTRL2_REG, &reg_value);

	//[15 : 6] volume
	real_value = reg_value >> 6;

	real_value = aw_pid_2032_reg_val_to_db(real_value);
	*value = real_value;

	return 0;
}

static void aw_pid_2032_i2s_enable(struct aw_device *aw_dev, bool flag)
{
	struct aw882xx *aw882xx = (struct aw882xx *)aw_dev->private_data;
	aw_dev_dbg(aw882xx->dev, "enter");

	if (flag) {
		aw882xx_i2c_write_bits(aw882xx, AW_PID_2032_I2SCFG1_REG,
				AW_PID_2032_I2STXEN_MASK,
				AW_PID_2032_I2STXEN_ENABLE_VALUE);
	} else {
		aw882xx_i2c_write_bits(aw882xx, AW_PID_2032_I2SCFG1_REG,
				AW_PID_2032_I2STXEN_MASK,
				AW_PID_2032_I2STXEN_DISABLE_VALUE);
	}
}

static void aw_pid_2032_update_rate(struct aw_device *aw_dev, unsigned int rate, int width)
{
	uint32_t cco_mux_value;
	int reg_value = 0;

	struct aw882xx *aw882xx = (struct aw882xx *)aw_dev->private_data;
	cco_mux_value = AW_PID_2032_CCO_MUX_BYPASS_VALUE;

	/* match rate */
	switch (rate) {
	case 8000:
		reg_value = AW_PID_2032_I2SSR_8_KHZ_VALUE;
		cco_mux_value = AW_PID_2032_CCO_MUX_DIVIDED_VALUE;
		break;
	case 16000:
		reg_value = AW_PID_2032_I2SSR_16_KHZ_VALUE;
		cco_mux_value = AW_PID_2032_CCO_MUX_DIVIDED_VALUE;
		break;
	case 32000:
		reg_value = AW_PID_2032_I2SSR_32_KHZ_VALUE;
		cco_mux_value = AW_PID_2032_CCO_MUX_DIVIDED_VALUE;
		break;
	case 44100:
		reg_value = AW_PID_2032_I2SSR_44_KHZ_VALUE;
		break;
	case 48000:
		reg_value = AW_PID_2032_I2SSR_48_KHZ_VALUE;
		break;
	case 96000:
		reg_value = AW_PID_2032_I2SSR_96_KHZ_VALUE;
		break;
	default:
		reg_value = AW_PID_2032_I2SSR_48_KHZ_VALUE;
		aw_dev_err(aw882xx->dev, "rate can not support");
		break;
	}

	aw882xx_i2c_write_bits(aw882xx, AW_PID_2032_PLLCTRL1_REG,
				AW_PID_2032_CCO_MUX_MASK, cco_mux_value);

	/* set chip rate */
	if (-1 != reg_value) {
		aw882xx_i2c_write_bits(aw882xx, AW_PID_2032_I2SCTRL_REG,
				AW_PID_2032_I2SSR_MASK, reg_value);
	}

	/* get bit width */
	aw_dev_dbg(aw882xx->dev, "width = %d", width);
	switch (width) {
	case 16:
		reg_value = AW_PID_2032_I2SFS_16_BITS_VALUE;
		break;
	case 20:
		reg_value = AW_PID_2032_I2SFS_20_BITS_VALUE;
		break;
	case 24:
		reg_value = AW_PID_2032_I2SFS_24_BITS_VALUE;
		break;
	case 32:
		reg_value = AW_PID_2032_I2SFS_32_BITS_VALUE;
		break;
	default:
		reg_value = AW_PID_2032_I2SFS_16_BITS_VALUE;
		aw_dev_err(aw882xx->dev, "width can not support");
		break;
	}

	/* get width */
	if (-1 != reg_value) {
		aw882xx_i2c_write_bits(aw882xx, AW_PID_2032_I2SCTRL_REG,
				AW_PID_2032_I2SFS_MASK, reg_value);
	}
}

static bool aw_pid_2032_check_rd_access(int reg)
{
	if (reg >= AW_PID_2032_REG_MAX) {
		return false;
	}

	if (aw_pid_2032_reg_access[reg] & AW_PID_2032_REG_RD_ACCESS) {
		return true;
	} else {
		return false;
	}
}

static bool aw_pid_2032_check_wr_access(int reg)
{
	if (reg >= AW_PID_2032_REG_MAX) {
		return false;
	}

	if (aw_pid_2032_reg_access[reg] & AW_PID_2032_REG_WR_ACCESS) {
		return true;
	} else {
		return false;
	}
}

static int aw_pid_2032_get_reg_num(void)
{
	return AW_PID_2032_REG_MAX;
}

static int aw_pid_2032_dev_init(struct aw882xx *aw882xx,  int index)
{
	struct aw_device *aw_pa = NULL;
	aw_pa = devm_kzalloc(aw882xx->dev, sizeof(struct aw_device), GFP_KERNEL);
	if (aw_pa == NULL) {
		aw_dev_err(aw882xx->dev, "dev kalloc failed");
		return -ENOMEM;
	}

	//call aw device init func
	memset(aw_pa->acf_name, 0, AW_NAME_MAX);
	memcpy(aw_pa->acf_name, AW_PID_2032_ACF_FILE, strlen(AW_PID_2032_ACF_FILE));
	memset(aw_pa->monitor_name, 0, AW_NAME_MAX);
	memcpy(aw_pa->monitor_name, AW_PID_2032_MONITOR_FILE, strlen(AW_PID_2032_MONITOR_FILE));

	aw_pa->prof_info.prof_desc = NULL;
	aw_pa->prof_info.count = 0;
	aw_pa->channel = 0;

	aw_pa->index = index;
	aw_pa->private_data = (void *)aw882xx;
	aw_pa->dev = aw882xx->dev;
	aw_pa->i2c = aw882xx->i2c;
	aw_pa->ops.aw_get_version = aw882xx_get_version;
	aw_pa->ops.aw_set_algo = aw882xx_dev_set_algo_en;
	aw_pa->ops.aw_i2c_read = aw882xx_dev_i2c_read;
	aw_pa->ops.aw_i2c_write = aw882xx_dev_i2c_write;
	aw_pa->ops.aw_i2c_write_bits = aw882xx_dev_i2c_write_bits;
	aw_pa->ops.aw_get_volume = aw_pid_2032_get_volume;
	aw_pa->ops.aw_set_volume = aw_pid_2032_set_volume;
	aw_pa->ops.aw_reg_val_to_db = aw_pid_2032_reg_val_to_db;
	aw_pa->ops.aw_i2s_enable = aw_pid_2032_i2s_enable;
	aw_pa->ops.aw_update_rate = aw_pid_2032_update_rate;
	aw_pa->ops.aw_check_rd_access = aw_pid_2032_check_rd_access;
	aw_pa->ops.aw_check_wr_access = aw_pid_2032_check_wr_access;
	aw_pa->ops.aw_get_reg_num = aw_pid_2032_get_reg_num;

	aw_pa->int_desc.reg = AW_PID_2032_SYSINTM_REG;
	aw_pa->int_desc.reg_default = AW_PID_2032_SYSINTM_DEFAULT;

	aw_pa->pwd_desc.reg = AW_PID_2032_SYSCTRL_REG;
	aw_pa->pwd_desc.mask = AW_PID_2032_PWDN_MASK;
	aw_pa->pwd_desc.enable = AW_PID_2032_PWDN_POWER_DOWN_VALUE;
	aw_pa->pwd_desc.disable = AW_PID_2032_PWDN_WORKING_VALUE;

	aw_pa->amppd_desc.reg = AW_PID_2032_SYSCTRL_REG;
	aw_pa->amppd_desc.mask = AW_PID_2032_AMPPD_MASK;
	aw_pa->amppd_desc.enable = AW_PID_2032_AMPPD_POWER_DOWN_VALUE;
	aw_pa->amppd_desc.disable = AW_PID_2032_AMPPD_WORKING_VALUE;

	aw_pa->mute_desc.reg = AW_PID_2032_SYSCTRL2_REG;
	aw_pa->mute_desc.mask = AW_PID_2032_HMUTE_MASK;
	aw_pa->mute_desc.enable = AW_PID_2032_HMUTE_ENABLE_VALUE;
	aw_pa->mute_desc.disable = AW_PID_2032_HMUTE_DISABLE_VALUE;

	aw_pa->vcalb_desc.vcalb_reg = AW_PID_2032_VTMCTRL3_REG;
	aw_pa->vcalb_desc.vcal_factor = AW_PID_2032_VCAL_FACTOR;
	aw_pa->vcalb_desc.cabl_base_value = AW_PID_2032_CABL_BASE_VALUE;

	aw_pa->vcalb_desc.icalk_value_factor = AW_PID_2032_ICABLK_FACTOR;
	aw_pa->vcalb_desc.icalk_reg = AW_PID_2032_EFRM1_REG;
	aw_pa->vcalb_desc.icalk_reg_mask = AW_PID_2032_EF_ISN_GESLP_MASK;
	aw_pa->vcalb_desc.icalk_sign_mask = AW_PID_2032_EF_ISN_GESLP_SIGN_MASK;
	aw_pa->vcalb_desc.icalk_neg_mask = AW_PID_2032_EF_ISN_GESLP_NEG;

	aw_pa->vcalb_desc.vcalk_reg = AW_PID_2032_EFRH_REG;
	aw_pa->vcalb_desc.vcalk_reg_mask = AW_PID_2032_EF_VSN_GESLP_MASK;
	aw_pa->vcalb_desc.vcalk_sign_mask = AW_PID_2032_EF_VSN_GESLP_SIGN_MASK;
	aw_pa->vcalb_desc.vcalk_neg_mask = AW_PID_2032_EF_VSN_GESLP_NEG;
	aw_pa->vcalb_desc.vcalk_value_factor = AW_PID_2032_VCABLK_FACTOR;


	aw_pa->sysst_desc.reg = AW_PID_2032_SYSST_REG;
	aw_pa->sysst_desc.mask = AW_PID_2032_SYSST_CHECK_MASK;
	aw_pa->sysst_desc.check = AW_PID_1852_SYSST_CHECK;


	aw_pa->voltage_desc.reg = AW_PID_2032_VBAT_REG;
	aw_pa->voltage_desc.int_bit = AW_PID_2032_MONITOR_INT_10BIT;
	aw_pa->voltage_desc.vbat_range = AW_PID_2032_MONITOR_VBAT_RANGE;

	aw_pa->temp_desc.reg = AW_PID_2032_TEMP_REG;
	aw_pa->temp_desc.neg_mask = AW_PID_2032_MONITOR_TEMP_NEG_MASK;
	aw_pa->temp_desc.sign_mask = AW_PID_2032_MONITOR_TEMP_SIGN_MASK;

	aw_pa->ipeak_desc.reg = AW_PID_2032_SYSCTRL2_REG;
	aw_pa->ipeak_desc.mask = AW_PID_2032_BST_IPEAK_MASK;

	aw_pa->volume_desc.reg = AW_PID_2032_SYSCTRL2_REG;
	aw_pa->volume_desc.mask = AW_PID_2032_VOL_MASK;
	aw_pa->volume_desc.shift = AW_PID_2032_VOL_START_BIT;
	aw_pa->volume_desc.mute_volume = AW_PID_2032_MUTE_VOL;

	aw_pa->soft_rst.reg = AW_PID_2032_ID_REG;
	aw_pa->soft_rst.reg_value = AW882XX_SOFT_RESET_VALUE;

	aw_device_probe(aw_pa);

	aw882xx->aw_pa = aw_pa;
	return 0;
}


int aw882xx_init(struct aw882xx *aw882xx, int index)
{
	if (aw882xx->chip_id == PID_1852_ID) {
		return aw_pid_1852_dev_init(aw882xx, index);
	} else if (aw882xx->chip_id == PID_2032_ID) {
		return aw_pid_2032_dev_init(aw882xx, index);
	}

	aw_dev_err(aw882xx->dev, "unsupported chip id %d", aw882xx->chip_id);
	return -EINVAL;
}

