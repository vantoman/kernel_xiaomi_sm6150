/*
 * Copyright (C) 2010 - 2018 Novatek, Inc.
 *
 * $Revision: 32206 $
 * $Date: 2018-08-10 19:23:04 +0800 (週五, 10 八月 2018) $
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/uaccess.h>
#include <linux/input/mt.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/double_click.h>
#include <linux/spi-xiaomi-tp.h>

#include <linux/msm_drm_notify.h>

#include "nt36xxx.h"

/*Input event*/
#define INPUT_EVENT_START			0
#define INPUT_EVENT_SENSITIVE_MODE_OFF		0
#define INPUT_EVENT_SENSITIVE_MODE_ON		1
#define INPUT_EVENT_STYLUS_MODE_OFF		2
#define INPUT_EVENT_STYLUS_MODE_ON		3
#define INPUT_EVENT_WAKEUP_MODE_OFF		4
#define INPUT_EVENT_WAKEUP_MODE_ON		5
#define INPUT_EVENT_COVER_MODE_OFF		6
#define INPUT_EVENT_COVER_MODE_ON		7
#define INPUT_EVENT_SLIDE_FOR_VOLUME		8
#define INPUT_EVENT_DOUBLE_TAP_FOR_VOLUME		9
#define INPUT_EVENT_SINGLE_TAP_FOR_VOLUME		10
#define INPUT_EVENT_LONG_SINGLE_TAP_FOR_VOLUME		11
#define INPUT_EVENT_PALM_OFF		12
#define INPUT_EVENT_PALM_ON		13
#define INPUT_EVENT_END				13

struct nvt_ts_data *ts;

#if BOOT_UPDATE_FIRMWARE
static struct workqueue_struct *nvt_fwu_wq;
static struct workqueue_struct *nvt_lockdown_wq;
extern void Boot_Update_Firmware(struct work_struct *work);
#endif

static int nvt_drm_notifier_callback(struct notifier_block *self, unsigned long event, void *data);

static int32_t nvt_ts_suspend(struct device *dev);
static int32_t nvt_ts_resume(struct device *dev);

uint32_t ENG_RST_ADDR  = 0x7FFF80;
uint32_t SWRST_N8_ADDR;
uint32_t SPI_RD_FAST_ADDR;

#if WAKEUP_GESTURE
const uint16_t gesture_key_array[] = {
	KEY_POWER,  //GESTURE_WORD_C
	KEY_POWER,  //GESTURE_WORD_W
	KEY_POWER,  //GESTURE_WORD_V
	KEY_WAKEUP,  //GESTURE_DOUBLE_CLICK
	KEY_POWER,  //GESTURE_WORD_Z
	KEY_POWER,  //GESTURE_WORD_M
	KEY_POWER,  //GESTURE_WORD_O
	KEY_POWER,  //GESTURE_WORD_e
	KEY_POWER,  //GESTURE_WORD_S
	KEY_POWER,  //GESTURE_SLIDE_UP
	KEY_POWER,  //GESTURE_SLIDE_DOWN
	KEY_POWER,  //GESTURE_SLIDE_LEFT
	KEY_POWER,  //GESTURE_SLIDE_RIGHT
};
#endif

static uint8_t bTouchIsAwake;
/*******************************************************
Description:
	Novatek touchscreen irq enable/disable function.

return:
	n.a.
*******************************************************/
static void nvt_irq_enable(bool enable)
{
	struct irq_desc *desc;

	if (enable) {
		if (!ts->irq_enabled) {
			enable_irq(ts->client->irq);
			ts->irq_enabled = true;
		}
	} else {
		if (ts->irq_enabled) {
			disable_irq(ts->client->irq);
			ts->irq_enabled = false;
		}
	}

	desc = irq_to_desc(ts->client->irq);
	NVT_LOG("enable=%d, desc->depth=%d\n", enable, desc->depth);
}

/*******************************************************
Description:
	Novatek touchscreen spi read/write core function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static inline int32_t spi_read_write(struct spi_device *client, uint8_t *buf, size_t len, NVT_SPI_RW rw)
{
	struct spi_message m;
	struct spi_transfer t = {
		.len    = len,
	};

	memcpy(ts->xbuf, buf, len);

	switch (rw) {
		case NVTREAD:
			t.tx_buf = ts->xbuf;
			t.rx_buf = ts->rbuf;
			t.len    = (len + DUMMY_BYTES);
			break;

		case NVTWRITE:
			t.tx_buf = ts->xbuf;
			break;
	}
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	return spi_sync(client, &m);
}

/*******************************************************
Description:
	Novatek touchscreen spi read function.

return:
	Executive outcomes. 2---succeed. -5---I/O error
*******************************************************/
int32_t CTP_SPI_READ(struct spi_device *client, uint8_t *buf, uint16_t len)
{
	int32_t ret = -1;

	mutex_lock(&ts->xbuf_lock);

	buf[0] = SPI_READ_MASK(buf[0]);

	ret = spi_read_write(client, buf, len, NVTREAD);
	if (ret != 0) {
		NVT_ERR("read error, ret=%d\n", ret);
		ret = -EIO;
	} else {
		memcpy((buf+1), (ts->rbuf+2), (len-1));
	}

	mutex_unlock(&ts->xbuf_lock);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen spi write function.

return:
	Executive outcomes. 1---succeed. -5---I/O error
*******************************************************/
int32_t CTP_SPI_WRITE(struct spi_device *client, uint8_t *buf, uint16_t len)
{
	int32_t ret = -1;

	mutex_lock(&ts->xbuf_lock);
	buf[0] = SPI_WRITE_MASK(buf[0]);
	
	ret = spi_read_write(client, buf, len, NVTWRITE);
	if (ret != 0) {
		NVT_ERR("error, ret=%d\n", ret);
		ret = -EIO;
	}
	mutex_unlock(&ts->xbuf_lock);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen set index/page/addr address.

return:
	Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
int32_t nvt_set_page(uint32_t addr)
{
	uint8_t buf[4] = {0};

	buf[0] = 0xFF;	//set index/page/addr command
	buf[1] = (addr >> 15) & 0xFF;
	buf[2] = (addr >> 7) & 0xFF;

	return CTP_SPI_WRITE(ts->client, buf, 3);
}

/*******************************************************
Description:
	Novatek touchscreen write data to specify address.

return:
	Executive outcomes. 0---succeed. -5---access fail.
*******************************************************/
int32_t nvt_write_addr(uint32_t addr, uint8_t data)
{
	int32_t ret = 0;
	uint8_t buf[4] = {0};

	NVT_ERR("nvt_write_addr enter\n");
	/* ---set xdata index--- */
	buf[0] = 0xFF;	/* set index/page/addr command */
	buf[1] = (addr >> 15) & 0xFF;
	buf[2] = (addr >> 7) & 0xFF;
	ret = CTP_SPI_WRITE(ts->client, buf, 3);
	if (ret) {
		NVT_ERR("set page 0x%06X failed, ret = %d\n", addr, ret);
		return ret;
	}

	/* ---write data to index--- */
	buf[0] = addr & (0x7F);
	buf[1] = data;
	ret = CTP_SPI_WRITE(ts->client, buf, 2);
	if (ret) {
		NVT_ERR("write data to 0x%06X failed, ret = %d\n", addr, ret);
		return ret;
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen enable hw bld crc function.

return:
	N/A.
*******************************************************/
void nvt_bld_crc_enable(void)
{
	uint8_t buf[2] = {0};

	/* ---set xdata index to BLD_CRC_EN_ADDR--- */
	nvt_set_page(ts->mmap->BLD_CRC_EN_ADDR);

	//---read data from index---
	buf[0] = ts->mmap->BLD_CRC_EN_ADDR & (0x7F);
	buf[1] = 0xFF;
	CTP_SPI_READ(ts->client, buf, 2);

	//---write data to index---
	buf[0] = ts->mmap->BLD_CRC_EN_ADDR & (0x7F);
	buf[1] = buf[1] | (0x01 << 7);
	CTP_SPI_WRITE(ts->client, buf, 2);
}

/*******************************************************
Description:
	Novatek touchscreen clear status & enable fw crc function.

return:
	N/A.
*******************************************************/
void nvt_fw_crc_enable(void)
{
	uint8_t buf[2] = {0};

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);

	//---clear fw reset status---
	buf[0] = EVENT_MAP_RESET_COMPLETE & (0x7F);
	buf[1] = 0x00;
	CTP_SPI_WRITE(ts->client, buf, 2);

	//---enable fw crc---
	buf[0] = EVENT_MAP_HOST_CMD & (0x7F);
	buf[1] = 0xAE;	//enable fw crc command
	CTP_SPI_WRITE(ts->client, buf, 2);
}

/*******************************************************
Description:
	Novatek touchscreen set boot ready function.

return:
	N/A.
*******************************************************/
void nvt_boot_ready(void)
{
	//---write BOOT_RDY status cmds---
	nvt_write_addr(ts->mmap->BOOT_RDY_ADDR, 1);

	mdelay(5);

	if (!ts->hw_crc) {
		//---write BOOT_RDY status cmds---
		nvt_write_addr(ts->mmap->BOOT_RDY_ADDR, 0);

		//---write POR_CD cmds---
		nvt_write_addr(ts->mmap->POR_CD_ADDR, 0xA0);
	}
}

/*******************************************************
Description:
	Novatek touchscreen eng reset cmd
    function.

return:
	n.a.
*******************************************************/
void nvt_eng_reset(void)
{
	//---eng reset cmds to ENG_RST_ADDR---
	NVT_ERR("%s\n", __func__);
	nvt_write_addr(ENG_RST_ADDR, 0x5A);
	NVT_ERR("%s leave\n", __func__);
	mdelay(1);	//wait tMCU_Idle2TP_REX_Hi after TP_RST
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU
    function.

return:
	n.a.
*******************************************************/
void nvt_sw_reset(void)
{
	//---software reset cmds to SWRST_N8_ADDR---
	nvt_write_addr(SWRST_N8_ADDR, 0x55);

	msleep(10);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU then into idle mode
    function.

return:
	n.a.
*******************************************************/
void nvt_sw_reset_idle(void)
{
	//---MCU idle cmds to SWRST_N8_ADDR---
	nvt_write_addr(SWRST_N8_ADDR, 0xAA);

	msleep(15);
}

/*******************************************************
Description:
	Novatek touchscreen reset MCU (boot) function.

return:
	n.a.
*******************************************************/
void nvt_bootloader_reset(void)
{
	//---reset cmds to SWRST_N8_ADDR---
	nvt_write_addr(SWRST_N8_ADDR, 0x69);

	mdelay(5);	//wait tBRST2FR after Bootload RST

	if (SPI_RD_FAST_ADDR) {
		/* disable SPI_RD_FAST */
		nvt_write_addr(SPI_RD_FAST_ADDR, 0x00);
	}
}

/*******************************************************
Description:
	Novatek touchscreen clear FW status function.

return:
	Executive outcomes. 0---succeed. -1---fail.
*******************************************************/
int32_t nvt_clear_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 20;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		//---clear fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_SPI_WRITE(ts->client, buf, 2);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0xFF;
		CTP_SPI_READ(ts->client, buf, 2);

		if (buf[1] == 0x00)
			break;

		usleep_range(10000, 10000);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW status function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_status(void)
{
	uint8_t buf[8] = {0};
	int32_t i = 0;
	const int32_t retry = 50;

	for (i = 0; i < retry; i++) {
		//---set xdata index to EVENT BUF ADDR---
		nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE);

		//---read fw status---
		buf[0] = EVENT_MAP_HANDSHAKING_or_SUB_CMD_BYTE;
		buf[1] = 0x00;
		CTP_SPI_READ(ts->client, buf, 2);

		if ((buf[1] & 0xF0) == 0xA0)
			break;

		usleep_range(10000, 10000);
	}

	if (i >= retry) {
		NVT_ERR("failed, i=%d, buf[1]=0x%02X\n", i, buf[1]);
		return -1;
	} else {
		return 0;
	}
}

/*******************************************************
Description:
	Novatek touchscreen check FW reset state function.

return:
	Executive outcomes. 0---succeed. -1---failed.
*******************************************************/
int32_t nvt_check_fw_reset_state(RST_COMPLETE_STATE check_reset_state)
{
	uint8_t buf[8] = {0};
	int32_t ret = 0;
	int32_t retry = 0;
	int32_t retry_max = (check_reset_state == RESET_STATE_INIT) ? 10 : 50;

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_RESET_COMPLETE);

	while (1) {
		//---read reset state---
		buf[0] = EVENT_MAP_RESET_COMPLETE;
		buf[1] = 0x00;
		CTP_SPI_READ(ts->client, buf, 6);

		if ((buf[1] >= check_reset_state) && (buf[1] <= RESET_STATE_MAX)) {
			ret = 0;
			break;
		}

		retry++;
		if (unlikely(retry > retry_max)) {
			NVT_ERR("error, retry=%d, buf[1]=0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
				retry, buf[1], buf[2], buf[3], buf[4], buf[5]);
			ret = -1;
			break;
		}

		usleep_range(10000, 10000);
	}

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen get novatek project id information
	function.

return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
int32_t nvt_read_pid(void)
{
	uint8_t buf[3] = {0};
	int32_t ret = 0;

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_PROJECTID);

	//---read project id---
	buf[0] = EVENT_MAP_PROJECTID;
	buf[1] = 0x00;
	buf[2] = 0x00;
	CTP_SPI_READ(ts->client, buf, 3);

	ts->nvt_pid = (buf[2] << 8) + buf[1];

	//---set xdata index to EVENT BUF ADDR---
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR);

	NVT_LOG("PID=%04X\n", ts->nvt_pid);

	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen get firmware related information
	function.

return:
	Executive outcomes. 0---success. -1---fail.
*******************************************************/
int32_t nvt_get_fw_info(void)
{
	uint8_t buf[64] = {0};
	uint32_t retry_count = 0;
	int32_t ret = 0;

info_retry:
	/*---set xdata index to EVENT BUF ADDR---*/
	nvt_set_page(ts->mmap->EVENT_BUF_ADDR | EVENT_MAP_FWINFO);

	/*---read fw info---*/
	buf[0] = EVENT_MAP_FWINFO;
	CTP_SPI_READ(ts->client, buf, 17);
	ts->x_num = buf[3];
	ts->y_num = buf[4];
	ts->abs_x_max = (uint16_t)((buf[5] << 8) | buf[6]);
	ts->abs_y_max = (uint16_t)((buf[7] << 8) | buf[8]);

	//---clear x_num, y_num if fw info is broken---
	if ((buf[1] + buf[2]) != 0xFF) {
		NVT_ERR("FW info is broken! fw_ver=0x%02X, ~fw_ver=0x%02X\n", buf[1], buf[2]);
		ts->x_num = 18;
		ts->y_num = 32;
		ts->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
		ts->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;

		if (retry_count < 3) {
			retry_count++;
			NVT_ERR("retry_count=%d\n", retry_count);
			goto info_retry;
		} else {
			NVT_ERR("Set default fw_ver=%d, x_num=%d, y_num=%d, abs_x_max=%d, abs_y_max=%d\n",
					ts->fw_ver, ts->x_num, ts->y_num,
					ts->abs_x_max, ts->abs_y_max);
			ret = -1;
		}
	} else {
		ret = 0;
	}

	NVT_LOG("FW type is 0x%02X, fw_ver=%d\n", buf[14], ts->fw_ver);

	//---Get Novatek PID---
	nvt_read_pid();
	return ret;
}

#if WAKEUP_GESTURE
#define GESTURE_WORD_C          12
#define GESTURE_WORD_W          13
#define GESTURE_WORD_V          14
#define GESTURE_DOUBLE_CLICK    15
#define GESTURE_WORD_Z          16
#define GESTURE_WORD_M          17
#define GESTURE_WORD_O          18
#define GESTURE_WORD_e          19
#define GESTURE_WORD_S          20
#define GESTURE_SLIDE_UP        21
#define GESTURE_SLIDE_DOWN      22
#define GESTURE_SLIDE_LEFT      23
#define GESTURE_SLIDE_RIGHT     24
/* customized gesture id */
#define DATA_PROTOCOL           30

/* function page definition */
#define FUNCPAGE_GESTURE         1

/*******************************************************
Description:
	Novatek touchscreen wake up gesture key report function.

return:
	n.a.
*******************************************************/
void nvt_ts_wakeup_gesture_report(uint8_t gesture_id, uint8_t *data)
{
	uint32_t keycode = 0;
	uint8_t func_type = data[2];
	uint8_t func_id = data[3];

	/* support fw specifal data protocol */
	if ((gesture_id == DATA_PROTOCOL) && (func_type == FUNCPAGE_GESTURE)) {
		gesture_id = func_id;
	} else if (gesture_id > DATA_PROTOCOL) {
		NVT_ERR("gesture_id %d is invalid, func_type=%d, func_id=%d\n", gesture_id, func_type, func_id);
		return;
	}

	NVT_LOG("gesture_id = %d\n", gesture_id);

	switch (gesture_id) {
		case GESTURE_WORD_C:
			NVT_LOG("Gesture : Word-C.\n");
			keycode = gesture_key_array[0];
			break;
		case GESTURE_WORD_W:
			NVT_LOG("Gesture : Word-W.\n");
			keycode = gesture_key_array[1];
			break;
		case GESTURE_WORD_V:
			NVT_LOG("Gesture : Word-V.\n");
			keycode = gesture_key_array[2];
			break;
		case GESTURE_DOUBLE_CLICK:
			NVT_LOG("Gesture : Double Click.\n");
			keycode = gesture_key_array[3];
			break;
		case GESTURE_WORD_Z:
			NVT_LOG("Gesture : Word-Z.\n");
			keycode = gesture_key_array[4];
			break;
		case GESTURE_WORD_M:
			NVT_LOG("Gesture : Word-M.\n");
			keycode = gesture_key_array[5];
			break;
		case GESTURE_WORD_O:
			NVT_LOG("Gesture : Word-O.\n");
			keycode = gesture_key_array[6];
			break;
		case GESTURE_WORD_e:
			NVT_LOG("Gesture : Word-e.\n");
			keycode = gesture_key_array[7];
			break;
		case GESTURE_WORD_S:
			NVT_LOG("Gesture : Word-S.\n");
			keycode = gesture_key_array[8];
			break;
		case GESTURE_SLIDE_UP:
			NVT_LOG("Gesture : Slide UP.\n");
			keycode = gesture_key_array[9];
			break;
		case GESTURE_SLIDE_DOWN:
			NVT_LOG("Gesture : Slide DOWN.\n");
			keycode = gesture_key_array[10];
			break;
		case GESTURE_SLIDE_LEFT:
			NVT_LOG("Gesture : Slide LEFT.\n");
			keycode = gesture_key_array[11];
			break;
		case GESTURE_SLIDE_RIGHT:
			NVT_LOG("Gesture : Slide RIGHT.\n");
			keycode = gesture_key_array[12];
			break;
		default:
			break;
	}

	if (keycode > 0) {
		input_report_key(ts->input_dev, keycode, 1);
		input_sync(ts->input_dev);
		input_report_key(ts->input_dev, keycode, 0);
		input_sync(ts->input_dev);
	}
}
#endif

/*******************************************************
Description:
	Novatek touchscreen parse device tree function.

return:
	n.a.
*******************************************************/
#ifdef CONFIG_OF
static int32_t nvt_parse_dt(struct device *dev)
{
	struct nvt_config_info *config_info;
	struct device_node *temp, *np = dev->of_node;
	int32_t ret = 0;
	uint32_t temp_val;

#if NVT_TOUCH_SUPPORT_HW_RST
	ts->reset_gpio = of_get_named_gpio_flags(np, "novatek,reset-gpio", 0, &ts->reset_flags);
	NVT_LOG("novatek,reset-gpio=%d\n", ts->reset_gpio);
#endif
	ts->irq_gpio = of_get_named_gpio_flags(np, "novatek,irq-gpio", 0, &ts->irq_flags);
	NVT_LOG("novatek,irq-gpio=%d\n", ts->irq_gpio);

	ret = of_property_read_u32(np, "novatek,swrst-n8-addr", &SWRST_N8_ADDR);
	if (ret) {
		NVT_ERR("error reading novatek,swrst-n8-addr. ret=%d\n", ret);
		return ret;
	} else {
		NVT_LOG("SWRST_N8_ADDR=0x%06X\n", SWRST_N8_ADDR);
	}

	ret = of_property_read_u32(np, "novatek,spi-rd-fast-addr", &SPI_RD_FAST_ADDR);
	if (ret) {
		NVT_LOG("not support novatek,spi-rd-fast-addr\n");
		SPI_RD_FAST_ADDR = 0;
		ret = 0;
	} else {
		NVT_LOG("SPI_RD_FAST_ADDR=0x%06X\n", SPI_RD_FAST_ADDR);
	}

	ret = of_property_read_u32(np, "novatek,config-array-size", &ts->config_array_size);
	if (ret) {
		NVT_LOG("Unable to get array size\n");
		return ret;
	} else {
		NVT_LOG("config-array-size: %u\n", ts->config_array_size);
	}

	ret = of_property_read_u32(np, "spi-max-frequency", &ts->spi_max_freq);
	if (ret) {
		NVT_LOG("Unable to get spi freq\n");
		return ret;
	} else {
		NVT_LOG("spi-max-frequency: %u\n", ts->spi_max_freq);
	}

	ts->config_array = devm_kzalloc(dev, ts->config_array_size * sizeof(struct nvt_config_info), GFP_KERNEL);
	if (!ts->config_array) {
		NVT_LOG("Unable to allocate memory\n");
		return -ENOMEM;
	}


	config_info = ts->config_array;
	for_each_child_of_node(np, temp) {
		if (config_info - ts->config_array >= ts->config_array_size) {
			NVT_ERR("parse %ld config down\n", config_info - ts->config_array);
			break;
		}

		ret = of_property_read_u32(temp, "novatek,tp-vendor", &temp_val);
		if (ret) {
			NVT_LOG("Unable to read tp vendor\n");
		} else {
			config_info->tp_vendor = (u8) temp_val;
			NVT_LOG("tp vendor: %u", config_info->tp_vendor);
		}

		ret = of_property_read_u32(temp, "novatek,display-maker", &temp_val);
		if (ret) {
			NVT_LOG("Unable to read tp hw version\n");
		} else {
			config_info->display_maker = (u8) temp_val;
			NVT_LOG("tp hw version: %u", config_info->display_maker);
		}

		ret = of_property_read_u32(temp, "novatek,panel-cg", &temp_val);
		if (ret) {
			NVT_LOG("Unable to read panel cg name\n");
		} else {
			config_info->panel_cg = (u8) temp_val;
			NVT_LOG("cg_name: %s", config_info->panel_cg);
		}

		ret = of_property_read_string(temp, "novatek,fw-name",
						 &config_info->nvt_fw_name);
		if (ret && (ret != -EINVAL)) {
			NVT_LOG("Unable to read fw name\n");
		} else {
			NVT_LOG("fw_name: %s", config_info->nvt_fw_name);
		}

		ret = of_property_read_string(temp, "novatek,mp-name",
						 &config_info->nvt_mp_name);
		if (ret && (ret != -EINVAL)) {
			NVT_LOG("Unable to read mp name\n");
		} else {
			NVT_LOG("mp_name: %s", config_info->nvt_mp_name);
		}

		config_info++;
	}
	return ret;
}
#else
static int32_t nvt_parse_dt(struct device *dev)
{
#if NVT_TOUCH_SUPPORT_HW_RST
	ts->reset_gpio = NVTTOUCH_RST_PIN;
#endif
	ts->irq_gpio = NVTTOUCH_INT_PIN;
	return 0;
}
#endif

static int nvt_get_panel_type(struct nvt_ts_data *ts_data)
{
	int i;
	u8 *lockdown = ts_data->lockdown_info;
	struct nvt_config_info *panel_list = ts->config_array;

	for (i = 0; i < ts->config_array_size; i++) {
		if (lockdown[7] == panel_list[i].panel_cg) {
			NVT_LOG("matched panel type, fw is [%s], mp is [%s]",
				panel_list[i].nvt_fw_name, panel_list[i].nvt_mp_name);
			break;
		}
	}

	ts->panel_index = i;

	if (i >= ts->config_array_size) {
		NVT_ERR("mismatched panel type, using default fw");
		ts->panel_index = -EINVAL;
	}

	return ts->panel_index;
}

bool is_lockdown_empty(u8 *lockdown)
{
	bool ret = true;
	int i;

	for (i = 0; i < NVT_LOCKDOWN_SIZE; i++) {
		if (lockdown[i] != 0) {
			ret = false;
			break;
		}
	}

	return ret;
}

void nvt_match_fw(void)
{
	NVT_LOG("start match fw name");
	if (is_lockdown_empty(ts->lockdown_info))
		flush_delayed_work(&ts->nvt_lockdown_work);
	if (nvt_get_panel_type(ts) < 0) {
		ts->fw_name = DEFAULT_BOOT_UPDATE_FIRMWARE_NAME;
		ts->mp_name = DEFAULT_MP_UPDATE_FIRMWARE_NAME;
	} else {
		ts->fw_name = ts->config_array[ts->panel_index].nvt_fw_name;
		ts->mp_name = ts->config_array[ts->panel_index].nvt_mp_name;
	}
}

/*******************************************************
Description:
	Novatek touchscreen config and request gpio

return:
	Executive outcomes. 0---succeed. not 0---failed.
*******************************************************/
static int nvt_gpio_config(struct nvt_ts_data *ts)
{
	int32_t ret = 0;

#if NVT_TOUCH_SUPPORT_HW_RST
	/* request RST-pin (Output/High) */
	if (gpio_is_valid(ts->reset_gpio)) {
		ret = gpio_request_one(ts->reset_gpio, GPIOF_OUT_INIT_LOW, "NVT-tp-rst");
		if (ret) {
			NVT_ERR("Failed to request NVT-tp-rst GPIO\n");
			goto err_request_reset_gpio;
		}
	}
#endif

	/* request INT-pin (Input) */
	if (gpio_is_valid(ts->irq_gpio)) {
		ret = gpio_request_one(ts->irq_gpio, GPIOF_IN, "NVT-int");
		if (ret) {
			NVT_ERR("Failed to request NVT-int GPIO\n");
			goto err_request_irq_gpio;
		}
	}
	NVT_ERR("%s Exit\n", __func__);
	return ret;

err_request_irq_gpio:
#if NVT_TOUCH_SUPPORT_HW_RST
	gpio_free(ts->reset_gpio);
err_request_reset_gpio:
#endif
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen deconfig gpio

return:
	n.a.
*******************************************************/
static void nvt_gpio_deconfig(struct nvt_ts_data *ts)
{
	if (gpio_is_valid(ts->irq_gpio))
		gpio_free(ts->irq_gpio);
#if NVT_TOUCH_SUPPORT_HW_RST
	if (gpio_is_valid(ts->reset_gpio))
		gpio_free(ts->reset_gpio);
#endif
}

#define POINT_DATA_LEN 65
/*******************************************************
Description:
	Novatek touchscreen work function.

return:
	n.a.
*******************************************************/
static irqreturn_t nvt_ts_work_func(int irq, void *data)
{
	int32_t ret;
	uint8_t point_data[POINT_DATA_LEN + 1 + DUMMY_BYTES] = { 0, };
	uint32_t position;
	uint32_t input_x;
	uint32_t input_y;
	uint8_t input_id;
	uint8_t press_id[TOUCH_MAX_FINGER_NUM] = {0};
	int32_t i;
	int32_t finger_cnt;

 pm_qos_update_request(&ts->pm_qos_req, 100);

#if WAKEUP_GESTURE
	if (likely(bTouchIsAwake == 0)) {
		pm_wakeup_event(&ts->input_dev->dev, 5000);
	}
#endif

	mutex_lock(&ts->lock);

	if (unlikely(ts->dev_pm_suspend)) {
		ret = wait_for_completion_timeout(&ts->dev_pm_suspend_completion, msecs_to_jiffies(500));
		if (!ret) {
			NVT_ERR("system(spi) can't finished resuming procedure, skip it\n");
			goto out;
		}
	}

	ret = CTP_SPI_READ(ts->client, point_data, POINT_DATA_LEN + 1);
	if (unlikely(ret < 0)) {
		NVT_ERR("CTP_SPI_READ failed.(%d)\n", ret);
		goto out;
	}

#if WAKEUP_GESTURE
	if (likely(bTouchIsAwake == 0)) {
		input_id = (uint8_t)(point_data[1] >> 3);
		nvt_ts_wakeup_gesture_report(input_id, point_data);
		nvt_irq_enable(true);
		goto out;
	}
#endif

	for (i = 0; i < ts->max_touch_num; i++) {
		position = 1 + 6 * i;
		input_id = (uint8_t)(point_data[position] >> 3);
		if ((input_id == 0) || (input_id > ts->max_touch_num))
			continue;

		if (likely(((point_data[position] & 0x07) == 0x01) || ((point_data[position] & 0x07) == 0x02))) {	//finger down (enter & moving)

			input_x = (uint32_t) (point_data[position + 1] << 4) + (uint32_t) (point_data[position + 3] >> 4);
			input_y = (uint32_t) (point_data[position + 2] << 4) + (uint32_t) (point_data[position + 3] & 0x0F);

			press_id[input_id - 1] = 1;
			input_mt_slot(ts->input_dev, input_id - 1);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
			input_report_key(ts->input_dev, BTN_TOUCH, 1);
			input_report_key(ts->input_dev, BTN_TOOL_FINGER, 1);

			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
			input_report_abs(ts->input_dev, ABS_MT_PRESSURE, TOUCH_FORCE_NUM);

			set_bit(input_id - 1, ts->slot_map);
			finger_cnt++;
		}
	}

	for (i = 0; i < ts->max_touch_num; i++) {
		if (likely(press_id[i] != 1)) {
			input_mt_slot(ts->input_dev, i);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
			if (finger_cnt == 0 && test_bit(i, ts->slot_map)) {
				input_report_key(ts->input_dev, BTN_TOUCH, 0);
				input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
			}
			clear_bit(i, ts->slot_map);
		}
	}

out:

	pm_qos_update_request(&ts->pm_qos_req, PM_QOS_DEFAULT_VALUE);

	input_sync(ts->input_dev);

	mutex_unlock(&ts->lock);

	return IRQ_HANDLED;
}


/*******************************************************
Description:
	Novatek touchscreen check chip version trim function.

return:
	Executive outcomes. 0---NVT IC. -1---not NVT IC.
*******************************************************/
static int8_t nvt_ts_check_chip_ver_trim(void)
{
	uint8_t buf[8] = {0};
	int32_t retry = 0;
	int32_t list = 0;
	int32_t i = 0;
	int32_t found_nvt_chip = 0;
	int32_t ret = -1;

	//---Check for 5 times---
	for (retry = 5; retry > 0; retry--) {

		nvt_bootloader_reset();

		//---set xdata index to 0x1F600---
		nvt_set_page(0x1F600);

		buf[0] = 0x4E;
		buf[1] = 0x00;
		buf[2] = 0x00;
		buf[3] = 0x00;
		buf[4] = 0x00;
		buf[5] = 0x00;
		buf[6] = 0x00;
		CTP_SPI_READ(ts->client, buf, 7);
		NVT_LOG("buf[1]=0x%02X, buf[2]=0x%02X, buf[3]=0x%02X, buf[4]=0x%02X, buf[5]=0x%02X, buf[6]=0x%02X\n",
			buf[1], buf[2], buf[3], buf[4], buf[5], buf[6]);

		// compare read chip id on supported list
		for (list = 0; list < (sizeof(trim_id_table) / sizeof(struct nvt_ts_trim_id_table)); list++) {
			found_nvt_chip = 0;

			// compare each byte
			for (i = 0; i < NVT_ID_BYTE_MAX; i++) {
				if (trim_id_table[list].mask[i]) {
					if (buf[i + 1] != trim_id_table[list].id[i])
						break;
				}
			}

			if (i == NVT_ID_BYTE_MAX) {
				found_nvt_chip = 1;
			}

			if (found_nvt_chip) {
				NVT_LOG("This is NVT touch IC\n");
				ts->mmap = trim_id_table[list].mmap;
				ts->carrier_system = trim_id_table[list].hwinfo->carrier_system;
				ts->hw_crc = trim_id_table[list].hwinfo->hw_crc;
				ret = 0;
				goto out;
			} else {
				ts->mmap = NULL;
				ret = -1;
			}
		}

		msleep(10);
	}

out:
	return ret;
}

static int nvt_pinctrl_init(struct nvt_ts_data *nvt_data)
{
	int retval = 0;
	/* Get pinctrl if target uses pinctrl */
	nvt_data->ts_pinctrl = devm_pinctrl_get(&nvt_data->pdev->dev);
	NVT_LOG("%s Enter\n", __func__);
	if (IS_ERR_OR_NULL(nvt_data->ts_pinctrl)) {
		retval = PTR_ERR(nvt_data->ts_pinctrl);
		NVT_ERR("Target does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}

	nvt_data->pinctrl_state_active
		= pinctrl_lookup_state(nvt_data->ts_pinctrl, PINCTRL_STATE_ACTIVE);

	if (IS_ERR_OR_NULL(nvt_data->pinctrl_state_active)) {
		retval = PTR_ERR(nvt_data->pinctrl_state_active);
		NVT_ERR("Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_ACTIVE, retval);
		goto err_pinctrl_lookup;
	}

	nvt_data->pinctrl_state_suspend
		= pinctrl_lookup_state(nvt_data->ts_pinctrl, PINCTRL_STATE_SUSPEND);

	if (IS_ERR_OR_NULL(nvt_data->pinctrl_state_suspend)) {
		retval = PTR_ERR(nvt_data->pinctrl_state_suspend);
		NVT_ERR("Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_SUSPEND, retval);
		goto err_pinctrl_lookup;
	}

	return 0;
err_pinctrl_lookup:
	devm_pinctrl_put(nvt_data->ts_pinctrl);
err_pinctrl_get:
	nvt_data->ts_pinctrl = NULL;
	return retval;
}


static int nvt_input_event(struct input_dev *dev, unsigned int type, unsigned int code, int value)
{
	struct nvt_ts_data *ts_data = input_get_drvdata(dev);
	struct nvt_mode_switch sw;

	if (!ts_data) {
		NVT_ERR("ts_data is NULL");
		return -EINVAL;
	}

	if (type == EV_SYN && code == SYN_CONFIG) {
		if (value >= INPUT_EVENT_START && value <= INPUT_EVENT_END) {
		sw.ts_data = ts_data;
		sw.mode = (unsigned char)value;
		if (value >= INPUT_EVENT_WAKEUP_MODE_OFF && value <= INPUT_EVENT_WAKEUP_MODE_ON) {
		if (ts_data->ic_state <= NVT_IC_SUSPEND_OUT && ts_data->ic_state != NVT_IC_INIT) {
			ts_data->gesture_command_delayed = value;
			NVT_LOG("Panel off, don't set dbclick gesture flag util panel on");
			ts_data->db_wakeup = 0;
		} else {
			ts_data->db_wakeup = value - INPUT_EVENT_WAKEUP_MODE_OFF;
			NVT_LOG("%s double click wakeup", ts_data->db_wakeup ? "ENABLE" : "DISABLE");
		}
	}

	/*enable dbclick in resume produce if at suspend state*/
	if (ts_data->ic_state >= NVT_IC_RESUME_IN) {
		tp_enable_doubleclick(!!ts->db_wakeup);
	}
		} else {
			NVT_ERR("Invalid event value");
			return -EINVAL;
		}
	}

	return 0;
}

extern int dsi_panel_get_lockdowninfo_for_tp(unsigned char *plockdowninfo);

static void nvt_resume_work(struct work_struct *work)
{
	struct nvt_ts_data *ts_core = container_of(work, struct nvt_ts_data, resume_work);

	nvt_ts_resume(&ts_core->client->dev);
}
static void get_lockdown_info(struct work_struct *work)
{
	int ret = 0;

	NVT_LOG("lkdown_readed = %d", ts->lkdown_readed);
	if (!ts->lkdown_readed) {
		ret = dsi_panel_get_lockdowninfo_for_tp(ts->lockdown_info);
		if (ret < 0) {
			NVT_ERR("can't get lockdown info");
		} else {
			NVT_LOG("Lockdown:0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x\n",
			ts->lockdown_info[0], ts->lockdown_info[1], ts->lockdown_info[2], ts->lockdown_info[3],
			ts->lockdown_info[4], ts->lockdown_info[5], ts->lockdown_info[6], ts->lockdown_info[7]);
		}
		ts->lkdown_readed = true;
		NVT_LOG("READ LOCKDOWN!!!");
	} else {
		NVT_LOG("use lockdown info that readed before");
		NVT_LOG("Lockdown:0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x\n",
			ts->lockdown_info[0], ts->lockdown_info[1], ts->lockdown_info[2], ts->lockdown_info[3],
			ts->lockdown_info[4], ts->lockdown_info[5], ts->lockdown_info[6], ts->lockdown_info[7]);
	}
}

/*******************************************************
Description:
	Novatek touchscreen driver probe function.

return:
	Executive outcomes. 0---succeed. negative---failed
*******************************************************/
static int32_t nvt_ts_probe(struct platform_device *pdev)
{
	struct spi_device *ts_xsfer;
	int32_t ret = 0;
#if WAKEUP_GESTURE
	int32_t retry = 0;
#endif
	struct attribute_group *attrs_p = NULL;

	NVT_LOG("start\n");

	ts = kzalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		NVT_ERR("failed to allocated memory for nvt ts data\n");
		return -ENOMEM;
	}
	ts->xbuf = (uint8_t *)kzalloc((NVT_TRANSFER_LEN+1), GFP_KERNEL);
	if (ts->xbuf == NULL) {
		NVT_ERR("kzalloc for xbuf failed!\n");
		if (ts) {
			kfree(ts);
			ts = NULL;
		}
		return -ENOMEM;
	}

	ts->pdev = pdev;
	for (retry = 1; retry <= 3; ++retry) {
		ret = tmp_hold_ts_xsfer(&ts_xsfer);
		if (ret < 0) {
			if (ret == -EBUSY) {
				NVT_LOG("tmp hold ts_xsfer failed, retry:%d\n", retry);
				mdelay(100);
				continue;
			} else if (ret == -EPERM) {
				NVT_ERR("ts_xsfer has been used, exit nvt probe\n");
				goto err_get_spi;
			} else if (ret == -EINVAL) {
				NVT_ERR("ts_xsfer not exist, exit nvt probe\n");
				goto err_get_spi;
			}
		} else {
			break;
		}
	}
	if (ret == -EBUSY) {
		NVT_ERR("ts_xsfer always busy, exit nvt probe\n");
		goto err_get_spi;
	}

	//---parse dts---
	ret = nvt_parse_dt(&pdev->dev);
	if (ret) {
		NVT_ERR("parse dt error\n");
		goto err_spi_setup;
	}


	ts->client = ts_xsfer;
	spi_set_drvdata(ts->client, ts);

	//---prepare for spi parameter---
	if (ts->client->master->flags & SPI_MASTER_HALF_DUPLEX) {
		NVT_ERR("Full duplex not supported by master\n");
		ret = -EIO;
		goto err_ckeck_full_duplex;
	}
	ts->client->bits_per_word = 8;
	ts->client->mode = SPI_MODE_0;
	ts->client->max_speed_hz = ts->spi_max_freq;

	ret = spi_setup(ts->client);
	if (ret < 0) {
		NVT_ERR("Failed to perform SPI setup\n");
		goto err_spi_setup;
	}

	NVT_LOG("mode=%d, max_speed_hz=%d\n", ts->client->mode, ts->client->max_speed_hz);

	ret = nvt_pinctrl_init(ts);
	if (!ret && ts->ts_pinctrl) {
		ret = pinctrl_select_state(ts->ts_pinctrl, ts->pinctrl_state_active);

		if (ret < 0) {
			NVT_ERR("Failed to select %s pinstate %d\n",
				PINCTRL_STATE_ACTIVE, ret);
		}
	} else {
		NVT_ERR("Failed to init pinctrl\n");
	}

	NVT_LOG("Request GPIO\n");
	//---request and config GPIOs---
	ret = nvt_gpio_config(ts);
	if (ret) {
		NVT_ERR("gpio config error!\n");
		goto err_gpio_config_failed;
	}

	mutex_init(&ts->lock);
	mutex_init(&ts->xbuf_lock);

	//---eng reset before TP_RESX high

	nvt_eng_reset();
#if NVT_TOUCH_SUPPORT_HW_RST
	gpio_set_value(ts->reset_gpio, 1);
#endif
	NVT_ERR("gpio set complete\n");
	// need 10ms delay after POR(power on reset)
	msleep(10);

	//---check chip version trim---
	NVT_LOG("start check chip\n");
	ret = nvt_ts_check_chip_ver_trim();
	if (ret) {
		NVT_ERR("chip is not identified\n");
		ret = -EINVAL;
		goto err_chipvertrim_failed;
	}
	NVT_LOG("finish check chip\n");

	get_ts_xsfer(NVT_SPI_NAME);
	tmp_drop_ts_xsfer();
	ts->abs_x_max = TOUCH_DEFAULT_MAX_WIDTH;
	ts->abs_y_max = TOUCH_DEFAULT_MAX_HEIGHT;
	//---allocate input device---
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		NVT_ERR("allocate input device failed\n");
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}

	ts->max_touch_num = TOUCH_MAX_FINGER_NUM;

	ts->int_trigger_type = INT_TRIGGER_TYPE;


	//---set input device info.---
	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH | BTN_TOOL_FINGER);
	ts->input_dev->propbit[0] = BIT(INPUT_PROP_DIRECT);

	input_mt_init_slots(ts->input_dev, ts->max_touch_num, 0);

#if TOUCH_MAX_FINGER_NUM > 1
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);    //area = 255

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max - 1, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max - 1, 0, 0);
#endif //TOUCH_MAX_FINGER_NUM > 1

#if WAKEUP_GESTURE
	for (retry = 0; retry < (ARRAY_SIZE(gesture_key_array)); retry++) {
		input_set_capability(ts->input_dev, EV_KEY, gesture_key_array[retry]);
	}
#endif

	sprintf(ts->phys, "input/ts");
	ts->input_dev->name = NVT_TS_NAME;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_SPI;
	ts->input_dev->event = nvt_input_event;

	input_set_drvdata(ts->input_dev, ts);
	//---register input device---
	ret = input_register_device(ts->input_dev);
	if (ret) {
		NVT_ERR("register input device (%s) failed. ret=%d\n", ts->input_dev->name, ret);
		goto err_input_register_device_failed;
	}

	//---set int-pin & request irq---
	ts->client->irq = gpio_to_irq(ts->irq_gpio);
	if (ts->client->irq) {
		NVT_LOG("int_trigger_type=%d\n", ts->int_trigger_type);
		ts->irq_enabled = true;
		ret = request_threaded_irq(ts->client->irq, NULL, nvt_ts_work_func,
				ts->int_trigger_type | IRQF_ONESHOT, NVT_SPI_NAME, ts);
		if (ret != 0) {
			NVT_ERR("request irq failed. ret=%d\n", ret);
			goto err_int_request_failed;
		} else {
			nvt_irq_enable(false);
			NVT_LOG("request irq %d succeed\n", ts->client->irq);
		}

		ts->pm_qos_req.type = PM_QOS_REQ_AFFINE_IRQ;
		ts->pm_qos_req.irq = ts->client->irq;
		pm_qos_add_request(&ts->pm_qos_req, PM_QOS_CPU_DMA_LATENCY,
				PM_QOS_DEFAULT_VALUE);

	}

	nvt_lockdown_wq = alloc_workqueue("nvt_lockdown_wq", WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!nvt_lockdown_wq) {
		NVT_ERR("nvt_fwu_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_lockdown_wq_failed;
	}
	INIT_DELAYED_WORK(&ts->nvt_lockdown_work, get_lockdown_info);
	// please make sure boot update start after display reset(RESX) sequence
	queue_delayed_work(nvt_lockdown_wq, &ts->nvt_lockdown_work, msecs_to_jiffies(5000));

#if WAKEUP_GESTURE
	device_init_wakeup(&ts->input_dev->dev, 1);
#endif
	ts->ic_state = NVT_IC_INIT;
	ts->dev_pm_suspend = false;
	ts->gesture_command_delayed = -1;
	init_completion(&ts->dev_pm_suspend_completion);

#if BOOT_UPDATE_FIRMWARE
	nvt_fwu_wq = alloc_workqueue("nvt_fwu_wq", WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!nvt_fwu_wq) {
		NVT_ERR("nvt_fwu_wq create workqueue failed\n");
		ret = -ENOMEM;
		goto err_create_nvt_fwu_wq_failed;
	}
	INIT_DELAYED_WORK(&ts->nvt_fwu_work, Boot_Update_Firmware);
	// please make sure boot update start after display reset(RESX) sequence
	queue_delayed_work(nvt_fwu_wq, &ts->nvt_fwu_work, msecs_to_jiffies(14000));
#endif

	attrs_p = (struct attribute_group *)devm_kzalloc(&pdev->dev, sizeof(*attrs_p), GFP_KERNEL);
	if (!attrs_p) {
		NVT_ERR("no mem to alloc");
	}
	ts->attrs = attrs_p;
	attrs_p->name = "panel_info";
	ret = sysfs_create_group(&pdev->dev.kobj, ts->attrs);

	ts->event_wq = alloc_workqueue("nvt-event-queue",
		WQ_UNBOUND | WQ_HIGHPRI | WQ_CPU_INTENSIVE, 1);
	if (!ts->event_wq) {
		NVT_ERR("Can not create work thread for suspend/resume!!");
		ret = -ENOMEM;
		goto err_alloc_work_thread_failed;
	}
	INIT_WORK(&ts->resume_work, nvt_resume_work);
	/*INIT_WORK(&ts->suspend_work, nvt_suspend_work);*/

	ts->drm_notif.notifier_call = nvt_drm_notifier_callback;
	ret = msm_drm_register_client(&ts->drm_notif);
	if (ret) {
		NVT_ERR("register drm_notifier failed. ret=%d\n", ret);
		goto err_register_drm_notif_failed;
	}

	bTouchIsAwake = 1;
	NVT_LOG("end\n");

	nvt_irq_enable(true);

	return 0;

	if (msm_drm_unregister_client(&ts->drm_notif))
		NVT_ERR("Error occurred while unregistering drm_notifier.\n");
err_register_drm_notif_failed:
	destroy_workqueue(ts->event_wq);
err_alloc_work_thread_failed:
#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq) {
		cancel_delayed_work_sync(&ts->nvt_fwu_work);
		destroy_workqueue(nvt_fwu_wq);
		nvt_fwu_wq = NULL;
	}
err_create_nvt_fwu_wq_failed:
	if (nvt_lockdown_wq) {
		cancel_delayed_work_sync(&ts->nvt_lockdown_work);
		destroy_workqueue(nvt_lockdown_wq);
		nvt_lockdown_wq = NULL;
	}
#endif
err_create_nvt_lockdown_wq_failed:
#if WAKEUP_GESTURE
	device_init_wakeup(&ts->input_dev->dev, 0);
#endif
	free_irq(ts->client->irq, ts);
err_int_request_failed:
	input_unregister_device(ts->input_dev);
	ts->input_dev = NULL;
err_input_register_device_failed:
	if (ts->input_dev) {
		input_free_device(ts->input_dev);
		ts->input_dev = NULL;
	}
err_input_dev_alloc_failed:
err_chipvertrim_failed:
	mutex_destroy(&ts->xbuf_lock);
	mutex_destroy(&ts->lock);
	nvt_gpio_deconfig(ts);
err_gpio_config_failed:
err_spi_setup:
err_ckeck_full_duplex:
	spi_set_drvdata(ts->client, NULL);
err_get_spi:
	tmp_drop_ts_xsfer();
	if (ts->xbuf) {
		kfree(ts->xbuf);
		ts->xbuf = NULL;
	}
	if (ts) {
		kfree(ts);
		ts = NULL;
	}
	return ret;
}

/*******************************************************
Description:
	Novatek touchscreen driver release function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_remove(struct platform_device *pdev)
{
	NVT_LOG("Removing driver...\n");

	if (msm_drm_unregister_client(&ts->drm_notif))
		NVT_ERR("Error occurred while unregistering drm_notifier.\n");

	pm_qos_remove_request(&ts->pm_qos_req);

#if BOOT_UPDATE_FIRMWARE
	if (nvt_fwu_wq) {
		cancel_delayed_work_sync(&ts->nvt_fwu_work);
		destroy_workqueue(nvt_fwu_wq);
		nvt_fwu_wq = NULL;
	}
#endif

#if WAKEUP_GESTURE
	device_init_wakeup(&ts->input_dev->dev, 0);
#endif

	nvt_irq_enable(false);
	free_irq(ts->client->irq, ts);

	mutex_destroy(&ts->xbuf_lock);
	mutex_destroy(&ts->lock);

	nvt_gpio_deconfig(ts);

	if (ts->input_dev) {
		input_unregister_device(ts->input_dev);
		ts->input_dev = NULL;
	}

	spi_set_drvdata(ts->client, NULL);

	put_ts_xsfer(NVT_SPI_NAME);
	if (ts) {
		kfree(ts);
		ts = NULL;
	}

	return 0;
}

static void nvt_ts_shutdown(struct platform_device *pdev)
{
	NVT_LOG("Shutdown driver...\n");

	nvt_irq_enable(false);

	if (msm_drm_unregister_client(&ts->drm_notif))
		NVT_ERR("Error occurred while unregistering drm_notifier.\n");

#if WAKEUP_GESTURE
	device_init_wakeup(&ts->input_dev->dev, 0);
#endif
}

/*******************************************************
Description:
	Novatek touchscreen driver suspend function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_suspend(struct device *dev)
{
	uint8_t buf[4] = {0};
	uint32_t i = 0;
	int ret = 0;

	if (!bTouchIsAwake) {
		NVT_LOG("Touch is already suspend\n");
		return 0;
	}
	pm_stay_awake(dev);
	ts->ic_state = NVT_IC_SUSPEND_IN;

	NVT_LOG("start\n");

	if (!ts->db_wakeup)
		nvt_irq_enable(false);	/*must before hold lock*/

	mutex_lock(&ts->lock);
	bTouchIsAwake = 0;

	mdelay(10);
	if (ts->db_wakeup) {
		//---write command to enter "wakeup gesture mode"---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x13;
		CTP_SPI_WRITE(ts->client, buf, 2);

		enable_irq_wake(ts->client->irq);

		NVT_LOG("Enabled touch wakeup gesture\n");

	} else {
		//---write command to enter "deep sleep mode"---
		buf[0] = EVENT_MAP_HOST_CMD;
		buf[1] = 0x11;
		CTP_SPI_WRITE(ts->client, buf, 2);
		if (ts->ts_pinctrl) {
			ret = pinctrl_select_state(ts->ts_pinctrl, ts->pinctrl_state_suspend);

			if (ret < 0) {
				NVT_ERR("Failed to select %s pinstate %d\n",
					PINCTRL_STATE_SUSPEND, ret);
			}
		} else {
			NVT_ERR("Failed to init pinctrl\n");
		}
	}
	mdelay(10);
	mutex_unlock(&ts->lock);

	for (i = 0; i < ts->max_touch_num; i++) {
		input_mt_slot(ts->input_dev, i);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, 0);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}

	input_report_key(ts->input_dev, BTN_TOOL_FINGER, 0);
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
	input_sync(ts->input_dev);

	if (likely(ts->ic_state == NVT_IC_SUSPEND_IN))
		ts->ic_state = NVT_IC_SUSPEND_OUT;
	else
		NVT_ERR("IC state may error,caused by suspend/resume flow, please CHECK!!");
	NVT_LOG("end\n");
	pm_relax(dev);
	return 0;
}

/*******************************************************
Description:
	Novatek touchscreen driver resume function.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
static int32_t nvt_ts_resume(struct device *dev)
{
	int ret;

	if (ts->dev_pm_suspend)
		pm_stay_awake(dev);
	if (!ts->db_wakeup) {
		if (ts->ts_pinctrl) {
			ret = pinctrl_select_state(ts->ts_pinctrl, ts->pinctrl_state_active);

			if (ret < 0) {
				NVT_ERR("Failed to select %s pinstate %d\n",
					PINCTRL_STATE_ACTIVE, ret);
			}
		} else {
			NVT_ERR("Failed to init pinctrl\n");
		}
	}
	if (bTouchIsAwake) {
		NVT_LOG("Touch is already resume\n");
		goto Exit;
	}

	ts->ic_state = NVT_IC_RESUME_IN;

	mutex_lock(&ts->lock);
	NVT_LOG("start\n");

	// please make sure display reset(RESX) sequence and mipi dsi cmds sent before this
#if NVT_TOUCH_SUPPORT_HW_RST
	gpio_set_value(ts->reset_gpio, 1);
#endif
	ret = nvt_update_firmware(ts->fw_name);
	if (ret)
		NVT_ERR("download firmware failed\n");
	nvt_check_fw_reset_state(RESET_STATE_REK);


	nvt_irq_enable(true);
	bTouchIsAwake = 1;

	mutex_unlock(&ts->lock);
	tp_enable_doubleclick(!!ts->db_wakeup);/*if true, dbclick work until next suspend*/
	if (likely(ts->ic_state == NVT_IC_RESUME_IN))
		ts->ic_state = NVT_IC_RESUME_OUT;
	else
		NVT_ERR("IC state may error,caused by suspend/resume flow, please CHECK!!");

	if (ts->gesture_command_delayed >= 0) {
		ts->db_wakeup = ts->gesture_command_delayed;
		ts->gesture_command_delayed = -1;
		NVT_LOG("execute delayed command, set double click wakeup %d\n", ts->db_wakeup);
		tp_enable_doubleclick(!!ts->db_wakeup);
	}
Exit:
	if (ts->dev_pm_suspend)
		pm_relax(dev);
	NVT_LOG("end\n");

	return 0;
}

static int nvt_drm_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct msm_drm_notifier *evdata = data;
	int *blank;
	struct nvt_ts_data *ts_data =
		container_of(self, struct nvt_ts_data, drm_notif);

	if (!evdata || (evdata->id != 0))
		return 0;

	if (evdata->data && ts_data) {
		blank = evdata->data;
		if (event == MSM_DRM_EARLY_EVENT_BLANK) {
			if (*blank == MSM_DRM_BLANK_POWERDOWN) {
				NVT_LOG("event=%lu, *blank=%d\n", event, *blank);
				flush_workqueue(ts_data->event_wq);
				nvt_ts_suspend(&ts_data->client->dev);
			}
		} else if (event == MSM_DRM_EVENT_BLANK) {
			if (*blank == MSM_DRM_BLANK_UNBLANK) {
				NVT_LOG("event=%lu, *blank=%d\n", event, *blank);
				flush_workqueue(ts_data->event_wq);
				queue_work(ts_data->event_wq, &ts_data->resume_work);
			}
		}
	}
	return 0;
}

static int nvt_pm_suspend(struct device *dev)
{
	if (device_may_wakeup(dev) && ts->db_wakeup) {
		NVT_LOG("enable touch irq wake\n");
		enable_irq_wake(ts->client->irq);
	}
	ts->dev_pm_suspend = true;
	reinit_completion(&ts->dev_pm_suspend_completion);

	return 0;
}

static int nvt_pm_resume(struct device *dev)
{
	if (device_may_wakeup(dev) && ts->db_wakeup) {
		NVT_LOG("disable touch irq wake\n");
		disable_irq_wake(ts->client->irq);
	}
	ts->dev_pm_suspend = false;
	complete(&ts->dev_pm_suspend_completion);

	return 0;
}

static const struct dev_pm_ops nvt_dev_pm_ops = {
	.suspend = nvt_pm_suspend,
	.resume = nvt_pm_resume,
};

static const struct platform_device_id nvt_ts_id[] = {
	{ NVT_SPI_NAME, 0 },
	{ }
};

#ifdef CONFIG_OF
static struct of_device_id nvt_match_table[] = {
	{ .compatible = "novatek,NVT-ts-spi",},
	{ },
};
#endif

static struct platform_driver nvt_driver = {
	.probe		= nvt_ts_probe,
	.remove		= nvt_ts_remove,
	.shutdown	= nvt_ts_shutdown,
	.id_table	= nvt_ts_id,
	.driver = {
		.name	= NVT_SPI_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &nvt_dev_pm_ops,
#endif
#ifdef CONFIG_OF
		.of_match_table = nvt_match_table,
#endif
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
};

/*******************************************************
Description:
	Driver Install function.

return:
	Executive Outcomes. 0---succeed. not 0---failed.
********************************************************/
static int32_t __init nvt_driver_init(void)
{
	int32_t ret = 0;

	NVT_LOG("start\n");

	//---add platform driver---
	ret = platform_driver_register(&nvt_driver);
	if (ret) {
		NVT_ERR("failed to add nvt touch driver");
		goto err_driver;
	}

	NVT_LOG("finished\n");

err_driver:
	return ret;
}

/*******************************************************
Description:
	Driver uninstall function.

return:
	n.a.
********************************************************/
static void __exit nvt_driver_exit(void)
{
	platform_driver_unregister(&nvt_driver);
}
late_initcall(nvt_driver_init);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");
