/* drivers/input/touchscreen/zforce.c
 *
 * Copyright (C) 2010 Barnes & Noble, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <mach/gpio.h>
#include <linux/platform_device.h>
#include <linux/zforce.h>
#include <linux/device.h>
#include <linux/sysfs.h>

#define WAIT_TIMEOUT		msecs_to_jiffies(500)

#define FRAME_START		0xEE

// Payload offsets
#define PAYLOAD_HEADER		0
#define PAYLOAD_LENGTH		1
#define PAYLOAD_BODY		2


// Response offsets
#define RESPONSE_ID		0
#define RESPONSE_DATA		1

// Commands
#define COMMAND_DEACTIVATE	0x00
#define COMMAND_ACTIVATE	0x01
#define COMMAND_RESOLUTION	0x02
#define COMMAND_SETCONFIGN	0x03
#define COMMAND_DATAREQUEST	0x04
#define COMMAND_SCANFREQ	0x08
#define COMMAND_VERSION		0x0A
#define COMMAND_PULSESTRENG	0x0F
#define COMMAND_LEVEL		0x1C
#define COMMAND_FORCECAL	0X1A
#define COMMAND_STATUS		0X1E
#define COMMAND_BIST		0X21


// Responses
#define RESPONSE_DEACTIVATE	0x00
#define RESPONSE_ACTIVATE	0x01
#define RESPONSE_RESOLUTION	0x02
#define RESPONSE_SETCONFIGN	0x03
#define RESPONSE_DATAREQUEST	0x04
#define RESPONSE_SCANFREQ	0x08
#define RESPONSE_VERSION	0x0A
#define RESPONSE_PULSESTRENG	0x0F
#define RESPONSE_LEVEL		0x1C
#define RESPONSE_STATUS		0X1E
#define RESPONSE_BIST		0X21
#define RESPONSE_INVALID	0xFE

// Platform specific
#define ZF_COORDATA_SIZE 7

#define ZF_BIST_XDATA_LEN 7
#define ZF_BIST_YDATA_LEN 9

DEFINE_MUTEX(zForce_sysfs_mutex);

static int zforce_synchronized_wait_for_completion_timeout(struct completion *res);

static struct workqueue_struct *zforce_wq;

struct zforce {
	struct input_dev	*input;
	struct i2c_client	*client;
	struct completion	command_done;
	int			command_result;
	int			irq;
	struct work_struct	work;
	char			phys[32];
	u8	zf_status_info[64];
	u8	zf_bist_xresult[ZF_BIST_XDATA_LEN]; //include the axis in 1st byte
	u8	zf_bist_yresult[ZF_BIST_YDATA_LEN]; //TODO make it more generic; once comm spec is finalized.
	int	err_cnt;
};

static u16 major = 0, minor = 0, build = 0, rev = 0;
static u8  tframe_size =0;
static u8 reported_finger_count = 0;

static unsigned short int deep_sleep = 1;

static void zforce_touch_hw_init(int);

static ssize_t deep_sleep_show(struct device *dev, struct device_attribute *attr,
			      char* buf);
static ssize_t deep_sleep_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf,
			       size_t count);

static DEVICE_ATTR(enable_deep_sleep,
		   S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH,
		   deep_sleep_show,
		   deep_sleep_store);

static ssize_t deep_sleep_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	return sprintf(buf, "%hu\n", deep_sleep);
}

static ssize_t deep_sleep_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf,
			       size_t count)
{
	if (sscanf(buf, "%hu", &deep_sleep) != 1)
		return -EINVAL;

	return count;
}
// DATA Request
// [1:cmd]
// #######
static int send_data_request(struct zforce *tsc)
{
	int ret = 0;
	int retry = 3;

	dev_dbg(&tsc->client->dev, "%s()\n", __FUNCTION__);

	while (retry-- > 0)
	{
		ret = i2c_smbus_write_byte(tsc->client, COMMAND_DATAREQUEST);
		if (ret >= 0)
			break;
	}
	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "i2c send data request error: %d\n", ret);
	}
	return ret;
}

// RESOLUTION Request
// [1:cmd] [2:width] [2:height]
// ############################
static int send_resolution(struct zforce *tsc, u16 width, u16 height)
{
	struct i2c_msg msg[2];
	u8 request[16];
	int ret;

	dev_info(&tsc->client->dev, "%s(%d,%d)\n", __FUNCTION__, width, height);

	request[0] = COMMAND_RESOLUTION;
	memcpy(&request[1], &width, sizeof(u16));
	memcpy(&request[3], &height, sizeof(u16));

	msg[0].addr = tsc->client->addr;
	msg[0].flags = 0;
	msg[0].len = 5;
	msg[0].buf = request;

	ret = i2c_transfer(tsc->client->adapter, msg, 1);
	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "i2c send resolution error: %d\n", ret);
		return ret;
	}

	if (zforce_synchronized_wait_for_completion_timeout(&tsc->command_done) == 0)
		return -ETIMEDOUT;

	// I2C operations was successful
	// Return the results from the controller. (0 == success)
	return tsc->command_result;
}

// SETCONFIGURATION Request
// [1:cmd] [2:width] [2:height]
// ############################
static int send_setconfig(struct zforce *tsc, u32 setconfig)
{
	struct i2c_msg msg[2];
	u8 request[16];
	int ret;

	dev_info(&tsc->client->dev, "%s(%d)\n", __FUNCTION__, setconfig);

	request[0] = COMMAND_SETCONFIGN;
	memcpy(&request[1], &setconfig, sizeof(u32));

	msg[0].addr = tsc->client->addr;
	msg[0].flags = 0;
	msg[0].len = 5;
	msg[0].buf = request;

	ret = i2c_transfer(tsc->client->adapter, msg, 1);
	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "i2c send setconfig error: %d\n", ret);
		return ret;
	}

	if (zforce_synchronized_wait_for_completion_timeout(&tsc->command_done) == 0)
		return -ETIMEDOUT;

	// I2C operations was successful
	// Return the results from the controller. (0 == success)
	return tsc->command_result;
}

// Fixed Pulse and Strength Request
// ############################
static int send_pulsestreng(struct zforce *tsc, u8 strength, u8 time)
{
	struct i2c_msg msg[2];
	u8 request[16];
	int ret;

	dev_info(&tsc->client->dev, "%s(%d,%d)\n", __FUNCTION__, strength, time);

	request[0] = COMMAND_PULSESTRENG;
	request[1] = (strength&0x0F) | ( time<<4 ) ;

	msg[0].addr = tsc->client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = request;

	ret = i2c_transfer(tsc->client->adapter, msg, 1);
	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "i2c send pulsestreng error: %d\n", ret);
		return ret;
	}

	if (zforce_synchronized_wait_for_completion_timeout(&tsc->command_done) == 0)
		return -ETIMEDOUT;

	// I2C operations was successful
	// Return the results from the controller. (0 == success)
	return tsc->command_result;
}


// Status Request
// ############################
static int send_status_request(struct zforce *tsc)
{
	struct i2c_msg msg[2];
	u8 request[2];
	int ret;

	dev_info(&tsc->client->dev, "%s\n", __FUNCTION__);
	
	request[0] = COMMAND_STATUS;

	msg[0].addr = tsc->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = request;

	ret = i2c_transfer(tsc->client->adapter, msg, 1);
	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "i2c send status error: %d\n", ret);
		return ret;
	}

	if (zforce_synchronized_wait_for_completion_timeout(&tsc->command_done) == 0)
		return -ETIMEDOUT;

	// I2C operations was successful
	// Return the results from the controller. (0 == success)
	return tsc->command_result;
}

// BIST Request
// ############################
static int send_bist_request(struct zforce *tsc, u8 axis)
{
	struct i2c_msg msg[2];
	u8 request[2];
	int ret;

	dev_info(&tsc->client->dev, "%s\n", __FUNCTION__);

	request[0] = COMMAND_BIST;
	request[1] = axis;
	msg[0].addr = tsc->client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = request;

	ret = i2c_transfer(tsc->client->adapter, msg, 1);
	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "i2c send status error: %d\n", ret);
		return ret;
	}

	if (zforce_synchronized_wait_for_completion_timeout(&tsc->command_done) == 0)
		return -ETIMEDOUT;

	// I2C operations was successful
	// Return the results from the controller. (0 == success)
	return tsc->command_result;
}
// DEACTIVATE Request
// [0:cmd]
// #######
static int send_cmd_request(struct zforce *tsc, u8 cmd)
{
	int ret;
	int retry = 3;

	zforce_info("Enter\n");

	while (retry-- > 0)
	{
		ret = i2c_smbus_write_byte(tsc->client, cmd);
		if (ret >= 0)
			break;
	}
	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "i2c send cmd(%X) request error: %d\n", cmd, ret);
		return ret;
	}

	if (zforce_synchronized_wait_for_completion_timeout(&tsc->command_done) == 0)
		return -ETIMEDOUT;

	// I2C operations was successful
	// Return the results from the controller. (0 == success)
	return tsc->command_result;
}

// DEACTIVATE Request
// [0:cmd]
// #######
static int send_deactivate_request(struct zforce *tsc)
{
	int ret;
	int retry = 3;

	dev_dbg(&tsc->client->dev, "%s()\n", __FUNCTION__);

	while (retry-- > 0)
	{
		ret = i2c_smbus_write_byte(tsc->client, COMMAND_DEACTIVATE);
		if (ret >= 0)
			break;
	}
	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "i2c send deactivate request error: %d\n", ret);
		return ret;
	}

	if (zforce_synchronized_wait_for_completion_timeout(&tsc->command_done) == 0)
		return -ETIMEDOUT;

	// I2C operations was successful
	// Return the results from the controller. (0 == success)
	return tsc->command_result;
}


// ACTIVATE Request
// [1:cmd]
// #######
static int send_activate_request(struct zforce *tsc)
{
	int ret = 0 ;
	int retry = 3;

	dev_dbg(&tsc->client->dev, "%s()\n", __FUNCTION__);

	while (retry-- > 0)
	{
		ret = i2c_smbus_write_byte(tsc->client, COMMAND_ACTIVATE);
		if (ret >= 0)
			break;
	}
	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "i2c send activate request error: %d\n", ret);
		return ret;
	}

	if (zforce_synchronized_wait_for_completion_timeout(&tsc->command_done) == 0)
		return -ETIMEDOUT;

	// I2C operations was successful
	// Return the results from the controller. (0 == success)
	return tsc->command_result;
}

// Force Calibration Request
// [1:cmd]
// #######
static int send_forcecal_request(struct zforce *tsc)
{
	int ret;

	dev_dbg(&tsc->client->dev, "%s()\n", __FUNCTION__);

	ret = i2c_smbus_write_byte(tsc->client, COMMAND_FORCECAL);
	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "i2c send version request error: %d\n", ret);
		return ret;
	}

	return tsc->command_result;
}

// Version Request
// [1:cmd]
// #######
static int send_version_request(struct zforce *tsc)
{
	int ret;

	dev_dbg(&tsc->client->dev, "%s()\n", __FUNCTION__);

	ret = i2c_smbus_write_byte(tsc->client, COMMAND_VERSION);
	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "i2c send version request error: %d\n", ret);
		return ret;
	}

	if (zforce_synchronized_wait_for_completion_timeout(&tsc->command_done) == 0)
		return -ETIMEDOUT;

	return tsc->command_result;
}

// Version Payload Results
// [2:major] [2:minor] [2:build] [2:rev]
// #####################################
static int process_version_response(struct zforce *tsc, u8* payload)
{
	memcpy(&major, &payload[0], sizeof(u16));
	memcpy(&minor, &payload[2], sizeof(u16));
	memcpy(&build, &payload[4], sizeof(u16));
	memcpy(&rev,   &payload[6], sizeof(u16));

	// Request the next event ASAP.
	if (major == 1)
	{
		tframe_size = 5;
	}
	else
	{
		tframe_size = 7;
	}

	dev_info(&tsc->client->dev, "Firmware Version %04x:%04x %04x:%04x\n", major, minor, build, rev);
	return 8;
}

// LED LEVEL Request
// [1:cmd]
// #######
static int send_level_request(struct zforce *tsc)
{
	int ret;

	dev_dbg(&tsc->client->dev, "%s()\n", __FUNCTION__);

	ret = i2c_smbus_write_byte(tsc->client, COMMAND_LEVEL);
	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "i2c send level request error: %d\n", ret);
		return ret;
	}

	if (zforce_synchronized_wait_for_completion_timeout(&tsc->command_done) == 0)
		return -ETIMEDOUT;

	return tsc->command_result;
}
#define ZF_NUMX 11
#define ZF_NUMY 15
#define ZF_X_AXIS 0
#define ZF_Y_AXIS 1
#define ZF_LEDDATA_LEN (2+(ZF_NUMX + ZF_NUMY)*3)
static u8 ledlevel[ZF_LEDDATA_LEN];

// LED Level Payload Results
// [1:x] [1:y] [3*x:xdata] [3*y:ydata]
// #####################################
static int process_level_response(struct zforce *tsc, u8* payload)
{
	int i = 0;

	dev_dbg(&tsc->client->dev, "%s()\n", __FUNCTION__);

	// save data
	for (i = 0; i < ZF_LEDDATA_LEN; i++ )
	{
		ledlevel[i] = payload[i] ;
	}
	return ZF_LEDDATA_LEN;
}


#define STATE_DOWN 0
#define STATE_MOVE 1
#define STATE_UP   2

struct touch_info_data_t tinfo[ZF_NUM_FINGER_SUPPORT];
//
// Clear touch info control block
//
void touchdata_clear(void)
{
	u8 i;
	for( i=0; i< ZF_NUM_FINGER_SUPPORT; i++ )
	{
		tinfo[i].x		= 0;
		tinfo[i].y		= 0;
		tinfo[i].id		= 0;
		tinfo[i].state	= 0;
		tinfo[i].valid	= 0;
		tinfo[i].rsvrd	= 0;
		tinfo[i].prblty	= 0;
		tinfo[i].z		= 0;
	}
}

//
// return the number of touches
// return error otherwise
static u32 framecounter = 0;
int touchdata_collect( u8* payload )
{
	u8 i;
	reported_finger_count = payload[0];
	framecounter++;
	if( reported_finger_count > ZF_NUM_FINGER_SUPPORT )
	{
		zforce_error("Detected (%d) more fingers the max(%d) number supported\n",
				reported_finger_count, ZF_NUM_FINGER_SUPPORT );
		return -EINVAL ;
	}

	for( i=0; i< reported_finger_count; i++ )
	{
		tinfo[i].x	= (u16)((payload[2+i*ZF_COORDATA_SIZE]<<8)|
					payload[1+i*ZF_COORDATA_SIZE]);
		tinfo[i].y	= (u16)((payload[4+i*ZF_COORDATA_SIZE]<<8)|
					payload[3+i*ZF_COORDATA_SIZE]);
		tinfo[i].id	= (u8)((payload[5+i*ZF_COORDATA_SIZE]&0x3C)>>2);
		tinfo[i].state	= (u8)((payload[5+i*ZF_COORDATA_SIZE]&0xC0)>>6);
		tinfo[i].rsvrd	= (u8)( payload[6+i*ZF_COORDATA_SIZE] );
		tinfo[i].prblty	= (u8)( payload[7+i*ZF_COORDATA_SIZE] );
		tinfo[i].valid	= 1;
		tinfo[i].z = reported_finger_count == 0 ? 0 : 20;
	}
	return reported_finger_count;
}

//
// display touch data buffer
//
#ifdef ZF_USE_DEBUG
void touchdata_show(void)
{
	u8 i;

	if (major > 1)
	{
		int i;
		printk("NumFingers=%02d\n", reported_finger_count);
		for(i=0; i<reported_finger_count; i++)
		{
			printk("[%05d](%03d, %03d, %02d) DMU%d ID%d R%02X P%02X V%02d\n",
				framecounter,
				tinfo[i].x,
				tinfo[i].y,
				tinfo[i].z,
				tinfo[i].state,
				tinfo[i].id,
				tinfo[i].rsvrd,
				tinfo[i].prblty,
				tinfo[i].valid );
		}
		printk("\n");
	}
}
#endif

// Fix Pulse Strength  Payload Results
// [1:x] [1:y] [3*x:xdata] [3*y:ydata]
// #####################################
#define ZF_FIXSP_BUFF_SIZE (ZF_NUMX*2+ZF_NUMY*2+2)
static u8 fixps_data[ZF_FIXSP_BUFF_SIZE];
static int process_pulsestreng_response(struct zforce *tsc, u8* payload)
{
	int i = 0;
	int numx = -1, numy = -1;
	int datasize;

	dev_dbg(&tsc->client->dev, "%s()\n", __FUNCTION__);
	numx = payload[0];
	numy = payload[1];
	datasize = (numx+numy+2);
	if( datasize != ZF_FIXSP_BUFF_SIZE )
	{
		dev_err(&tsc->client->dev, "fixps buffer mismatch.(E%d, G%d)(TC%d).\n", ZF_FIXSP_BUFF_SIZE, datasize, ++(tsc->err_cnt) );
	}
	if( datasize > ZF_FIXSP_BUFF_SIZE )
	{
		dev_err(&tsc->client->dev, "fixps buff overflow:(E%d, G%d)(TC%d).\n", ZF_FIXSP_BUFF_SIZE, datasize, ++(tsc->err_cnt) );
		// Clamp datasize to prevent buffer overflow
		datasize = ZF_FIXSP_BUFF_SIZE;
		
	}
	// Save fix pulse strength data
	for(i=0; i<datasize; i++)
	{
		fixps_data[i]=payload[i];
	}
	return ZF_FIXSP_BUFF_SIZE;
}

static int process_status_response(struct zforce *tsc, u8* payload)
{
	u8 *pyld =  NULL;
	
	dev_dbg(&tsc->client->dev, "%s()\n", __FUNCTION__);
	
	pyld = (u8 *)tsc->zf_status_info;
	#define ZF_STATUS_SIZE 64
	memcpy(pyld, payload, ZF_STATUS_SIZE);
	
	return ZF_STATUS_SIZE;
}

static int process_bist_response(struct zforce *tsc, const u8* payload)
{
	u8 *pyld =  NULL;
	u8 len = 0;

	dev_dbg(&tsc->client->dev, "%s()\n", __FUNCTION__);

	if( payload[0] == ZF_X_AXIS )
	{
		pyld = (u8 *)tsc->zf_bist_xresult;
		len = ZF_BIST_XDATA_LEN;
	}
	else if( payload[0] == ZF_Y_AXIS )
	{
		pyld = (u8 *)tsc->zf_bist_yresult;
		len = ZF_BIST_YDATA_LEN;
	}
	else
	{
		return 1;
	}
	memcpy(pyld, payload, len);
	return len;
}

// Touch Payload Results
// [1:count] [2:x] [2:y] [1:state]
// ###############################
static int process_touch_event(struct zforce *tsc, u8* payload)
{
	u16 x,y;
	u8  status;
	int count;
	u8 id = 0;
	u8 state = 0;
	int size = 0;

	// Request the next event ASAP.
	if (major == 1)
	{
		send_data_request(tsc);
		size = 5;
	}
	else
	{
		size = 7;
	}

#ifdef ZF_USE_DEBUG
	// =-=-=-=-=-=-=-=-=-=
	//  Get touch data
	// =-=-=-=-=-=-=-=-=-=
	retval = touchdata_collect( payload ) ;
	if( retval < 0 )
		return retval;

	touchdata_show();
#endif

	count = payload[0];
	#define ZF_COORDATA_SIZE 7
	#ifdef ZF_USE_DEBUG
	if (major > 1)
	{
		int i;
		printk("NumFingers=%02d\n", count);
		for(i=0; i<count; i++)
			printk("(%03d, %03d) %d %d %02X %02X ",
				(int)((payload[2+i*ZF_COORDATA_SIZE]<<8)|payload[1+i*ZF_COORDATA_SIZE]),
				(int)((payload[4+i*ZF_COORDATA_SIZE]<<8)|payload[3+i*ZF_COORDATA_SIZE]),
				(int)((payload[5+i*ZF_COORDATA_SIZE]&0xC0)>>6),
				(int)((payload[5+i*ZF_COORDATA_SIZE]&0x3C)>>2),
				payload[6+i*ZF_COORDATA_SIZE],
				payload[7+i*ZF_COORDATA_SIZE] );
		printk("\n");
	}
	#endif
	if (count != 1)
	{
		dev_dbg(&tsc->client->dev, "Invalid number of coordinates: %d\n", count);
	}
	memcpy(&x, &payload[1], sizeof(u16));
	memcpy(&y, &payload[3], sizeof(u16));
	status = payload[5];

	if (major == 1)
	{
		state = status & 0x03;
		id = 1;
	}
	else
	{
		state = (status & 0xC0) >> 6;
		id =    (status & 0x3C) >> 2;
	}

	//x = 600 - x;
	if (major == 1)
		y = 800 - y;

	// Process
	switch (state)
	{
		case STATE_MOVE:
			dev_dbg(&tsc->client->dev, "%d move(%d,%d)\n", id, x, y);
			input_report_abs(tsc->input, ABS_X, x);
			input_report_abs(tsc->input, ABS_Y, y);
			break;

		case STATE_DOWN:
			dev_dbg(&tsc->client->dev, "%d down(%d,%d)\n", id, x, y);
			input_report_abs(tsc->input, ABS_X, x);
			input_report_abs(tsc->input, ABS_Y, y);
			input_report_key(tsc->input, BTN_TOUCH, 1);
			break;

		case STATE_UP:
			dev_dbg(&tsc->client->dev, "%d up(%d,%d)\n", id, x, y);
			input_report_abs(tsc->input, ABS_X, x);
			input_report_abs(tsc->input, ABS_Y, y);
			input_report_key(tsc->input, BTN_TOUCH, 0);
			break;

		default:
			dev_err(&tsc->client->dev, "Invalid state: %d\n", state);
			return (count * size) + 1;
	}
	input_sync(tsc->input);
	return (count * size) + 1;
}

static int read_packet(struct zforce *tsc, u8 *buffer)
{
	int ret;
	struct i2c_msg msg[2];

	msg[0].addr = tsc->client->addr;
	msg[0].flags = I2C_M_RD;
	msg[0].len = 2;
	msg[0].buf = buffer;

	// Read 2 byte header
	ret = i2c_transfer(tsc->client->adapter, msg, 1);
	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "i2c header error: %d\n", ret);
		return -1;
	}

	if (buffer[PAYLOAD_HEADER] != FRAME_START)
	{
		dev_err(&tsc->client->dev, "invalid frame: %d\n", buffer[0]);
		return -1;
	}

	if ((buffer[PAYLOAD_LENGTH] <= 0) || (buffer[PAYLOAD_LENGTH] > 255))
	{
		dev_err(&tsc->client->dev, "invalid payload length: %d\n", buffer[PAYLOAD_LENGTH]);
		return -1;
	}

	// Read payload
	msg[0].addr = tsc->client->addr;
	msg[0].flags = I2C_M_RD;
	msg[0].len = buffer[PAYLOAD_LENGTH];
	msg[0].buf = &buffer[PAYLOAD_BODY];
	ret = i2c_transfer(tsc->client->adapter, msg, 1);
	if (ret < 0)
	{
		dev_err(&tsc->client->dev, "i2c payload error: %d\n", ret);
		return -1;
	}
	return 0;
}

static int zforce_synchronized_wait_for_completion_timeout(struct completion *res)
{
	int ret = 0;

	mutex_lock(&zForce_sysfs_mutex);
	ret = wait_for_completion_timeout(res, WAIT_TIMEOUT);
	mutex_unlock(&zForce_sysfs_mutex);

	if (ret)
		return 1;

	return 0;
}

// Response Bytes
// [1:0xEE] [1:len] [len:payload]
// ##############################
static void zforce_tsc_work_func(struct work_struct *work)
{
	struct zforce *tsc = container_of(work, struct zforce, work);

	int cmd_len = 0;
	int payload_length = 0;
	u8 payload_buffer[512];
	u8* payload =  NULL;
	u32 i;

	if (read_packet(tsc, payload_buffer) < 0)
		return;

	payload_length = payload_buffer[PAYLOAD_LENGTH];
	payload =  &payload_buffer[PAYLOAD_BODY];

	while (payload_length > 0)
	{
		switch (payload[RESPONSE_ID])
		{
		case  RESPONSE_DATAREQUEST:
			cmd_len = process_touch_event(tsc, &payload[RESPONSE_DATA]);
			break;

		case  RESPONSE_ACTIVATE:
		case  RESPONSE_DEACTIVATE:
		case  RESPONSE_SETCONFIGN:
		case  RESPONSE_RESOLUTION:
			tsc->command_result = payload[RESPONSE_DATA];
			complete(&tsc->command_done);
			cmd_len = 1;
			break;

		case  RESPONSE_VERSION:
			cmd_len = process_version_response(tsc, &payload[RESPONSE_DATA]);
			tsc->command_result = 0;
			complete(&tsc->command_done);
			break;

		case  RESPONSE_LEVEL:
			cmd_len = process_level_response(tsc, &payload[RESPONSE_DATA]);
			tsc->command_result = 0;
			complete(&tsc->command_done);
			break;

		case  RESPONSE_PULSESTRENG:
			cmd_len = process_pulsestreng_response(tsc, &payload[RESPONSE_DATA]);
			tsc->command_result = 0;
			complete(&tsc->command_done);
			break;
		
		case  RESPONSE_STATUS:
			cmd_len = process_status_response(tsc, &payload[RESPONSE_DATA]);
			tsc->command_result = 0;
			complete(&tsc->command_done);
			break;
			
		case  RESPONSE_BIST:
			cmd_len = process_bist_response(tsc, &payload[RESPONSE_DATA]);
			tsc->command_result = 0;
			complete(&tsc->command_done);
			break;

		case  RESPONSE_INVALID:
			cmd_len = 1;
			dev_err(&tsc->client->dev, "Invalid Command ID: %d\n(TC%d)", payload[RESPONSE_DATA], ++(tsc->err_cnt) );
			return;

		default:
			dev_err(&tsc->client->dev, "Unrecognized Response ID: %d (TC%d)\n", payload[RESPONSE_ID], ++(tsc->err_cnt) );
			dev_err(&tsc->client->dev, " Responds frame:  " );
			for( i=0; i < (2+payload_length); i++ ){
				printk( " 0x%02X ", payload_buffer[i] );
			}
			printk( "\n" );
			return;
		}
		cmd_len += 1;  // Compensate for cmd byte.

		payload_length -= cmd_len;
		payload += cmd_len;
	}

	return;
}

static irqreturn_t zforce_tsc_irq_handler(int irq, void *dev)
{
	struct zforce *tsc = dev;

	queue_work(zforce_wq, &tsc->work);
	return IRQ_HANDLED;
}

#define DEBUG_CLK_GPIO   140
#define DEBUG_DATA_GPIO  141
#define HAPT_ENABLE_GPIO  37

static void zforce_touch_hw_init(int resume)
{
	u8 v;

	printk("zforce_touch_hw_init ...\n");
	if (!resume && gpio_request(DEBUG_CLK_GPIO, "DEBUG_CLK_GPIO") < 0)
	{
			printk(KERN_ERR "can't get DEBUG_CLK_GPIO\n");
			return;
	}
	gpio_direction_output(DEBUG_CLK_GPIO, 0);

	if (!resume && gpio_request(DEBUG_DATA_GPIO, "DEBUG_DATA_GPIO") < 0)
	{
			printk(KERN_ERR "can't get DEBUG_DATA_GPIO\n");
			return;
	}
	gpio_direction_output(DEBUG_DATA_GPIO, 0);

	if (!resume && gpio_request(HAPT_ENABLE_GPIO, "HAPT_ENABLE_GPIO") < 0)
	{
			printk(KERN_ERR "can't get HAPT_ENABLE_GPIO\n");
			return;
	}
	gpio_direction_output(HAPT_ENABLE_GPIO, 0);
	//No need to turn power off; as it is already off
	//gpio_set_value(HAPT_ENABLE_GPIO, 0);
	gpio_set_value(DEBUG_CLK_GPIO, 0);
	gpio_set_value(DEBUG_DATA_GPIO, 0); // data line doubles as 430 reset; pull reset low
    gpio_set_value(HAPT_ENABLE_GPIO, 1);// apply power to touch
	mdelay(10);
	gpio_set_value(DEBUG_CLK_GPIO, 0);
	gpio_set_value(DEBUG_DATA_GPIO, 1);//de-assert reset

	/* Clear events possibly logged in UART2 by
	   the init procedure - this is a hack but
	   pulling in the structures defined in omap-serial.c
	   would bloat the driver without reason */
	/* Reset UART fifos */
	v = omap_readb(OMAP_UART2_BASE + 0x08);
	v |= 0x06;
	omap_writeb(v, OMAP_UART2_BASE + 0x08);

	mdelay(50);
}
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-==-=-=-
// sysfs
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-==-=-=-

static ssize_t zforce_ledlevel_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int cnt = 0;
	u8 xcount, ycount;
	int offset = 0;
	int i = 0;
	u8* led_info = NULL;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct zforce *tsc = i2c_get_clientdata(client);

	zforce_info("Enter\n");

	// Get LED Levels
	if (send_level_request(tsc))
	{
		dev_err(&tsc->client->dev, "Unable to request levels\n");
	}

	xcount = ledlevel[0];
	ycount = ledlevel[1];
	// Bounds check
	if( ZF_NUMX != xcount || ZF_NUMY != ycount )
	{
		dev_err(&tsc->client->dev, "Wrong touch dimension.\n");
		return -EINVAL;
	}
	zforce_info("LedCount x=%d, y=%d\n", xcount, ycount);

	offset = 2;
	for (i = 0; i < xcount; i++)
	{
		led_info = &ledlevel[offset + (i*3)];
		if( PAGE_SIZE < (cnt+16) )
		{
			dev_err(&tsc->client->dev, "SYSFS buffer overflow predicted.\n");
			return -ENFILE;
		}

		cnt = cnt + sprintf(&buf[cnt], "%02d %02d %03d %03d ", (led_info[0]&0xF0)>>4, led_info[0]&0x0F, led_info[1], led_info[2]);
	}
	cnt = cnt + sprintf(&buf[cnt], "\n" );
	offset += (xcount * 3);
	for (i = 0; i < ycount; i++)
	{
		led_info = &ledlevel[offset + (i*3)];
		if( PAGE_SIZE < (cnt+16) )
		{
			dev_err(&tsc->client->dev, "SYSFS Buffer overflow predicted.\n");
			return -ENFILE;
		}
		cnt = cnt + sprintf(&buf[cnt], "%02d %02d %03d %03d ",  (led_info[0]&0xF0)>>4, led_info[0]&0x0F, led_info[1], led_info[2]);
	}
	cnt = cnt + sprintf(&buf[cnt], "\n" );

	return cnt;
}

static ssize_t zforce_ledlevel_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	/* place holder for future use */
	return -ENOTSUPP;
}

static ssize_t zforce_versions_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int cnt = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct zforce *tsc = NULL;
	int ret = 0;


	if (client){
		tsc  = i2c_get_clientdata(client);
	}

	zforce_info("Request fw ver from zforce\n");

	if (!zforce_wq){/*DRIVER PROBE FAILED */
		cnt = cnt + sprintf(&buf[cnt],"%04x:%04x %04x:%04x\n", 0, 0, 0, 0);
		printk("sending touch firmware version 00000\n");
		return cnt;
	}
	else if (tsc){
		ret = send_version_request(tsc);
		if (ret){
			dev_err(&client->dev, "UnableToRequestVersion\n");
		}
	}
	cnt = cnt + sprintf(&buf[cnt],"%04x:%04x %04x:%04x\n", major, minor, build, rev);

	return cnt;
}

static ssize_t zforce_versions_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	/* place holder for future use */
	return -ENOTSUPP;
}


static u8 fixps_stren	= 12;
static u8 fixps_time	= 3;

static ssize_t zforce_pulsestrength_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i, cnt = 0;
	int numx, numy;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct zforce *tsc = i2c_get_clientdata(client);

	zforce_info("Pulse width and led strength\n");
	#define ZF_FIXTIME 1
	#define ZF_FIXSTRENGTH 12
	if( fixps_time > 0xF && fixps_stren > 0xF )
	{
		return -EINVAL;
	}
	//TODO: make time & strength sysfs entries
	if( send_pulsestreng(tsc, fixps_stren, fixps_time) )
	{
		dev_err(&client->dev, "Unable to fix pulse strenght\n");
		return -EINVAL;
	}
	numx = fixps_data[0];
	numy = fixps_data[1];

	// print data
	// TODO: add check for PAGESIZE overflow
	printk("\nNum xdata:%d; Num ydata:%d\n", numx, numy);
	cnt = cnt + sprintf(&buf[cnt],"X PD:");

	for (i = 2; i < (2+numx); i++ )
	{
		cnt = cnt + sprintf(&buf[cnt]," %03d", fixps_data[i]);
	}
	cnt = cnt + sprintf(&buf[cnt],"\nY PD:");
	for ( i=(2+numx); i < (2+numx+numy); i++ )
	{
		cnt = cnt + sprintf(&buf[cnt]," %03d", fixps_data[i]);
	}
	cnt = cnt + sprintf(&buf[cnt],"\n");

	return cnt;
}

static ssize_t zforce_pulsestrength_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t size)
{
	unsigned long value;
	int err = 0;
	if (size > 3)
		return -EINVAL;
	err = strict_strtoul(buf, 16, &value);
	if (err != 0)
		return err;
	fixps_stren=(u8)((0xF0&value)>>4);
	fixps_time=value&0x0F;
	printk("\n%s: %d %d; T%d, S%d\n", __FUNCTION__, (int)value, (int)size, (u8)fixps_stren, (u8)fixps_time);
	return size;
}


static ssize_t zforce_forcecal_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -ENOTSUPP;
}

static ssize_t zforce_on_off_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct zforce *tsc = i2c_get_clientdata(client);
	int err = 0;
	unsigned long value;

	zforce_info("Enter\n");

	if (size > 2)
			return -EINVAL;

	err = strict_strtoul(buf, 10, &value);
	if (err != 0)
			return err;

	switch (value) 
	{
	case 0:
		// request deactivate
		if (send_deactivate_request(tsc))
		{
			dev_err(&tsc->client->dev, "Unable to request zfdeactivate\n");
			return -1;
		}
		printk("%s: zforce deactivated.\n", __FUNCTION__);
		break;
	case 1:
		// request activate
		if (send_activate_request(tsc))
		{
			dev_err(&tsc->client->dev, "Unable to request zfactivate\n");
			return -1;
		}
		printk("%s: zforced activated\n", __FUNCTION__);
		break;

	default:
			break;
	}
	return size;

}

static ssize_t zforce_cmd_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct zforce *tsc = i2c_get_clientdata(client);
	int err = 0;
	unsigned long value;

	zforce_info("Enter (%d)\n", size);

	if (size > 2)
			return -EINVAL;

	err = strict_strtoul(buf, 16, &value);
	
	if (err != 0)
			return err;
	zforce_info("Enter (%X)\n", (u8)value);

	if( value > 0xFF )
		return -EINVAL;
	if (send_cmd_request(tsc, (u8)value))
	{
		dev_err(&tsc->client->dev, "Unable to request cmd %X\n", (u8)value);
		return -1;
	}
	zforce_debug("zforce cmd %X send.\n", (u8)value);
	return size;
}

static ssize_t zforce_forcecal_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct zforce *tsc = i2c_get_clientdata(client);
	int err = 0;
	unsigned long value;

	zforce_info("Enter\n");

	if (size > 2)
			return -EINVAL;

	err = strict_strtoul(buf, 10, &value);
	if (err != 0)
			return err;

	switch (value) {
	case 1:
		// request force calibration
		if (send_forcecal_request(tsc))
		{
			dev_err(&tsc->client->dev, "Unable to request forcecal\n");
			return -1;
		}
		printk("%s: Forcing TP cal NOW.\n", __FUNCTION__);
		break;
	default:
			break;
	}
	return size;

}


// =-=-=-=-=-=-=-=-=-=-=-=-=
//  Zforce internal status 
// =-=-=-=-=-=-=-=-=-=-=-=-=
static ssize_t zforce_zfstatus_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i, cnt = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct zforce *tsc = i2c_get_clientdata(client);
	u8 *payload = (u8 *)tsc->zf_status_info;

	// move to tsc struct
	u8 act, sr_idle, sr_full, sa_x_start, sa_x_end, sa_y_start, sa_y_end;
	u16 scancounter, xres, yres;
	u32 cfg;
	
	zforce_info("Zforce internal status strength\n");

	if( send_status_request(tsc) )
	{
		dev_err(&client->dev, "Unable retrieve zforce status.\n");
		return -EINVAL;
	}

	// Total error count
	zforce_debug("TTLERR, %0d \n", tsc->err_cnt);
	cnt = cnt + sprintf(&buf[cnt],"TTLERR, %0d \n", tsc->err_cnt);
	

	// Show Versions
	zforce_debug("TPFWVER, %04x:%04x %04x:%04x ;\n", major, minor, build, rev);
	cnt = cnt + sprintf(&buf[cnt],"TPFWVER, %04x:%04x %04x:%04x ;\n", major, minor, build, rev);	
	// Show active status and scan counter
	act=payload[8];
	zforce_debug("ACT, %02d \n", act);
	cnt = cnt + sprintf(&buf[cnt],"ACT, %02d \n", act);
	
	// scan counter
	memcpy(&scancounter, &payload[9], sizeof(u16));
	zforce_debug("SCNTR, %d \n", scancounter);
	cnt = cnt + sprintf(&buf[cnt],"SCNTR, %d \n", scancounter);
	
	// Show cfg (11, 12, 13, 14) and sr (15, 16)
	memcpy(&cfg, &payload[11], sizeof(u32));
	zforce_debug("CFG, %04X \n", cfg);
	cnt = cnt + sprintf(&buf[cnt],"CFG, %04X \n", cfg);
	
	sr_idle = payload[15];
	sr_full = payload[16];
	zforce_debug("SR_IDFU, %d, %d\n", sr_idle, sr_full);
	cnt = cnt + sprintf(&buf[cnt],"SR_IDFU, %d, %d\n", sr_idle, sr_full);
	
	// 17th byte is resvd
	
	// xres (18, 19 ), yres( 20, 21 )
	memcpy(&xres, &payload[18], sizeof(u16));
	memcpy(&yres, &payload[20], sizeof(u16));
	zforce_debug("XYRES, %d, %d \n", xres, yres);	
	cnt = cnt + sprintf(&buf[cnt],"XYRES, %d, %d \n", xres, yres);
	
	//active led (scan area)
	sa_x_start 	= payload[22];
	sa_x_end 	= payload[23];
	sa_y_start	= payload[24];
	sa_y_end	= payload[25];
	zforce_debug("SAXYSE, %d, %d, %d, %d \n", sa_x_start, sa_x_end, sa_y_start, sa_y_end);
	cnt = cnt + sprintf(&buf[cnt],"SAXYSE, %d, %d, %d, %d \n", sa_x_start, sa_x_end, sa_y_start, sa_y_end);

	zforce_debug("STAT_RESV:");
	cnt = cnt + sprintf(&buf[cnt],"STAT_RESV:");
	for( i=26; i<64; i++ )
	{
		printk("%02X ", payload[i] ); 
		cnt = cnt + sprintf(&buf[cnt],"%02X ", payload[i] ); 
	}
	printk("\n");
	cnt = cnt + sprintf(&buf[cnt], "\n");
	return cnt;
}

// =-=-=-=-=-=-=-=-=-=-=-=-=
//  Zforce BIST
// =-=-=-=-=-=-=-=-=-=-=-=-=
static ssize_t zforce_bist_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int i, cnt = 0, ret = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct zforce *tsc = i2c_get_clientdata(client);
	u8 *payload = (u8 *)tsc->zf_bist_xresult;

	zforce_info("Zforce internal status strength\n");

	// run open and short test for x axis
	ret = send_bist_request(tsc, ZF_X_AXIS);
	if(ret)
	{
		dev_err(&client->dev, "Unable to retrieve zforce status: %x.\n", ret);
		return -EINVAL;
	}

	// run open and short test for y axis
	ret = send_bist_request(tsc, ZF_Y_AXIS);
	if(ret)
	{
		dev_err(&client->dev, "Unable to retrieve zforce status: %x.\n", ret);
		return -EINVAL;
	}

	cnt = cnt + sprintf(&buf[cnt],"BISTX:");
	for( i=0; i<ZF_BIST_XDATA_LEN; i++ )
	{
		cnt = cnt + sprintf(&buf[cnt]," %02X", payload[i] );
	}

	payload = (u8 *)tsc->zf_bist_yresult;
	cnt = cnt + sprintf(&buf[cnt],"\nBISTY:");
	for( i=0; i<ZF_BIST_YDATA_LEN; i++ )
	{
		cnt = cnt + sprintf(&buf[cnt]," %02X", payload[i] );
	}
	cnt = cnt + sprintf(&buf[cnt], "\n");
	return cnt;
}



static DEVICE_ATTR(ledlevel, S_IRUGO|S_IWUSR, zforce_ledlevel_show, zforce_ledlevel_store);
static DEVICE_ATTR(versions, S_IRUGO|S_IWUSR, zforce_versions_show, zforce_versions_store);
static DEVICE_ATTR(forcecal, S_IRUGO|S_IWUSR, zforce_forcecal_show, zforce_forcecal_store);
static DEVICE_ATTR(fixps, S_IRUGO|S_IWUSR, zforce_pulsestrength_show, zforce_pulsestrength_store);
static DEVICE_ATTR(zfstatus, S_IRUGO, zforce_zfstatus_show, NULL);
static DEVICE_ATTR(bist, S_IRUGO, zforce_bist_show, NULL);
static DEVICE_ATTR(on_off, S_IWUSR, NULL, zforce_on_off_store);
static DEVICE_ATTR(cmd,	S_IWUSR, NULL, zforce_cmd_store);

static struct attribute *zforce_attributes[] = {
	&dev_attr_ledlevel.attr,
	&dev_attr_versions.attr,
	&dev_attr_forcecal.attr,
	&dev_attr_fixps.attr,
	&dev_attr_zfstatus.attr,
	&dev_attr_bist.attr,
	&dev_attr_on_off.attr,
	&dev_attr_cmd.attr,
	NULL
};

static struct attribute_group zforce_attribute_group = {
	.attrs = zforce_attributes
};

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-==-=-=-

static int zforce_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct zforce *tsc;
	struct zforce_platform_data *pdata = client->dev.platform_data;
	struct input_dev *input_dev;
	int err;

	if (!pdata)
	{
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk(KERN_ERR "zforce_probe: need I2C_FUNC_I2C\n");
		return -ENODEV;
	}

	zforce_touch_hw_init(0);

	tsc = kzalloc(sizeof(struct zforce), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!tsc || !input_dev)
	{
		err = -ENOMEM;
		goto err_free_mem;
	}

	INIT_WORK(&tsc->work, zforce_tsc_work_func);
	init_completion(&tsc->command_done);

	//--== sysfs entries ==--
	err = sysfs_create_group(&client->dev.kobj, &zforce_attribute_group);
	if (err)
	{
		zforce_alert("sysfs_create_group() failed!!\n");
		goto err_free_mem;
	}

	tsc->client = client;
	i2c_set_clientdata(client, tsc);

	tsc->input = input_dev;

	input_dev->name = "zForce Touchscreen";
	input_dev->id.bustype = BUS_I2C;

	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_X, 0, pdata->width, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, pdata->height, 0, 0);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, pdata->width, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, pdata->height, 0, 0);

	tsc->irq = client->irq;
	tsc->err_cnt = 0;

	err = input_register_device(input_dev);
	if (err)
		goto err_free_mem;

	err = request_irq(client->irq, zforce_tsc_irq_handler, pdata->irqflags, client->name, tsc);
	if (err < 0)
	{
		dev_err(&client->dev, "Unable to register irq %d\n", tsc->irq);
		goto err_free_dev;
	}

	dev_info(&client->dev, "registered with irq (%d)\n", client->irq);

	// We are now ready for some events..
	if (send_activate_request(tsc))
	{
		dev_err(&client->dev, "Unable to activate\n");
		goto err_free_irq;
	}

	// Set the touch panel width & height
	if (send_resolution(tsc, pdata->width, pdata->height))
	{
		dev_err(&client->dev, "Unable to set resolution\n");
		goto err_free_irq;
	}

	#define ZF_SETCONFIG_DUALTOUCH 0x00000001
	// Set configuration, enable dual touch
	if (send_setconfig(tsc, ZF_SETCONFIG_DUALTOUCH))
	{
		dev_err(&client->dev, "Unable to set config\n");
		goto err_free_irq;
	}

	// Get Firmware version and frame size
	if (send_version_request(tsc))
	{
		dev_err(&client->dev, "Unable to request version\n");
	}

	// This will start sending touch events.
	if (send_data_request(tsc))
	{
		dev_err(&client->dev, "Unable to request data\n");
		goto err_free_irq;
	}

	// Per NN, initial cal. take max. of 200msec.
	// Allow time to complete this calibration
	msleep(200);

	err = sysfs_create_file(&client->dev.kobj, &dev_attr_enable_deep_sleep.attr);
	if (err != 0) {
		dev_err(&client->dev, "Unable to create sysfs entry\n");
		goto err_free_irq;
	}

	return 0;

 err_free_irq:
	free_irq(tsc->irq, tsc);

 err_free_dev:
	if (zforce_wq)
	{
		destroy_workqueue(zforce_wq);
		zforce_wq = NULL;
	}

	input_unregister_device(input_dev);
	input_dev = NULL;

 err_free_mem:
	input_free_device(input_dev);
	kfree(tsc);
	return err;
}

static int zforce_remove(struct i2c_client *client)
{
	struct zforce *tsc = i2c_get_clientdata(client);
	struct zforce_platform_data *pdata;

	pdata = client->dev.platform_data;
	/* housekeeping */
	sysfs_remove_group(&client->dev.kobj, &zforce_attribute_group);

	free_irq(tsc->irq, tsc);
	input_unregister_device(tsc->input);
	kfree(tsc);

	return 0;
}

#if defined (CONFIG_MACH_OMAP3621_GOSSAMER)
static int zforce_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct zforce *tsc = i2c_get_clientdata(client);
	
	disable_irq(tsc->irq);
	flush_workqueue(zforce_wq);
	gpio_set_value(DEBUG_DATA_GPIO, 0); // data line doubles as 430 reset; pull reset low
	/*
	 * disable power only if told to do so
	 */
	if (deep_sleep != 0)
		gpio_direction_output(HAPT_ENABLE_GPIO, 0);

	return 0;
}

static int zforce_shutdown(struct i2c_client *client, pm_message_t mesg)
{
	struct zforce *tsc = i2c_get_clientdata(client);

	dev_info(&client->dev, "ZforceDrvrShtdwn\n");
	disable_irq(tsc->irq);
	flush_workqueue(zforce_wq);	
	gpio_set_value(DEBUG_DATA_GPIO, 0); // data line doubles as 430 reset; pull reset low
	gpio_direction_output(HAPT_ENABLE_GPIO, 0);

	return 0;
}

static int zforce_resume(struct i2c_client *client)
{
	struct zforce *tsc = i2c_get_clientdata(client);
	struct zforce_platform_data *pdata = client->dev.platform_data;

	/*
	 * power was not disabled during suspend,
	 * skip re-initialization
	 */
	if (deep_sleep == 0)
		return 0;

	zforce_touch_hw_init(1);
	enable_irq(tsc->irq);

	// TODO touch fw 2.0.0.13 and later will have default 
	//      resolution and config to speed up suspend&resume
	// TODO Therefore read version first and skip setting 
	//	resolution and setconfig if newer fw is running.
	if (send_activate_request(tsc) ||
		send_resolution(tsc, pdata->width, pdata->height) ||
		send_setconfig(tsc, ZF_SETCONFIG_DUALTOUCH)) 
	{
		dev_err(&client->dev, "Unable to wakeup the device\n");
	}
	if (send_data_request(tsc))
	{
		dev_err(&client->dev, "Unable to request data\n");
	}
	// Allow time for initial cal to complete
	msleep(200);
	
	if (send_version_request(tsc)) 
	{
		dev_err(&client->dev, "Unable to request version or level data\n");
	}

	return 0;
}
#endif /* CONFIG_MACH_OMAP3621_GOSSAMER */

static struct i2c_device_id zforce_idtable[] = {
	{ ZFORCE_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, zforce_idtable);

static struct i2c_driver zforce_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= ZFORCE_NAME
	},
	.id_table	= zforce_idtable,
	.probe		= zforce_probe,
	.remove		= zforce_remove,
#if defined (CONFIG_MACH_OMAP3621_GOSSAMER)
	.suspend	= zforce_suspend,
	.resume		= zforce_resume,
	.shutdown	= zforce_shutdown,
#endif /* CONFIG_MACH_OMAP3621_GOSSAMER */
};

static int __init zforce_init(void)
{
	zforce_wq = create_singlethread_workqueue("zforce_wq");
	if (!zforce_wq)
		return -ENOMEM;
	return i2c_add_driver(&zforce_driver);
}

static void __exit zforce_exit(void)
{
	i2c_del_driver(&zforce_driver);
	if (zforce_wq)
		destroy_workqueue(zforce_wq);
}

module_init(zforce_init);
module_exit(zforce_exit);

MODULE_AUTHOR("Pieter Truter");
MODULE_DESCRIPTION("zForce TouchScreen Driver");
MODULE_LICENSE("GPL");

