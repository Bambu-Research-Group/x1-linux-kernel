/*
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * * VERSION		DATE			AUTHOR		Note
 *
 */

#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
/* #include <soc/sprd/regulator.h> */
#include <linux/input/mt.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>

#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/err.h>

#include <linux/proc_fs.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <asm/unistd.h>
#include <asm/io.h>
#include <linux/pm_runtime.h>
#include <linux/types.h>
#include "tlsc6x_main.h"

#if defined(CONFIG_ADF)
#include <linux/notifier.h>
#include <video/adf_notifier.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#ifdef TP_GESTRUE
#include <linux/fs.h>
#include <linux/uaccess.h>
#endif

#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/ioctl.h>
#include <linux/wakelock.h>
#include <linux/suspend.h>
#include <linux/irq.h>
#include <linux/string.h>


#ifdef TP_GESTRUE
static int tlsc6x_read_Gestruedata(void);
static unsigned char gesture_enable = 0;
static unsigned char gesture_state = 0;
#if 1
static struct wake_lock gesture_timeout_wakelock;
#endif
#define GESTURE_LEFT		0x20
#define GESTURE_RIGHT		0x21
#define GESTURE_UP		    0x22
#define GESTURE_DOWN		0x23
#define GESTURE_DOUBLECLICK	0x24
#define GESTURE_O		    0x30
#define GESTURE_W		    0x31
#define GESTURE_M		    0x32
#define GESTURE_E		    0x33
#define GESTURE_C		    0x34
#define GESTURE_S           0x46
#define GESTURE_V           0x54
#define GESTURE_Z           0x65
#define GESTURE_L           0x44
#endif

 //#define	TOUCH_VIRTUAL_KEYS
#define	MULTI_PROTOCOL_TYPE_B	1
#define	TS_MAX_FINGER		2

#define MAX_CHIP_ID   (10)
#define TS_NAME		"tlsc6x_ts"
unsigned char tlsc6x_chip_name[MAX_CHIP_ID][20] = {"null", "tlsc6206a", "0x6306", "tlsc6206", "tlsc6324", "tlsc6332", "tlsc6440","tlsc6432","tlsc6424","tlsc6448"};

int g_is_telink_comp = 0;
extern unsigned int g_mccode;
struct tlsc6x_data *g_tp_drvdata = NULL;
static struct i2c_client *this_client;
static struct wake_lock tlsc6x_wakelock;

DEFINE_MUTEX(i2c_rw_access);

#if defined(CONFIG_ADF)
static int tlsc6x_adf_suspend(void);
static int tlsc6x_adf_resume(void);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void tlsc6x_ts_suspend(struct early_suspend *handler);
static void tlsc6x_ts_resume(struct early_suspend *handler);
#endif

#ifdef TLSC_ESD_HELPER_EN
static int tpd_esd_flag = 0;
static struct hrtimer tpd_esd_kthread_timer;
static DECLARE_WAIT_QUEUE_HEAD(tpd_esd_waiter);

static int tpd_esd_flag_status = 0;
static struct hrtimer tpd_esd_kthread_timer_status;
static DECLARE_WAIT_QUEUE_HEAD(tpd_esd_waiter_status);
#endif

#ifdef TLSC_TPD_PROXIMITY
unsigned char tpd_prox_old_state = 0;
static int tpd_prox_active = 0;
static struct class *sprd_tpd_class;
static struct device *sprd_ps_cmd_dev;
#endif

static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct tlsc6x_data *data = i2c_get_clientdata(this_client);
	struct tlsc6x_platform_data *pdata = data->platform_data;

	return snprintf(buf, PAGE_SIZE, "%s:%s:%d:%d:%d:%d:%s:%s:%d:%d:%d:%d:%s:%s:%d:%d:%d:%d\n",
			__stringify(EV_KEY), __stringify(KEY_APPSELECT),
			pdata->virtualkeys[0], pdata->virtualkeys[1], pdata->virtualkeys[2],
		       pdata->virtualkeys[3]
		       , __stringify(EV_KEY), __stringify(KEY_HOMEPAGE), pdata->virtualkeys[4], pdata->virtualkeys[5],
		       pdata->virtualkeys[6], pdata->virtualkeys[7]
		       , __stringify(EV_KEY), __stringify(KEY_BACK), pdata->virtualkeys[8], pdata->virtualkeys[9],
		       pdata->virtualkeys[10], pdata->virtualkeys[11]);
}

static struct kobj_attribute virtual_keys_attr = {
	.attr = {
		 .name = "virtualkeys.tlsc6x_touch",
		 .mode = 0444,
		 },
	.show = &virtual_keys_show,
};

static struct attribute *properties_attrs[] = {
	&virtual_keys_attr.attr,
	NULL
};

static struct attribute_group properties_attr_group = {
	.attrs = properties_attrs,
};

static void tlsc6x_virtual_keys_init(void)
{
	int ret = 0;
	struct kobject *properties_kobj;

	TLSC_FUNC_ENTER();

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj) {
		ret = sysfs_create_group(properties_kobj, &properties_attr_group);
	}
	if (!properties_kobj || ret) {
		tlsc_err("failed to create board_properties\n");
	}
}

/*
    iic access interface
*/
int tlsc6x_i2c_read_sub(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen)
{
	int ret = 0;

	if (client == NULL) {
		tlsc_err("[IIC][%s]i2c_client==NULL!\n", __func__);
		return -EINVAL;
	}

	if (readlen > 0) {
		if (writelen > 0) {
			struct i2c_msg msgs[] = {
				{
				 .addr = client->addr,
				 .flags = 0,
				 .len = writelen,
				 .buf = writebuf,
				 },
				{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
				 },
			};
			ret = i2c_transfer(client->adapter, msgs, 1);
			if (ret < 0) {
				tlsc_err("[IIC]: i2c_transfer(2) error, addr= 0x%x!!\n", writebuf[0]);
				tlsc_err("[IIC]: i2c_transfer(2) error, ret=%d, rlen=%d, wlen=%d!!\n", ret, readlen,
				       writelen);
			}
			else {
				ret = i2c_transfer(client->adapter, &msgs[1], 1);
				if (ret < 0) {
					tlsc_err("[IIC]: i2c_transfer(2) error, addr= 0x%x!!\n", writebuf[0]);
					tlsc_err("[IIC]: i2c_transfer(2) error, ret=%d, rlen=%d, wlen=%d!!\n", ret, readlen,
					       writelen);
				}
			}
		} else {
			struct i2c_msg msgs[] = {
				{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
				 },
			};
			ret = i2c_transfer(client->adapter, msgs, 1);
			if (ret < 0) {
				tlsc_err("[IIC]: i2c_transfer(read) error, ret=%d, rlen=%d, wlen=%d!!", ret, readlen,
				       writelen);
			}
		}
	}

	return ret;
}

/* fail : <0 */
int tlsc6x_i2c_read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen)
{
	int ret = 0;

	/* lock in this function so we can do direct mode iic transfer in debug fun */
	mutex_lock(&i2c_rw_access);
	ret = tlsc6x_i2c_read_sub(client, writebuf, writelen, readbuf, readlen);

	mutex_unlock(&i2c_rw_access);

	return ret;
}

/* fail : <0 */
int tlsc6x_i2c_write_sub(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret = 0;

	if (client == NULL) {
		tlsc_err("[IIC][%s]i2c_client==NULL!\n", __func__);
		return -EINVAL;
	}

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0) {
			tlsc_err("[IIC]: i2c_transfer(write) error, ret=%d!!\n", ret);
		}
	}

	return ret;

}

/* fail : <0 */
int tlsc6x_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret = 0;

	mutex_lock(&i2c_rw_access);
	ret = tlsc6x_i2c_write_sub(client, writebuf, writelen);
	mutex_unlock(&i2c_rw_access);

	return ret;

}

/* fail : <0 */
int tlsc6x_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = { 0 };

	buf[0] = regaddr;
	buf[1] = regvalue;

	return tlsc6x_i2c_write(client, buf, sizeof(buf));
}

/* fail : <0 */
int tlsc6x_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
	return tlsc6x_i2c_read(client, &regaddr, 1, regvalue, 1);
}

static void tlsc6x_clear_report_data(struct tlsc6x_data *drvdata)
{
	int i;

	for (i = 0; i < TS_MAX_FINGER; i++) {
#if MULTI_PROTOCOL_TYPE_B
		input_mt_slot(drvdata->input_dev, i);
		input_mt_report_slot_state(drvdata->input_dev, MT_TOOL_FINGER, false);
#endif
	}

	input_report_key(drvdata->input_dev, BTN_TOUCH, 0);
#if !MULTI_PROTOCOL_TYPE_B
	input_mt_sync(drvdata->input_dev);
#endif
	input_sync(drvdata->input_dev);
}

void tlsc6x_irq_disable(void)
{
	TLSC_FUNC_ENTER();
	if (!g_tp_drvdata->irq_disabled) {
		disable_irq_nosync(this_client->irq);
		g_tp_drvdata->irq_disabled = true;
	}
}

void tlsc6x_irq_enable(void)
{
	TLSC_FUNC_ENTER();
	if (g_tp_drvdata->irq_disabled) {
		enable_irq(this_client->irq);
		g_tp_drvdata->irq_disabled = false;
	}
}

#ifdef TLSC_APK_DEBUG
static unsigned char send_data_flag = 0;
static unsigned char get_data_flag = 0;
static int send_count = 0;
static unsigned char get_data_buf[10] = {0};
static char buf_in[1026] = {0};
static char buf_out[1026] = {0};


void get_data_start(unsigned char *local_buf)
{
    u8 writebuf[4];
    
    g_tp_drvdata->esdHelperFreeze = 1;
    
    tlsc6x_set_dd_mode();
    {
        writebuf[0] = 0x9f; 
        writebuf[1] = 0x22; 
        writebuf[2] = local_buf[2]; 
        writebuf[3] = local_buf[3]; 
        tlsc6x_i2c_write(this_client, writebuf, 4);

        writebuf[0] = 0x9f; 
        writebuf[1] = 0x20; 
        writebuf[2] = 61; 
        tlsc6x_i2c_write(this_client, writebuf, 3);

        writebuf[0] = 0x9f; 
        writebuf[1] = 0x24; 
        writebuf[2] = 0x01; 
        tlsc6x_i2c_write(this_client, writebuf, 3);
    }
}

void get_data_stop(void)
{
    u8 writebuf[4];
    
    writebuf[0] = 0x9f; 
    writebuf[1] = 0x22; 
    writebuf[2] = 0xff; 
    writebuf[3] = 0xff; 
    tlsc6x_i2c_write(this_client, writebuf, 4);		
    msleep(20);
    g_tp_drvdata->esdHelperFreeze = 0;
    send_data_flag = 0;
    tlsc6x_set_nor_mode();
}

int tssc_get_debug_info(struct i2c_client *i2c_client, char *p_data)
{
	char writebuf[10] = {0};
	short size = 61;
	static unsigned int cnt;
	unsigned char loop, k;
	unsigned char cmd[2];
	unsigned short check, rel_size;
	char buft[128];
	unsigned short *p16_buf = (unsigned short *)buft;
	cmd[0] = 1;

	loop = 0;
	rel_size = size * 2;
	
	while (loop++ < 2) {
		writebuf[0] = 0x9f; 
		writebuf[1] = 0x26; 
		tlsc6x_i2c_read(i2c_client, writebuf,  2, buft, rel_size + 2);  //124 2->checksum
		for (k = 0, check = 0; k < size; k++) {
			check += p16_buf[k];
		}
		if (check == p16_buf[size]) {
			p16_buf[size] = 0x5555;
			break;
		} else {
			p16_buf[size] = 0xaaaa;
		}
	}
	buft[124] = (cnt) & 0xff;
	buft[125] = (cnt >> 8) & 0xff;
	buft[126] = (cnt >> 16) & 0xff;
	buft[127] = (cnt >> 24) & 0xff;
	cnt++;
        
        memcpy(&buf_in[send_count * 128], buft, (sizeof(char) * 128));
        if(send_count++ >= 7){
            memcpy(buf_out, buf_in, (sizeof(char) * 1024));
            memset(buf_in, 0xff,(sizeof(char) * 1024));
            send_count = 0;
            get_data_flag = 1;
        }

    {
        static unsigned char msk_o;
        unsigned char msk = 0;
        unsigned short *p_point = &p16_buf[61 - 5];
        unsigned char tcnt = buft[61*2 - 2] & 0xf;
        unsigned short x0 = p_point[0];
        unsigned char id0 = (x0 & 0x8000) >> 15;
        unsigned short y0;
        unsigned char id1;
        unsigned short x1;
        unsigned short y1;
        unsigned char mch;
        unsigned char act;

	x0 = x0 & 0x3fff;
	y0 = p_point[1];
	if(x0>0 && y0>0) {
		msk = 1 << id0;
	}
	x1 = p_point[2];
	id1 = (x1 & 0x8000) >> 15;
	x1 = x1 & 0x3fff;
	y1 = p_point[3];
	if(x1>0 && y1>0) {
		msk |= 1 << id1;
	}
	mch = msk ^ msk_o;
	if ((3 == mch) && (1 == tcnt)) {
		tcnt = 0;
		msk = 0;
		mch = msk_o;
		x0 = x1 = 0;
	}
	msk_o = msk;
	memset(p_data, 0xff, 18);
	
	p_data[0] = 0;
	p_data[1] = 0;

    #ifdef TLSC_TPD_PROXIMITY
        if(tpd_prox_active) {
            if(p_point[0]&0x4000) {
                p_data[1] = 0xC0;
            }
            else {
                p_data[1] = 0xE0;
            }
        }
    #endif

	p_data[2] = tcnt;
	act = 0;
	if (x0 > 0 && y0 > 0) {
		act = (0 == (mch & (0x01 << id0))) ? 0x80 : 0;
	} else {
		id0 = !id1;
		act = 0x40;
	}
	p_data[3] = (act | (x0 >> 8));
	p_data[4] = (x0 & 0xff);
	p_data[5] = (id0 << 4) | (y0 >> 8);
	p_data[6] = (y0 & 0xff);
	p_data[7] = 0x0d;
	p_data[8] = 0x10;
	
	if (x1 > 0 && y1 > 0) {
		act = (0 == (mch & (0x01 << id1))) ? 0x80 : 0;
	} else {
		id1 = !id0;
		act = 0x40;
	}
	p_data[9] = (act | (x1 >> 8));
	p_data[10] = (x1 & 0xff);
	p_data[11] = (id1 << 4) | (y1 >> 8);
	p_data[12] = (y1 & 0xff);
	p_data[13] = 0x0d;
	p_data[14] = 0x10;
    }
	return 0;
}
#endif

#ifdef TLSC_TPD_PROXIMITY

static int tlsc6x_prox_ctrl(int enable)
{
	TLSC_FUNC_ENTER();

	tpd_prox_old_state = 0xf0;	/* default is far away*/
	
	if (enable == 1) {
		tpd_prox_active = 1;
		tlsc6x_write_reg(this_client, 0xb0, 0x01);
	} else if (enable == 0) {
		tpd_prox_active = 0;
		tlsc6x_write_reg(this_client, 0xb0, 0x00);
	}
	
	return 1;
}

static ssize_t tlsc6x_prox_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "ps enable %d\n", tpd_prox_active);
}

static ssize_t tlsc6x_prox_enable_store(struct device *dev, struct device_attribute *attr, const char *buf,
					size_t count)
{
	TLSC_FUNC_ENTER();

	if('1' == buf[0]) {
	    tlsc6x_prox_ctrl(1);
	}else  if ('0' == buf[0]){
	    tlsc6x_prox_ctrl(0);
	}

	return count;
}

static DEVICE_ATTR(proximity, 0664, tlsc6x_prox_enable_show, tlsc6x_prox_enable_store);

/* default cmd interface(refer to sensor HAL):"/sys/class/sprd-tpd/device/proximity" */
static void tlsc6x_prox_cmd_path_init(void)
{
	sprd_tpd_class = class_create(THIS_MODULE, "sprd-tpd");
	if (IS_ERR(sprd_tpd_class)) {
	} else {
		sprd_ps_cmd_dev = device_create(sprd_tpd_class, NULL, 0, NULL, "device");
		if (IS_ERR(sprd_ps_cmd_dev)) {
			dev_err(&this_client->dev, "tlsc6x error::create ges&ps cmd-io fail\n");
		} else {
			/* sys/class/sprd-tpd/device/proximity */
			if (device_create_file(sprd_ps_cmd_dev, &dev_attr_proximity) < 0) {
			}
		}
	}

}
#endif

static int tlsc6x_update_data(void)
{
	struct tlsc6x_data *data = i2c_get_clientdata(this_client);
	struct ts_event *event = &data->event;
	u8 buf[20] = { 0 };
	int ret = -1;
	int i;
	u16 x, y;
	u8 tlsc_pressure, tlsc_size;

#ifdef TLSC_TPD_PROXIMITY
	u8 state;    
#endif

#ifdef TP_GESTRUE
    if(gesture_enable && gesture_state){
#if 1 
	wake_lock_timeout(&gesture_timeout_wakelock, msecs_to_jiffies(2000));
#endif 
        tlsc6x_read_Gestruedata();
        return 0 ;
    }
#endif

    #ifdef TLSC_APK_DEBUG
        if(send_data_flag) {
	    tssc_get_debug_info(this_client,buf);
        }
        else {
            ret = tlsc6x_i2c_read(this_client, buf, 1, buf, 18);
            if (ret < 0) {
            	tlsc_err("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
            	return ret;
            }
        }
    #else
	ret = tlsc6x_i2c_read(this_client, buf, 1, buf, 18);
	if (ret < 0) {
		tlsc_err("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}
    #endif

	memset(event, 0, sizeof(struct ts_event));
	event->touch_point = buf[2] & 0x07;

#ifdef TLSC_TPD_PROXIMITY
	if (tpd_prox_active && (event->touch_point == 0)) {
		tlsc6x_read_reg(this_client, 0xb0, &state);
		if(0x01 != state){
			tlsc6x_prox_ctrl(1);
		}
		if (((buf[1] == 0xc0) || (buf[1] == 0xe0)) && (tpd_prox_old_state != buf[1])) {
			input_report_abs(g_tp_drvdata->ps_input_dev, ABS_DISTANCE, (buf[1] == 0xc0) ? 0 : 1);
			input_mt_sync(g_tp_drvdata->ps_input_dev);
			input_sync(g_tp_drvdata->ps_input_dev);
			tlsc_info("tpd-proximity code:%x\n",buf[1]);
		}
		tpd_prox_old_state = buf[1];
	}
#endif

	for (i = 0; i < TS_MAX_FINGER; i++) {
		if ((buf[6 * i + 3] & 0xc0) == 0xc0) {
			continue;
		}
		x = (s16) (buf[6 * i + 3] & 0x0F) << 8 | (s16) buf[6 * i + 4];
		y = (s16) (buf[6 * i + 5] & 0x0F) << 8 | (s16) buf[6 * i + 6];
		tlsc_pressure = buf[6 * i + 7];
		if (tlsc_pressure > 127) {
			tlsc_pressure = 127;
		}
		tlsc_size = (buf[6 * i + 8] >> 4) & 0x0F;
		if ((buf[6 * i + 3] & 0x40) == 0x0) {
#if MULTI_PROTOCOL_TYPE_B
			input_mt_slot(data->input_dev, buf[6 * i + 5] >> 4);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);
#else
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, buf[6 * i + 5] >> 4);
#endif
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, y);
//			input_report_abs(data->input_dev, ABS_MT_PRESSURE, 15);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, tlsc_size);
			input_report_key(data->input_dev, BTN_TOUCH, 1);
#if !MULTI_PROTOCOL_TYPE_B
			input_mt_sync(data->input_dev);
#endif
		} else {
#if MULTI_PROTOCOL_TYPE_B
			input_mt_slot(data->input_dev, buf[6 * i + 5] >> 4);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
#endif
		}
	}
	if (event->touch_point == 0) {
		tlsc6x_clear_report_data(data);
	}
	input_sync(data->input_dev);

	return 0;

}

#ifdef TP_GESTRUE
static int check_gesture(int gesture_id)
{	
    int keycode = 0;
    struct tlsc6x_data *data = i2c_get_clientdata(this_client);

	pr_notice("[tlsc]%s: ges id:%d\n", __func__, gesture_id);
    switch(gesture_id){
        case GESTURE_LEFT:
            keycode = KEY_LEFT;
            break;
        case GESTURE_RIGHT:
            keycode = KEY_RIGHT;
            break;
        case GESTURE_UP:
            keycode = KEY_UP;
            break;
        case GESTURE_DOWN:
            keycode = KEY_DOWN;
            break;
        case GESTURE_DOUBLECLICK:
            keycode = KEY_U;    //KEY_POWER;//
            break;
        case GESTURE_O:
            keycode = KEY_O;
            break;
        case GESTURE_W:
            keycode = KEY_W;
            break;
        case GESTURE_M:
            keycode = KEY_M;
            break;
        case GESTURE_E:
            keycode = KEY_E;
            break;
        case GESTURE_C:
            keycode = KEY_C;
            break;
        case GESTURE_S:
            keycode = KEY_S;
            break;
         case GESTURE_V:
            keycode = KEY_V;
            break;
        case GESTURE_Z:
            keycode = KEY_UP;
            break;
        case GESTURE_L:
            keycode = KEY_L;
            break;
        default:
            break;
    }
    if(keycode){
        input_report_key(data->input_dev, keycode, 1);
        input_sync(data->input_dev);
        input_report_key(data->input_dev, keycode, 0);
        input_sync(data->input_dev);
    }
    return keycode;
}

static int tlsc6x_read_Gestruedata(void)
{
    int ret = -1;
    int gestrue_id = 0;
    u8 buf[4] = {0xd3, 0xd3};
    
    ret = tlsc6x_i2c_read(this_client, buf, 1, buf, 2);
    if(ret < 0){
        pr_err("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
        return ret;
    }

	pr_notice("[tlsc]%s: ges(buf[0]:%d,buf[1]:%d) Enter\n", __func__, buf[0], buf[1]);

    if(buf[1] != 0){
        gestrue_id = 0x24;
    }else{
        gestrue_id = buf[0];
    }
    check_gesture(gestrue_id);;
    return 0;
}
#endif

static irqreturn_t touch_event_thread_handler(int irq, void *devid)
{

	tlsc6x_update_data();
	
	return IRQ_HANDLED;
}

void tlsc6x_tpd_reset_force(void)
{
	struct tlsc6x_platform_data *pdata = g_tp_drvdata->platform_data;

	TLSC_FUNC_ENTER();
	gpio_direction_output(pdata->reset_gpio_number, 1);
	usleep_range(10000, 11000);
	gpio_set_value(pdata->reset_gpio_number, 0);
	msleep(20);
	gpio_set_value(pdata->reset_gpio_number, 1);
	msleep(50);
}

static void tlsc6x_tpd_reset(void)
{
	TLSC_FUNC_ENTER();
	if (g_tp_drvdata->needKeepRamCode) {
		return;
	}

	tlsc6x_tpd_reset_force();
}

static unsigned char real_suspend_flag = 0;

#if defined(CONFIG_ADF) || defined(CONFIG_HAS_EARLYSUSPEND)
static int tlsc6x_do_suspend(void)
{
	int ret = -1;

	TLSC_FUNC_ENTER();

#ifdef TLSC_ESD_HELPER_EN
        hrtimer_cancel(&tpd_esd_kthread_timer_status);
        hrtimer_cancel(&tpd_esd_kthread_timer);
#endif

#ifdef TLSC_TPD_PROXIMITY
	if (tpd_prox_active) {
		real_suspend_flag = 0;
		return 0;
	}
#endif

#ifdef TP_GESTRUE
    if(gesture_enable == 1){
        tlsc6x_irq_disable();
        gesture_state = 0x01;
        enable_irq_wake(this_client->irq);
        ret = tlsc6x_write_reg(this_client, 0xd0, 0x01);
        tlsc6x_irq_enable();
        real_suspend_flag =1;
        return 0;
    }
#endif

	tlsc6x_irq_disable();
	ret = tlsc6x_write_reg(this_client, 0xa5, 0x03);
	if (ret < 0) {
		tlsc_err("tlsc6x error::setup suspend fail!\n");
	}
	real_suspend_flag = 1;
	tlsc6x_clear_report_data(g_tp_drvdata);
	return 0;
}

static int tlsc6x_do_resume(void)
{
	TLSC_FUNC_ENTER();
#ifdef TLSC_ESD_HELPER_EN
        hrtimer_start(&tpd_esd_kthread_timer, ktime_set(3, 0), HRTIMER_MODE_REL);
        hrtimer_start(&tpd_esd_kthread_timer_status, ktime_set(2, 0), HRTIMER_MODE_REL);
#endif

	queue_work(g_tp_drvdata->tp_resume_workqueue, &g_tp_drvdata->resume_work);
	return 0;
}
#endif

#if defined(CONFIG_ADF)
static int tlsc6x_adf_suspend(void)
{
	TLSC_FUNC_ENTER();

	return tlsc6x_do_suspend();
}

static int tlsc6x_adf_resume(void)
{
	TLSC_FUNC_ENTER();

	return tlsc6x_do_resume();
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void tlsc6x_ts_suspend(struct early_suspend *handler)
{
	TLSC_FUNC_ENTER();

	tlsc6x_do_suspend();
}

static void tlsc6x_ts_resume(struct early_suspend *handler)
{
	TLSC_FUNC_ENTER();
	
	tlsc6x_do_resume();
}
#endif

static void tlsc6x_resume_work(struct work_struct *work)
{
	TLSC_FUNC_ENTER();
      
#ifdef TLSC_TPD_PROXIMITY
	tlsc6x_prox_ctrl(tpd_prox_active);
	if (tpd_prox_active && (real_suspend_flag == 0)) {
               tlsc6x_write_reg(this_client,0xa0,0xa0);
               msleep(1); 
		return;
	}
#endif

#ifdef TP_GESTRUE
    if(gesture_enable == 1){
        disable_irq_wake(this_client->irq);
        tlsc6x_irq_disable();
        gesture_state = 0;
        tlsc6x_write_reg(this_client,0xD0,0x00);
    }
#endif

	tlsc6x_tpd_reset();
	
#ifdef TLSC_APK_DEBUG    
    if(send_data_flag) { 
        msleep(30);       
        get_data_start(get_data_buf);
    }
#endif    

	if (g_tp_drvdata->needKeepRamCode) {	/* need wakeup cmd in this mode */
		tlsc6x_write_reg(this_client, 0xa5, 0x00);
	}

	tlsc6x_clear_report_data(g_tp_drvdata);

	tlsc6x_irq_enable();

	real_suspend_flag = 0;
}

#if defined(CONFIG_ADF)
/*
 * touchscreen's suspend and resume state should rely on screen state,
 * as fb_notifier and early_suspend are all disabled on our platform,
 * we can only use adf_event now
 */
static int ts_adf_event_handler(struct notifier_block *nb, unsigned long action, void *data)
{

	struct adf_notifier_event *event = data;
	int adf_event_data;

	if (action != ADF_EVENT_BLANK) {
		return NOTIFY_DONE;
	}
	adf_event_data = *(int *)event->data;
	tlsc_info("receive adf event with adf_event_data=%d", adf_event_data);

	switch (adf_event_data) {
	case DRM_MODE_DPMS_ON:
		tlsc6x_adf_resume();
		break;
	case DRM_MODE_DPMS_OFF:
		tlsc6x_adf_suspend();
		break;
	default:
		tlsc_info("receive adf event with error data, adf_event_data=%d", adf_event_data);
		break;
	}

	return NOTIFY_OK;
}
#endif

static int tlsc6x_hw_init(struct tlsc6x_data *drvdata)
{
	struct tlsc6x_platform_data *pdata = drvdata->platform_data;

	TLSC_FUNC_ENTER();
	if (gpio_request(pdata->irq_gpio_number, NULL) < 0) {
		goto OUT;
	}
	if (gpio_request(pdata->reset_gpio_number, NULL) < 0) {
		goto OUT;
	}
	gpio_direction_output(pdata->reset_gpio_number, 1);
	gpio_direction_input(pdata->irq_gpio_number);
	tlsc6x_tpd_reset();
	return 0;
OUT:
	return -EPERM;
}

#ifdef CONFIG_OF
static struct tlsc6x_platform_data *tlsc6x_parse_dt(struct device *dev)
{
	int ret;
	struct tlsc6x_platform_data *pdata;
	struct device_node *np = dev->of_node;

	TLSC_FUNC_ENTER();
	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "Could not allocate struct tlsc6x_platform_data");
		return NULL;
	}

    pdata->reset_gpio_number = of_get_named_gpio_flags(np, "tlsc6x,reset-gpio", 0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio_number < 0) {
		dev_err(dev, "fail to get reset_gpio_number\n");
		goto fail;
	}

	pdata->irq_gpio_number = of_get_named_gpio_flags(np, "tlsc6x,irq-gpio", 0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio_number < 0) {
		dev_err(dev, "fail to get reset_gpio_number\n");
		goto fail;
	}

	ret=of_property_read_u32(np, "tpd-firmware-update", &pdata->tpd_firmware_update);
	printk("tpd-firmware-update:%d\n",pdata->tpd_firmware_update);
    if (ret)dev_err(dev,"tpd-firmware-update undefined!");
	
	ret=of_property_read_u32(np, "have-virtualkey", &pdata->have_virtualkey);
	if (ret)dev_err(dev,"have-virtualkey undefined!");
	if(pdata->have_virtualkey){
		ret = of_property_read_u32_array(np, "virtualkeys", pdata->virtualkeys, 12);
		if (ret) {
			dev_err(dev, "fail to get virtualkeys\n");
		}
	}

	ret = of_property_read_u32(np, "TP_MAX_X", &pdata->x_res_max);
	if (ret) {
		dev_err(dev, "fail to get TP_MAX_X\n");
		goto fail;
	}

	ret = of_property_read_u32(np, "TP_MAX_Y", &pdata->y_res_max);
	if (ret) {
		dev_err(dev, "fail to get TP_MAX_Y\n");
		goto fail;
	}

	return pdata;
fail:
	kfree(pdata);
	return NULL;
}
#endif

#ifdef TP_GESTRUE
static ssize_t tlsc6x_gesture_write(struct file *filp, const char __user * buff, size_t len, loff_t * off)
{
    s32 ret = 0;
    unsigned char temp;

    ret = copy_from_user(&temp, buff, 1);
    if (ret) {
        return -EPERM;
    }
    gesture_enable = temp == '1'?1:0;
	
    return len;
}

static const struct file_operations gesture_fops = {
    .owner = THIS_MODULE,
    .write = tlsc6x_gesture_write,
};
#endif

/* file interface - write*/
int tlsc6x_fif_write(char *fname, u8 *pdata, u16 len)
{
	int ret = 0;
	loff_t pos = 0;
	static struct file *pfile = NULL;
	mm_segment_t old_fs = KERNEL_DS;

	pfile = filp_open(fname, O_TRUNC | O_CREAT | O_RDWR, 0644);
	if (IS_ERR(pfile)) {
		ret = -EFAULT;
		tlsc_err("tlsc6x tlsc6x_fif_write:open error!\n");
	} else {
		tlsc_info("tlsc6x tlsc6x_fif_write:start write!\n");
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		ret = (int)vfs_write(pfile, (__force const char __user *)pdata, (size_t)len, &pos);
		vfs_fsync(pfile, 0);
		filp_close(pfile, NULL);
		set_fs(old_fs);
	}
	return ret;
}
#if (defined TPD_AUTO_UPGRADE_PATH) || (defined TLSC_APK_DEBUG)
extern int tlsx6x_update_running_cfg(u16 *ptcfg);
extern int tlsx6x_update_burn_cfg(u16 *ptcfg);
extern int tlsc6x_load_ext_binlib(u8 *pcode, u16 len);
extern int tlsc6x_update_f_combboot(u8 *pdata, u16 len);
int auto_upd_busy = 0;
/* 0:success */
/* 1: no file OR open fail */
/* 2: wrong file size OR read error */
/* -1:op-fial */
int tlsc6x_proc_cfg_update(u8 *dir, int behave)
{
	int ret = 1;
	u8 *pbt_buf = NULL;
	u32 fileSize;
	mm_segment_t old_fs;
	static struct file *file = NULL;

	TLSC_FUNC_ENTER();
	tlsc_info("tlsc6x proc-file:%s\n", dir);

	file = filp_open(dir, O_RDONLY, 0);
	if (IS_ERR(file)) {
		tlsc_err("tlsc6x proc-file:open error!\n");
	} else {
		ret = 2;
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		fileSize = file->f_op->llseek(file, 0, SEEK_END);
		tlsc_info("tlsc6x proc-file, size:%d\n", fileSize);
		pbt_buf = kmalloc(fileSize, GFP_KERNEL);

		file->f_op->llseek(file, 0, SEEK_SET);
		if (fileSize == vfs_read(file, (char *)pbt_buf, fileSize, &file->f_pos)) {
			tlsc_info("tlsc6x proc-file, read ok1!\n");
			ret = 3;
		}

		if (ret == 3) {
			auto_upd_busy = 1;
			tlsc6x_irq_disable();
			msleep(1000);
			wake_lock_timeout(&tlsc6x_wakelock, msecs_to_jiffies(2000));
			if (behave == 0) {
				if (fileSize == 204) {
					ret = tlsx6x_update_running_cfg((u16 *) pbt_buf);
				} else if (fileSize > 0x400) {
					tlsc6x_load_ext_binlib((u8 *) pbt_buf, (u16) fileSize);
				}
			} else if (behave == 1) {
				if (fileSize == 204) {
					ret = tlsx6x_update_burn_cfg((u16 *) pbt_buf);
				} else if (fileSize > 0x400) {
					ret = tlsc6x_update_f_combboot((u8 *) pbt_buf, (u16) fileSize);
				}
				tlsc6x_tpd_reset();
			}
			tlsc6x_irq_enable();
			auto_upd_busy = 0;
		}

		filp_close(file, NULL);
		set_fs(old_fs);

		kfree(pbt_buf);
	}

	return ret;
}

#endif

#ifdef TLSC_APK_DEBUG
unsigned char proc_out_len;
unsigned char proc_out_buf[256];

unsigned char debug_type;
unsigned char iic_reg[2];
unsigned char sync_flag_addr[3];
unsigned char sync_buf_addr[2];
unsigned char reg_len;

static struct proc_dir_entry *tlsc6x_proc_entry = NULL;

static int debug_read(char *writebuf, int writelen, char *readbuf, int readlen)
{
	int ret = 0;

	mutex_lock(&i2c_rw_access);
	tlsc6x_set_dd_mode_sub();

	ret = tlsc6x_i2c_read_sub(this_client, writebuf, writelen, readbuf, readlen);

	tlsc6x_set_nor_mode_sub();
	mutex_unlock(&i2c_rw_access);
	if (ret > 0) {
		ret = readlen;
	}
	return ret;
}

static int debug_write(char *writebuf, int writelen)
{
	int ret = 0;

	mutex_lock(&i2c_rw_access);
	tlsc6x_set_dd_mode_sub();

	ret = tlsc6x_i2c_write_sub(this_client, writebuf, writelen);

	tlsc6x_set_nor_mode_sub();
	mutex_unlock(&i2c_rw_access);
	if (ret > 0) {
		ret = writelen;
	}
	return ret;
}

static int debug_read_sync(char *writebuf, int writelen, char *readbuf, int readlen)
{
	int ret = 0;
	int retryTime;

	mutex_lock(&i2c_rw_access);
	tlsc6x_set_dd_mode_sub();
	sync_flag_addr[2] = 1;
	ret = tlsc6x_i2c_write_sub(this_client, sync_flag_addr, 3);

	retryTime = 100;
	do {
		ret = tlsc6x_i2c_read_sub(this_client, sync_flag_addr, 2, &sync_flag_addr[2], 1);
		if (ret < 0) {
			mutex_unlock(&i2c_rw_access);
			return ret;
		}
		retryTime--;
	} while (retryTime>0&&sync_flag_addr[2] == 1);
	if(retryTime==0&&sync_flag_addr[2] == 1) {
		mutex_unlock(&i2c_rw_access);
		return -EFAULT;
	}
	if (ret >= 0) {
		/* read data */
		ret = tlsc6x_i2c_read_sub(this_client, sync_buf_addr, 2, readbuf, readlen);
	}

	tlsc6x_set_nor_mode_sub();
	mutex_unlock(&i2c_rw_access);
	if (ret > 0) {
		ret = readlen;
	}
	return ret;
}
#ifdef TLSC_TPD_PROXIMITY
static int tlsc6x_prox_ctrl(int enable);
#endif
extern int tlsc6x_load_ext_binlib(u8 *pdata, u16 len);
static int tlsc6x_rawdata_test_3535allch(u8 * buf,int len)
{
	int ret;
	int retryTime;
	u8 writebuf[4];
	buf[len] = '\0';
	ret=0;
	tlsc6x_irq_disable();
	g_tp_drvdata->esdHelperFreeze=1;
	tlsc6x_tpd_reset();
	printk("jiejianadd:tlsc6x_rawdata_test_3535allch\n");
	if (tlsc6x_load_ext_binlib((u8 *) &buf[2], len-2)){	
	ret = -EIO;
	}
	msleep(30);

	mutex_lock(&i2c_rw_access);
	//write addr
	writebuf[0]= 0x9F;
	writebuf[1]= 0x20;
	writebuf[2]= 48;
	writebuf[3]= 0xFF;
	ret = tlsc6x_i2c_write_sub(this_client, writebuf, 4);
	writebuf[0]= 0x9F;
	writebuf[1]= 0x24;
	writebuf[2]= 1;
	
	ret = tlsc6x_i2c_write_sub(this_client, writebuf, 3);
	retryTime = 100;
	do {
		ret = tlsc6x_i2c_read_sub(this_client, writebuf, 2, &writebuf[2], 1);
		if (ret < 0) {
			break;
		}
		retryTime--;
		msleep(30);
	} while (retryTime>0&&writebuf[2] == 1);

	if (ret>=0) {
		writebuf[0]= 0x9F;
		writebuf[1]= 0x26;
		ret = tlsc6x_i2c_read_sub(this_client, writebuf, 2, proc_out_buf, 96);
		if (ret>=0){
			proc_out_len=96;
		}
	}

	mutex_unlock(&i2c_rw_access);

	tlsc6x_tpd_reset();
	
	g_tp_drvdata->esdHelperFreeze=0;
	tlsc6x_irq_enable();
	return ret;
}
static ssize_t tlsc6x_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int ret;
	int buflen = len;
	unsigned char *local_buf;
	if (buflen > 4100) {
		return -EFAULT;
	}
	local_buf = kmalloc(buflen+1, GFP_KERNEL);
	if(local_buf == NULL) {
		tlsc_err("%s,Can not malloc the buf!\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(local_buf, buff, buflen)) {
		tlsc_err("%s:copy from user error\n", __func__);
		return -EFAULT;
	}
	ret = 0;
	debug_type = local_buf[0];
	/* format:cmd+para+data0+data1+data2... */
	switch (local_buf[0]) {
	case 0:		/* cfg version */
		proc_out_len = 4;
		proc_out_buf[0] = g_tlsc6x_cfg_ver;
		proc_out_buf[1] = g_tlsc6x_cfg_ver >> 8;
		proc_out_buf[2] = g_tlsc6x_cfg_ver >> 16;
		proc_out_buf[3] = g_tlsc6x_cfg_ver >> 24;
		break;
	case 1:
		local_buf[buflen] = '\0';
		if (tlsc6x_proc_cfg_update(&local_buf[2], 0)<0) {
			len = -EIO;
		}
		break;
	case 2:
		local_buf[buflen] = '\0';
		if (tlsc6x_proc_cfg_update(&local_buf[2], 1)<0) {
			len = -EIO;
		}
		break;
	case 3:
		ret = debug_write(&local_buf[1], len - 1);
		break;
	case 4:		/* read */
		reg_len = local_buf[1];
		iic_reg[0] = local_buf[2];
		iic_reg[1] = local_buf[3];
		break;
	case 5:		/* read with sync */
		ret = debug_write(&local_buf[1], 4);	/* write size */
		if (ret >= 0) {
			ret = debug_write(&local_buf[5], 4);	/* write addr */
		}
		sync_flag_addr[0] = local_buf[9];
		sync_flag_addr[1] = local_buf[10];
		sync_buf_addr[0] = local_buf[11];
		sync_buf_addr[1] = local_buf[12];
		break;
	case 8: // Force reset ic
		tlsc6x_tpd_reset_force();
		break;
	case 9: // Force reset ic
		ret=tlsc6x_rawdata_test_3535allch(local_buf,buflen);
		break;
	case 14:	/* e, esd control */
		g_tp_drvdata->esdHelperFreeze = (int)local_buf[1];
		break;
    case 15:	
                memset(get_data_buf, 0x00, (sizeof(char) * 8));
                memcpy(get_data_buf, local_buf, (sizeof(char) * 8));
                get_data_start(get_data_buf);

                send_data_flag = 1;
                send_count = 0;
                get_data_flag = 0;        
                break;
    case 16:
                get_data_stop();   
                break;		
    default:
		break;
	}
	if (ret < 0) {
		len = ret;
	}
	kfree(local_buf);
	return len;
}

static ssize_t tlsc6x_proc_read(struct file *filp, char __user *page, size_t len, loff_t *pos)
{
	int ret = 0;
	if (*pos!=0) {
		return 0;
	}
	switch (debug_type) {
	case 0:		/* version information */
		proc_out_len = 4;
		proc_out_buf[0] = g_tlsc6x_cfg_ver;
		proc_out_buf[1] = g_tlsc6x_cfg_ver >> 8;
		proc_out_buf[2] = g_tlsc6x_cfg_ver >> 16;
		proc_out_buf[3] = g_tlsc6x_cfg_ver >> 24;
		if (copy_to_user(page, proc_out_buf, proc_out_len)) {
			ret = -EFAULT;
		} else {
			ret = proc_out_len;
		}
		break;
	case 1:
		break;
	case 2:
		break;
	case 3:
		break;
	case 4:
		len = debug_read(iic_reg, reg_len, proc_out_buf, len);
		if (len > 0) {
			if (copy_to_user(page, proc_out_buf, len)) {
				ret = -EFAULT;
			} else {
				ret = len;
			}
		} else {
			ret = len;
		}
		break;
	case 5:
		len = debug_read_sync(iic_reg, reg_len, proc_out_buf, len);
		if (len > 0) {
			if (copy_to_user(page, proc_out_buf, len)) {
				ret = -EFAULT;
			} else {
				ret = len;
			}
		} else {
			ret = len;
		}
		break;
	case 9:
		if (proc_out_buf>0){
			if (copy_to_user(page, proc_out_buf, len)) {
				ret = -EFAULT;
			} else {
				ret = proc_out_len;
			}
		}
		break;
    case 15:
		if (1 == get_data_flag){  
			get_data_flag = 0;  
			if (copy_to_user(page, buf_out, len)) {
				ret = -EFAULT;   
			} else { 
				ret = len;   
			} 
		}
		else{
			ret = -EFAULT;  
		}  

		break;
	default:
		break;
	}
	if(ret>0) {
		*pos +=ret;
	}

	return ret;
}

static struct file_operations tlsc6x_proc_ops = {
	.owner = THIS_MODULE,
	.read = tlsc6x_proc_read,
	.write = tlsc6x_proc_write,
};

void tlsc6x_release_apk_debug_channel(void)
{
	if (tlsc6x_proc_entry) {
		remove_proc_entry("tlsc6x-debug", NULL);
	}
}

int tlsc6x_create_apk_debug_channel(struct i2c_client *client)
{
	tlsc6x_proc_entry = proc_create("tlsc6x-debug", 0777, NULL, &tlsc6x_proc_ops);

	if (tlsc6x_proc_entry == NULL) {
		dev_err(&client->dev, "Couldn't create proc entry!\n");
		return -ENOMEM;
	}
	dev_info(&client->dev, "Create proc entry success!\n");

	return 0;
}
#endif

static ssize_t show_tlsc_version(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	u8 reg[2];
	u8 readBuf[4];
	char *ptr = buf;
	u8 vender_id;
	mutex_lock(&i2c_rw_access);
	tlsc6x_set_dd_mode_sub();

	reg[0] = 0x80;
	reg[1] = 0x04;
	tlsc6x_i2c_read_sub(this_client, reg, 2, readBuf, 2);
	ptr += sprintf(ptr,"The boot version is %04X.\n",(readBuf[0]+(readBuf[1]<<8)));

	if (g_mccode == 0) {
		reg[0] = 0xD6;
		reg[1] = 0xE0;
	} else {
		reg[0] = 0x9E;
		reg[1] = 0x00;
	}
	tlsc6x_i2c_read_sub(this_client, reg, 2, readBuf, 4);
	ptr += sprintf(ptr,"The config version is %d.\n", readBuf[3]>>2);

	vender_id = readBuf[1]>>1;
	ptr += sprintf(ptr,"The vender id is %d, the vender name is ", vender_id);

	switch (vender_id) {
	case 1:
		ptr += sprintf(ptr,"xufang");
		break;
	case 2:
		ptr += sprintf(ptr,"xuri");
		break;
	case 3:
		ptr += sprintf(ptr,"yuye");
		break;
	case 26:
		ptr += sprintf(ptr,"hz");
		break;
	case 29:
		ptr += sprintf(ptr,"dawosi");
		break;
	default:
		ptr += sprintf(ptr,"unknown");
		break;
	}
	ptr += sprintf(ptr,".\n");

	ptr += sprintf(ptr,"The display version is 0x%04X.\n", g_tlsc6x_cfg_ver);

	tlsc6x_set_nor_mode_sub();
	mutex_unlock(&i2c_rw_access);


	return (ptr-buf);
}

static ssize_t store_tlsc_version(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	
	return -EPERM;
}

static ssize_t show_tlsc_info(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	char *ptr = buf;
	ptr += sprintf(ptr,"Max finger number is %0d.\n",TS_MAX_FINGER);
	ptr += sprintf(ptr,"Int irq is %d.\n",this_client->irq);
	ptr += sprintf(ptr,"I2c address is 0x%02X(0x%02X).\n",this_client->addr,(this_client->addr)<<1);

	return (ptr-buf);
}

static ssize_t store_tlsc_info(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	
	return -EPERM;
}

static ssize_t show_tlsc_ps(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	char *ptr = buf;
#ifdef TLSC_TPD_PROXIMITY

	ptr += sprintf(ptr,"%d\n",tpd_prox_active);
#else
	ptr += sprintf(ptr,"No proximity function.\n");
#endif

	return (ptr-buf);
}

static ssize_t store_tlsc_ps(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TLSC_TPD_PROXIMITY
	if (buf[0] == '0') {
		tlsc6x_prox_ctrl(0);
		//ps off
		
	} else if (buf[0] == '1') {
		//ps on
		tlsc6x_prox_ctrl(1);
	}
#endif
	return count;
}

static ssize_t show_tlsc_esd(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	char *ptr = buf;
#ifdef TLSC_ESD_HELPER_EN

	ptr += sprintf(ptr,"%d\n",g_tp_drvdata->esdHelperFreeze);
#else
	ptr += sprintf(ptr,"No esd function.\n");
#endif

	return (ptr-buf);
}

static ssize_t store_tlsc_esd(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TLSC_ESD_HELPER_EN
	if (buf[0] == '0') {
		g_tp_drvdata->esdHelperFreeze = 0;
		//esd on
		
	} else if (buf[0] == '1') {
		//esd off
		g_tp_drvdata->esdHelperFreeze = 1;
	}
#endif
	return count;
}

static ssize_t show_tlsc_reset(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	return -EPERM;
}

static ssize_t store_tlsc_reset(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
    if (buf[0] == '1') {
    	tlsc6x_tpd_reset_force();
    }
        
    return count;
}

static ssize_t show_tlsc_lcm_test(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	return -EPERM;
}

static ssize_t store_tlsc_lcm_test(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
    int ret = 0;
    
    if (buf[0] == '1') {
        g_tp_drvdata->esdHelperFreeze = 1;
        ret = tlsc6x_write_reg(this_client, 0xa5, 0x03);
        if (ret < 0) {
            tlsc_err("tlsc6x error::setup lcm_test fail!\n");
	}
    } else if (buf[0] == '0') {
        g_tp_drvdata->esdHelperFreeze = 0;
        ret = tlsc6x_write_reg(this_client, 0xa5, 0x00);
    }
        
    return count;
}


static ssize_t show_tlsc_gesture(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	char *ptr = buf;
#ifdef TP_GESTRUE

	ptr += sprintf(ptr,"%d\n",gesture_enable);
#else
	ptr += sprintf(ptr,"No gesture function.\n");
#endif

	return (ptr-buf);
}

static ssize_t store_tlsc_gesture(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef TP_GESTRUE
	if (buf[0] == '0') {
		//gesture off
		gesture_enable = 0;
		
	} else if (buf[0] == '1') {
		//gesture on
		gesture_enable = 1;
	}
#endif
	return count;
}
u8 readFlashbuf[204];

u8 cfgStatic[] = {
0x07,0x3A,0x0E,0x10,0x03,0x33,0x32,0x53,0x37,0x03,0x02,0x1F,0x75,0xA1,0x00,0x00,
0x00,0x00,0x55,0x55,0x00,0x00,0x00,0x1B,0x26,0x00,0x24,0x1A,0x23,0x19,0x22,0x18,
0x00,0x00,0x21,0x17,0x20,0x14,0x10,0x07,0x11,0x08,0x12,0x09,0x16,0x1F,0x00,0x00,
0x15,0x1E,0x0D,0x04,0x0E,0x05,0x0F,0x06,0x1D,0x13,0x1C,0x0A,0x00,0x00,0x01,0x0B,
0x02,0x0C,0x25,0x00,0x03,0x00,0x48,0x9A,0x6E,0x02,0x45,0x00,0xD0,0x02,0xA0,0x05,
0x50,0x00,0xF0,0x00,0x90,0x01,0x00,0x02,0x84,0x03,0x0B,0x0B,0x96,0x96,0xB4,0xB4,
0x64,0x40,0x08,0x84,0x06,0x1E,0x28,0x28,0x28,0x28,0x5C,0x06,0x97,0x07,0x3D,0x3D,
0x21,0x21,0x67,0xAD,0x1B,0x26,0x01,0x39,0x49,0x15,0x86,0xAF,0x9A,0x64,0x91,0x01,
0xC8,0x5C,0x24,0x25,0x25,0x11,0x2D,0x3A,0x73,0x32,0x9F,0x4D,0xAA,0x28,0xEB,0x5F,
0x5A,0x0D,0x82,0xA2,0x14,0x06,0x14,0x08,0x00,0x06,0xEB,0x07,0xAD,0x05,0x6E,0x07,
0x29,0x06,0x67,0x08,0x01,0x01,0x39,0x31,0x39,0x31,0x00,0x00,0x00,0x00,0xD5,0x02,
0x0E,0x20,0x0B,0x0B,0x28,0x24,0x50,0x03,0x29,0x3C,0x1E,0x14,0x14,0x11,0x11,0x15,
0x16,0x17,0x0B,0x14,0x14,0x11,0x11,0x14,0x14,0x14,0x35,0xC0
};

static ssize_t show_tlsc_debug_flash(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	char *ptr = buf;
	int i;

	for(i=0;i<204;i++) {
		ptr += sprintf(ptr,"%d,",readFlashbuf[i]);	
	}

	ptr += sprintf(ptr,"\n");

	return (ptr-buf);
}
extern int tlsc6x_download_ramcode(u8 *pcode, u16 len);
extern  int tlsc6x_write_burn_space(u8 *psrc, u16 adr, u16 len);
extern  int tlsc6x_read_burn_space(u8 *pdes, u16 adr, u16 len);
extern unsigned char fw_fcode_burn[2024];
extern int tlsc6x_set_nor_mode(void);
int writeFlash(u8* buf ,u16 addr,int len)
{
	auto_upd_busy = 1;
	tlsc6x_irq_disable();
	if (tlsc6x_download_ramcode(fw_fcode_burn, sizeof(fw_fcode_burn))) {
		tlsc_err("Tlsc6x:write flash error:ram-code error!\n");
		return -EPERM;
	}

	if (tlsc6x_write_burn_space((unsigned char *)buf, addr, len)) {
		tlsc_err("Tlsc6x:write flash  fail!\n");
		return -EPERM;
	}
	tlsc6x_set_nor_mode();

	tlsc6x_tpd_reset();

	auto_upd_busy=0;

	tlsc6x_irq_enable();
	return 0;
}

int readFlash(u16 addr,int len)
{
	auto_upd_busy = 1;
	tlsc6x_irq_disable();
	if (tlsc6x_download_ramcode(fw_fcode_burn, sizeof(fw_fcode_burn))) {
		tlsc_err("Tlsc6x:write flash error:ram-code error!\n");
		return -EPERM;
	}

	if (tlsc6x_read_burn_space((unsigned char *)readFlashbuf, addr, len)) {
		tlsc_err("Tlsc6x:write flash  fail!\n");
		return -EPERM;
	}
	tlsc6x_set_nor_mode();

	tlsc6x_tpd_reset();

	auto_upd_busy=0;

	tlsc6x_irq_enable();
	return 0;
}

static ssize_t store_tlsc_debug_flash(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	u8 wbuf[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	if (buf[0] == '0') {
		//gesture off
		
		
	} else if (buf[0] == '1') {
		//gesture on
		writeFlash(cfgStatic ,0xF000,204);
	} else if (buf[0] == '2') {
		//gesture on
		writeFlash(wbuf ,0,16);
	} else if (buf[0] == '3') {
		//gesture on
		writeFlash(wbuf ,0x8000,16);
	} else if (buf[0] == '4') {
		//gesture on
		readFlash(0xF000,204);
	} else if (buf[0] == '5') {
		//gesture on
		readFlash(0,204);
	} else if (buf[0] == '6') {
		//gesture on
		readFlash(0x8000,204);
	}

	return count;
}

static DEVICE_ATTR(tlsc_flash_ctl, 0664, show_tlsc_debug_flash, store_tlsc_debug_flash);
static DEVICE_ATTR(tlsc_version, 0664, show_tlsc_version, store_tlsc_version);
static DEVICE_ATTR(tlsc_tp_info, 0664, show_tlsc_info, store_tlsc_info);
static DEVICE_ATTR(tlsc_ps_ctl, 0664, show_tlsc_ps, store_tlsc_ps);
static DEVICE_ATTR(tlsc_esd_ctl, 0664, show_tlsc_esd, store_tlsc_esd);
static DEVICE_ATTR(tlsc_reset_ctl, 0664, show_tlsc_reset, store_tlsc_reset);
static DEVICE_ATTR(tlsc_lcm_ctl, 0664, show_tlsc_lcm_test, store_tlsc_lcm_test);
static DEVICE_ATTR(tlsc_gs_ctl, 0664, show_tlsc_gesture, store_tlsc_gesture);
static struct attribute *tlsc_attrs[] = {
	&dev_attr_tlsc_version.attr,
	&dev_attr_tlsc_tp_info.attr,
	&dev_attr_tlsc_ps_ctl.attr,
	&dev_attr_tlsc_esd_ctl.attr,
	&dev_attr_tlsc_reset_ctl.attr,
	&dev_attr_tlsc_lcm_ctl.attr,
	&dev_attr_tlsc_gs_ctl.attr,
	&dev_attr_tlsc_flash_ctl.attr,
	NULL, // Can not delete this line!!! The phone will reset.
};
static struct attribute_group tlsc_attr_group = {
	.attrs = tlsc_attrs,
};

#ifdef TLSC_ESD_HELPER_EN
unsigned char g_tlsc6x_esdtar = 0x36;
unsigned char g_tlsc6x_esdactive = 0;

static int esd_check_work(void)
{
	int ret = -1;
	u8 test_val = 0;

	/*TLSC_FUNC_ENTER();*/

       g_tlsc6x_esdactive = 0;
	
	if (g_tp_drvdata->esdHelperFreeze) {
               g_tlsc6x_esdactive = 1;
		return 1;	
	}
        
	ret = tlsc6x_read_reg(this_client, 0xa3, &test_val);

	if (ret < 0) {		/* maybe confused by some noise,so retry is make sense. */
		msleep(10);
		ret = tlsc6x_read_reg(this_client, 0xa3, &test_val);
	}
	
	if (ret >= 0) {
		if (g_tlsc6x_esdtar != test_val) {
			ret = -1;
		}
	}
	if (ret < 0) {		
		tlsc6x_tpd_reset_force();

		tlsc6x_clear_report_data(g_tp_drvdata);	

               tlsc_err("read 0xa3 fail \n");
		
		#ifdef TLSC_TPD_PROXIMITY
		    tlsc6x_prox_ctrl(tpd_prox_active);
		#endif	
	}
	#ifdef TLSC_TPD_PROXIMITY
	if(tpd_prox_active){
		/* ps-function enabled, but tp lost this cmd  */
		if(tlsc6x_read_reg(this_client, 0xb0, &test_val) >= 0){
			if(test_val != 0x01){
				tlsc6x_prox_ctrl(tpd_prox_active);
			}
		}
	}
	#endif	
        
        g_tlsc6x_esdactive = 1;
        
	return ret;
}

static int esd_checker_handler(void *unused)
{
	ktime_t ktime;	
	
	do {
		wait_event_interruptible(tpd_esd_waiter, tpd_esd_flag != 0);
		tpd_esd_flag = 0;

		ktime = ktime_set(3, 0);
		hrtimer_start(&tpd_esd_kthread_timer, ktime, HRTIMER_MODE_REL);

		if (g_tp_drvdata->esdHelperFreeze) {
			continue;
		}
#if (defined TPD_AUTO_UPGRADE_PATH) || (defined TLSC_APK_DEBUG)
		if (auto_upd_busy) {
			continue;
		}
#endif
		esd_check_work();

	} while (!kthread_should_stop());

	return 0;
}

enum hrtimer_restart tpd_esd_kthread_hrtimer_func(struct hrtimer *timer)
{
	tpd_esd_flag = 1;
	wake_up_interruptible(&tpd_esd_waiter);

	return HRTIMER_NORESTART;
}

static int esd_checker_handler_status(void *unused)
{
       static int esd_check_loop ;

	ktime_t ktime;	

       esd_check_loop = 0;
	
	do {
		wait_event_interruptible(tpd_esd_waiter_status, tpd_esd_flag_status != 0);
		tpd_esd_flag_status = 0;

		ktime = ktime_set(2, 0);
		hrtimer_start(&tpd_esd_kthread_timer_status, ktime, HRTIMER_MODE_REL);

		if (g_tp_drvdata->esdHelperFreeze) {
			continue;
		}
#if (defined TPD_AUTO_UPGRADE_PATH) || (defined TLSC_APK_DEBUG)
		if (auto_upd_busy) {
			continue;
		}
#endif               
               if(1 != g_tlsc6x_esdactive) {
                    esd_check_loop++;
                }else {
                    esd_check_loop = 0;
                }

                if (0 == g_tp_drvdata->esdHelperFreeze) {
                    if(esd_check_loop > 1) {
                        tlsc6x_tpd_reset();
                        tlsc_err("esd_check_loop > 1 \n");
                      #ifdef TLSC_TPD_PROXIMITY
                        	tlsc6x_prox_ctrl(tpd_prox_active);
                     #endif
                        esd_check_loop = 0;
                    }
                }

	} while (!kthread_should_stop());

	return 0;
}

enum hrtimer_restart tpd_esd_kthread_hrtimer_func_status(struct hrtimer *timer)
{
	tpd_esd_flag_status = 1;
	wake_up_interruptible(&tpd_esd_waiter_status);

	return HRTIMER_NORESTART;
}

#endif


static int tlsc6x_request_irq_work(void)
{
	int ret = 0;

	this_client->irq = gpio_to_irq(g_tp_drvdata->platform_data->irq_gpio_number);

	tlsc_info("The irq node num is %d", this_client->irq);
	
	ret = request_threaded_irq(this_client->irq,
				   NULL, touch_event_thread_handler,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND,
				   "tlsc6x_tpd_irq", NULL);
	if (ret < 0) {
		tlsc_err("Request irq thread error!");
		return  ret;
	}

	return ret;
}

#define SHOW_TLSC_TPINFO
#ifdef SHOW_TLSC_TPINFO
#include <asm/uaccess.h>
#include <linux/proc_fs.h>  /*proc*/
static ssize_t tlsc_tp_info(struct file *file, char __user *buffer, size_t count, loff_t *ppos)
{
	char *page = NULL;
	char *ptr = NULL;
	int err = -1;
	int len = 0;
	extern unsigned int g_tlsc6x_boot_ver;
	extern int tlsc_cfg_version;
	extern int tlsc_vendor_id;

	page = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!page) 
	{
		kfree(page);
		return -ENOMEM;
	}
	ptr = page;
	switch(tlsc_vendor_id){
		case 27:{
			ptr += sprintf(page, "tlsc6x_HXD_%03d_%03x\n ", tlsc_cfg_version, g_tlsc6x_boot_ver);
		}break;
		case 3: {
			ptr += sprintf(page, "tlsc6x_YY_%03d_%03x\n ", tlsc_cfg_version, g_tlsc6x_boot_ver);
		}break;
		default: {
			ptr += sprintf(page, "tlsc6x_XXX_%03d_%03x\n ", tlsc_cfg_version, g_tlsc6x_boot_ver);
		}break;
	}
	
	len = ptr - page;
	if(*ppos >= len)
	{
		kfree(page);
		return 0;
	}
	err = copy_to_user(buffer,(char *)page,len);
	*ppos += len;
	if(err)
	{
		kfree(page);
		return err;
	}
	kfree(page);
	return len;
}

static const struct file_operations tp_proc_info_fops = {
	.read = tlsc_tp_info,
};
#endif

static int tlsc6x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	int reset_count;
	struct input_dev *input_dev;
	struct tlsc6x_platform_data *pdata = NULL;
#ifdef TP_GESTRUE
	struct proc_dir_entry *proc_entry = NULL;
#endif

	TLSC_FUNC_ENTER();

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_alloc_platform_data_failed;
	}
#ifdef CONFIG_OF		/* NOTE:THIS IS MUST!!! */
	if (client->dev.of_node) {
		pdata = tlsc6x_parse_dt(&client->dev);
		if (pdata) {
			client->dev.platform_data = pdata;
		}
	}
#endif

	if (pdata == NULL) {
		err = -ENOMEM;
		tlsc_err("%s: no platform data!!!\n", __func__);
		goto exit_alloc_platform_data_failed;
	}

	g_tp_drvdata = kzalloc(sizeof(*g_tp_drvdata), GFP_KERNEL);	/* auto clear */
	if (!g_tp_drvdata) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	this_client = client;
	g_tp_drvdata->client = client;
	g_tp_drvdata->platform_data = pdata;

	err = tlsc6x_hw_init(g_tp_drvdata);
	if (err < 0) {
		goto exit_gpio_request_failed;
	}

	i2c_set_clientdata(client, g_tp_drvdata);

	/* #ifdef CONFIG_I2C_SPRD */
	/* sprd_i2c_ctl_chg_clk(client->adapter->nr, 400000); */
	/* #endif */
	reset_count = 0;
	g_is_telink_comp = 0;
	while (++reset_count <= 3) {
		tlsc6x_tpd_reset();
		g_is_telink_comp = tlsc6x_tp_dect(client);
		if (g_is_telink_comp) {
			break;
		}
	}

	g_tp_drvdata->needKeepRamCode = g_needKeepRamCode;

	if (g_is_telink_comp) {
		tlsc6x_tpd_reset();
#ifdef SHOW_TLSC_TPINFO
		proc_create("sprocomm_tpInfo", 0444, NULL, &tp_proc_info_fops);
#endif
	} else {
		tlsc_err("tlsc6x:%s, no tlsc6x!\n", __func__);
		err = -ENODEV;
		goto exit_chip_check_failed;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "tlsc6x error::failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	g_tp_drvdata->input_dev = input_dev;

	__set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	__set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
	__set_bit(KEY_MENU, input_dev->keybit);
	__set_bit(KEY_BACK, input_dev->keybit);
	__set_bit(KEY_HOMEPAGE, input_dev->keybit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
    
#ifdef TP_GESTRUE
	input_set_capability(input_dev, EV_KEY, KEY_POWER);
	input_set_capability(input_dev, EV_KEY, KEY_LEFT);
	input_set_capability(input_dev, EV_KEY, KEY_RIGHT);
	input_set_capability(input_dev, EV_KEY, KEY_UP);
	input_set_capability(input_dev, EV_KEY, KEY_DOWN);
	input_set_capability(input_dev, EV_KEY, KEY_D);
	input_set_capability(input_dev, EV_KEY, KEY_O);
	input_set_capability(input_dev, EV_KEY, KEY_W);
	input_set_capability(input_dev, EV_KEY, KEY_M);
	input_set_capability(input_dev, EV_KEY, KEY_E);
	input_set_capability(input_dev, EV_KEY, KEY_C);
	input_set_capability(input_dev, EV_KEY, KEY_S);
	input_set_capability(input_dev, EV_KEY, KEY_V);
	input_set_capability(input_dev, EV_KEY, KEY_Z);
	__set_bit(KEY_LEFT,  input_dev->keybit);
	__set_bit(KEY_RIGHT,  input_dev->keybit);
	__set_bit(KEY_UP,  input_dev->keybit);
	__set_bit(KEY_DOWN,  input_dev->keybit);
	__set_bit(KEY_D,  input_dev->keybit);
	__set_bit(KEY_O,  input_dev->keybit);
	__set_bit(KEY_W,  input_dev->keybit);
	__set_bit(KEY_M,  input_dev->keybit);
	__set_bit(KEY_E,  input_dev->keybit);
	__set_bit(KEY_C,  input_dev->keybit);
	__set_bit(KEY_S,  input_dev->keybit);
	__set_bit(KEY_V,  input_dev->keybit);
	__set_bit(KEY_Z,  input_dev->keybit);
#endif

#if MULTI_PROTOCOL_TYPE_B
	input_mt_init_slots(input_dev, TS_MAX_FINGER, 0);
#else
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);
#endif
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, pdata->x_res_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, pdata->y_res_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 15, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
//	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 127, 0, 0);
	input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);	/* give this capability aways */
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);

	input_dev->name = "tlsc6x_touch";
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev, "tlsc6x error::failed to register input device: %s\n", dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}
    
#ifdef TLSC_TPD_PROXIMITY
	tlsc6x_prox_cmd_path_init();
	g_tp_drvdata->ps_input_dev = input_allocate_device();
	if (!g_tp_drvdata->ps_input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "tlsc6x error::failed to allocate ps-input device\n");
		goto exit_input_register_device_failed;
	}
	g_tp_drvdata->ps_input_dev->name = "proximity_tp";
	set_bit(EV_ABS, g_tp_drvdata->ps_input_dev->evbit);
	input_set_capability(g_tp_drvdata->ps_input_dev, EV_ABS, ABS_DISTANCE);
	input_set_abs_params(g_tp_drvdata->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);
	err = input_register_device(g_tp_drvdata->ps_input_dev);
	if (err) {
		dev_err(&client->dev, "tlsc6x error::failed to register ps-input device: %s\n", dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}
#endif

	 if (pdata->have_virtualkey){
		printk("entry:tlsc6x_virtual_keys_init\n");
		tlsc6x_virtual_keys_init();
	}

#ifdef TP_GESTRUE
#if 1 
	wake_lock_init(&gesture_timeout_wakelock, WAKE_LOCK_SUSPEND, "gesture_timeout_wakelock");
#endif
	proc_entry = proc_create("gesture_enable", 0644, NULL, &gesture_fops);
#endif

	INIT_WORK(&g_tp_drvdata->resume_work, tlsc6x_resume_work);
	g_tp_drvdata->tp_resume_workqueue = create_singlethread_workqueue("tlsc6x_resume_work");
	if (!g_tp_drvdata->tp_resume_workqueue) {
		err = -ESRCH;
		goto exit_input_register_device_failed;
	}

	wake_lock_init(&tlsc6x_wakelock, WAKE_LOCK_SUSPEND, "tlsc6x_wakelock");

	err = tlsc6x_request_irq_work();
	if (err < 0) {
		dev_err(&client->dev, "tlsc6x error::request irq failed %d\n", err);
		goto exit_irq_request_failed;
	}
#if defined(CONFIG_ADF)
	int ret = -1;
	g_tp_drvdata->fb_notif.notifier_call = ts_adf_event_handler;
	g_tp_drvdata->fb_notif.priority = 1000;
	ret = adf_register_client(&g_tp_drvdata->fb_notif);
	if (ret) {
		dev_err(&client->dev, "tlsc6x error::unable to register fb_notifier: %d", ret);
	}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	g_tp_drvdata->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	g_tp_drvdata->early_suspend.suspend = tlsc6x_ts_suspend;
	g_tp_drvdata->early_suspend.resume = tlsc6x_ts_resume;
	register_early_suspend(&g_tp_drvdata->early_suspend);
#endif

#ifdef TLSC_APK_DEBUG
	tlsc6x_create_apk_debug_channel(client);
#endif

	err=sysfs_create_group(&client->dev.kobj, &tlsc_attr_group);
	if (err < 0) {
		tlsc_err("Can not create sysfs group!");
	}

#ifdef TLSC_ESD_HELPER_EN
	{			/* esd issue: i2c monitor thread */
		ktime_t ktime = ktime_set(30, 0);
               ktime_t ktime_status = ktime_set(30, 0);

		hrtimer_init(&tpd_esd_kthread_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		tpd_esd_kthread_timer.function = tpd_esd_kthread_hrtimer_func;
		hrtimer_start(&tpd_esd_kthread_timer, ktime, HRTIMER_MODE_REL);
		kthread_run(esd_checker_handler, 0, "tlsc6x_esd_helper");

               hrtimer_init(&tpd_esd_kthread_timer_status, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		tpd_esd_kthread_timer_status.function = tpd_esd_kthread_hrtimer_func_status;
		hrtimer_start(&tpd_esd_kthread_timer_status, ktime_status, HRTIMER_MODE_REL);
		kthread_run(esd_checker_handler_status, 0, "tlsc6x_esd_helper_status");
	}
#endif

	return 0;

exit_irq_request_failed:
	input_unregister_device(input_dev);
exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
exit_chip_check_failed:
	gpio_free(pdata->irq_gpio_number);
	gpio_free(pdata->reset_gpio_number);
exit_gpio_request_failed:
	kfree(g_tp_drvdata);
exit_alloc_data_failed:
	if (pdata != NULL) {
		kfree(pdata);
	}
	g_tp_drvdata = NULL;
	i2c_set_clientdata(client, g_tp_drvdata);
exit_alloc_platform_data_failed:
	return err;
}

static int tlsc6x_remove(struct i2c_client *client)
{
	struct tlsc6x_data *drvdata = i2c_get_clientdata(client);

	TLSC_FUNC_ENTER();
#ifdef TP_GESTRUE
#if 1 // (LINUX_VERSION_CODE < KERNEL_VERSION(4,14,0))
	wake_lock_destroy(&gesture_timeout_wakelock);
#endif
#endif

#ifdef TLSC_APK_DEBUG
	tlsc6x_release_apk_debug_channel();
#endif

#ifdef TLSC_ESD_HELPER_EN
	hrtimer_cancel(&tpd_esd_kthread_timer);
        hrtimer_cancel(&tpd_esd_kthread_timer_status);
#endif

	if (drvdata == NULL) {
		return 0;
	}
#if defined(CONFIG_ADF)
	adf_unregister_client(&g_tp_drvdata->fb_notif);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&drvdata->early_suspend);
#endif

	free_irq(client->irq, drvdata);
	input_unregister_device(drvdata->input_dev);
	input_free_device(drvdata->input_dev);

#ifdef CONFIG_HAS_EARLYSUSPEND
	cancel_work_sync(&drvdata->resume_work);
	destroy_workqueue(drvdata->tp_resume_workqueue);
#endif
	kfree(drvdata);
	drvdata = NULL;
	i2c_set_clientdata(client, drvdata);

	return 0;
}

static const struct i2c_device_id tlsc6x_id[] = {
	{TS_NAME, 0}, {}
};

MODULE_DEVICE_TABLE(i2c, tlsc6x_id);

static const struct of_device_id tlsc6x_of_match[] = {
	{.compatible = "tlsc6x,tlsc6x_ts",},
	{}
};

MODULE_DEVICE_TABLE(of, tlsc6x_of_match);
static struct i2c_driver tlsc6x_driver = {
	.probe = tlsc6x_probe,
	.remove = tlsc6x_remove,
	.id_table = tlsc6x_id,
	.driver = {
		   .name = TS_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = tlsc6x_of_match,
	},
};

static int __init tlsc6x_init(void)
{
	tlsc_info("%s: ++\n", __func__);
	return i2c_add_driver(&tlsc6x_driver);
}

static void __exit tlsc6x_exit(void)
{
	i2c_del_driver(&tlsc6x_driver);
}

module_init(tlsc6x_init);
module_exit(tlsc6x_exit);

MODULE_DESCRIPTION("Chipsemi touchscreen driver");
MODULE_LICENSE("GPL");
