/*
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
 * VERSION			DATE			AUTHOR
 *
 */

#ifndef __tlsc6x_main_h__
#define __tlsc6x_main_h__

#if defined(CONFIG_ADF)
#include <linux/notifier.h>
#include <video/adf_notifier.h>
#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif

#define TLSC_TPD_PROXIMITY 
//#define TLSC_MUL_VENDOR  /*是否兼容多家屏厂与多款TP*/

#define TLSC_APK_DEBUG		/* apk debugger, close:undef */
//#define TLSC_AUTO_UPGRADE         //cfg auto update 
#define TLSC_ESD_HELPER_EN	/* esd helper, close:undef */
/* #defineTLSC_FORCE_UPGRADE */
#define TP_GESTRUE
/* #define TLSC_TP_PROC_SELF_TEST */
// #define TLSC_BUILDIN_BOOT     // firware update 

/*********************************************************/

#if 1
#define tlsc_info(x...) pr_notice("[tlsc] " x)
#define tlsc_err(x...) pr_err("[tlsc][error] " x)
#define TLSC_FUNC_ENTER() pr_notice("[tlsc]%s: Enter\n", __func__)
#else
#define tlsc_info(x...)
#define tlsc_err(x...)
#define TLSC_FUNC_ENTER()
#endif

struct tlsc6x_platform_data {
	u32 irq_gpio_flags;
	u32 reset_gpio_flags;
	u32 irq_gpio_number;
	u32 reset_gpio_number;
	u32 tpd_firmware_update;
	u32 have_virtualkey;
	u32 virtualkeys[12];
	u32 x_res_max;
	u32 y_res_max;
};

struct ts_event {
	u16 x1;
	u16 y1;
	u16 x2;
	u16 y2;
	u16 x3;
	u16 y3;
	u16 x4;
	u16 y4;
	u16 x5;
	u16 y5;
	u16 pressure;
	u8 touch_point;
};

struct tlsc6x_data {
	struct input_dev *input_dev;
	struct input_dev *ps_input_dev;
	struct i2c_client *client;
	struct ts_event event;
#if defined(CONFIG_ADF)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	struct work_struct resume_work;
	struct workqueue_struct *tp_resume_workqueue;
	int irq_gpio_number;
	int reset_gpio_number;
	int isVddAlone;
	int needKeepRamCode;
	int esdHelperFreeze;
	int irq_disabled;
	struct regulator *reg_vdd;
	struct tlsc6x_platform_data *platform_data;
};

struct tlsc6x_stest_crtra {
    unsigned short xch_n;
    unsigned short ych_n;
    unsigned short allch_n;
    unsigned short st_nor_os_L1;
    unsigned short st_nor_os_L2;
    unsigned short st_nor_os_bar;
    unsigned short st_nor_os_key;
    unsigned short m_os_nor_std;
    unsigned short ffset;
    unsigned short fsset;
    unsigned short  fsbse_max;
    unsigned short  fsbse_bar;
    unsigned char remap[48];
    unsigned short rawmax[48];
    unsigned short rawmin[48];
};

struct tlsc6x_updfile_header {
	u32 sig;
	u32 resv;
	u32 n_cfg;
	u32 n_match;
	u32 len_cfg;
	u32 len_boot;
};
extern struct tlsc6x_data *g_tp_drvdata;
extern struct mutex i2c_rw_access;

extern unsigned int g_tlsc6x_cfg_ver;
extern unsigned int g_tlsc6x_boot_ver;
extern unsigned short g_tlsc6x_chip_code;
extern unsigned int g_needKeepRamCode;
extern struct tlsc6x_data *g_tp_drvdata;

extern int tlsc6x_tp_dect(struct i2c_client *client);
extern int tlsc6x_auto_upgrade_buidin(void);
extern int tlsc6x_load_gesture_binlib(void);
extern void tlsc6x_data_crash_deal(void);

extern int tlsx6x_update_burn_cfg(u16 *ptcfg);
extern int tlsx6x_update_running_cfg(u16 *ptcfg);
extern int tlsc6x_set_dd_mode_sub(void);
extern int tlsc6x_set_nor_mode_sub(void);
extern int tlsc6x_set_dd_mode(void);
extern int tlsc6x_set_nor_mode(void);

extern int tlsc6x_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue);
extern int tlsc6x_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue);
extern int tlsc6x_i2c_read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen);
extern int tlsc6x_i2c_write(struct i2c_client *client, char *writebuf, int writelen);
extern int tlsc6x_i2c_read_sub(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen);
extern int tlsc6x_i2c_write_sub(struct i2c_client *client, char *writebuf, int writelen);

#if (defined TPD_AUTO_UPGRADE_PATH) || (defined TLSC_APK_DEBUG)
extern int tlsc6x_proc_cfg_update(u8 *dir, int behave);
#endif
extern void tlsc6x_tpd_reset_force(void);
extern int tlsc6x_fif_write(char *fname, u8 *pdata, u16 len);
#endif
