/* drivers/sharp/shterm/shterm_kobject.c
 *
 * Copyright (C) 2010 Sharp Corporation
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

#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <sharp/shterm_k.h>

#define EVENT_NAME_MAX 24

typedef struct {
    const char *name;
    struct kobject *kobj;
    struct kobj_type *ktype;
} shterm_data;

typedef struct {
    unsigned long int id;
    const char *name;
} shterm_id_tbl;

#define CREATE_ATTR( idname )                                        \
    static struct kobj_attribute shterm_info_##idname##_attribute =  \
         __ATTR( idname, 0666, NULL, NULL );                         \

#define CREATE_ATTR_ROOT( idname )                                   \
    static struct kobj_attribute shterm_info_##idname##_attribute =  \
         __ATTR( idname, 0600, NULL, NULL );                         \

static struct kobject shterm_kobj;
static struct kset *shterm_kset;
static struct semaphore shterm_sem;
static struct semaphore shterm_flip_sem;

static int shterm_info[SHTERM_MAX];
static int shterm_info_read[SHTERM_MAX];
static int shterm_flip_status = SHTERM_FLIP_STATE_CLOSE;
static unsigned long int flip_counter = 0;

static int onex_min = 4;
static int onex_max = -1;
static int onex_old = -1;
static int evdo_min = 4;
static int evdo_max = -1;
static int evdo_old = -1;

static void shterm_release( struct kobject *kobj );
static ssize_t shterm_info_show( struct kobject *kobj, struct attribute *attr, char *buff );
static ssize_t shterm_info_store( struct kobject *kobj, struct attribute *attr, const char *buff, size_t len );

static shterm_id_tbl id_k_tbl[SHTERM_MAX] =
    {{ SHTERM_INFO_SPEAKER, "SHTERM_INFO_SPEAKER" },
     { SHTERM_INFO_VIB, "SHTERM_INFO_VIB" },
     { SHTERM_INFO_CAMERA, "SHTERM_INFO_CAMERA" },
     { SHTERM_INFO_LINE, "SHTERM_INFO_LINE" },
     { SHTERM_INFO_QTV, "SHTERM_INFO_QTV" },
     { SHTERM_INFO_DTB, "SHTERM_INFO_DTB" },
     { SHTERM_INFO_LCDPOW, "SHTERM_INFO_LCDPOW" },
     { SHTERM_INFO_BACKLIGHT, "SHTERM_INFO_BACKLIGHT" },
     { SHTERM_INFO_BLUETOOTH, "SHTERM_INFO_BLUETOOTH" },
     { SHTERM_INFO_MOBILE_LIGHT, "SHTERM_INFO_MOBILE_LIGHT" },
     { SHTERM_INFO_MUSIC, "SHTERM_INFO_MUSIC" },
     { SHTERM_INFO_LINE_RINGING, "SHTERM_INFO_LINE_RINGING" },
     { SHTERM_INFO_FM_TX, "SHTERM_INFO_FM_TX" },
     { SHTERM_INFO_WLAN_TXRX, "SHTERM_INFO_WLAN_TXRX" },
     { SHTERM_INFO_SPEAKER_LEV, "SHTERM_INFO_SPEAKER_LEV" },
     { SHTERM_INFO_BACKLIGHT_LEV, "SHTERM_INFO_BACKLIGHT_LEV" },
     { SHTERM_INFO_IR, "SHTERM_INFO_IR" },
     { SHTERM_INFO_SD, "SHTERM_INFO_SD" },
     { SHTERM_INFO_GBNAND, "SHTERM_INFO_GBNAND" },
     { SHTERM_INFO_USB, "SHTERM_INFO_USB" },
     { SHTERM_INFO_WIFI, "SHTERM_INFO_WIFI" },
     { SHTERM_INFO_GPS, "SHTERM_INFO_GPS" },
     { SHTERM_INFO_ACCELE, "SHTERM_INFO_ACCELE" },
     { SHTERM_INFO_COMPS, "SHTERM_INFO_COMPS" },
     { SHTERM_INFO_KEYBACKLIGHT, "SHTERM_INFO_KEYBACKLIGHT" },
     { SHTERM_INFO_CTSBACKLIGHT, "SHTERM_INFO_CTSBACKLIGHT" },
     { SHTERM_INFO_SUBCAMERA, "SHTERM_INFO_SUBCAMERA" },
     { SHTERM_INFO_GEIGER, "SHTERM_INFO_GEIGER" }};

CREATE_ATTR( SHTERM_INFO_SPEAKER );
CREATE_ATTR( SHTERM_INFO_VIB );
CREATE_ATTR( SHTERM_INFO_CAMERA );
CREATE_ATTR( SHTERM_INFO_LINE );
CREATE_ATTR( SHTERM_INFO_QTV );
CREATE_ATTR( SHTERM_INFO_DTB );
CREATE_ATTR( SHTERM_INFO_LCDPOW );
CREATE_ATTR( SHTERM_INFO_BACKLIGHT );
CREATE_ATTR( SHTERM_INFO_BLUETOOTH );
CREATE_ATTR( SHTERM_INFO_MOBILE_LIGHT );
CREATE_ATTR( SHTERM_INFO_MUSIC );
CREATE_ATTR( SHTERM_INFO_LINE_RINGING );
CREATE_ATTR( SHTERM_INFO_FM_TX );
CREATE_ATTR( SHTERM_INFO_WLAN_TXRX );
CREATE_ATTR( SHTERM_INFO_SPEAKER_LEV );
CREATE_ATTR( SHTERM_INFO_BACKLIGHT_LEV );
CREATE_ATTR( SHTERM_INFO_IR );
CREATE_ATTR( SHTERM_INFO_SD );
CREATE_ATTR( SHTERM_INFO_GBNAND );
CREATE_ATTR( SHTERM_INFO_USB );
CREATE_ATTR( SHTERM_INFO_WIFI );
CREATE_ATTR( SHTERM_INFO_GPS );
CREATE_ATTR( SHTERM_INFO_ACCELE );
CREATE_ATTR( SHTERM_FLIP_STATE );
CREATE_ATTR( SHTERM_FLIP_COUNT );
CREATE_ATTR( SHTERM_INFO_COMPS );
CREATE_ATTR( ONEX );
CREATE_ATTR( EVDO );
CREATE_ATTR( MIN_ONEX );
CREATE_ATTR( MAX_ONEX );
CREATE_ATTR( MIN_EVDO );
CREATE_ATTR( MAX_EVDO );
CREATE_ATTR( SHTERM_INFO_KEYBACKLIGHT );
CREATE_ATTR( SHTERM_INFO_CTSBACKLIGHT );
CREATE_ATTR( SHTERM_INFO_SUBCAMERA );
CREATE_ATTR( SHTERM_INFO_GEIGER );

static struct attribute *shterm_default_attrs[] = {
    &shterm_info_SHTERM_INFO_SPEAKER_attribute.attr,
    &shterm_info_SHTERM_INFO_VIB_attribute.attr,
    &shterm_info_SHTERM_INFO_CAMERA_attribute.attr,
    &shterm_info_SHTERM_INFO_LINE_attribute.attr,
    &shterm_info_SHTERM_INFO_QTV_attribute.attr,
    &shterm_info_SHTERM_INFO_DTB_attribute.attr,
    &shterm_info_SHTERM_INFO_LCDPOW_attribute.attr,
    &shterm_info_SHTERM_INFO_BACKLIGHT_attribute.attr,
    &shterm_info_SHTERM_INFO_BLUETOOTH_attribute.attr,
    &shterm_info_SHTERM_INFO_MOBILE_LIGHT_attribute.attr,
    &shterm_info_SHTERM_INFO_MUSIC_attribute.attr,
    &shterm_info_SHTERM_INFO_LINE_RINGING_attribute.attr,
    &shterm_info_SHTERM_INFO_FM_TX_attribute.attr,
    &shterm_info_SHTERM_INFO_WLAN_TXRX_attribute.attr,
    &shterm_info_SHTERM_INFO_SPEAKER_LEV_attribute.attr,
    &shterm_info_SHTERM_INFO_BACKLIGHT_LEV_attribute.attr,
    &shterm_info_SHTERM_INFO_IR_attribute.attr,
    &shterm_info_SHTERM_INFO_SD_attribute.attr,
    &shterm_info_SHTERM_INFO_GBNAND_attribute.attr,
    &shterm_info_SHTERM_INFO_USB_attribute.attr,
    &shterm_info_SHTERM_INFO_WIFI_attribute.attr,
    &shterm_info_SHTERM_INFO_GPS_attribute.attr,
    &shterm_info_SHTERM_INFO_ACCELE_attribute.attr,
    &shterm_info_SHTERM_INFO_COMPS_attribute.attr,
    &shterm_info_SHTERM_FLIP_STATE_attribute.attr,
    &shterm_info_SHTERM_FLIP_COUNT_attribute.attr,
    &shterm_info_ONEX_attribute.attr,
    &shterm_info_EVDO_attribute.attr,
    &shterm_info_MIN_ONEX_attribute.attr,
    &shterm_info_MAX_ONEX_attribute.attr,
    &shterm_info_MIN_EVDO_attribute.attr,
    &shterm_info_MAX_EVDO_attribute.attr,
    &shterm_info_SHTERM_INFO_KEYBACKLIGHT_attribute.attr,
    &shterm_info_SHTERM_INFO_CTSBACKLIGHT_attribute.attr,
    &shterm_info_SHTERM_INFO_SUBCAMERA_attribute.attr,
    &shterm_info_SHTERM_INFO_GEIGER_attribute.attr,
    NULL,
};

static struct sysfs_ops shterm_sysfs_ops = {
    .show  = shterm_info_show,
    .store = shterm_info_store,
};

static struct kobj_type shterm_ktype = {
    .release = shterm_release,
    .sysfs_ops = &shterm_sysfs_ops,
    .default_attrs = shterm_default_attrs,
};

static shterm_data data = {
    .name  = "info",
    .kobj  = &shterm_kobj,
    .ktype = &shterm_ktype,
};

int shterm_k_set_info( unsigned long int shterm_info_id, unsigned long int shterm_info_value )
{
    if( shterm_info_id < 0 || shterm_info_id >= SHTERM_MAX ){
        return SHTERM_FAILURE;
    }

    if( down_interruptible(&shterm_sem) ){
        printk( "%s down_interruptible for write failed\n", __FUNCTION__ );
        return SHTERM_FAILURE;
    }

    shterm_info[shterm_info_id] = shterm_info_value;
    if( shterm_info_value ){
        shterm_info_read[shterm_info_id] = shterm_info_value;
    }

    up( &shterm_sem );

    return SHTERM_SUCCESS;
}

int shterm_k_set_event( shbattlog_info_t *info )
{
    char *envp[16];
    char e_num[EVENT_NAME_MAX];
    char e_chg_vol[EVENT_NAME_MAX];
    char e_chg_cur[EVENT_NAME_MAX];
    char e_latest_cur[EVENT_NAME_MAX];
    char e_bat_vol[EVENT_NAME_MAX];
    char e_bat_tmp[EVENT_NAME_MAX];
    char e_chg_tmp[EVENT_NAME_MAX];
    char e_cam_tmp[EVENT_NAME_MAX];
    char e_pmic_tmp[EVENT_NAME_MAX];
    char e_pa_tmp[EVENT_NAME_MAX];
    char e_avg_cur[EVENT_NAME_MAX];
    char e_avg_vol[EVENT_NAME_MAX];
    char e_acc_cur[EVENT_NAME_MAX];
    char e_vol_per[EVENT_NAME_MAX];
#ifdef CONFIG_SH_DETECT_HIGH_TEMP
    char e_tmp_cut[EVENT_NAME_MAX];
#endif /* CONFIG_SH_DETECT_HIGH_TEMP */
    char e_mem_free[EVENT_NAME_MAX];
    int idx = 0;
    int ret;

    memset( e_num, 0x00, sizeof(e_num) );
    snprintf( e_num, EVENT_NAME_MAX - 1, "EVENT_NUM=%d", info->event_num );
    envp[idx++] = e_num;

    switch( info->event_num ){
    case SHBATTLOG_EVENT_BATT_REPORT_NORM:
    case SHBATTLOG_EVENT_BATT_REPORT_CHG:
        memset( e_chg_vol, 0x00, sizeof(e_chg_vol) );
        snprintf( e_chg_vol, EVENT_NAME_MAX - 1, "CHG_VOL=%d", info->chg_vol );
        envp[idx++] = e_chg_vol;

        memset( e_chg_cur, 0x00, sizeof(e_chg_cur) );
        snprintf( e_chg_cur, EVENT_NAME_MAX - 1, "CHG_CUR=%d", info->chg_cur );
        envp[idx++] = e_chg_cur;

        memset( e_latest_cur, 0x00, sizeof(e_latest_cur) );
        snprintf( e_latest_cur, EVENT_NAME_MAX - 1, "LAST_CUR=%d", info->latest_cur );
        envp[idx++] = e_latest_cur;

        memset( e_acc_cur, 0x00, sizeof(e_acc_cur) );
        snprintf( e_acc_cur, EVENT_NAME_MAX - 1, "ACC_CUR=%d", info->acc_cur );
        envp[idx++] = e_acc_cur;

        memset( e_mem_free, 0x00, sizeof(e_mem_free) );
        snprintf( e_mem_free, EVENT_NAME_MAX - 1, "MEM_FREE=%8lu", info->mem_free );
        envp[idx++] = e_mem_free;

    case SHBATTLOG_EVENT_FGIC_EX10:
    case SHBATTLOG_EVENT_FGIC_EX20:
    case SHBATTLOG_EVENT_FGIC_EX30:
    case SHBATTLOG_EVENT_FGIC_EX40:
    case SHBATTLOG_EVENT_FGIC_EX50:
    case SHBATTLOG_EVENT_FGIC_EX60:
    case SHBATTLOG_EVENT_FGIC_EX70:
    case SHBATTLOG_EVENT_FGIC_EX80:
    case SHBATTLOG_EVENT_FGIC_EX90:
    case SHBATTLOG_EVENT_FGIC_EX100:
    case SHBATTLOG_EVENT_HIGH_TEMP:
#ifdef CONFIG_SH_DETECT_HIGH_TEMP
    case SHBATTLOG_EVENT_DETECT_HIGH_TEMP:
    case SHBATTLOG_EVENT_DETECT_LOW_TEMP:
#endif /* CONFIG_SH_DETECT_HIGH_TEMP */
        memset( e_bat_vol, 0x00, sizeof(e_bat_vol) );
        snprintf( e_bat_vol, EVENT_NAME_MAX - 1, "BAT_VOL=%d", info->bat_vol );
        envp[idx++] = e_bat_vol;

        memset( e_bat_tmp, 0x00, sizeof(e_bat_tmp) );
        snprintf( e_bat_tmp, EVENT_NAME_MAX - 1, "BAT_TEMP=%d", info->bat_temp );
        envp[idx++] = e_bat_tmp;

        memset( e_chg_tmp, 0x00, sizeof(e_chg_tmp) );
        snprintf( e_chg_tmp, EVENT_NAME_MAX - 1, "CHG_TEMP=%d", info->chg_temp );
        envp[idx++] = e_chg_tmp;

        memset( e_cam_tmp, 0x00, sizeof(e_cam_tmp) );
        snprintf( e_cam_tmp, EVENT_NAME_MAX - 1, "CAM_TEMP=%d", info->cam_temp );
        envp[idx++] = e_cam_tmp;

        memset( e_pmic_tmp, 0x00, sizeof(e_pmic_tmp) );
        snprintf( e_pmic_tmp, EVENT_NAME_MAX - 1, "PMIC_TEMP=%d", info->pmic_temp );
        envp[idx++] = e_pmic_tmp;

        memset( e_pa_tmp, 0x00, sizeof(e_pa_tmp) );
        snprintf( e_pa_tmp, EVENT_NAME_MAX - 1, "PA_TEMP=%d", info->pa_temp );
        envp[idx++] = e_pa_tmp;

        memset( e_avg_cur, 0x00, sizeof(e_avg_cur) );
        snprintf( e_avg_cur, EVENT_NAME_MAX - 1, "AVG_CUR=%d", info->avg_cur );
        envp[idx++] = e_avg_cur;

        memset( e_avg_vol, 0x00, sizeof(e_avg_vol) );
        snprintf( e_avg_vol, EVENT_NAME_MAX - 1, "AVG_VOL=%d", info->avg_vol );
        envp[idx++] = e_avg_vol;

        memset( e_vol_per, 0x00, sizeof(e_vol_per) );
        snprintf( e_vol_per, EVENT_NAME_MAX - 1, "VOL_PER=%d", info->vol_per );
        envp[idx++] = e_vol_per;

#ifdef CONFIG_SH_DETECT_HIGH_TEMP
        memset( e_tmp_cut, 0x00, sizeof(e_tmp_cut) );
        snprintf( e_tmp_cut, EVENT_NAME_MAX - 1, "TMP_CUT=%d", info->tmp_cut );
        envp[idx++] = e_tmp_cut;
#endif /* CONFIG_SH_DETECT_HIGH_TEMP */
        break;

    case SHBATTLOG_EVENT_FATAL_BATT:
    case SHBATTLOG_EVENT_INDICATER_0:
    case SHBATTLOG_EVENT_CHG_IDLE_ST:
    case SHBATTLOG_EVENT_CHG_FAST_ST:
    case SHBATTLOG_EVENT_CHG_END:
    case SHBATTLOG_EVENT_BATT_ID_INVALID:
    case SHBATTLOG_EVENT_CHG_RESTART:
    case SHBATTLOG_EVENT_CHG_ERROR:
    case SHBATTLOG_EVENT_CHG_TRICKLE_ST:
    case SHBATTLOG_EVENT_CHG_MAINT_ST:
    case SHBATTLOG_EVENT_CHG_HOT_FAST_ST:
    case SHBATTLOG_EVENT_CHG_ERR_BD_BAT_UNUSUAL_ST:
    case SHBATTLOG_EVENT_CHG_ERR_BAT_ID_INVALID_ST:
    case SHBATTLOG_EVENT_FGIC_NOMAL:
    case SHBATTLOG_EVENT_FGIC_INVALID:
    case SHBATTLOG_EVENT_CHG_HOT_STOP_ST:
    case SHBATTLOG_EVENT_CHG_COLD_STOP_ST:
    case SHBATTLOG_EVENT_CHG_MAINT_STOP_ST:
    case SHBATTLOG_EVENT_CHG_MAINT_HOT_STOP_ST:
    case SHBATTLOG_EVENT_CHG_MAINT_COLD_STOP_ST:
        memset( e_bat_vol, 0x00, sizeof(e_bat_vol) );
        snprintf( e_bat_vol, EVENT_NAME_MAX - 1, "BAT_VOL=%d", info->bat_vol );
        envp[idx++] = e_bat_vol;

        memset( e_bat_tmp, 0x00, sizeof(e_bat_tmp) );
        snprintf( e_bat_tmp, EVENT_NAME_MAX - 1, "BAT_TEMP=%d", info->bat_temp );
        envp[idx++] = e_bat_tmp;

        memset( e_chg_tmp, 0x00, sizeof(e_chg_tmp) );
        snprintf( e_chg_tmp, EVENT_NAME_MAX - 1, "CHG_TEMP=%d", info->chg_temp );
        envp[idx++] = e_chg_tmp;

        memset( e_cam_tmp, 0x00, sizeof(e_cam_tmp) );
        snprintf( e_cam_tmp, EVENT_NAME_MAX - 1, "CAM_TEMP=%d", info->cam_temp );
        envp[idx++] = e_cam_tmp;

        memset( e_pmic_tmp, 0x00, sizeof(e_pmic_tmp) );
        snprintf( e_pmic_tmp, EVENT_NAME_MAX - 1, "PMIC_TEMP=%d", info->pmic_temp );
        envp[idx++] = e_pmic_tmp;

        memset( e_pa_tmp, 0x00, sizeof(e_pa_tmp) );
        snprintf( e_pa_tmp, EVENT_NAME_MAX - 1, "PA_TEMP=%d", info->pa_temp );
        envp[idx++] = e_pa_tmp;

        memset( e_vol_per, 0x00, sizeof(e_vol_per) );
        snprintf( e_vol_per, EVENT_NAME_MAX - 1, "VOL_PER=%d", info->vol_per );
        envp[idx++] = e_vol_per;
        break;

    case SHBATTLOG_EVENT_CHG_INSERT_USB:
    case SHBATTLOG_EVENT_CHG_START:
    case SHBATTLOG_EVENT_CHG_REMOVE_USB:
    case SHBATTLOG_EVENT_CHG_INSERT_CHGR:
    case SHBATTLOG_EVENT_CHG_REMOVE_CHGR:
    case SHBATTLOG_EVENT_CHG_ERR_BD_CHG_UNUSUAL_ST:
    case SHBATTLOG_EVENT_CHG_ERR_CHG_POWER_SHORTAGE_ST:
    case SHBATTLOG_EVENT_CHG_COUNT_OVER_STOP_ST:
    case SHBATTLOG_EVENT_CHGR_OSCILLATION:
        memset( e_bat_vol, 0x00, sizeof(e_bat_vol) );
        snprintf( e_bat_vol, EVENT_NAME_MAX - 1, "BAT_VOL=%d", info->bat_vol );
        envp[idx++] = e_bat_vol;

        memset( e_bat_tmp, 0x00, sizeof(e_bat_tmp) );
        snprintf( e_bat_tmp, EVENT_NAME_MAX - 1, "BAT_TEMP=%d", info->bat_temp );
        envp[idx++] = e_bat_tmp;

        memset( e_chg_tmp, 0x00, sizeof(e_chg_tmp) );
        snprintf( e_chg_tmp, EVENT_NAME_MAX - 1, "CHG_TEMP=%d", info->chg_temp );
        envp[idx++] = e_chg_tmp;

        memset( e_cam_tmp, 0x00, sizeof(e_cam_tmp) );
        snprintf( e_cam_tmp, EVENT_NAME_MAX - 1, "CAM_TEMP=%d", info->cam_temp );
        envp[idx++] = e_cam_tmp;

        memset( e_pmic_tmp, 0x00, sizeof(e_pmic_tmp) );
        snprintf( e_pmic_tmp, EVENT_NAME_MAX - 1, "PMIC_TEMP=%d", info->pmic_temp );
        envp[idx++] = e_pmic_tmp;

        memset( e_pa_tmp, 0x00, sizeof(e_pa_tmp) );
        snprintf( e_pa_tmp, EVENT_NAME_MAX - 1, "PA_TEMP=%d", info->pa_temp );
        envp[idx++] = e_pa_tmp;

        memset( e_chg_vol, 0x00, sizeof(e_chg_vol) );
        snprintf( e_chg_vol, EVENT_NAME_MAX - 1, "CHG_VOL=%d", info->chg_vol );
        envp[idx++] = e_chg_vol;

        memset( e_chg_cur, 0x00, sizeof(e_chg_cur) );
        snprintf( e_chg_cur, EVENT_NAME_MAX - 1, "CHG_CUR=%d", info->chg_cur );
        envp[idx++] = e_chg_cur;

        memset( e_vol_per, 0x00, sizeof(e_vol_per) );
        snprintf( e_vol_per, EVENT_NAME_MAX - 1, "VOL_PER=%d", info->vol_per );
        envp[idx++] = e_vol_per;
        break;

    case SHBATTLOG_EVENT_OVER_CURRENT1:
    case SHBATTLOG_EVENT_OVER_CURR1_DET:
    case SHBATTLOG_EVENT_OVER_CURR1_RELEASE:
    case SHBATTLOG_EVENT_CHG_COMP:
    case SHBATTLOG_EVENT_BATT_REPORT_DETERIORATED:
    case SHBATTLOG_EVENT_CHG_HOT_ADD_FAST_ST:
        memset( e_bat_vol, 0x00, sizeof(e_bat_vol) );
        snprintf( e_bat_vol, EVENT_NAME_MAX - 1, "BAT_VOL=%d", info->bat_vol );
        envp[idx++] = e_bat_vol;

        memset( e_bat_tmp, 0x00, sizeof(e_bat_tmp) );
        snprintf( e_bat_tmp, EVENT_NAME_MAX - 1, "BAT_TEMP=%d", info->bat_temp );
        envp[idx++] = e_bat_tmp;

        memset( e_chg_tmp, 0x00, sizeof(e_chg_tmp) );
        snprintf( e_chg_tmp, EVENT_NAME_MAX - 1, "CHG_TEMP=%d", info->chg_temp );
        envp[idx++] = e_chg_tmp;

        memset( e_cam_tmp, 0x00, sizeof(e_cam_tmp) );
        snprintf( e_cam_tmp, EVENT_NAME_MAX - 1, "CAM_TEMP=%d", info->cam_temp );
        envp[idx++] = e_cam_tmp;

        memset( e_pmic_tmp, 0x00, sizeof(e_pmic_tmp) );
        snprintf( e_pmic_tmp, EVENT_NAME_MAX - 1, "PMIC_TEMP=%d", info->pmic_temp );
        envp[idx++] = e_pmic_tmp;

        memset( e_pa_tmp, 0x00, sizeof(e_pa_tmp) );
        snprintf( e_pa_tmp, EVENT_NAME_MAX - 1, "PA_TEMP=%d", info->pa_temp );
        envp[idx++] = e_pa_tmp;

        memset( e_latest_cur, 0x00, sizeof(e_latest_cur) );
        snprintf( e_latest_cur, EVENT_NAME_MAX - 1, "LAST_CUR=%d", info->latest_cur );
        envp[idx++] = e_latest_cur;

        memset( e_vol_per, 0x00, sizeof(e_vol_per) );
        snprintf( e_vol_per, EVENT_NAME_MAX - 1, "VOL_PER=%d", info->vol_per );
        envp[idx++] = e_vol_per;
        break;

    case SHBATTLOG_EVENT_SYS_REBOOT:
        memset( e_bat_tmp, 0x00, sizeof(e_bat_tmp) );
        snprintf( e_bat_tmp, EVENT_NAME_MAX - 1, "BAT_TEMP=%d", info->bat_temp );
        envp[idx++] = e_bat_tmp;

        memset( e_chg_tmp, 0x00, sizeof(e_chg_tmp) );
        snprintf( e_chg_tmp, EVENT_NAME_MAX - 1, "CHG_TEMP=%d", info->chg_temp );
        envp[idx++] = e_chg_tmp;
        break;

    default:
        break;
    }

    envp[idx++] = NULL;
    ret = kobject_uevent_env( data.kobj, KOBJ_CHANGE, envp );

    return ret;
}

int shterm_flip_status_set( int state )
{
    int ret = SHTERM_FAILURE;

    if( down_interruptible(&shterm_flip_sem) ){
        printk( "%s down_interruptible for read failed\n", __FUNCTION__ );
        return -ERESTARTSYS;
    }

    if( SHTERM_FLIP_STATE_OPEN == state ){
        if( SHTERM_FLIP_STATE_CLOSE == shterm_flip_status &&
            flip_counter < 0xffffffff ){
            flip_counter++;
        }
        ret = SHTERM_SUCCESS;
        shterm_flip_status = state;
    }
    else if( SHTERM_FLIP_STATE_CLOSE == state ){
        ret = SHTERM_SUCCESS;
        shterm_flip_status = state;
    }

    up( &shterm_flip_sem );

    return ret;
}

static void shterm_release( struct kobject *kobj )
{
    kfree( kobj );
}

static ssize_t shterm_info_show( struct kobject *kobj, struct attribute *attr, char *buff )
{
    int n, ret;

    if( !strncmp(attr->name, "MIN_ONEX", strlen("MIN_ONEX")) ){
        ret = sprintf( buff, "%d", onex_min );
        onex_min = onex_old;
    }
    else if( !strncmp(attr->name, "MAX_ONEX", strlen("MAX_ONEX")) ){
        ret = sprintf( buff, "%d", onex_max );
        onex_max = onex_old;
    }
    else if( !strncmp(attr->name, "MIN_EVDO", strlen("MIN_EVDO")) ){
        ret = sprintf( buff, "%d", evdo_min );
        evdo_min = evdo_old;
    }
    else if( !strncmp(attr->name, "MAX_EVDO", strlen("MAX_EVDO")) ){
        ret = sprintf( buff, "%d", evdo_max );
        evdo_max = evdo_old;
    }
    else if( !strncmp(attr->name, "SHTERM_FLIP_COUNT", strlen("SHTERM_FLIP_COUNT")) ){
        if( down_interruptible(&shterm_flip_sem) ){
            printk( "%s down_interruptible for read failed\n", __FUNCTION__ );
            return -ERESTARTSYS;
        }
        ret = sprintf( buff, "%lu", flip_counter );
        up( &shterm_flip_sem );
    }
    else if( !strncmp(attr->name, "SHTERM_FLIP_STATE", strlen("SHTERM_FLIP_STATE")) ){
        if( down_interruptible(&shterm_flip_sem) ){
            printk( "%s down_interruptible for read failed\n", __FUNCTION__ );
            return -ERESTARTSYS;
        }
        ret = sprintf( buff, "%d", shterm_flip_status );
        up( &shterm_flip_sem );
    }
    else {
        for( n = 0; n < SHTERM_MAX; n++ ){
            if( !strncmp(attr->name, id_k_tbl[n].name, strlen(attr->name)) ){
                break;
            }
            else if( n == SHTERM_MAX - 1 ){
                return -1;
            }
        }

        if( down_interruptible(&shterm_sem) ){
            printk( "%s down_interruptible for read failed\n", __FUNCTION__ );
            return -ERESTARTSYS;
        }

        ret = sprintf( buff, "%d", shterm_info_read[n] );
        shterm_info_read[n] = shterm_info[n];

        up( &shterm_sem );
    }

    return ret;

}

static ssize_t shterm_info_store( struct kobject *kobj, struct attribute *attr, const char *buff, size_t len )
{
    int n;
    int val;

    if( !strncmp(attr->name, "ONEX", strlen("ONEX")) ){
        val = (int)simple_strtol( buff, (char **)NULL, 10 );
        if( val < onex_min ){
            onex_min = val;
        }
        if( val > onex_max ){
            onex_max = val;
        }
        onex_old = val;
    }
    else if( !strncmp(attr->name, "EVDO", strlen("EVDO")) ){
        val = (int)simple_strtol( buff, (char **)NULL, 10 );
        if( val < evdo_min ){
            evdo_min = val;
        }
        if( val > evdo_max ){
            evdo_max = val;
        }
        evdo_old = val;
    }
    else if( !strncmp(attr->name, "SHTERM_FLIP_STATE", strlen("SHTERM_FLIP_STATE")) ){
        if( down_interruptible(&shterm_flip_sem) ){
            printk( "%s down_interruptible for read failed\n", __FUNCTION__ );
            return -ERESTARTSYS;
        }
        shterm_flip_status = simple_strtol( buff, (char **)NULL, 10 );
        up( &shterm_flip_sem );
    }
    else if( !strncmp(attr->name, "SHTERM_FLIP_COUNT", strlen("SHTERM_FLIP_COUNT")) ){
        if( down_interruptible(&shterm_flip_sem) ){
            printk( "%s down_interruptible for read failed\n", __FUNCTION__ );
            return -ERESTARTSYS;
        }
        flip_counter = simple_strtoul( buff, (char **)NULL, 10 );
        up( &shterm_flip_sem );
    }
    else {
        for( n = 0; n < SHTERM_MAX; n++ ){
            if( !strncmp(attr->name, id_k_tbl[n].name, strlen(attr->name)) ){
                break;
            }
            else if( n == SHTERM_MAX - 1 ){
                return -1;
            }
        }

        val = (int)simple_strtol( buff, (char **)NULL, 10 );
        /* errno check */
        if( down_interruptible(&shterm_sem) ){
            printk( "%s down_interruptible for write failed\n", __FUNCTION__ );
            return -ERESTARTSYS;
        }
        if( SHTERM_INFO_MUSIC == n ){
            if( val ){
                if( shterm_info[n] < 100 ){
                    shterm_info[n]++;
                    shterm_info_read[n] = val;
                }
            }
            else {
                if( shterm_info[n] > 0 ){
                    shterm_info[n]--;
                }
            }
        }
        else {
            shterm_info[n] = val;
            if( val ){
                shterm_info_read[n] = val;
            }
        }

        up( &shterm_sem );
    }

    return len;
}

static int __init shterm_kobject_init( void )
{
    int ret;

    /* Create a kset with the name of "shterm" */
    /* located under /sys/kernel/ */
    shterm_kset = kset_create_and_add( "shterm", NULL, kernel_kobj );
    if( !shterm_kset ){
        printk( "%s : line %d error\n", __FUNCTION__, __LINE__ );
        return -ENOMEM;
    }

    data.kobj->kset = shterm_kset;
    ret = kobject_init_and_add( data.kobj, data.ktype, NULL, "%s", data.name );
    if( ret ){
        printk( "%s : line %d error\n", __FUNCTION__, __LINE__ );
        kobject_put( data.kobj );
    }

    /* Allowing only 1 user r/w access to a file */
    sema_init( &shterm_sem, 1 );
    sema_init( &shterm_flip_sem, 1 );
    memset( shterm_info, 0x00, sizeof(shterm_info) );
    memset( shterm_info_read, 0x00, sizeof(shterm_info_read) );

    shterm_info[SHTERM_INFO_LCDPOW] = 1;
    shterm_info_read[SHTERM_INFO_LCDPOW] = 1;
    shterm_info[SHTERM_INFO_BACKLIGHT] = 1;
    shterm_info_read[SHTERM_INFO_BACKLIGHT] = 1;

    return ret;
}

static void __exit shterm_kobject_exit( void )
{
    kset_unregister( shterm_kset );
}

module_init(shterm_kobject_init);
module_exit(shterm_kobject_exit);
