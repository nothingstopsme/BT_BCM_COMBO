/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.


 *  Copyright (C) 2009-2014 Broadcom Corporation
 */

/************************************************************************************
 *
 *  Filename:    fmdrv.h
 *
 *  Description: Common header for all FM driver sub-modules.
 *
 ***********************************************************************************/
#ifndef _FM_DRV_H
#define _FM_DRV_H

#include <linux/skbuff.h>
#include <linux/interrupt.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <linux/timer.h>
#include <linux/version.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>

/*******************************************************************************
**  Constants & Macros
*******************************************************************************/

#define FM_DRV_VERSION            "1.01"

/* Should match with FM_DRV_VERSION */
#define FM_DRV_RADIO_VERSION      KERNEL_VERSION(1, 0, 1)
#define FM_DRV_NAME               "brcm_fmdrv"
#define FM_DRV_CARD_SHORT_NAME    "BRCM FM Radio"
#define FM_DRV_CARD_LONG_NAME     "Broadcom corporation FM Radio"

/* Flag info */
#define FM_CORE_READY                 3

#define FM_DRV_TX_TIMEOUT       (1000)  /* 1 sec */
#define FM_DRV_RX_SEEK_TIMEOUT       (20000)  /* 20 sec */

#define NO_OF_ENTRIES_IN_ARRAY(array) (sizeof(array) / sizeof(array[0]))

/* bits in register FM_RDS_SYS    0x00 */
#define FM_OFF                     0x00
#define FM_ON                      0x01
#define FM_RDS_ON                  0x02

/* bits in register I2C_FM_REG_AUD_CTL0  0x05 */
#define FM_RF_MUTE                 0x0001    /* bit0 */
#define FM_MANUAL_MUTE             0x0002    /* bit1 */
#define FM_Z_MUTE_LEFT_OFF         0x0004    /* bit2 */
#define FM_Z_MUTE_RITE_OFF         0x0008   /* bit3 */
#define FM_AUDIO_DAC_ON            0x0010    /* bit4 */
#define FM_AUDIO_I2S_ON            0x0020    /* bit5 */
#define FM_DEEMPHA_75_ON           0x0040    /* bit6 */
#define FM_AUDIO_BAND_WIDTH        0x0080    /* bit7 */

/* bits in register FM_REG_FM_CTRL  0x01 */
#define FM_BAND_REG_WEST           0x00
#define FM_BAND_REG_EAST           0x01
#define FM_STEREO_AUTO             0x02
#define FM_STEREO_MANUAL           0x04
#define FM_STEREO_SWITCH           0x08
#define FM_HI_LO_INJ               0x10

/* bits in register FM_REG_RDS_CTL0  0x02 */
#define FM_RDS_CTRL_RDS            0x00
#define FM_RDS_CTRL_RBDS           0x01
#define FM_RDS_CTRL_FIFO_FLUSH     0x02

/* the highest bit on I2C_FM_REG_PCM_ROUTE register */
#define FM_PCM_ROUTE_ON_BIT        0x80    /* FM on SCO */

#define FM_STEP_NONE               0xff

/* FM/RDS flag bits */
#define FM_RDS_FLAG_CLEAN_BIT       0x01 /* clean FM_RDS_FLAG region */
#define FM_RDS_FLAGSCH_FRZ_BIT          0x02 /* interrupt freeze */
#define FM_RDS_FLAG_SCH_BIT         0x04 /* Pending search_tune */

/* bits in I2C_FM_REG_FM_RDS_FLAG 0x12 */
#define I2C_MASK_SRH_TUNE_CMPL_BIT     (0x0001 << 0)    /* FM/RDS register search/tune cmpl bit */
#define I2C_MASK_SRH_TUNE_FAIL_BIT     (0x0001 << 1)    /* FM/RDS register search/tune fail bit */
#define I2C_MASK_RSSI_LOW_BIT          (0x0001 << 2)    /* FM/RDS registerRSSI low bit */
#define I2C_MASK_CARR_HI_ERR_BIT       (0x0001 << 3)    /* FM/RDS register carrier high err bit */
#define I2C_MASK_AUDIO_PAUSE_BIT       (0x0001 << 4)    /* audio has paused for the specific threshold and duration */
#define I2C_MASK_STEREO_DETC_BIT       (0x0001 << 5)    /* FM/RDS register search/tune cmpl bit */
#define I2C_MASK_STEREO_ACTIVE_BIT     (0x0001 << 6)    /* FM/RDS register search/tune fail bit */
/* second bytes of FM_RDS_FLG 0x13 */
#define I2C_MASK_RDS_FIFO_WLINE_BIT    (0x0100 << 1)    /* RDS tuples are currently available at
                                                        a level >= waterline */
#define I2C_MASK_BB_MATCH_BIT          (0x0100 << 3)    /* PI code match found */
#define I2C_MASK_SYNC_LOST_BIT         (0x0100 << 4)    /* RDS synchronization was lost */
#define I2C_MASK_PI_MATCH_BIT          (0x0100 << 5)    /* PI code match found */

/* FM_REG_RDS_DATA 0x80 reading */
#define FM_RDS_END_TUPLE_1ST_BYTE     0x7c /* 1st byte of a RDS ending tuple */
#define FM_RDS_END_TUPLE_2ND_BYTE    0xff /* 2nd byte of a RDS ending tuple */
#define FM_RDS_END_TUPLE_3RD_BYTE    0xff /* 3rd byte of a RDS ending tuple */

#if 0
/* FM/RDS flag bits used with fm_dev->rx.fm_rds_flag */
#define     FM_RDS_FLAG_CLEAN_BIT         0x01    /* clean FM_RDS_FLAG register */
#define     FM_RDS_FLAG_SCH_FRZ_BIT     0x02    /* interrupt freeze */
#define     FM_RDS_FLAG_SCH_BIT             0x04    /* pending search_tune */
#endif

#define     FM_RDS_UPD_TUPLE            64       /* 64 tuples per read(64*3 = 192 bytes),
                                                    one tuple contains 1 RDS block  */

#define     FM_READ_1_BYTE_DATA     1
#define     FM_READ_2_BYTE_DATA     2

#define TRUE 1
#define FALSE 0

#define WORKER_QUEUE TRUE

/* Debug/Error Messaging for V4L2 FM driver */
#if V4L2_FM_DEBUG
#define V4L2_FM_DRV_DBG(flag, fmt, arg...) \
        do { \
            if (fm_dbg_param & flag) \
                printk(KERN_DEBUG "(v4l2fmdrv):%s  "fmt"\n" , \
                                           __func__,## arg); \
        } while(0)
#else
#define V4L2_FM_DRV_DBG(flag, fmt, arg...)
#endif
#define V4L2_FM_DRV_ERR(fmt, arg...)  printk(KERN_ERR "(v4l2fmdrv):%s  "fmt"\n" , \
                                           __func__,## arg)

extern int fm_dbg_param;

enum {
    FM_MODE_OFF,
    FM_MODE_TX,
    FM_MODE_RX,
    FM_MODE_ENTRY_MAX
};

enum fm_seek_tune_state
{
    FM_STATE_NONE,
    FM_STATE_TUNING,
    FM_STATE_TUNE_CMPL,
    FM_STATE_TUNE_ERR,
    FM_STATE_SEEKING,
    FM_STATE_SEEK_CMPL,
    FM_STATE_SEEK_ERR
};

/*******************************************************************************
**  Type definitions
*******************************************************************************/

/* FM region (Europe/US, Japan) info */
struct band_info {
    unsigned short  low_bound;   /* lowest frequency boundary */
    unsigned short  high_bound;     /* highest frequency boundary */
    unsigned char   deemphasis;     /* FM de-emphasis time constant */
    unsigned char    scan_step;      /* scanning step */
    unsigned char    fm_band;
    unsigned char   rds_support;       /* RDS support */
};

/* RDS info */
struct fm_rds {
    unsigned char rds_flag; /* RX RDS on/off status */
    /* RDS buffer */
    wait_queue_head_t read_queue;
    unsigned int buf_size; /* Size is always multiple of 3 */
    unsigned int wr_index;
    unsigned int rd_index;
    unsigned char *cbuffer;

    spinlock_t cbuff_lock;        /* To protect access to RDS Circular buffer */
};

/* FM RX mode info */
struct fm_rx {
    struct band_info band_config;      /* Current selected band */
    unsigned char curr_band;
    unsigned int curr_freq;         /* Current RX frquency */
    unsigned char curr_mute_mode;   /* Current mute mode */
    unsigned short curr_volume;     /* Current volume level */
    char  curr_snr_threshold;       /* Current SNR threshold level */
    unsigned char curr_rssi_threshold;  /* Current RSSI threshold value */
    unsigned char curr_sch_mode;    /* current search mode */
    unsigned char curr_noise_floor; /* current noise floor estimation */
    unsigned char rds_mode;         /* RDS operation mode (RDS/RDBS) */
    unsigned char curr_rssi;        /* Cached value of RSSI for the current frequency */
    unsigned char audio_mode;
    unsigned char audio_path;
    unsigned short aud_ctrl;
    unsigned char pcm_reg;
    unsigned char seek_direction;
    unsigned char seek_wrap;
    unsigned char curr_search_state;
    unsigned short fm_rds_mask;     /* FM/RDS interrupt mask */

    struct fm_rds rds;


    u8 af_mode;         /* Alternate frequency on/off */
    //u8 no_of_chans;     /* Number stations found */
    char current_pins[3]; /*Current pins configuration. either I2S or PCM*/
};

struct fm_device_info {
    unsigned int capabilities;       /* Device capabilities */
    unsigned int tuner_capability;   /* Tuner capability */
    enum v4l2_tuner_type type;
    unsigned int rxsubchans;
    unsigned int aud_mode;
};

/* FM driver operation structure */
struct fmdrv_ops {
    struct video_device *radio_dev;   /* V4L2 video device pointer */
    struct v4l2_device v4l2_dev;      /* V4L2 top level struct */

    long flag;                         /*  FM driver state machine info */
    struct sk_buff_head rx_q;          /* RX queue */


    struct workqueue_struct *rx_wq;     /* Fm workqueue */
    struct work_struct rx_work;    /* Rx work queue */

    /* Seek task completion handler */
    struct completion seektask_completion;

    unsigned char curr_fmmode;   /* Current FM chip mode (TX, RX, OFF) */
    unsigned char aud_ctrl;     /* Current Audio Control (STEREO/MONO/NONE) */
    struct fm_rx rx;                         /* FM receiver info */
    struct fm_device_info device_info; /* FM Device info */

};

#endif
