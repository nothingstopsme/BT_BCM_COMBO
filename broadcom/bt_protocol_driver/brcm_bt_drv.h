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


/*****************************************************************************
**
**  Name:           brcm_bt_drv.h
**
**  Description:    This is the header file for.Bluetooth protocol driver
**
*****************************************************************************/


#ifndef _BT_DRV_H
#define _BT_DRV_H
#include <linux/interrupt.h>
#include "v4l2_target.h"
#include "v4l2_logs.h"

#define WORKER_QUEUE TRUE

/* Defines number of seconds to wait for reg completion
 * callback getting called from ST (in case,registration
 * with ST returns PENDING status)
 */
#define BT_REGISTER_TIMEOUT   msecs_to_jiffies(6000)    /* 6 sec */

/* BT driver's local status */
#define BT_ST_REGISTERED   0
#define BT_RX_ONGOING      2
#define BT_TX_ONGOING      3
#define BT_DEV_OPENING     4
#define BT_DEV_CLOSING     5


#define BRCM_BT_DEV_MAJOR 0


#ifndef BTDRV_DEBUG
#define BTDRV_DEBUG TRUE
#endif


/* Debug/Error Messaging for BT protocol driver */
#if BTDRV_DEBUG
#define BT_DRV_DBG(flag, fmt, arg...) \
        do { \
            if (bt_dbg_param & flag) \
                printk(KERN_DEBUG "(btdrv):%s  "fmt"\n" , \
                                           __func__,## arg); \
        } while(0)
#else
    #define BT_DRV_DBG(flag, fmt, arg...)
#endif

#define BT_DRV_ERR(fmt, arg...)  printk(KERN_ERR "(btdrv):%s  "fmt"\n" , \
                                           __func__,## arg)

extern int bt_dbg_param;

struct brcm_bt_dev {
    /*register device to linux system*/
    struct cdev c_dev;
    struct class *cl;

    /* used locally,to maintain various BT driver status */
    unsigned long flags;


    long flag;                           /*  BT driver state machine info */
    struct sk_buff_head rx_q;            /* RX queue */

    struct rx_tx_desc curr_tx;

    /* queue for polling */
    wait_queue_head_t inq;
};

#endif

