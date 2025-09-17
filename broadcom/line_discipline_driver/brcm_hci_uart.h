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
**  Name:           brcm_hci_uart.h
**
**  Description:    This is the header file for.Bluetooth HCI UART driver
**
*****************************************************************************/

#ifndef BRCM_HCI_UART_H
#define BRCM_HCI_UART_H

#include <uapi/linux/tty.h>
#include <linux/platform_device.h>
#include <linux/firmware.h>
#include "brcm_ldisc_sh.h"
#include "v4l2_target.h"
#include "v4l2_logs.h"


/*****************************************************************************
**  Constants & Macros
*****************************************************************************/

/*
 * Note: do not assume N_BRCM_HCI = 25,
 * as global headers (like tty.h) might already define this
 * macro to another value
 */
#ifndef N_BRCM_HCI
#define N_BRCM_HCI      25
#endif

/* Ioctls */
#define HCIUARTSETPROTO     _IOW('U', 200, int)
#define HCIUARTGETPROTO     _IOR('U', 201, int)
#define HCIUARTGETDEVICE    _IOR('U', 202, int)

/* UART protocols */
#define HCI_UART_MAX_PROTO      1

#define HCI_UART_BRCM           0


#define VENDOR_PARAMS_LEN 300

/* Debug/Error Messaging */
#ifndef BTLDISC_DEBUG
#define BTLDISC_DEBUG TRUE
#endif

#if (BTLDISC_DEBUG)
#define BRCM_LDISC_DBG(flag, fmt, arg...) \
            do { \
                if (ldisc_dbg_param & flag) \
                    printk(KERN_DEBUG "(brcmhci):%s:%d  "fmt"\n" , \
                                               __func__, __LINE__, ## arg); \
            } while(0)
#else
#define BRCM_LDISC_DBG(flag,fmt, arg...)
#endif

#define BRCM_LDISC_ERR(fmt, arg...)  printk(KERN_ERR "(brcmhci):%s:%d  "fmt"\n" , \
                                           __func__, __LINE__,## arg)

extern int ldisc_dbg_param;


struct hci_uart_proto {
    unsigned int id;
    int (*open)(struct hci_uart *hu);
    int (*close)(struct hci_uart *hu);
    int (*flush)(struct hci_uart *hu);
    int (*recv)(struct hci_uart *hu, void *data, int len);
    int (*enqueue)(struct hci_uart *hu, struct sk_buff *skb);
    struct sk_buff *(*dequeue)(struct hci_uart *hu);
};

struct hci_uart_proto_desc;

struct hci_uart {
    struct tty_struct *tty;
    struct hci_dev *hdev;
    unsigned long flags;

    struct hci_uart_proto_desc *curr_proto_desc;

    spinlock_t component_descs_lock;
    struct component_desc *component_descs[COMPONENT_MAX];
    unsigned char components_registered;

    struct completion ldisc_installed;
    struct completion ldisc_patchram_complete;

    struct platform_device *brcm_pdev;
    const struct firmware *fw_entry;

    unsigned char ldisc_install;
    unsigned char ldisc_bt_err;
    unsigned char ldisc_fm_err;
    spinlock_t err_lock;

    /* 
     * vendor params represented as a comma-separated string
     * read from bt_vendor.conf
     */
    char vendor_params[VENDOR_PARAMS_LEN];

#if V4L2_SNOOP_ENABLE
    spinlock_t hcisnoop_lock;
    spinlock_t hcisnoop_write_lock;
#endif
    struct completion tty_close_complete;

    bool tx_on;
    struct workqueue_struct *tx_wq;
    struct work_struct tx_work;
};


/*****************************************************************************
**  Functions
*****************************************************************************/

int brcm_hci_uart_register_proto(struct hci_uart_proto *p);
int brcm_hci_uart_unregister_proto(struct hci_uart_proto *p);
void brcm_hci_uart_route_frame(struct hci_uart *hu, enum component_type comp_type, struct sk_buff *skb);
#if V4L2_SNOOP_ENABLE
int brcm_hci_snoop_send(struct hci_uart *hu, struct hci_snoop_hdr *snoop_hdr, const unsigned char* data, unsigned int log_flag);
#endif

int brcm_init(void);
int brcm_deinit(void);

#endif // BRCM_HCI_UART_H
