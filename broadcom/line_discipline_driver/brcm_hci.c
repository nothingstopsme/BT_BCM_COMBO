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
*
*  Filename:      brcm_hci.c
*
*  Description:   Broadcom Bluetooth Low Power UART protocol
*
*****************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>

#include <linux/init.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/poll.h>
#include <linux/timer.h>
/*#include <asm/gpio.h>*/

#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/signal.h>
#include <linux/ioctl.h>
#include <linux/skbuff.h>
#include <linux/serial_core.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "brcm_hci_uart.h"
#include "v4l2_target.h"
#include "fm.h"
#include "ant.h"
#include "v4l2_logs.h"


/*****************************************************************************
**  Constants & Macros for dynamic logging
*****************************************************************************/

/* set this module parameter to enable debug info */
extern struct sock *nl_sk_hcisnoop;



#if V4L2_SNOOP_ENABLE
/* parameter to enable HCI snooping */
extern int ldisc_snoop_enable_param;
#endif


/*****************************************************************************
**  Constants & Macros
*****************************************************************************/

/* HCIBRCM commands */
#define HCIBRCM_GO_TO_SLEEP_IND    0x30
#define HCIBRCM_GO_TO_SLEEP_ACK    0x31
#define HCIBRCM_WAKE_UP_IND    0x32
#define HCIBRCM_WAKE_UP_ACK    0x33


#define TIMER_PERIOD 100    /* 100 ms */
#define HOST_CONTROLLER_IDLE_TSH 4000  /* 4 s */

#define RESP_BUFF_SIZE 30

#define BT_WAKE  22


/* Message event ID passed from Host/Controller lib to stack */
#define MSG_HC_TO_STACK_HCI_ERR        0x1300 /* eq. BT_EVT_TO_BTU_HCIT_ERR */
#define MSG_HC_TO_STACK_HCI_ACL        0x1100 /* eq. BT_EVT_TO_BTU_HCI_ACL */
#define MSG_HC_TO_STACK_HCI_SCO        0x1200 /* eq. BT_EVT_TO_BTU_HCI_SCO */
#define MSG_HC_TO_STACK_HCI_EVT        0x1000 /* eq. BT_EVT_TO_BTU_HCI_EVT */
#define MSG_HC_TO_FM_HCI_EVT           0x3000 /* Response code for FM HCI event */
#define MSG_HC_TO_STACK_L2C_SEG_XMIT   0x1900 /*eq. BT_EVT_TO_BTU_L2C_SEG_XMIT*/

/* Message event ID passed from stack to Host/Controller lib */
#define MSG_STACK_TO_HC_HCI_ACL        0x2100 /* eq. BT:_EVT_TO_LM_HCI_ACL */
#define MSG_STACK_TO_HC_HCI_SCO        0x2200 /* eq. BT_EVT_TO_LM_HCI_SCO */
#define MSG_STACK_TO_HC_HCI_CMD        0x2000 /* eq. BT_EVT_TO_LM_HCI_CMD */
#define MSG_FM_TO_HC_HCI_CMD           0x4000

struct priv_data {
    struct rx_tx_desc curr_rx;
    struct sk_buff_head txq;
};


/* Function forward declarations */
static int brcm_open(struct hci_uart *hu);
static int brcm_close(struct hci_uart *hu);
static int brcm_recv(struct hci_uart *hu, void *data, int count);
static int brcm_enqueue(struct hci_uart *hu, struct sk_buff *skb);
static struct sk_buff *brcm_dequeue(struct hci_uart *hu);
static int brcm_flush(struct hci_uart *hu);

static struct hci_uart_proto brcm_proto = {
    .id      = HCI_UART_BRCM,
    .open    = brcm_open,
    .close   = brcm_close,
    .recv    = brcm_recv,
    .enqueue = brcm_enqueue,
    .dequeue = brcm_dequeue,
    .flush   = brcm_flush,
};

static struct priv_data brcm_data;

/*****************************************************************************
**   UART API calls - Start
*****************************************************************************/

/*****************************************************************************
**
** Function - brcm_open()
**
** Description - Initialize protocol
**
** Returns - 0 if success; else errno
**
*****************************************************************************/
static int brcm_open(struct hci_uart *hu)
{
    BRCM_LDISC_DBG(V4L2_DBG_INIT, "hu %p", hu);

    memset(&brcm_data, 0, sizeof(brcm_data));

    skb_queue_head_init(&brcm_data.txq);

    return 0;
}

/*****************************************************************************
**
** Function - brcm_flush()
**
** Description - Flush protocol data
**
** Returns - 0 if success; else errno
**
*****************************************************************************/
static int brcm_flush(struct hci_uart *hu)
{
    struct sk_buff *skb;

    BRCM_LDISC_DBG(V4L2_DBG_INIT, "hu %p", hu);

    spin_lock(&brcm_data.txq.lock);
    __skb_queue_purge(&brcm_data.txq);
    spin_unlock(&brcm_data.txq.lock);

    return 0;
}

/*****************************************************************************
**
** Function - brcm_close()
**
** Description - Close protocol
**
** Returns - 0 if success; else errno
**
*****************************************************************************/
static int brcm_close(struct hci_uart *hu)
{

    BRCM_LDISC_DBG(V4L2_DBG_INIT, "hu %p", hu);

    brcm_flush(hu);

    if (brcm_data.curr_rx.skb) {
        kfree_skb(brcm_data.curr_rx.skb);
        brcm_data.curr_rx.skb = NULL;
    }

    return 0;
}

/*****************************************************************************
**
** Function - brcm_enqueue()
**
** Description - Enqueue frame for transmittion (padding, crc, etc).
**               May be called by two simultaneous tasklets
**
** Returns - 0 if success; else errno
**
*****************************************************************************/
static int brcm_enqueue(struct hci_uart *hu, struct sk_buff *skb)
{
    struct hci_snoop_hdr snoop_hdr = {0};

    BRCM_LDISC_DBG(V4L2_DBG_TX, "hu %p skb %p", hu, skb);

#if V4L2_SNOOP_ENABLE
    snoop_hdr.len = skb->len - 1;

    switch(skb->data[0])
    {
        /* Construct header for sent packet */
        case HCI_COMMAND_PKT:
            BRCM_LDISC_DBG(V4L2_DBG_TX, "HCI_COMMAND_PKT");
            snoop_hdr.event = MSG_STACK_TO_HC_HCI_CMD;
            break;
        case HCI_ACLDATA_PKT:
            BRCM_LDISC_DBG(V4L2_DBG_TX, "HCI_ACLDATA_PKT");
            snoop_hdr.event = MSG_STACK_TO_HC_HCI_ACL;
            break;
        case HCI_SCODATA_PKT:
            BRCM_LDISC_DBG(V4L2_DBG_TX, "HCI_SCODATA_PKT");
            snoop_hdr.event = MSG_STACK_TO_HC_HCI_SCO;
            break;
        case HCI_FM_PKT:
            BRCM_LDISC_DBG(V4L2_DBG_TX, "HCI_FM_PKT");
            snoop_hdr.event = MSG_FM_TO_HC_HCI_CMD;
            break;
        default:
            BRCM_LDISC_ERR("Unknown HCI tx packet type 0x%02x", skb->data[0]);
            break;
    }

    if (snoop_hdr.event) {
        /* Copying the first 2 bytes of data
         * (excluding the leading byte indicating the packet type)
         * to layer_specific
         */
        memcpy(&snoop_hdr.layer_specific, skb->data+1, 2);

        brcm_hci_snoop_send(hu, &snoop_hdr, skb->data+1, V4L2_DBG_TX);
    }

#endif


    spin_lock(&brcm_data.txq.lock);
	if (hu->tx_on) {
        __skb_queue_tail(&brcm_data.txq, skb);
        skb = NULL;
    }
    spin_unlock(&brcm_data.txq.lock);

    if (skb) {
        BRCM_LDISC_DBG(V4L2_DBG_TX, "tx has benn shut down; skb dropped");
        kfree_skb(skb);
    }
    return 0;
}

/*****************************************************************************
**
** Function - brcm_recv()
**
** Description - Recv data
**
** Returns - 0 if success; else errno
**
*****************************************************************************/
static int brcm_recv(struct hci_uart *hu, void *data, int count)
{
    struct rx_tx_desc *curr_rx = &brcm_data.curr_rx;
    int len, snoop_skipped_len;
    int err = 0;


    BRCM_LDISC_DBG(V4L2_DBG_RX, "hu %p count %d", hu, count);

    while (count) {
        if (!curr_rx->skb) {
            if (curr_rx->state == HCIBRCM_W4_PACKET_TYPE) {
                curr_rx->snoop_hdr.len = 1;

                // Starting with a small buffer
                curr_rx->skb = alloc_skb(16, GFP_KERNEL);
            } else {
                if (!curr_rx->snoop_hdr.len) {
                    curr_rx->snoop_hdr.len = min(count, curr_rx->remaining);
                    curr_rx->remaining -= curr_rx->snoop_hdr.len;
                }

                curr_rx->skb = alloc_skb(curr_rx->snoop_hdr.len, GFP_KERNEL);
            }

            if (!curr_rx->skb) {
                BRCM_LDISC_ERR("Failed to allocate a new rx buffer");
                err = -ENOMEM;
                goto out;
            }
        }

        len = curr_rx->snoop_hdr.len - curr_rx->skb->len;
        if (len) {
            len = min(len, count);
            memcpy(skb_put(curr_rx->skb, len), data, len);
            count -= len; data += len;

            if (curr_rx->skb->len < curr_rx->snoop_hdr.len)
                continue;
        }

        switch (curr_rx->state) {
            case HCIBRCM_W4_PACKET_TYPE: {
                switch (curr_rx->skb->data[0]) {
                    case HCI_EVENT_PKT:
                        curr_rx->snoop_hdr.len += HCI_EVENT_HDR_SIZE;
                        curr_rx->snoop_hdr.event = MSG_HC_TO_STACK_HCI_EVT;
                        break;
                    case HCI_ACLDATA_PKT:
                        curr_rx->snoop_hdr.len += HCI_ACL_HDR_SIZE;
                        curr_rx->snoop_hdr.event = MSG_HC_TO_STACK_HCI_ACL;
                        break;
                    case HCI_SCODATA_PKT:
                        curr_rx->snoop_hdr.len += HCI_SCO_HDR_SIZE;
                        curr_rx->snoop_hdr.event = MSG_HC_TO_STACK_HCI_SCO;
                        break;
                    /* Channel 8(FM) packet */
                    case HCI_FM_PKT:
                        curr_rx->snoop_hdr.len += FM_EVENT_HDR_SIZE;
                        curr_rx->snoop_hdr.event = MSG_HC_TO_FM_HCI_EVT;
                        break;
                    default:
                        BRCM_LDISC_ERR("Unknown HCI rx packet type 0x%02x", curr_rx->skb->data[0]);
                        err = -EINVAL;
                        goto out;
                }

                curr_rx->state = HCIBRCM_W4_HEADER;

                break;
            }
            case HCIBRCM_W4_HEADER: {
                const unsigned char *hdr_start = curr_rx->skb->data+1;
                __u32 expansion_size = 0;
                switch (curr_rx->skb->data[0]) {
                    case HCI_EVENT_PKT: {
                        struct hci_event_hdr *evt_hdr = (struct hci_event_hdr*)hdr_start;
                        BRCM_LDISC_DBG(V4L2_DBG_RX, "Event packet: evt = 0x%02x, plen = %hhu",
                                      evt_hdr->evt, evt_hdr->plen);
                        expansion_size = evt_hdr->plen;
                        curr_rx->snoop_hdr.len += expansion_size;
                        curr_rx->comp_type = COMPONENT_BT;
                        break;
                    }
                    case HCI_ACLDATA_PKT: {
                        struct hci_acl_hdr *acl_hdr = (struct hci_acl_hdr*)hdr_start;
                        len = __le16_to_cpu(acl_hdr->dlen);
                        BRCM_LDISC_DBG(V4L2_DBG_RX, "ACL packet: handle = 0x%04x, dlen = %hu",
                                      __le16_to_cpu(acl_hdr->handle), len);
                        curr_rx->remaining = len;
                        curr_rx->comp_type = COMPONENT_BT;
                        break;
                    }
                    case HCI_SCODATA_PKT: {
                        struct hci_sco_hdr *aco_hdr = (struct hci_sco_hdr*)hdr_start;
                        BRCM_LDISC_DBG(V4L2_DBG_RX, "SCO packet: handle = 0x%04x,  dlen = %hhu",
                                      __le16_to_cpu(aco_hdr->handle), aco_hdr->dlen);
                        curr_rx->remaining = aco_hdr->dlen;
                        curr_rx->comp_type = COMPONENT_BT;
                        break;
                    }
                    /* Channel 8(FM) packet */
                    case HCI_FM_PKT: {
                        struct fm_event_hdr *fm_hdr = (struct fm_event_hdr*)hdr_start;

                        BRCM_LDISC_DBG(V4L2_DBG_RX, "FM CH8 packet: event = 0x%02x, plen = %hhu", fm_hdr->event, fm_hdr->plen);
                        expansion_size = fm_hdr->plen;
                        curr_rx->snoop_hdr.len += expansion_size;
                        curr_rx->comp_type = COMPONENT_FM;
                        break;
                    }
                    default:
                        BRCM_LDISC_ERR("An impossible branch was reached with an unknown rx packet type = %02x; aborting", curr_rx->skb->data[0]);
                        err = -EINVAL;
                        goto out;
                }

                if (expansion_size && skb_tailroom(curr_rx->skb) < expansion_size) {
                    if (pskb_expand_head(curr_rx->skb, 0, expansion_size - skb_tailroom(curr_rx->skb), GFP_KERNEL)){
                        BRCM_LDISC_ERR("Failed to expand the buffer size to accommodate more data");
                        err = -ENOMEM;
                        goto out;
                    }
                }

                /* When curr_rx->skb->len == curr_rx->snoop_hdr.len,
                 * this rx skb is passed on directly by falling
                 * through to the following two cases sequentially
                 */
                if (curr_rx->skb->len < curr_rx->snoop_hdr.len) {
                    curr_rx->state = HCIBRCM_PEEKING_DATA;
                    break;
                }
            }
            case HCIBRCM_PEEKING_DATA: {
                if (curr_rx->skb->data[0] == HCI_EVENT_PKT) {
                    /* In normal case all the HCI events gets routed to BT protocol driver
                     * But in this case opcode FC61 sent by FM driver hence the response
                     * has to go to FM driver so modifying the  type and protoid
                     */
                    if (hu->components_registered & (1 << COMPONENT_FM) && ((curr_rx->skb->data[1]==0x0e
                        && (curr_rx->skb->data[4]==0x61 || curr_rx->skb->data[4]==0x15)
                        && curr_rx->skb->data[5]==0xFC ) ||
                        (curr_rx->skb->data[1]==0xFF
                        && curr_rx->skb->data[3]==0x08)))
                    {
                        curr_rx->skb->data[0] = HCI_FM_PKT;
                        curr_rx->comp_type = COMPONENT_FM;
                    }
#ifdef V4L2_ANT
                    else if (unlikely(curr_rx->skb->data[1]==0x0e
                        && curr_rx->skb->data[4]==0xec
                        && curr_rx->skb->data[5]==0xFC ))
                    {
                        curr_rx->skb->data[0] = HCI_ANT_PKT;
                        curr_rx->comp_type = COMPONENT_ANT;
                        BRCM_LDISC_DBG(V4L2_DBG_RX, "brcm_recv ANT cmd complete evt");
                    }
                    else if (unlikely(curr_rx->skb->data[1]==0xff
                        && curr_rx->skb->data[3]==0x2d ))
                    {
                        curr_rx->skb->data[0] = HCI_ANT_PKT;
                        curr_rx->comp_type = COMPONENT_ANT;
                        BRCM_LDISC_DBG(V4L2_DBG_RX, "brcm_recv ANT evt");
                    }
#endif

                }
                snoop_skipped_len = 1;

                /* Falling through deliberately */
            }
            case HCIBRCM_W4_DATA: {
                sh_ldisc_cb(curr_rx->skb)->comp_type = curr_rx->comp_type;

#if V4L2_SNOOP_ENABLE
                curr_rx->snoop_hdr.len -= snoop_skipped_len;
                brcm_hci_snoop_send(hu, &snoop_hdr, curr_rx->skb->data+snoop_skipped_len, V4L2_DBG_RX);

#endif
                snoop_skipped_len = 0;

                brcm_hci_uart_route_frame(hu, curr_rx->comp_type, curr_rx->skb);

                curr_rx->snoop_hdr.offset += curr_rx->snoop_hdr.len;
                if (curr_rx->remaining) {
                    /* There are data remaining for the current packet;
                     * only partial info in the curr_rx needs to be reset
                     */
                    curr_rx->skb = NULL;
                    curr_rx->snoop_hdr.len = 0;
                    curr_rx->state = HCIBRCM_W4_DATA;
                }
                else {
                    /* The current packet has been fully recieved;
                     * resetting the content of curr_rx to
                     * accept a new packet
                     */
                    memset(curr_rx, 0, sizeof(*curr_rx));
                }

                break;
            }
            default: {
                BRCM_LDISC_ERR("An impossible branch was reached with an unknown rx state = %d; aborting", curr_rx->state);
                err = -EINVAL;
                goto out;

                break;
            }

        }

    }
out:
    if (err) {
        if (curr_rx->skb)
            kfree_skb(curr_rx->skb);

        memset(curr_rx, 0, sizeof(*curr_rx));
    }
    return err;

}


/*****************************************************************************
**
** Function - brcm_dequeue()
**
** Description - Dequeue skb from the skb queue
**
** Returns - 0 if success; else errno
**
*****************************************************************************/
static struct sk_buff *brcm_dequeue(struct hci_uart *hu)
{
    struct sk_buff *skb;

    spin_lock(&brcm_data.txq.lock);
    skb = __skb_dequeue(&brcm_data.txq);
    spin_unlock(&brcm_data.txq.lock);

    return skb;
}

/*****************************************************************************
**   UART API calls - End
*****************************************************************************/



/*****************************************************************************
**
** Function - brcm_init()
**
** Description - Registers the UART function pointers to the Line
                  discipline driver. Called after the UART driver
                  is loaded
**
** Returns - 0 if success; else errno
**
*****************************************************************************/
int brcm_init(void)
{
    int err = brcm_hci_uart_register_proto(&brcm_proto);

    if (!err)
        BRCM_LDISC_DBG(V4L2_DBG_INIT, "HCIBRCM protocol initialized");
    else
        BRCM_LDISC_ERR("HCIBRCM protocol registration failed");

    return err;
}

/*****************************************************************************
**
** Function - brcm_deinit()
**
** Description - Unregisters the UART function pointers from the Line
                  discipline driver
**
** Returns - 0 if success; else errno
**
*****************************************************************************/

int brcm_deinit(void)
{
    return brcm_hci_uart_unregister_proto(&brcm_proto);
}


