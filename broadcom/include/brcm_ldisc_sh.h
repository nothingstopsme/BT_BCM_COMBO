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


/*******************************************************************************
 *
 *  Filename:      brcm_ldisc_sh.h
 *
 *  Description:   Shared Transport Header file
 *   To be included by the protocol stack drivers for
 *   Broadcom BT,FM and GPS combo chip drivers
 *
 ******************************************************************************/

#ifndef LDISC_SH_H
#define LDISC_SH_H
#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci.h>
#include <linux/skbuff.h>
#define CONFIG_BT_HCIUART_BRCM

/*******************************************************************************
**  Constants
*******************************************************************************/


/* HCIBRCM rx/tx States */
#define HCIBRCM_W4_PACKET_TYPE 0
#define HCIBRCM_W4_HEADER      1
#define HCIBRCM_PEEKING_DATA   2
#define HCIBRCM_W4_DATA        3


/*
 * enum component_type - various components on the chip which share a
 *  common physical interface like UART.
 */
enum component_type {
    COMPONENT_BT,
    COMPONENT_FM,
    COMPONENT_GPS,
    COMPONENT_ANT,
    COMPONENT_MAX,
};

/*
 * enum sleep-type - Sleep control is board specific
 */
enum sleep_type {
    SLEEP_DEFAULT,
    SLEEP_BLUESLEEP,
};


void brcm_btsleep_wake( enum sleep_type type);
int brcm_btsleep_start(enum sleep_type type);
void brcm_btsleep_stop(enum sleep_type type);


#define sh_ldisc_cb(skb) ((struct sh_ldisc_skb_cb *)((skb)->cb))

/*******************************************************************************
**  Type definitions
*******************************************************************************/

struct hci_snoop_hdr {
    unsigned short          event;
    unsigned short          len;
    unsigned short          offset;
    unsigned short          layer_specific;
} __attribute__ ((packed));

struct rx_tx_desc {
    unsigned char state;
    __u32 remaining;
    enum component_type comp_type;
    struct hci_snoop_hdr snoop_hdr;
    struct sk_buff *skb;
};

struct sh_ldisc_skb_cb {
    enum component_type comp_type;
} __attribute__ ((packed));

/**
 * struct component_interface - the interface to BT/FM/GPS component registerd to the shared ldisc
 * @type: type of the protocol being registered among the
 *  available proto_type(BT, FM, GPS the protocol which share TTY).
 * @recv: the receiver callback pointing to a function in the
 *  protocol drivers called by the shared ldisc driver upon receiving
 *  relevant data.
 * @match_packet: reserved for future use, to make ST more generic
 * @extract_sync_id: if provided, called to extract the id of the input skb
 *  based on its content; used in synchronous sending to match responses
 *  with requests of the same id
 * @write: pointer to function in shared ldisc provided to protocol drivers,
 *  to be made use when protocol drivers have data to send to TTY.
 */
struct component_interface {
    enum component_type type;
    long (*recv) (struct sk_buff *);
    unsigned char (*match_packet) (const unsigned char *data);
    __u32 (*extract_sync_id) (struct sk_buff *);
    struct sk_buff *(*write) (struct sk_buff *skb, bool never_blocking);
};

/*******************************************************************************
** macro/inline/extern functions
*******************************************************************************/

extern long brcm_sh_ldisc_register(struct component_interface *);
extern long brcm_sh_ldisc_unregister(enum component_type, bool btsleep_open);


#endif /* LDISC_H */
