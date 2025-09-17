/*
 *  FM Driver for Connectivity chip of Broadcom Corporation.
 *
 *  This sub-module of FM driver is common for FM RX and TX
 *  functionality. This module is responsible for:
 *  1) Forming group of Channel-8 commands to perform particular
 *     functionality (eg., frequency set require more than
 *     one Channel-8 command to be sent to the chip).
 *  2) Sending each Channel-8 command to the chip and reading
 *     response back over Shared Transport.
 *  3) Managing TX and RX Queues and Tasklets.
 *  4) Handling FM Interrupt packet and taking appropriate action.
 *
 *  Copyright (C) 2009 Texas Instruments
 *  Copyright (C) 2009-2014 Broadcom Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/************************************************************************************
 *
 *  Filename:      fmdrv_main.c
 *
 *  Description:   Common sub-module for both FM Rx and Tx. Currently, only
 *                  is supported
 *
 ***********************************************************************************/

#include <linux/module.h>
#include <linux/delay.h>
#include "fmdrv_config.h"
#include "fmdrv.h"
#include "fmdrv_v4l2.h"
#include "fmdrv_main.h"
#include "brcm_ldisc_sh.h"
#include "fmdrv_rx.h"
#include "fm_public.h"
#include "v4l2_logs.h"


/* set this module parameter to enable debug info */
int fm_dbg_param = 0;

/* Region info */
const struct band_info band_configs[] = {
    /* Europe */
    {
        .low_bound = FM_GET_FREQ(8750),    /* 87.5 MHz */
        .high_bound = FM_GET_FREQ(10800),    /* 108 MHz */
        .deemphasis = FM_DEEMPHA_50U,
        .scan_step = FM_STEP_100KHZ,
        .fm_band = FM_BAND_EUROPE,
        .rds_support = FM_RDS_BIT,
    },

    /* Japan */
    {
        .low_bound = FM_GET_FREQ(7600),    /* 76 MHz */
        .high_bound = FM_GET_FREQ(10800),    /* 108 MHz */
        .deemphasis = FM_DEEMPHA_50U,
        .scan_step = FM_STEP_100KHZ,
        .fm_band = FM_BAND_JAPAN,
        .rds_support = FM_RDS_BIT,
    },

    /* North America */
    {
        .low_bound = FM_GET_FREQ(8750),    /* 87.5 MHz */
        .high_bound = FM_GET_FREQ(10800),    /* 108 MHz */
        .deemphasis = FM_DEEMPHA_75U,
        .scan_step = FM_STEP_200KHZ,
        .fm_band = FM_BAND_NA,
        .rds_support = FM_RBDS_BIT,
    },

    /* Russia-Ext */
    {
        .low_bound = FM_GET_FREQ(6580),    /* 65.8 MHz */
        .high_bound = FM_GET_FREQ(10800),    /* 108 MHz */
        .deemphasis = FM_DEEMPHA_75U,
        .scan_step = FM_STEP_100KHZ,
        .fm_band = FM_BAND_RUSSIAN,
        .rds_support = FM_RDS_BIT,
    },

    /* China Region */
    {
        .low_bound = FM_GET_FREQ(7600),    /* 76 MHz */
        .high_bound = FM_GET_FREQ(10800),    /* 108 MHz */
        .deemphasis = FM_DEEMPHA_75U,
        .scan_step = FM_STEP_100KHZ,
        .fm_band = FM_BAND_CHINA,
        .rds_support = FM_RDS_BIT,
    },

    /* Italy/Thailand */
    {
        .low_bound = FM_GET_FREQ(8750),    /* 87.5 MHz */
        .high_bound = FM_GET_FREQ(10800),    /* 108 MHz */
        .deemphasis = FM_DEEMPHA_50U,
        .scan_step = FM_STEP_50KHZ,
        .fm_band = FM_BAND_ITALY,
        .rds_support = FM_RDS_BIT,
    },
};

/*******************************************************************************
**  Forward function declarations
*******************************************************************************/

static int parse_inrpt_flags(struct fmdrv_ops *fmdev, struct sk_buff *skb);
static int parse_rds_data(struct fmdrv_ops *fmdev, struct sk_buff *skb);
static void send_read_intrp_cmd(struct fmdrv_ops *fmdev);
static int read_rds_data(struct fmdrv_ops *);

static long fm_st_receive(struct sk_buff *skb);
static __u32 fm_extract_sync_id(struct sk_buff *skb);


/*******************************************************************************
**  Static Variables
*******************************************************************************/
//#define BTYES_TO_UINT16(u16, lsb, msb) {u16 = (UINT16)(((UINT16)(lsb) << 8) + (UINT16)(msb)); }
/* Program Type */
static const char * const pty_str[]= {
                         "None", "News", "Current Affairs",
                         "Information","Sport", "Education",
                         "Drama", "Culture","Science",
                         "Varied Speech", "Pop Music",
                         "Rock Music","Easy Listening",
                         "Light Classic Music", "Serious Classics",
                         "other Music","Weather", "Finance",
                         "Childrens Progs","Social Affairs",
                         "Religion", "Phone In", "Travel",
                         "Leisure & Hobby","Jazz", "Country",
                         "National Music","Oldies","Folk",
                         "Documentary", "Alarm Test", "Alarm"};

static struct {
   /*These are the RDS elements that we are intested to parse*/
   /*Each variable represents one RDS element*/
   /*pi_code   -Program Identification code*/
   __u16 pi_code;
   /* pty  --Program Type, Actually an integer value is transmitted and */
   /* if we pass the value to pty_str, we get the appropriate string */
   __u8 pty:5;
   /*tp*/
   __u8 tp:1;
   /*ta */
   __u8 ta:1;
   /*ms_code, a bit represneting Music or Speech*/
   __u8 ms_code:1;

   /*ps is the program/station name in string format, max length is 8 bytes + null terminate */
   char ps[9];
   /*rt is radio text message, this could be custom message*/
   char rt[65];

   /*holds the 3 byte RDS tuple data*/
   __u8 rds_tuple[3];

   __u8 skip_flag:1;
   __u8 rt_AB:1;

   __u8 ps_markers;
   __u16 rt_markers;
   __u16 rt_checkers;


   __u32 events;

   /*CT is time and date information*/
   struct
   {
      __u32 day;
      __u8 hour;
      __u8 minute;

      __u8 hoffset_reserved:2;
      __u8 hoffset_sign:1;
      __u8 hoffset_magnitude:5;
   } ct;
} rds_parser = { .skip_flag = 1 };

/*Global to store the CT information after parsing it from RDS stream*/
//   static struct ct current_ct;

#if 0
/* Band selection */
static unsigned char default_radio_region;    /* US */
module_param(default_radio_region, byte, 0);
MODULE_PARM_DESC(default_radio_region, "Region: 0=US, 1=Europe, 2=Japan");
#endif

/* RDS buffer blocks */
static unsigned int default_rds_buf = 300;
module_param(default_rds_buf, uint, 0444);
MODULE_PARM_DESC(rds_buf, "RDS buffer entries");

/* Radio Nr */
static int radio_nr = -1;
module_param(radio_nr, int, 0);
MODULE_PARM_DESC(radio_nr, "Radio Nr");

/* Defining the info for registering with the ldisc driver;
 * note that some fields would get updated later in the registration proces
 */
static struct {
  struct component_interface interface;
  struct fmdrv_ops *fmdev;
} fm_comp_def = {
    .interface = {
        .type = COMPONENT_FM,
        .extract_sync_id = fm_extract_sync_id,
        .recv = fm_st_receive
    }
};

/*******************************************************************************
**  Functions
*******************************************************************************/

#ifdef FM_DUMP_TXRX_PKT
 /* To dump outgoing FM Channel-8 packets */
inline void dump_tx_skb_data(struct sk_buff *skb)
{
    int len, len_org;
    char index;
    struct fm_cmd_msg_hdr *cmd_hdr;

    cmd_hdr = (struct fm_cmd_msg_hdr *)skb->data;
    V4L2_FM_DRV_DBG(V4L2_DBG_TX, "<<hdr:%02x len:%02x opcode:%02x type:%s", \
           cmd_hdr->header, \
           cmd_hdr->len, cmd_hdr->fm_opcode, \
           cmd_hdr->rd_wr ? "RD" : "WR");

    len_org = skb->len - FM_CMD_MSG_HDR_SIZE;
    if (len_org > 0)
    {
        //V4L2_FM_DRV_DBG("\n   data(%d): ", cmd_hdr->dlen);
        len = min(len_org, 14);
        for (index = 0; index < len; index++)
            V4L2_FM_DRV_DBG(V4L2_DBG_TX, "%x ", \
                   skb->data[FM_CMD_MSG_HDR_SIZE + index]);
        V4L2_FM_DRV_DBG(V4L2_DBG_TX, "%s", (len_org > 14) ? ".." : "");
    }
}

 /* To dump incoming FM Channel-8 packets */
inline void dump_rx_skb_data(struct sk_buff *skb)
{
    int len, len_org;
    char index;
    struct fm_event_msg_hdr  *evt_hdr;

    evt_hdr = (struct fm_event_msg_hdr *)skb->data;
    V4L2_FM_DRV_DBG(V4L2_DBG_RX, ">> header:%02x event:%02x len:%02x",\
        evt_hdr->header, evt_hdr->event_id, evt_hdr->len);

    len_org = skb->len - FM_EVT_MSG_HDR_SIZE;
    if (len_org > 0)
    {
        V4L2_FM_DRV_DBG(V4L2_DBG_RX, "   data(%d): ", evt_hdr->len);
        len = min(len_org, 14);
        for (index = 0; index < len; index++)
            V4L2_FM_DRV_DBG(V4L2_DBG_RX, "%x ",\
                   skb->data[FM_EVT_MSG_HDR_SIZE + index]);
        V4L2_FM_DRV_DBG(V4L2_DBG_RX, "%s", (len_org > 14) ? ".." : "");
    }
}

#endif

/*
* FM common sub-module will schedule this tasklet whenever it receives
* FM packet from ST driver.
*/
static void fm_receive_data_ldisc(struct work_struct *w)
{
    struct fmdrv_ops *fmdev = container_of(w, struct fmdrv_ops,rx_work);
    struct fm_event_msg_hdr *fm_evt_hdr;
    struct sk_buff *skb;

    /* Process all packets in the RX queue */
    while (true) {

        spin_lock(&fmdev->rx_q.lock);
        skb = __skb_dequeue(&fmdev->rx_q);
        spin_unlock(&fmdev->rx_q.lock);

        if (!skb)
            break;

        fm_evt_hdr = (void *)skb->data;
        switch (fm_evt_hdr->event_id) {
            case HCI_EV_CMD_COMPLETE: {
                // In this case, the skb must be of fm_cmd_complete type
                struct fm_cmd_complete_hdr *fmcmd_complete_hdr = (struct fm_cmd_complete_hdr *)(fm_evt_hdr+1);
                switch (fmcmd_complete_hdr->fm_opcode) {
                    /* Parse the interrupt flags in 0x12 */
                    case FM_REG_FM_RDS_FLAG: {
                        parse_inrpt_flags(fmdev, skb);
                        /* the ownership of skb has been taken over in parse_inrpt_flags() */
                        skb = NULL;
                        break;
                    }
                    case FM_REG_RDS_DATA: {
                        parse_rds_data(fmdev, skb);
                        /* the ownership of skb has been taken over in parse_rds_data() */
                        skb = NULL;
                        break;
                    }
                    default:
                        break;
                }

                break;
            }
            case BRCM_FM_VS_EVENT: {
                unsigned char sub_event = *(unsigned char *)(fm_evt_hdr+1);

                V4L2_FM_DRV_DBG(V4L2_DBG_RX, ": Got Vendor specific Event");
                if(sub_event == BRCM_VSE_SUBCODE_FM_INTERRUPT)
                {
                    V4L2_FM_DRV_DBG(V4L2_DBG_RX, "(fmdrv) VSE Interrupt event for FM received. Calling fmc_send_intrp_cmd().");
                    send_read_intrp_cmd(fmdev);
                }

                break;
            }
            default:
                break;
        }

        if (skb)
            kfree_skb(skb);
    }
}

/* Sending FM Channel-8/VSC HCI packet to ldisc driver for tx transmission */
static struct sk_buff* __fm_send_cmd(struct fmdrv_ops *fmdev, unsigned char fmreg_index,
                void *payload, int payload_len, unsigned char type,
                bool waiting_for_resp)
{
    struct sk_buff *skb, *resp = NULL;
    int size;

    if (!fm_comp_def.interface.write) {
        V4L2_FM_DRV_ERR("%s Error!!! fm_comp_def.interface.write is NULL", __func__);
        return ERR_PTR(-ENOENT);
    }

    if(type == VSC_HCI_CMD) {
        size = FM_VSC_HCI_CMD_MSG_HDR_SIZE + ((payload == NULL) ? 0 : payload_len);
    }
    else {
        size = FM_CMD_MSG_HDR_SIZE + ((payload == NULL) ? 0 : payload_len);
    }

    skb = alloc_skb(size, GFP_KERNEL);
    if (!skb)
    {
        V4L2_FM_DRV_ERR("No memory to create new SKB");
        return ERR_PTR(-ENOMEM);
    }

    /* Fill command header info */
    if (type == VSC_HCI_CMD) {
        struct fm_vsc_hci_cmd_msg_hdr *cmd_hdr =(struct fm_vsc_hci_cmd_msg_hdr *)skb_put(skb, FM_VSC_HCI_CMD_MSG_HDR_SIZE);

        cmd_hdr->header = HCI_COMMAND;    /* 0x01 */

        cmd_hdr->cmd = __cpu_to_le16(hci_opcode_pack(HCI_GRP_VENDOR_SPECIFIC, VSC_HCI_WRITE_PCM_PINS_OCF));
        cmd_hdr->len = 0x05;  /*Fixed value for FC61 command*/

    } else {
        struct fm_cmd_msg_hdr *cmd_hdr =(struct fm_cmd_msg_hdr *)skb_put(skb, FM_CMD_MSG_HDR_SIZE);

        cmd_hdr->header = HCI_COMMAND;    /* 0x01 */

        cmd_hdr->cmd = __cpu_to_le16(hci_opcode_pack(HCI_GRP_VENDOR_SPECIFIC, FM_2048_OP_CODE));
        /* FM opcode */
        cmd_hdr->fm_opcode = fmreg_index;
        /* read/write type */
        cmd_hdr->rd_wr = type;
        cmd_hdr->len = ((payload == NULL) ? 0 : payload_len) + 2;
    }

    /* SYED : Hack to set the right packet type for FM */
    sh_ldisc_cb(skb)->comp_type = COMPONENT_FM;

    if (payload != NULL)
        memcpy(skb_put(skb, payload_len), payload, payload_len);
    resp = fm_comp_def.interface.write(skb, !waiting_for_resp /* never_blocking */);

    if (IS_ERR(resp)) {
        V4L2_FM_DRV_ERR("Failed to send a tx skb(%p)", skb);
        kfree_skb(skb);
    }

    return resp;

}

/* Sends FM Channel-8 command to the chip and waits for the response */
int fmc_send_cmd(struct fmdrv_ops *fmdev, unsigned char fmreg_index,
            void *payload, int payload_len, unsigned char type,
            void *response, int *response_len)
{
    struct sk_buff *skb;
    struct fm_event_msg_hdr *fm_evt_hdr;
    int ret;

    V4L2_FM_DRV_DBG(V4L2_DBG_TX, "In fmc_send_cmd");

    skb = __fm_send_cmd(fmdev, fmreg_index, payload, payload_len, type, true /* waiting_for_resp */);
    if (IS_ERR(skb))
        return PTR_ERR(skb);
    else if (skb == NULL) {
        V4L2_FM_DRV_ERR("(fmdrv): Reponse SKB is missing ");
        return -EFAULT;
    }

    fm_evt_hdr = (void *)skb->data;
    if (fm_evt_hdr->event_id == HCI_EV_CMD_COMPLETE) /* Vendor specific command response */
    {
        int data_size, full_header_size = FM_EVT_MSG_HDR_SIZE;
        struct hci_ev_cmd_complete *cmd_complete_hdr = (struct hci_ev_cmd_complete *)(fm_evt_hdr+1);
        __u8 status = *(__u8 *)(cmd_complete_hdr+1);
        if (status != 0)
        {
            V4L2_FM_DRV_ERR("(fmdrv): Reponse status not success ");
            kfree (skb);
            return -EFAULT;
        }

        if (__le16_to_cpu(cmd_complete_hdr->opcode) == hci_opcode_pack(HCI_GRP_VENDOR_SPECIFIC, VSC_HCI_WRITE_PCM_PINS_OCF)) {
            /*
             * Adding 1 to treat the following status byte as
             * part of the header
             */
            full_header_size += sizeof(*cmd_complete_hdr) + 1;
            data_size = fm_evt_hdr->len - sizeof(*cmd_complete_hdr) - 1;
        } else {
            // __le16_to_cpu(cmd_complete_hdr->opcode) == hci_opcode_pack(HCI_GRP_VENDOR_SPECIFIC, FM_2048_OP_CODE)

            full_header_size += FM_CMD_COMPLETE_HDR_SIZE;
            data_size = fm_evt_hdr->len - FM_CMD_COMPLETE_HDR_SIZE;
        }

        /* Copying response data to caller */
        if (response != NULL && response_len != NULL && data_size > 0) {
            /* Skip header info and copy only response data */
            skb_pull(skb, full_header_size);
            memcpy(response, skb->data, data_size);
            *response_len = data_size;
        }
        else if (response_len != NULL && data_size <= 0) {
            *response_len = 0;
        }
    }
    else
    {
        V4L2_FM_DRV_ERR("(fmdrv): Unhandled event ID : %d", fm_evt_hdr->event_id);
    }
    kfree_skb(skb);
    return 0;
}

/* This function should be called when FM station changes
 * and thus old rds info has to be erased
 */
static void reset_rds(struct fmdrv_ops *fmdev)
{
    /* Clearing up rds parser */
    memset(&rds_parser, 0, sizeof(rds_parser));
    rds_parser.skip_flag = 1;

    /* Clearing up rds circular buffer cache */
    spin_lock(&fmdev->rx.rds.cbuff_lock);
    //fmdev->rx.rds.rds_flag = FM_RDS_DISABLE;
    fmdev->rx.rds.wr_index = 0;
    fmdev->rx.rds.rd_index = 0;
    spin_unlock(&fmdev->rx.rds.cbuff_lock);

    fmdev->device_info.rxsubchans &= ~V4L2_TUNER_SUB_RDS;

}

/* Helper function to parse the interrupt bits
* in FM_REG_FM_RDS_FLAG (0x12).
* Called locally by fmdrv_main.c
*/
static int parse_inrpt_flags(struct fmdrv_ops *fmdev, struct sk_buff *skb)
{
    bool try_reset_mask = false;
    unsigned short fm_rds_flag;
    unsigned char response[2];

    memcpy(&response, &skb->data[FM_EVT_MSG_HDR_SIZE + FM_CMD_COMPLETE_HDR_SIZE], 2);
    fm_rds_flag= (unsigned short)response[0] + ((unsigned short)response[1] << 8) ;


    V4L2_FM_DRV_DBG(V4L2_DBG_TX, "(fmdrv) Processing the interrupt flag. Flag read is 0x%x 0x%x",\
        response[0], response[1]);
    V4L2_FM_DRV_DBG(V4L2_DBG_TX, "(fmdrv) : flag register(0x%x)", fm_rds_flag);
    if(fm_rds_flag & (I2C_MASK_SRH_TUNE_CMPL_BIT|I2C_MASK_SRH_TUNE_FAIL_BIT))
    {
        if(fm_rds_flag & I2C_MASK_SRH_TUNE_FAIL_BIT)
        {
            V4L2_FM_DRV_ERR("(fmdrv) MASK BIT : Search failure");
            if(fmdev->rx.curr_search_state == FM_STATE_SEEKING)
            {
                fmdev->rx.curr_search_state = FM_STATE_SEEK_ERR;
                complete(&fmdev->seektask_completion);
            }
            else if(fmdev->rx.curr_search_state == FM_STATE_TUNING) {
                fmdev->rx.curr_search_state = FM_STATE_TUNE_ERR;
                complete(&fmdev->seektask_completion);
            }
        }
        else
        {
            V4L2_FM_DRV_DBG(V4L2_DBG_TX, "(fmdrv) MASK BIT : Search success");
            if(fmdev->rx.curr_search_state == FM_STATE_SEEKING)
            {
                fmdev->rx.curr_search_state = FM_STATE_SEEK_CMPL;

                reset_rds(fmdev);
                complete(&fmdev->seektask_completion);
            }
            else if(fmdev->rx.curr_search_state == FM_STATE_TUNING) {
                fmdev->rx.curr_search_state = FM_STATE_TUNE_CMPL;

                reset_rds(fmdev);
                complete(&fmdev->seektask_completion);
            }
        }
    }
    else if (fm_rds_flag & I2C_MASK_RDS_FIFO_WLINE_BIT)
    {
        V4L2_FM_DRV_DBG(V4L2_DBG_TX, "(fmdrv) Detected WLINE interrupt; Reading RDS.");
        if (read_rds_data(fmdev))
            try_reset_mask = true;
    }
    else {
        try_reset_mask = true;
    }

    if (try_reset_mask) {
        /*
         * reactivating RDS retrieval if RDS is required
         * so that we can continue getting the notification
         */
        if (fmdev->rx.fm_rds_mask & I2C_MASK_RDS_FIFO_WLINE_BIT)
            fm_rx_set_mask(fmdev, fmdev->rx.fm_rds_mask);
    }

    kfree_skb(skb);
    return 0;
}

int fmc_get_rds_element_value(int ioctl_num, void __user *ioctl_value)
{
/* Even though the values that are returned from this function are global to this file */
/* There is no need to lock  Since this is just a read operation */
/* And we don't maitain the history of any of these values, we just return the */
/* Current value irrespective of it's previous value */
    __u8 tmp;
    int ret = 0;
    switch(ioctl_num)
    {
        case GET_EVENT_MASK: {
            __u32 snapshot = rds_parser.events;
            V4L2_FM_DRV_DBG(V4L2_DBG_RX, "IOCTL_GET_EVENT_MASK");
            if(copy_to_user(ioctl_value, &snapshot, sizeof(snapshot)))
            {
                V4L2_FM_DRV_ERR("(rds) Failed to copy PI code");
                ret = -EIO;
            }

            /*
             * 1. Even though parser's event mask is updated here,
             * it is still fine to conducted it without locking,
             * as it doesn't really matter if there are events unread/uncleared
             * 2. Only clearing events which have been copied
             * to the user buffer
             */
            rds_parser.events &= ~snapshot;
            break;
        }

        case GET_PI_CODE:
            V4L2_FM_DRV_DBG(V4L2_DBG_RX, "IOCTL_GET_PI_CODE");
            if(copy_to_user(ioctl_value, &rds_parser.pi_code, sizeof(rds_parser.pi_code)))
            {
                V4L2_FM_DRV_ERR("(rds) Failed to copy PI code");
                ret = -EIO;
            }
            break;

        case GET_TP_CODE:
            V4L2_FM_DRV_DBG(V4L2_DBG_RX, "IOCTL_GET_TP_CODE");
            tmp = rds_parser.tp;
            if(copy_to_user(ioctl_value, &tmp, sizeof(tmp)))
            {
                V4L2_FM_DRV_ERR("(rds) Failed to copy TP code");
                ret = -EIO;
            }
            break;

        case GET_PTY_CODE:
            V4L2_FM_DRV_DBG(V4L2_DBG_RX, "IOCTL_GET_PTY_CODE");
            tmp = rds_parser.pty;
            if(copy_to_user(ioctl_value, &tmp, sizeof(tmp)))
            {
                V4L2_FM_DRV_ERR("(rds) Failed to copy PTY code");
                ret = -EIO;
            }
            break;

        case GET_TA_CODE:
            V4L2_FM_DRV_DBG(V4L2_DBG_RX, "IOCTL_GET_TA_CODE");
            tmp = rds_parser.ta;
            if(copy_to_user(ioctl_value, &tmp, sizeof(tmp)))
            {
                V4L2_FM_DRV_ERR("(rds) Failed to copy TA code");
                ret = -EIO;
            }
            break;

        case GET_MS_CODE:
            V4L2_FM_DRV_DBG(V4L2_DBG_RX, "IOCTL_GET_MS_CODE");
            tmp = rds_parser.ms_code;
            if(copy_to_user(ioctl_value, &tmp, sizeof(tmp)))
            {
                V4L2_FM_DRV_ERR("(rds) Failed to copy MS code");
                ret = -EIO;
            }
            break;

        case GET_PS_CODE:
            V4L2_FM_DRV_DBG(V4L2_DBG_RX, "IOCTL_GET_PS_CODE");
            if(copy_to_user(ioctl_value, rds_parser.ps, strlen(rds_parser.ps)+ 1))
            {
                V4L2_FM_DRV_ERR("(rds) Failed to copy PS code");
                ret = -EIO;
            }
            break;

        case GET_RT_MSG:
            V4L2_FM_DRV_DBG(V4L2_DBG_RX, "IOCTL_GET_RT_MSG");
            if(copy_to_user(ioctl_value, rds_parser.rt, strlen(rds_parser.rt) + 1))
            {
                V4L2_FM_DRV_ERR("(rds) Failed to copy RT Message");
                ret = -EIO;
            }
            break;

        case GET_CT_DATA:
            V4L2_FM_DRV_DBG(V4L2_DBG_RX, "IOCTL_GET_CT_DATA");
            if(copy_to_user(ioctl_value, &rds_parser.ct, sizeof(rds_parser.ct)))
            {
                V4L2_FM_DRV_ERR("(rds) Failed to copy CT data");
                ret = -EIO;
            }
            break;

        case GET_TMC_CHANNEL: {
            V4L2_FM_DRV_DBG(V4L2_DBG_RX, "IOCTL_GET_TMC_CHANNEL");
            tmp = 4;
            if(copy_to_user(ioctl_value, &tmp, sizeof(tmp)))
            {
                V4L2_FM_DRV_ERR("(rds) Failed to copy TMC_CHANNEL");
                ret = -EIO;
            }
            break;
        }
        default:
            ret = -EINVAL;
            break;
    }

    return ret;
}

/* Each RDS packet is 104bytes = 4*16 bits + 40 bits (10bits error code for each 16bits data)*/
/* Each RDS packet is devided into 4 blocks of 16bits*/
/* when we receive RDS packet it is devided into 4 * 3Byts tuples, */
/*2 bytes of information and 1byte of meta data */
static void parse_rds_tuple(void)
{
    __u8 byte1, byte2;
    static __u8 blkc_byte1, blkc_byte2, blkb_byte1, blkb_byte2;
    __u32 tmp1;
    __u32 tmp2;
    __u8 offset = 0, group = 0, rt_AB;

    byte1 = rds_parser.rds_tuple[1];
    byte2 = rds_parser.rds_tuple[0];

    switch ((rds_parser.rds_tuple[2] & 0x07)) {
        case V4L2_RDS_BLOCK_A: /* Block A */
            if (rds_parser.rds_tuple[2] & BRCM_RDS_BIT_7) {
                break;
            }

            rds_parser.pi_code = (byte1 << 8) | byte2;
            break;

        case V4L2_RDS_BLOCK_B: /* Block B */
            if (rds_parser.rds_tuple[2] & BRCM_RDS_BIT_7) {
                /* invalid tupple */
                rds_parser.skip_flag = 1; /* skip Block D decode */
                break;
            }

            rds_parser.skip_flag = 0;
            rds_parser.pty = ((byte1 << 3) & 0x18) | ((byte2 >> 5) & 0x07);
            rds_parser.tp = (byte1 >> 2) & 0x01;

            blkb_byte1 = byte1;
            blkb_byte2 = byte2;

            V4L2_FM_DRV_DBG(V4L2_DBG_RX, "Block B - group=%d%c tp=%d",
            (byte1 >> 4) & 0x0f,
            ((byte1 >> 3) & 0x01) + 'A',
            rds_parser.tp);

            break;

        case V4L2_RDS_BLOCK_C: /* Block C */
        case V4L2_RDS_BLOCK_C_ALT: /* Block C' */
            if (rds_parser.rds_tuple[2] & BRCM_RDS_BIT_7) {
               /* invalid tuple */
               break;
            }

            blkc_byte1 = byte1;
            blkc_byte2 = byte2;
            break;

        case V4L2_RDS_BLOCK_D: /* Block D */
            /* When Block B is invalid (with skip_flag set),
             * there is no necessary info for deciding
             * how to interpret the content of Block D,
             * and thus having to skip this block as well
             */
            if (rds_parser.skip_flag || (rds_parser.rds_tuple[2] & BRCM_RDS_BIT_7)) {
               /* invalid/not interpretable tuple */
               rds_parser.skip_flag = 0;
               break;
            }

            /* Parsing the PI code, PI code will be present in all the Groups in Block-c*/
            group = (blkb_byte1 >> 3) & 0x1f;
            if (group & 0x01) {
                /* This is a message B, and there is a pi_code stored in Block C as well */
                rds_parser.pi_code = (blkc_byte1 << 8) | blkc_byte2;
            }

            /*
             * Event flags associated with field in previous blocks are
             * not marked until a valid block D is received
             */
            rds_parser.events |=
                RDS_EVENT_PI_CODE | RDS_EVENT_TP | RDS_EVENT_PTY;
            switch (group) {
                /*There are 32 Groups in total but we are only interested in the following Groups*/
                case 0: /* Group 0A */
                case 1: /* Group 0B */
                    rds_parser.ms_code = (blkb_byte2 >> 3) & 0x01;
                    rds_parser.ta = (blkb_byte2 >> 4) & 0x01;

                    rds_parser.events |= RDS_EVENT_TA | RDS_EVENT_MS;
                    offset = blkb_byte2 & 0x03;

                    rds_parser.ps[2*offset+0] = byte1;
                    rds_parser.ps[2*offset+1] = byte2;

                    /*
                     * Signalling the programm/station name is ready for retrieval every time a full update is completed
                     */

                    rds_parser.ps_markers |= 0x1 << offset;

                    if (rds_parser.ps_markers == 0x0F) {
                        rds_parser.ps_markers = 0;
                        rds_parser.events |= RDS_EVENT_PS;

                        V4L2_FM_DRV_DBG(V4L2_DBG_RX, "PSN: %s, PTY: %s, MS: %s\n",
                        rds_parser.ps,
                        (rds_parser.pty < sizeof(pty_str)/sizeof(pty_str[0]))
                        ? pty_str[rds_parser.pty] : "unknown",
                        rds_parser.ms_code?"Music":"Speech");
                    }
                    break;
                case 4: /* Group 2A */
                case 5: /* Group 2B */
                    offset = blkb_byte2 & 0x0F;
                    rt_AB = (blkb_byte2 >> 4) & 0x01;
                    if (rt_AB != rds_parser.rt_AB) {
                        memset(rds_parser.rt, 0, sizeof(rds_parser.rt));
                        rds_parser.rt_markers = 0;
                        rds_parser.rt_checkers = 0;
                        rds_parser.rt_AB = rt_AB;
                    }

                    if (group & 0x01) {
                        rds_parser.rt[2*offset+0] = byte1;
                        rds_parser.rt[2*offset+1] = byte2;
                    }
                    else {
                        rds_parser.rt[4*offset+0] = blkc_byte1;
                        rds_parser.rt[4*offset+1] = blkc_byte2;
                        rds_parser.rt[4*offset+2] = byte1;
                        rds_parser.rt[4*offset+3] = byte2;
                    }

                    /*
                     * Signalling the radio text is ready for retrieval every time a full update is completed
                     */
                    tmp1 = 0x1 << offset;
                    rds_parser.rt_markers |= tmp1;
                    if (tmp1 > rds_parser.rt_checkers) {
                        rds_parser.rt_checkers = (tmp1 << 1) - 1;
                    }

                    if (rds_parser.rt_markers == rds_parser.rt_checkers)
                    {
                        rds_parser.rt_markers = 0;
                        rds_parser.events |= RDS_EVENT_RT;

                        V4L2_FM_DRV_DBG(V4L2_DBG_RX, "Radio Text: %s", rds_parser.rt);
                    }
                    break;
                /* Parsing  CT information*/
                case 8: /*Group 4A*/
                    /* b14-b0@day = (b7-b0@bc_1 << 7) | b7-b1@bc_2 */
                    tmp1 = (__u32)((blkc_byte1 << 8) | blkc_byte2);
                    /* b16,b15@day = b1,b0@bb_2 */
                    tmp2 = (__u32)(blkb_byte2 & 0x03);
                    rds_parser.ct.day = (tmp2 << 15) | (tmp1 >> 1);
                    tmp1 = (__u32)(blkc_byte2 & 0x01);/* b4@hour = b0@bc_2 */
                    tmp2 = (__u32)(byte1 & 0xf0);/* b3-b0@hour = b7-b4@bd_1 */
                    rds_parser.ct.hour = (__u8)((tmp1 << 4) | (tmp2 >> 4));
                    /* b5-b0@minute = (b3-b0@bd_1 << 2) | b7-b6@bd_2 */
                    rds_parser.ct.minute = ((byte1 & 0x0f) << 2)|((byte2 >> 6)  & 0x03);
                    /* half hour offset sign bit = b5@bd_2 */
                    rds_parser.ct.hoffset_sign = (byte2 >> 5) & 0x01;
                    /* half hour offset magnitude  = b4-b0@bd_2 */
                    rds_parser.ct.hoffset_magnitude = byte2 & 0x1F;
                    rds_parser.events |= RDS_EVENT_CT;
                    V4L2_FM_DRV_DBG(V4L2_DBG_RX, "ct day: %u, hour: %hhu, minute: %hhu, hoffset_sign: %hhu, hoffset_magnitude: %hhu",
                           rds_parser.ct.day,
                           rds_parser.ct.hour,
                           rds_parser.ct.minute,
                           rds_parser.ct.hoffset_sign,
                           rds_parser.ct.hoffset_magnitude);
                    break;
                case 15:
                    rds_parser.ta = (blkb_byte2 >> 4) & 0x01;
                    rds_parser.events |= RDS_EVENT_TA;
                    break;
            }
            break;
        default:
            V4L2_FM_DRV_DBG(V4L2_DBG_RX, "Unhandled/Unknown block [%hhu]\n",rds_parser.rds_tuple[2]&0x07);
            break;
    }
}

/* Helper function to parse the RDS data
* in FM_REG_FM_RDS_DATA (0x80).
* Called locally by fmdrv_main.c
*/
static int parse_rds_data(struct fmdrv_ops *fmdev, struct sk_buff *skb)
{
    //unsigned long flags;
    unsigned char *rds_data;
    unsigned char type, block_index;
    tBRCM_RDS_QUALITY qlty_index;
    //int ret = 0, response_len, index=0;
    int response_len, index=0;
    void *err_ptr;
    bool rds_buffer_full = false;

    skb_pull(skb, FM_EVT_MSG_HDR_SIZE + FM_CMD_COMPLETE_HDR_SIZE);
    rds_data = skb->data;
    response_len = skb->len;

    V4L2_FM_DRV_DBG(V4L2_DBG_RX, "(rds) RDS length : %d", response_len);

    /* Read RDS data */
    while (response_len > 0)
    {
        /* Filling RDS circular buffer with data
         * in the format described in V4L2 specification
         */

        type = (rds_data[0] & BRCM_RDS_GRP_TYPE_MASK);
        block_index = (type >> 4);
        if (block_index < V4L2_RDS_BLOCK_A || block_index > V4L2_RDS_BLOCK_C_ALT)
        {
            V4L2_FM_DRV_ERR("(rds) Unrecognised block index: 0x%x", block_index);
            block_index = V4L2_RDS_BLOCK_INVALID;
        }

        qlty_index = (tBRCM_RDS_QUALITY)((rds_data[0] & BRCM_RDS_GRP_QLTY_MASK) >> 2);

        rds_parser.rds_tuple[2] = (block_index & V4L2_RDS_BLOCK_MSK);    /* Offset name */
        rds_parser.rds_tuple[2] |= (block_index & V4L2_RDS_BLOCK_MSK) << 3;  /* Received offset */


        switch(qlty_index)
        {
            case BRCM_RDS_NO_ERR:
                // V4L2_FM_DRV_DBG(V4L2_DBG_RX, "(rds) qlty : BRCM_RDS_NO_ERR");
                break;
            case BRCM_RDS_2BIT_ERR:
            case BRCM_RDS_3BIT_ERR:
                V4L2_FM_DRV_DBG(V4L2_DBG_RX, "(rds) qlty : %s", ((qlty_index==BRCM_RDS_2BIT_ERR)? \
                    "BRCM_RDS_2BIT_ERR":"BRCM_RDS_3BIT_ERR"));
                /* Set bit 6 to 1 and bit 7 to 0 indicate no error
                    but correction made*/
                rds_parser.rds_tuple[2] |= (BRCM_RDS_BIT_6);
                rds_parser.rds_tuple[2] &= ~(BRCM_RDS_BIT_7);
                break;

            case BRCM_RDS_UNRECOVER:
                V4L2_FM_DRV_DBG(V4L2_DBG_RX, "(rds) qlty : BRCM_RDS_UNRECOVER for data [ 0x%x 0x%x 0x%x]",\
                    rds_data[0], rds_data[1], rds_data[2]);
                /* Set bit 6 to 0 and bit 7 to 1 indicate error */
                rds_parser.rds_tuple[2] &= ~(BRCM_RDS_BIT_6);
                rds_parser.rds_tuple[2] |= (BRCM_RDS_BIT_7);
                break;
             default :
                V4L2_FM_DRV_ERR("(rds) Unknown quality code");
                rds_parser.rds_tuple[2] &= ~(BRCM_RDS_BIT_6);
                rds_parser.rds_tuple[2] |= (BRCM_RDS_BIT_7);
                break;
        }

        /* Store data byte. Swap bytes*/
        rds_parser.rds_tuple[0] = rds_data[2]; /* LSB of V4L2 spec block */
        rds_parser.rds_tuple[1] = rds_data[1]; /* MSB of V4L2 spec block */

        spin_lock(&fmdev->rx.rds.cbuff_lock);
        memcpy(&fmdev->rx.rds.cbuffer[fmdev->rx.rds.wr_index], rds_parser.rds_tuple, FM_RDS_TUPLE_LENGTH);
        fmdev->rx.rds.wr_index =
            (fmdev->rx.rds.wr_index + FM_RDS_TUPLE_LENGTH) % fmdev->rx.rds.buf_size;

        /* Checking for overflow */
        if ((rds_buffer_full = (fmdev->rx.rds.wr_index == fmdev->rx.rds.rd_index))) {
            fmdev->rx.rds.rd_index =
                (fmdev->rx.rds.rd_index + FM_RDS_TUPLE_LENGTH) % fmdev->rx.rds.buf_size;
        }
        spin_unlock(&fmdev->rx.rds.cbuff_lock);

        if (rds_buffer_full)
            V4L2_FM_DRV_DBG(V4L2_DBG_RX, "RDS cache buffer full; throwing away the oddest cache");

        /* Also parsing raw rds data internally
         * to provide easy access of it through ioctl commands
         */
        parse_rds_tuple();

        /*Check for end of RDS tuple */
        if ((rds_data + FM_RDS_TUPLE_LENGTH)[FM_RDS_TUPLE_BYTE1] == FM_RDS_END_TUPLE_1ST_BYTE &&
            (rds_data + FM_RDS_TUPLE_LENGTH)[FM_RDS_TUPLE_BYTE2] == FM_RDS_END_TUPLE_2ND_BYTE &&
            (rds_data + FM_RDS_TUPLE_LENGTH)[FM_RDS_TUPLE_BYTE3] == FM_RDS_END_TUPLE_3RD_BYTE )
        {
            pr_err("(fmdrv) End of RDS tuple reached @ %d index", index);
            break;
        }
        response_len -= FM_RDS_TUPLE_LENGTH;
        rds_data += FM_RDS_TUPLE_LENGTH;
        index += FM_RDS_TUPLE_LENGTH;
    }

     /* Set Tuner RDS capability bit as RDS data has been detected */
    fmdev->device_info.rxsubchans |= V4L2_TUNER_SUB_RDS;

    /* Wakeup read queue */
    if (fmdev->rx.rds.wr_index != fmdev->rx.rds.rd_index)
        wake_up_interruptible(&fmdev->rx.rds.read_queue);

    /*
     * Calling fm_rx_set_mask with fm_rds_mask to
     * raise the I2C_MASK_RDS_FIFO_WLINE_BIT again
     * and continue the rds data notification
     */
    fmdev->rx.fm_rds_mask |= I2C_MASK_RDS_FIFO_WLINE_BIT;
    fm_rx_set_mask(fmdev, fmdev->rx.fm_rds_mask);


    kfree(skb);
    return 0;
}

/*
 * Read the FM_REG_FM_RDS_FLAG by sending a read command.
 * Called locally by fmdrv_main.c
 */
static void send_read_intrp_cmd(struct fmdrv_ops *fmdev)
{
    unsigned char read_length;
    void *err_ptr;

    /* Asynchronous fm_rds interrupt events retrieval */
    read_length = FM_READ_2_BYTE_DATA;
    err_ptr = __fm_send_cmd(fmdev, FM_REG_FM_RDS_FLAG, &read_length,
                            sizeof(read_length), REG_RD,
                            false /* waiting_for_resp */);
    if(IS_ERR(err_ptr))
    {
        pr_err("(fmdrv) Error reading FM_REG_FM_RDS_FLAG");
    }
    V4L2_FM_DRV_DBG(V4L2_DBG_TX, "(fmdrv) Sent read to Interrupt flag FM_REG_FM_RDS_FLAG");
}

/* Initiate a read from RDS register. Called locally by fmdrv_main.c */
static int read_rds_data(struct fmdrv_ops *fmdev)
{
    unsigned char payload;
    void *err_ptr;

    //payload = FM_RDS_FIFO_MAX;
    payload = FM_RDS_UPD_TUPLE*3;

    V4L2_FM_DRV_DBG(V4L2_DBG_RX, "(fmdrv) Going to read RDS data from FM_REG_RDS_DATA!!");

    /* Asynchronous rds data retrieval */
    err_ptr = __fm_send_cmd(fmdev, FM_REG_RDS_DATA, &payload, 1, REG_RD, false /* waiting_for_resp */);
    if (IS_ERR(err_ptr))
        return PTR_ERR(err_ptr);
    else
        return 0;
}

/*
* Returns availability of RDS data in internel buffer.
* If data is present in RDS buffer, return 0. Else, return -EAGAIN.
* The V4L2 driver's poll() uses this method to determine RDS data availability.
*/
int fmc_is_rds_data_available(struct fmdrv_ops *fmdev, struct file *file, struct poll_table_struct *pts)
{
    poll_wait(file, &fmdev->rx.rds.read_queue, pts);

    if (fmdev->rx.rds.rd_index != fmdev->rx.rds.wr_index) {
        V4L2_FM_DRV_DBG(V4L2_DBG_RX, "(fmdrv) Poll success. RDS data is "\
            "available in buffer");
        return 0;
    }
    V4L2_FM_DRV_ERR("(fmdev) RDS Buffer is empty");
    return -EAGAIN;
}

/*
 * Function to copy RDS data from the FM ring buffer
 * to the userspace buffer.
 */
ssize_t fmc_transfer_rds_from_cbuff(struct fmdrv_ops *fmdev, struct file *file,
                    char __user * buf, size_t count)
{
    unsigned int block_count;
    //unsigned long flags;
    ssize_t ret;

    /* Block if no new data available */
    if (fmdev->rx.rds.wr_index == fmdev->rx.rds.rd_index) {
        if (file->f_flags & O_NONBLOCK)
            return -EWOULDBLOCK;

        ret = wait_event_interruptible(fmdev->rx.rds.read_queue,
                    (fmdev->rx.rds.wr_index != fmdev->rx.rds.rd_index));
        if (ret) {
            pr_err("(rds) %s Error : EINTR ", __func__);
            return -EINTR;
        }
    }
    /* Calculate block count from byte count */
    count /= 3;
    block_count = 0;
    ret = 0;


    /* Copy RDS blocks from the internal buffer and to user buffer */
    while (block_count < count) {
        bool is_empty = false;
        unsigned char tmp_buffer[FM_RDS_BLOCK_SIZE];

        spin_lock(&fmdev->rx.rds.cbuff_lock);
        if (!(is_empty = (fmdev->rx.rds.wr_index == fmdev->rx.rds.rd_index))) {
            memcpy(tmp_buffer,
                    fmdev->rx.rds.cbuffer+fmdev->rx.rds.rd_index,
                    FM_RDS_BLOCK_SIZE);
            /* Increment and wrap the read pointer */
            fmdev->rx.rds.rd_index = (fmdev->rx.rds.rd_index + FM_RDS_BLOCK_SIZE) % fmdev->rx.rds.buf_size;
        }
        spin_unlock(&fmdev->rx.rds.cbuff_lock);

        if (is_empty)
            break;

        /*
         * Always transfer complete RDS blocks;
         * note that copy_to_user() might sleep,
         * so it can not be called while holding a spin_lock
         */
        if (copy_to_user(buf, tmp_buffer, FM_RDS_BLOCK_SIZE))
            break;


        /* Increment counters */
        block_count++;
        buf += FM_RDS_BLOCK_SIZE;
        ret += FM_RDS_BLOCK_SIZE;
    }

    V4L2_FM_DRV_DBG(V4L2_DBG_RX, "(rds) %s Done copying %d", __func__, ret);

    return ret;
}

/* Sets the frequency */
int fmc_set_frequency(struct fmdrv_ops *fmdev, unsigned int freq_to_set)
{
    int ret;

    V4L2_FM_DRV_DBG(V4L2_DBG_TX, "In %s, frequency to set %d", __func__, freq_to_set);

    switch (fmdev->curr_fmmode) {
        case FM_MODE_RX:
            ret = fm_rx_set_frequency(fmdev, freq_to_set);
            if(ret != 0)
                V4L2_FM_DRV_ERR("Unable to set frequency. ret = %d", ret);
            break;

        case FM_MODE_TX:
            /* Currently FM TX is not supported */
            V4L2_FM_DRV_ERR("Currently FM TX is not supported");

        default:
            ret = -EINVAL;
    }
    return ret;
}

/* Returns the current tuned frequency */
int fmc_get_frequency(struct fmdrv_ops *fmdev, unsigned int *cur_tuned_frq)
{
    int ret = 0;

    switch (fmdev->curr_fmmode) {
        case FM_MODE_RX:
            ret = fm_rx_get_frequency(fmdev, cur_tuned_frq);
            break;

        case FM_MODE_TX:
            /* Currently FM TX is not supported */
            V4L2_FM_DRV_ERR("Currently FM TX is not supported");
            /* Deliberately falling through */

        default:
            ret = -EINVAL;
    }
    return ret;
}

/* Function to initiate SEEK operation */
int fmc_seek_station(struct fmdrv_ops *fmdev, unsigned char direction_upward,
                    unsigned char wrap_around)
{
    return fm_rx_seek_station(fmdev, direction_upward, wrap_around);
}

/* Returns current band index (0-Europe/US; 1-Japan) */
int fmc_get_band(struct fmdrv_ops *fmdev, unsigned char *band)
{
    *band = fmdev->rx.curr_band;
    return 0;
}

/* Set the world region */
int fmc_set_band(struct fmdrv_ops *fmdev, unsigned char region_to_set)
{
    int ret;

    switch (fmdev->curr_fmmode) {
        case FM_MODE_RX:
            if (region_to_set == fmdev->rx.curr_band) {
                V4L2_FM_DRV_DBG(V4L2_DBG_TX, "(fmdrv): Already region is set(%d)", region_to_set);
                ret = 0;
            } else if (region_to_set < FM_BAND_EUROPE
                        || region_to_set > FM_BAND_MAX) {
                ret = -EINVAL;
            } else {
                ret = fm_rx_set_band(fmdev, region_to_set);
            }
            break;

        case FM_MODE_TX:
            /* Currently FM TX is not supported */
            V4L2_FM_DRV_ERR("Currently FM TX is not supported");
            /* Deliberately falling through */

        default:
            ret = -EINVAL;
            break;
    }
    return ret;
}

/* Sets the audio mode */
int fmc_set_audio_mode(struct fmdrv_ops *fmdev, unsigned char audio_mode)
{
    int ret;

    switch (fmdev->curr_fmmode) {
        case FM_MODE_RX:
            ret = fm_rx_set_audio_mode(fmdev, audio_mode, fmdev->rx.curr_band);
            break;

        case FM_MODE_TX:
            /* Currently FM TX is not supported */
            V4L2_FM_DRV_ERR("Currently FM TX is not supported");
            /* Deliberately falling through */

        default:
            ret = -EINVAL;
    }
    return ret;
}

/* Gets the audio mode */
int fmc_get_audio_mode(struct fmdrv_ops *fmdev, unsigned char *audio_mode)
{
    int ret;

    switch (fmdev->curr_fmmode) {
        case FM_MODE_RX:
            ret = fm_rx_get_audio_mode(fmdev, audio_mode);
            break;

        case FM_MODE_TX:
            /* Currently FM TX is not supported */
            V4L2_FM_DRV_ERR("Currently FM TX is not supported");
            /* Deliberately falling through */

        default:
            ret = -EINVAL;
    }
    return ret;
}

/* Sets the scan step */
int fmc_set_scan_step(struct fmdrv_ops *fmdev, unsigned char scan_step)
{
    int ret;

    switch (fmdev->curr_fmmode) {
        case FM_MODE_RX:
            ret = fm_rx_set_scan_step(fmdev, scan_step);
            break;

        case FM_MODE_TX:
            /* Currently FM TX is not supported */
            V4L2_FM_DRV_ERR("Currently FM TX is not supported");
            /* Deliberately falling through */

        default:
            ret = -EINVAL;
    }
    return ret;
}

/*
 * Turn FM ON by sending FM_REG_RDS_SYS commmand
 */
int fmc_turn_fm_on (struct fmdrv_ops *fmdev, bool rds_enabled)
{
    int ret;
    unsigned char payload;

    if (rds_enabled)
        payload = (FM_ON | FM_RDS_ON);
    else
        payload = FM_ON;

    ret = fmc_send_cmd(fmdev, FM_REG_RDS_SYS, &payload, sizeof(payload),
            REG_WR, NULL, NULL);
    FM_CHECK_SEND_CMD_STATUS(ret);

    V4L2_FM_DRV_DBG(V4L2_DBG_TX, "(fmdrv): FM_REG_RDS_SYS write done");

    return ret;
}

/*
 * Turn off FM
 */
int fmc_turn_fm_off(struct fmdrv_ops *fmdev)
{
    int ret = -EINVAL;
    unsigned char payload;

    /* Mute audio */
    payload = FM_MUTE_ON;
    ret = fm_rx_set_mute_mode(fmdev, payload);

    FM_CHECK_SEND_CMD_STATUS(ret);

    if(ret < 0)
    {
        V4L2_FM_DRV_ERR("(fmdrv): FM mute off during FM Disable operation has failed");
        return ret;
    }

    /* Disable FM */
    payload = FM_OFF;

    ret = fmc_send_cmd(fmdev, FM_REG_RDS_SYS, &payload, sizeof(payload),
            REG_WR, NULL, NULL);
    FM_CHECK_SEND_CMD_STATUS(ret);

    return ret;
}

/*
 * Set FM Modes(TX, RX, OFF)
 * TX and RX modes are exclusive
 */
int fmc_set_mode(struct fmdrv_ops *fmdev, unsigned char fm_mode)
{
    int ret = 0;

    if (fm_mode >= FM_MODE_ENTRY_MAX) {
        pr_err("(fmdrv): Invalid FM mode : %d", fm_mode);
        ret = -EINVAL;
        return ret;
    }
    if (fmdev->curr_fmmode == fm_mode) {
        V4L2_FM_DRV_DBG(V4L2_DBG_TX, "(fmdrv): Already fm is in mode(%d)", fm_mode);
         return ret;
    }
    fmdev->curr_fmmode = fm_mode;
    return ret;
}

/*
 * Turn on FM, and other initialization to enable FM
 */
int fmc_enable (struct fmdrv_ops *fmdev, unsigned char fm_band)
{
    int ret;
    bool rds_enabled = false;

    unsigned short aud_ctrl;
    unsigned char read_length;
    unsigned char resp_buf [1];
    int resp_len;

    if (!test_bit(FM_CORE_READY, &fmdev->flag))
    {
        V4L2_FM_DRV_ERR("(fmdrv): FM core is not ready");
        return -EPERM;
    }

    fmc_set_mode (fmdev, FM_MODE_RX);


    /* turn FM ON */
    rds_enabled = (band_configs[fm_band].rds_support & (FM_RDS_BIT | FM_RBDS_BIT));

    ret = fmc_turn_fm_on(fmdev, rds_enabled);

    if (ret < 0)
    {
        pr_err ("(fmdrv): FM turn on failed");
        return ret;
    }
    /* wait for 300 ms before sending any more commands */
    mdelay (V4L2_FM_ENABLE_DELAY);

    if (ret < 0)
    {
        V4L2_FM_DRV_ERR("(fmdrv): set rds mode failed");
        return ret;
    }
    ret = fm_rx_set_band(fmdev, fm_band);

    if (ret < 0)
    {
        V4L2_FM_DRV_ERR("(fmdrv): set region has failed");
        return ret;
    }
    /* Read PCM Route settings */
    read_length = FM_READ_1_BYTE_DATA;
    ret = fmc_send_cmd(fmdev, FM_REG_PCM_ROUTE, &read_length, sizeof(read_length), REG_RD, &resp_buf, &resp_len);
    FM_CHECK_SEND_CMD_STATUS(ret);
    fmdev->rx.pcm_reg = resp_buf[0];
    V4L2_FM_DRV_DBG(V4L2_DBG_TX, "(fmdrv): pcm_reg value %d", fmdev->rx.pcm_reg);

    /* FM is enabled in the muted state by setting the bit FM_MANUAL_MUTE;
     * being initially muted can avoid possible burst noise at start
     */
    aud_ctrl = (unsigned short)(
                    FM_AUDIO_DAC_ON
                    | FM_RF_MUTE | FM_Z_MUTE_LEFT_OFF | FM_Z_MUTE_RITE_OFF
                    | FM_MANUAL_MUTE
                    | fmdev->rx.band_config.deemphasis);

    ret = fm_rx_set_audio_ctrl(fmdev, aud_ctrl);

    fmdev->rx.curr_rssi_threshold = DEF_V4L2_FM_SIGNAL_STRENGTH;

    /* Enable RDS */
    fm_rx_enable_rds(fmdev, rds_enabled);

    return ret;
}

/*
* Returns current FM mode (TX, RX, OFF) */
int fmc_get_mode(struct fmdrv_ops *fmdev, unsigned char *fmmode)
{
    if (!test_bit(FM_CORE_READY, &fmdev->flag)) {
        pr_err("(fmdrv): FM core is not ready");
        return -EPERM;
    }
    if (fmmode == NULL) {
        pr_err("(fmdrv): Invalid memory");
        return -ENOMEM;
    }

    *fmmode = fmdev->curr_fmmode;
    return 0;
}

static __u32 fm_extract_sync_id(struct sk_buff *skb)
{
    __u32 sync_id = 0, len_expected = 1;

    if (skb->len < len_expected)
        return sync_id;

    switch (skb->data[0]) {
        case HCI_COMMAND: {
            len_expected += sizeof(struct hci_command_hdr);
            if (skb->len < len_expected)
                break;

            struct hci_command_hdr *cmd_hdr = (struct hci_command_hdr *)(skb->data+1);
            __u16 opcode = __le16_to_cpu(cmd_hdr->opcode);
            switch (opcode) {
                case hci_opcode_pack(HCI_GRP_VENDOR_SPECIFIC, VSC_HCI_WRITE_PCM_PINS_OCF):
                    sync_id = opcode << 8;
                    break;
                case hci_opcode_pack(HCI_GRP_VENDOR_SPECIFIC, FM_2048_OP_CODE): {
                    len_expected = sizeof(struct fm_cmd_msg_hdr);
                    if (skb->len < len_expected)
                        break;

                    struct fm_cmd_msg_hdr *fm_cmd_hdr = (struct fm_cmd_msg_hdr *)skb->data;
                    sync_id = opcode << 8 | fm_cmd_hdr->fm_opcode;
                    break;
                }
                default:
                    break;
            }

            break;
        }
        case FM_PKT_LOGICAL_CHAN_NUMBER: {
            len_expected = sizeof(struct fm_event_msg_hdr);
            if (skb->len < len_expected)
                break;

            struct fm_event_msg_hdr * fm_evt_hdr = (struct fm_event_msg_hdr *)skb->data;
            if (fm_evt_hdr->event_id == HCI_EV_CMD_COMPLETE) {
                len_expected += sizeof(struct hci_ev_cmd_complete);
                if (skb->len < len_expected)
                    break;

                struct hci_ev_cmd_complete *cmd_complete_hdr = (struct hci_ev_cmd_complete *)(fm_evt_hdr+1);
                __u16 opcode = __le16_to_cpu(cmd_complete_hdr->opcode);

                switch (opcode) {

                    case hci_opcode_pack(HCI_GRP_VENDOR_SPECIFIC, VSC_HCI_WRITE_PCM_PINS_OCF):
                        sync_id = opcode << 8;
                        break;
                    case hci_opcode_pack(HCI_GRP_VENDOR_SPECIFIC, FM_2048_OP_CODE): {
                        len_expected += sizeof(struct fm_cmd_complete_hdr) - sizeof(struct hci_ev_cmd_complete);
                        if (skb->len < len_expected)
                            break;

                        struct fm_cmd_complete_hdr *fmcmd_complete_hdr = (struct fm_cmd_complete_hdr *)(fm_evt_hdr+1);
                        sync_id = opcode << 8 | fmcmd_complete_hdr->fm_opcode;
                        break;
                    }
                    default:
                        break;
                }
            }
            break;
        }
        default:
            break;
    }


    return sync_id;
}

/*
* Called by LDisc layer when FM packet is available. The pointer to
* this function is registered to LDisc during brcm_sh_ldisc_register() call.*/
static long fm_st_receive(struct sk_buff *skb)
{
    struct fmdrv_ops *fmdev = fm_comp_def.fmdev;
    struct fm_event_msg_hdr *fm_evt_hdr;
    bool deferred = false;

    if (skb == NULL) {
        V4L2_FM_DRV_ERR("(fmdrv): Invalid SKB received from LDisp");
        return -EFAULT;
    }
    if (skb->data[0] != FM_PKT_LOGICAL_CHAN_NUMBER) {
        V4L2_FM_DRV_ERR("(fmdrv): Received SKB (%p) is not FM Channel 8 pkt", skb);
        return -EINVAL;
    }

    if (skb->len < sizeof(struct fm_event_msg_hdr))
    {
        V4L2_FM_DRV_ERR("(fmdrv): skb(%p) has only %d bytes"
                        "atleast need %lu bytes to decode",
                                    skb, skb->len,
                            (unsigned long)sizeof(struct fm_event_msg_hdr));
        return -EINVAL;
    }


    fm_evt_hdr = (struct fm_event_msg_hdr *)skb->data;

#ifdef FM_DUMP_TXRX_PKT
    dump_rx_skb_data(skb);
#endif
    switch (fm_evt_hdr->event_id) {
        case HCI_EV_CMD_COMPLETE: {
            struct hci_ev_cmd_complete *cmd_complete_hdr = (struct hci_ev_cmd_complete *)(fm_evt_hdr+1);

            /* Only handling the opcode specific to FM */
            if (__le16_to_cpu(cmd_complete_hdr->opcode) == hci_opcode_pack(HCI_GRP_VENDOR_SPECIFIC, FM_2048_OP_CODE)) {
                deferred = true;
            }

            break;
        }
        /* Vendor specific Event */
        case BRCM_FM_VS_EVENT: {
            deferred = true;
            break;
        }
        default: {
            break;
        }
    }

    if (deferred) {
        spin_lock(&fmdev->rx_q.lock);
        __skb_queue_tail(&fmdev->rx_q, skb);
        spin_unlock(&fmdev->rx_q.lock);

        queue_work(fmdev->rx_wq, &fmdev->rx_work);
    } else {
        // skipping and removing this skb as being handled successfully
        kfree_skb(skb);
    }

    return 0;
}

/*
 * This function will be called from FM V4L2 open function.
 * Register with shared ldisc driver and initialize driver data.
 */
int fmc_prepare(struct fmdrv_ops *fmdev)
{
    //static struct component_interface comp_interface;
    int ret = 0;

    if (test_bit(FM_CORE_READY, &fmdev->flag)) {
        V4L2_FM_DRV_DBG(V4L2_DBG_OPEN, "(fmdrv): FM Core is already up");
        return ret;
    }

    fmdev->rx_wq = alloc_ordered_workqueue("fm_drv_rx", 0);
    if (!fmdev->rx_wq) {
        V4L2_FM_DRV_ERR("%s(): Unable to create workqueue fm_drv_rx\n", __func__);
        ret = -ENOMEM;
        goto out;
    }

    fmdev->rx.rds.buf_size = default_rds_buf * FM_RDS_TUPLE_LENGTH;
    /* Allocate memory for RDS ring buffer */
    fmdev->rx.rds.cbuffer = kzalloc(fmdev->rx.rds.buf_size, GFP_KERNEL);
    if (!fmdev->rx.rds.cbuffer) {
        V4L2_FM_DRV_ERR("Can't allocate rds ring buffer");
        ret = -ENOMEM;
        goto out;
    }


    fm_comp_def.fmdev = fmdev;
    fm_comp_def.interface.write = NULL;

    /* Register with the shared line discipline */
    ret = brcm_sh_ldisc_register(&fm_comp_def);
    if (ret < 0) {
        pr_err("(fmdrv): brcm_sh_ldisc_register failed %d", ret);
        ret = -EAGAIN;
        goto out;
    }
    else {
        V4L2_FM_DRV_DBG(V4L2_DBG_OPEN,"(fmdrv): fmc_prepare: brcm_sh_ldisc_register sucess %d", ret);
    }

    if (fm_comp_def.interface.write == NULL) {
        V4L2_FM_DRV_ERR("(fmdrv): Failed to get shared ldisc write func pointer");
        ret = brcm_sh_ldisc_unregister(COMPONENT_FM, 1);
        if (ret < 0)
            V4L2_FM_DRV_ERR("(fmdrv): brcm_sh_ldisc_unregister failed %d", ret);

        ret = -EAGAIN;
        goto out;
    }

    /* Initialize RX Queue and RX tasklet */
    skb_queue_head_init(&fmdev->rx_q);

    INIT_WORK(&fmdev->rx_work,fm_receive_data_ldisc);

    init_completion(&fmdev->seektask_completion);


    /* Do all the broadcom FM hardware specific initialization */
    fmdev->rx.curr_mute_mode = FM_MUTE_OFF;
    //fmdev->rx.rds.rds_flag = FM_RDS_DISABLE;
    fmdev->rx.curr_band = DEF_V4L2_FM_BAND;
    memcpy(&fmdev->rx.band_config, band_configs+fmdev->rx.curr_band,
                            sizeof(struct band_info));
    fmdev->rx.curr_freq = fmdev->rx.band_config.low_bound;
    fmdev->rx.rds_mode = FM_RDS_SYSTEM_NONE;
    fmdev->rx.curr_snr_threshold = FM_RX_SNR_MAX + 1;
    fmdev->rx.curr_sch_mode = FM_SCAN_NONE;
    fmdev->rx.curr_noise_floor = FM_NFE_DEFAILT;
    fmdev->rx.curr_volume = FM_RX_VOLUME_MAX;
    fmdev->rx.audio_mode = FM_AUTO_MODE;
    fmdev->rx.audio_path = FM_AUDIO_NONE;
    fmdev->device_info.capabilities = V4L2_CAP_HW_FREQ_SEEK | V4L2_CAP_TUNER |
                                V4L2_CAP_RADIO | V4L2_CAP_MODULATOR |
                                V4L2_CAP_AUDIO | V4L2_CAP_READWRITE | V4L2_CAP_RDS_CAPTURE;
    fmdev->device_info.type = V4L2_TUNER_RADIO;
    fmdev->device_info.rxsubchans = V4L2_TUNER_SUB_MONO | V4L2_TUNER_SUB_STEREO;
    fmdev->device_info.tuner_capability =V4L2_TUNER_CAP_STEREO | V4L2_TUNER_CAP_LOW | V4L2_TUNER_CAP_RDS;

    /* RDS initialization */
    reset_rds(fmdev);
    spin_lock_init(&fmdev->rx.rds.cbuff_lock);

    init_waitqueue_head(&fmdev->rx.rds.read_queue);

    set_bit(FM_CORE_READY, &fmdev->flag);

out:
    if (ret) {
        if (fmdev->rx_wq) {
            destroy_workqueue(fmdev->rx_wq);
            fmdev->rx_wq = NULL;
        }

        if (fmdev->rx.rds.cbuffer) {
            kfree(fmdev->rx.rds.cbuffer);
            fmdev->rx.rds.cbuffer = NULL;
        }
    }

    return ret;
}

/* This function will be called from FM V4L2 release function.
 * Unregister from line discipline driver.
 */
int fmc_release(struct fmdrv_ops *fmdev)
{
    int ret;
    V4L2_FM_DRV_DBG(V4L2_DBG_CLOSE, "(fmdrv) %s", __func__);

    if (!test_bit(FM_CORE_READY, &fmdev->flag)) {
        V4L2_FM_DRV_DBG(V4L2_DBG_CLOSE, "(fmdrv): FM Core is already down");
        return 0;
    }

    if (fmdev->rx_wq) {
        destroy_workqueue(fmdev->rx_wq);
        fmdev->rx_wq = NULL;
    }

    ret = brcm_sh_ldisc_unregister(COMPONENT_FM, 1);
    if (ret < 0)
        V4L2_FM_DRV_ERR("(fmdrv): Failed to de-register FM from HCI LDisc - %d", ret);
    else
        V4L2_FM_DRV_DBG(V4L2_DBG_CLOSE, "(fmdrv): Successfully unregistered from  HCI LDisc");

    reset_rds(fmdev);
    if (fmdev->rx.rds.cbuffer) {
        kfree(fmdev->rx.rds.cbuffer);
        fmdev->rx.rds.cbuffer = NULL;
    }

    /* Waking up pending reads */
    wake_up_interruptible_all(&fmdev->rx.rds.read_queue);


    spin_lock(&fmdev->rx_q.lock);
    __skb_queue_purge(&fmdev->rx_q);
    spin_unlock(&fmdev->rx_q.lock);

    fmdev->rx.curr_freq = 0;

    clear_bit(FM_CORE_READY, &fmdev->flag);
    return ret;
}

/* Module init function. Ask FM V4L module to register video device.
 * Allocate memory for FM driver context
 */
static int __init fm_drv_init(void)
{
    struct fmdrv_ops *fmdev = NULL;
    int ret = 0;

    pr_info("(fmdrv): FM driver version %s", FM_DRV_VERSION);

    fmdev = kzalloc(sizeof(struct fmdrv_ops), GFP_KERNEL);
    if (!fmdev) {
        V4L2_FM_DRV_ERR("(fmdrv): Can't allocate operation structure memory");
        ret = -ENOMEM;
        goto out;
    }

    ret = fm_v4l2_init_video_device(fmdev, radio_nr);
    if (ret < 0)
        goto out;

    fmdev->curr_fmmode = FM_MODE_OFF;

out:
    if (ret < 0) {

        if (fmdev)
            kfree(fmdev);
    }

    return ret;
}

/* Module exit function. Ask FM V4L module to unregister video device */
static void __exit fm_drv_exit(void)
{
    struct fmdrv_ops *fmdev = NULL;
    V4L2_FM_DRV_DBG(V4L2_DBG_INIT, "(fmdrv): fm_drv_exit");

    fmdev = fm_v4l2_deinit_video_device();
    if (fmdev != NULL) {
        kfree(fmdev);
    }
}

module_init(fm_drv_init);
module_exit(fm_drv_exit);

module_param(fm_dbg_param, int, S_IRUGO);
MODULE_PARM_DESC(fm_dbg_param, \
               "Set to integer value from 1 to 31 for enabling/disabling" \
               " specific categories of logs");


/* ------------- Module Info ------------- */
MODULE_AUTHOR("Satyajit Roy <roys@broadcom.com>, Syed Ibrahim Moosa <syedibrahim.moosa@broadcom.com>");
MODULE_DESCRIPTION("FM Driver for Connectivity chip of Broadcom Corporation");
MODULE_VERSION(VERSION); /* defined in makefile */
MODULE_LICENSE("GPL");
