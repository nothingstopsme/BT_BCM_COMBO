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


/******************************************************************************
*
*  Filename:      brcm_sh_ldisc.c
*
*  Description:   Broadcom HCI Shared line discipline driver implementation
*
*******************************************************************************/

#include <linux/module.h>

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/poll.h>
#include <linux/stat.h>
#include <linux/moduleparam.h>

#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/signal.h>
#include <linux/ioctl.h>
#include <linux/skbuff.h>
#include <net/sock.h>
#include <linux/netlink.h>
#include <linux/semaphore.h>
#include <linux/version.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "brcm_hci_uart.h"
#include "fm.h"
#include "ant.h"
#include "v4l2_logs.h"
#include "wait_ext.h"
#include "completion_ext.h"

#include <linux/dirent.h>
#include <linux/ctype.h>
#include <linux/tty.h>

#include <linux/time.h>


/*****************************************************************************
**  Constants & Macros for dynamic logging
*****************************************************************************/

/* set this module parameter to enable debug info */
int ldisc_dbg_param = 0;

#if V4L2_SNOOP_ENABLE
/* parameter to enable HCI snooping */
int ldisc_snoop_enable_param = 0;
#endif

/*******************************************************************************
**  Constants & Macros
************************************************************************************/

/*
 * ldisc states
 */
#define LDISC_OPENING   0
#define LDISC_CLOSING   1
#define LDISC_SUB_PROTO_REG_UNREG_IN_PROGRESS   2
#define LDISC_STARTING   3
#define LDISC_STOPPING   4
#define LDISC_OPENED   5

#define V4L2_HCI_PKT_MAX_SIZE 1050

/* Various waiting times defined in msec */

/* line discipline to be installed */
#define LDISC_TIME             1500
#define HCI_RESP_TIME          1000 // 800
/* time for workaround in msec to wait for closed tty */
#define TTY_CLOSE_TIME          20000

#define POR_RETRY_COUNT        5

/* install sysfs entry values */
#define V4L2_STATUS_ERR '2'  // error occured in BT application (HCI command timeout or HW error)
#define V4L2_STATUS_ON  '1'  // Atleast one procol driver is registered
#define V4L2_STATUS_OFF '0'  // No procol drivers registered

/* BT err flag values (bt_err) */
#define  V4L2_ERR_FLAG_RESET '0'
#define  V4L2_ERR_FLAG_SET   '1'


/** Bluetooth Address */
typedef struct {
    uint8_t address[6];
} __attribute__((packed))bt_bdaddr_t;


/* Baudrate threshold to change UART clock rate to 48 MHz */
#define CLOCK_SET_BAUDRATE 3000000

/* Max length for FW patchfile name */
#define FW_PATCH_FILENAME_MAXLEN 80

#define BD_ADDR_SIZE 18
#define LPM_PARAM_SIZE 100



/* struct to parse vendor params */
typedef int (conf_action_t)(char *p_conf_name, char *p_conf_value);

typedef struct {
    const char *conf_entry;
    conf_action_t *p_action;
    long param;
} conf_entry_t;

/*******************************************************************************
**  Forward declaration
*******************************************************************************/
static long brcm_sh_ldisc_stop(struct hci_uart *hu, bool btsleep_open);
static long brcm_sh_ldisc_start(struct hci_uart *hu);
static struct sk_buff *brcm_sh_ldisc_write(struct sk_buff *skb, bool never_blocking);

static int put_proto(struct hci_uart *hu, struct hci_uart_proto_desc *proto_desc);
static struct hci_uart_proto_desc *get_proto(struct hci_uart *hu, struct hci_uart_proto_desc *proto_desc);

static bool put_component_desc(struct hci_uart *hu, enum component_type comp_type);
static struct component_desc *get_component_desc(struct hci_uart *hu, enum component_type comp_type);
static int reset_component_desc(struct hci_uart *hu, struct component_interface *interface);

static __u32 brcm_sh_ldisc_extract_hcicmd_sync_id(struct sk_buff *skb);

/*******************************************************************************
**  Local type definitions
*******************************************************************************/
struct component_desc {
    __u32 curr_sync_id;
    struct sk_buff *curr_sync_response;

    wait_queue_head_t sync_waiter;

    __u32 ref_count;
    struct component_interface *interface;
};

struct hci_uart_proto_desc {
    struct hci_uart_proto *proto;

    spinlock_t ref_count_lock;
    unsigned int ref_count;
};

static int is_print_reg_error = 1;
static unsigned long jiffi1, jiffi2;

/* setting custom baudrate received from UIM */
struct ktermios ktermios;
static unsigned char bd_addr_array[6] = {0};
char bd_addr[BD_ADDR_SIZE] = {0,};
char fw_name[FW_PATCH_FILENAME_MAXLEN];

#if V4L2_SNOOP_ENABLE
/* enable/disable HCI snoop logging */
char snoop_enable[2] = {0, 0};
#endif

/* module parameters */
static char lpm_param[LPM_PARAM_SIZE];
static long int custom_baudrate = 0;
static int patchram_settlement_delay = 0;
static int ControllerAddrRead = 0;
static int LpmUseBluesleep = 0;
static enum sleep_type sleep = SLEEP_DEFAULT;

#if V4L2_SNOOP_ENABLE
/* HCI snoop and netlink socket related variables */
/* Variables for netlink sockets for hcisnoop */
#define NETLINK_USER 29

struct sock *nl_sk_hcisnoop = NULL;
static int hcisnoop_client_pid = 0;
#endif

/* HIC commands emitted by libbt which shall be ignored. */
#define IGNORE_CMD_SIZE 14
static const __u16 IGNORE_CMDS[IGNORE_CMD_SIZE] =
                                      {0x0c03, 0xfc45, 0xfc18, 0x0c14, 0xfc2e,
                                       0xfc01, 0xfc27, 0xfc1c, 0xfc1e, 0xfc6d,
                                       0xfc7e, 0xfc4e, 0x0c33, 0xfc4c};
#define READ_CMD_DATA_LEN 33
#define MAX_HCI_EVENT_LEN 37 // (+ 4 bytes header)

//static bool hci_filter_enabled = false;

/*******************************************************************************
**  Function forward-declarations and Function callback declarations
*******************************************************************************/

static struct hci_uart_proto *hup[HCI_UART_MAX_PROTO];

//#define MAX_BRCM_DEVICES    3   /* Imagine 1 on each UART for now */

static struct platform_device *brcm_plt_device;
#define HU_REF ((struct hci_uart *)dev_get_drvdata(&brcm_plt_device->dev))

/**
  * internal functions to read chip name and
 * download patchram file
 */
static long download_patchram(struct hci_uart*);
static int brcm_ldisc_remove(struct platform_device *pdev);

/********************************************************************/


/*****************************************************************************
**  Type definitions
*****************************************************************************/

static struct component_interface init_interface = {
	.type = COMPONENT_BT,
  .extract_sync_id = brcm_sh_ldisc_extract_hcicmd_sync_id
};

/* parsing functions */
int parse_custom_baudrate(char *p_conf_name, char *p_conf_value)
{
    pr_info("%s = %s\n", p_conf_name, p_conf_value);
    sscanf(p_conf_value, "%ld", &custom_baudrate);
    return 0;
}

int parse_patchram_settlement_delay(char *p_conf_name, char *p_conf_value)
{
    pr_info("%s = %s\n", p_conf_name, p_conf_value);
    sscanf(p_conf_value, "%d", &patchram_settlement_delay);
    return 0;
}

int parse_LpmUseBluesleep(char *p_conf_name, char *p_conf_value)
{
    pr_info("%s = %s\n", p_conf_name, p_conf_value);
    sscanf(p_conf_value, "%d", &LpmUseBluesleep);
    if( LpmUseBluesleep )
        sleep = SLEEP_BLUESLEEP;
    BRCM_LDISC_DBG(V4L2_DBG_INIT, "%s sleep %d", __func__,sleep);
    return 0;
}

int parse_ControllerAddrRead(char *p_conf_name, char *p_conf_value)
{
    pr_info("%s = %s\n", p_conf_name, p_conf_value);
    sscanf(p_conf_value, "%d", &ControllerAddrRead);
    return 0;
}

#if V4L2_SNOOP_ENABLE
int parse_ldisc_snoop_enable_param(char *p_conf_name, char *p_conf_value)
{
    pr_info("%s = %s\n", p_conf_name, p_conf_value);
    sscanf(p_conf_value, "%d", &ldisc_snoop_enable_param);
    return 0;
}
#endif

int parse_lpm_param(char *p_conf_name, char *p_conf_value)
{
    pr_info("%s = %s strlen = %d\n", p_conf_name, p_conf_value, strlen(p_conf_value));
    memset(lpm_param, 0, LPM_PARAM_SIZE);
    strncpy(lpm_param, p_conf_value, strlen(p_conf_value));
    pr_info("parse_lpm_param = %s\n", lpm_param);
    return 0;
}

int dbg_ldisc_drv(char *p_conf_name, char *p_conf_value)
{
    pr_info("%s = %s\n", p_conf_name, p_conf_value);
    sscanf(p_conf_value, "%d", &ldisc_dbg_param);
    return 0;
}

int dbg_bt_drv(char *p_conf_name, char *p_conf_value)
{
    pr_info("%s = %s\n", p_conf_name, p_conf_value);
    return 0;
}

int dbg_fm_drv(char *p_conf_name, char *p_conf_value)
{
    pr_info("%s = %s\n", p_conf_name, p_conf_value);
    return 0;
}

static const conf_entry_t conf_table[] = {
    {"custom_baudrate", parse_custom_baudrate, 0},
    {"patchram_settlement_delay", parse_patchram_settlement_delay, 0},
    {"LpmUseBluesleep", parse_LpmUseBluesleep, 0},
    {"ControllerAddrRead", parse_ControllerAddrRead, 0},
#if V4L2_SNOOP_ENABLE
    {"ldisc_snoop_enable_param", parse_ldisc_snoop_enable_param, 0},
#endif
    {"lpm_param" , parse_lpm_param, 0},
    {"ldisc_dbg_param",dbg_ldisc_drv},
    {"bt_dbg_param",dbg_bt_drv},
    {"fm_dbg_param",dbg_fm_drv},
    {(const char *) NULL, NULL, 0}
};


#if V4L2_SNOOP_ENABLE
/* Function callback for netlink socket */
static void brcm_hcisnoop_recv_msg(struct sk_buff *skb)
{
    struct nlmsghdr *nlh;

    if(skb != NULL)
    {
        nlh = (struct nlmsghdr *)skb->data;
        BRCM_LDISC_DBG(V4L2_DBG_TX,"hcisnoop: received read command from hcisnoop client: %s",\
                                                       (char *)nlmsg_data(nlh));
        hcisnoop_client_pid = nlh->nlmsg_pid; /*pid of sending process */
        BRCM_LDISC_DBG(V4L2_DBG_TX,"hcisnoop: accepting request from pid=%d",\
                                                           hcisnoop_client_pid);
    }
    else {
        BRCM_LDISC_ERR("skb is NULL");
    }

    return;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)
struct netlink_kernel_cfg cfg = {
    .input = brcm_hcisnoop_recv_msg,
};
#endif

static int enable_snoop(void)
{
    int err;
    if(ldisc_snoop_enable_param)
    {
        /* check whether snoop is enabled */
        BRCM_LDISC_DBG(V4L2_DBG_TX, "ldisc_snoop_enable_param = %d",\
            ldisc_snoop_enable_param);

        /* close previously created socket */
        if(nl_sk_hcisnoop)
        {
            netlink_kernel_release(nl_sk_hcisnoop);
            nl_sk_hcisnoop = NULL;
        }
        /* start hci snoop to hcidump */
        /* Create socket for hcisnoop */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,6,0)
        nl_sk_hcisnoop = netlink_kernel_create(&init_net, NETLINK_USER, 0,
                    brcm_hcisnoop_recv_msg, NULL, THIS_MODULE);
#else
        nl_sk_hcisnoop = netlink_kernel_create(&init_net, NETLINK_USER, &cfg);
#endif
        if (!nl_sk_hcisnoop)
        {
            BRCM_LDISC_ERR("Error creating netlink socket for HCI snoop");
            err = -10;
            return err;
        }
        return 0;
    }
    else {
        return 0;
    }
}
#endif

/* parse and load vendor params */
static int parse_vendor_params(void)
{
    struct hci_uart *hu = HU_REF;
    conf_entry_t *p_entry;
    char *p;
    char *p_name, *p_value;

    p = hu->vendor_params;
    pr_info("parse_vendor_params = %s", hu->vendor_params);
    while ((p_name = strsep(&p, " ")))
    {
        p_entry = (conf_entry_t *)conf_table;
        while (p_entry->conf_entry != NULL)
        {
            if (strncmp(p_entry->conf_entry, (const char *)p_name, \
                strlen(p_entry->conf_entry)) == 0)
            {
                p_value = strsep(&p_name, "=");
                p_entry->p_action(p_value, p_name);
                break;
            }
            p_entry++;
        }
    }
    return 0;
}

static ssize_t show_install(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct hci_uart *hu = dev_get_drvdata(dev);
    memcpy(buf, &hu->ldisc_install, sizeof(unsigned char));
    return 1;
}

static ssize_t store_install(struct device *dev,
        struct device_attribute *attr, char *buf,size_t size)
{
    struct hci_uart *hu = HU_REF;

    // Avoid race condition if all app (BT/FM/Ant) simultaneously try to write to install entry
    spin_lock(&hu->err_lock);

    /* When HCI command timeout or hardware error event occurs in an application,
    * BT/FM/Ant apps should set a value 0x02 to "install sysfs" entry. Ldisc will set
    * "bt_err" and "fm_err" for error indication. All applications which use V4L2 solution
    * should now restart and reset bt_err and fm_err after recovery.
    * E.g: If fm_err is already set, FM app concludes that BT app has already reported an error.
    * FM app only needs to restart.
    */
    if ((hu->ldisc_install == V4L2_STATUS_ON) && (buf[0] == V4L2_STATUS_ERR) \
        && ((hu->ldisc_bt_err == V4L2_ERR_FLAG_RESET) \
        || (hu->ldisc_fm_err == V4L2_ERR_FLAG_RESET)))
    {
        hu->ldisc_install = V4L2_STATUS_ERR;
        hu->ldisc_bt_err = V4L2_ERR_FLAG_SET;
        hu->ldisc_fm_err = V4L2_ERR_FLAG_SET;
        sysfs_notify(&hu->brcm_pdev->dev.kobj, NULL, "install");
        sysfs_notify(&hu->brcm_pdev->dev.kobj, NULL, "bt_err");
        sysfs_notify(&hu->brcm_pdev->dev.kobj, NULL, "fm_err");
        BRCM_LDISC_ERR("Error!: Userspace app has reported error. install = %c",\
            buf[0]);
    }
    spin_unlock(&hu->err_lock);
    return size;
}

static ssize_t show_bt_err(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct hci_uart *hu = dev_get_drvdata(dev);
    memcpy(buf, &hu->ldisc_bt_err, sizeof(unsigned char));
    return 1;
}

static ssize_t store_bt_err(struct device *dev,
        struct device_attribute *attr, char *buf,size_t size)
{
    struct hci_uart *hu = HU_REF;
    hu->ldisc_bt_err = buf[0];
    return size;
}

static ssize_t show_fm_err(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct hci_uart *hu = dev_get_drvdata(dev);
    memcpy(buf, &hu->ldisc_fm_err, sizeof(unsigned char));
    return 1;
}

static ssize_t store_fm_err(struct device *dev,
        struct device_attribute *attr, char *buf,size_t size)
{
    struct hci_uart *hu = HU_REF;
    hu->ldisc_fm_err = buf[0];

    return size;
}

static ssize_t show_vendor_params(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct hci_uart *hu = dev_get_drvdata(dev);
    memcpy(buf, hu->vendor_params, VENDOR_PARAMS_LEN);
    return VENDOR_PARAMS_LEN;
}

static ssize_t store_vendor_params(struct device *dev,
        struct device_attribute *attr, char *buf,size_t size)
{
    struct hci_uart *hu = HU_REF;
    memcpy(hu->vendor_params, buf, VENDOR_PARAMS_LEN);
    parse_vendor_params();

#if V4L2_SNOOP_ENABLE
    // enable/disable snoop
    if(enable_snoop()!=0)
    {
        BRCM_LDISC_ERR("Unable to enable HCI snoop in ldisc");
    }
#endif

    return size;
}

static ssize_t store_bdaddr(struct device *dev,
        struct device_attribute *attr, char *buf,size_t size)
{
    sprintf(bd_addr, "%s\n", buf);

    pr_info("store_bdaddr  %s  size %d",bd_addr,size);
    return size;
}

/* UIM will read firmware patch filename.
    From one of the following
    1. "FwPatchFileName" entry is present in bt_vendor.conf
                                 OR
    2. Read chip name through HCI command.and search through the directory
        to find the exact match.
    UIM passes the filename to ldisc as sysfs entry */
static ssize_t store_fw_patchfile(struct device *dev,
        struct device_attribute *attr, char *buf,size_t size)
{
    sprintf(fw_name, "%s",buf);
    BRCM_LDISC_DBG(V4L2_DBG_INIT,"store_fw_patchfile  %s size %d ",fw_name,size);
    return size;
}

#if V4L2_SNOOP_ENABLE
static ssize_t show_snoop_enable(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%s", snoop_enable);
}

static ssize_t store_snoop_enable(struct device *dev,
        struct device_attribute *attr, char *buf,size_t size)
{
    memcpy(&snoop_enable, buf, 1);
    if(!ldisc_snoop_enable_param) {
        // enable HCI snoop in line discipline driver
        if(!strcmp(snoop_enable,"1")) {
            if(nl_sk_hcisnoop)
            {
                netlink_kernel_release(nl_sk_hcisnoop);
                nl_sk_hcisnoop = NULL;
            }
            /* start hci snoop to hcidump */
            /* Create socket for hcisnoop */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,6,0)
            nl_sk_hcisnoop = netlink_kernel_create(&init_net, NETLINK_USER, 0,
                        brcm_hcisnoop_recv_msg, NULL, THIS_MODULE);
#else
            nl_sk_hcisnoop = netlink_kernel_create(&init_net, NETLINK_USER, &cfg);
#endif
            if (!nl_sk_hcisnoop)
            {
                BRCM_LDISC_ERR("Error creating netlink socket for HCI snoop");
            }
            BRCM_LDISC_DBG(V4L2_DBG_TX, "enabling HCI snoop");
        }
        else {
            /* stop hci snoop to hcidump */
            if(nl_sk_hcisnoop)
                netlink_kernel_release(nl_sk_hcisnoop);
            nl_sk_hcisnoop = NULL;
            BRCM_LDISC_DBG(V4L2_DBG_TX, "disabling HCI snoop");
        }
    }

    return size;
}
#endif

/* structures specific for sysfs entries */
static struct kobj_attribute ldisc_bdaddr =
__ATTR(bdaddr, S_IRUGO | S_IWUSR | S_IWGRP, NULL,(void *)store_bdaddr);

/* structures specific for sysfs entries */
static struct kobj_attribute ldisc_install =
__ATTR(install, S_IRUGO | S_IWUSR | S_IWGRP, (void *)show_install, (void *)store_install);

/* structures specific for sysfs entries */
static struct kobj_attribute ldisc_vendor_params =
__ATTR(vendor_params, S_IRUGO | S_IWUSR | S_IWGRP, (void *)show_vendor_params, (void *)store_vendor_params);

/* structures specific for sysfs entries */
static struct kobj_attribute ldisc_bt_err =
__ATTR(bt_err, S_IRUGO | S_IWUSR | S_IWGRP, (void *)show_bt_err, (void *)store_bt_err);

/* structures specific for sysfs entries */
static struct kobj_attribute ldisc_fm_err =
__ATTR(fm_err, S_IRUGO | S_IWUSR | S_IWGRP, (void *)show_fm_err, (void *)store_fm_err);

/* structures specific for sysfs entries */
static struct kobj_attribute ldisc_fw_patchfile =
__ATTR(fw_patchfile, S_IRUGO | S_IWUSR | S_IWGRP, NULL, (void *)store_fw_patchfile);

#if V4L2_SNOOP_ENABLE
/* structures specific for sysfs entries */
static struct kobj_attribute ldisc_snoop_enable =
__ATTR(snoop_enable, S_IRUGO | S_IWUSR | S_IWGRP, (void *)show_snoop_enable, (void *)store_snoop_enable);
#endif

static struct attribute *uim_attrs[] = {
    &ldisc_install.attr,
    &ldisc_bdaddr.attr,
    &ldisc_fw_patchfile.attr,
#if V4L2_SNOOP_ENABLE
    &ldisc_snoop_enable.attr,
#endif
    &ldisc_vendor_params.attr,
    &ldisc_bt_err.attr,
    &ldisc_fm_err.attr,
    NULL,
};

static struct attribute_group uim_attr_grp = {
    .attrs = uim_attrs,
};





/*******************************************************************************
**  Functions
*******************************************************************************/

/*******************************************************************************
**
** Function - brcm_hci_tty_op_write()
**
** Description - Writing data bytes to tty
**
** Returns - 0 if success; a negative errno otherwise
**
*******************************************************************************/
static int brcm_hci_tty_op_write(struct hci_uart *hu, const unsigned char* data,
                                                             unsigned int count)
{
    int written_len = 0;
    int remaining_len = count;


    while (remaining_len > 0) {
        written_len = hu->tty->ops->write(hu->tty, data, remaining_len);
        if (written_len < 0) {
            return -EIO;
        }

        data += written_len;
        remaining_len -= written_len;
    }

    if (remaining_len < 0) {
        return -EIO;
    } else {
        return count - remaining_len;
    }
}

/*******************************************************************************
**
** Function - brcm_hci_queue_tx()
**
** Description - Queuing a skb for tx transmission
**
** Returns - when in blocking mode
**             A sk_buff pointer to the response if success; an error ptr otherwise
**         - when not in blocking mode
**             NULL if success; an error ptr otherwise
**
*******************************************************************************/
static struct sk_buff *brcm_hci_queue_tx(struct hci_uart *hu, struct sk_buff *skb, bool never_blocking)
{
    bool wait_for_response = false;
    struct hci_uart_proto_desc *proto_desc = NULL;
    struct component_desc *comp_desc = NULL;
    int ret = 0, err;
    struct sk_buff *response = NULL;
    __u32 sync_id = 0;

    if (unlikely(!hu))
        return ERR_PTR(-EFAULT);

    proto_desc = get_proto(hu, NULL);
    if (unlikely(IS_ERR_OR_NULL(proto_desc))) {
        return ERR_PTR(-EFAULT);
    }

    if (!never_blocking) {
        comp_desc = get_component_desc(hu, sh_ldisc_cb(skb)->comp_type);

        if (comp_desc && comp_desc->interface->extract_sync_id)
            sync_id = comp_desc->interface->extract_sync_id(skb);


        if (sync_id) {
            wait_for_response = true;
            spin_lock(&comp_desc->sync_waiter.lock);
            comp_desc->curr_sync_id = sync_id;
            comp_desc->curr_sync_response = NULL;
            spin_unlock(&comp_desc->sync_waiter.lock);
        } else {
            BRCM_LDISC_DBG(V4L2_DBG_TX, "Failed to extract a valid sync_id; no response synchronisation");
        }
    }

    /* Note: enqueue() will always take over the ownership of skb */
    err = proto_desc->proto->enqueue(hu, skb);
    put_proto(hu, proto_desc);
    if (err) {

        BRCM_LDISC_ERR("An error occured while doing enqueue(): err = %d", err);
        goto out;
    }

    queue_work(hu->tx_wq, &hu->tx_work);

    if (wait_for_response) {
        unsigned long completed = 0;
        /* comp_desc can not be null in this case */

        spin_lock(&comp_desc->sync_waiter.lock);
        completed = wait_event_locked_timeout(
                comp_desc->sync_waiter,
                comp_desc->curr_sync_response,
                msecs_to_jiffies(HCI_RESP_TIME));
        response = comp_desc->curr_sync_response;
        comp_desc->curr_sync_response = NULL;
        comp_desc->curr_sync_id = 0;
        spin_unlock(&comp_desc->sync_waiter.lock);

        wait_for_response = false;

        if (!completed) {
            BRCM_LDISC_ERR("Timeout while waiting for a response; continuing");
        }
    }

out:
    if (comp_desc) {

        if (wait_for_response) {
            spin_lock(&comp_desc->sync_waiter.lock);
            comp_desc->curr_sync_response = NULL;
            comp_desc->curr_sync_id = 0;
            spin_unlock(&comp_desc->sync_waiter.lock);
        }

        put_component_desc(hu, comp_desc->interface->type);
    }

    return response;
}

/*******************************************************************************
**
** Function - brcm_hci_snoop_send()
**
** Description - Sending snooping packets
**
** Returns - 0 if success; a negative errno otherwise
**
*******************************************************************************/
#if V4L2_SNOOP_ENABLE
int brcm_hci_snoop_send(struct hci_uart *hu, struct hci_snoop_hdr *snoop_hdr, const unsigned char* data, unsigned int log_flag)
{
    int err=0;
    unsigned long flags;
		struct nlmsghdr *nlh;
		struct sk_buff *hcisnoop_skb_out;
		void *msg_ptr;
		unsigned int count = sizeof(struct hci_snoop_hdr) + snoop_hdr->len;

    BRCM_LDISC_DBG(log_flag , "brcm_hci_snoop");
    /* Snoop and send data to hcidump */
    if(hcisnoop_client_pid > 0)
    {
        spin_lock_irqsave(&hu->hcisnoop_lock, flags);
        if (nl_sk_hcisnoop) {

            BRCM_LDISC_DBG(log_flag, "Routing packet to hcisnoop"\
                " client pid=%d count=%d", hcisnoop_client_pid, count);
            /* Also route packet to hcisnoop client */
            hcisnoop_skb_out = nlmsg_new(count, 0);

            if (!hcisnoop_skb_out)
            {
                BRCM_LDISC_ERR("hcisnoop: Failed to allocate new skb");
                spin_unlock_irqrestore(&hu->hcisnoop_lock, flags);
                return -1;
            }
            nlh = nlmsg_put(hcisnoop_skb_out, 0, 0, NLMSG_DONE, count, 0);
            NETLINK_CB(hcisnoop_skb_out).dst_group = 0; /* not in mcast group */
                    msg_ptr = nlmsg_data(nlh);
            memcpy(msg_ptr, snoop_hdr, sizeof(struct hci_snoop_hdr));
            memcpy(msg_ptr+sizeof(struct hci_snoop_hdr), data, snoop_hdr->len);

            if ((err = nlmsg_unicast(nl_sk_hcisnoop, hcisnoop_skb_out,
                    hcisnoop_client_pid)) < 0)
            {
                BRCM_LDISC_ERR("Error errcode=%d while sending packet to pid = %d",
                    err, hcisnoop_client_pid);
                hcisnoop_client_pid = 0;
            }
            else
                BRCM_LDISC_DBG(log_flag, "Forwarded hci packet"\
                                                         "to hcisnoop client");
        }
        spin_unlock_irqrestore(&hu->hcisnoop_lock, flags);
    }

    return 0;
}
#endif

/*******************************************************************************
**
** Function - brcm_hci_uart_register_proto()
**
** Description - Used by the lower layer to register UART protocol
**             which can be H4, LL, BCSP etc.
**             This should only be called during initialisation where
**             registration of various protos is performed sequentially
**
** Returns - 0 if success; errno otherwise
**
*******************************************************************************/
int brcm_hci_uart_register_proto(struct hci_uart_proto *p)
{
    struct hci_uart_proto_desc *proto_desc;

    if (p->id >= HCI_UART_MAX_PROTO)
        return -EINVAL;

    if (hup[p->id])
        return -EEXIST;

    proto_desc = kzalloc(sizeof(struct hci_uart_proto_desc), GFP_KERNEL);

    if (!proto_desc)
        return -ENOMEM;

    proto_desc->proto = p;
    spin_lock_init(&proto_desc->ref_count_lock);

    hup[p->id] = proto_desc;

    BRCM_LDISC_DBG(V4L2_DBG_INIT, "%p", p);

    return 0;
}

/*******************************************************************************
**
** Function - brcm_hci_uart_unregister_proto()
**
** Description - Used by the lower layer to unregister UART protocol
**             which can be H4, LL, BCSP etc.
**             This should only be called during deinitialisation
**             where unregistration of various protos is performed
**             sequentially
**
** Returns - 0 if success; errno otherwise
**
*******************************************************************************/
int brcm_hci_uart_unregister_proto(struct hci_uart_proto *p)
{
    struct hci_uart_proto_desc *proto_desc;

    if (p->id >= HCI_UART_MAX_PROTO)
        return -EINVAL;

    proto_desc = hup[p->id];
    if (!proto_desc)
        return -EINVAL;
    else {
        if (proto_desc->proto != p)
            return -EINVAL;

        kfree(proto_desc);
        hup[p->id] = NULL;
    }

    return 0;
}

/*******************************************************************************
**
** Function - brcm_hci_uart_route_frame()
**
** Description - push the skb received to relevant upper layer
**             protocol stacks, which could be BT, FM or GPS etc
**
** Returns - 0 if success; errno otherwise
**
*******************************************************************************/
void brcm_hci_uart_route_frame(struct hci_uart *hu, enum component_type comp_type, struct sk_buff *skb)
{
    struct component_desc *comp_desc = NULL;
    __u32 sync_id = 0;

    BRCM_LDISC_DBG(V4L2_DBG_RX, "comp_type = %d", comp_type);

    if (unlikely(!hu || !skb)) {
        BRCM_LDISC_ERR("hci_uart or skb ptr is null");
        goto out;
    }


    comp_desc = get_component_desc(hu, comp_type);

    if (likely(comp_desc)) {
        if (comp_desc->interface->extract_sync_id) {
            sync_id = comp_desc->interface->extract_sync_id(skb);
            if (sync_id) {
                spin_lock(&comp_desc->sync_waiter.lock);
                if (comp_desc->curr_sync_id == sync_id) {
                    comp_desc->curr_sync_response = skb_get(skb);
                    comp_desc->curr_sync_id = 0;
                    wake_up_locked(&comp_desc->sync_waiter);
                }

                spin_unlock(&comp_desc->sync_waiter.lock);

            }
        }


        if (likely(comp_desc->interface->recv)) {
            long ret = comp_desc->interface->recv(skb);
            if (unlikely(ret)) {
                BRCM_LDISC_ERR(" component %d's recv() failed with ret = %d", comp_desc->interface->type, ret);
            } else
                skb = NULL;

        } else {
            BRCM_LDISC_DBG(V4L2_DBG_RX, "component %d does not provide recv() for handling its rx data; skb dropped", comp_desc->interface->type);
        }

        put_component_desc(hu, comp_desc->interface->type);
    } else {
        BRCM_LDISC_DBG(V4L2_DBG_RX, "component %d does not register to this ldisc; skb dropped", comp_type);
    }

out:
    if (skb)
        kfree_skb(skb);

    return;
}



/*******************************************************************************
**
** Function - put_proto()
**
** Description - Decreasing by 1 the reference count of the designated
**             UART Protocol, which can be H4, LL, BCSP etc,
**             and calling its close() when the ref_count drops to 0
**
** Returns - 0 if successful, a negative errno otherwise
**
*******************************************************************************/
static int put_proto(struct hci_uart *hu, struct hci_uart_proto_desc *proto_desc)
{
    int err = 0;

    if (!proto_desc) {
        proto_desc = hu->curr_proto_desc;

        if (!proto_desc)
            return -EINVAL;
    }

    spin_lock(&proto_desc->ref_count_lock);
    if (proto_desc->ref_count == 1) {
        proto_desc->ref_count = 0;
        err = proto_desc->proto->close(hu);
    } else if (proto_desc->ref_count > 1){
        proto_desc->ref_count -= 1;
    }
    spin_unlock(&proto_desc->ref_count_lock);


    return err;
}

/*******************************************************************************
**
** Function - get_proto()
**
** Description - Increasing by 1 the reference count of the designated
**             UART Protocol, which can be H4, LL, BCSP etc,
**             and calling its open() when ref_count changes from 0 to 1
**
** Returns - Pointer to struct hci_uart_proto
**
*******************************************************************************/
static struct hci_uart_proto_desc *get_proto(struct hci_uart *hu, struct hci_uart_proto_desc *proto_desc)
{
    int err = 0;

    if (!proto_desc) {
        proto_desc = hu->curr_proto_desc;

        if (!proto_desc)
            return ERR_PTR(-EFAULT);
    }


    spin_lock(&proto_desc->ref_count_lock);
    if (!proto_desc->ref_count) {
        err = proto_desc->proto->open(hu);
    }
    if (!err)
        proto_desc->ref_count += 1;
    spin_unlock(&proto_desc->ref_count_lock);

    if (err)
        return ERR_PTR(err);
    else
        return proto_desc;
}

/*******************************************************************************
**
** Function - put_component_desc()
**
** Description - Decreasing by 1 the reference count of the designated component descriptor,
**             which can be BT, FM, GPS etc, and deleting it when the ref_count drops to 0
**
** Returns - true if reference drop indeed happens, false otherwise
**
*******************************************************************************/
static bool put_component_desc(struct hci_uart *hu, enum component_type comp_type)
{
    struct component_desc *comp_desc = NULL;
    bool put = false;

    spin_lock(&hu->component_descs_lock);
    comp_desc = hu->component_descs[comp_type];
    if (comp_desc) {
        if (comp_desc->ref_count == 1)
            hu->component_descs[comp_type] = NULL;
        else {
            comp_desc->ref_count -= 1;
            comp_desc = NULL;
        }
        put = true;
    }
    spin_unlock(&hu->component_descs_lock);

    if (comp_desc) {
        kfree(comp_desc);
    }

    return put;
}

/*******************************************************************************
**
** Function - get_component_desc()
**
** Description - Increasing by 1 the reference count of the descriptor of the given
**             component type, which can be BT, FM, GPS etc, and returning
**             that descriptor pointer (if exists)
**
** Returns - The descriptor pointer of the given type if exists, NULL otherwise
**
*******************************************************************************/
static struct component_desc *get_component_desc(struct hci_uart *hu, enum component_type comp_type)
{
    struct component_desc *comp_desc = NULL;

    spin_lock(&hu->component_descs_lock);
    comp_desc = hu->component_descs[comp_type];
    if (comp_desc) {
        comp_desc->ref_count += 1;
    }
    spin_unlock(&hu->component_descs_lock);

    return comp_desc;
}

/*******************************************************************************
**
** Function - reset_component_desc()
**
** Description - Creating a new component descriptor keeping the given interface
**             of some component type, which can be BT, FM, GPS etc, with its
**             initial reference count equal to 1
**
** Returns - 0 if successful, a negative errno otherwise
**
*******************************************************************************/
static int reset_component_desc(struct hci_uart *hu, struct component_interface *interface)
{
    int ret = 0;
    struct component_desc *removed_desc = NULL;
    struct component_desc *new_desc = kzalloc(sizeof(struct component_desc), GFP_KERNEL);

    if (!new_desc) {
        BRCM_LDISC_ERR("Failed to allocate memory for comp_desc %d", interface->type);
        return -ENOMEM;
    }

    init_waitqueue_head(&new_desc->sync_waiter);
    new_desc->ref_count = 1;
    new_desc->interface = interface;

    spin_lock(&hu->component_descs_lock);
    removed_desc = hu->component_descs[interface->type];
    if (removed_desc) {
        if (removed_desc->ref_count > 1) {
            removed_desc = new_desc;
        } else {
            hu->component_descs[interface->type] = new_desc;
        }
    } else {
        hu->component_descs[interface->type] = new_desc;
    }
    spin_unlock(&hu->component_descs_lock);

    if (removed_desc == new_desc)
        ret = -EBUSY;

    if (removed_desc)
        kfree(removed_desc);

    return ret;
}


/*******************************************************************************
**
** Function - brcm_hci_uart_tx_complete()
**
** Description - Called upon TX completion to print packet type for the debugging
**               purposes
**
** Returns - void
**
*******************************************************************************/
static inline void brcm_hci_uart_tx_complete(struct hci_uart *hu,
                                                                   int pkt_type)
{
    /* Update HCI stat counters */
    switch (pkt_type)
    {
        case HCI_COMMAND_PKT:
            BRCM_LDISC_DBG(V4L2_DBG_TX, "HCI command packet txed");
            break;

        case HCI_ACLDATA_PKT:
            BRCM_LDISC_DBG(V4L2_DBG_TX, "HCI ACL DATA packet txed");
            break;

        case HCI_SCODATA_PKT:
            BRCM_LDISC_DBG(V4L2_DBG_TX ,"HCI SCO DATA packet txed");
            break;
    }
}


/*******************************************************************************
**
** Function - brcm_hci_uart_tx_work_handler()
**
** Description - The tx work handler called by kernel workqueue infrastructure to
**             dequeue data and write them to tty
**
** Returns - none
**
*******************************************************************************/
static void brcm_hci_uart_tx_work_handler(struct work_struct *w) {

    struct hci_uart *hu = container_of(w, struct hci_uart, tx_work);
    struct hci_uart_proto_desc *proto_desc;
    struct sk_buff *skb = NULL;
    bool first = true;

    while (hu->tx_on) {

        proto_desc = get_proto(hu, NULL);
        if (unlikely(IS_ERR_OR_NULL(proto_desc)))
            break;

        skb = proto_desc->proto->dequeue(hu);
        put_proto(hu, proto_desc);


        if (!skb)
            break;

        if (first) {
            /* If this is the first time entering this loop,
             * making sure the hardware gets woken up before writing
             */
            brcm_btsleep_wake(sleep);
            first = false;
        }

        brcm_hci_tty_op_write(hu, skb->data, skb->len);

        kfree_skb(skb);

    }

}

/*******************************************************************************
**
** Function - send_hci_pkt_internal()
**
** Description - A helper to put raw data into a sk_buff packet and conduct tx transmission
**             in the blocking mode
**
** Returns - when deleting_resp is false
**              A sk_buff pointer to the response if successful; an error ptr otherwise
**           when deleting_resp is true
**              NULL if successful; an error ptr otherwise
**
*******************************************************************************/
static struct sk_buff *send_hci_pkt_internal(struct hci_uart *hu, const unsigned char *data, int size, bool deleting_resp)
{
    int ret = 0;
    struct sk_buff *skb, *response = NULL;
    //		bool sync = false;
    //		struct response_sync resp_sync = {0};

    skb = alloc_skb(size, GFP_KERNEL);
    if (!skb)
        return ERR_PTR(-ENOMEM);

    memcpy(skb_put(skb, size), data, size);
    sh_ldisc_cb(skb)->comp_type = COMPONENT_BT;

    response = brcm_hci_queue_tx(hu, skb, false /*never_blocking*/);

    if (IS_ERR(response)) {
        kfree_skb(skb);

    } else {
        // Note that even in this case where the skb has been sent successfully,
        // response could also be null due to timeout or being associated with a type not supporting the sync mode
        if (deleting_resp && response) {
            kfree_skb(response);
            response = NULL;
        }

    }

    return response;
}


/*******************************************************************************
**
** Function - brcm_sh_ldisc_lpm_disable()
**
** Description - disable LPM before turning OFF the chip.
**
** Returns - 0 if success; errno otherwise
**
*******************************************************************************/
int brcm_sh_ldisc_lpm_disable(struct hci_uart *hu)
{
    const char hci_lpm_disable_cmd[] = {0x01,0x27,0xfc,0x0c,0x00,0x00,0x00,0x00,
                                        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

    int ret = 0;

    /* Perform LPM disable  */
    ret = PTR_ERR(send_hci_pkt_internal(hu, hci_lpm_disable_cmd, sizeof(hci_lpm_disable_cmd), true));

    if (ret) {
        BRCM_LDISC_ERR("An error (%d) occurred while disabling LPM", ret);
    } else {
        BRCM_LDISC_DBG(V4L2_DBG_INIT, "LPM disabled");
    }

    return ret;
}

/*******************************************************************************
**
** Function - reset_err_flags()
**
** Description - Reseting the error flag of a newly registered protocol driver
**
** Returns - void
**
*******************************************************************************/
void reset_err_flags(struct component_interface *new_proto)
{
    struct hci_uart *hu = HU_REF;

    switch(new_proto->type) {
        case COMPONENT_BT:
            hu->ldisc_bt_err = V4L2_ERR_FLAG_RESET;
            BRCM_LDISC_DBG(V4L2_DBG_OPEN, "resetting bt_err");
            sysfs_notify(&hu->brcm_pdev->dev.kobj, NULL, "bt_err");
            break;
        case COMPONENT_FM:
            hu->ldisc_fm_err = V4L2_ERR_FLAG_RESET;
            sysfs_notify(&hu->brcm_pdev->dev.kobj, NULL, "fm_err");
            BRCM_LDISC_DBG(V4L2_DBG_OPEN, "resetting fm_err");
            break;
        default:
            BRCM_LDISC_ERR("Unknown proto id");
            break;
    }

}

/*******************************************************************************
**
** Function - brcm_sh_ldisc_register()
**
** Description - Registering components;
**              called from component stack drivers (BT or FM)
**
** Returns - 0 if success; errno otherwise
**
*******************************************************************************/
long brcm_sh_ldisc_register(struct component_interface *new_proto)
{
    long err = 0;
    unsigned long diff;
    struct hci_uart *hu = HU_REF;

    BRCM_LDISC_DBG(V4L2_DBG_OPEN, "%p",hu);

    if(new_proto->type != 0)
    {
        BRCM_LDISC_DBG(V4L2_DBG_OPEN, "(%d) ", new_proto->type);
    }
    if (hu == NULL || new_proto == NULL || new_proto->recv == NULL)
    {
        if (is_print_reg_error)
        {
            BRCM_LDISC_DBG(V4L2_DBG_OPEN,"%d) ", new_proto->type);
            pr_err("hu/new_proto/recv or reg_complete_cb not ready");
            jiffi1 = jiffies;
            is_print_reg_error = 0;
        }
        else
        {
            jiffi2 = jiffies;
            diff = (long)jiffi2 - (long)jiffi1;
            if ( ((diff * HZ * 10) / HZ) >= 1000)
                is_print_reg_error = 1;
        }
        return -EINVAL;
    }

    /* check if received proto is a valid proto */
    if (new_proto->type < COMPONENT_BT || new_proto->type >= COMPONENT_MAX)
    {
        BRCM_LDISC_ERR("protocol %d not supported", new_proto->type);
        return -EPROTONOSUPPORT;
    }

    if (test_and_set_bit_lock(LDISC_SUB_PROTO_REG_UNREG_IN_PROGRESS, &hu->flags))
        return -EBUSY;

    if (test_bit(LDISC_OPENING, &hu->flags) || test_bit(LDISC_CLOSING, &hu->flags)) {
        err = -EBUSY;
        goto out;
    }

    /* check if protocol already registered */
    if (hu->components_registered & (1 << new_proto->type)) {
        BRCM_LDISC_DBG(V4L2_DBG_OPEN, "protocol %d already registered", new_proto->type);
        err = -EALREADY;
        goto out;
    }

    if (!hu->components_registered) {
        if ((err = reset_component_desc(hu, &init_interface))) {
            BRCM_LDISC_ERR("Failed to set init_interface for initialisation");
            goto out;
        }
        BRCM_LDISC_DBG(V4L2_DBG_OPEN, " chnl_id list empty :%d ", new_proto->type);

        err = brcm_sh_ldisc_start(hu);
        BRCM_LDISC_DBG(V4L2_DBG_OPEN, "brcm_sh_ldisc_start response = %ld", err);
        if (err) {
            put_component_desc(hu, init_interface.type);
            err = (err == -EBUSY) ? err : -EINVAL;
            goto out;
        }
    }

    new_proto->write = brcm_sh_ldisc_write;
    reset_err_flags(new_proto);
    err = reset_component_desc(hu, new_proto);
    if (err) {
        brcm_sh_ldisc_stop(hu, 1);
        goto out;
    }

    hu->components_registered |= (1 << new_proto->type);
    BRCM_LDISC_DBG(V4L2_DBG_OPEN, "Changed hu->components_registered = 0x%X",
            hu->components_registered);


    BRCM_LDISC_DBG(V4L2_DBG_OPEN, "done (%d) ", new_proto->type);

out:
    clear_bit_unlock(LDISC_SUB_PROTO_REG_UNREG_IN_PROGRESS, &hu->flags);
    return err;
}
EXPORT_SYMBOL(brcm_sh_ldisc_register);


/*******************************************************************************
**
** Function - brcm_sh_ldisc_unregister()
**
** Description - Unregister components;
**              called from component stack drivers (BT or FM)
**
** Returns - 0 if success; errno otherwise
**
*******************************************************************************/
long brcm_sh_ldisc_unregister(enum component_type type, bool btsleep_open)
{
    unsigned long flags;
    struct hci_uart *hu = HU_REF;
	int components_registered = 0, err = 0;


    BRCM_LDISC_DBG(V4L2_DBG_CLOSE,"%d ", type);

    if (type < COMPONENT_BT || type >= COMPONENT_MAX)
    {
        BRCM_LDISC_ERR(" protocol %d not supported", type);
        return -EPROTONOSUPPORT;
    }

    BRCM_LDISC_DBG(V4L2_DBG_CLOSE, "%p ", hu);

    if (hu == NULL)
    {
        BRCM_LDISC_ERR("hu is NULL");
        return -EINVAL;
    }

    if (test_and_set_bit_lock(LDISC_SUB_PROTO_REG_UNREG_IN_PROGRESS, &hu->flags))
        return -EBUSY;

    if (test_bit(LDISC_OPENING, &hu->flags) || test_bit(LDISC_CLOSING, &hu->flags)) {
        err = -EBUSY;
        goto out;
    }

    if (!put_component_desc(hu, type)) {
        BRCM_LDISC_ERR("protocol %d is already unregistered", type);
        err = -EALREADY;
        goto out;
    }

    hu->components_registered &= ~(1 << type);

    BRCM_LDISC_DBG(V4L2_DBG_CLOSE, "Changed hu->components_registered = 0x%X", hu->components_registered);

    /* Power OFF chip only if no components are registered.
       Send notification to UIM to power OFF chip */
    if (!hu->components_registered) {

        BRCM_LDISC_DBG(V4L2_DBG_CLOSE," all chnl_ids unregistered ");

        /* stop traffic on tty */
        if (hu->tty){
            BRCM_LDISC_DBG(V4L2_DBG_CLOSE," calling tty_ldisc_flush ");
            tty_ldisc_flush(hu->tty);
            BRCM_LDISC_DBG(V4L2_DBG_CLOSE," calling stop_tty ");
            stop_tty(hu->tty);
        }

        brcm_sh_ldisc_stop(hu, btsleep_open);
    }

out:
    clear_bit_unlock(LDISC_SUB_PROTO_REG_UNREG_IN_PROGRESS, &hu->flags);

    return err;
}
EXPORT_SYMBOL(brcm_sh_ldisc_unregister);


/*******************************************************************************
**
** Function - brcm_sh_ldisc_extract_hcicmd_sync_id()
**
** Description - The sync-id extracting function from hcicmd packets sent
**             internally in the blocking mode;
**
** Returns - an id > 0 if success; 0 otherwise
**
*******************************************************************************/
static __u32 brcm_sh_ldisc_extract_hcicmd_sync_id(struct sk_buff *skb)
{
    __u32 sync_id = 0, len_expected = 1;
    if (skb->len < len_expected)
        return sync_id;

    switch (skb->data[0]) {
        case HCI_COMMAND_PKT: {
            len_expected += sizeof(struct hci_command_hdr);
            if (skb->len < len_expected)
                break;

            sync_id = __le16_to_cpu(((struct hci_command_hdr *)(skb->data+1))->opcode);

            break;
        }
        case HCI_EVENT_PKT: {
            len_expected += sizeof(struct hci_event_hdr);
            if (skb->len < len_expected)
                break;

            struct hci_event_hdr *ev_hdr = (struct hci_event_hdr *)(skb->data+1);

            switch (ev_hdr->evt) {
                case HCI_EV_CMD_COMPLETE: {
                    len_expected += sizeof(struct hci_ev_cmd_complete);
                    if (skb->len < len_expected)
                        break;

                    sync_id = __le16_to_cpu(((struct hci_ev_cmd_complete *)(ev_hdr+1))->opcode);
                    break;
                }
#if 0
                case HCI_EV_CMD_STATUS: {
                    len_expected += sizeof(struct hci_ev_cmd_status);
                    if (skb->len < len_expected)
                        break;

                    sync_id = __le16_to_cpu(((struct hci_ev_cmd_status *)(ev_hdr+1))->opcode);
                    break;
                }
#endif
                default:
                    break;
            }
            break;
        }
        default:
            break;
    }

    return sync_id;
}

/*****************************************************************************
* Function to encode baudrate from int to char sequence
*****************************************************************************/
void BRCM_encode_baud_rate(uint baud_rate, unsigned char *encoded_baud)
{
    if(baud_rate == 0 || encoded_baud == NULL) {
        pr_err("Baudrate not supported!");
        return;
    }

    encoded_baud[3] = (unsigned char)(baud_rate >> 24);
    encoded_baud[2] = (unsigned char)(baud_rate >> 16);
    encoded_baud[1] = (unsigned char)(baud_rate >> 8);
    encoded_baud[0] = (unsigned char)(baud_rate & 0xFF);
}

typedef struct {
    int baud_rate;
    int termios_value;
} tBaudRates;

tBaudRates baud_rates[] = {
    { 115200, B115200 },
    { 230400, B230400 },
    { 460800, B460800 },
    { 500000, B500000 },
    { 576000, B576000 },
    { 921600, B921600 },
    { 1000000, B1000000 },
    { 1152000, B1152000 },
    { 1500000, B1500000 },
    { 2000000, B2000000 },
    { 2500000, B2500000 },
    { 3000000, B3000000 },
	{ 4000000, B4000000 },
};

/*****************************************************************************
* Function to lookup baudrate
*****************************************************************************/
int
lookup_baudrate(long int baud_rate)
{
    unsigned int i;

    for (i = 0; i < (sizeof(baud_rates) / sizeof(tBaudRates)); i++) {
        if (baud_rates[i].baud_rate == baud_rate) {
            return i;
        }
    }

    return(-1);
}


/*****************************************************************************
* Function to encode baudrate from int to char sequence
*****************************************************************************/
void
BRCM_encode_bd_address( unsigned char  *bd_addrr)
{
    if(bd_addrr == NULL) {
        pr_info("%s : BD addr not supported!", __func__);
        return;
    }

    bd_addrr[0] = bd_addr_array[5];
    bd_addrr[1] = bd_addr_array[4];
    bd_addrr[2] = bd_addr_array[3];
    bd_addrr[3] = bd_addr_array[2];
    bd_addrr[4] = bd_addr_array[1];
    bd_addrr[5] = bd_addr_array[0];
}

/*****************************************************************************
* Function to read BD ADDR from bluetooth chip.
* Note that the resp_buffer will only get updated when LDISC_EMPTY, and therefore this call can only be invoked during setup
* (e.g. triggered by brcm_sh_ldisc_start())
*
*****************************************************************************/
static long read_bd_addr(struct hci_uart *hu)
{
    long err = 0;

    const char hci_read_bdaddr[] = { 0x01, 0x09, 0x10, 0x00 };

    struct sk_buff *resp = send_hci_pkt_internal(hu, hci_read_bdaddr, sizeof(hci_read_bdaddr), false);

    if (IS_ERR_OR_NULL(resp)) {
        err = PTR_ERR(resp);
        if (!resp)
            err = -EFAULT;

        BRCM_LDISC_ERR("An error (%d) while reading bdaddr", err);
    } else {
        int i = 0;

        BRCM_LDISC_DBG(V4L2_DBG_INIT, "read_bd_addr, response received");
        for (i=0;i<=5;i++)
            bd_addr_array[i] = (unsigned char)resp->data[7+i];

        kfree_skb(resp);
    }

    return err;
}

int str2bd(char *str, bt_bdaddr_t *addr)
{
    int32_t i = 0;
    for (i = 0; i < 6; i++)
    {
        addr->address[i] = (uint8_t)simple_strtoul(str, &str, 16);
        str++;
    }
    return 0;
}


int str2arr(char *str, uint8_t *addr,int len)
{
    int32_t i = 0;
    for (i = 0; i < len; i++)
    {
      addr[i] = (uint8_t)simple_strtoul(str, &str, 16);
       str++;
    }
    return 0;
}

/*****************************************************************************
* Function to download firmware patchfile to bluetooth chip.
*****************************************************************************/
static long download_patchram(struct hci_uart *hu)
{
    long err = 0;
    long len =0;
    struct tty_struct *tty = hu->tty;
    uint8_t hci_writesleepmode_cmd[16] = {0};
    unsigned char *ptr;

    const unsigned char hci_download_minidriver[] = { 0x01, 0x2e, 0xfc, 0x00 };
    const unsigned char hci_reset_cmd[] = {0x01, 0x03, 0x0C, 0x00};
    unsigned char hci_update_baud_rate[] = { 0x01, 0x18, 0xFC, 0x06,0x00, \
                                               0x00,0x00,0x00,0x00,0x00 };
    unsigned char hci_update_bd_addr[] = { 0x01, 0x01, 0xFC, 0x06,0x00, \
                                               0x00,0x00,0x00,0x00,0x00 };
    const char hci_uartclockset_cmd[] = {0x01, 0x45, 0xFC, 0x01, 0x01};

    unsigned char *buf = NULL;
    struct ktermios ktermios;

	if (!(buf = kmalloc(HCI_MAX_FRAME_SIZE, GFP_KERNEL))) {
        BRCM_LDISC_ERR("Unable to allocate memory for buf");
        err = -ENOMEM;
        goto error_state;
    }

    if (unlikely(hu == NULL || hu->tty == NULL)) {
        err = -EINVAL;
        goto error_state;
    }
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
    memcpy(&ktermios, tty->termios, sizeof(ktermios));
#else
    memcpy(&ktermios, &(tty->termios), sizeof(ktermios));
#endif
    ptr = NULL;

    BRCM_LDISC_DBG(V4L2_DBG_INIT, "tty = %p hu = %p", tty,hu);

    /* request_firmware searches for patchram file. only in /vendor/firmware OR /etc/firmware */
    BRCM_LDISC_DBG(V4L2_DBG_INIT, "firmware patchram file: %s", fw_name);
    err = request_firmware(&hu->fw_entry, fw_name,
                 &hu->brcm_pdev->dev);

    /* patchram download failure */
    if ((unlikely((err != 0) || (hu->fw_entry->data == NULL) ||
         (hu->fw_entry->size == 0)))) {
         BRCM_LDISC_ERR("************* ERROR !!! Unable to download patchram \
            OR file %s not found **************", fw_name);
    }
    else {
        ptr = (void *)hu->fw_entry->data;
        len = hu->fw_entry->size;

        BRCM_LDISC_DBG(V4L2_DBG_INIT, " with header patchram data ptr %p, \
            len %ld ",ptr,len);

        /* write command for hci_download_minidriver. Perform this before patchram download */
        BRCM_LDISC_DBG(V4L2_DBG_INIT, "writing hci_download_minidriver");

        err = PTR_ERR(send_hci_pkt_internal(hu, hci_download_minidriver, sizeof(hci_download_minidriver), true));

        if (!err) {
            /* delay before patchram download */
            msleep(50);
            /* start downloading firmware */
            while(len>0 && !err) {
                buf[0] = 0x01;
                memcpy(buf+1, ptr, 3);
                ptr += 3;
                /* buf[3] holds the len of data to send */
                memcpy(buf+4, ptr, buf[3]);
                len -= buf[3] + 3;

                err = PTR_ERR(send_hci_pkt_internal(hu, buf, buf[3] + 4, true));
                ptr += buf[3];

            }

        }

        if (err == -ETIMEDOUT) {
            BRCM_LDISC_ERR(" waiting for download patchram command response - timed out ");
        }
        if (err)
            goto error_state;

        BRCM_LDISC_DBG(V4L2_DBG_INIT, "fw patchram download complete");

        /* settlement delay */
        BRCM_LDISC_DBG(V4L2_DBG_INIT, "patchram_settlement_delay = %d", \
            patchram_settlement_delay);
        msleep(patchram_settlement_delay);

        /* firmware patchram download complete */
        release_firmware(hu->fw_entry);

        /* set baud rate to default */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
        memcpy(&ktermios, tty->termios, sizeof(ktermios));
#else
        memcpy(&ktermios, &(tty->termios), sizeof(ktermios));
#endif
        BRCM_LDISC_DBG(V4L2_DBG_INIT, "before setting baudrate = %d\n",
                                     (speed_t)(ktermios.c_cflag & CBAUD));
        ktermios.c_cflag = (ktermios.c_cflag & ~CBAUD) | (B115200 & CBAUD);
        tty_set_termios(tty, &ktermios);

        msleep(20);

        BRCM_LDISC_DBG(V4L2_DBG_INIT, "after setting baudrate = %d\n",
                                           (speed_t)(ktermios.c_cflag & CBAUD));
    }

    /* Perform HCI Reset */
    BRCM_LDISC_DBG(V4L2_DBG_INIT, "Performing HCI Reset");

    err = PTR_ERR(send_hci_pkt_internal(hu, hci_reset_cmd, sizeof(hci_reset_cmd), true));
    if (err == -ETIMEDOUT) {
        BRCM_LDISC_ERR(" waiting for HCI Reset command response - timed out ");
    }
    if (err)
        goto error_state;

    BRCM_LDISC_DBG(V4L2_DBG_INIT, "%s HCI Reset complete", __func__);

    /* set UART clock rate to 48 MHz */
    if( custom_baudrate > CLOCK_SET_BAUDRATE) {
        BRCM_LDISC_DBG(V4L2_DBG_INIT, "Baudrate > %ld UART clock set to 48 MHz",
                (unsigned long)CLOCK_SET_BAUDRATE);

        err = PTR_ERR(send_hci_pkt_internal(hu, hci_uartclockset_cmd, sizeof(hci_uartclockset_cmd), true));
        if (err == -ETIMEDOUT) {
            BRCM_LDISC_ERR(" waiting for UART clock set command response - timed out");
        }
        if (err)
            goto error_state;
    }

    /* set baud rate back to custom baudrate */
    BRCM_LDISC_DBG(V4L2_DBG_INIT,"set baud rate back to %ld", custom_baudrate);

    BRCM_encode_baud_rate(custom_baudrate, &hci_update_baud_rate[6]);

    err = PTR_ERR(send_hci_pkt_internal(hu, hci_update_baud_rate, sizeof(hci_update_baud_rate), true));

    if (err == -ETIMEDOUT) {
        BRCM_LDISC_ERR(" waiting for set baud rate back to %ld command response - timed out", custom_baudrate);
    }
    if (err)
        goto error_state;

    ktermios.c_cflag = (ktermios.c_cflag & ~CBAUD) |
        (baud_rates[lookup_baudrate(custom_baudrate)].termios_value & CBAUD);
    tty_set_termios(tty, &ktermios);
    BRCM_LDISC_DBG(V4L2_DBG_INIT, "after setting baudrate = %d\n",
                                          (speed_t)(ktermios.c_cflag & CBAUD));

    if(ControllerAddrRead){
        /*Read controller address and set*/
        pr_info("%s Read controller address and set", __func__);
        err = read_bd_addr(hu);
        if (err != 0) {
            pr_err("ldisc: failed to read local name for patchram");
            goto error_state;
        }
    }
    else
        str2bd(bd_addr,(bt_bdaddr_t*)bd_addr_array);

    BRCM_LDISC_DBG(V4L2_DBG_INIT, "BD ADDRESS going to  set is "\
        "%02X:%02X:%02X:%02X:%02X:%02X",
         bd_addr_array[0], bd_addr_array[1], bd_addr_array[2],
         bd_addr_array[3], bd_addr_array[4], bd_addr_array[5]);

    /* set BD address */
    BRCM_encode_bd_address(&hci_update_bd_addr[4]);

    err = PTR_ERR(send_hci_pkt_internal(hu, hci_update_bd_addr, sizeof(hci_update_bd_addr), true));
    if (err == -ETIMEDOUT) {
        BRCM_LDISC_ERR(" waiting for set bd addr command response \
                - timed out");
    }
    if (err)
        goto error_state;

    BRCM_LDISC_DBG(V4L2_DBG_INIT, "BD ADDRESS set to "\
            "%02X:%02X:%02X:%02X:%02X:%02X",
            bd_addr_array[0], bd_addr_array[1], bd_addr_array[2],
            bd_addr_array[3], bd_addr_array[4], bd_addr_array[5]);

    /* Enable/Disable LPM should be configurable based on the module param */
    BRCM_LDISC_DBG(V4L2_DBG_INIT, "lpm param %s", lpm_param);
    str2arr(lpm_param, (uint8_t*) hci_writesleepmode_cmd, 16);

    BRCM_LDISC_DBG(V4L2_DBG_INIT,"LPM PARAM set to "\
            "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:"\
            "%02X:%02X:%02X:%02X:%02X:%02X:%02X",
            hci_writesleepmode_cmd[0], hci_writesleepmode_cmd[1], hci_writesleepmode_cmd[2],
            hci_writesleepmode_cmd[3],hci_writesleepmode_cmd[4], hci_writesleepmode_cmd[5],
            hci_writesleepmode_cmd[6],hci_writesleepmode_cmd[7], hci_writesleepmode_cmd[8],
            hci_writesleepmode_cmd[9],hci_writesleepmode_cmd[10], hci_writesleepmode_cmd[11],
            hci_writesleepmode_cmd[12],hci_writesleepmode_cmd[13],hci_writesleepmode_cmd[14],
            hci_writesleepmode_cmd[15]);

    err = PTR_ERR(send_hci_pkt_internal(hu, hci_writesleepmode_cmd, sizeof(hci_writesleepmode_cmd), true));
    if (err == -ETIMEDOUT) {
        BRCM_LDISC_ERR(" waiting for set LPM command response timed out");
    }
    if (err)
        goto error_state;

    err = 0;

error_state:
    if (buf != NULL) kfree(buf);
    return err;
}



/*******************************************************************************
**
** Function - brcm_sh_ldisc_stop()
**
** Description - Initiating the stop process, which involves
**             collaboration with UIM closing the ldisc device
**
** Returns - 0 if successful; an negative errno otherwise
**
*******************************************************************************/
static long brcm_sh_ldisc_stop(struct hci_uart *hu, bool btsleep_open)
{
    long err = 0;

    if (test_and_set_bit_lock(LDISC_STOPPING, &hu->flags))
        return -EBUSY;

    if (!test_bit(LDISC_STARTING, &hu->flags) && !test_bit(LDISC_OPENED, &hu->flags)) {

        BRCM_LDISC_ERR("brcm_sh_ldisc_stop should only be called while the device is starting or has been opened");
        err = -EINVAL;
        goto out;
    }

    reinit_completion(&hu->ldisc_installed);
    reinit_completion(&hu->tty_close_complete);

    if(btsleep_open)
        brcm_btsleep_stop(sleep);
    hu->ldisc_install = V4L2_STATUS_OFF;

    /* send uninstall notification to UIM */
    sysfs_notify(&hu->brcm_pdev->dev.kobj, NULL, "install");

    /* wait for ldisc to be un-installed */
    if (!wait_for_completion_timeout(&hu->ldisc_installed, msecs_to_jiffies(LDISC_TIME))) {
        /* timeout */
        BRCM_LDISC_ERR(" timed out waiting for ldisc to be un-installed");
        err = -ETIMEDOUT;
        goto out;
    }

    if (!wait_for_completion_timeout(&hu->tty_close_complete, msecs_to_jiffies(TTY_CLOSE_TIME))) {
        BRCM_LDISC_ERR("tty close timed out");
        err = -ETIMEDOUT;
        goto out;
    }

out:
    clear_bit_unlock(LDISC_STOPPING, &hu->flags);
    return err;
}

/*******************************************************************************
**
** Function - brcm_sh_ldisc_start()
**
** Description - Initiating the start process, which involves
**             collaboration with UIM and invokes download_patchram()
**             to reset the device with a proper firmware
**
** Returns - 0 if successful; an negative errno otherwise
**
*******************************************************************************/
static long brcm_sh_ldisc_start(struct hci_uart *hu)
{
    long err = 0;
    long retry = POR_RETRY_COUNT;
    long cl_err = 0;

    struct tty_struct *tty = hu->tty;

    BRCM_LDISC_DBG(V4L2_DBG_INIT, " %p",tty);

    if (test_and_set_bit_lock(LDISC_STARTING, &hu->flags))
        return -EBUSY;


    do {
        if (brcm_btsleep_start(sleep) == -EBUSY) {
            err = -EBUSY;
            break;
        }

        reinit_completion(&hu->ldisc_installed);
        /* send notification to UIM */
        hu->ldisc_install = V4L2_STATUS_ON;
        BRCM_LDISC_DBG(V4L2_DBG_INIT, "ldisc_install = %c",\
                hu->ldisc_install);
        sysfs_notify(&hu->brcm_pdev->dev.kobj,
                NULL, "install");

        /* wait for ldisc to be installed */
        err = wait_for_completion_timeout(&hu->ldisc_installed,
                msecs_to_jiffies(LDISC_TIME));
        if (!err) { /* timeout */
            pr_err("line disc installation timed out ");
            cl_err = brcm_sh_ldisc_stop(hu, 1);
            if (cl_err)
                break;
            continue;
        } else {
            /* ldisc installed now */
            BRCM_LDISC_DBG(V4L2_DBG_INIT, " line discipline installed ");
            err = download_patchram(hu);
            if (err != 0) {
                pr_err("patchram download failed");
                cl_err = brcm_sh_ldisc_stop(hu, 1);
                if (cl_err)
                    break;
                continue;
            } else {/* on success don't retry */
                BRCM_LDISC_DBG(V4L2_DBG_INIT, "patchram downloaded successfully");
                // initialize lock for err flags
                spin_lock_init(&hu->err_lock);

                break;
            }
        }
    } while (retry--);

    clear_bit_unlock(LDISC_STARTING, &hu->flags);
    return err;
}


/*******************************************************************************
**
** Function - ignore_hci_cmd()
**
** Description - Checks if the received packet is a HCI command that shall be
**               ignored based on the IGNORE_CMDS array. Commands that shall be
**               ignored are resets and configuration commands emmitted by libbt.
**
** Returns - true if the packet is HCI cmd to ignore; false otherwise
**
*******************************************************************************/
static bool ignore_hci_cmd(struct sk_buff *skb)
{
    if(skb->len < 3 || (skb->len > 0 && (skb->data)[0] != 0x01)) {
        return false;
    }
    else {
        int cmd_cnt = 0;
        struct hci_command_hdr *hdr = (struct hci_command_hdr *)(skb->data+1);
        __u16 opcode = __le16_to_cpu(hdr->opcode);

        for( ; cmd_cnt < IGNORE_CMD_SIZE; cmd_cnt++) {
            if(opcode == IGNORE_CMDS[cmd_cnt]) {
                return true;
            }
        }
    }

    return false;
}

/*******************************************************************************
**
** Function - brcm_sh_ldisc_write()
**
** Description - Writing a sk_buff packet to the shared line discipline driver;
**              called from component stack drivers (BT or FM)
**              via a function pointer to it in a process context (kernel thread).
**
**              Note that only when input skbs are processed successfully,
**              will their ownership be transfered; therefore,
**              callers are still responsible for packets' lifespan if errors occur
**
** Returns - 0 if success; errno otherwise
**
*******************************************************************************/
static struct sk_buff *brcm_sh_ldisc_write(struct sk_buff *skb, bool never_blocking)
{
    struct hci_uart *hu = HU_REF;

    BRCM_LDISC_DBG(V4L2_DBG_TX, "%p",hu);

    if (unlikely(skb == NULL))
    {
        BRCM_LDISC_ERR("data unavailable to perform write");
        return -EFAULT;
    }

    if (unlikely(hu == NULL || hu->tty == NULL))
    {
        BRCM_LDISC_ERR("tty unavailable to perform write");
        return -EFAULT;
    }

    return brcm_hci_queue_tx(hu, skb, never_blocking);
}


/*****************************************************************************
**   Shared line discipline driver APIs
*****************************************************************************/
/*******************************************************************************
**
** Function - hci_uart_tty_open
**
** Description - Called when line discipline changed to HCI_UART.
**
** Arguments - tty    pointer to tty info structure
**
** Returns - 0 if success, otherwise error code
**
*******************************************************************************/
static int brcm_hci_uart_tty_open(struct tty_struct *tty)
{
    struct hci_uart *hu = HU_REF;
    struct hci_uart_proto_desc *default_proto_desc;
    unsigned long flags;
    int err = 0;

    BRCM_LDISC_DBG(V4L2_DBG_INIT, "tty open %p hu %p", tty,hu);

    if (test_and_set_bit_lock(LDISC_OPENING, &hu->flags)) {
        return -EBUSY;
    }

    if (test_bit(LDISC_OPENED, &hu->flags)) {

        BRCM_LDISC_ERR("this ldisc had been opened");
        err = -EINVAL;
        goto out;
    }

    hu->tx_wq = alloc_ordered_workqueue("brcm_ldisc_tx", 0);
    if (!hu->tx_wq) {
        BRCM_LDISC_ERR("could not allocate memory for tx_wq");
        err = -ENOMEM;
        goto out;
    }

    /* don't do an wakeup at this point */
    clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);

    tty->disc_data = hu;
    hu->tty = tty;
    BRCM_LDISC_DBG(V4L2_DBG_INIT, "tty open %p hu %p", tty,hu);
    tty->receive_room = N_TTY_BUF_SIZE;

    hu->tx_on = true;
    INIT_WORK(&hu->tx_work, brcm_hci_uart_tx_work_handler);

#if V4L2_SNOOP_ENABLE
    spin_lock_init(&hu->hcisnoop_lock);
#endif

    /* Flush any pending characters in the driver and line discipline. */
    tty_ldisc_flush(tty);
    tty_driver_flush_buffer(tty);

    default_proto_desc = hup[0];
    /* Increasing its reference count by 1 */
    default_proto_desc = get_proto(hu, default_proto_desc);
    if (IS_ERR(default_proto_desc)) {
        err = PTR_ERR(default_proto_desc);
    } else if (default_proto_desc == NULL) {
        err = -EFAULT;
    } else {
        hu->curr_proto_desc = default_proto_desc;
    }

    if (err) {
        BRCM_LDISC_ERR("Failed to acquire proto 0 as the default one");
        goto out;
    }

    set_bit(LDISC_OPENED, &hu->flags);

    /* Note that complete(&hu->ldisc_installed) also has the effect of write memory barriers */
    complete(&hu->ldisc_installed);

out:
    if (err) {
        if (hu->tx_wq) {
            destroy_workqueue(hu->tx_wq);
            hu->tx_wq = NULL;
        }
    }

    clear_bit_unlock(LDISC_OPENING, &hu->flags);
    return err;
}

/*******************************************************************************
**
** Function - hci_uart_tty_close
**
** Description - Called when the line discipline is changed to something
**              else, the tty is closed, or the tty detects a hangup.
**
** Arguments - tty    pointer to associated tty instance data
**
** Returns - void
**
*******************************************************************************/
static void brcm_hci_uart_tty_close(struct tty_struct *tty)
{
    struct hci_uart *hu = (void *)tty->disc_data; /* assign tty instance to hci_uart */
    int err = 0;

    if (unlikely(!hu)) {

        BRCM_LDISC_ERR("hci_uart ptr is null");
        return -EINVAL;
    }

    if (test_and_set_bit_lock(LDISC_CLOSING, &hu->flags)) {
        return -EBUSY;
    }

    if (!test_bit(LDISC_OPENED, &hu->flags)) {
        BRCM_LDISC_ERR("ldisc is not opened");
        err = -EINVAL;
        goto out;
    }

    if (test_bit(LDISC_SUB_PROTO_REG_UNREG_IN_PROGRESS, &hu->flags) && !test_bit(LDISC_STOPPING, &hu->flags)) {
        BRCM_LDISC_ERR("Closing ldisc is not allowed when sup proto registration is in progress, but ldisc is not waiting in LDISC_STOPPING state");
        err = -EINVAL;
        goto out;
    }

    BRCM_LDISC_DBG(V4L2_DBG_INIT, "tty= %p", tty);

    hu->tx_on = false;
    smp_wmb();
    destroy_workqueue(hu->tx_wq);

    /* Detach from the tty */
    tty->disc_data = NULL;

    tty_ldisc_flush(tty);
    tty_driver_flush_buffer(tty);

    if (hu->components_registered) {
        int i;
        hu->components_registered = 0;
        for (i = COMPONENT_BT; i < COMPONENT_MAX; i++)
        {
            put_component_desc(hu, i);
        }

        BRCM_LDISC_DBG(V4L2_DBG_CLOSE, "brcm_hci_uart_tty_close protos_registered = 0x00");
    }

    /* This will trigger the shutdown procedure of the current selected proto:
     * The last call to put_proto(), which might not be this one,
     * will close the associated proto
     */
    put_proto(hu, NULL);

    clear_bit(LDISC_OPENED, &hu->flags);

    hu->tty = 0;
    /* Note that complete(&hu->ldisc_installed)/complete(&hu->tty_close_complete)
     * also has the effect of write memory barriers
     */
    complete(&hu->ldisc_installed);
    complete(&hu->tty_close_complete);

out:
    clear_bit_unlock(LDISC_CLOSING, &hu->flags);

    BRCM_LDISC_DBG(V4L2_DBG_INIT, "tty close exit");
}

/*******************************************************************************
**
** Function - hci_uart_tty_wakeup
**
** Description - Callback for transmit wakeup. Called when low level
**              device driver can accept more send data.
**
** Arguments - tty    pointer to associated tty instance data
**
** Returns - void
**
*******************************************************************************/
static void brcm_hci_uart_tty_wakeup(struct tty_struct *tty)
{
    struct hci_uart *hu = (void *)tty->disc_data; /* assign tty instance to hci_uart */

    if (!hu)
        return;

    if (tty != hu->tty)
        return;

    clear_bit(TTY_DO_WRITE_WAKEUP, &tty->flags);
}

/*******************************************************************************
**
** Function - hci_uart_tty_receive
**
** Description - Called by tty low level driver when receive data is available.
**
* Arguments - tty          pointer to tty isntance data
*             data         pointer to received data
*             flags        pointer to flags for data
*             count        count of received data in bytes
** Returns - void
**
*******************************************************************************/
static void brcm_hci_uart_tty_receive(struct tty_struct *tty,
                                         const u8 *data, char *flags, int count)
{
    struct hci_uart *hu = (void *)tty->disc_data; /* assign tty instance to hci_uart */
    unsigned long lock_flags;
    struct hci_uart_proto_desc *proto_desc;

    if (!hu || tty != hu->tty)
        return;

    proto_desc = get_proto(hu, NULL);

    if (likely(!IS_ERR_OR_NULL(proto_desc))) {
        proto_desc->proto->recv(hu, (void *) data, count);
        put_proto(hu, proto_desc);
    }

}

/*******************************************************************************
**
** Function - hci_uart_tty_ioctl
**
** Description - Process IOCTL system call for the tty device.
**
** Arguments - tty  pointer to tty instance data
**    file          pointer to open file object for device
**    cmd           IOCTL command code
**    arg           argument for IOCTL call (cmd dependent)
**
** Returns - 0 if success, otherwise error code
**
*******************************************************************************/
static int brcm_hci_uart_tty_ioctl(struct tty_struct *tty,
                        struct file * file, unsigned int cmd, unsigned long arg)
{
    struct hci_uart *hu = (void *)tty->disc_data; /* assign tty instance to hci_uart */
    int ret = 0;

    BRCM_LDISC_DBG(V4L2_DBG_INIT,"cmd %d", cmd);

    /* Verify the status of the device */
    if (!hu)
        return -EFAULT;

    switch (cmd)
    {
        case HCIUARTSETPROTO: {
            void *err_ptr;
            BRCM_LDISC_DBG(V4L2_DBG_INIT, "SETPROTO %lu hu %p", arg, hu);
            if (arg >= HCI_UART_MAX_PROTO)
                ret = -EPROTONOSUPPORT;
            else {
                struct hci_uart_proto_desc *new_desc = hup[arg];
                if (new_desc == NULL) {
                    ret = -ENXIO;
                } else {
                    /* Calling get_proto() once to increase
                     * its reference count by 1
                     */
                    new_desc = get_proto(hu, new_desc);
                    if (IS_ERR_OR_NULL(new_desc)) {
                        ret = -EFAULT;
                    } else {
                        struct hci_uart_proto_desc *old_desc = hu->curr_proto_desc;
                        hu->curr_proto_desc = new_desc;
                        smp_wmb();
                        /* Don't care if there is an error returned
                         * by put_proto(), for now
                         */
                        put_proto(hu, old_desc);
                    }
                }
            }

            break;
        }
        case HCIUARTGETPROTO: {
            if (!hu->curr_proto_desc)
                ret = -EFAULT;
            else
                ret = hu->curr_proto_desc->proto->id;
            break;
        }
        case HCIUARTGETDEVICE:
            BRCM_LDISC_DBG(V4L2_DBG_INIT,"GETDEVICE");
            ret = hu->hdev->id;
            break;
        default:
            ret = n_tty_ioctl_helper(tty, file, cmd, arg);
            break;
    }

    return ret;
}

/*******************************************************************************
**
** Function - brcm_hci_uart_tty_read
**
** Description - Reading interface for sh_ldisc driver. Not supported
**
*******************************************************************************/
static ssize_t brcm_hci_uart_tty_read(struct tty_struct *tty,
       struct file *file, unsigned char __user *buf, size_t nr)
{
    BRCM_LDISC_DBG(V4L2_DBG_RX, "%s", __func__);

    return -EOPNOTSUPP;
}

/*******************************************************************************
**
** Function - brcm_hci_uart_tty_write
**
** Description - Writing interface for sh_ldisc driver. Not supported
**
*******************************************************************************/
static ssize_t brcm_hci_uart_tty_write(struct tty_struct *tty,
                     struct file *file, const unsigned char *data, size_t count)
{
    BRCM_LDISC_DBG(V4L2_DBG_TX, "%s", __func__);

    return -EOPNOTSUPP;
}

/*******************************************************************************
**
** Function - brcm_hci_uart_tty_poll
**
** Description - Polling interface for sh_ldisc driver. Not supported
**
*******************************************************************************/
static unsigned int brcm_hci_uart_tty_poll(struct tty_struct *tty,
                    struct file *filp, poll_table *wait)
{
    BRCM_LDISC_DBG(V4L2_DBG_RX, "%s", __func__);

    return -EOPNOTSUPP;
}

static void brcm_hci_uart_tty_set_termios(struct tty_struct *tty,
                                                   struct ktermios *new_termios)
{
    struct ktermios *newktermios;
    BRCM_LDISC_DBG(V4L2_DBG_INIT, "new_termios->c_ispeed = %d, "\
        "new_termios->c_ospeed = %d",
        new_termios->c_ispeed, new_termios->c_ospeed);
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
    newktermios =  tty->termios;
#else
    newktermios =  &(tty->termios);
#endif
    newktermios->c_ispeed= tty_termios_input_baud_rate(newktermios);
    newktermios->c_ospeed = tty_termios_baud_rate(newktermios);
}

/*****************************************************************************
**   Module INIT interface
*****************************************************************************/
static int brcm_hci_uart_init( void )

{
    static struct tty_ldisc_ops hci_uart_ldisc;
    int err;

    BRCM_LDISC_DBG(V4L2_DBG_INIT, "HCI BRCM UART driver ver %s", VERSION);

    /* Registering this line discipline to tty */

    memset(&hci_uart_ldisc, 0, sizeof (hci_uart_ldisc));
    hci_uart_ldisc.magic        = TTY_LDISC_MAGIC;
    hci_uart_ldisc.name     = "n_brcm_hci";
    hci_uart_ldisc.open     = brcm_hci_uart_tty_open;
    hci_uart_ldisc.close        = brcm_hci_uart_tty_close;
    hci_uart_ldisc.read     = brcm_hci_uart_tty_read;
    hci_uart_ldisc.write        = brcm_hci_uart_tty_write;
    hci_uart_ldisc.ioctl        = brcm_hci_uart_tty_ioctl;
    hci_uart_ldisc.poll     = brcm_hci_uart_tty_poll;
    hci_uart_ldisc.receive_buf  = brcm_hci_uart_tty_receive;
    hci_uart_ldisc.write_wakeup = brcm_hci_uart_tty_wakeup;
    hci_uart_ldisc.owner        = THIS_MODULE;
    hci_uart_ldisc.set_termios = brcm_hci_uart_tty_set_termios;

    if ((err = tty_register_ldisc(N_BRCM_HCI, &hci_uart_ldisc)))
    {
        pr_err("HCI line discipline registration failed. (%d)", err);
        return err;
    }

    brcm_init();


    return 0;
}


/*****************************************************************************
**   Module EXIT interface
*****************************************************************************/
static void brcm_hci_uart_exit(struct hci_uart* hu)
{
    int err = 0;
    BRCM_LDISC_DBG(V4L2_DBG_INIT, "%s", __func__);

    brcm_deinit();

    /* Unregistering this line discipline from tty */
    if ((err = tty_unregister_ldisc(N_BRCM_HCI)))
        BRCM_LDISC_ERR("Can't unregister HCI line discipline (%d)", err);
}


/*******************************************************************************
**
** Function - hu_ref()
**
** Description - Retrieving the pointer to the core data of the designated
**             ST platform device. This would enable the support of multiple
**             platform devices on one signle platform
**
** Returns - void
**
*******************************************************************************/
#if 0
static void hu_ref(struct hci_uart**priv, int id)
{
    struct platform_device  *pdev;
    struct hci_uart *hu;
    /* get hu reference from platform device */
    pdev = brcm_plt_devices[id];
    if (!pdev) {
        *priv = NULL;
        return;
    }
    hu = dev_get_drvdata(&pdev->dev);
    *priv = hu;
}
#endif

/*******************************************************************************
**
** Function - brcm_ldisc_probe()
**
** Description - Called tor prepare the given device
**
** Returns - 0 if successful; a negative errno otherwise
**
*******************************************************************************/
static int brcm_ldisc_probe(struct platform_device *pdev)
{
    int rc = 0, pdev_index;
    struct hci_uart *hu;

    /*
     * Making suer this is the platform_device created by this driver
     */
    if (pdev == brcm_plt_device) {
        BRCM_LDISC_DBG(V4L2_DBG_INIT, "Binding the brcm_ldisc driver to the device %s", pdev->name);
    } else {
        BRCM_LDISC_ERR("Unknown device %s; skipped", pdev->name);
        return -EINVAL;
    }

    hu = kzalloc(sizeof(struct hci_uart), GFP_KERNEL);
    if (!hu) {
        BRCM_LDISC_ERR("no mem to allocate");
        rc = -ENOMEM;
        goto out;
    }
    dev_set_drvdata(&pdev->dev, hu);

    BRCM_LDISC_DBG(V4L2_DBG_INIT, "%s calling brcm_hci_uart_init ", __func__);

    /* register the tty line discipline driver */
    rc = brcm_hci_uart_init();
    if (rc) {
        BRCM_LDISC_ERR("%s: brcm_hci_uart_init failed\n", __func__);
        goto error_uart_init;
    }

    /* get reference of pdev for request_firmware */
    hu->brcm_pdev = pdev;
    init_completion(&hu->ldisc_installed);
    init_completion(&hu->tty_close_complete);

		spin_lock_init(&hu->component_descs_lock);

    rc = sysfs_create_group(&pdev->dev.kobj, &uim_attr_grp);
    if (rc) {
        BRCM_LDISC_ERR("failed to create sysfs entries");
        goto error_sysfs;
    }

    goto out;

error_sysfs:
    brcm_hci_uart_exit(hu);
error_uart_init:
    if (hu) {
        kfree(hu);
        dev_set_drvdata(&pdev->dev, NULL);
    }
out:
    return rc;
}

/*******************************************************************************
**
** Function - brcm_ldisc_remove()
**
** Description - Called to release the given device
**
** Returns - 0
**
*******************************************************************************/
static int brcm_ldisc_remove(struct platform_device *pdev)
{
    struct hci_uart* hu;
    BRCM_LDISC_DBG(V4L2_DBG_INIT, "brcm_ldisc_remove() unloading ");
#if V4L2_SNOOP_ENABLE
    if(ldisc_snoop_enable_param)
    {
        /* stop hci snoop to hcidump */
        if(nl_sk_hcisnoop)
            netlink_kernel_release(nl_sk_hcisnoop);
    }
#endif
    hu = dev_get_drvdata(&pdev->dev);
    sysfs_remove_group(&pdev->dev.kobj, &uim_attr_grp);
    brcm_hci_uart_exit(hu);

    kfree(hu);
    return 0;
}

#if 0
static const struct of_device_id brcm_ldisc_dt_ids[] = {
    { .compatible = "brcm_ldisc"},
    {}
};
#endif


static struct platform_driver brcm_ldisc_platform_driver = {
    //.probe = brcm_ldisc_probe,
    .remove = brcm_ldisc_remove,
    .driver = {
           .name = "brcm_ldisc",
           .owner = THIS_MODULE,
           //.of_match_table = brcm_ldisc_dt_ids,
           },
};

static int __init brcm_ldisc_init(void) {
    /*
     * Creating one fake platform device associated with this line discipline driver here
     * instead of via dts, as this fake device requires no properties/configuration; moreover,
     * multiple-device support is already inapplicable, due to the fact that only one device
     * (the one bound to index 0) would ever be used in the original implementation.
     *
     */

    /*
     * The device name and the driver name are made the same, so that they can be matched
     */
    int ret = 0;
    struct platform_device_info pdev_info = {
        .name = brcm_ldisc_platform_driver.driver.name,
        .id = PLATFORM_DEVID_NONE
    };
    brcm_plt_device = platform_device_register_full(&pdev_info);
    if (!brcm_plt_device) {
        ret = -ENOMEM;
    } else if (IS_ERR(brcm_plt_device)) {
        ret = PTR_ERR(brcm_plt_device);
    }

    if (ret) {
        BRCM_LDISC_ERR("Failed to register the device %s, errno = %d", pdev_info.name, ret);
    } else if ((ret = platform_driver_probe(&brcm_ldisc_platform_driver, brcm_ldisc_probe))) {
        BRCM_LDISC_ERR("Failed to register the driver %s, errno = %d", brcm_ldisc_platform_driver.driver.name, ret);
        platform_device_unregister(brcm_plt_device);
        brcm_plt_device = NULL;
    }

    return ret;
}

static void __exit brcm_ldisc_exit(void) {
    platform_driver_unregister(&brcm_ldisc_platform_driver);

    if (brcm_plt_device) {
        platform_device_unregister(brcm_plt_device);
        brcm_plt_device = NULL;
    }
}

module_init(brcm_ldisc_init);
module_exit(brcm_ldisc_exit);

/*****************************************************************************
**   Module Details
*****************************************************************************/

MODULE_PARM_DESC(reset, "Send HCI reset command on initialization");

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Line Discipline Driver for Shared Transport" \
                                                      "over UART ver ");
MODULE_VERSION(VERSION); /* defined in makefile */
MODULE_LICENSE("GPL");
MODULE_ALIAS_LDISC(N_BRCM_HCI);
