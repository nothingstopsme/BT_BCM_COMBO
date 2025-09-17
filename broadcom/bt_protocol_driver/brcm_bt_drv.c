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
*      Module Name : Bluetooth Protocol Driver
*
*      Description:     This driver module allows Mini HCI layer in Bluedroid to send/receive
*                            packets to/from bluetooth chip. This driver registers with
*                            Line discipline driver to communicate with bluetooth chip.
*
*******************************************************************************/
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include "brcm_ldisc_sh.h"
#include "brcm_bt_drv.h"


/* set this module parameter to enable debug info */
int bt_dbg_param = 0;


/* global variables */
struct brcm_bt_dev *bt_dev_p;        /* allocated in init_module */

static int is_print_reg_error = 1;

/* timers for ldisc to send callback on register complete */
static unsigned long jiffi1, jiffi2;


/* forward declaration */
static int brcm_bt_drv_open(struct inode *inode, struct file *filp);

static int brcm_bt_drv_close(struct inode *i, struct file *f);

static ssize_t brcm_bt_drv_read(struct file *f, char __user *buf, size_t
  len, loff_t *off);

static ssize_t brcm_bt_write(struct file *f, const char __user *buf,
  size_t len, loff_t *off);

static long brcm_bt_st_receive(struct sk_buff *skb);

static struct component_interface comp_interface = {
    .type = COMPONENT_BT,
    .recv = brcm_bt_st_receive,
};


/*****************************************************************************
**
** Function - brcm_bt_drv_open
**
** Description - Open call to BT protocol driver. Called when user-space program opens fd
**                    on BT protocol character driver. This function will register BT protocol driver
**                    to Line discipline driver if not already registered. Receives write function
**                    pointer from Line discipline driver.
**
** Returns - 0 if success; else errno
**
*****************************************************************************/
static int brcm_bt_drv_open(struct inode *inode, struct file *filp)
{
    struct brcm_bt_dev *bt_dev;

    /* register to ldisc driver */
    int err=0;
    unsigned long timeleft;
    unsigned long diff;
    struct sk_buff *tx_hdr_cache;

    bt_dev = container_of(inode->i_cdev, struct brcm_bt_dev, c_dev);

    if (test_and_set_bit_lock(BT_DEV_OPENING, &bt_dev->flags)) {
        BT_DRV_ERR("This device is being opened");
        return -EBUSY;
    }


    /* Already registered with Line discipline ST ? */
    if (test_bit(BT_ST_REGISTERED, &bt_dev->flags)) {
        BT_DRV_ERR("This device has been opened");
        err = -EINVAL;
        goto out;
    }

    filp->private_data = bt_dev;

    comp_interface.write = NULL;

    /* Register with ST layer */
    BT_DRV_DBG(V4L2_DBG_OPEN, "calling ldisc register");
    err = brcm_sh_ldisc_register(&comp_interface);

    if (err < 0) {
        if (is_print_reg_error) {
            BT_DRV_ERR("st_register failed %d", err);
            jiffi1 = jiffies;
            is_print_reg_error = 0;
        }
        else {
            jiffi2 = jiffies;
            diff = (long)jiffi2 - (long)jiffi1;
            if ( ((diff * HZ * 10) / HZ) >= 1000)
                is_print_reg_error = 1;
        }
        err = -EAGAIN;
        BT_DRV_DBG(V4L2_DBG_OPEN, "End ret=%d", err);
        goto out;
    }

    /* Do we have a proper ST write function pointer? */
    if (comp_interface.write == NULL) {
        BT_DRV_ERR("failed to get ST write func pointer");

        /* Undo registration with ST */
        err = brcm_sh_ldisc_unregister(COMPONENT_BT, err != -EBUSY);
        if (err < 0)
            BT_DRV_ERR("st_unregister failed %d", err);

        err = -EAGAIN;
        BT_DRV_DBG(V4L2_DBG_OPEN, "End ret=%d", err);
        goto out;
    }

out:
    if (!err)
        set_bit(BT_ST_REGISTERED, &bt_dev->flags);

    clear_bit_unlock(BT_DEV_OPENING, &bt_dev->flags);

    BT_DRV_DBG(V4L2_DBG_OPEN, "End ret=%d", err);
    return err;

}


/*****************************************************************************
**
** Function - brcm_bt_drv_close
**
** Description - Performs cleanup of BT protocol driver.
**
*****************************************************************************/
static int brcm_bt_drv_close(struct inode *i, struct file *f)
{
    int err=0;
    struct brcm_bt_dev *bt_dev_p = f->private_data;

    if (test_and_set_bit_lock(BT_DEV_CLOSING, &bt_dev_p->flags)) {
        BT_DRV_ERR("closing is in progress");
        return -EBUSY;
    }

    if (test_bit(BT_TX_ONGOING, &bt_dev_p->flags)
        || test_bit(BT_RX_ONGOING, &bt_dev_p->flags)) {
        BT_DRV_ERR("tx or rx is still ongoing");
        err = -EBUSY;
        goto out;
    }


    /* Unregister from ST layer */
    if (test_bit(BT_ST_REGISTERED, &bt_dev_p->flags)) {
        err = brcm_sh_ldisc_unregister(COMPONENT_BT, 1);
        if (err) {
            if (err != -EALREADY) {
                BT_DRV_ERR("%s st_unregister failed %d", __func__, err);
                goto out;
            } else {
                err = 0;
            }
        }
    }

    spin_lock(&bt_dev_p->rx_q.lock);
    __skb_queue_purge(&bt_dev_p->rx_q);
    spin_unlock(&bt_dev_p->rx_q.lock);

out:
    if (!err) {
        clear_bit(BT_ST_REGISTERED, &bt_dev_p->flags);
        wake_up_interruptible_all(&bt_dev_p->inq);
    }

    clear_bit_unlock(BT_DEV_CLOSING, &bt_dev_p->flags);

    BT_DRV_DBG(V4L2_DBG_CLOSE, "End ret=%d", err);
    return err;
}


/*****************************************************************************
**
** Function - brcm_bt_drv_read
**
** Description - Called when user-space program tries to read a packet.
**
** Returns - Number of bytes in packet.
*****************************************************************************/
static ssize_t brcm_bt_drv_read(struct file *f, char __user *buf, size_t
  len, loff_t *off)
{
    struct sk_buff *skb;
    struct brcm_bt_dev *bt_dev_p = f->private_data;
    ssize_t ret = 0, read_size;

    if (test_and_set_bit_lock(BT_RX_ONGOING, &bt_dev_p->flags)) {
        BT_DRV_ERR("reading is in progress");
        return -EBUSY;
    }

    if (test_bit(BT_DEV_CLOSING, &bt_dev_p->flags)
        || !test_bit(BT_ST_REGISTERED, &bt_dev_p->flags)) {
        BT_DRV_ERR("This device is not ready for rx");
        ret = -EINVAL;
        goto out;
    }


    spin_lock(&bt_dev_p->rx_q.lock);
    skb = skb_peek(&bt_dev_p->rx_q);
    spin_unlock(&bt_dev_p->rx_q.lock);

    if(!skb)
    {
        BT_DRV_ERR("No skb packet in rx queue. This should " \
            "not happen as user-space program should poll for data "\
            "availability before reading");

    }
    else {
        read_size = skb->len > len ? len : skb->len;

        BT_DRV_DBG(V4L2_DBG_RX, "Copying %d bytes to the user buffer", read_size);
        /* copy packet to user-space */
        if(copy_to_user(buf, skb->data, read_size)){
          BT_DRV_ERR("copy to user failed");
          ret = -EFAULT;
        }
        else {
          skb_pull(skb, read_size);
          ret = read_size;
          if (!skb->len) {
            /* free the skb after all its content has been copied to user space */
            spin_lock(&bt_dev_p->rx_q.lock);
            __skb_unlink(skb, &bt_dev_p->rx_q);
            spin_unlock(&bt_dev_p->rx_q.lock);
            kfree_skb(skb);
          }
        }
    }

out:
    clear_bit_unlock(BT_RX_ONGOING, &bt_dev_p->flags);
    return ret;

}


/*****************************************************************************
**
** Function - brcm_bt_write
**
** Description - Called when user-space program writes to fd.
**
** Returns - Number of bytes written.
*****************************************************************************/
static ssize_t brcm_bt_write(struct file *f, const char __user *buf,
  size_t buf_len, loff_t *off)
{
    ssize_t ret = buf_len;
    struct sk_buff *skb = NULL;
    struct brcm_bt_dev *bt_dev_p = f->private_data;

    int len;
    void *err_ptr;
    struct rx_tx_desc *curr_tx = &bt_dev_p->curr_tx;


    if (comp_interface.write == NULL) {
        BT_DRV_ERR("%s Error!!! comp_interface.write is NULL", __func__);
        return -ENXIO;
    }

    if (!buf || !buf_len) {
        BT_DRV_ERR("Error: Buffer from user space is NULL of its length is 0\n");
        return -EINVAL;
    }

    if (test_and_set_bit_lock(BT_TX_ONGOING, &bt_dev_p->flags)) {
        BT_DRV_ERR("writing is in progress");
        return -EBUSY;
    }

    if (test_bit(BT_DEV_CLOSING, &bt_dev_p->flags)
        || !test_bit(BT_ST_REGISTERED, &bt_dev_p->flags)) {
        BT_DRV_ERR("This device is not ready for tx");
        ret = -EINVAL;
        goto out;
    }

    while (buf_len) {
        if (!curr_tx->skb) {
            if (curr_tx->state == HCIBRCM_W4_PACKET_TYPE) {
                curr_tx->snoop_hdr.len = 1;

                /* Starting with a small buffer */
                curr_tx->skb = alloc_skb(16, GFP_KERNEL);
            }

            if (!curr_tx->skb) {
                BT_DRV_ERR("Failed to allocate a new rx buffer");
                ret = -ENOMEM;
                goto out;
            }
        }

        len = curr_tx->snoop_hdr.len - curr_tx->skb->len;
        if (len) {
            len = min(len, buf_len);

            if (copy_from_user(skb_put(curr_tx->skb, len), buf, len)) {
                BT_DRV_ERR("Error:Could not copy all requested data bytes from user space");
                ret = -EIO;
                goto out;
            }

            buf_len -= len; buf += len;

            if (curr_tx->skb->len < curr_tx->snoop_hdr.len)
                continue;
        }

        switch (curr_tx->state) {
            case HCIBRCM_W4_PACKET_TYPE: {
                switch (curr_tx->skb->data[0]) {
                    case HCI_COMMAND_PKT:
                        curr_tx->comp_type = COMPONENT_BT;
                        curr_tx->snoop_hdr.len += HCI_COMMAND_HDR_SIZE;
                        break;
                    case HCI_ACLDATA_PKT:
                        curr_tx->comp_type = COMPONENT_BT;
                        curr_tx->snoop_hdr.len += HCI_ACL_HDR_SIZE;
                        break;
                    case HCI_SCODATA_PKT:
                        curr_tx->comp_type = COMPONENT_BT;
                        curr_tx->snoop_hdr.len += HCI_SCO_HDR_SIZE;
                        break;
                    default:
                        BT_DRV_ERR("Unknown HCI tx packet type 0x%02x", curr_tx->skb->data[0]);
                        ret = -EINVAL;
                        goto out;
                }

                curr_tx->state = HCIBRCM_W4_HEADER;

                break;
            }
            case HCIBRCM_W4_HEADER: {
                const unsigned char *hdr_start = curr_tx->skb->data+1;
                __u32 expansion_size = 0;
                switch (curr_tx->skb->data[0]) {
                    case HCI_COMMAND_PKT: {
                        struct hci_command_hdr *cmd_hdr = (struct hci_command_hdr*)hdr_start;
                        BT_DRV_DBG(V4L2_DBG_TX, "COMMAND packet: opcode = 0x%04x, plen = %hhu",
                                      __le16_to_cpu(cmd_hdr->opcode), cmd_hdr->plen);
                        expansion_size = cmd_hdr->plen;
                        curr_tx->snoop_hdr.len += expansion_size;
                        break;
                    }
                    case HCI_ACLDATA_PKT: {
                        struct hci_acl_hdr *acl_hdr = (struct hci_acl_hdr*)hdr_start;
                        len = __le16_to_cpu(acl_hdr->dlen);
                        BT_DRV_DBG(V4L2_DBG_TX, "ACL packet: handle = 0x%04x, dlen = %hu",
                                      __le16_to_cpu(acl_hdr->handle), len);
                        expansion_size = len;
                        curr_tx->snoop_hdr.len += expansion_size;
                        break;
                    }
                    case HCI_SCODATA_PKT: {
                        struct hci_sco_hdr *aco_hdr = (struct hci_sco_hdr*)hdr_start;
                        BT_DRV_DBG(V4L2_DBG_TX, "SCO packet: handle = 0x%04x,  dlen = %hhu",
                                      __le16_to_cpu(aco_hdr->handle), aco_hdr->dlen);
                        expansion_size = aco_hdr->dlen;
                        curr_tx->snoop_hdr.len += expansion_size;
                        break;
                    }
                    default:
                        BT_DRV_ERR("An impossible branch was reached with an unknown tx packet type = %02x; aborting", curr_tx->skb->data[0]);
                        ret = -EINVAL;
                        goto out;
                }

                if (skb_tailroom(curr_tx->skb) < expansion_size) {
                    if (pskb_expand_head(curr_tx->skb, 0, expansion_size - skb_tailroom(curr_tx->skb), GFP_KERNEL)){
                        BT_DRV_ERR("Failed to expand the buffer size to accommodate more data");
                        ret = -ENOMEM;
                        goto out;
                    }
                }

                /* When curr_tx->skb->len == curr_tx->snoop_hdr.len,
                 * this tx skb is sent directly by falling through
                 * to the following case HCIBRCM_W4_DATA
                 */
                if (curr_tx->skb->len < curr_tx->snoop_hdr.len) {
                    curr_tx->state = HCIBRCM_W4_DATA;
                    break;
                }
            }
            case HCIBRCM_W4_DATA: {

                sh_ldisc_cb(curr_tx->skb)->comp_type = curr_tx->comp_type;
                BT_DRV_DBG(V4L2_DBG_TX, "Calling comp_interface.write() to write %d bytes", curr_tx->skb->len);
                err_ptr = comp_interface.write(curr_tx->skb, true /* never_blocking */);
                if (IS_ERR(err_ptr)) {
                    ret = PTR_ERR(err_ptr);
                    BT_DRV_ERR("Failed to pass this tx skb to ldisc: err = %d. Packet dropped", ret);
                    goto out;
                }

                /* This will also reset curr_tx->state to HCIBRCM_W4_PACKET_TYPE */
                memset(curr_tx, 0, sizeof(*curr_tx));

                break;
            }
            default: {
                BT_DRV_ERR("An impossible branch was reached with an unknown tx state = %d; aborting", curr_tx->state);
                ret = -EINVAL;
                goto out;
            }

        }

    }

out:
    if (ret < 0) {
        if (curr_tx->skb)
            kfree_skb(curr_tx->skb);

        memset(curr_tx, 0, sizeof(*curr_tx));
    }

    clear_bit_unlock(BT_TX_ONGOING, &bt_dev_p->flags);
    BT_DRV_DBG(V4L2_DBG_TX, "End with ret = %lld", ret);
    return ret;
}


/*****************************************************************************
**
** Function - brcm_bt_drv_poll
**
** Description - Called by Linux kernel when user-space programs calls select. Kernel system
*                      calls this function to check for data availability.
**
** Returns - Number of bytes written.
*****************************************************************************/
static unsigned int brcm_bt_drv_poll(struct file *filp,
                                         struct poll_table_struct *pwait)
{
    unsigned int mask = 0;
    struct brcm_bt_dev *bt_dev = filp->private_data;

    poll_wait(filp, &bt_dev->inq, pwait);

    smp_mb__before_atomic();

    if (test_bit(BT_DEV_CLOSING, &bt_dev_p->flags)
        || !test_bit(BT_ST_REGISTERED, &bt_dev_p->flags)) {
        BT_DRV_ERR("The device is not ready for polling");
        mask = POLLNVAL;
    } else {
      unsigned int queue_len;

      spin_lock(&bt_dev->rx_q.lock);
      queue_len = skb_queue_len(&bt_dev->rx_q);
      spin_unlock(&bt_dev->rx_q.lock);

      if (queue_len > 0) mask = POLLIN | POLLRDNORM;
    }

    BT_DRV_DBG(V4L2_DBG_RX, "mask = 0x%08x", mask);
    return mask;
}


/*  File operations which can be performed on this driver  */
static struct file_operations brcm_bt_drv_fops =
{
  .owner = THIS_MODULE,
  .open = brcm_bt_drv_open,
  .release = brcm_bt_drv_close,
  .read = brcm_bt_drv_read,
  .write = brcm_bt_write,
  .poll = brcm_bt_drv_poll
};


/* ------- Interfaces to Line discipline driver  ------ */

/*****************************************************************************
**
** Function - brcm_bt_st_receive
**
** Description - Called by Line discipline driver when receive data is available.
**
*****************************************************************************/
static long brcm_bt_st_receive(struct sk_buff *skb)
{
    unsigned int rx_q_len = 0;

    if (skb == NULL)
    {
        BT_DRV_ERR("Invalid SKB received from ST");
        return -EFAULT;
    }
    if (!bt_dev_p)
    {
        BT_DRV_ERR("Invalid hci_st memory,freeing SKB");
        return -EFAULT;
    }

    spin_lock(&bt_dev_p->rx_q.lock);
    __skb_queue_tail(&bt_dev_p->rx_q, skb);
    rx_q_len = skb_queue_len(&bt_dev_p->rx_q);
    spin_unlock(&bt_dev_p->rx_q.lock);

    wake_up_interruptible(&bt_dev_p->inq);

    BT_DRV_DBG(V4L2_DBG_RX, "rx_q len = %u", rx_q_len);


    return 0;
}


/* Global static variables */

static dev_t dev; /* Global variable for dev device number */


/* Driver module initialization and cleanup functions */
static int __init brcm_bt_drv_init(void) /* Constructor */
{
    int err=0;

    if ((err = alloc_chrdev_region(&dev, 0, 1, "brcm_bt_drv")) < 0)
    {
        BT_DRV_ERR("alloc_chrdev_region FAILED");
        return err;
    }
    BT_DRV_DBG(V4L2_DBG_INIT, "<Major, Minor>: <%d, %d>\n", MAJOR(dev), MINOR(dev));

    bt_dev_p = kzalloc(sizeof(struct brcm_bt_dev), GFP_KERNEL);

    if (!bt_dev_p)
    {
        err = -ENOMEM;
        BT_DRV_ERR("kmalloc FAILED");
        goto error_bt_dev;
    }

    if ((bt_dev_p->cl \
           = (struct class *)class_create(THIS_MODULE, "brcm_bt_drv")) == NULL)
    {
        err = -1;
        BT_DRV_ERR("class_create FAILED");
        goto error_class;
    }

    if (device_create(bt_dev_p->cl, NULL, dev, NULL, "brcm_bt_drv") \
            == NULL)
    {
        BT_DRV_ERR("device_create FAILED");
        err = -1;
        goto error_device;
    }

    cdev_init(&bt_dev_p->c_dev, &brcm_bt_drv_fops);
    if ((err = cdev_add(&bt_dev_p->c_dev, dev, 1)))
    {
        BT_DRV_ERR("cdev_add FAILED");
        goto error_cdev;
    }


    skb_queue_head_init(&bt_dev_p->rx_q);
    init_waitqueue_head(&bt_dev_p->inq);

    return 0;

error_cdev:
    device_destroy(bt_dev_p->cl, dev);
error_device:
    class_destroy(bt_dev_p->cl);
error_class:
    kfree(bt_dev_p);
    bt_dev_p = NULL;
error_bt_dev:
    unregister_chrdev_region(dev, 1);

    return err;

}


static void __exit brcm_bt_drv_exit(void) /* Destructor */
{
    if (bt_dev_p->curr_tx.skb)
        kfree_skb(bt_dev_p->curr_tx.skb);

    cdev_del(&bt_dev_p->c_dev);
    device_destroy(bt_dev_p->cl, dev);
    class_destroy(bt_dev_p->cl);
    unregister_chrdev_region(dev, 1);
    BT_DRV_DBG(V4L2_DBG_INIT, "Unregistering driver done");

    kfree(bt_dev_p);
    BT_DRV_DBG(V4L2_DBG_INIT, "Driver memory deallocated");

    BT_DRV_DBG(V4L2_DBG_INIT, "BT_protocol driver exited");
}

module_init(brcm_bt_drv_init);
module_exit(brcm_bt_drv_exit);

module_param(bt_dbg_param, int, S_IRUGO);
MODULE_PARM_DESC(ldisc_dbg_param, \
               "Set to integer value from 1 to 31 for enabling/disabling" \
               " specific categories of logs");


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Broadcom");
MODULE_VERSION(VERSION); /* defined in makefile */
MODULE_DESCRIPTION("Bluetooth driver for Bluedroid. \
 Integrates with Line discipline driver (Shared Transport)");



