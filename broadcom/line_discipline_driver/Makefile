# Makefile for Line discipline driver

brcm_hci_ldisc-objs := brcm_hci.o brcm_sh_ldisc.o brcm_bluesleep.o
obj-$(CONFIG_BCM_COMBO_LINE_DISCIPLINE) := brcm_hci_ldisc.o

ccflags-y := -w -DVERSION="\"1.2\"" -I$(src)/../include
ccflags-$(CONFIG_BT_MSM_SLEEP) += -DLPM_BLUESLEEP
