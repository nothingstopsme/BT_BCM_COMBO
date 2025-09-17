# Makefile for V4L2 FM driver

fm_drv-objs := fmdrv_main.o fmdrv_v4l2.o fmdrv_rx.o
obj-$(CONFIG_BCM_COMBO_V4L2_FM) := fm_drv.o

ccflags-y := -w -DVERSION="\"1.2\"" -I$(src)/../include
