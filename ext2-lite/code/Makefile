# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the linux ext2-lite fs module.
#
# Dimitris Siakavaras <jimsiak@cslab.ece.ntua.gr>

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
EXTRA_CFLAGS = -Wno-unused-variable -Wno-unused-label

obj-m += ext2-lite.o

ext2-lite-objs := balloc.o dir.o file.o ialloc.o inode.o namei.o super.o

all: modules

modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) $(KERNEL_VERBOSE) $(KERNEL_MAKE_ARGS) modules

clean: 
	$(MAKE) -C $(KERNELDIR) M=$(PWD) $(KERNEL_VERBOSE) $(KERNEL_MAKE_ARGS) clean
