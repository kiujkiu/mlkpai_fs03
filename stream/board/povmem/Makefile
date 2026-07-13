# povmem.ko - write-combine mmap window for the POV frame reserve region.
#
# 需要板内核 (6.6.0-xilinx) 的 headers / 源码树 (本仓库不带, 见
# memory: mlkpai-kernel 自编流程)。两种编法:
#
#   1) 板上原生编 (板装了 linux-headers):
#        make
#   2) WSL 交叉编 (KDIR 指向配好 .config 并 make modules_prepare 过的
#      linux-xlnx 树):
#        make KDIR=~/mlkpai-kernel/linux-xlnx ARCH=arm \
#             CROSS_COMPILE=arm-linux-gnueabihf-
#
# 装载: insmod povmem.ko            (默认 base=0x10000000 size=0x1000000)
# 验证: ls -l /dev/povmem && dmesg | tail -1

obj-m := povmem.o

KDIR ?= /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean 2>/dev/null || \
	rm -f *.o *.ko *.mod *.mod.c *.mod.o .*.cmd Module.symvers modules.order

.PHONY: all clean
