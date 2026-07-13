// SPDX-License-Identifier: GPL-2.0
/*
 * povmem.c - /dev/povmem: write-combine mmap window over the POV frame
 * reserve region (Zynq-7020, MLKPAI-FS03, kernel 6.6.0-xilinx).
 *
 * 背景 (docs/design_icnd2047/04_sw_stream_26fps.md §3.4): Linux 以 mem=256M
 * 启动, phys 0x10000000..0x1FFFFFFF 是内核不可见的帧保留区。/dev/mem 对
 * !pfn_valid 的 pfn 给 pgprot_noncached = Strongly-Ordered, 每个 store 不可
 * 缓冲, memcpy 实效只有 ~60-120 MB/s (4.4 MB 帧要 37-73 ms)。本模块把同一
 * 区域以 pgprot_writecombine (Normal Non-Cacheable) 映射, 写可缓冲/合并,
 * 实效 300-800 MB/s (4.4 MB ≈ 6-15 ms)。
 *
 * 一致性: Normal-NC 不经 L1/L2, PL HP 口直读 DRAM 仍然看得到; 但 WC 弱序,
 * 用户态在翻页 (写 slice_base 寄存器) 前必须 DSB 排空 write buffer ——
 * pov_rxd 的 wmb_frame() 已覆盖。寄存器页不走本设备 (寄存器就该 SO)。
 *
 * 用法:
 *   insmod povmem.ko                       # 默认 base=0x10000000 size=16MB
 *   insmod povmem.ko base=0x10000000 size=0x1000000
 *   mmap(NULL, len, PROT_READ|PROT_WRITE, MAP_SHARED, fd, off)
 *     off = 目标物理地址 - base (页对齐), off+len 必须落在窗口内。
 *
 * 交叉编译 (板内核 headers 在手时, 见同目录 Makefile):
 *   make KDIR=/path/to/linux-xlnx ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>

static unsigned long base = 0x10000000UL;  /* 帧保留区物理基址 */
static unsigned long size = 0x01000000UL;  /* 窗口长度 (16 MB, 覆盖 3+ bank) */
module_param(base, ulong, 0444);
MODULE_PARM_DESC(base, "physical base of the POV frame reserve region");
module_param(size, ulong, 0444);
MODULE_PARM_DESC(size, "length of the mappable window in bytes");

static int povmem_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long len = vma->vm_end - vma->vm_start;
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;

	/* 只许映射窗口内部, 防越界打到别的物理地址 */
	if (off >= size || len > size - off)
		return -EINVAL;

	/* 核心: Normal-NC (write-combine) 而非 /dev/mem 的 Strongly-Ordered */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start,
			    (base + off) >> PAGE_SHIFT, len,
			    vma->vm_page_prot))
		return -EAGAIN;
	return 0;
}

static const struct file_operations povmem_fops = {
	.owner  = THIS_MODULE,
	.mmap   = povmem_mmap,
	.llseek = noop_llseek,
};

static struct miscdevice povmem_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "povmem",
	.fops  = &povmem_fops,
	.mode  = 0600,               /* root only, 跟 /dev/mem 一个待遇 */
};

static int __init povmem_init(void)
{
	int rc = misc_register(&povmem_dev);
	if (rc)
		return rc;
	pr_info("povmem: /dev/povmem WC window at 0x%08lx +0x%lx\n", base, size);
	return 0;
}

static void __exit povmem_exit(void)
{
	misc_deregister(&povmem_dev);
}

module_init(povmem_init);
module_exit(povmem_exit);

MODULE_AUTHOR("pov3d");
MODULE_DESCRIPTION("write-combine mmap window over the POV frame reserve region");
MODULE_LICENSE("GPL");
