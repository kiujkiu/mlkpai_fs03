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
 * 窗口默认大小 (v3.1 三缓冲, 跟 pov_rxd.c 文件头的帧区地址表对齐):
 *   0x10000000  bank A (翻页缓冲 0), 实际用 0..0x870000
 *   0x11000000  bank B (翻页缓冲 1), 实际用 0..0x870000   <- BANK_STRIDE 16 MB
 *   0x12000000  bank C (翻页缓冲 2), 实际用 0..0x870000   <- 2026-07-31 起启用
 * pov_rxd 一次 mmap 整个 [bank A 起, bank C 末) = FRAME_MAP_LEN =
 * 2*0x1000000 + 0x870000 = 0x2870000 (40.4 MB)。
 *
 * 🔴 这个默认值涨过两次, 每次都是**静默失效**, 所以写清楚原委:
 *   一帧最大 0x438000 (360 片) → 0x870000 (720 片双面) ⇒ 16 MB 不够 → 25.6 MB;
 *   双缓冲 → 三缓冲 (ACK 与翻页解耦, 见 pov_rxd §三缓冲) ⇒ 25.6 MB 又不够 → 41 MB。
 * 窗口不够时 mmap 返回 -EINVAL, pov_rxd **静默回落**到 /dev/mem 的
 * Strongly-Ordered 映射, 8.85 MB 的 bank 拷贝要 74-148 ms, 帧率预算直接归零 ——
 * 不报错, 只是慢 5-10 倍。povmem_init() 会 pr_warn, pov_rxd 启动也会打提示行。
 *
 * 故默认 size = 0x2900000 (41.0 MB) = 0x2870000 向上取整并留 ~0.6 MB 余量。
 * base+size = 0x12900000, 仍远在保留区上限 0x20000000 之内 (256MB 保留区)。
 *
 * 用法:
 *   insmod povmem.ko                       # 默认 base=0x10000000 size=0x2900000
 *   insmod povmem.ko base=0x10000000 size=0x1900000   # 只要双缓冲时可缩回
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

/* pov_rxd 一次映射的窗口长度: (FRAME_BANKS-1)*BANK_STRIDE + BANK_BYTES
 * = 2*0x1000000 + 0x870000 (三缓冲)。低于这个值 pov_rxd 的 mmap 必然失败。 */
#define POVMEM_MIN_SIZE   0x02870000UL     /* 40.4 MB, 见文件头地址表 */

static unsigned long base = 0x10000000UL;  /* 帧保留区物理基址 */
/* 0x2900000 = 41.0 MB: 覆盖 bank A 起 .. bank C 末 (0x2870000) 并留余量 */
static unsigned long size = 0x02900000UL;
module_param(base, ulong, 0444);
MODULE_PARM_DESC(base, "physical base of the POV frame reserve region (default 0x10000000)");
module_param(size, ulong, 0444);
MODULE_PARM_DESC(size, "length of the mappable window in bytes (default 0x2900000, >= 0x2870000)");

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
	/* 装小了不致命 (小窗口照样能映射), 但 pov_rxd 的整窗 mmap 会 -EINVAL
	 * 并静默回落到 /dev/mem SO 映射 -> 帧拷贝慢 5-10 倍。先在 dmesg 喊一声。 */
	if (size < POVMEM_MIN_SIZE)
		pr_warn("povmem: size=0x%lx < 0x%lx: pov_rxd's 40.4 MB triple-buffer mmap will fail and fall back to slow /dev/mem\n",
			size, POVMEM_MIN_SIZE);
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
