#!/usr/bin/env python3
"""diag_fakespin.py — 板端诊断: 用 PL 的 fake-spin 代替真电机推进 slice 计数。

为什么需要它:
  POV 引擎默认是 **sensor 模式** (POV_CTRL[1] fake_en=0), slice_idx 由电机的
  index/hall 脉冲推进。电机不转时 slice_idx **恒为 0** —— 而 pov_rxd 的 flip
  线程正是靠轮询 slice_idx 找翻页窗, 于是:
      翻页窗 (slice<8) 永远命中不了「先离开再进入」的去抖条件
      -> 每帧都要等满 FLIP_TIMEOUT_MS=2000ms 才 FORCED 翻一次
      -> flip ≈ 0.95 帧/秒, 其余全部计入 drop。
  这就是历史上「丢帧 48-85%」的真身, 与网络无关 (本地空闲动画一样丢)。
  所以任何帧率测量都必须先确认引擎在转; 没有电机时用本脚本造一个。

与 pov_rxd --fake 的区别:
  pov_rxd --fake 会把 POV_CTRL 整个覆写成 (360<<16)|0x3, **抹掉 dual_en(bit2)**
  -> 屏B 不再取数, PL 侧 DDR 读带宽掉一半, 测出来的 memcpy 争用不真实。
  本脚本走 RMW: 只把 fake_en 置位, 其余位原样保留 (从 0x24 影子回读)。

用法:
  sudo python3 diag_fakespin.py 16.1     # 按 rps 开 fake-spin (967RPM ≈ 16.1)
  sudo python3 diag_fakespin.py off      # 关掉, 回 sensor 模式
  sudo python3 diag_fakespin.py status   # 只看状态 + 实测 slice 推进速率
"""
import mmap, os, sys, time

REG_BASE   = 0x40010000
ACLK_HZ    = 50_000_000
CTRL_FAKE  = 1 << 1

f = os.open('/dev/mem', os.O_RDWR | os.O_SYNC)
p = mmap.mmap(f, 0x1000, offset=REG_BASE)
rd = lambda o: int.from_bytes(p[o:o+4], 'little')
def wr(o, v): p[o:o+4] = (v & 0xFFFFFFFF).to_bytes(4, 'little')


def measure(dur=1.0):
    """采样 slice_idx, 回 (实测 rev/s, 不同取值个数)。wrap 计数 = 圈数。"""
    t0 = time.time(); last = rd(0x10) & 0xffff; wraps = 0; seen = set()
    while time.time() - t0 < dur:
        s = rd(0x10) & 0xffff
        seen.add(s)
        if s < last:
            wraps += 1
        last = s
    dt = time.time() - t0
    return wraps / dt, len(seen)


def show():
    ctrl = rd(0x24)
    n_sl = (ctrl >> 16) & 0xffff
    rps, distinct = measure(1.0)
    print('STATUS=0x%08X POV_CTRL_RB=0x%08X (n_slices=%d pov_en=%d fake_en=%d '
          'dual_en=%d fold_a=%d)' % (rd(0x00), ctrl, n_sl, ctrl & 1,
                                     (ctrl >> 1) & 1, (ctrl >> 2) & 1, (ctrl >> 6) & 1))
    print('fake_period(w)/rev_period(r) 0x14 = %d ticks' % rd(0x14))
    print('实测 slice 推进: %.2f rev/s, 1 秒内看到 %d 个不同 slice 值%s'
          % (rps, distinct, '  <-- 引擎没转!' if distinct <= 1 else ''))
    return rps


if __name__ == '__main__':
    arg = sys.argv[1] if len(sys.argv) > 1 else 'status'
    if arg == 'status':
        show()
    elif arg == 'off':
        ctrl = rd(0x24)
        wr(0x10, ctrl & ~CTRL_FAKE)
        print('fake_en 清除 -> 回 sensor 模式 (POV_CTRL <= 0x%08X)' % (ctrl & ~CTRL_FAKE))
        show()
    else:
        rps = float(arg)
        ctrl = rd(0x24)
        n_sl = (ctrl >> 16) & 0xffff or 360
        period = int(ACLK_HZ / (rps * n_sl) + 0.5)
        wr(0x14, period)
        wr(0x10, ctrl | CTRL_FAKE)
        print('fake-spin %.2f rps: n_slices=%d fake_period=%d ticks, '
              'POV_CTRL 0x%08X -> 0x%08X (RMW, dual_en 等位原样保留)'
              % (rps, n_sl, period, ctrl, ctrl | CTRL_FAKE))
        show()
