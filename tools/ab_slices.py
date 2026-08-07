#!/usr/bin/env python3
"""ab_slices.py — 板上静态切换每转片数 (360 / 316 / …), 用于 A/B 对比面板扫描完整度。

    sudo python3 ab_slices.py <N> <bin>      # 例: sudo python3 ab_slices.py 316 groot_316.bin
    sudo python3 ab_slices.py --show         # 只读当前配置

背景 (2026-08-07): 面板 2D 刷新 4748 Hz, 900 RPM = 15 rps ⇒ 每转只扫得完
4748/15 = 316 遍。渲 360 片时每片只被扫 0.88 遍, 12% 的行不显示。把每转片数
降到 316 让"每片正好一次完整扫描", 数据还少 12%, 有效角分辨率不变 (1.14°)。

🔴 PHASE_B 的单位是**片号不是度** (pov_dual_top.v: idx_b = idx + phase_b_r)。
   360 片时 180 恰好既是"半圈"又是"180°", 两种解释重合, 所以这个坑一直没暴露;
   316 片下必须写 158。写 180 会偏 (180/316-0.5)*360 = 25.1°, 现象是
   "两个体各绕各的中心" (见 project_pov3d_v31_dualface_geometry_solved 的排错表)。
   RTL 另有硬约束 PHASE_B < n_slices —— 它只做一次条件减, 不是取模。

⚠ 本脚本只管**静态显示** (直接写 bank A)。要走推流, pov_rxd.c 的
   PHASE_B_DUAL 常量 (硬编码 180) 也得跟着 n_slices 走, 否则板端一推流就写回 180。
"""
import mmap, os, sys

REG_BASE   = 0x40010000
BANK_A     = 0x10000000
BANK_BYTES = 0x870000          # 720 片双面帧的最大尺寸
SLICE_SZ   = 0x3000


def open_regs(f):
    return mmap.mmap(f, 4096, offset=REG_BASE)


ACLK_HZ = 50_000_000


def show(p):
    """🔴 0x18/0x28 **没有读口** —— 读它们拿到的是别的东西 (0x18 读口是
    {locked_ever, slice_max}, 0x28 落到 default 分支 = status)。别拿读回来的值
    当"写进去的基址"核对, 那是 [[feedback_verify_signal_actually_connected]]
    那个坑。能核对基址的只有 status[16] = base_b_act (slice_base_b != 0)。"""
    rd = lambda o: int.from_bytes(p[o:o + 4], 'little')
    ctrl, st = rd(0x24), rd(0x00)         # 0x24 = POV_CTRL 影子 (0x10 是只写)
    n = (ctrl >> 16) & 0xffff
    r10, rev = rd(0x10), rd(0x14)

    print(f'  POV_CTRL(0x24) = 0x{ctrl:08X}  n_slices={n}  pov_en={ctrl & 1} '
          f'dual_en={(ctrl >> 2) & 1} mirror_b={(ctrl >> 4) & 1} '
          f'mirror_a={(ctrl >> 5) & 1} fold_a={(ctrl >> 6) & 1}')
    print(f'  status(0x00)   = 0x{st:08X}  base_b_act={(st >> 16) & 1} '
          f'dual_en={(st >> 11) & 1} pov_en={(st >> 9) & 1} locked={(st >> 8) & 1} '
          f'overlap={(st >> 6) & 1} use_fb={(st >> 5) & 1} auto={(st >> 4) & 1}')
    print(f'  PHASE_B(0x1C)  idx_b_live={(rd(0x1C) >> 1) & 0x1ff}  pair_miss={rd(0x1C) & 1}')
    if rev and rev != 0xFFFFFFFF:
        print(f'  转速: rev_period(0x14) = {rev} 拍 @{ACLK_HZ // 1000000}MHz = '
              f'{rev / ACLK_HZ * 1000:.1f} ms/圈 = {ACLK_HZ / rev:.2f} rps = '
              f'{ACLK_HZ / rev * 60:.0f} RPM')
    else:
        print(f'  转速: rev_period(0x14) = {rev} (未测到整圈 ⇒ 没转)')
    print(f'  slice(0x10) = {r10 & 0xffff}  locked={(r10 >> 31) & 1}   '
          f'slice_max(0x18) = {rd(0x18) & 0xffff}')
    return n


def main():
    if len(sys.argv) >= 2 and sys.argv[1] == '--show':
        f = os.open('/dev/mem', os.O_RDWR | os.O_SYNC)
        show(open_regs(f))
        return

    if len(sys.argv) != 3:
        sys.exit(__doc__)
    n = int(sys.argv[1])
    path = sys.argv[2]

    need = n * 2 * SLICE_SZ                       # 双面: [面A n 片][面B n 片]
    if need > BANK_BYTES:
        sys.exit(f'✗ {n} 片双面需 {need} B > bank 容量 {BANK_BYTES} B')
    if n > 511:
        sys.exit(f'✗ PHASE_B 是 9 bit, n/2={n // 2} 存不下 (n 须 ≤ 511*2)')
    sz = os.path.getsize(path)
    if sz != need:
        sys.exit(f'✗ {path} 是 {sz} B, 与 {n} 片双面应有的 {need} B 不符 '
                 f'(该文件对应 {sz // SLICE_SZ // 2} 片/面)')

    phase_b = n // 2                              # 半圈 = 180°, 单位是片
    if n % 2:
        print(f'⚠ n={n} 是奇数, PHASE_B 取 {phase_b} 片 = '
              f'{phase_b * 360 / n:.2f}° (差 {abs(180 - phase_b * 360 / n):.2f}°)')

    f = os.open('/dev/mem', os.O_RDWR | os.O_SYNC)
    p = open_regs(f)

    # 1) 数据进 bank A (尾部清零, 免得上一套更长的内容残留被当成有效片)
    ba = mmap.mmap(f, BANK_BYTES, offset=BANK_A)
    with open(path, 'rb') as fh:
        d = fh.read(BANK_BYTES)
    ba[:len(d)] = d
    ba[len(d):] = b'\0' * (BANK_BYTES - len(d))
    ba.close()
    print(f'[data] {path} -> bank A, {len(d)} B ({n} 片/面 × 2 面)')

    # 2) 寄存器 (顺序: 先基址后 CTRL, 让引擎切过去时两个基址已经对)
    def pw(off, v):
        p[off:off + 4] = v.to_bytes(4, 'little')

    pw(0x18, BANK_A)
    pw(0x28, BANK_A + n * SLICE_SZ)
    pw(0x1C, phase_b)
    pw(0x10, (n << 16) | 0x5)                     # pov_en(bit0) + dual_en(bit2)
    print(f'[reg ] n_slices={n}  PHASE_B={phase_b} 片 (={phase_b * 360 / n:.1f}°)  '
          f'slice_base_b=0x{BANK_A + n * SLICE_SZ:08X}')
    print('[回读]')
    got = show(p)
    print('✓ 一致' if got == n else f'✗ 回读 n_slices={got} != {n}')


if __name__ == '__main__':
    main()
