#!/bin/sh
# v2: 同一份探针, 只是提权 —— /dev/mem 需要 root, 登录身份是 uisrc。
# 仍然全程只读: O_RDONLY + PROT_READ, 不写寄存器/不写文件/不重启。
echo root | sudo -S -v 2>/dev/null   # 缓存凭据, 后续 sudo 免密
echo "===== [1] 现役开机脚本里的 0x0C 常量 (证据 A: 配的是什么) ====="
grep -n '0x0C' /home/uisrc/pov_boot.sh 2>/dev/null || echo '  (没有 /home/uisrc/pov_boot.sh)'
echo "--- 开机日志头 (确认跑的是哪个版本) ---"
head -3 /home/uisrc/pov_boot.log 2>/dev/null

echo
echo "===== [2] 寄存器回读 (证据 B: PL 里那个触发器现在是什么) ====="
sudo busybox devmem 0x40010000 32   # R 0x00 status
sudo busybox devmem 0x40010024 32   # R 0x24 POV_CTRL 影子 (用来确认 bit 版本)
sudo busybox devmem 0x40010014 32   # R 0x14 rev_period (转速, 用于换算)

echo
echo "===== [3] OE 行节拍实测 (证据 C: 与配置无关的行为测量) ====="
sudo python3 - <<'PY'
import mmap, os, time, statistics as st

# ---- 只读映射 ----
f = os.open('/dev/mem', os.O_RDONLY | os.O_SYNC)
p = mmap.mmap(f, 4096, offset=0x40010000, prot=mmap.PROT_READ)

st0 = int.from_bytes(p[0:4], 'little')
ctrl = int.from_bytes(p[0x24:0x28], 'little')
print('status(0x00) = 0x%08X   dclk_fast/ddr_slow(bit7)=%d  auto_en(bit4)=%d '
      'use_fb(bit5)=%d pov_en(bit9)=%d dual_en(bit11)=%d'
      % (st0, (st0 >> 7) & 1, (st0 >> 4) & 1, (st0 >> 5) & 1,
         (st0 >> 9) & 1, (st0 >> 11) & 1))
print('POV_CTRL影子(0x24) = 0x%08X  n_slices=%d  pov_en=%d dual_en=%d'
      % (ctrl, ctrl >> 16, ctrl & 1, (ctrl >> 2) & 1))
if not ((st0 >> 4) & 1):
    print('VERDICT: INCONCLUSIVE — auto_en=0, 引擎没在扫, OE 不动, 测不了行节拍')
    raise SystemExit

# ---- 高速采样 status[3] = oe_a_state (0=OE低=显示中) ----
N = 300000
t0 = time.perf_counter()
s = [p[0] for _ in range(N)]          # 只取 byte0, bit3 就在里面
t1 = time.perf_counter()
dt = (t1 - t0) / N * 1e6              # us / 样本

bits = [(v >> 3) & 1 for v in s]
runs_lo, runs_hi = [], []
cur, n = bits[0], 1
for b in bits[1:]:
    if b == cur:
        n += 1
    else:
        (runs_lo if cur == 0 else runs_hi).append(n)
        cur, n = b, 1
# 首尾两段不完整, 丢掉
for r in (runs_lo, runs_hi):
    if r:
        r.pop(0)
    if r:
        r.pop()

print('采样: N=%d  平均间隔 dt=%.3f us  (总 %.1f ms)' % (N, dt, (t1 - t0) * 1e3))
print('  低电平段(OE低=显示) 段数=%d' % len(runs_lo))
print('  高电平段(OE高=消隐) 段数=%d' % len(runs_hi))

if len(runs_lo) < 200 or len(runs_hi) < 200:
    print('VERDICT: INCONCLUSIVE — 跳变太少 (OE 可能常高/常低, 或采样被调度卡住)')
    raise SystemExit

mlo, mhi = st.median(runs_lo), st.median(runs_hi)
print('  低段中位 %.1f 样本 = %.2f us   高段中位 %.1f 样本 = %.2f us'
      % (mlo, mlo * dt, mhi, mhi * dt))

# 🔴 仪器自检: 采样间隔必须显著小于半个行周期, 否则会系统性低估跳变
#    -> 只会把"快"误判成"慢", 是有偏的, 所以宁可拒答
if mlo < 3 or mhi < 2:
    print('VERDICT: INCONCLUSIVE — 采样太慢 (dt=%.2fus), 段长不足 3 样本。'
          '这个方向的误差是有偏的(会把 fast 误报成 slow), 拒绝下结论。' % dt)
    raise SystemExit

t_low = mlo * dt
t_row = (mlo + mhi) * dt
print('  => 行周期 ~%.2f us, OE 低宽 ~%.2f us, 整屏(54行) ~%.1f us, 2D 刷新 ~%.0f Hz'
      % (t_row, t_low, t_row * 54, 1e6 / (t_row * 54)))

# 四种合法组合的预期 (aclk=50MHz, rows=54, 行周期=387+max(0,2*oe-303) 或 195+max(0,oe-111))
cand = {'FAST(50Mbps) + oe=111': (3.90, 2.22),
        'FAST(50Mbps) + oe=187': (5.42, 3.74),
        'SLOW(25Mbps) + oe=111': (7.74, 4.44),
        'SLOW(25Mbps) + oe=187': (9.16, 7.48)}
best = min(cand, key=lambda k: abs(cand[k][0] - t_row) / cand[k][0]
                              + abs(cand[k][1] - t_low) / cand[k][1])
err = abs(cand[best][0] - t_row) / cand[best][0]
print('  最接近: %s (期望 行%.2fus / 低%.2fus), 相对偏差 %.1f%%'
      % (best, cand[best][0], cand[best][1], err * 100))
print('VERDICT: %s' % (best if err < 0.20 else
                       'INCONCLUSIVE — 与任何已知组合都对不上 (偏差 %.0f%%)' % (err * 100)))
PY
echo "===== 探测结束 (全程只读) ====="
