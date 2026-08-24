#!/usr/bin/env python3
"""把 3-bit 帧目录拆成"每帧一个位平面"的 1-bit 目录 —— 圈级 BCM 实验用。

行内 BCM 是在**一屏之内**发 3 遍数据换灰度, 代价是整屏时间 x3、角分辨率掉到 1/3。
圈级 BCM 换个方向: 每屏只发 1 个位平面, 让**连续 3 圈**在视网膜上叠加成灰度。
角分辨率一点不掉, 代价换成闪烁 (完整图像的重复频率 = 转速/3)。

输出顺序 plane0(MSB) -> plane1 -> plane2(LSB), 与板端 --ring-bcm 轮转的
oe_window 权重 184/92/46 一一对应。⚠ 同步全靠不丢帧, flip<rx 时结果不可信。
"""
import json, os, shutil, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pack_obs

def main():
    src, dst = sys.argv[1], sys.argv[2]
    frame = int(sys.argv[3]) if len(sys.argv) > 3 else 0
    m = json.load(open(os.path.join(src, 'meta.json'), encoding='utf-8'))
    assert m['bpp'] == 3, f"源必须是 3-bit, 实际 bpp={m['bpp']}"
    raw = open(os.path.join(src, f'frame_{frame:04d}.bin'), 'rb').read()
    st3, st1 = pack_obs.slice_stride(3), pack_obs.slice_stride(1)
    n = len(raw) // st3
    os.makedirs(dst, exist_ok=True)
    for p in range(3):                       # plane p 已是 MSB-first 存放
        out = b''.join(raw[i * st3 + p * st1: i * st3 + (p + 1) * st1]
                       for i in range(n))
        open(os.path.join(dst, f'frame_{p:04d}.bin'), 'wb').write(out)
    m1 = dict(m)
    m1.update(bpp=1, frames=3, frame_raw=n * st1,
              geom_flags=m['geom_flags'] & ~0x80)   # 去掉 3BIT 标志
    json.dump(m1, open(os.path.join(dst, 'meta.json'), 'w', encoding='utf-8'),
              ensure_ascii=False, indent=1)
    print(f'{src} frame{frame} -> {dst}: 3 帧 x {n} 片 x 0x{st1:x} = {n*st1} B/帧')

main()
