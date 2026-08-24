#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
povstream.py — PC 侧 POV 体显示推流器 (PVS1 协议, 见 protocol.md / ../protocol.h).

frame = n_slices × 片距 (pack_obs 硬件实测映射, 不可改), 传统单面 360 片
1-bit = 4,423,680B。**帧长不再是常量** (2026-07-31 v3.1 偏心屏): 头里的
n_slices 才是权威, raw_len = n_slices × 片距, 见下面 --dual-face / --fold-a。
**片距也不再是常量** (2026-08-20 v3.4 3-bit): --bpp 1 → 0x3000 (默认, 老行为
逐字节不变) / --bpp 3 → 0x9000 = 一片里 3 个位平面 (行内 BCM, 权重 27/54/108
沿 = 1:2:4), 帧头置 PVS_FLAG_3BIT, 板端按 flags 推片距。见 05_3bit_bcm.md。
管线: 动画源 → 逐帧点云变换 → 体素化 → 切片渲染 → 量化抖动
(1-bit: Bayer 阈值 / 3-bit: gamma 解码 + 8 级 + 残差 Bayer; 相位都随
slice+frame 双变化, 时间抖动平滑) → pack → 压缩 → TCP → 板 ACK.
压缩用 --codec 选: zlib (默认, 老行为逐字节不变) / lz4 (LZ4 raw block,
压缩比几乎相同, 板端 A9 解压快 4 倍 → 48 fps, 见 protocol.h PVS_FLAG_LZ4)。
--stream-split balanced 再把载荷按板端两核的**工作量**切成多条独立流
(PVS_FLAG_MSTREAM), fold540 下 180/90/270 三条, makespan 20.6 → 15.47 ms。

numpy 现渲 ~秒级/帧, 正常流程先 render 预渲染到磁盘再 stream:

  python3 povstream.py render --anim spinpulse --frames 8 --render-slices 90
  python3 povstream.py render --anim glbseq --glb-dir my_seq/ --render-slices 90
  python3 povstream.py render --anim glbanim --glb walk.glb --anim-take 0 --frames 12
  python3 povstream.py stream --dir frames_spinpulse --host <board> --fps 10 --loop
  python3 povstream.py stream --anim globe --frames 60 --loop   # 现渲直推 (慢)
  python3 povstream.py bench                                    # 压缩测量

3-bit 色深 (v3.4, 每通道 0..7, 行内 BCM):
  python3 povstream.py render --anim spin --bpp 3 --n-slices 60  # 60 片 = 2.2MB/帧
  python3 povstream.py stream --dir frames_spin --host <board>   # bpp 从 meta.json 读
  python3 tools/gen_wedge.py --bpp 3 --out wedge.bin             # 8 级灰度楔 (上板目视)

v3.1 偏心屏 (两面垂距 0 / 13.4mm, 不再对称 ⇒ 不能共用一份数据):
  python3 povstream.py render --anim spin --dual-face           # 720 片 = 8.8MB/帧
  python3 povstream.py render --anim spin --dual-face --fold-a  # 540 片 = 6.6MB/帧
  python3 povstream.py render --anim spin --face-off-mm 0       # 只驱动穿心面A
不给这些参数 = v3 对称几何老路径, 输出逐字节不变 (frames_* 老目录继续可用)。
DUAL_FACE 帧的线上 payload = [u32 comp_len_A][面A 流][面B 流] 两条独立压缩流
(板端两核并行 inflate, 单帧解码时间减半); 单面帧排布不变, 无前缀。
推流默认发送窗口 --window 2 (2 帧在途, 传输与板端解码重叠)。

动画源:
  spinpulse: anime GLB 点云 + 呼吸缩放 ±5% + 上下浮动 + 披风 x-shear 摆动
  globe:     程序化经纬球点云, NASA 贴图大陆, 逐帧自转
  glbseq:    --glb-dir 目录内 sorted *.glb 逐文件一帧 (外部工具烘的帧序列),
             全序列共用 bbox 归一 (防帧间 jitter), 逐文件采样结果 npz 缓存
  glbanim:   --glb 单文件真 glTF 动画 (骨骼 skinning / morph targets / 节点
             TRS), --anim-take 选 take, --frames N 均匀采样 timeline
             (t = k/N × duration, 首尾相接可 loop), 见 glb_anim.py
  spin:      任意静态 GLB 绕竖轴 (y) 自转, --frames = 一圈; --scale 整体
             缩放 (地球仪用 0.48 = 直径半幅), 贴图色要过 1-bit 用
             --lighting none + brighten/gamma/saturation 全 1.0
  palace:    程序化紫禁城空中巡游 (palace.py): 绕 y 轴一整圈 yaw +
             zoom 0.75→1.15→0.75 正弦一循环 (拉远/拉近), 红墙金顶
             白玉台/金水河/角楼, 全 1-bit 纯色, 首尾相接可 loop
  notredame: 程序化巴黎圣母院空中巡游 (notredame.py), 同 palace 巡游
             (共用 orbit_frames): 白石双塔/玫瑰窗(蓝盘红心)/铜绿坡顶/
             飞扶壁/交叉点尖塔(黄尖)/半圆后殿, 哥特竖直感, 全 1-bit 纯色
  (stream --dir = 预渲染 .bin 目录, 即任务里的 'file' 源)
"""
import os
import sys
import math
import time
import glob
import json
import zlib
import socket
import struct
import hashlib
import collections
from PIL import Image
import argparse
import threading
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.abspath(os.path.join(HERE, '..', '..', 'tools'))
sys.path.insert(0, TOOLS)
import pack_obs
import gen_anime_slices as gas

# ---- PVS1 协议常量 (= stream/protocol.h) ----
MAGIC = b'PVS1'
N_SLICES = 360                                        # 传统单面帧片数 (默认值)
# ---- 色深 (2026-08-20 v3.4, docs/design_icnd2047/05_3bit_bcm.md) ----
# BPP=1: 一片 = 0x3000, 与历史逐字节一致 (默认, 空闲动画/板上默认内容都靠它)。
# BPP=3: 每通道 3 bit, 一片 = 三个位平面 = 0x9000 (plane p 在 slice_base+p*0x3000)。
# 🔴 片距不再是常量: **凡是算帧长/片数的地方一律用模块全局 SLICE_STRIDE**,
#    别再写 pack_obs.SLICE_STRIDE (那是 1-bit 的兼容常量)。板端同款约定 =
#    protocol.h 的 PVS_STRIDE(flags)。
BPP = 1
SLICE_STRIDE = pack_obs.slice_stride(BPP)             # 0x3000 / 0x9000
FRAME_RAW = N_SLICES * SLICE_STRIDE                   # 4423680 (传统单面帧长)
# 🔴 硬上限是**字节数** (板端 staging 缓冲/DDR bank 间距), 不是片数:
#    1-bit 720 片 × 0x3000 = 3-bit 240 片 × 0x9000 = 8847360B。
FRAME_RAW_MAX = 0x1500000                             # = PVS_FRAME_RAW_MAX (21 MB)
# 🔴 2026-08-24 从 8847360 抬到 0xA00000: 半屏扫描后每圈画得出 283 槽, 旧上限
# 只够 3-bit 240 片。必须与 protocol.h 和 pov_boot.sh 的 povmem size 三处同改
# (povmem 那个是手写常量, 漏改 = mmap 覆盖不到 bank C 尾部 = 静默越界写)。
N_SLICES_MAX = FRAME_RAW_MAX // SLICE_STRIDE          # 1-bit 720 / 3-bit 240
N_SLICES_FOLD = 180                                   # --fold-a 折叠后的面A 片数


def _apply_bpp(bpp):
    """把色深落到模块全局。**必须在 _apply_slot_count 和任何渲染/打包之前调用。**

    只改「一片有多大」和「码值有几级」两件事: 面拆分、MSTREAM 流表、DELTA、
    压缩位、槽↔角度映射全部与 1-bit 一模一样 (protocol.h §v3.4)。
    bpp=1 时本函数是恒等变换 ⇒ 1-bit 输出逐字节不变。"""
    global BPP, SLICE_STRIDE, N_SLICES_MAX, FRAME_RAW
    BPP = bpp
    SLICE_STRIDE = pack_obs.slice_stride(bpp)
    N_SLICES_MAX = FRAME_RAW_MAX // SLICE_STRIDE
    FRAME_RAW = N_SLICES * SLICE_STRIDE


def _apply_slot_count(ns):
    """把「一圈多少槽」落到模块全局。**必须在任何渲染/打包之前调用。**

    为什么要能改: 面板实测 2D 刷新只有 ~1340 Hz, 15 rps 下每圈只画得出 ~89 个
    不同角度 (见 project_pov3d_refresh_vs_rpm 的 oe 扫描)。渲 360 片里有 3/4
    根本没机会上屏, 而链路/解码/memcpy 为它们付的代价是实打实的。
    协议侧本来就支持任意片数 (protocol.h:37 「头里的 n_slices 是权威」, 1..720),
    所以这是**纯 PC 侧改动, 板端零改动**。
    """
    global N_SLICES, FRAME_RAW, N_SLICES_FOLD
    N_SLICES = ns
    FRAME_RAW = N_SLICES * SLICE_STRIDE
    N_SLICES_FOLD = N_SLICES // 2      # --fold-a: 穿心面只渲半圈
FLAG_RLE, FLAG_ZLIB, FLAG_DELTA = 0x0001, 0x0002, 0x0004
FLAG_DUAL_FACE, FLAG_FOLD_A = 0x0008, 0x0010          # v3.1 偏心屏 (protocol.h)
FLAG_LZ4 = 0x0020                                     # v3.3 LZ4 raw block (与 ZLIB 互斥)
FLAG_MSTREAM = 0x0040                                 # v3.3 多流流表 (取代双面 4B 前缀)
FLAG_3BIT = 0x0080                                    # v3.4 每通道 3-bit (片距 0x9000)
FLAG_GEOM = FLAG_DUAL_FACE | FLAG_FOLD_A              # 描述载荷**几何**排布的位
# 每帧 flags 恒 OR 上的"这一帧长什么样"的位 = 几何 + 色深 (与压缩位正交)。
# 🔴 3BIT 必须在这里面: 板端 PVS_STRIDE(flags) 靠它推片距, 漏了就整帧错位。
FLAG_LAYOUT = FLAG_GEOM | FLAG_3BIT
# 能做 DELTA 的编解码 (板端 face_decode 先解码再 XOR, 与压缩位正交)
DELTA_CODECS = ('zlib', 'lz4')
# LZ4_compress_HC 级别。默认 12: 同一份 anime_dual720.bin 整帧单流实测
#   HC9 388166B(22.79x) / HC10 413178B(21.41x) / HC11 381889B / HC12 370699B(23.87x)
# ⚠ HC10 **比 HC9 还差 6.4%**, 可复现, 不是噪声 —— 别用 10。
# HC12 甚至比 zlib-6 (377009B) 还小, 但编码 ~1s/帧 ⇒ 只能走 --dir 离线预压缩。
DEFAULT_LZ4_LEVEL = 12
LZ4_BAD_LEVELS = (10,)                                # 实测反而更差的级别, 用到就告警
DEC_WORKERS = 2                                       # 板端解码线程数 (pov_rxd.c DEC_WORKERS)
ACK, NAK = 0x06, 0x15
# ⚠ 帧头 n_slices = **本帧载荷里的片数** (360/540/720), 与 PL 寄存器 POV_CTRL
#   里的 n_slices 字段不是一回事 —— 显示引擎每转仍然扫 360 片, 折叠面的
#   180..359 是 PL 用 idx-180 + 镜像置换现补出来的。
HDR = struct.Struct('<4sIIHH')                        # 16B
PAD = pack_obs.PLANE_PAD                              # 624B: 每个 plane 尾部补零
DEFAULT_PORT = 9500
DEFAULT_KEYINT = 26                                   # 关键帧周期 (发送端策略, 不进协议)
DEFAULT_WINDOW = 2                                    # 发送窗口 (帧在途上限), 见 --window
DEFAULT_LINK_MBPS = 28.0                              # 2.4G WiFi 实测链路估值
CACHE_WARN_MB = 1024                                  # 预压缩缓存内存告警线


# ================= 压缩 =================

def rle_encode(data):
    """零游程 RLE: 0x00 → [0x00][run:u16le], 其余字节字面直传."""
    a = np.frombuffer(data, np.uint8)
    out = bytearray()
    pos = 0
    for j in np.flatnonzero(a):
        j = int(j)
        run = j - pos
        while run > 0:
            r = min(run, 65535)
            out += b'\x00' + r.to_bytes(2, 'little')
            run -= r
        out.append(int(a[j]))
        pos = j + 1
    run = len(a) - pos
    while run > 0:
        r = min(run, 65535)
        out += b'\x00' + r.to_bytes(2, 'little')
        run -= r
    return bytes(out)


def rle_decode(data):
    out = bytearray()
    i, n = 0, len(data)
    while i < n:
        b = data[i]
        if b == 0:
            out += b'\x00' * int.from_bytes(data[i + 1:i + 3], 'little')
            i += 3
        else:
            out.append(b)
            i += 1
    return bytes(out)


# ---- LZ4 raw block (ctypes → liblz4.so) ------------------------------------
# 🔴 这里刻意**不**用 python-lz4 的 lz4.frame, 也不 shell out 到 `lz4` 命令:
#    那两个出的是 .lz4 **帧格式** (魔数 0x184D2204 + 帧描述符 + 分块头 + xxhash),
#    板端的 LZ4_decompress_safe() 吃不了 —— 它只认 raw block。见 protocol.h。
#    ctypes 直接调 liblz4 的 LZ4_compress_HC()/LZ4_decompress_safe(), 出入的就是
#    raw block, 与板端逐字节同一套格式, 且不引入新的 pip 依赖。
# WSL 里没有 liblz4.so 时: `sudo apt-get install -y liblz4-1` (只要运行时,
# 不需要 liblz4-dev —— ctypes 不看头文件)。
# ⚠ Windows 侧 (POV_Studio.bat 用的 Python312) 默认**没有** liblz4.dll,
# --codec lz4 在那边会直接报错退出。推流入口目前在 WSL, 先这样; 真要在
# Windows 上用, 把一个 liblz4.dll 放进 PATH 即可 (名字见下)。
_LZ4_SONAMES = ('liblz4.so.1', 'liblz4.so', 'liblz4.dll', 'lz4.dll')
_lz4_lib = None


def _lz4():
    """惰性加载 liblz4 并绑好签名; 只在真的用 lz4 时才要求它存在。"""
    global _lz4_lib
    if _lz4_lib is not None:
        return _lz4_lib
    import ctypes
    import ctypes.util
    lib = None
    for name in _LZ4_SONAMES + ((ctypes.util.find_library('lz4') or ''),):
        if not name:
            continue
        try:
            lib = ctypes.CDLL(name)
            break
        except OSError:
            continue
    if lib is None:
        sys.exit('找不到 liblz4.so —— 装一下运行时库: '
                 'sudo apt-get install -y liblz4-1')
    c = ctypes
    lib.LZ4_versionString.restype = c.c_char_p
    lib.LZ4_compressBound.argtypes = [c.c_int]
    lib.LZ4_compressBound.restype = c.c_int
    # int LZ4_compress_HC(const char* src, char* dst, int srcSize,
    #                     int dstCapacity, int compressionLevel)
    lib.LZ4_compress_HC.argtypes = [c.c_char_p, c.c_char_p, c.c_int, c.c_int, c.c_int]
    lib.LZ4_compress_HC.restype = c.c_int
    # int LZ4_decompress_safe(const char* src, char* dst, int compressedSize,
    #                         int dstCapacity)
    lib.LZ4_decompress_safe.argtypes = [c.c_char_p, c.c_char_p, c.c_int, c.c_int]
    lib.LZ4_decompress_safe.restype = c.c_int
    _lz4_lib = lib
    return lib


def lz4_compress(data, level=DEFAULT_LZ4_LEVEL):
    """LZ4 raw block (LZ4_compress_HC)。level 1..12, 9 = 实测选定的 HC9。"""
    import ctypes
    lib = _lz4()
    n = len(data)
    cap = lib.LZ4_compressBound(n)
    if cap <= 0:
        raise ValueError(f'LZ4_compressBound({n}) = {cap} (输入过大?)')
    dst = ctypes.create_string_buffer(cap)
    got = lib.LZ4_compress_HC(data, dst, n, cap, int(level))
    if got <= 0:
        raise ValueError(f'LZ4_compress_HC 失败 rc={got}')
    return dst.raw[:got]


def lz4_decompress(data, max_out):
    """LZ4 raw block → 原数据。raw block 里**不带原长**, dstCapacity 必须由
    调用方给 (线上是 hdr.raw_len / 各面的 nX*片距)。"""
    import ctypes
    lib = _lz4()
    dst = ctypes.create_string_buffer(int(max_out))
    got = lib.LZ4_decompress_safe(data, dst, len(data), int(max_out))
    if got < 0:
        raise ValueError(f'LZ4_decompress_safe 失败 rc={got} '
                         f'(载荷不是 raw block? .lz4 帧格式会解成负数)')
    return dst.raw[:got]


def _encode(data, codec, zlevel, lz4_level=DEFAULT_LZ4_LEVEL):
    if codec == 'zlib':
        return zlib.compress(data, zlevel), FLAG_ZLIB
    if codec == 'lz4':
        return lz4_compress(data, lz4_level), FLAG_LZ4
    if codec == 'rle':
        return rle_encode(data), FLAG_RLE
    return data, 0


def _decode(data, flags, max_out=None):
    if flags & FLAG_ZLIB:
        return zlib.decompress(data)
    if flags & FLAG_LZ4:
        # raw block 不自带原长 → 给一个上界缓冲, 按返回值截断
        return lz4_decompress(data, FRAME_RAW_MAX if max_out is None else max_out)
    if flags & FLAG_RLE:
        return rle_decode(data)
    return data


# DUAL_FACE 双流前缀: [u32 LE comp_len_A][面A 流][面B 流]
DUAL_HDR = struct.Struct('<I')
# MSTREAM 流表: [u32 n_streams][n × {u32 comp_len_i, u32 n_slices_i}][流…]
MSTR_N = struct.Struct('<I')
MSTR_E = struct.Struct('<II')
MAX_STREAMS = 16                                      # = protocol.h PVS_MAX_STREAMS


def stream_plan(n_slices, geom_flags, mode='face', workers=DEC_WORKERS):
    """→ 每条流的**片数**列表; None = 用老排布 (单流 / 按面两流), 逐字节不变。

    mode='face'     : 恒返回 None = 老行为 (默认, 保住所有已有 frames_* 的等价性)。
    mode='balanced' : 在面边界的基础上再按 w/workers 的累计片数位置切一刀, 让
                      板端**连续分组**后每个核拿到的片数尽量相等。

    为什么不能只按面切 (fold540 = 面A 折 180 + 面B 360 的实测):
        按面切 180/360  两核 makespan = 20.6 ms (被面B 的 360 片封顶)
        朴素三分 180×3  还是 20.6 ms (3 条流放 2 个核 = 2+1, 零收益)
        均衡三分 180/90/270 = 15.47 ms (两核各 270 片)
    ⚠ 由此也纠正一个旧说法: FOLD_A **只省链路 (−31%), 解码一分钱不省** ——
      按面切的 makespan 由面B 封顶, 折不折叠都一样。

    切分结果与按面切完全相同时 (例如 720 片双面 = 360/360 本来就平衡) 返回
    None, 退回老格式 —— 这样老固件照样能收, 兼容性最大化。"""
    n_a = N_SLICES_FOLD if geom_flags & FLAG_FOLD_A else N_SLICES
    faces = ([n_a, n_slices - n_a] if geom_flags & FLAG_DUAL_FACE
             else [n_slices])
    if mode == 'face':
        return None
    if mode != 'balanced':
        raise ValueError(f'未知的流切分模式 {mode!r}')
    cuts, acc = set(), 0
    for f in faces[:-1]:            # 面边界一定是流边界 (每条流只属于一个面)
        acc += f
        cuts.add(acc)
    for w in range(1, workers):     # 再按工作量等分点切
        cuts.add(n_slices * w // workers)
    cuts.discard(0)
    cuts.discard(n_slices)
    segs, prev = [], 0
    for b in sorted(cuts) + [n_slices]:
        segs.append(b - prev)
        prev = b
    if segs == faces:               # 与按面切一样 → 用老格式 (老固件也能收)
        return None
    if len(segs) > MAX_STREAMS:
        raise ValueError(f'流数 {len(segs)} > {MAX_STREAMS} (protocol.h 上限)')
    return segs


def compress_frame(raw, codec, zlevel, split=None, lz4_level=DEFAULT_LZ4_LEVEL,
                   streams=None):
    """→ (payload, codec_flags)。

    streams=[片数, …]: **MSTREAM 多流** (protocol.h bit6), 优先级最高
        payload = [u32 n][n×{u32 comp_len_i, u32 n_slices_i}][流 0..n-1]
        流数可变, 边界必须落在片边界上; 板端按片数把流连续分组摊到两个核。
        表里两个字段都全给 (不省最后一条的长度): 388KB 载荷里那 4 字节
        = 0.001%, 换来 Σcomp_len / Σn_slices 两个求和自校验, 载荷截断会当场
        NAK 而不是解出半帧垃圾还照样 ACK。
    split=nA 字节: 老的 DUAL_FACE **两条独立压缩流**
        payload = [u32 LE comp_len_A][面A 流][面B 流]
    split=None 且 streams=None: 单流 (单面帧 = 老行为, 逐字节不变)。

    拆流的意义在板端: 流互不依赖 ⇒ 可以并行解到 A9 双核, 单帧解码时间按最
    慢那条流算, 且不必引入奇偶帧交错那套额外缓冲和一帧延迟。
    代价只是丢了跨流的后向引用 (实测 zlib 33.7x → 33.6x; lz4-HC9 整帧单流
    388166B → 按面两流 388307B, +0.04%)。
    codec='lz4' 时每条流各是一个 LZ4 raw block, 排布一模一样 (protocol.h)。"""
    if streams:
        parts, off, fl = [], 0, 0
        for ns in streams:
            nb = ns * SLICE_STRIDE
            c, fl = _encode(raw[off:off + nb], codec, zlevel, lz4_level)
            parts.append((c, ns))
            off += nb
        if off != len(raw):
            raise ValueError(f'流表片数合计 {off}B != 帧长 {len(raw)}B')
        tbl = MSTR_N.pack(len(parts)) + b''.join(MSTR_E.pack(len(c), ns)
                                                 for c, ns in parts)
        return tbl + b''.join(c for c, _ in parts), fl | FLAG_MSTREAM
    if split is None:
        return _encode(raw, codec, zlevel, lz4_level)
    a, fl = _encode(raw[:split], codec, zlevel, lz4_level)
    b, _ = _encode(raw[split:], codec, zlevel, lz4_level)
    return DUAL_HDR.pack(len(a)) + a + b, fl


def decompress_frame(payload, flags):
    """线上 payload → raw。多流时按流表/u32 前缀拆开分别解再拼回 —— 解出来
    与拆流前的同一帧逐字节相同 (板端各个核各解各的, 直接写进 staging 缓冲的
    对应偏移, 连拼接都省了)。

    🔴 这是**接收侧**: 片距一律从 flags 推 (= protocol.h 的 PVS_STRIDE(flags)),
    不看模块全局 BPP —— 收帧的进程 (fake_board / 测试) 根本没调过 _apply_bpp,
    拿全局会把 3-bit 帧的 lz4 输出缓冲算小 3 倍。"""
    stride = pack_obs.slice_stride(3 if flags & FLAG_3BIT else 1)
    if flags & FLAG_MSTREAM:
        (n,) = MSTR_N.unpack_from(payload)
        if not 1 <= n <= MAX_STREAMS:
            raise ValueError(f'MSTREAM n_streams={n} 越界 (1..{MAX_STREAMS})')
        tbl = [MSTR_E.unpack_from(payload, MSTR_N.size + i * MSTR_E.size)
               for i in range(n)]
        off = MSTR_N.size + n * MSTR_E.size
        if off + sum(c for c, _ in tbl) != len(payload):
            raise ValueError(f'MSTREAM Σcomp_len 与 payload {len(payload)}B 不符')
        out = []
        for clen, nsl in tbl:
            out.append(_decode(payload[off:off + clen], flags,
                               nsl * stride))
            off += clen
        return b''.join(out)
    if flags & FLAG_DUAL_FACE:
        (n_a,) = DUAL_HDR.unpack_from(payload)
        end_a = DUAL_HDR.size + n_a
        if n_a > len(payload) - DUAL_HDR.size:
            raise ValueError(f'DUAL_FACE comp_len_A={n_a} 越界 (payload {len(payload)}B)')
        return _decode(payload[DUAL_HDR.size:end_a], flags) + _decode(payload[end_a:], flags)
    return _decode(payload, flags)


def xor_frames(a, b):
    """raw XOR raw (delta 编码/解码共用, numpy 向量化)."""
    return (np.frombuffer(a, np.uint8) ^ np.frombuffer(b, np.uint8)).tobytes()


def slices_of(nbytes, where=''):
    """帧字节数 → n_slices, 顺带校验协议约束 (片距整数倍 且 ≤ 帧长上限)。
    片距随 --bpp 变 (1-bit 0x3000 / 3-bit 0x9000), 上限恒是 8847360B。

    v3.1 起帧长不再是常量 (单面 360 / 双面 720 / 双面折叠 540 …), 凡是过去
    硬比 FRAME_RAW 的地方都改走这里。"""
    st = SLICE_STRIDE
    if nbytes <= 0 or nbytes % st or nbytes > FRAME_RAW_MAX:
        raise ValueError(f'{where}{nbytes}B 不是合法帧长 '
                         f'(bpp={BPP} 片距 0x{st:X}: 须为它的整数倍且 ≤ '
                         f'{FRAME_RAW_MAX})')
    return nbytes // st


def face_split_bytes(geom_flags):
    """DUAL_FACE 帧里面A 数据的字节数 (= raw 的拆流点); 单面帧返回 None。
    nA = FOLD_A ? 180 : 360 片 (protocol.h 的合法组合表)。"""
    if not (geom_flags & FLAG_DUAL_FACE):
        return None
    n_a = N_SLICES_FOLD if geom_flags & FLAG_FOLD_A else N_SLICES
    return n_a * SLICE_STRIDE


# ================= 预压缩缓存 (26fps: 推流热路径零压缩开销) =================
# FrameEntry: 每帧线上 payload 预先算好. delta 模式 key/delta 双份都存 —
# 关键帧周期与重连恢复由发送时刻决定, 任意帧都可能被要求当 keyframe.
FrameEntry = collections.namedtuple(
    'FrameEntry', 'key delta key_flags delta_flags raw_len')


def _precomp_job(job):
    """进程池 worker: 读帧 (+前帧), 出 (key payload, delta payload|None, raw_len).
    split 非 None (DUAL_FACE) 时出的是双流 payload, 与现场压缩路径同一套编码。

    🔴 bpp 必须随 job 传进来再落一次全局: py3.14 起 Linux 的默认 start method
    是 forkserver, 子进程是**重新 import** 本模块, 父进程 _apply_bpp/_apply_slot_count
    改的全局一个都不继承 ⇒ 子进程里 SLICE_STRIDE 会退回 0x3000, 3-bit 的流表
    片数×片距当场对不上帧长 (fork 时代碰巧不炸, 别指望)。"""
    path, prev_path, zlevel, split, codec, lz4_level, streams, bpp = job
    if bpp != BPP:
        _apply_bpp(bpp)
    raw = open(path, 'rb').read()
    slices_of(len(raw), f'{path}: ')
    key, _ = compress_frame(raw, codec, zlevel, split, lz4_level, streams)
    delta = None
    if prev_path is not None:
        prev = open(prev_path, 'rb').read()
        if len(prev) != len(raw):
            raise ValueError(f'{prev_path}: {len(prev)}B != {path} {len(raw)}B '
                             f'(同目录帧长必须一致)')
        # 逐字节 XOR 后再按同一个 split 拆流 ≡ 各面各自 XOR 自己的同面参考数据
        # (XOR 是逐字节的, 两面边界又对齐) —— 板端两个核各自 XOR 各自的参考帧。
        delta, _ = compress_frame(xor_frames(prev, raw), codec, zlevel, split,
                                  lz4_level, streams)
    return key, delta, len(raw)


def frame_files_from_dir(d):
    files = sorted(glob.glob(os.path.join(d, 'frame_*.bin'))
                   or glob.glob(os.path.join(d, '*.bin')))
    if not files:
        sys.exit(f'no .bin frames in {d}')
    return files


def build_precomp(d, zlevel=6, delta=False, jobs=0, geom_flags=0,
                  codec='zlib', lz4_level=DEFAULT_LZ4_LEVEL,
                  stream_split='face'):
    """--dir 整目录预压缩到内存: [FrameEntry]. delta=True 时每帧还预算
    对流序前一帧的 XOR+压缩 delta 链; 帧 0 的 delta 参考最后一帧 (loop 回
    绕), 非 loop/首连时帧 0 由发送策略强制走 key payload, 回绕 delta 只在
    loop 第 2 圈起被用到, 语义正确."""
    files = frame_files_from_dir(d)
    n = len(files)
    split = face_split_bytes(geom_flags)       # DUAL_FACE → 预压缩也出双流
    n_sl0 = slices_of(os.path.getsize(files[0]), f'{files[0]}: ')
    streams = stream_plan(n_sl0, geom_flags, stream_split)
    if streams:
        print(f'[precomp] 多流切分 {streams} 片 (MSTREAM), 板端 {DEC_WORKERS} 核'
              f'连续分组后每核 ≈{sum(streams) // DEC_WORKERS} 片', flush=True)
    jl = [(f, files[(i - 1) % n] if delta else None, zlevel, split,
           codec, lz4_level, streams, BPP)
          for i, f in enumerate(files)]
    t0 = time.time()
    if jobs == 1 or n == 1:
        results = [_precomp_job(j) for j in jl]
    else:
        import multiprocessing as mp
        with mp.Pool(processes=jobs or None) as pool:
            results = pool.map(_precomp_job, jl, chunksize=1)
    cflag = FLAG_LZ4 if codec == 'lz4' else FLAG_ZLIB
    if streams:
        cflag |= FLAG_MSTREAM
    entries = [FrameEntry(key=k, delta=dl, key_flags=cflag,
                          delta_flags=cflag | FLAG_DELTA,
                          raw_len=rl)
               for k, dl, rl in results]
    n_sl = {e.raw_len // SLICE_STRIDE for e in entries}
    if n_sl != {N_SLICES}:
        print(f'[precomp] 帧片数 {sorted(n_sl)} (非传统 360) — '
              f'头里的 n_slices 按实际帧长写', flush=True)
    key_mb = sum(len(e.key) for e in entries) / 1e6
    delta_mb = sum(len(e.delta or b'') for e in entries) / 1e6
    total_mb = key_mb + delta_mb
    print(f'[precomp] {n} 帧预压缩完成 {time.time() - t0:.1f}s: '
          f'key {key_mb:.1f}MB'
          + (f' + delta {delta_mb:.1f}MB' if delta else '')
          + f' = {total_mb:.1f}MB 常驻内存', flush=True)
    if total_mb > CACHE_WARN_MB:
        print(f'[precomp] ⚠ 缓存 {total_mb:.0f}MB 超过告警线 {CACHE_WARN_MB}MB, '
              f'注意内存压力 (帧多/压缩比差时考虑分段或去 --delta 双份)', flush=True)
    return entries


# ================= 帧渲染 (点云 → 4.4MB packed frame) =================

HALF_ASPECT = False      # --half-aspect: 半径方向也压一半, 保持宽高比
HALF_SCREEN = False      # --half-screen: 内容压到 Y 90..179 (配 RTL 的 half_scan)


def _to_half_screen(q, aspect=False):
    """把整屏码值图 (180,160,3) 压成下半屏: 相邻两行取平均 -> 90 行, 放进 Y 90..179。

    为什么是"下半": pack_obs 的 Y 映射 `_Y_H = 11 - Y//15`, 即 Y=179 落在芯片 0
    (移位链数据入口端)。RTL 的 half_scan 每行只发 96 bit, 更新的正是靠入口那 6 颗
    = 芯片 0..5 = Y 90..179。远端 6 颗拿到的是**上一个扫描行**被推过去的数据,
    所以上半屏会出现一份几乎相同的拷贝 (只差一个扫描行) —— 这是 192bit 移位链的
    固有行为, 芯片没有短链配置可以消除它 (datasheet REG1/REG2 只有电流增益/白平衡/
    消影/开路检测)。**用法是只看其中一半, 另一半的拷贝不管。**

    换来的是整屏 31590 -> 16038 拍, 槽数翻倍 (3.6° -> 1.8°)。
    """
    # 🔴 取 max 不取平均: 立方体/人物这类**表面**内容在垂直方向是稀疏的
    # (一行亮一行黑), 取平均等于把亮线和黑底混在一起 —— 实测点亮处平均码值
    # 从 2.43 腰斩到 1.40、码值7 占比 17.7%->6.3%, 屏上直接暗一倍。
    # max 丢的是"两行都有内容时的细节", 对表面渲染远比腰斩亮度划算。
    half = np.maximum(q[0::2], q[1::2])          # 180 -> 90 行
    out = np.zeros_like(q)
    if aspect:
        # 等比例: 半径方向也压一半并居中。半屏只压高度不压半径 ⇒ 立体像必然扁
        # (倾斜 30° 的圆柱视在倾角会变成 16°)。把 X 也压一半, 物体只占屏中间
        # 一半宽度, 但**宽高比恢复正确** —— 代价是立体像整体小一号。
        hw = np.maximum(half[:, 0::2], half[:, 1::2])    # 160 -> 80 列
        x0 = (q.shape[1] - hw.shape[1]) // 2
        out[q.shape[0] // 2:, x0:x0 + hw.shape[1]] = hw
    else:
        out[q.shape[0] // 2:] = half
    return out


def render_packed_frame(vox, frame_idx, render_slices, sub, thresh, dither,
                        freeze_phase=False, axis_off=0.0, mirror_u=True, gain=None,
                        n_out=N_SLICES, led_gamma=gas.LED_GAMMA):
    """体素格 → 单面 n_out×SLICE_STRIDE 数据块 (默认 n_out=360 = 完整一面).

    色深走模块全局 BPP (见 _apply_bpp):
      BPP=1 → gas.to_1bit + pack_slice(bpp=1, pad=True)  (逐字节等于老代码:
              老代码是 pack_slice(on) 再手工接 624B PAD, pad=True 就是它)
      BPP=3 → gas.to_3bit (gamma 解码 + 8 级量化 + 残差 Bayer 抖动) +
              pack_slice(bpp=3), 一片 3 个位平面 = 0x9000。
              抖动相位 phase 与 1-bit 完全同一套 (逐槽 + 逐帧), 时域平滑不退化。
    render_slices < 360 时每个渲染角复制填
    360/render_slices 个槽 (布局不变, 省渲染时间); Bayer 相位仍逐槽+逐帧变.
    freeze_phase=True: 相位只随 slot 不随 frame_idx (时域抖动冻结) —
    静止区域帧间字节不变, delta 编码红利兑现; 代价是丢时域抖动的灰度平滑
    (空间抖动纹理静止化), 见 04 设计稿 §1②/S5。

    axis_off / mirror_u: 屏面几何 — 到转轴的垂距 (体素px) + 全局中心轴镜像。
    必须与板上实际机械结构一致, 否则渲出来的帧跟机器对不上
    (07-09 那批 frames_* 就是旧几何)。用 gas.resolve_axis_off() 解析:
      · v3   对称装 13.8mm → 每面 7.36px, 两面靠 PHASE_B=180 共用这一份
      · v3.1 偏心装        → A 面 0.0px / B 面 14.29px, **两面各一份, 不能共用**
    ⚠ 本函数只渲**一个面**; 偏心装下驱动两面 = 调两次再拼接 (见 gen_packed_frames
      的 --dual-face) + RTL slice_base_B。

    n_out: 本面输出的槽数 (默认 360 = 整圈)。180 = --fold-a 折叠面A, 只出
    θ=0..179° —— 槽↔角度映射与整圈渲染完全一致, 前 180 槽逐字节相同,
    180..359 由 PL 取 idx-180 再做镜像置换补出 (仅穿心面成立)。"""
    assert N_SLICES % render_slices == 0, \
        f'--render-slices {render_slices} 必须整除槽数 {N_SLICES}'
    dup = N_SLICES // render_slices
    assert 0 < n_out <= N_SLICES and n_out % dup == 0, \
        f'n_out={n_out} 须 ≤360 且是复制因子 dup={dup} 的整数倍'
    d_step = 2 * math.pi / render_slices
    parts = []
    for k in range(render_slices):
        if k * dup >= n_out:                 # 折叠: 后半圈根本不渲 (省一半时间)
            break
        img = gas.render_slice(vox, k * d_step, sub, d_step, axis_off, mirror_u, gain)
        for j in range(dup):
            slot = k * dup + j
            # 7 与 16 互素, 逐帧遍历相位; freeze 时只随 slot
            phase = slot if freeze_phase else slot + frame_idx * 7
            if BPP == 1:
                q = gas.to_1bit(img, thresh, dither, phase)
            else:
                q = gas.to_3bit(img, thresh, dither, phase, gamma=led_gamma)
            if HALF_SCREEN:
                q = _to_half_screen(q, HALF_ASPECT)
            parts.append(pack_obs.pack_slice(q, bpp=BPP, pad=True))
    return b''.join(parts)


# ================= 动画源: spinpulse =================

def _cache_dir():
    return os.environ.get('POVSTREAM_CACHE', os.path.join(HERE, 'cache'))


def load_anime_points(args):
    """GLB 采样点云, 结果缓存 npz (采样是最贵的一步, 只做一次)."""
    samples = args.samples or 1800000
    key = f'{os.path.splitext(os.path.basename(args.glb))[0]}_{samples}'
    cache = os.path.join(_cache_dir(), f'pts_{key}.npz')
    if os.path.exists(cache):
        z = np.load(cache)
        xyz, col = z['xyz'], z['col']
        print(f'[cache] {cache}: {len(xyz)} pts', flush=True)
    else:
        xyz, col = gas.points_from_glb(args.glb, samples, args.lighting, args.ambient)
        os.makedirs(os.path.dirname(cache), exist_ok=True)
        np.savez_compressed(cache, xyz=xyz, col=col)
        print(f'[cache] saved {cache}', flush=True)
    col = gas.color_adjust(col, args.brighten, args.gamma, args.saturation)
    p = gas.normalize_points(xyz, args.z_stretch)
    return p.astype(np.float32), col.astype(np.float32)


def spinpulse_frames(args):
    """呼吸 ±5% + 竖直浮动 + 披风 sinusoidal x-shear. 周期 = --frames, 无缝循环."""
    p0, col = load_anime_points(args)
    p0 = p0 * 0.92                       # 留呼吸/浮动余量, 防边界 clip 糊
    n = args.frames
    for t in range(n):
        u = 2 * math.pi * t / n
        p = p0 * (1.0 + args.breath * math.sin(u))          # 呼吸
        p[:, 1] += args.bob * math.sin(u + math.pi / 2)     # 上下浮动
        # 披风摆动: 按离质心竖直距离的 x 方向剪切 (上下反相)
        p[:, 0] += args.sway * math.sin(2 * u + 0.7) * (p0[:, 1] / gas.H_BUDGET)
        yield gas.voxel_grid(p, col, verbose=False, ssaa=args.ssaa)


# ================= 动画源: globe =================

# 手绘 80x40 世界陆海掩膜 (等距圆柱, 行=北→南 90..-90, 列=经度 -180..+180; #=陆)
# 办公网拦外链, 内嵌数据; 精度 4.5°/格, 显示 160px 下可辨认各大洲
WORLD_MAP = [
    "................................................................................",
    "............................#####..............................................",
    "............##....##........#######........#...................................",
    "..........########..####....########.....................######..######........",
    "...###...#################..#######........####....##############.#####........",
    "..#####..################....#####...##...######..#############.########.......",
    "..######..###############.....###.....#..#####...##############..#######.......",
    "...####...##############.............##..####...###############.########.......",
    "....##.....##############...........##..#########################.#####........",
    "............##############..........############################..###..........",
    "............##############.........#############################...##..........",
    "............#############........############..#################...##..........",
    ".............###########.........####..####..#####..############...#...........",
    ".............####..#####.........#################...####..####................",
    "..............######..............################...####...###.##.............",
    "...............#####.............################....###....###................",
    ".................####.............###############....##.....###.#..............",
    "..................####............##############.....#......##..##.............",
    "....................####...........############......#.......#..#..............",
    ".....................######........###########.......#..##..##...####..........",
    ".....................#########......##########..........####.#..######.........",
    ".....................##########.....##########...........##......#####.........",
    "......................##########....#########..#.....##########................",
    "......................##########....########...#....############...............",
    ".......................#########....#######....#...##############..............",
    ".......................########.....######.........#############...............",
    ".......................#######......#####...........############...............",
    ".......................######........###.............#####..###................",
    ".......................#####..................................#..##............",
    ".......................####..................................#..##.............",
    ".......................###......................................#..............",
    ".......................##.......................................................",
    ".......................##.......................................................",
    "................................................................................",
    "................................................................................",
    "................................................................................",
    "........########################################################################",
    "################################################################################",
    "################################################################################",
    "################################################################################",
]
_WM = np.array([[c == '#' for c in row.ljust(80, '.')[:80]] for row in WORLD_MAP], np.bool_)


def globe_frames(args):
    """空心壳地球 (用户定): 壳面贴 NASA earth_clean.jpg 真实大陆,
    海纯蓝/陆纯绿/冰白 (zynq_pov 实测分类), 三层壳保密度, 逐帧转纹理."""
    R = gas.R_BUDGET * 0.48                                 # 直径半幅 (2026-07-09 用户定)
    tex_path = os.path.join(HERE, 'earth_clean.jpg')
    tex = np.asarray(Image.open(tex_path).convert('RGB'), np.int32)
    TH_, TW_ = tex.shape[:2]
    n_lat, n_lon = 240, 720
    lat = np.linspace(-math.pi / 2 * 0.98, math.pi / 2 * 0.98, n_lat, dtype=np.float32)
    lon = np.linspace(0, 2 * math.pi, n_lon, endpoint=False, dtype=np.float32)
    LA, LO = np.meshgrid(lat, lon, indexing='ij')
    la1, lo1 = LA.ravel(), LO.ravel()
    la = np.concatenate([la1] * 3)
    lo = np.concatenate([lo1] * 3)
    rr = np.concatenate([np.full_like(la1, R), np.full_like(la1, R - 1.3),
                         np.full_like(la1, R - 2.6)])       # 三层壳
    p = np.stack([rr * np.cos(la) * np.cos(lo),
                  rr * np.sin(la),
                  rr * np.cos(la) * np.sin(lo)], axis=1).astype(np.float32)
    n = args.frames
    for t in range(n):
        le = (lo + 2 * math.pi * t / n) % (2 * math.pi)     # 自转: 转纹理不转点
        row = np.clip(((math.pi / 2 - la) / math.pi * TH_).astype(np.int32), 0, TH_ - 1)
        ci = np.clip((le / (2 * math.pi) * TW_).astype(np.int32), 0, TW_ - 1)
        r_, g_, b_ = tex[row, ci, 0], tex[row, ci, 1], tex[row, ci, 2]
        ocean = (b_ > g_ + 10) & (b_ > r_ + 10)
        ice = (r_ > 170) & (g_ > 170) & (b_ > 170)
        col = np.zeros((len(p), 3), np.float32)
        col[:] = (0, 255, 0)                                # 默认陆 = 纯绿
        col[ocean] = (0, 0, 255)                            # 海 = 纯蓝
        col[ice] = (255, 255, 255)                          # 冰盖 = 白
        yield gas.voxel_grid(p, col, verbose=False, ssaa=args.ssaa)


# ================= 动画源: glbseq / glbanim (GLB 动画装载器) =================
# TODO: pov_studio.py PRESETS 尚未接入 glbseq/glbanim (本轮不做 GUI 集成)

def normalize_common(xyz, cmin, cmax, z_stretch):
    """gas.normalize_points 的固定-bbox 版: 整段动画所有帧共用同一
    center/scale (逐帧各自归一会让动画整体缩放/平移抖动)."""
    p = xyz - (cmin + cmax) / 2.0
    p[:, 2] *= z_stretch
    h = np.maximum((cmax - cmin) / 2.0, 1e-6)
    s = min(gas.R_BUDGET / h[0], gas.H_BUDGET / h[1],
            gas.R_BUDGET / max(h[2] * z_stretch, 1e-6))
    return (p * s).astype(np.float32)


def _load_glb_points_cached(path, samples, lighting, ambient):
    """单 GLB 文件采样点云 + npz 缓存 (同 load_anime_points, key 加路径
    hash 防序列目录里同名文件互撞)."""
    tag = hashlib.md5(os.path.abspath(path).encode()).hexdigest()[:8]
    key = f'{os.path.splitext(os.path.basename(path))[0]}_{tag}_{samples}'
    cache = os.path.join(_cache_dir(), f'pts_{key}.npz')
    if os.path.exists(cache):
        z = np.load(cache)
        xyz, col = z['xyz'], z['col']
        print(f'[cache] {cache}: {len(xyz)} pts', flush=True)
    else:
        xyz, col = gas.points_from_glb(path, samples, lighting, ambient)
        os.makedirs(os.path.dirname(cache), exist_ok=True)
        np.savez_compressed(cache, xyz=xyz, col=col)
        print(f'[cache] saved {cache}', flush=True)
    return np.asarray(xyz, np.float32), np.asarray(col, np.float32)


def glbseq_frames(args):
    """GLB 帧序列: --glb-dir 内 sorted *.glb, 每文件 = 一帧 (外部 DCC 工具
    逐帧导出的动画). 帧数 = 文件数 (--frames 被忽略). 先全量装载算全局
    bbox, 所有帧共用同一归一变换."""
    if not args.glb_dir:
        sys.exit('--anim glbseq 需要 --glb-dir <目录>')
    files = sorted(glob.glob(os.path.join(args.glb_dir, '*.glb')))
    if not files:
        sys.exit(f'no .glb files in {args.glb_dir}')
    samples = args.samples or 400000
    seq = [_load_glb_points_cached(f, samples, args.lighting, args.ambient)
           for f in files]
    cmin = np.min([x.min(axis=0) for x, _ in seq], axis=0)
    cmax = np.max([x.max(axis=0) for x, _ in seq], axis=0)
    args.frames = len(files)
    print(f'[glbseq] {len(files)} frames x {samples} pts, '
          f'bbox {cmin.round(2)}..{cmax.round(2)}', flush=True)
    for xyz, col in seq:
        col = gas.color_adjust(col, args.brighten, args.gamma, args.saturation)
        yield gas.voxel_grid(normalize_common(xyz, cmin, cmax, args.z_stretch),
                             col, verbose=False, ssaa=args.ssaa)


def pure_rgb_snap(col, dom=0.55):
    """1-bit 海报化: 每点主导通道组合置 1.0 其余置 0 (通道≥dom×max 算主导),
    抖动存活率 100%, 密度=几何覆盖上限 (地球仪纯通道色经验的通用化).
    代价: 色彩变 7 色海报风 (R/G/B/黄/品红/青/白)."""
    m = col.max(axis=1, keepdims=True)
    return (col >= np.maximum(dom * m, 1e-6)).astype(np.float32) * 255.0


def glbanim_frames(args):
    """单 GLB 真 glTF 动画 (骨骼 skinning / morph targets / 节点 TRS,
    见 glb_anim.py). --frames N 均匀采 timeline: t = k/N × duration
    (k=0..N-1, 首尾相接可 loop); --anim-take 选 take (名或索引).
    所有帧共用全时段 bbox 归一."""
    import glb_anim
    samples = args.samples or 400000
    smp = glb_anim.AnimSampler(args.glb, take=args.anim_take, samples=samples,
                               lighting=args.lighting, ambient=args.ambient)
    n = args.frames
    if smp.duration <= 0:
        print('[glbanim] WARNING: 无动画 timeline, 全帧静态姿态', flush=True)
    times = [smp.duration * k / n for k in range(n)]
    if args.robust_fit:
        # 鲁棒 bbox: 每帧 [2,98] 分位取并集, 排除飞行特效骨骼/技能位移
        # 拖爆包围盒把人物压小 (LoL R 技能锤/箭/针线能飞 10+ 身位)
        los, his = [], []
        for t in times:
            xyz, _ = smp.points_at(t)
            los.append(np.percentile(xyz, 2, axis=0))
            his.append(np.percentile(xyz, 98, axis=0))
        cmin = np.min(los, axis=0).astype(np.float32)
        cmax = np.max(his, axis=0).astype(np.float32)
        print(f'[glbanim] robust bbox {cmin.round(2)}..{cmax.round(2)}', flush=True)
    else:
        cmin, cmax = smp.bbox_over(times)
    # ---- 逐帧自适应缩放 (--fit-frames, 2026-07-28) ----
    # 并集 bbox 让单帧又小又偏: 同一 Robot 静态渲高 140 / 动画渲仅 86 (缩 39%)。
    # 逐帧各自归一能充满体积, 但会带来帧间缩放/平移抖动 —— 故对逐帧的
    # scale 与 center 序列做**循环滑动平均**平滑 (动画首尾相接, 用 wrap 卷积),
    # 既吃满体积又不抖。--fit-smooth 0 = 纯逐帧 (最满但可能抖)。
    per = None
    if args.fit_frames:
        sc_l, cen_l = [], []
        for t in times:
            xyz, _ = smp.points_at(t)
            lo = np.percentile(xyz, args.fit_pct, axis=0)
            hi = np.percentile(xyz, 100.0 - args.fit_pct, axis=0)
            cen = (lo + hi) / 2.0
            h = np.maximum((hi - lo) / 2.0, 1e-6)
            sc_l.append(min(gas.R_BUDGET / h[0], gas.H_BUDGET / h[1],
                            gas.R_BUDGET / max(h[2] * args.z_stretch, 1e-6)))
            cen_l.append(cen)
        sc_a = np.asarray(sc_l, np.float32)
        cen_a = np.asarray(cen_l, np.float32)
        w = int(args.fit_smooth)
        if w > 1:
            k = np.ones(w, np.float32) / w
            pad = w // 2 + 1
            def _wrap(v):                     # 循环卷积 (动画首尾相接)
                e = np.concatenate([v[-pad:], v, v[:pad]])
                return np.convolve(e, k, 'same')[pad:pad + len(v)]
            sc_a = _wrap(sc_a)
            cen_a = np.stack([_wrap(cen_a[:, i]) for i in range(3)], axis=1)
        per = (sc_a, cen_a)
        print('[glbanim] fit-frames: scale %.1f..%.1f (均 %.1f), 平滑窗 %d, 分位 %.1f%%'
              % (sc_a.min(), sc_a.max(), sc_a.mean(), w, args.fit_pct), flush=True)

    for fi, t in enumerate(times):
        xyz, col = smp.points_at(t)
        col = gas.color_adjust(col, args.brighten, args.gamma, args.saturation)
        if args.pure_rgb:
            col = pure_rgb_snap(col, args.pure_dom)
        if per is not None:
            q = xyz.astype(np.float32) - per[1][fi]
            q[:, 2] *= args.z_stretch
            p = q * per[0][fi] * args.scale
        else:
            p = normalize_common(xyz, cmin, cmax, args.z_stretch) * args.scale
        p[:, 0] += args.x_offset
        p[:, 1] += args.y_offset      # 竖直平移 (voxel); 正值向上
        yield gas.voxel_grid(p, col, verbose=False, ssaa=args.ssaa)


def spin_frames(args):
    """任意静态 GLB 绕竖轴 (y) 自转: 采样一次, 逐帧旋转点云 (--frames =
    一圈, 首尾相接可 loop). --scale 归一化后整体缩放 (<1 缩小留白);
    --shells N 洋葱状向内复制 N 层 (间距 --shell-gap voxel, 表面网格壳
    太薄时加厚, 同 globe 源三层壳套路)."""
    xyz, col = _load_glb_points_cached(args.glb, args.samples or 400000,
                                       args.lighting, args.ambient)
    col = gas.color_adjust(col, args.brighten, args.gamma, args.saturation)
    p0 = gas.normalize_points(xyz, args.z_stretch) * args.scale
    if args.shells > 1:
        r_eff = float(np.linalg.norm(p0, axis=1).max())
        p0 = np.concatenate([p0 * (1.0 - k * args.shell_gap / r_eff)
                             for k in range(args.shells)])
        col = np.concatenate([col] * args.shells)
    n = args.frames
    for t in range(n):
        a = 2 * math.pi * t / n
        c, s = math.cos(a), math.sin(a)
        p = np.empty_like(p0)
        p[:, 0] = p0[:, 0] * c - p0[:, 2] * s
        p[:, 1] = p0[:, 1]
        p[:, 2] = p0[:, 0] * s + p0[:, 2] * c
        yield gas.voxel_grid(p, col, verbose=False, ssaa=args.ssaa)


# ================= 动画源: palace / notredame (空中巡游 orbit) =================

ORBIT_ZOOM_MAX = 1.15
PALACE_ZOOM_MAX = ORBIT_ZOOM_MAX          # 兼容旧名 (test_palace.py 引用)


def orbit_prescale(p0, col):
    """建筑点云预缩放到 zoom 最大 (1.15) 时任意 yaw 角都不出预算:
    旋转约束是径向 sqrt(x²+z²) ≤ R_BUDGET (footprint 对角在 45° 时甩到
    最远, 出圆柱会被 clip 糊边), 竖直 |y| ≤ H_BUDGET."""
    r_max = float(np.hypot(p0[:, 0], p0[:, 2]).max())
    y_max = float(np.abs(p0[:, 1]).max())
    s = min((gas.R_BUDGET - 0.5) / (r_max * ORBIT_ZOOM_MAX),
            (gas.H_BUDGET - 0.5) / (y_max * ORBIT_ZOOM_MAX))
    return (p0 * s).astype(np.float32), col.astype(np.float32)


def palace_points_prescaled():
    """紫禁城点云 (palace.py), 已 orbit 预缩放."""
    import palace
    return orbit_prescale(*palace.build_palace())


def notredame_points_prescaled():
    """巴黎圣母院点云 (notredame.py), 已 orbit 预缩放."""
    import notredame
    return orbit_prescale(*notredame.build_notredame())


def orbit_frames(points, colors, args):
    """空中巡游 (palace/notredame 共用): 每循环绕 y 轴 yaw 一整圈, 同时
    zoom 按正弦 0.75→1.15→0.75 一循环 (拉远→拉近→拉远, 读作 fly-out/
    fly-in). points 须已 orbit_prescale. 先旋转再缩放再体素化,
    --frames = 循环周期, 首尾相接可 loop."""
    n = args.frames
    for t in range(n):
        u = 2 * math.pi * t / n
        zoom = 0.95 - 0.20 * math.cos(u)         # 0.75 → 1.15 → 0.75
        c, s = math.cos(u), math.sin(u)          # yaw 一整圈
        p = np.empty_like(points)
        p[:, 0] = (points[:, 0] * c - points[:, 2] * s) * zoom
        p[:, 1] = points[:, 1] * zoom
        p[:, 2] = (points[:, 0] * s + points[:, 2] * c) * zoom
        yield gas.voxel_grid(p, colors, verbose=False, ssaa=args.ssaa)


def palace_frames(args):
    """紫禁城空中巡游 (几何 palace.py, 巡游 orbit_frames)."""
    p0, col = palace_points_prescaled()
    yield from orbit_frames(p0, col, args)


def notredame_frames(args):
    """巴黎圣母院空中巡游 (几何 notredame.py, 巡游 orbit_frames)."""
    p0, col = notredame_points_prescaled()
    yield from orbit_frames(p0, col, args)



def rgbcube_frames(args):
    """RGB 色立方体: 表面每点的颜色 = 它自己的归一化 (x,y,z)。

    为什么用它验色深: 8 个顶点正好是 黑/红/绿/蓝/黄/品红/青/白, 整个表面是
    三个通道各自沿一条棱的**平滑渐变**。色阶断层在平滑渐变上最刺眼 ——
    有几级就会看到几条等色带(Mach band), 数带子就能数出色深, 比色块图直观。
    立方体还有个好处: 三个可见面朝向不同, 一眼能同时看到三组不同的通道组合。

    绕竖轴(y)匀速自转, --frames 一个整周期, 无缝循环。
    """
    a = gas.R_BUDGET * 0.52                      # 半边长; 对角线 a*sqrt3 仍在预算内
    m = max(8, int(args.cube_grid))              # 每面 m x m 采样
    g = np.linspace(-1.0, 1.0, m, dtype=np.float32)
    U, V = np.meshgrid(g, g, indexing='ij')
    u, v = U.ravel(), V.ravel()
    one = np.ones_like(u)
    faces = [                                    # 6 个面: (x, y, z)
        (one, u, v), (-one, u, v),
        (u, one, v), (u, -one, v),
        (u, v, one), (u, v, -one),
    ]
    p0 = np.concatenate([np.stack(f, axis=1) for f in faces], axis=0) * a
    # 颜色 = 归一化坐标 -> 0..255, 三通道各沿一条棱渐变
    col = (((p0 / a) + 1.0) * 0.5 * 255.0).clip(0, 255).astype(np.uint8)
    n = args.frames
    for t in range(n):
        th = 2 * math.pi * t / n
        c, sn = math.cos(th), math.sin(th)
        p = p0.copy()
        p[:, 0] = p0[:, 0] * c + p0[:, 2] * sn   # 绕 y 轴转
        p[:, 2] = -p0[:, 0] * sn + p0[:, 2] * c
        yield gas.voxel_grid(p, col, verbose=False, ssaa=args.ssaa)



def rgbcyl_frames(args):
    """倾斜的渐变色圆柱, 绕竖轴(Y)自转 —— 比立方体更适合看 POV 的立体感。

    为什么倾斜: 竖直圆柱绕竖轴转是**旋转对称**的, 转起来看不出在转, 也看不出
    角分辨率好坏。倾斜之后每个角度的截面都不同, 转起来像陀螺进动, 角分辨率
    不够会立刻表现为轮廓卡顿/棱边。

    配色: 周向 φ 走 R/G/B 三相彩虹 (相位差 120°), 轴向叠一层亮度渐变
    —— 一眼能同时判断色深(渐变顺不顺)和角分辨率(周向色带边缘干不干净)。
    """
    tilt = math.radians(args.tilt)
    # ⚠ --half-screen 会把 180 行压成 90 行显示, 垂直方向 2:1 压缩而半径方向不变
    # ⇒ 立体像必然被压扁 (倾斜 30° 的圆柱视在倾角会变成 16°, 两端斜边角度不一致,
    #   看起来像"中间拐了个弯")。缩小物体补偿不了, 因为压的只有高度。
    # 唯一能恢复比例的办法: 把半径也缩一半 ⇒ 物体变小、只用屏中间一半宽度,
    # 但宽高比正确。--cyl-radius 就是这个旋钮 (半屏时给 0.23, 全屏时 0.46)。
    r = gas.R_BUDGET * args.cyl_radius
    hh = gas.H_BUDGET * 0.80                      # 半高
    n_phi = max(64, int(args.cube_grid))
    n_t = max(32, int(args.cube_grid) // 2)
    phi = np.linspace(0, 2 * math.pi, n_phi, endpoint=False, dtype=np.float32)
    tt = np.linspace(-1.0, 1.0, n_t, dtype=np.float32)
    P, T = np.meshgrid(phi, tt, indexing='ij')
    # 🔴 径向也要采样 (--cyl-shells 层同心壳), 不能只画表面:
    # POV 显示的是"切片平面 x 物体"的交线, 空心圆柱被过轴平面一切就只剩左右
    # 两条侧壁, 中间是空的 —— 屏上看起来就是"圆柱中间断开"。立方体没这问题
    # 是因为它的面常整面朝向切片平面; 圆柱侧壁是曲面, 任何角度切下去都只有两条线。
    nsh = max(1, int(args.cyl_shells))
    p_side, c_side = [], []
    for rr in np.linspace(1.0, 0.12, nsh):        # 由外向内几层壳
        p_side.append(np.stack([r * rr * np.cos(P), T * hh,
                                r * rr * np.sin(P)], axis=-1).reshape(-1, 3))
        c_side.append(np.stack([P.ravel(), (T.ravel() + 1) * 0.5], axis=-1))
    p_side = np.concatenate(p_side, axis=0)
    c_side = np.concatenate(c_side, axis=0)
    # 两个端面 (同心圆环采样), 颜色沿用侧面的周向色相
    rings = []
    cring = []
    for sgn in (-1.0, 1.0):
        for rr in np.linspace(0.08, 1.0, max(8, n_t // 4)):
            rings.append(np.stack([r * rr * np.cos(phi),
                                   np.full_like(phi, sgn * hh),
                                   r * rr * np.sin(phi)], axis=-1))
            cring.append(np.stack([phi, np.full_like(phi, (sgn + 1) * 0.5)], axis=-1))
    p0 = np.concatenate([p_side] + rings, axis=0)
    cc = np.concatenate([c_side] + cring, axis=0)
    # 颜色: 周向三相彩虹 x 轴向亮度渐变 (0.35..1.0, 别让一端全黑)
    ph, lv = cc[:, 0], 0.35 + 0.65 * cc[:, 1]
    col = np.stack([(np.cos(ph) + 1) * 0.5,
                    (np.cos(ph - 2 * math.pi / 3) + 1) * 0.5,
                    (np.cos(ph + 2 * math.pi / 3) + 1) * 0.5], axis=-1)
    col = (col * lv[:, None] * 255.0).clip(0, 255).astype(np.uint8)
    # 先固定倾斜 (绕 Z), 再逐帧绕 Y 自转 => 陀螺进动
    ct, st_ = math.cos(tilt), math.sin(tilt)
    pt = np.empty_like(p0)
    pt[:, 0] = p0[:, 0] * ct - p0[:, 1] * st_
    pt[:, 1] = p0[:, 0] * st_ + p0[:, 1] * ct
    pt[:, 2] = p0[:, 2]
    n = args.frames
    for k in range(n):
        th = 2 * math.pi * k / n
        c, sn = math.cos(th), math.sin(th)
        p = np.empty_like(pt)
        p[:, 0] = pt[:, 0] * c + pt[:, 2] * sn
        p[:, 1] = pt[:, 1]
        p[:, 2] = -pt[:, 0] * sn + pt[:, 2] * c
        yield gas.voxel_grid(p, col, verbose=False, ssaa=args.ssaa)



def _hsv_ring(h):
    """色相 h∈[0,1) → RGB (S=V=1), 逐元素。红→黄→绿→青→蓝→品红→红。"""
    h6 = (np.asarray(h, np.float32) % 1.0) * 6.0
    i = np.floor(h6).astype(np.int32) % 6
    f = h6 - np.floor(h6)
    zeros, ones = np.zeros_like(f), np.ones_like(f)
    r = np.select([i == 0, i == 1, i == 2, i == 3, i == 4, i == 5],
                  [ones, 1 - f, zeros, zeros, f, ones])
    g = np.select([i == 0, i == 1, i == 2, i == 3, i == 4, i == 5],
                  [f, ones, ones, 1 - f, zeros, zeros])
    b = np.select([i == 0, i == 1, i == 2, i == 3, i == 4, i == 5],
                  [zeros, zeros, f, ones, ones, 1 - f])
    return np.stack([r, g, b], axis=-1)


def rgbhelix_frames(args):
    """螺旋管 (蛇形绕竖轴盘旋), 颜色沿路径渐变, **中心留空**。

    为什么中心要留空: POV 的每个切片都是过旋转轴的平面, 所有角度的切片在轴
    附近**全部重叠** —— 靠近中心的体素会被几百个不同角度反复写, 必然糊成一团,
    而且那里的角分辨率天然过采样(同样角度间隔, 半径越小弧长越短), 画了也白画。
    螺旋管把内容全部放在 helix_r 这个半径上, 正好避开这块区域。

    蛇形还有个好处: 它在每个角度的截面都不同(不像竖直圆柱是旋转对称的),
    转起来能看出角分辨率够不够 —— 不够的话螺旋的边缘会出现棱和台阶。
    """
    turns = max(1.0, float(args.helix_turns))
    R = gas.R_BUDGET * args.helix_r          # 螺旋中心线半径 (避开轴)
    tr = gas.R_BUDGET * args.tube_r          # 管半径
    hh = gas.H_BUDGET * 0.82
    n_t = max(200, int(args.cube_grid) * 2)  # 沿路径采样
    n_c = max(16, int(args.cube_grid) // 8)  # 管截面采样
    shells = max(1, int(args.cyl_shells))
    t = np.linspace(0.0, turns * 2 * math.pi, n_t, dtype=np.float32)
    ct, st_ = np.cos(t), np.sin(t)
    # 中心线 + 切线 (解析求导)
    cen = np.stack([R * ct, np.linspace(-hh, hh, n_t, dtype=np.float32), R * st_], axis=-1)
    dy = 2 * hh / (turns * 2 * math.pi)
    tan = np.stack([-R * st_, np.full_like(t, dy), R * ct], axis=-1)
    tan /= np.linalg.norm(tan, axis=1, keepdims=True)
    # 法平面上的两个正交向量 (用竖直向量做参考构 Frenet 近似)
    up = np.array([0, 1, 0], np.float32)
    n1 = np.cross(tan, up); n1 /= np.linalg.norm(n1, axis=1, keepdims=True)
    n2 = np.cross(tan, n1)
    ang = np.linspace(0, 2 * math.pi, n_c, endpoint=False, dtype=np.float32)
    pts, cols = [], []
    for rr in np.linspace(1.0, 0.15, shells):    # shells=1 => 只有管外壳
        for a in ang:
            off = (tr * rr) * (math.cos(a) * n1 + math.sin(a) * n2)
            pts.append(cen + off)
            # 颜色: 沿路径走**真 HSV 色相环** (S=V=1), 管截面叠明暗做立体感。
            # ⚠ 不用三相余弦: 三个 cos 的和不恒定, 某些相位会发白、饱和度不均;
            #   HSV 每一档都是纯色 (红→黄→绿→青→蓝→品红), 3-bit 只有 8 级/通道,
            #   饱和色才撑得住。
            hue = (t / (2 * math.pi * turns)) % 1.0        # 沿整条路径走满一圈色相
            lv = 0.45 + 0.55 * (0.5 + 0.5 * math.cos(a))
            c = _hsv_ring(hue)
            cols.append((c * lv * 255.0).clip(0, 255).astype(np.uint8))
    p0 = np.concatenate(pts, axis=0).astype(np.float32)
    col = np.concatenate(cols, axis=0)
    n = args.frames
    for k in range(n):
        th = 2 * math.pi * k / n
        c, sn = math.cos(th), math.sin(th)
        p = np.empty_like(p0)
        p[:, 0] = p0[:, 0] * c + p0[:, 2] * sn
        p[:, 1] = p0[:, 1]
        p[:, 2] = -p0[:, 0] * sn + p0[:, 2] * c
        yield gas.voxel_grid(p, col, verbose=False, ssaa=args.ssaa)


ANIMS = {'rgbhelix': rgbhelix_frames, 'rgbcyl': rgbcyl_frames, 'rgbcube': rgbcube_frames, 'spinpulse': spinpulse_frames, 'globe': globe_frames,
         'glbseq': glbseq_frames, 'glbanim': glbanim_frames,
         'spin': spin_frames, 'palace': palace_frames,
         'notredame': notredame_frames}


# ---- 帧布局: 单面 / 双面 / 面A 折叠 (v3.1 偏心屏, 2026-07-31) ----
# Face.n_slices 是该面在载荷里占的槽数; 载荷 = 各面按顺序拼接 (A 在前),
# 帧头 n_slices = 各面之和, raw_len = n_slices × 片距 (bpp 1→0x3000 / 3→0x9000)。
Face = collections.namedtuple('Face', 'name axis_off n_slices how')


def face_plan(args):
    """CLI → ([Face...], geom_flags)。纯函数 (只报错不打印), 可重复调用。

    默认 (既不给 --dual-face 也不给 --fold-a) = 单面整圈, 与 v3.1 之前逐字节
    一致 —— 30+ 套 frames_* 预渲染目录靠这条不变式。"""
    dual = bool(getattr(args, 'dual_face', False))
    fold = bool(getattr(args, 'fold_a', False))
    if dual:
        if args.face_off_mm is not None:
            sys.exit(f'--dual-face 自带两面垂距 ({gas.V31_OFF_A_MM}/{gas.V31_OFF_B_MM}mm), '
                     f'与 --face-off-mm 互斥')
        off_a, how_a = gas.resolve_axis_off(gas.V31_OFF_A_MM, args.gap_mm)
        off_b, how_b = gas.resolve_axis_off(gas.V31_OFF_B_MM, args.gap_mm)
        faces = [Face('A', off_a, N_SLICES, how_a), Face('B', off_b, N_SLICES, how_b)]
        flags = FLAG_DUAL_FACE
    else:
        off, how = gas.resolve_axis_off(args.face_off_mm, args.gap_mm)
        faces = [Face('A', off, N_SLICES, how)]
        flags = 0
    if fold:
        a = faces[0]
        if a.axis_off != 0.0:
            sys.exit(f'--fold-a 只对**穿心面**合法 (垂距必须是 0), 当前面A 垂距 '
                     f'{a.axis_off:.3f}px [{a.how}] —— 偏移面上 slice_i 与 '
                     f'slice_{{i+180}} 不互为镜像, 折叠会让后半圈全错。'
                     f'用 --dual-face 或 --face-off-mm 0')
        if args.render_slices % 2:
            sys.exit(f'--fold-a 需要 --render-slices 为偶数 (当前 {args.render_slices}): '
                     f'180 槽必须是复制因子 360/render_slices 的整数倍')
        faces[0] = a._replace(n_slices=N_SLICES_FOLD)
        flags |= FLAG_FOLD_A
    return faces, flags | bpp_flag()


def bpp_flag():
    """色深位: 板端靠它推片距 (protocol.h PVS_STRIDE(flags)), 与几何位正交。"""
    return FLAG_3BIT if BPP == 3 else 0


def frame_raw_len(faces):
    return sum(f.n_slices for f in faces) * SLICE_STRIDE


def expected_n_slices(geom_flags):
    """给定几何 flags 时板端要求的 n_slices (板端不符直接 NAK):
      DUAL_FACE      → (FOLD_A ? 180 : 360) + 360
      单独 FOLD_A    → 180
      无几何 flag    → 360 (传统单面帧)"""
    if geom_flags & FLAG_DUAL_FACE:
        return (N_SLICES_FOLD if geom_flags & FLAG_FOLD_A else N_SLICES) + N_SLICES
    if geom_flags & FLAG_FOLD_A:
        return N_SLICES_FOLD
    return N_SLICES


def check_layout(n_slices, geom_flags, where=''):
    """发之前先自己对一遍面拆分一致性 —— 对不上板端直接 NAK 关连接,
    与其在链路上炸不如在这里报错 (--dir 目录和 meta.json 被人手动拼过时会踩)。"""
    want = expected_n_slices(geom_flags)
    if n_slices != want:
        sys.exit(f'{where}n_slices={n_slices} 与几何 flags=0x{geom_flags:04x} 不自洽: '
                 f'板端要求 {want} 片 '
                 f'(DUAL_FACE ⇒ (FOLD_A?180:360)+360, 单独 FOLD_A ⇒ 180, 无 flag ⇒ 360)')


def describe_faces(faces, geom_flags):
    n = sum(f.n_slices for f in faces)
    detail = ' + '.join(f'{f.name}:{f.n_slices}' for f in faces)
    return (f'{len(faces)} 面 [{detail}] = {n} 片 × 0x{SLICE_STRIDE:X} = '
            f'{n * SLICE_STRIDE}B, bpp={BPP}, flags=0x{geom_flags:04x}'
            + (' DUAL_FACE' if geom_flags & FLAG_DUAL_FACE else '')
            + (' FOLD_A' if geom_flags & FLAG_FOLD_A else '')
            + (' 3BIT' if geom_flags & FLAG_3BIT else ''))


def verify_fold_a(vox, face, args):
    """置 PVS_FLAG_FOLD_A 前的**打包域前提自检**: 穿心面上 slice_i 必须与
    slice_{i+180} 互为左右镜像 (板端就是按这个恒等式用 idx-180 + 镜像置换补
    后半圈的)。gas.check_meridian_mirror 比的是**图像域、且内部不加抖动**,
    正是这里要的几何门禁。不过 → 直接退出, 绝不带着 FOLD_A 把错数据发出去。"""
    ok = gas.check_meridian_mirror(vox, face.axis_off, args.sub,
                                   2 * math.pi / args.render_slices,
                                   args.mirror_u, args.render_slices)
    if not ok:
        sys.exit('[fold-a] 穿心镜像自检未通过 → 拒绝置 PVS_FLAG_FOLD_A。'
                 '去掉 --fold-a 走完整 360 片, 或先把几何/镜像约定修对')


def gen_packed_frames(args):
    """动画名 → 逐帧 packed bytes (长度 = frame_raw_len(face_plan(args)[0]))。"""
    faces, geom_flags = face_plan(args)
    for f in faces:
        tag = f'面{f.name}: ' if len(faces) > 1 else ''
        note = f' [折叠: 只渲 {f.n_slices} 片 θ=0..179°]' if f.n_slices != N_SLICES else ''
        gas.print_geom(f.axis_off, args.mirror_u, tag + f.how + note)
    if geom_flags:
        print(f'[frame] {describe_faces(faces, geom_flags)}', flush=True)
    if geom_flags & FLAG_FOLD_A:
        # 折叠的代价 (2026-07-31): 抖动相位跟着数据被复制 —— slot i+180 拿的是
        # slot i 的 Bayer 相位, 每转独立相位从 360 种降到 180 种, 时域抖动平滑
        # 效果减半 (灰度过渡略糙)。这是折叠的固有代价, 不是 bug:
        # 「折叠输出 ≠ 完整 360 片输出 的前 180 片 + 后 180 片」在抖动开启时
        # 本来就不该逐字节成立, 别去追这个等式。--no-dither 时才严格相等。
        print('[fold-a] ⚠ 抖动相位随数据复制: 每转独立 Bayer 相位 360→180 种, '
              '时域抖动平滑打对折 (灰度过渡略糙, 几何完全正确)', flush=True)
    checked = not (geom_flags & FLAG_FOLD_A)
    # 逐列亮度补偿增益 (每面各一条; 只在 --radial-comp 时非 None)
    gains = {f.name: None for f in faces}
    if getattr(args, 'radial_comp', False):
        # 双面覆盖阶跃只补**穿心面的内圈** —— 偏移面全部像素都在双覆盖区
        # (它最小半径就是 off), 不该再加倍。
        off_b_px = gas.V31_OFF_B_MM / gas.PITCH_MM
        for f in faces:
            dual_off = off_b_px if (len(faces) > 1 and f.axis_off == 0.0) else None
            gains[f.name] = gas.radial_gain(f.axis_off, dual_off,
                                            args.radial_floor, args.mirror_u)
            g = gains[f.name]
            print(f'[gain] 面{f.name}: 增益 {g.min():.3f}..{g.max():.3f}'
                  f'{" (含双覆盖 2x 补偿)" if dual_off else ""}', flush=True)

    for i, vox in enumerate(ANIMS[args.anim](args)):
        if not checked:                      # 首帧体素上做一次几何门禁
            verify_fold_a(vox, faces[0], args)
            checked = True
        t0 = time.time()
        raw = b''.join(
            render_packed_frame(vox, i, args.render_slices, args.sub,
                                args.thresh, not args.no_dither,
                                freeze_phase=args.freeze_phase,
                                axis_off=f.axis_off,
                                mirror_u=args.mirror_u,
                                n_out=f.n_slices, gain=gains[f.name],
                                led_gamma=getattr(args, 'led_gamma', gas.LED_GAMMA))
            for f in faces)
        print(f'[render] frame {i}/{args.frames} {time.time() - t0:.1f}s', flush=True)
        yield raw


# ================= render 子命令 (预渲染到磁盘) =================

def cmd_render(args):
    out_dir = args.out_dir or os.path.join(HERE, f'frames_{args.anim}')
    os.makedirs(out_dir, exist_ok=True)
    faces, geom_flags = face_plan(args)
    expect = frame_raw_len(faces)                 # 单面 4423680 / 双面 8847360 / …
    check_layout(sum(f.n_slices for f in faces), geom_flags, '渲染计划: ')
    t0 = time.time()
    for i, raw in enumerate(gen_packed_frames(args)):
        assert len(raw) == expect, f'frame {i}: {len(raw)}B != {expect}B'
        path = os.path.join(out_dir, f'frame_{i:04d}.bin')
        with open(path, 'wb') as f:
            f.write(raw)
    # geom_flags/faces 写进 meta: stream --dir 时要靠它还原帧头 flags
    # (老目录没有这两个键 → 读成 0 = 传统单面, 逐字节兼容)
    # bpp 与 geom_flags 同等重要: stream --dir 光看字节数分不清 "360 片 1-bit"
    # 和 "120 片 3-bit" (都是 4423680B) —— 片距推错整帧就错位。
    meta = {'anim': args.anim, 'frames': args.frames, 'render_slices': args.render_slices,
            'frame_raw': expect, 'freeze_phase': bool(args.freeze_phase),
            'bpp': BPP,
            'led_gamma': (getattr(args, 'led_gamma', gas.LED_GAMMA)
                          if BPP == 3 else None),
            'n_slices': sum(f.n_slices for f in faces), 'geom_flags': geom_flags,
            'faces': [{'name': f.name, 'axis_off_px': round(f.axis_off, 4),
                       'n_slices': f.n_slices} for f in faces],
            'generated': time.strftime('%Y-%m-%d %H:%M:%S')}
    with open(os.path.join(out_dir, 'meta.json'), 'w') as f:
        json.dump(meta, f, indent=1)
    print(f'[render] {args.frames} frames -> {out_dir} ({time.time() - t0:.1f}s total)', flush=True)


# ================= 推流核心 (类 + 回调, GUI/CLI 共用) =================

class StreamerError(Exception):
    """协议级致命错 (NAK / 无重连时连接失败)。"""


class Streamer:
    """PVS1 推流器. run(make_iter) 阻塞直到帧尽/stop/致命错.

    make_iter: 无参可调用, 返回逐帧迭代器 (loop 时反复调用), 每项是
    raw 字节 (现场压缩) 或 FrameEntry (预压缩缓存, 热路径零压缩).
    帧头的 raw_len/n_slices 按每帧**实际长度**算 (n_slices = len//片距),
    不再假定 360 片 —— v3.1 双面帧 720 片 / 折叠双面 540 片都直接支持.
    geom_flags: 每帧 flags 恒 OR 上的几何位 (PVS_FLAG_DUAL_FACE/FOLD_A),
    描述载荷排布, 与压缩位正交; 由渲染参数或预渲染目录 meta.json 给出.
    DUAL_FACE 时 payload 自动出**两条独立压缩流** (见 compress_frame).
    window: 允许在途不等 ACK 的帧数 (默认 2 = 传输与板端解码重叠).
    几何变化点会先把窗口排空再切, 保证新旧几何的帧不同时在途.
    delta=True: 帧间 XOR delta (PVS_FLAG_DELTA), 关键帧周期 keyint;
    连接建立/重连后首帧强制 keyframe; **几何变化帧 (n_slices 或 DUAL_FACE/
    FOLD_A 相对上一帧变了) 也强制 keyframe** —— DELTA 是与上一帧逐字节 XOR,
    帧长/面布局一变参考帧就对不上 (板端直接 NAK 关连接);
    收到 delta 帧的 NAK (板端丢参考帧)
    自动降级重发 keyframe 后继续 (设计稿 §2.3), keyframe 的 NAK 仍致命.
    reconnect=True 时 ACK 超时/连接断 (板重启) → 每 retry_interval 秒重连,
    重连成功后重发当前帧继续.
    回调 (可选, 在推流线程里调):
      on_frame(stats)               每帧 ACK 后
      on_status(event, detail)      event ∈ connected/lost/retry/nak_delta/
                                    geom_change/done
      on_stats(line)                每 stats_interval 秒一行统计 (默认 print)
    stop: threading.Event, 置位后尽快退出 (sleep/重连等待均可打断).
    """

    def __init__(self, host, port=DEFAULT_PORT, fps=10.0, loop=False,
                 window=DEFAULT_WINDOW,
                 codec='zlib', zlevel=6, reconnect=False, retry_interval=5.0,
                 ack_timeout=30.0, delta=False, keyint=DEFAULT_KEYINT,
                 link_mbps=DEFAULT_LINK_MBPS, stats_interval=5.0,
                 max_frames=0, on_frame=None, on_status=None, on_stats=None,
                 stop=None, geom_flags=0, lz4_level=DEFAULT_LZ4_LEVEL,
                 stream_split='face'):
        self.host, self.port = host, port
        self.fps, self.loop = fps, loop
        self.codec, self.zlevel = codec, zlevel
        self.lz4_level = lz4_level
        self.stream_split = stream_split
        self.reconnect, self.retry_interval = reconnect, retry_interval
        self.ack_timeout = ack_timeout
        self.delta, self.keyint = delta, max(int(keyint), 1)
        self.geom_flags = int(geom_flags) & FLAG_LAYOUT
        self.link_mbps = link_mbps
        self.stats_interval = stats_interval
        self.max_frames = max_frames    # >0: ACK 满 N 帧自动停 (测试用)
        self.window = max(int(window), 1)   # 发送窗口: 1=stop-and-wait
        self._inflight = collections.deque()  # 已发未 ACK 的 (is_delta, wire, raw_len)
        self.on_frame = on_frame or (lambda st: None)
        self.on_status = on_status or (lambda ev, detail: None)
        self.on_stats = on_stats or (lambda line: print(line, flush=True))
        self.stop = stop if stop is not None else threading.Event()
        self.frames = 0                 # 已发送帧数 (window=1 时 == 已 ACK 帧数)
        self.sent_raw = 0
        self.sent_wire = 0
        self.last_wire = 0              # 最近一帧线上字节 (含 16B 头)
        self.last_raw_len = 0
        self.reconnects = 0
        self.delta_frames = 0           # 累计已 ACK 的 delta 帧数
        self.t_start = None
        self._since_key = None          # None = 下一帧必须 keyframe
        self._prev_raw = None           # 现场压缩 delta 的参考帧
        self._geom_key = None           # 上一帧的 (n_slices, 几何 flags)
        self._drain_err = None          # 排空窗口时收到的 NAK (run() 收尾抛)
        self._win = None                # 5s 统计窗 dict

    # -- 内部: 连接 (reconnect 时无限重试, stop 可打断; 返回 None = 被停) --
    def _connect(self, first):
        while not self.stop.is_set():
            try:
                sock = socket.create_connection((self.host, self.port), timeout=10)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                sock.settimeout(self.ack_timeout)
                self.on_status('connected', f'{self.host}:{self.port}')
                return sock
            except OSError as e:
                if not self.reconnect:
                    raise StreamerError(f'connect {self.host}:{self.port} failed: {e}')
                if not first:
                    self.reconnects += 1
                self.on_status('retry', f'{e}; retry in {self.retry_interval}s')
                self.stop.wait(self.retry_interval)
        return None

    # -- 内部: 按 keyframe 策略从 item 出线上 (payload, flags, is_delta, raw) --
    def _build_frame(self, item, want_key):
        if isinstance(item, FrameEntry):
            if want_key or item.delta is None:
                return item.key, item.key_flags, False, None
            return item.delta, item.delta_flags, True, None
        raw = item                       # 现场压缩路径 (bytes)
        split = face_split_bytes(self.geom_flags)   # DUAL_FACE → 两条独立流
        streams = stream_plan(len(raw) // SLICE_STRIDE,
                              self.geom_flags, self.stream_split)
        if (self.delta and not want_key and self._prev_raw is not None
                and len(self._prev_raw) == len(raw)
                and self.codec in DELTA_CODECS):
            # DELTA 位与压缩位正交: 板端先按压缩位解码再 XOR, 所以 zlib/lz4 都行
            payload, cflag = compress_frame(xor_frames(self._prev_raw, raw),
                                            self.codec, self.zlevel, split,
                                            self.lz4_level, streams)
            return payload, cflag | FLAG_DELTA, True, raw
        payload, flags = compress_frame(raw, self.codec, self.zlevel, split,
                                        self.lz4_level, streams)
        return payload, flags, False, raw

    def _send_one(self, sock, item):
        """发一帧等 ACK. 返回存活 sock (可能重连过) 或 None (被停).
        重连后 / delta 被 NAK 后, 本帧强制以 keyframe 形态重发."""
        force_key = False
        # 帧长按 item 实际字节算 (v3.1: 360/540/720 片都可能); n_slices 是
        # 接收方算长度的权威字段, 必须与 raw_len 自洽。
        raw_len = item.raw_len if isinstance(item, FrameEntry) else len(item)
        n_slices = raw_len // SLICE_STRIDE
        # ---- 几何变化 ⇒ 强制关键帧 (2026-07-31) ----
        # DELTA = 与上一帧逐字节 XOR。n_slices 或面布局 flags 一变, 参考帧长度/
        # 语义就对不上 (XOR 直接越界), 板端的行为是 NAK + 关连接。协议里
        # 「首帧必须关键帧」的触发条件在这里扩展到「几何变化帧」: 作废参考帧,
        # 本帧以 keyframe 发出, 之后再照常续 delta 链。
        # window>1 时窗口里可能还压着旧几何的帧 ⇒ **先把窗口排空再切**:
        # 否则新几何的帧与旧几何的帧同时在途, 板端的参考帧/面拆分会串。
        geom_key = (n_slices, self.geom_flags)
        if self._geom_key is not None and geom_key != self._geom_key:
            self._drain_inflight(sock)      # 排空: 切换点上退化成 stop-and-wait
            self._prev_raw = None
            self._since_key = None
            self.on_status('geom_change',
                           f'n_slices/flags {self._geom_key} → {geom_key}, '
                           f'排空发送窗口 + 强制关键帧')
            force_key = True
        self._geom_key = geom_key
        while not self.stop.is_set():
            if sock is None:
                sock = self._connect(first=False)
                if sock is None:
                    return None
                force_key = True         # 重连: 板端参考帧已失效
            want_key = (force_key or self._since_key is None
                        or self._since_key + 1 >= self.keyint)
            payload, flags, is_delta, raw = self._build_frame(item, want_key)
            hdr = HDR.pack(MAGIC, len(payload), raw_len, n_slices,
                           flags | self.geom_flags)
            try:
                sock.sendall(hdr + payload)
                # ---- 发送窗口 (--window N, 默认 2, 2026-07-31 上线) ----
                # window=1 = stop-and-wait: 发完立即等 ACK, 传输与板端解码完全串行,
                #   周期 = 传输 + 解码 (实测 40+64ms → ~9.6 fps)。
                # window>1: 允许 N 帧在途, 窗口满才回收一个 ACK ⇒ 传输与解码重叠,
                #   周期 → max(传输, 解码)。协议零改动 (04 §4.4: 板端 recv 缓冲
                #   天然吸收), 但板端 SO_RCVBUF 得吃得下约 1.5 帧压缩数据。
                #
                # ⚠ 关键: _prev_raw / _since_key 必须在**发送时**更新而非 ACK 时 ——
                # TCP 有序, 板端必按序处理, 故下一帧的 delta 参考帧一定有效。
                # 代价是 NAK 时参考帧失配, 但 NAK 路径本就走"断开重连 + 强制关键帧"。
                self._since_key = self._since_key + 1 if is_delta else 0
                if raw is not None:
                    self._prev_raw = raw
                # 字节记账在**发送时**做: window>1 时本帧的 ACK 可能这一轮不回收,
                # 放到 ACK 分支里会漏计 (旧代码靠"帧长恒定"侥幸对上)。
                wire_n = len(hdr) + len(payload)
                self.last_raw_len, self.last_wire = raw_len, wire_n
                self.sent_raw += raw_len
                self.sent_wire += wire_n
                self._inflight.append((is_delta, wire_n, raw_len))
                ack = None
                if len(self._inflight) >= self.window:
                    ack = sock.recv(1)
                else:
                    return sock          # 窗口未满: 不等 ACK, 直接发下一帧
                if ack == bytes([ACK]):
                    d_flag, wire, rl = self._inflight.popleft()
                    self._stat_frame(d_flag, wire, rl)
                    return sock
                if ack == bytes([NAK]):
                    # 收到的 ACK/NAK 属于窗口里**最早**那帧。只有它就是本帧
                    # (窗口里仅 1 个) 时才能原地降级重发, 否则管道里还排着后续
                    # 帧的 ACK 字节, 原地续发必然读错位 → 断连重来最干净。
                    mine = len(self._inflight) <= 1
                    d_flag = self._inflight[0][0]
                    self._inflight.clear()
                    if mine and is_delta:   # 板端丢参考帧 → 降级重发 keyframe (§2.3)
                        self.on_status('nak_delta',
                                       f'frame {self.frames}: NAK on delta, resend keyframe')
                        force_key = True
                        continue
                    try:
                        sock.close()
                    except OSError:
                        pass
                    sock = None
                    if not (d_flag and self.reconnect):
                        raise StreamerError(f'frame {self.frames}: NAK, abort')
                    # 窗口里的 delta 帧被 NAK: 重连后强制关键帧续流 (§2.3 的
                    # 流水线版本 —— 丢掉在途的几帧, 但不中断整条流)
                    self.reconnects += 1
                    self.on_status('nak_delta',
                                   f'frame {self.frames}: NAK on pipelined delta, '
                                   f'reconnect + keyframe')
                    force_key = True
                    continue
                raise OSError(f'bad/empty ack {ack!r} (peer closed?)')
            except (socket.timeout, OSError) as e:
                try:
                    sock.close()
                except OSError:
                    pass
                sock = None
                self._inflight.clear()      # 重连: 在途帧全部作废
                if not self.reconnect:
                    raise StreamerError(f'frame {self.frames}: {e}')
                self.reconnects += 1
                self.on_status('lost', f'{e}; reconnect in {self.retry_interval}s')
                self.stop.wait(self.retry_interval)
        return None

    def _drain_inflight(self, sock):
        """把发送窗口里剩余的在途帧 ACK 收完 (window>1 才会有)。
        收尾时调, 几何切换前也调 (切换点要退化成 stop-and-wait)。

        不排空的话最后 window-1 帧不进统计, 且板端 ACK 无人接收。
        网络异常忽略 (流已经结束, 收不到就算了); 但**排到 NAK 要记下来** ——
        window>1 时最后一帧的 NAK 只可能在这里被看到, 吞掉的话坏帧会静悄悄
        地过去 (run() 结束时再抛 StreamerError)。"""
        if sock is None:
            return
        try:
            while self._inflight:
                b = sock.recv(1)
                if b != bytes([ACK]):
                    if b == bytes([NAK]):
                        self._drain_err = (f'frame {self.frames}: NAK '
                                           f'(排空发送窗口时收到), abort')
                    break
                d_flag, wire, rl = self._inflight.popleft()
                self._stat_frame(d_flag, wire, rl)
        except OSError:
            pass
        self._inflight.clear()

    # -- 内部: 每收到一个 ACK 记一帧 (5s 滑窗: 页率/码率/链路占用/delta 压缩比) --
    # ⚠ ACK 回收有两条路径 (窗口满时的 recv 和收尾/切换时的 _drain_inflight),
    #   累计计数放这里, 两条路都算得到 (window>1 时最后 window-1 帧走 drain)。
    # raw_len 必须由调用方给 (两处都给了): 默认值会在 import 时就把 FRAME_RAW
    # 冻成 1-bit 360 片的 4423680, --bpp 3 / --n-slices 之后再取就是错的,
    # 而它只影响统计行 —— 悄悄报一个假的 raw 字节数/压缩比, 没人会发现。
    def _stat_frame(self, is_delta, wire, raw_len):
        if is_delta:
            self.delta_frames += 1
        now = time.time()
        w = self._win
        if w is None:
            w = self._win = dict(t0=now, frames=0, wire=0, d_frames=0, d_wire=0,
                                 d_raw=0, k_frames=0, k_wire=0, k_raw=0)
        w['frames'] += 1
        w['wire'] += wire
        if is_delta:
            w['d_frames'] += 1; w['d_wire'] += wire; w['d_raw'] += raw_len
        else:
            w['k_frames'] += 1; w['k_wire'] += wire; w['k_raw'] += raw_len
        dt = now - w['t0']
        if dt < self.stats_interval:
            return
        mbps = w['wire'] * 8 / dt / 1e6
        line = (f"[stats/{self.stats_interval:.0f}s] 页率 {w['frames'] / dt:.1f}/s"
                f" | 码率 {mbps:.2f} Mbps"
                f" | 链路占用 {mbps / max(self.link_mbps, 1e-6) * 100:.0f}%"
                f" (@{self.link_mbps:g} Mbps)")
        if self.delta:
            dr = (w['d_raw'] / w['d_wire']) if w['d_wire'] else 0.0
            kr = (w['k_raw'] / w['k_wire']) if w['k_wire'] else 0.0
            line += (f" | delta {w['d_frames']}/{w['frames']} 帧"
                     f" 压缩 {dr:.0f}x (key {kr:.0f}x)")
        self._win = None
        self.on_stats(line)

    def run(self, make_iter):
        self.t_start = time.time()
        t_next = self.t_start
        sock = self._connect(first=True)
        try:
            while sock is not None and not self.stop.is_set():
                for item in make_iter():
                    if self.stop.is_set():
                        break
                    sock = self._send_one(sock, item)
                    if sock is None:
                        break
                    self.frames += 1     # 字节记账已在 _send_one 发送时做
                    self.on_frame(self)
                    if self.max_frames and self.frames >= self.max_frames:
                        self.stop.set()
                        break
                    if self.fps > 0:
                        t_next = max(t_next + 1.0 / self.fps, time.time() - 1.0 / self.fps)
                        dt = t_next - time.time()
                        if dt > 0:
                            self.stop.wait(dt)
                if not self.loop:
                    break
        finally:
            # ⚠ 顺序: 必须先排空在途 ACK 再关 socket。反了的话 sock 已关,
            # 排空必失败, 且对端正在写 ACK 会撞 ConnectionAbortedError。
            self._drain_inflight(sock)
            if sock is not None:
                sock.close()
            self.on_status('done', f'{self.frames} frames')
        # 排空窗口时收到的 NAK 在这里才抛 —— 放 finally 里抛会把正在传播的
        # 真异常盖掉 (try 里已经出错时, 那个错才是根因)。
        if self._drain_err:
            err, self._drain_err = self._drain_err, None
            raise StreamerError(err)
        return self

    # -- 统计便利 --
    def elapsed(self):
        return max(time.time() - (self.t_start or time.time()), 1e-6)

    def wire_mbps(self):
        return self.sent_wire / self.elapsed() / 1e6

    def ratio(self):
        return self.sent_raw / max(self.sent_wire, 1)


# ================= stream 子命令 =================

def frame_iter_from_dir(d):
    n0 = None
    for p in frame_files_from_dir(d):
        raw = open(p, 'rb').read()
        try:
            slices_of(len(raw), f'{p}: ')
        except ValueError as e:
            sys.exit(str(e))
        if n0 is None:
            n0 = len(raw)
        elif len(raw) != n0:
            sys.exit(f'{p}: {len(raw)}B != 首帧 {n0}B (同目录帧长必须一致)')
        yield raw


def geom_flags_from_dir(d):
    """预渲染目录 → 几何 flags (DUAL_FACE/FOLD_A), 取自 cmd_render 写的
    meta.json。老目录没这个键 → 0 = 传统单面 360 片, 行为逐字节不变。

    ⚠ 必须带上: 载荷排布 (单面/双面/折叠) 光靠 n_slices 推不出来 ——
    540 片既可能是 [折叠A 180][B 360] 也可能是别的切法, 板端要 flags 才知道
    怎么分给两个 DDR 基址。"""
    try:
        with open(os.path.join(d, 'meta.json')) as f:
            meta = json.load(f)
    except (OSError, ValueError):
        return 0
    fl = int(meta.get('geom_flags', 0)) & FLAG_GEOM
    if fl:
        names = ' '.join(n for b, n in ((FLAG_DUAL_FACE, 'DUAL_FACE'),
                                        (FLAG_FOLD_A, 'FOLD_A')) if fl & b)
        print(f'[meta] {d}: geom flags=0x{fl:04x} ({names}), '
              f'n_slices={meta.get("n_slices")}, faces={meta.get("faces")}', flush=True)
    return fl


def bpp_from_dir(d, cli_bpp):
    """预渲染目录 → 色深, 取自 cmd_render 写的 meta.json (与几何 flags 同理:
    数据已经渲好了, 目录说了算)。老目录没这个键 → 1 = 1-bit, 行为不变。

    ⚠ 必须落回全局: 片距一错, slices_of / 面拆分 / MSTREAM 全线错位, 而且
    "360 片 1-bit" 与 "120 片 3-bit" 的字节数一模一样, 光看长度查不出来。"""
    try:
        with open(os.path.join(d, 'meta.json')) as f:
            bpp = int(json.load(f).get('bpp', 1))
    except (OSError, ValueError, TypeError):
        bpp = 1
    if bpp not in pack_obs.BPP_MODES:
        sys.exit(f'{d}/meta.json: bpp={bpp} 非法 (只有 {pack_obs.BPP_MODES})')
    if bpp != cli_bpp:
        print(f'[meta] {d}: bpp={bpp} (命令行给的是 {cli_bpp}) — 以目录为准, '
              f'片距 0x{pack_obs.slice_stride(bpp):X}', flush=True)
    _apply_bpp(bpp)
    _apply_slot_count(N_SLICES)          # 片距变了, 重算 FRAME_RAW
    return bpp


# meta 的 n_slices 是**各面之和**, 反解回"单面整圈槽数" N_SLICES 的比例
# (= expected_n_slices 的逆): 总数 = N × num/den。
_SLOT_RATIO = {0: (1, 1),                                   # 单面整圈   → N
               FLAG_FOLD_A: (1, 2),                         # 单面折叠   → N/2
               FLAG_DUAL_FACE: (2, 1),                      # 双面整圈   → 2N
               FLAG_DUAL_FACE | FLAG_FOLD_A: (3, 2)}        # 双面+折叠  → 1.5N


def slots_from_dir(d, cli_ns, geom_flags):
    """预渲染目录 → 一圈槽数, 落回模块全局 N_SLICES (与 bpp_from_dir 同理:
    数据已经渲好了, **目录说了算**, 命令行 --n-slices 只是老目录的兜底)。

    🔴 2026-08-20 修: 以前 --dir 只从 meta 取 bpp 和几何 flags, 槽数还是拿命令行
    的默认 360, 于是任何 --n-slices != 360 渲出来的目录都推不出去:
      · 轻的 (单面): check_layout 直接判 "n_slices=50 与 flags 不自洽, 板端要求
        360 片" 退出 —— 明明目录自己写着 n_slices=50;
      · 重的 (DUAL_FACE): face_split_bytes / stream_plan 仍按 N_SLICES=360 定
        面A 的字节数, 拆流点整体错位。长度还是对的, 板端两个核照收不误,
        解出来却是错的 —— 这种错没有任何一层会报出来。
    老目录 (meta 里没有 n_slices, 或压根没有 meta.json) → 沿用命令行值,
    逐字节兼容; 360 片的老目录也走同一条路径, 反解回来就是 360。"""
    try:
        with open(os.path.join(d, 'meta.json')) as f:
            total = json.load(f).get('n_slices')
        total = None if total is None else int(total)
    except (OSError, ValueError, TypeError):
        total = None
    if total is None:
        _apply_slot_count(cli_ns)
        return cli_ns
    num, den = _SLOT_RATIO[geom_flags & FLAG_GEOM]
    ns = total * den // num
    if not 1 <= ns <= N_SLICES_MAX or ns * num != total * den:
        sys.exit(f'{d}/meta.json: n_slices={total} 与几何 flags='
                 f'0x{geom_flags & FLAG_GEOM:04x} 反解不出合法的单面槽数 '
                 f'(总数须是单面槽数的 {num}/{den} 倍, 且单面槽数 ∈ '
                 f'1..{N_SLICES_MAX})')
    if ns != cli_ns:
        print(f'[meta] {d}: 一圈 {ns} 槽 (命令行给的是 {cli_ns}) — 以目录为准, '
              f'帧共 {total} 片 × 0x{SLICE_STRIDE:X}', flush=True)
    _apply_slot_count(ns)
    return ns


def cmd_stream(args):
    if args.delta and args.codec not in DELTA_CODECS:
        sys.exit(f'--delta 需要 --codec {"/".join(DELTA_CODECS)} '
                 f'(协议 DELTA 与压缩位正交, 但 raw/rle 侧没实现)')
    # 几何 flags: --dir 走目录 meta.json, 现渲走 CLI (face_plan 同时做合法性检查)
    if args.dir:
        bpp_from_dir(args.dir, args.bpp)          # 片距: 目录 meta 说了算
        geom_flags = geom_flags_from_dir(args.dir) | bpp_flag()
        # 槽数也一样以目录为准 (要排在 bpp 之后: N_SLICES_MAX 随片距变)
        slots_from_dir(args.dir, args.n_slices, geom_flags)
        if args.dual_face or args.fold_a:
            print('[net] ⚠ --dir 模式下 --dual-face/--fold-a 不生效, '
                  '几何以目录 meta.json 为准 (帧数据已经渲好了)', flush=True)
        f0 = frame_files_from_dir(args.dir)[0]
        try:
            n0 = slices_of(os.path.getsize(f0), f'{f0}: ')
        except ValueError as e:
            sys.exit(str(e))
        check_layout(n0, geom_flags, f'{args.dir}: ')
    else:
        _faces, geom_flags = face_plan(args)
        check_layout(sum(f.n_slices for f in _faces), geom_flags, '渲染计划: ')
    if args.codec == 'lz4':
        if args.lz4_level in LZ4_BAD_LEVELS:
            print(f'[net] ⚠ --lz4-level {args.lz4_level} 实测**反而比 9 更差** '
                  f'(413178 vs 388166 B, 可复现), 建议 9/11/12', flush=True)
        if not args.dir or args.no_precomp:
            # HC12 x86 单核 ~1 s/帧, HC9 也要 ~135 ms/帧 —— 现渲直推兜不住
            print(f'[net] ⚠ --codec lz4 走的是**逐帧现场压缩** (HC{args.lz4_level} '
                  f'约 0.1~1 s/帧), 帧率会被编码封顶。正常用法是 '
                  f'`render` 预渲染到目录后 `stream --dir` 走预压缩缓存。', flush=True)
    entries = None
    if args.dir and args.codec in DELTA_CODECS and not args.no_precomp:
        entries = build_precomp(args.dir, zlevel=args.zlevel,
                                delta=args.delta, jobs=args.jobs,
                                geom_flags=geom_flags, codec=args.codec,
                                lz4_level=args.lz4_level,
                                stream_split=args.stream_split)

    def make_iter():
        if entries is not None:
            return iter(entries)
        return frame_iter_from_dir(args.dir) if args.dir else gen_packed_frames(args)

    def on_status(ev, detail):
        if ev == 'connected':
            print(f'[net] connected {detail} codec={args.codec} fps={args.fps}'
                  + (f' delta keyint={args.keyint}' if args.delta else ''), flush=True)
        elif ev in ('lost', 'retry', 'nak_delta'):
            print(f'[net] {ev}: {detail}', flush=True)

    def on_frame(st):
        if args.verbose:
            print(f'  frame {st.frames}: {st.last_wire - HDR.size}B wire '
                  f'({st.last_raw_len / max(st.last_wire - HDR.size, 1):.1f}x)', flush=True)

    s = Streamer(args.host, args.port, fps=args.fps, loop=args.loop,
                 window=args.window,
                 codec=args.codec, zlevel=args.zlevel, lz4_level=args.lz4_level,
                 stream_split=args.stream_split, reconnect=args.reconnect,
                 delta=args.delta, keyint=args.keyint, link_mbps=args.link_mbps,
                 max_frames=args.max_frames, geom_flags=geom_flags,
                 on_frame=on_frame, on_status=on_status)
    try:
        s.run(make_iter)
    except KeyboardInterrupt:
        print('[net] interrupted', flush=True)
    except StreamerError as e:
        print(f'[net] {e}', flush=True)
        if s.frames == 0:
            sys.exit(1)
    dt = s.elapsed()
    print(f'[stats] {s.frames} frames in {dt:.2f}s = {s.frames / dt:.2f} model fps | '
          f'wire {s.wire_mbps():.2f} MB/s (raw {s.sent_raw / dt / 1e6:.2f} MB/s) | '
          f'compression {s.ratio():.1f}x', flush=True)
    print(f'[stats] projected fps @ 9.4 MB/s link: '
          f'{9.4e6 / (s.sent_wire / max(s.frames, 1)):.1f}', flush=True)


# ================= bench 子命令 =================

def cmd_bench(args):
    raw = open(args.file, 'rb').read()
    zero = (np.frombuffer(raw, np.uint8) == 0).mean()
    print(f'{args.file}: {len(raw)}B, {zero * 100:.1f}% zeros')
    rows = []
    # dec_ms 这一列是 **x86 上** 的解压耗时, 只能用来横向比较编解码器;
    # 板端 (A9) 的绝对值另测 (protocol.h 的 PVS_FLAG_LZ4 注释里有实测表)。
    # ⚠ lz4 那两行的 dec_ms 还含 ctypes 每次新建 8.8MB 输出缓冲(清零)的开销,
    #   纯解压其实快一个量级 (同一份数据的 C 侧对拍: lz4 0.59ms vs zlib 7.5ms)。
    lz4_dec = lambda d: lz4_decompress(d, len(raw))          # noqa: E731
    for name, fn, dec in [('zlib-1', lambda d: zlib.compress(d, 1), zlib.decompress),
                          ('zlib-6', lambda d: zlib.compress(d, 6), zlib.decompress),
                          ('lz4-1', lambda d: lz4_compress(d, 1), lz4_dec),
                          ('lz4-HC9', lambda d: lz4_compress(d, 9), lz4_dec),
                          ('rle', rle_encode, rle_decode)]:
        t0 = time.time(); c = fn(raw); te = time.time() - t0
        t0 = time.time(); d = dec(c); td = time.time() - t0
        assert d == raw, name
        rows.append((name, len(c), len(raw) / len(c), te, td))
    print(f'{"codec":8} {"bytes":>9} {"ratio":>6} {"enc_ms":>7} {"dec_ms":>7} {"fps@9.4MB/s":>12}')
    for name, sz, ratio, te, td in rows:
        print(f'{name:8} {sz:9} {ratio:5.1f}x {te * 1e3:7.0f} {td * 1e3:7.0f} {9.4e6 / sz:12.1f}')


# ================= CLI =================

def add_render_opts(ap):
    ap.add_argument('--anim', choices=sorted(ANIMS), default='spinpulse')
    ap.add_argument('--frames', type=int, default=36, help='动画帧数 (=循环周期)')
    ap.add_argument('--half-aspect', action='store_true',
                    help='配 --half-screen: 半径方向也压一半保持宽高比 (物体小一号但不变形)')
    ap.add_argument('--half-screen', action='store_true',
                    help='内容压到下半屏 Y90..179, 配 RTL half_scan (整屏拍数减半, 槽数翻倍)')
    ap.add_argument('--helix-turns', type=float, default=2.5, help='rgbhelix 螺旋圈数')
    ap.add_argument('--helix-r', type=float, default=0.55,
                    help='rgbhelix 螺旋中心线半径 (占 R_BUDGET); 大 => 中心留空更多')
    ap.add_argument('--tube-r', type=float, default=0.13, help='rgbhelix 管半径')
    ap.add_argument('--cyl-radius', type=float, default=0.46,
                    help='rgbcyl 半径 (占 R_BUDGET 的比例); --half-screen 下用 0.23 才不变形')
    ap.add_argument('--cyl-shells', type=int, default=10,
                    help='rgbcyl 径向壳层数 (1=只有表面 => 切片上圆柱会中间断开)')
    ap.add_argument('--tilt', type=float, default=30.0,
                    help='rgbcyl 圆柱轴的倾斜角(度), 0=竖直(旋转对称, 看不出在转)')
    ap.add_argument('--cube-grid', type=int, default=340,
                    help='rgbcube 每面采样网格边长 (6 面共 6*n^2 点)')
    ap.add_argument('--render-slices', type=int, default=0,
                    help='实际渲染角度数 (整除槽数, 减少省时, 布局仍是槽数那么多槽); '
                         '0 = 跟 --n-slices 走 (每槽一个真实角度)')
    # 🔴 2026-08-10: 把"一圈多少槽"从写死的 360 变成可调。
    # 为什么: 面板实测 2D 刷新只有 ~1340 Hz, 15 rps 下每圈只画得出 ~89 个不同角度
    # (见 project_pov3d_refresh_vs_rpm)。渲 360 片里有 3/4 根本没机会上屏,
    # 而链路/解码/memcpy 为它们付的代价是实打实的。
    # 协议侧本来就支持: protocol.h:37「头里的 n_slices 是权威, 不再硬校验 == 360」,
    # 范围 1..720 ⇒ **板端零改动**。
    ap.add_argument('--n-slices', type=int, default=360,
                    help='一圈的槽数 (=帧里的 n_slices, 1..720)。默认 360 保持原行为; '
                         '90 ≈ 面板 1340Hz @15rps 的真实能力, 载荷/解码/memcpy 全线 ÷4')
    ap.add_argument('--sub', type=int, default=3)
    # 🔴 2026-08-20 v3.4: 每通道色深。默认 1 = 今天在跑的一切 (空闲动画、板上
    # 默认内容、30+ 套 frames_* 目录) 逐字节不变; 3 = 行内 BCM 8 级,
    # 一片 0x3000 → 0x9000, 帧头置 PVS_FLAG_3BIT。见 05_3bit_bcm.md。
    ap.add_argument('--bpp', type=int, choices=sorted(pack_obs.BPP_MODES), default=1,
                    help='每通道位深: 1 = 老行为 (逐字节兼容, 默认); '
                         '3 = 每通道 8 级 (行内 BCM, 片距 0x9000, 帧头 PVS_FLAG_3BIT)。'
                         '⚠ 3-bit 时片数上限 240 (帧长上限 8847360B 是硬的), '
                         '方案推荐每面 60 槽')
    # LED 是线性发光 (BCM 权重 27/54/108 = 1:2:4), 人眼与素材都是 ~2.2 次方的
    # 感知域 ⇒ 量化前必须先解码到线性光。取值理由见 gas.to_3bit / 下面 help。
    ap.add_argument('--led-gamma', type=float, default=gas.LED_GAMMA,
                    help=f'[--bpp 3] 量化前的 gamma 解码指数 (默认 {gas.LED_GAMMA}): '
                         'code = 7·(v/255)^gamma。素材是 sRGB (γ≈2.2) 编码而 LED '
                         '码值与发光时间成正比 ⇒ 2.2 是「测出来的光 = 素材的意图」'
                         '的那个值, 也让线性灰度楔看起来是均匀的 8 阶。'
                         '嫌暗部被压掉太多可调到 1.8~2.0 (提亮暗部, 高光变平)。'
                         '1.0 = 不做解码 (整幅偏亮, 中灰会亮成两倍多)')
    ap.add_argument('--thresh', type=float, default=128,
                    help='1-bit: Bayer 阈值均值; 3-bit: 曝光倍数 = 128/thresh '
                         '(默认 128 ⇒ 1.0 = 满量程 0..255), 方向与 1-bit 一致 (小=亮)')
    ap.add_argument('--no-dither', action='store_true')
    ap.add_argument('--freeze-phase', action='store_true',
                    help='Bayer 抖动相位只随 slot 不随 frame (时域抖动冻结, '
                         'delta 模式静止区域帧间零变化; 代价: 抖动纹理静止化)')
    # spinpulse / glbanim
    ap.add_argument('--glb', default=gas.DEFAULT_GLB)
    ap.add_argument('--samples', type=int, default=None,
                    help='GLB 采样点数 (默认: spinpulse 1800000, glbseq/glbanim 400000)')
    ap.add_argument('--z-stretch', type=float, default=1.0)
    ap.add_argument('--ssaa', type=int, default=1,
                    help='N× 空间超采样抗锯齿: 细分格覆盖率→边缘调暗→Bayer 变点密度 (3 推荐, 配 --samples 1.2M+)')
    ap.add_argument('--scale', type=float, default=1.0,
                    help='spin: 归一化后整体缩放 (地球仪 0.48=直径半幅)')
    ap.add_argument('--shells', type=int, default=1,
                    help='spin: 点云洋葱状向内复制层数 (壳太薄时加厚)')
    ap.add_argument('--shell-gap', type=float, default=1.3,
                    help='spin: 壳层间距 (voxel)')
    ap.add_argument('--window', type=int, default=DEFAULT_WINDOW,
                    help=f'发送窗口: 允许 N 帧在途不等 ACK (默认 {DEFAULT_WINDOW})。'
                         '1=stop-and-wait(旧行为), 周期 = 传输+解码; '
                         '2 起传输与板端解码重叠, 周期降到 max(传输,解码)。'
                         '协议零改动, 但板端 SO_RCVBUF 要吃得下约 1.5 帧压缩数据 '
                         '(双面 540 片 ≈ 300KB); 板端缓冲小就调回 1')
    ap.add_argument('--fit-frames', action='store_true',
                    help='glbanim: 逐帧自适应缩放(吃满体积), 配 --fit-smooth 平滑防抖; '
                         '不加则用整段并集 bbox (单帧偏小)')
    ap.add_argument('--fit-smooth', type=int, default=9,
                    help='--fit-frames 的循环滑动平均窗口(帧); 0/1=不平滑')
    ap.add_argument('--fit-pct', type=float, default=1.0,
                    help='--fit-frames 的分位裁剪 %% (剔除离群点, 默认 1 → 用 [1,99])')
    ap.add_argument('--y-offset', type=float, default=0.0,
                    help='glbanim: 归一化后 y 平移 (voxel, 正值向上)。整段动画的并集 bbox '
                         '常让单帧偏离体积中心, 用它校正; 先渲 1 帧量包络再定值')
    ap.add_argument('--gap-mm', type=float, default=13.8,
                    help='[v3 对称装] 双屏间距 mm (每屏到轴垂距=一半); 0=穿心旧几何')
    ap.add_argument('--radial-comp', action='store_true',
                    help='逐列亮度补偿: ①径向 1/r (轴心过亮 -> 增益∝r) ②双面覆盖阶跃 '
                         '(r<13.4mm 只被穿心面照到, 补 2x)。1-bit 内容靠抖动密度体现, '
                         '**只能变暗不能变亮**, 代价是暗列损失空间细节。'
                         '⚠ 径向补偿在 POV 上可能触及 Voxon P3 专利, 产品化前须确认。')
    ap.add_argument('--radial-floor', type=float, default=0.12,
                    help='--radial-comp 的增益下限 (default 0.12); 纯∝r 会让轴心全黑')
    ap.add_argument('--face-off-mm', type=float, default=None, metavar='MM',
                    help='[v3.1 偏心装] 本面到转轴的垂距 mm, 覆盖 --gap-mm。'
                         'A 面(贴轴)=0 / B 面=13.4。两面不对称 → 各渲一份, '
                         '不能再用 PHASE_B=180 共用同一份数据')
    ap.add_argument('--dual-face', action='store_true',
                    help='[v3.1 偏心装] 一帧渲**两个面**并拼接 [面A 全部片][面B 全部片]: '
                         f'面A 垂距 {gas.V31_OFF_A_MM}mm (穿心) / 面B {gas.V31_OFF_B_MM}mm。'
                         '帧长翻倍 (720 片 = 8,847,360B), 帧头置 PVS_FLAG_DUAL_FACE。'
                         '两面不对称后 PHASE_B=180 共用一份数据已作废, 必须各渲一份')
    ap.add_argument('--fold-a', action='store_true',
                    help='[v3.1 偏心装] 面A 只渲 180 片 (θ=0..179°), 帧头再置 '
                         'PVS_FLAG_FOLD_A, 板端/PL 用 idx-180 + 镜像置换补齐 180..359。'
                         '仅面A 垂距 0 (穿心) 时合法 —— 发帧前跑穿心镜像自检, '
                         '不过直接报错退出。⚠ 代价: 抖动相位随数据一起被复制, '
                         '每转独立 Bayer 相位 360→180 种, 时域抖动平滑效果减半 '
                         '(灰度过渡略糙); 也因此折叠输出与完整 360 片输出**不会**逐字节相同')
    ap.add_argument('--no-mirror-u', dest='mirror_u', action='store_false', default=True,
                    help='关全局中心轴镜像 (默认开, 2026-07-28 上板实测)')
    ap.add_argument('--robust-fit', action='store_true',
                    help='glbanim: [2,98] 分位 bbox 归一, 抗技能位移/特效骨骼压小人物')
    ap.add_argument('--x-offset', type=float, default=0.0,
                    help='glbanim: 归一化后 x 平移 (voxel), 正值离转轴 (屏中央显示差规避)')
    ap.add_argument('--pure-rgb', action='store_true',
                    help='glbanim: 1-bit 海报化, 每点snap到主导纯通道组合 (密度上限, 7色)')
    ap.add_argument('--pure-dom', type=float, default=0.55,
                    help='pure-rgb 主导阈值 (通道≥dom×max 点亮, 越低越偏白)')
    ap.add_argument('--brighten', type=float, default=1.5)
    ap.add_argument('--gamma', type=float, default=0.9)
    ap.add_argument('--saturation', type=float, default=2.0)
    ap.add_argument('--lighting', default='lambert')
    ap.add_argument('--ambient', type=float, default=0.7)
    ap.add_argument('--breath', type=float, default=0.05, help='呼吸缩放幅度')
    ap.add_argument('--bob', type=float, default=2.5, help='竖直浮动幅度 (voxel)')
    ap.add_argument('--sway', type=float, default=3.0, help='披风 x-shear 幅度 (voxel)')
    # glbseq / glbanim
    ap.add_argument('--glb-dir', default=None,
                    help='glbseq: GLB 帧序列目录 (sorted *.glb, 每文件一帧)')
    ap.add_argument('--anim-take', default='0',
                    help='glbanim: 动画 take 名或索引 (默认 0)')


def main():
    ap = argparse.ArgumentParser(description='POV volumetric display PC streamer (PVS1)')
    sub = ap.add_subparsers(dest='cmd', required=True)

    r = sub.add_parser('render', help='预渲染动画帧到磁盘')
    add_render_opts(r)
    r.add_argument('--out-dir', default=None, help='默认 stream/pc/frames_<anim>/')
    r.set_defaults(fn=cmd_render)

    s = sub.add_parser('stream', help='推流 (--dir 预渲染目录, 或 --anim 现渲)')
    add_render_opts(s)
    s.add_argument('--dir', default=None, help='预渲染 .bin 帧目录 (file 源)')
    s.add_argument('--host', default='127.0.0.1')
    s.add_argument('--port', type=int, default=DEFAULT_PORT)
    s.add_argument('--fps', type=float, default=12.0,
                   help='发送节奏上限。🔴 设成**等于转速** (900 RPM = 15) —— '
                        '别再"设高于转速让 ACK 自己贴住翻页率"了, 那条老经验只在'
                        '链路是瓶颈时成立。链路修好后 (2026-08-06) 发 40 板端能'
                        '解 22 帧/秒, 但每圈只翻得动 1 帧 = 最多 15, 多解的 7 帧'
                        '纯属白解, 还要和 flip 线程抢 DDR: 实测 cpy 25→32ms、'
                        'dec 25→32ms, 上屏反而从 15.00 掉到 12.99 帧/秒 '
                        '(交错 A/B 4x20s, 效应量 1.43)。')
    s.add_argument('--loop', action='store_true')
    s.add_argument('--reconnect', action='store_true',
                   help='连接断/ACK 超时不退出, 每 5s 重连 (板重启自动续推)')
    s.add_argument('--codec', choices=['zlib', 'lz4', 'rle', 'raw'], default='zlib',
                   help='压缩位 (protocol.h)。默认 zlib = 与所有已有 frames_* '
                        '预渲染目录逐字节兼容的老行为; lz4 = LZ4 raw block, '
                        '压缩比几乎相同但板端解压快 4 倍 (A9 实测 163.5→41.2ms)')
    s.add_argument('--zlevel', type=int, default=6)
    s.add_argument('--lz4-level', type=int, default=DEFAULT_LZ4_LEVEL,
                   help=f'--codec lz4 的 LZ4_compress_HC 级别 1..12 '
                        f'(默认 {DEFAULT_LZ4_LEVEL}; ⚠ 10 实测比 9 还差 6%%, '
                        f'用 9/11/12)')
    s.add_argument('--stream-split', choices=['face', 'balanced'], default='face',
                   help='载荷拆成几条独立压缩流 (板端并行解码的粒度)。'
                        'face = 按面切 (默认, 与老固件/老行为逐字节一致); '
                        'balanced = 按板端两核的**工作量**切 (PVS_FLAG_MSTREAM), '
                        'fold540 下 180/90/270 三条, 两核 makespan 20.6→15.47ms')
    s.add_argument('--delta', action='store_true',
                   help='帧间 XOR delta (PVS_FLAG_DELTA), 首帧/重连首帧自动 keyframe')
    s.add_argument('--keyint', type=int, default=DEFAULT_KEYINT,
                   help=f'delta 关键帧周期 (默认 {DEFAULT_KEYINT})')
    s.add_argument('--link-mbps', type=float, default=DEFAULT_LINK_MBPS,
                   help=f'链路估值 Mbps, 统计行算占用%% (默认 {DEFAULT_LINK_MBPS:g}: 2.4G)')
    s.add_argument('--no-precomp', action='store_true',
                   help='禁用 --dir 预压缩缓存 (回退逐帧现场压缩)')
    s.add_argument('--jobs', type=int, default=0,
                   help='预压缩进程数 (0=CPU 数)')
    s.add_argument('--max-frames', type=int, default=0,
                   help='ACK 满 N 帧自动停 (回环测试用, 0=不限)')
    s.add_argument('-v', '--verbose', action='store_true')
    s.set_defaults(fn=cmd_stream)

    b = sub.add_parser('bench', help='压缩方案测量')
    b.add_argument('--file', default=os.path.join(TOOLS, 'anime_slices.bin'))
    b.set_defaults(fn=cmd_bench)

    args = ap.parse_args()

    # 🔴 顺序要紧: 槽数必须在任何渲染/打包之前落到全局 (FRAME_RAW 等按它算)。
    # 🔴 色深要排在槽数前面: 片距变了槽数上限和 FRAME_RAW 都跟着变。
    bpp = getattr(args, 'bpp', 1)
    if bpp != 1:
        _apply_bpp(bpp)
        print(f'[bpp] 每通道 {bpp} bit: 片距 0x{SLICE_STRIDE:X}, 片数上限 '
              f'{N_SLICES_MAX}, 帧头置 PVS_FLAG_3BIT(0x{FLAG_3BIT:04x}), '
              f'--led-gamma {getattr(args, "led_gamma", gas.LED_GAMMA)}', flush=True)
    if getattr(args, 'half_screen', False):
        global HALF_SCREEN, HALF_ASPECT
        HALF_SCREEN = True
        HALF_ASPECT = bool(getattr(args, 'half_aspect', False))
        print('[half] 内容压到下半屏 Y90..179 (相邻两行取 max); 需配 RTL half_scan '
              '(0x0C sub01 [18]) —— 整屏 31590->16038 拍, 槽数可翻倍。'
              '⚠ 上半屏会出现一份差一个扫描行的拷贝, 移位链固有, 只看下半屏。', flush=True)
    ns = getattr(args, 'n_slices', None)
    if ns is not None:
        if not 1 <= ns <= N_SLICES_MAX:
            sys.exit(f'--n-slices 须在 1..{N_SLICES_MAX} '
                     f'(bpp={BPP} 片距 0x{SLICE_STRIDE:X} 下的上限; 给的是 {ns})')
        _apply_slot_count(ns)
    # --render-slices 0 = 每槽一个真实角度 (不复制填充)
    if not getattr(args, 'render_slices', 0):
        args.render_slices = N_SLICES
    if N_SLICES % args.render_slices:
        sys.exit(f'--render-slices {args.render_slices} 必须整除槽数 {N_SLICES}')

    args.fn(args)


if __name__ == '__main__':
    main()
