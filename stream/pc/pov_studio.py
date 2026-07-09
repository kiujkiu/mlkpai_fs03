#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pov_studio.py — POV Studio: POV 体显示一站式 GUI (Windows Python 3.12, tkinter).

三段式界面:
  设备:  IP / 扫描 (/23 网段找板) / 在线·推流口·ssh 状态灯 (5s 刷新) / 板日志
  内容:  源 (GLB / GLB 序列目录 / 内置地球仪 / 预渲染目录) + 源相关预设
         (GLB: 静态/呼吸/GLB自带动画+take; 序列目录: 逐帧, 帧数=文件数)
         + 帧数 → 渲染 + slice 预览
  推流:  fps / 循环 / 自动重连 (板重启每 5s 重连续推) + 实时统计
         + 设为开机默认动画 (scp 覆盖板上 /home/uisrc/anime_slices.bin + md5 校验)

依赖: stdlib + numpy + PIL, 复用 povstream.py (渲染/推流) + pack_obs.py (解包预览)。
配置持久化: 本目录 pov_studio.json。
非 GUI 逻辑全部模块级函数/类, WSL 无 tkinter 也可 import (test_studio.py headless 测)。
"""
import os
import sys
import json
import time
import glob
import queue
import socket
import hashlib
import argparse
import threading
import subprocess
import ipaddress
import concurrent.futures

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.abspath(os.path.join(HERE, '..', '..', 'tools'))
sys.path.insert(0, TOOLS)
sys.path.insert(0, HERE)

import numpy as np
from PIL import Image

import pack_obs
import gen_anime_slices as gas
import povstream
from povstream import FRAME_RAW, DEFAULT_PORT, Streamer, StreamerError

# ================= 常量 =================

BOARD_MAC = '90:de:80:35:1c:47'
BOARD_USER = 'uisrc'
BOARD_BIN = '/home/uisrc/anime_slices.bin'
BOARD_LOGS = ['/home/uisrc/pov_rxd.log', '/home/uisrc/pov_boot.log']
CONFIG_PATH = os.path.join(HERE, 'pov_studio.json')

DEFAULT_CONFIG = {
    'ip': '10.10.20.234',
    'ssh_key': (r'C:\Users\kiujkiu\.ssh\pov_ed25519' if os.name == 'nt'
                else os.path.expanduser('~/.ssh/pov_ed25519')),
    'source': 'glb',            # glb | glbdir | globe | dir
    'glb': '',                  # 空 = povstream 默认 anime GLB
    'glb_dir': '',              # GLB 帧序列目录 (glbseq)
    'dir': '',                  # 预渲染目录
    'preset': 'spinpulse',      # PRESETS key (源相关, 见 SOURCE_PRESETS)
    'anim_take': '0',           # glb_anim: 动画 take 名或索引
    'frames': 8,
    'fps': 4,
    'loop': True,
    'reconnect': True,
    'render_slices': 120,       # GUI 渲染角度数 (整除 360, 越小越快)
    'last_render_dir': '',
}

PRESETS = {                     # key → (中文名, povstream anim)
    'static': ('静态', None),
    'spinpulse': ('呼吸动画 spinpulse', 'spinpulse'),
    'glb_anim': ('GLB自带动画', 'glbanim'),
    'globe_spin': ('地球仪自转', 'globe'),
    'glbseq': ('序列逐帧', 'glbseq'),
}
PRESET_LABELS = {k: v[0] for k, v in PRESETS.items()}
LABEL_TO_PRESET = {v[0]: k for k, v in PRESETS.items()}
SOURCE_PRESETS = {              # source → 可选 preset key (dir 直推, 无预设)
    'glb': ['static', 'spinpulse', 'glb_anim'],
    'globe': ['globe_spin'],
    'glbdir': ['glbseq'],
    'dir': [],
}

_IS_WIN = (os.name == 'nt')
_NOWIN = {'creationflags': 0x08000000} if _IS_WIN else {}   # CREATE_NO_WINDOW


# ================= 配置 =================

def load_config(path=CONFIG_PATH):
    cfg = dict(DEFAULT_CONFIG)
    try:
        with open(path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        if isinstance(data, dict):
            for k in cfg:
                if k in data:
                    cfg[k] = data[k]
    except (OSError, ValueError):
        pass
    return cfg


def save_config(cfg, path=CONFIG_PATH):
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(cfg, f, ensure_ascii=False, indent=1)


# ================= 设备探测 =================

def ping_ok(ip, timeout_s=1.0):
    """系统 ping 一发. Windows -n/-w(ms), Linux -c/-W(s)."""
    if _IS_WIN:
        cmd = ['ping', '-n', '1', '-w', str(int(timeout_s * 1000)), ip]
    else:
        cmd = ['ping', '-c', '1', '-W', str(max(int(timeout_s), 1)), ip]
    try:
        r = subprocess.run(cmd, capture_output=True, timeout=timeout_s + 3, **_NOWIN)
        return r.returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False


def port_open(ip, port=DEFAULT_PORT, timeout_s=1.0):
    try:
        with socket.create_connection((ip, port), timeout=timeout_s):
            return True
    except OSError:
        return False


def ssh_cmd(ip, key, *remote_cmd, timeout=4):
    dev_null = 'NUL' if _IS_WIN else '/dev/null'
    return ['ssh', '-i', key, '-o', 'BatchMode=yes',
            '-o', f'ConnectTimeout={timeout}', '-o', 'StrictHostKeyChecking=no',
            '-o', f'UserKnownHostsFile={dev_null}',
            f'{BOARD_USER}@{ip}'] + list(remote_cmd)


def ssh_run(ip, key, remote_cmd, timeout_s=15):
    """跑一条远端命令, 返回 (returncode, stdout+stderr 文本). 异常归一为 (-1, msg)."""
    try:
        r = subprocess.run(ssh_cmd(ip, key, remote_cmd), capture_output=True,
                           timeout=timeout_s, **_NOWIN)
        out = r.stdout.decode('utf-8', 'replace') + r.stderr.decode('utf-8', 'replace')
        return r.returncode, out
    except (OSError, subprocess.TimeoutExpired) as e:
        return -1, f'{e}'


def ssh_ok(ip, key):
    rc, _ = ssh_run(ip, key, 'true', timeout_s=10)
    return rc == 0


def check_status(ip, key, do_port=True):
    """三灯: (在线, 推流口, ssh). do_port=False (推流中) 时推流口返回 None."""
    online = ping_ok(ip)
    port = port_open(ip) if do_port else None
    ssh = ssh_ok(ip, key) if online else False
    return online, port, ssh


def arp_find_mac(mac=BOARD_MAC):
    """arp 表按 MAC 找 IP (Windows 用 '-' 分隔, Linux 用 ':'). 找不到 None."""
    try:
        r = subprocess.run(['arp', '-a'], capture_output=True, timeout=10, **_NOWIN)
    except (OSError, subprocess.TimeoutExpired):
        return None
    text = r.stdout.decode('utf-8', 'replace')
    want = mac.lower()
    for line in text.splitlines():
        norm = line.lower().replace('-', ':')
        if want in norm:
            for tok in line.replace('(', ' ').replace(')', ' ').split():
                try:
                    ipaddress.IPv4Address(tok)
                    return tok
                except ValueError:
                    continue
    return None


def scan_subnet(base_ip, port=DEFAULT_PORT, prefix=23, timeout_s=0.4,
                workers=64, progress=None, stop=None):
    """/prefix 网段并发 connect :port 扫板. 返回开口 IP 列表 (按扫描完成序)."""
    net = ipaddress.ip_network(f'{base_ip}/{prefix}', strict=False)
    hosts = [str(h) for h in net.hosts()]
    found = []
    done = [0]
    lock = threading.Lock()

    def probe(ip):
        ok = False if (stop and stop.is_set()) else port_open(ip, port, timeout_s)
        with lock:
            done[0] += 1
            if ok:
                found.append(ip)
            if progress:
                progress(done[0], len(hosts), ip if ok else None)
        return ok

    with concurrent.futures.ThreadPoolExecutor(max_workers=workers) as ex:
        list(ex.map(probe, hosts))
    return found


# ================= 渲染 =================

def glb_seq_count(glb_dir):
    """glbseq 目录里 *.glb 文件数 (= 帧数). 目录空/无效 → 0."""
    if not glb_dir or not os.path.isdir(glb_dir):
        return 0
    return len(glob.glob(os.path.join(glb_dir, '*.glb')))


def glb_take_names(glb_path):
    """GLB 动画 take 名列表 ([] = 无动画). 惰性 import glb_anim (pygltflib)."""
    import glb_anim
    return glb_anim.list_takes(glb_path)


def render_args(preset, frames, glb, render_slices, glb_dir='', anim_take='0'):
    """povstream add_render_opts 默认值 → Namespace, 按 GUI 选择覆盖."""
    ap = argparse.ArgumentParser()
    povstream.add_render_opts(ap)
    args = ap.parse_args([])
    args.frames = max(int(frames), 1)
    args.render_slices = int(render_slices)
    if glb:
        args.glb = glb
    if glb_dir:
        args.glb_dir = glb_dir
    args.anim_take = str(anim_take).strip() or '0'
    anim = PRESETS[preset][1]
    if preset == 'static':
        args.frames = 1
        anim = 'spinpulse'          # 静态源: 用点云但不做动作
    elif preset == 'glbseq':
        args.frames = max(glb_seq_count(glb_dir), 1)    # 帧数 = 文件数
    args.anim = anim
    return args


def _static_frames(args, source):
    """静态预设: 单帧体素格 (GLB 点云原姿态 / 地球仪 t=0)."""
    if source == 'globe':
        args.frames = 1
        yield from povstream.globe_frames(args)
    else:
        p, col = povstream.load_anime_points(args)
        yield gas.voxel_grid(p, col, verbose=False)


def frame_voxel_iter(preset, source, args):
    if preset == 'static':
        return _static_frames(args, source)
    if source == 'globe' or preset == 'globe_spin':
        return povstream.globe_frames(args)
    return povstream.ANIMS[args.anim](args)


def render_dir_name(preset, source, frames, glb, render_slices,
                    glb_dir='', anim_take='0'):
    if source == 'glb' and glb:
        tag = os.path.splitext(os.path.basename(glb))[0]
    elif source == 'glbdir' and glb_dir:
        tag = os.path.basename(os.path.normpath(glb_dir))
    else:
        tag = source
    key = f'{source}|{preset}|{frames}|{glb}|{render_slices}'
    if source == 'glbdir':
        key += f'|{glb_dir}'
    if preset == 'glb_anim':
        key += f'|take={anim_take}'
    h = hashlib.md5(key.encode('utf-8')).hexdigest()[:8]
    return os.path.join(HERE, f'frames_studio_{tag}_{preset}_{frames}f_{h}')


def render_job(preset, source, frames, glb, render_slices=120,
               out_dir=None, progress=None, stop=None, glb_dir='', anim_take='0'):
    """渲染 N 帧 packed bin 到 out_dir. progress(done, total) 逐帧回调.
    已存在完整输出 (meta 匹配) 直接复用. 返回 out_dir."""
    args = render_args(preset, frames, glb, render_slices, glb_dir, anim_take)
    out_dir = out_dir or render_dir_name(preset, source, args.frames, glb,
                                         render_slices, glb_dir, args.anim_take)
    meta_path = os.path.join(out_dir, 'meta.json')
    if os.path.exists(meta_path):
        try:
            with open(meta_path, 'r', encoding='utf-8') as f:
                meta = json.load(f)
            bins = sorted(glob.glob(os.path.join(out_dir, 'frame_*.bin')))
            if meta.get('frames') == args.frames and len(bins) == args.frames:
                if progress:
                    progress(args.frames, args.frames)
                return out_dir
        except (OSError, ValueError):
            pass
    os.makedirs(out_dir, exist_ok=True)
    total = args.frames
    if progress:
        progress(0, total)
    for i, vox in enumerate(frame_voxel_iter(preset, source, args)):
        if stop and stop.is_set():
            raise InterruptedError('render cancelled')
        raw = povstream.render_packed_frame(vox, i, args.render_slices, args.sub,
                                            args.thresh, not args.no_dither)
        assert len(raw) == FRAME_RAW
        with open(os.path.join(out_dir, f'frame_{i:04d}.bin'), 'wb') as f:
            f.write(raw)
        if progress:
            progress(i + 1, total)
    meta = {'preset': preset, 'source': source, 'glb': glb, 'frames': total,
            'glb_dir': glb_dir, 'anim_take': args.anim_take,
            'render_slices': args.render_slices, 'frame_raw': FRAME_RAW,
            'generated': time.strftime('%Y-%m-%d %H:%M:%S')}
    with open(meta_path, 'w', encoding='utf-8') as f:
        json.dump(meta, f, ensure_ascii=False, indent=1)
    return out_dir


def first_frame_bin(d):
    files = sorted(glob.glob(os.path.join(d, 'frame_*.bin')) or
                   glob.glob(os.path.join(d, '*.bin')))
    return files[0] if files else None


def preview_image(bin_path, slice_idx=0, scale=1):
    """packed bin → unpack slice → PIL RGB 图 (W*scale, H*scale)."""
    with open(bin_path, 'rb') as f:
        f.seek(slice_idx * pack_obs.SLICE_STRIDE)
        buf = f.read(pack_obs.SLICE_DATA)
    img = pack_obs.unpack_slice(buf).astype(np.uint8) * 255
    im = Image.fromarray(img)
    if scale != 1:
        im = im.resize((pack_obs.W * scale, pack_obs.H * scale), Image.NEAREST)
    return im


# ================= 开机默认动画上传 =================

def upload_default_anim(ip, key, bin_path, progress=None):
    """scp bin → 板 anime_slices.bin, ssh md5sum 校验. 返回 (ok, 消息)."""
    size = os.path.getsize(bin_path)
    if size != FRAME_RAW:
        return False, f'{bin_path} 大小 {size} != {FRAME_RAW} (须为单帧 360 切片 bin)'
    with open(bin_path, 'rb') as f:
        local_md5 = hashlib.md5(f.read()).hexdigest()
    if progress:
        progress('scp 上传中...')
    dev_null = 'NUL' if _IS_WIN else '/dev/null'
    cmd = ['scp', '-i', key, '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=6',
           '-o', 'StrictHostKeyChecking=no', '-o', f'UserKnownHostsFile={dev_null}',
           bin_path, f'{BOARD_USER}@{ip}:{BOARD_BIN}']
    try:
        r = subprocess.run(cmd, capture_output=True, timeout=180, **_NOWIN)
    except (OSError, subprocess.TimeoutExpired) as e:
        return False, f'scp 失败: {e}'
    if r.returncode != 0:
        return False, 'scp 失败: ' + r.stderr.decode('utf-8', 'replace').strip()
    if progress:
        progress('md5 校验中...')
    rc, out = ssh_run(ip, key, f'md5sum {BOARD_BIN}', timeout_s=30)
    if rc != 0:
        return False, f'ssh md5sum 失败: {out.strip()}'
    remote_md5 = out.split()[0] if out.split() else ''
    if remote_md5 != local_md5:
        return False, f'md5 不匹配: 本地 {local_md5} != 板上 {remote_md5}'
    return True, f'md5 校验通过 ({local_md5}), 下次开机生效'


def fetch_board_logs(ip, key, lines=30):
    """两份板日志尾部, 一条 ssh 拉回."""
    parts = ';'.join(f'echo "===== {p} ====="; tail -n {lines} {p} 2>&1; echo'
                     for p in BOARD_LOGS)
    rc, out = ssh_run(ip, key, parts, timeout_s=15)
    if rc != 0 and not out.strip():
        out = f'(ssh 失败 rc={rc})'
    return out


# ================= GUI =================

def main():
    import tkinter as tk
    from tkinter import ttk, filedialog, messagebox, scrolledtext
    from PIL import ImageTk

    PREVIEW_SCALE = 1               # 160x180 canvas ("128x144-ish")

    class App:
        def __init__(self, root):
            self.root = root
            self.cfg = load_config()
            self.ui_q = queue.Queue()
            self.stream_stop = None
            self.streamer = None
            self.render_stop = None
            self.rendering = False
            self.scanning = False
            self.status_stop = threading.Event()
            self._preview_photo = None
            self._log_win = None
            self._ip = self.cfg['ip']       # 后台线程用的 IP 快照 (主线程同步)
            self._build()
            self.ip_var.trace_add('write', lambda *a: setattr(
                self, '_ip', self.ip_var.get().strip()))
            self.root.protocol('WM_DELETE_WINDOW', self.on_close)
            self.root.after(100, self._drain_ui_q)
            threading.Thread(target=self._status_loop, daemon=True).start()

        # ---------- UI 构建 ----------
        def _build(self):
            self.root.title('POV Studio — POV 体显示控制台')
            self.root.minsize(560, 640)
            pad = {'padx': 6, 'pady': 3}

            # ===== 1. 设备 =====
            dev = ttk.LabelFrame(self.root, text=' 设备 ')
            dev.pack(fill='x', padx=8, pady=(8, 4))
            row1 = ttk.Frame(dev); row1.pack(fill='x', **pad)
            ttk.Label(row1, text='板 IP:').pack(side='left')
            self.ip_var = tk.StringVar(value=self.cfg['ip'])
            ttk.Entry(row1, textvariable=self.ip_var, width=16).pack(side='left', padx=4)
            self.scan_btn = ttk.Button(row1, text='扫描', command=self.on_scan)
            self.scan_btn.pack(side='left', padx=4)
            self.log_btn = ttk.Button(row1, text='板日志', command=self.on_board_log)
            self.log_btn.pack(side='left', padx=4)

            row2 = ttk.Frame(dev); row2.pack(fill='x', **pad)
            self.led_online = self._led(row2, '在线')
            self.led_port = self._led(row2, '推流口:9500')
            self.led_ssh = self._led(row2, 'ssh')
            self.dev_msg = tk.StringVar(value='')
            ttk.Label(row2, textvariable=self.dev_msg).pack(side='left', padx=10)

            # ===== 2. 内容 =====
            cont = ttk.LabelFrame(self.root, text=' 内容 ')
            cont.pack(fill='both', expand=True, padx=8, pady=4)
            left = ttk.Frame(cont); left.pack(side='left', fill='both', expand=True)
            right = ttk.Frame(cont); right.pack(side='right', padx=6, pady=4)

            src = ttk.Frame(left); src.pack(fill='x', **pad)
            self.source_var = tk.StringVar(value=self.cfg['source'])
            r = ttk.Frame(src); r.pack(fill='x')
            ttk.Radiobutton(r, text='GLB 文件', value='glb', variable=self.source_var,
                            command=self.on_source_change).pack(side='left')
            self.glb_var = tk.StringVar(value=self.cfg['glb'])
            ttk.Entry(r, textvariable=self.glb_var, width=34).pack(side='left', padx=4,
                                                                   fill='x', expand=True)
            ttk.Button(r, text='浏览...', width=7,
                       command=self.on_browse_glb).pack(side='left')
            r = ttk.Frame(src); r.pack(fill='x', pady=2)
            ttk.Radiobutton(r, text='GLB 序列目录', value='glbdir',
                            variable=self.source_var,
                            command=self.on_source_change).pack(side='left')
            self.glbdir_var = tk.StringVar(value=self.cfg['glb_dir'])
            ttk.Entry(r, textvariable=self.glbdir_var, width=30).pack(
                side='left', padx=4, fill='x', expand=True)
            ttk.Button(r, text='浏览...', width=7,
                       command=self.on_browse_glbdir).pack(side='left')
            r = ttk.Frame(src); r.pack(fill='x', pady=2)
            ttk.Radiobutton(r, text='内置地球仪', value='globe', variable=self.source_var,
                            command=self.on_source_change).pack(side='left')
            r = ttk.Frame(src); r.pack(fill='x')
            ttk.Radiobutton(r, text='预渲染目录', value='dir', variable=self.source_var,
                            command=self.on_source_change).pack(side='left')
            self.dir_var = tk.StringVar(value=self.cfg['dir'])
            ttk.Entry(r, textvariable=self.dir_var, width=32).pack(side='left', padx=4,
                                                                   fill='x', expand=True)
            ttk.Button(r, text='浏览...', width=7,
                       command=self.on_browse_dir).pack(side='left')

            opts = ttk.Frame(left); opts.pack(fill='x', **pad)
            ttk.Label(opts, text='预设:').pack(side='left')
            self.preset_var = tk.StringVar(
                value=PRESET_LABELS.get(self.cfg['preset'], PRESET_LABELS['spinpulse']))
            self.preset_cb = ttk.Combobox(opts, textvariable=self.preset_var,
                                          values=list(PRESET_LABELS.values()),
                                          state='readonly', width=20)
            self.preset_cb.pack(side='left', padx=4)
            self.preset_cb.bind('<<ComboboxSelected>>',
                                lambda e: self._update_content_widgets())
            ttk.Label(opts, text='动画take:').pack(side='left', padx=(10, 0))
            self.take_var = tk.StringVar(value=str(self.cfg['anim_take']))
            self.take_entry = ttk.Entry(opts, textvariable=self.take_var, width=6)
            self.take_entry.pack(side='left', padx=4)
            ttk.Label(opts, text='帧数:').pack(side='left', padx=(10, 0))
            self.frames_var = tk.IntVar(value=int(self.cfg['frames']))
            self.frames_sb = ttk.Spinbox(opts, from_=1, to=360,
                                         textvariable=self.frames_var, width=5)
            self.frames_sb.pack(side='left', padx=4)

            act = ttk.Frame(left); act.pack(fill='x', **pad)
            self.render_btn = ttk.Button(act, text='渲染', command=self.on_render)
            self.render_btn.pack(side='left')
            self.render_pb = ttk.Progressbar(act, length=200, mode='determinate')
            self.render_pb.pack(side='left', padx=8, fill='x', expand=True)
            self.render_msg = tk.StringVar(value='')
            ttk.Label(left, textvariable=self.render_msg).pack(anchor='w', padx=8)

            ttk.Label(right, text='预览 (帧0 切片0)').pack()
            self.canvas = tk.Canvas(right, width=pack_obs.W * PREVIEW_SCALE,
                                    height=pack_obs.H * PREVIEW_SCALE,
                                    bg='#101010', highlightthickness=1,
                                    highlightbackground='#444')
            self.canvas.pack()

            # ===== 3. 推流 + 默认 =====
            st = ttk.LabelFrame(self.root, text=' 推流 + 开机默认 ')
            st.pack(fill='x', padx=8, pady=(4, 8))
            r = ttk.Frame(st); r.pack(fill='x', **pad)
            ttk.Label(r, text='fps:').pack(side='left')
            self.fps_var = tk.IntVar(value=int(self.cfg['fps']))
            ttk.Spinbox(r, from_=1, to=30, textvariable=self.fps_var,
                        width=4).pack(side='left', padx=4)
            self.loop_var = tk.BooleanVar(value=bool(self.cfg['loop']))
            ttk.Checkbutton(r, text='循环', variable=self.loop_var).pack(side='left', padx=8)
            self.reconn_var = tk.BooleanVar(value=bool(self.cfg['reconnect']))
            ttk.Checkbutton(r, text='自动重连', variable=self.reconn_var).pack(side='left', padx=8)
            self.stream_btn = ttk.Button(r, text='开始推流', command=self.on_stream_toggle)
            self.stream_btn.pack(side='left', padx=10)
            self.default_btn = ttk.Button(r, text='设为开机默认动画',
                                          command=self.on_set_default)
            self.default_btn.pack(side='left', padx=6)
            self.stream_msg = tk.StringVar(value='未推流')
            ttk.Label(st, textvariable=self.stream_msg).pack(anchor='w', padx=8, pady=(0, 4))

            # 源相关预设 / take / 帧数 状态初始化 + 序列目录变化即刷帧数
            self._update_preset_choices()
            self._update_content_widgets()
            self.glbdir_var.trace_add(
                'write', lambda *a: self._update_content_widgets())

            # 初始预览: 上次渲染目录
            d = self.cfg.get('last_render_dir')
            if d and os.path.isdir(d):
                fb = first_frame_bin(d)
                if fb:
                    self._show_preview(fb)

        def _led(self, parent, label):
            f = ttk.Frame(parent); f.pack(side='left', padx=6)
            c = tk.Canvas(f, width=14, height=14, highlightthickness=0)
            c.create_oval(2, 2, 12, 12, fill='#777', outline='#333', tags='dot')
            c.pack(side='left')
            ttk.Label(f, text=label).pack(side='left', padx=2)
            return c

        def _set_led(self, led, state):
            color = {True: '#2ecc40', False: '#ff4136', None: '#ffb700'}[state]
            led.itemconfigure('dot', fill=color)

        # ---------- 线程 → UI ----------
        def ui(self, fn, *a):
            self.ui_q.put((fn, a))

        def _drain_ui_q(self):
            try:
                while True:
                    fn, a = self.ui_q.get_nowait()
                    try:
                        fn(*a)
                    except tk.TclError:
                        pass
            except queue.Empty:
                pass
            self.root.after(100, self._drain_ui_q)

        # ---------- 设备状态轮询 ----------
        def _status_loop(self):
            while not self.status_stop.is_set():
                ip = self._ip
                key = self.cfg['ssh_key']
                if ip:
                    streaming = self.streamer is not None
                    try:
                        online, port, ssh = check_status(ip, key, do_port=not streaming)
                    except Exception:
                        online = port = ssh = False
                    def upd(o=online, p=port, s=ssh, streaming=streaming):
                        self._set_led(self.led_online, o)
                        self._set_led(self.led_port, None if streaming else p)
                        self._set_led(self.led_ssh, s)
                    self.ui(upd)
                self.status_stop.wait(5)

        # ---------- 扫描 ----------
        def on_scan(self):
            if self.scanning:
                return
            base = self.ip_var.get().strip() or DEFAULT_CONFIG['ip']
            try:
                ipaddress.IPv4Address(base)
            except ValueError:
                messagebox.showerror('POV Studio', f'IP 无效: {base}')
                return
            self.scanning = True
            self.scan_btn.configure(state='disabled')
            self.dev_msg.set('扫描 /23 网段中...')
            threading.Thread(target=self._scan_thread, args=(base,), daemon=True).start()

        def _scan_thread(self, base):
            def progress(done, total, hit):
                if hit or done % 64 == 0 or done == total:
                    self.ui(self.dev_msg.set,
                            f'扫描 {done}/{total}' + (f'  发现 {hit}!' if hit else ''))
            try:
                found = scan_subnet(base, progress=progress)
                mac_ip = arp_find_mac()
            except Exception as e:
                found, mac_ip = [], None
                self.ui(self.dev_msg.set, f'扫描出错: {e}')
            best = None
            if mac_ip and (mac_ip in found or port_open(mac_ip)):
                best = mac_ip
            elif found:
                best = found[0]
            elif mac_ip:
                best = mac_ip
            def fin():
                self.scanning = False
                self.scan_btn.configure(state='normal')
                if best:
                    self.ip_var.set(best)
                    extra = ' (MAC 匹配)' if best == mac_ip else ''
                    others = [f for f in found if f != best]
                    self.dev_msg.set(f'找到板: {best}{extra}'
                                     + (f' 其他开口: {", ".join(others)}' if others else ''))
                else:
                    self.dev_msg.set('没找到板 (无 :9500 开口, arp 无 MAC)')
            self.ui(fin)

        # ---------- 板日志 ----------
        def on_board_log(self):
            if self._log_win is not None and self._log_win.winfo_exists():
                self._log_win.lift()
                return
            win = tk.Toplevel(self.root)
            win.title(f'板日志 — {self.ip_var.get().strip()}')
            win.geometry('720x480')
            txt = scrolledtext.ScrolledText(win, font=('Consolas', 9), state='disabled')
            txt.pack(fill='both', expand=True)
            self._log_win = win
            stop = threading.Event()
            win.protocol('WM_DELETE_WINDOW', lambda: (stop.set(), win.destroy()))

            def poll():
                while not stop.is_set() and not self.status_stop.is_set():
                    ip = self._ip
                    out = fetch_board_logs(ip, self.cfg['ssh_key'])
                    stamp = time.strftime('%H:%M:%S')
                    def show(o=out, s=stamp):
                        if not win.winfo_exists():
                            return
                        txt.configure(state='normal')
                        txt.delete('1.0', 'end')
                        txt.insert('end', f'[{s} 刷新, 3s 轮询]\n{o}')
                        txt.see('end')
                        txt.configure(state='disabled')
                    self.ui(show)
                    stop.wait(3)
            threading.Thread(target=poll, daemon=True).start()

        # ---------- 内容 ----------
        def _update_preset_choices(self):
            """预设下拉随源变化; 当前预设不适用时切到该源第一个预设."""
            keys = SOURCE_PRESETS.get(self.source_var.get(), [])
            labels = [PRESET_LABELS[k] for k in keys]
            self.preset_cb.configure(values=labels)
            if not keys:                        # 预渲染目录: 直推, 无预设
                self.preset_var.set('')
                self.preset_cb.configure(state='disabled')
                return
            self.preset_cb.configure(state='readonly')
            if LABEL_TO_PRESET.get(self.preset_var.get()) not in keys:
                self.preset_var.set(labels[0])

        def _update_content_widgets(self):
            """动画take 只在 GLB自带动画 预设可编; glbdir 源帧数=文件数只读."""
            preset = LABEL_TO_PRESET.get(self.preset_var.get())
            self.take_entry.configure(
                state='normal' if preset == 'glb_anim' else 'disabled')
            frames_locked = str(self.frames_sb.cget('state')) == 'disabled'
            if self.source_var.get() == 'glbdir':
                if not frames_locked:           # 进入 glbdir 前记住手动帧数
                    try:
                        self._frames_manual = max(int(self.frames_var.get()), 1)
                    except (ValueError, tk.TclError):
                        pass
                self.frames_var.set(glb_seq_count(self.glbdir_var.get().strip()))
                self.frames_sb.configure(state='disabled')
            else:
                if frames_locked:
                    self.frames_var.set(getattr(self, '_frames_manual',
                                                int(self.cfg['frames'])))
                self.frames_sb.configure(state='normal')

        def on_source_change(self):
            self._update_preset_choices()
            self._update_content_widgets()

        def on_browse_glb(self):
            p = filedialog.askopenfilename(title='选择 GLB 模型',
                                           filetypes=[('GLB 模型', '*.glb'), ('所有文件', '*.*')])
            if p:
                self.glb_var.set(p)
                self.source_var.set('glb')
                self.on_source_change()

        def on_browse_glbdir(self):
            p = filedialog.askdirectory(
                title='选择 GLB 帧序列目录 (sorted *.glb, 每文件一帧)',
                initialdir=HERE)
            if p:
                self.glbdir_var.set(p)
                self.source_var.set('glbdir')
                self.on_source_change()

        def on_browse_dir(self):
            p = filedialog.askdirectory(title='选择预渲染帧目录', initialdir=HERE)
            if p:
                self.dir_var.set(p)
                self.source_var.set('dir')
                self.on_source_change()
                fb = first_frame_bin(p)
                if fb:
                    self._show_preview(fb)

        def _gather(self):
            """UI → cfg dict (并持久化). glbdir 源帧数=文件数只是显示,
            持久化仍存手动帧数."""
            try:
                frames = max(int(self.frames_var.get() or 1), 1)
            except (ValueError, tk.TclError):
                frames = max(int(self.cfg['frames']), 1)
            if self.source_var.get() == 'glbdir':
                frames = getattr(self, '_frames_manual',
                                 max(int(self.cfg['frames']), 1))
            self.cfg.update({
                'ip': self.ip_var.get().strip(),
                'source': self.source_var.get(),
                'glb': self.glb_var.get().strip(),
                'glb_dir': self.glbdir_var.get().strip(),
                'dir': self.dir_var.get().strip(),
                'preset': LABEL_TO_PRESET.get(self.preset_var.get(), 'spinpulse'),
                'anim_take': self.take_var.get().strip() or '0',
                'frames': frames,
                'fps': max(int(self.fps_var.get() or 1), 1),
                'loop': bool(self.loop_var.get()),
                'reconnect': bool(self.reconn_var.get()),
            })
            try:
                save_config(self.cfg)
            except OSError:
                pass
            return self.cfg

        def on_render(self):
            if self.rendering:
                if self.render_stop:
                    self.render_stop.set()
                return
            cfg = self._gather()
            if cfg['source'] == 'dir':
                d = cfg['dir']
                if not d or not first_frame_bin(d):
                    messagebox.showerror('POV Studio', '预渲染目录里没有 .bin 帧')
                    return
                self.cfg['last_render_dir'] = d
                save_config(self.cfg)
                self._show_preview(first_frame_bin(d))
                self.render_msg.set(f'预渲染目录无需渲染: {d}')
                return
            if cfg['source'] == 'glb' and cfg['glb'] and not os.path.exists(cfg['glb']):
                messagebox.showerror('POV Studio', f'GLB 不存在: {cfg["glb"]}')
                return
            if cfg['source'] == 'glbdir' and glb_seq_count(cfg['glb_dir']) == 0:
                messagebox.showerror('POV Studio',
                                     'GLB 序列目录里没有 *.glb 文件: '
                                     f'{cfg["glb_dir"] or "(未选)"}')
                return
            if cfg['source'] == 'glb' and cfg['preset'] == 'glb_anim':
                path = cfg['glb'] or gas.DEFAULT_GLB
                try:
                    takes = glb_take_names(path)
                except Exception as e:
                    messagebox.showerror('POV Studio', f'读 GLB 动画失败: {e}')
                    return
                if not takes:
                    messagebox.showinfo(
                        'POV Studio',
                        f'{os.path.basename(path)} 没有自带动画 take,\n'
                        '将按 "静态" 单帧渲染。')
                    cfg['preset'] = 'static'
                    self.preset_var.set(PRESET_LABELS['static'])
                    self._update_content_widgets()
            self.rendering = True
            self.render_stop = threading.Event()
            self.render_btn.configure(text='取消渲染')
            threading.Thread(target=self._render_thread, args=(cfg,), daemon=True).start()

        def _render_thread(self, cfg):
            t0 = time.time()

            def progress(done, total):
                def upd():
                    self.render_pb.configure(maximum=total, value=done)
                    self.render_msg.set(f'渲染中 {done}/{total} 帧 ({time.time() - t0:.0f}s)')
                self.ui(upd)
            try:
                out_dir = render_job(cfg['preset'], cfg['source'], cfg['frames'],
                                     cfg['glb'], cfg['render_slices'],
                                     progress=progress, stop=self.render_stop,
                                     glb_dir=cfg['glb_dir'],
                                     anim_take=cfg['anim_take'])
                self.cfg['last_render_dir'] = out_dir
                save_config(self.cfg)
                fb = first_frame_bin(out_dir)
                def fin():
                    self.render_msg.set(f'渲染完成 → {os.path.basename(out_dir)} '
                                        f'({time.time() - t0:.0f}s)')
                    if fb:
                        self._show_preview(fb)
                self.ui(fin)
            except InterruptedError:
                self.ui(self.render_msg.set, '渲染已取消')
            except Exception as e:
                self.ui(self.render_msg.set, f'渲染失败: {e}')
            finally:
                def reset():
                    self.rendering = False
                    self.render_btn.configure(text='渲染')
                self.ui(reset)

        def _show_preview(self, bin_path):
            try:
                im = preview_image(bin_path, 0, PREVIEW_SCALE)
            except Exception:
                return
            self._preview_photo = ImageTk.PhotoImage(im)
            self.canvas.delete('all')
            self.canvas.create_image(0, 0, anchor='nw', image=self._preview_photo)

        # ---------- 推流 ----------
        def _stream_dir(self, cfg):
            if cfg['source'] == 'dir' and cfg['dir']:
                return cfg['dir']
            return self.cfg.get('last_render_dir') or ''

        def on_stream_toggle(self):
            if self.streamer is not None:
                self.stream_stop.set()
                self.stream_btn.configure(state='disabled')
                return
            cfg = self._gather()
            d = self._stream_dir(cfg)
            if not d or not first_frame_bin(d):
                messagebox.showerror('POV Studio', '没有可推流的帧: 先渲染或选预渲染目录')
                return
            if not cfg['ip']:
                messagebox.showerror('POV Studio', '请填板 IP')
                return
            self.stream_stop = threading.Event()
            self.streamer = Streamer(
                cfg['ip'], DEFAULT_PORT, fps=cfg['fps'], loop=cfg['loop'],
                reconnect=cfg['reconnect'], retry_interval=5.0, ack_timeout=15.0,
                on_frame=lambda st: self.ui(self._stream_stats, st),
                on_status=lambda ev, det: self.ui(self._stream_status, ev, det),
                stop=self.stream_stop)
            self.stream_btn.configure(text='停止推流')
            self.stream_msg.set(f'连接 {cfg["ip"]}:{DEFAULT_PORT}...')
            threading.Thread(target=self._stream_thread, args=(d,), daemon=True).start()

        def _stream_thread(self, d):
            s = self.streamer
            err = None
            try:
                s.run(lambda: povstream.frame_iter_from_dir(d))
            except StreamerError as e:
                err = f'{e}'
            except SystemExit as e:         # frame_iter_from_dir 坏帧 sys.exit
                err = f'{e}'
            except Exception as e:
                err = f'{e}'
            def fin():
                self.streamer = None
                self.stream_btn.configure(text='开始推流', state='normal')
                base = (f'已停止: {s.frames} 帧, {s.wire_mbps():.2f} MB/s, '
                        f'压缩 {s.ratio():.1f}x, 重连 {s.reconnects} 次')
                self.stream_msg.set(base + (f' | 出错: {err}' if err else ''))
            self.ui(fin)

        def _stream_stats(self, st):
            self.stream_msg.set(f'推流中: {st.frames} 帧 | {st.wire_mbps():.2f} MB/s | '
                                f'压缩 {st.ratio():.1f}x | 实际 {st.frames / st.elapsed():.1f} fps'
                                + (f' | 重连 {st.reconnects} 次' if st.reconnects else ''))

        def _stream_status(self, ev, detail):
            if ev == 'connected':
                self.stream_msg.set(f'已连接 {detail}, 推流中...')
            elif ev in ('lost', 'retry'):
                self.stream_msg.set(f'连接断开 (板重启?), 5s 后重连... [{detail}]')

        # ---------- 设为开机默认动画 ----------
        def on_set_default(self):
            cfg = self._gather()
            if not cfg['ip']:
                messagebox.showerror('POV Studio', '请填板 IP')
                return
            d = self._stream_dir(cfg)
            fb = first_frame_bin(d) if d else None
            src_desc = None
            if fb and os.path.getsize(fb) == FRAME_RAW:
                src_desc = f'现有帧 {os.path.relpath(fb, HERE)}'
            if not messagebox.askyesno(
                    'POV Studio',
                    f'将覆盖板上 {BOARD_BIN}\n(开机默认动画, {FRAME_RAW} 字节)\n\n'
                    f'来源: {src_desc or "将先渲染静态单帧"}\n继续?'):
                return
            self.default_btn.configure(state='disabled')
            threading.Thread(target=self._set_default_thread,
                             args=(cfg, fb), daemon=True).start()

        def _set_default_thread(self, cfg, fb):
            try:
                if fb is None or os.path.getsize(fb) != FRAME_RAW:
                    self.ui(self.stream_msg.set, '渲染静态单帧中...')
                    out_dir = render_job('static', cfg['source'], 1, cfg['glb'],
                                         cfg['render_slices'],
                                         progress=lambda d, t: self.ui(
                                             self.stream_msg.set, f'渲染静态帧 {d}/{t}'))
                    fb = first_frame_bin(out_dir)
                ok, msg = upload_default_anim(cfg['ip'], self.cfg['ssh_key'], fb,
                                              progress=lambda m: self.ui(
                                                  self.stream_msg.set, m))
                def fin():
                    self.stream_msg.set(('✓ 开机默认已更新: ' if ok else '✗ 失败: ') + msg)
                    if not ok:
                        messagebox.showerror('POV Studio', msg)
                self.ui(fin)
            except Exception as e:
                self.ui(self.stream_msg.set, f'✗ 失败: {e}')
            finally:
                self.ui(lambda: self.default_btn.configure(state='normal'))

        # ---------- 退出 ----------
        def on_close(self):
            try:
                self._gather()
            except Exception:
                pass
            self.status_stop.set()
            if self.stream_stop:
                self.stream_stop.set()
            if self.render_stop:
                self.render_stop.set()
            self.root.destroy()

    root = tk.Tk()
    try:
        from tkinter import font as tkfont
        for name in ('TkDefaultFont', 'TkTextFont', 'TkMenuFont', 'TkHeadingFont'):
            tkfont.nametofont(name).configure(family='Microsoft YaHei UI'
                                              if _IS_WIN else 'sans-serif', size=9)
    except Exception:
        pass
    App(root)
    root.mainloop()


if __name__ == '__main__':
    main()
