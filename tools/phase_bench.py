#!/usr/bin/env python3
"""phase_bench.py — 端到端帧率的**可重复**测量台 (2026-08-05)

为什么要重写一个 (flip_bench.sh 已经有了):
  flip_bench.sh 每个配置**只跑一次**并对 DIAG 行求**算术平均**。而实测环境里
  WiFi 每 20-40 s 会有 1-3 次 300-840 ms 的停顿 (环境性, 不是我们的代码),
  一次 20 秒的窗口里撞上 0 次还是 2 次, 平均值能差 7.49 vs 10.14 帧/秒 ——
  **同一个配置**。用这种数字判断优化是否有效等于抛硬币。
  ⇒ 本脚本: 同配置跑 N 次, 汇报**分布** (中位数 / p5 / p95 / σ), 而不是单次均值。
     中位数对偶发停顿免疫; p5 才是"最差情况有多差"; σ 用来判断两组是否真的分开。

🔴 读板端日志的三条判据 (踩过三次, 别再踩):
  1. 空闲动画 (--idle-anim) 和推流**共用同一个 pov_rxd 进程和同一份日志**,
     两者都会让 rx/flip 增长。**光看 rx 涨不代表推流在跑**。
  2. 空闲动画路径不更新 dec 累加器 ⇒ DIAG 行里 `dec 0.0/0.0ms` 的一律是空闲动画,
     必须滤掉。本脚本 _parse() 里就是这么干的。
  3. `drop=` 恒定不变也是空闲动画 (空闲路径不经过 ready/consumed 竞争)。
  另: 板子跑 UTC, 开发机跑 CST, 墙钟看着差 8 小时其实是**同一时刻** ——
     别按墙钟去对窗口, 本脚本一律按"每次 run 重启 unit 后的整段 journal"取。

用法:
  tools/phase_bench.py --runs 5 --secs 20 --label baseline
  tools/phase_bench.py --runs 5 --rxd-args "--phase-lock on" --label locked
  tools/phase_bench.py --compare baseline.json locked.json      # 只比对, 不测

环境变量: BOARD (默认 tools/board_ip.sh), PLINK, HK, RXD (板端二进制路径)
"""
import argparse
import json
import math
import os
import re
import statistics
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PLINK = os.environ.get('PLINK', '/mnt/c/Program Files/PuTTY/plink.exe')
HK = os.environ.get('HK', 'SHA256:u14U8c0RuKnVinQuaGH5ey6OKScaPOlRF3vMNqSnEGI')
UNIT = os.environ.get('UNIT', 'povbench')
RXD = os.environ.get('RXD', '/home/uisrc/pov_rxd')
FAKESPIN = '/home/uisrc/diag_fakespin.py'
POVSTREAM = os.path.join(ROOT, 'stream', 'pc', 'povstream.py')
DEFAULT_DIR = '/mnt/d/claude_workspace/hls_scratch/frames_fold'


def board_ip():
    ip = os.environ.get('BOARD')
    if ip:
        return ip
    return subprocess.check_output([os.path.join(HERE, 'board_ip.sh')],
                                   text=True).strip()


BOARD = None


def sh(cmd, timeout=90):
    """板端跑一条命令 (普通用户)。"""
    p = subprocess.run([PLINK, '-ssh', '-batch', '-hostkey', HK, '-pw', 'root',
                        f'uisrc@{BOARD}', cmd],
                       capture_output=True, text=True, timeout=timeout,
                       errors='replace')
    return p.stdout


def sudo(cmd, timeout=90):
    """板端 root。sudo 密码 root, 从 stdin 喂 (板上没配 NOPASSWD)。"""
    return sh("echo root | sudo -S sh -c " + shq(cmd) + " 2>/dev/null", timeout)


def shq(s):
    return "'" + s.replace("'", "'\\''") + "'"


# ---- DIAG 解析 -------------------------------------------------------------
# [09:04:02.890] DIAG 2.1s eng=0.0rev/s rx=9.40/s flip=0.94/s drop=45227 |
#   dec 0.0/0.0ms (core0 0.0 core1 0.0) | cpy 37.7/40.1ms | wait 1000.7/2001.3ms
DIAG_RE = re.compile(
    r'DIAG\s+(?P<dt>[\d.]+)s\s+eng=(?P<eng>[\d.]+)rev/s\s+rx=(?P<rx>[\d.]+)/s\s+'
    r'flip=(?P<flip>[\d.]+)/s\s+drop=(?P<drop>\d+)\s*\|\s*'
    r'dec\s+(?P<dec>[\d.]+)/(?P<decmax>[\d.]+)ms\s+'
    r'\(core0\s+(?P<c0>[\d.]+)\s+core1\s+(?P<c1>[\d.]+)\)\s*\|\s*'
    r'cpy\s+(?P<cpy>[\d.]+)/(?P<cpymax>[\d.]+)ms\s*\|\s*'
    r'wait\s+(?P<wait>[\d.]+)/(?P<waitmax>[\d.]+)ms'
    r'(?:\s*\|\s*hdr\s+(?P<hdr>[\d.]+)/(?P<hdrmax>[\d.]+)ms'
    r'\s*\|\s*body\s+(?P<body>[\d.]+)/(?P<bodymax>[\d.]+)ms)?')
# 相位仪表 (本分支新加, 老二进制没有这行 -> 解析结果为空是正常的)
PHASE_RE = re.compile(
    r'PHASE\s+arr=(?P<arr>[\d,]+)\s+gap\s+(?P<gapavg>[\d.]+)/(?P<gapmax>[\d.]+)ms\s+'
    r'rev1=(?P<rev1>\d+)\s+rev2\+=(?P<rev2>\d+)(?:\s+lock=(?P<lock>\w+))?')

FLOATS = ('dt', 'eng', 'rx', 'flip', 'dec', 'decmax', 'c0', 'c1',
          'cpy', 'cpymax', 'wait', 'waitmax')
# hdr/body 是 2026-08-05 才加的字段, 老二进制的 DIAG 行没有 -> 缺了记 0
FLOATS_OPT = ('hdr', 'hdrmax', 'body', 'bodymax')

# 见 parse_diag: --diag-rxonly 消融模式要放行 dec==0 的样本
ALLOW_DEC0 = False


def parse_diag(text, warm=2, tail=1):
    """journal 文本 -> 每秒一条的样本 list。滤掉空闲动画行, 掐头去尾。"""
    out = []
    for line in text.splitlines():
        m = DIAG_RE.search(line)
        if not m:
            continue
        d = m.groupdict()
        s = {k: float(d[k]) for k in FLOATS}
        s.update({k: float(d[k]) if d.get(k) else 0.0 for k in FLOATS_OPT})
        s['drop'] = int(d['drop'])
        # 判据 2: dec 累加器为 0 = 空闲动画, 不是推流。
        # ⚠ 例外: --diag-rxonly 消融模式下 dec 本来就恒为 0, 那时必须放行,
        #   否则整组样本被这条判据吃光 (调用方置 ALLOW_DEC0=True)。
        if s['dec'] <= 0.0 and not ALLOW_DEC0:
            continue
        # DIAG 窗口本该 ~1s; 明显异常的窗口 (进程刚起/刚停) 丢掉
        if not (0.7 <= s['dt'] <= 2.0):
            continue
        out.append(s)
    if len(out) > warm + tail:
        out = out[warm:len(out) - tail]
    return out


def parse_phase(text, warm=2, tail=1):
    out = []
    for line in text.splitlines():
        m = PHASE_RE.search(line)
        if not m:
            continue
        d = m.groupdict()
        arr = [int(x) for x in d['arr'].split(',') if x != '']
        rev1, rev2 = int(d['rev1']), int(d['rev2'])
        if rev1 + rev2 == 0:
            continue                       # 没翻页 = 没推流
        out.append(dict(arr=arr, gapavg=float(d['gapavg']),
                        gapmax=float(d['gapmax']), rev1=rev1, rev2=rev2,
                        lock=d.get('lock') or '?'))
    if len(out) > warm + tail:
        out = out[warm:len(out) - tail]
    return out


# ---- 统计 ------------------------------------------------------------------
def pct(xs, q):
    """线性插值分位数 (numpy 不一定在, 手写)。"""
    if not xs:
        return float('nan')
    s = sorted(xs)
    if len(s) == 1:
        return s[0]
    i = q / 100.0 * (len(s) - 1)
    lo = int(math.floor(i))
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (i - lo)


def dist(xs):
    if not xs:
        return dict(n=0)
    return dict(n=len(xs), median=statistics.median(xs), mean=statistics.fmean(xs),
                p5=pct(xs, 5), p95=pct(xs, 95),
                sd=statistics.pstdev(xs) if len(xs) > 1 else 0.0,
                min=min(xs), max=max(xs))


def fmt(d, unit=''):
    if not d.get('n'):
        return '(无样本)'
    return (f"中位 {d['median']:.2f}{unit}  p5 {d['p5']:.2f}  p95 {d['p95']:.2f}  "
            f"σ {d['sd']:.2f}  (均值 {d['mean']:.2f}, n={d['n']})")


# ---- 一次测量 --------------------------------------------------------------
def setup_board(rps, rxd_args):
    """重启板端 unit + 设 fake-spin。返回 (实测 rev/s, InvocationID)。

    🔴 InvocationID 不是可有可无的: systemd 的 journal 是**按 unit 累积**的,
    `journalctl -u povbench -n 2000` 会把**上几次 run 的 DIAG 行一起拉回来**
    (FLIP/FRAME 行很密, 2000 行只覆盖最近几十秒, 正好横跨两三次 run)。
    第一版就是这么写的, 结果 5x20s 的 run 汇报出 443 个"每秒样本" —— 对照组
    和实验组的行会互相污染, 差异会被稀释到看不见。改成按本次启动的
    InvocationID 过滤, 一次 run 只拿一次 run 的行。"""
    sudo(f"systemctl stop {UNIT} 2>/dev/null; systemctl reset-failed {UNIT} 2>/dev/null; "
         f"systemctl stop povrxd; sleep 1; "
         f"systemd-run --unit={UNIT} {RXD} {rxd_args}")
    time.sleep(2)
    inv = sudo(f"systemctl show -p InvocationID --value {UNIT}").strip().splitlines()
    inv = inv[-1].strip() if inv else ''
    spin = sudo(f"python3 {FAKESPIN} {rps}")
    m = re.search(r'实测 slice 推进: ([\d.]+) rev/s', spin)
    return (float(m.group(1)) if m else float('nan')), inv


def run_once(args, run_idx):
    eng, inv = setup_board(args.rps, args.rxd_args)
    cmd = [sys.executable, POVSTREAM, 'stream', '--dir', args.dir,
           '--host', BOARD, '--codec', 'lz4', '--lz4-level', str(args.lz4_level),
           '--stream-split', 'balanced', '--loop', '--fps', str(args.fps)]
    cmd += args.stream_args.split()
    t0 = time.time()
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                         text=True, errors='replace')
    try:
        p.wait(timeout=args.secs)
    except subprocess.TimeoutExpired:
        p.terminate()
        try:
            p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            p.kill()
    sender_out = p.stdout.read() if p.stdout else ''
    wall = time.time() - t0
    time.sleep(1.5)
    sel = (f"_SYSTEMD_INVOCATION_ID={inv}" if re.fullmatch(r'[0-9a-f]{32}', inv)
           else f"-u {UNIT}")
    jr = sudo(f"journalctl {sel} --no-pager -o cat -n 20000", timeout=120)
    samples = parse_diag(jr, args.warm, args.tail)
    drop_delta = (samples[-1]['drop'] - samples[0]['drop']) if samples else 0
    phase = parse_phase(jr, args.warm, args.tail)
    # FRAME 行的 2str/3str 标签: 确认流数配置真的生效了
    tags = sorted(set(re.findall(r'\b(\dstr/\w+)', jr)))
    sender_fps = None
    m = re.findall(r'页率 ([\d.]+)/s', sender_out)
    if m:
        sender_fps = statistics.median(float(x) for x in m)
    print(f"  run {run_idx}: eng={eng:.2f}rev/s  {len(samples)} 个有效秒窗  "
          f"tags={','.join(tags) or '-'}  发送端页率中位={sender_fps if sender_fps is None else round(sender_fps,2)}"
          f"  wall={wall:.0f}s")
    if not samples:
        print("    ⚠ 0 个有效秒窗 —— 检查推流是否真的连上 (只有空闲动画的 DIAG 会被滤掉)")
        print("    sender tail:", sender_out.strip().splitlines()[-3:] if sender_out.strip() else '(空)')
    return dict(eng=eng, samples=samples, phase=phase, tags=tags,
                sender_fps=sender_fps, wall=wall, drop=drop_delta,
                rx_total=sum(s['rx'] for s in samples))


def bench(args):
    runs = [run_once(args, i + 1) for i in range(args.runs)]
    pool = [s for r in runs for s in r['samples']]
    res = dict(label=args.label, rps=args.rps, runs=args.runs, secs=args.secs,
               rxd_args=args.rxd_args, stream_args=args.stream_args,
               fps=args.fps, dir=args.dir,
               run_medians=dict(
                   flip=[statistics.median([s['flip'] for s in r['samples']])
                         for r in runs if r['samples']],
                   rx=[statistics.median([s['rx'] for s in r['samples']])
                       for r in runs if r['samples']]),
               eng=[r['eng'] for r in runs],
               tags=sorted({t for r in runs for t in r['tags']}),
               pooled={k: dist([s[k] for s in pool])
                       for k in ('flip', 'rx', 'eng', 'dec', 'cpy', 'wait',
                                 'decmax', 'cpymax', 'waitmax',
                                 'hdr', 'body', 'hdrmax', 'bodymax')},
               n_pool=len(pool),
               drop=[r['drop'] for r in runs],
               rx_total=[round(r['rx_total']) for r in runs])
    # 每秒 flip 数 / 每秒转数 = 每圈翻中几次 (1.0 = 每圈都翻中)
    ratios = [s['flip'] / s['eng'] for s in pool if s['eng'] > 1.0]
    res['pooled']['flip_per_rev'] = dist(ratios)
    ph = [p for r in runs for p in r['phase']]
    if ph:
        nb = len(ph[0]['arr'])
        res['phase'] = dict(
            arr=[sum(p['arr'][i] for p in ph) for i in range(nb)],
            rev1=sum(p['rev1'] for p in ph), rev2=sum(p['rev2'] for p in ph),
            gapavg=dist([p['gapavg'] for p in ph]),
            lock=sorted({p['lock'] for p in ph}))
    res['samples'] = pool
    return res


def report(res):
    p = res['pooled']
    print(f"\n===== {res['label']}  (rps={res['rps']} runs={res['runs']}x{res['secs']}s "
          f"rxd[{res['rxd_args']}] fps={res['fps']}) =====")
    print(f"  流标签: {','.join(res['tags']) or '(无 FRAME 行)'}   "
          f"引擎实测: {['%.2f' % e for e in res['eng']]}")
    print(f"  flip/s  {fmt(p['flip'])}          <-- 真正上屏的帧率")
    print(f"  rx/s    {fmt(p['rx'])}            <-- 解码吞吐 (不等于上屏)")
    print(f"  每圈翻中 {fmt(p['flip_per_rev'])}  <-- 1.00 = 每圈一帧")
    print(f"  dec     {fmt(p['dec'], 'ms')}  峰值中位 {p['decmax']['median']:.1f}ms")
    print(f"  cpy     {fmt(p['cpy'], 'ms')}  峰值中位 {p['cpymax']['median']:.1f}ms")
    print(f"  wait    {fmt(p['wait'], 'ms')}  峰值中位 {p['waitmax']['median']:.1f}ms")
    if 'hdr' not in p:
        return
    print(f"  hdr     {fmt(p['hdr'], 'ms')}   <-- 等发送端 (空 = 发送端没跟上)")
    print(f"  body    {fmt(p['body'], 'ms')}  峰值中位 {p['bodymax']['median']:.1f}ms"
          f"   <-- 真·链路投递时间")
    svc = p['hdr']['median'] + p['body']['median'] + p['dec']['median']
    print(f"  => RX 线程一帧服务时间 hdr+body+dec = {svc:.1f}ms; 再 +cpy "
          f"{p['cpy']['median']:.1f} = {svc + p['cpy']['median']:.1f}ms "
          f"vs 一圈 {1000.0 / max(p['eng']['median'], 1e-9):.1f}ms"
          f"  ({'装得下, 相位锁定有意义' if svc + p['cpy']['median'] < 1000.0 / max(p['eng']['median'], 1e-9) else '装不下 -> 锁相无解, 得先砍这一段'})")
    print(f"  每次 run 的 flip 中位数: {[round(x, 2) for x in res['run_medians']['flip']]}")
    if 'drop' in res:
        print(f"  每次 run 丢帧 (ready 被顶替) {res['drop']} / 收到 {res['rx_total']} "
              f"<-- 丢帧≈0 说明**没有一帧因为错过翻页窗而白收**, 相位不是损失点")
    if 'phase' in res:
        ph = res['phase']
        tot = ph['rev1'] + ph['rev2']
        print(f"  相位: 到货 slice 直方图 {ph['arr']}  "
              f"1 圈翻中 {ph['rev1']}/{tot} = {100.0*ph['rev1']/max(tot,1):.0f}%"
              f"   lock 状态 {','.join(ph.get('lock', []))}")


def compare(a, b):
    print(f"\n===== 对比: {a['label']} -> {b['label']} =====")
    for k, unit in (('flip', '/s'), ('rx', '/s'), ('flip_per_rev', ''),
                    ('wait', 'ms'), ('cpy', 'ms'), ('dec', 'ms'),
                    ('hdr', 'ms'), ('body', 'ms')):
        da, db = a['pooled'].get(k, {}), b['pooled'].get(k, {})
        if not da.get('n') or not db.get('n'):
            continue
        d = db['median'] - da['median']
        # 效应量: 中位数差 / 合并 σ。|d|<0.5 基本等于噪声, >1 才算真的分开
        sp = math.sqrt((da['sd'] ** 2 + db['sd'] ** 2) / 2) or 1e-9
        print(f"  {k:14s} {da['median']:7.2f} -> {db['median']:7.2f}{unit}  "
              f"({d:+.2f}, σ {da['sd']:.2f}/{db['sd']:.2f}, 效应量 {d/sp:+.2f})"
              f"   p5 {da['p5']:.2f}->{db['p5']:.2f}")


def restore():
    sudo(f"systemctl stop {UNIT} 2>/dev/null; systemctl reset-failed {UNIT} 2>/dev/null; "
         f"systemctl start povrxd")


def main():
    global BOARD
    ap = argparse.ArgumentParser()
    ap.add_argument('--runs', type=int, default=5)
    ap.add_argument('--secs', type=int, default=20)
    ap.add_argument('--rps', type=float, default=15.0)
    ap.add_argument('--fps', type=float, default=40.0)
    ap.add_argument('--dir', default=DEFAULT_DIR)
    ap.add_argument('--lz4-level', type=int, default=9)
    ap.add_argument('--rxd-args', default='')
    ap.add_argument('--stream-args', default='')
    ap.add_argument('--label', default='run')
    ap.add_argument('--warm', type=int, default=2, help='每次 run 掐掉的头部秒窗数')
    ap.add_argument('--tail', type=int, default=1)
    ap.add_argument('--out', default=None)
    ap.add_argument('--compare', nargs=2, default=None,
                    help='只比对两个已有 json, 不碰板子')
    ap.add_argument('--no-restore', action='store_true')
    args = ap.parse_args()

    if args.compare:
        a = json.load(open(args.compare[0]))
        b = json.load(open(args.compare[1]))
        report(a); report(b); compare(a, b)
        return

    BOARD = board_ip()
    print(f"board={BOARD} unit={UNIT} rxd={RXD}")
    try:
        res = bench(args)
    finally:
        if not args.no_restore:
            restore()
    report(res)
    if args.out:
        json.dump(res, open(args.out, 'w'), ensure_ascii=False, indent=1)
        print(f"  -> {args.out}")


if __name__ == '__main__':
    main()
