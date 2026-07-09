#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
glb_anim.py — glTF/GLB 动画求值 + 变形网格点云采样 (povstream 'glbanim' 源).

支持:
  节点 TRS 动画: translation/rotation/scale channel, LINEAR/STEP 插值,
    rotation 走 SLERP; CUBICSPLINE 取关键帧中间值退化成 LINEAR;
  skeletal skinning: JOINTS_0/WEIGHTS_0 × (jointWorld @ inverseBindMatrix),
    glTF 2.0 规范: skinned mesh 忽略 mesh node 自身 world transform;
  morph targets: POSITION deltas × (animated) weights, skinning 前应用;
  颜色: baseColorTexture UV 采样 / COLOR_0 顶点色 / baseColorFactor, 加
    lambert / half-lambert 光照 (语义同 zynq_pov/host/glb_to_points.
    sample_triangles, 这里向量化重实现).

采样策略 (sticky sampling): 在参考姿态 (t=0) 上做一次 area-weighted 重心
采样, 采样点 "粘" 在各自三角形的重心坐标上逐帧跟随顶点变形 —— 时间连贯
(无逐帧重采样噪声), 且每帧只需向量化顶点变形 + 光照重算, 快得多.
颜色 albedo (纹理/顶点色) 只采一次, 光照强度按变形后法线逐帧重算.

注意: 不复用 glb_to_points._node_matrix (其 TRS 组合顺序是 S@R@T, 与
glTF 规范 M = T·R·S 相反; 单一变换的节点无所谓, 动画节点必须用正确顺序).
"""
import os
import sys
import math
import numpy as np
from pygltflib import GLTF2

HERE = os.path.dirname(os.path.abspath(__file__))
ZYNQ_POV_HOST = os.path.abspath(os.path.join(HERE, '..', '..', '..', 'zynq_pov', 'host'))
if ZYNQ_POV_HOST not in sys.path:
    sys.path.insert(0, ZYNQ_POV_HOST)
from glb_to_points import _get_accessor_data, _mat_texture  # noqa: E402


# ================= 基础数学 =================

def _read_accessor(g, idx):
    """accessor → float 数组, 处理 normalized int (quantized 动画输出等)."""
    arr = _get_accessor_data(g, idx)
    acc = g.accessors[idx]
    if acc.normalized and arr.dtype.kind in 'iu':
        arr = arr.astype(np.float32) / float(np.iinfo(arr.dtype).max)
        if acc.componentType in (5120, 5122):        # signed: max(x/imax, -1)
            arr = np.maximum(arr, -1.0)
    return arr


def _quat_mat3(q):
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ], np.float32)


def _trs(t, q, s):
    """glTF 规范局部变换 M = T · R · S."""
    m = np.eye(4, dtype=np.float32)
    m[:3, :3] = _quat_mat3(q) * np.asarray(s, np.float32)[None, :]
    m[:3, 3] = np.asarray(t, np.float32)
    return m


def _slerp(q0, q1, u):
    d = float(np.dot(q0, q1))
    if d < 0.0:
        q1 = -q1
        d = -d
    if d > 0.9995:                                   # 夹角太小, lerp 足够
        q = q0 + u * (q1 - q0)
    else:
        th = math.acos(min(d, 1.0))
        q = (math.sin((1 - u) * th) * q0 + math.sin(u * th) * q1) / math.sin(th)
    n = float(np.linalg.norm(q))
    return q / n if n > 0 else np.array([0, 0, 0, 1], np.float32)


def _key_interp(times, vals, t, interp, is_quat=False):
    """关键帧采样: STEP / LINEAR (+四元数 SLERP). t 越界按端点 clamp."""
    if len(times) == 1:
        return vals[0]
    t = float(np.clip(t, times[0], times[-1]))
    i = int(np.searchsorted(times, t, side='right')) - 1
    i = max(0, min(i, len(times) - 2))
    t0, t1 = float(times[i]), float(times[i + 1])
    v0, v1 = vals[i], vals[i + 1]
    if interp == 'STEP' or t1 <= t0:
        return v0
    u = (t - t0) / (t1 - t0)
    if is_quat:
        return _slerp(v0, v1, u)
    return (1.0 - u) * v0 + u * v1


# ================= GLB 装载 + 姿态求值 =================

class AnimatedGLB:
    """GLB 全量装载: 节点树 / skins / morph targets / 动画 takes.
    vertices_at(take, t) → 各 primitive 的世界坐标顶点 (变形后)."""

    def __init__(self, path, verbose=False):
        g = GLTF2().load(path)
        self.g = g
        n = len(g.nodes)
        parent = [-1] * n
        for i, nd in enumerate(g.nodes):
            for c in (nd.children or []):
                parent[c] = i
        self.children = [nd.children or [] for nd in g.nodes]
        self.roots = [i for i in range(n) if parent[i] < 0]

        # 节点基础局部变换 (matrix 或 TRS)
        self.base = []
        for nd in g.nodes:
            t = np.array(nd.translation or [0, 0, 0], np.float32)
            q = np.array(nd.rotation or [0, 0, 0, 1], np.float32)
            s = np.array(nd.scale or [1, 1, 1], np.float32)
            m = (np.array(nd.matrix, np.float32).reshape(4, 4).T
                 if nd.matrix else None)             # glTF column-major
            self.base.append((t, q, s, m))

        # skins: (joint node 列表, inverseBindMatrices (J,4,4))
        self.skins = []
        for sk in (g.skins or []):
            if sk.inverseBindMatrices is not None:
                ibm = (_read_accessor(g, sk.inverseBindMatrices)
                       .reshape(-1, 4, 4).transpose(0, 2, 1).astype(np.float32))
            else:
                ibm = np.tile(np.eye(4, dtype=np.float32), (len(sk.joints), 1, 1))
            self.skins.append((list(sk.joints), ibm))

        # primitives
        self.prims = []
        for ni, nd in enumerate(g.nodes):
            if nd.mesh is None:
                continue
            mesh = g.meshes[nd.mesh]
            for prim in mesh.primitives:
                attrs = prim.attributes
                if attrs.POSITION is None or prim.indices is None:
                    continue
                rec = {'node': ni, 'skin': nd.skin,
                       'pos': _read_accessor(g, attrs.POSITION).astype(np.float32),
                       'idx': _get_accessor_data(g, prim.indices).ravel().astype(np.int64)}
                tex_im, tset = _mat_texture(g, prim.material)
                if tex_im is not None:
                    uv_attr = getattr(attrs, f'TEXCOORD_{tset}', None)
                    if uv_attr is not None:
                        rec['tex'] = np.asarray(tex_im, np.uint8)
                        rec['uv'] = _read_accessor(g, uv_attr).astype(np.float32)
                if attrs.COLOR_0 is not None:
                    vc = _read_accessor(g, attrs.COLOR_0).astype(np.float32)
                    if vc.max() > 2.0:
                        vc = vc / 255.0
                    rec['vcol'] = np.clip(vc[:, :3] * 255.0, 0, 255)
                rec['flat'] = (255, 180, 200)
                if prim.material is not None and prim.material < len(g.materials or []):
                    pbr = g.materials[prim.material].pbrMetallicRoughness
                    if pbr and pbr.baseColorFactor:
                        rec['flat'] = tuple(int(c * 255) for c in pbr.baseColorFactor[:3])
                if nd.skin is not None and attrs.JOINTS_0 is not None \
                        and attrs.WEIGHTS_0 is not None:
                    rec['j0'] = _get_accessor_data(g, attrs.JOINTS_0).astype(np.int64)
                    w = _get_accessor_data(g, attrs.WEIGHTS_0)
                    if w.dtype.kind in 'iu':          # normalized ubyte/ushort
                        w = w.astype(np.float32) / float(np.iinfo(w.dtype).max)
                    w = w.astype(np.float32)
                    rec['w0'] = w / np.maximum(w.sum(axis=1, keepdims=True), 1e-8)
                tgts = []
                for tgt in (prim.targets or []):
                    pa = (tgt.get('POSITION') if isinstance(tgt, dict)
                          else getattr(tgt, 'POSITION', None))
                    tgts.append(_read_accessor(g, pa).astype(np.float32)
                                if pa is not None else None)
                if tgts:
                    rec['targets'] = tgts
                    bw = (getattr(nd, 'weights', None) or mesh.weights
                          or [0.0] * len(tgts))
                    rec['base_w'] = np.asarray(bw, np.float32)
                self.prims.append(rec)

        self.takes = [self._load_take(a) for a in (g.animations or [])]
        if verbose:
            print(f'[glb_anim] {os.path.basename(path)}: {len(self.prims)} prims, '
                  f'{len(self.skins)} skins, takes='
                  f'{[(t["name"], round(t["duration"], 2)) for t in self.takes]}', flush=True)

    def _load_take(self, a):
        g = self.g
        chans = {}
        dur = 0.0
        for ch in a.channels:
            tgt = ch.target
            if tgt is None or tgt.node is None or tgt.path is None:
                continue
            samp = a.samplers[ch.sampler]
            times = _read_accessor(g, samp.input).ravel().astype(np.float32)
            vals = _read_accessor(g, samp.output).astype(np.float32)
            interp = (samp.interpolation or 'LINEAR').upper()
            if interp == 'CUBICSPLINE':               # 取中间值, 退化 LINEAR
                vals = vals.reshape(len(times), 3, -1)[:, 1, :]
                interp = 'LINEAR'
            else:
                vals = vals.reshape(len(times), -1)
            chans.setdefault(tgt.node, {})[tgt.path] = (times, vals, interp)
            dur = max(dur, float(times[-1]))
        return {'name': a.name, 'channels': chans, 'duration': dur}

    def pose(self, take, t):
        """时刻 t 的世界矩阵列表 + 每节点 morph weights (动画覆盖的)."""
        n = len(self.g.nodes)
        L = [None] * n
        morph = {}
        ch_all = take['channels'] if take else {}
        for ni in range(n):
            bt, bq, bs, bm = self.base[ni]
            ch = ch_all.get(ni)
            if not ch:
                L[ni] = bm if bm is not None else _trs(bt, bq, bs)
                continue

            def gv(path, default, quat=False):
                c = ch.get(path)
                return default if c is None else _key_interp(c[0], c[1], t, c[2], quat)

            L[ni] = _trs(gv('translation', bt), gv('rotation', bq, True), gv('scale', bs))
            if 'weights' in ch:
                morph[ni] = np.asarray(gv('weights', None), np.float32)
        W = [None] * n
        stack = [(r, np.eye(4, dtype=np.float32)) for r in self.roots]
        while stack:
            ni, pm = stack.pop()
            W[ni] = pm @ L[ni]
            for c in self.children[ni]:
                stack.append((c, W[ni]))
        return W, morph

    def vertices_at(self, take, t):
        """时刻 t 各 primitive 的世界坐标顶点 list[(N,3) float32]."""
        W, morph = self.pose(take, t)
        out = []
        for rec in self.prims:
            pos = rec['pos']
            if 'targets' in rec:                      # morph 先于 skinning
                w = morph.get(rec['node'], rec['base_w'])
                for k, tgt in enumerate(rec['targets']):
                    if tgt is not None and k < len(w) and float(w[k]) != 0.0:
                        pos = pos + float(w[k]) * tgt
            hom = np.hstack([pos, np.ones((len(pos), 1), np.float32)])
            if rec.get('j0') is not None and rec['skin'] is not None:
                joints, ibm = self.skins[rec['skin']]
                jw = np.stack([W[j] for j in joints])            # (J,4,4)
                jm = jw @ ibm
                M = np.einsum('nk,nkij->nij', rec['w0'], jm[rec['j0']])
                world = np.einsum('nij,nj->ni', M, hom)[:, :3]
            else:
                world = (hom @ W[rec['node']].T)[:, :3]
            out.append(world.astype(np.float32))
        return out


# ================= 采样器 =================

def _resolve_take(takes, sel):
    s = str(sel)
    if s.lstrip('-').isdigit():
        i = int(s)
        if -len(takes) <= i < len(takes):
            return takes[i]
        raise ValueError(f'take index {i} out of range (0..{len(takes) - 1})')
    for tk in takes:
        if tk['name'] == s:
            return tk
    raise ValueError(f'take {sel!r} not found; available: {[t["name"] for t in takes]}')


class AnimSampler:
    """GLB 动画点云采样器. points_at(t) → (xyz (S,3), rgb (S,3) 0..255).

    __init__ 在 t=0 参考姿态做一次 area-weighted 采样并烘焙 albedo
    (纹理 / COLOR_0 / baseColorFactor), 之后每帧只做顶点变形 + lambert 重算.
    没有动画的 GLB 也能用 (duration=0, 恒静态)."""

    def __init__(self, path, take='0', samples=400000, lighting='lambert',
                 ambient=0.7, light_dir=(0.3, 0.7, 0.6), seed=20260709, verbose=True):
        self.glb = AnimatedGLB(path, verbose=verbose)
        if not self.glb.prims:
            raise ValueError(f'{path}: no triangle primitives')
        if self.glb.takes:
            self.take = _resolve_take(self.glb.takes, take)
        else:
            self.take = None
            if verbose:
                print(f'[glbanim] WARNING: {os.path.basename(path)} 无动画, 输出静态姿态',
                      flush=True)
        self.duration = self.take['duration'] if self.take else 0.0
        self.lighting = lighting
        self.ambient = float(ambient)
        light = np.asarray(light_dir, np.float32)
        self.light = light / max(float(np.linalg.norm(light)), 1e-9)

        # -- 参考姿态: 三角形表 + area-weighted 采样 --
        verts = self.glb.vertices_at(self.take, 0.0)
        offs = np.concatenate([[0], np.cumsum([len(v) for v in verts])]).astype(np.int64)
        V0 = np.vstack(verts)
        tri_list, prim_ids = [], []
        for pi, rec in enumerate(self.glb.prims):
            tv = rec['idx'].reshape(-1, 3) + offs[pi]
            tri_list.append(tv)
            prim_ids.append(np.full(len(tv), pi, np.int64))
        self.tri = np.vstack(tri_list)                # (T,3) 全局顶点索引
        tri_prim = np.concatenate(prim_ids)
        self.offs = offs

        e1 = V0[self.tri[:, 1]] - V0[self.tri[:, 0]]
        e2 = V0[self.tri[:, 2]] - V0[self.tri[:, 0]]
        areas = 0.5 * np.linalg.norm(np.cross(e1, e2), axis=1).astype(np.float64)
        total = areas.sum()
        if total <= 0:
            raise ValueError(f'{path}: degenerate mesh (zero area) at t=0')
        rng = np.random.default_rng(seed)
        self.face = rng.choice(len(areas), size=samples, p=areas / total)
        u = rng.random(samples, dtype=np.float32)
        v = rng.random(samples, dtype=np.float32)
        flip = u + v > 1.0
        u[flip] = 1.0 - u[flip]
        v[flip] = 1.0 - v[flip]
        self.bary = np.stack([1.0 - u - v, u, v], axis=1)     # (S,3) w,u,v

        # -- albedo 烘焙 (每采样点一次) --
        alb = np.empty((samples, 3), np.float32)
        fprim = tri_prim[self.face]
        for pi, rec in enumerate(self.glb.prims):
            m = fprim == pi
            if not m.any():
                continue
            tv_local = self.tri[self.face[m]] - offs[pi]
            b = self.bary[m]
            if 'tex' in rec and 'uv' in rec:
                uv = (rec['uv'][tv_local] * b[:, :, None]).sum(axis=1)
                tex = rec['tex']
                th, tw = tex.shape[:2]
                tx = (np.mod(uv[:, 0], 1.0) * tw).astype(np.int64) % tw
                ty = (np.mod(uv[:, 1], 1.0) * th).astype(np.int64) % th
                alb[m] = tex[ty, tx, :3].astype(np.float32)
            elif 'vcol' in rec:
                alb[m] = (rec['vcol'][tv_local] * b[:, :, None]).sum(axis=1)
            else:
                alb[m] = np.asarray(rec['flat'], np.float32)
        self.albedo = alb
        if verbose:
            tk = self.take['name'] if self.take else None
            print(f'[glbanim] {len(self.tri)} tris, {samples} sticky samples, '
                  f'take={tk} duration={self.duration:.2f}s', flush=True)

    def points_at(self, t):
        verts = np.vstack(self.glb.vertices_at(self.take, t))
        p = (verts[self.tri[self.face]] * self.bary[:, :, None]).sum(axis=1)
        col = self.albedo
        if self.lighting != 'none':
            e1 = verts[self.tri[:, 1]] - verts[self.tri[:, 0]]
            e2 = verts[self.tri[:, 2]] - verts[self.tri[:, 0]]
            nrm = np.cross(e1, e2)
            nrm /= np.maximum(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-12)
            ndl = np.clip(nrm @ self.light, -1.0, 1.0)
            if self.lighting == 'half-lambert':
                inten = self.ambient + (1 - self.ambient) * (0.5 + 0.5 * ndl) ** 2
            else:                                     # lambert
                inten = self.ambient + (1 - self.ambient) * np.maximum(ndl, 0.0)
            c = col * inten[self.face][:, None]
            mx = c.max(axis=1, keepdims=True)         # hue-preserving 溢出回缩
            c = np.where(mx > 255.0, c * (255.0 / np.maximum(mx, 1e-6)), c)
            col = c
        return p.astype(np.float32), np.clip(col, 0, 255).astype(np.float32)

    def bbox_over(self, times):
        """多个时刻的全局 bbox (采样点是顶点凸组合, 顶点 bbox 即包络)."""
        cmin = np.full(3, np.inf, np.float32)
        cmax = np.full(3, -np.inf, np.float32)
        for t in times:
            V = np.vstack(self.glb.vertices_at(self.take, t))
            cmin = np.minimum(cmin, V.min(axis=0))
            cmax = np.maximum(cmax, V.max(axis=0))
        return cmin, cmax
