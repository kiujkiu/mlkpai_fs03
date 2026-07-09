#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""make_globe_glb.py — 生成 POV 用地球仪 GLB (UV 球 + 1-bit 友好贴图).

两种模式:
  flat   (默认): 纯通道三色分类 (海纯蓝/陆纯绿/冰白), 光滑球面
  relief:        太空视角浮雕版 — 顶点按海拔径向位移 (--exag 夸张比例)
                 + 分层设色 (hypsometric): 低地绿→中山黄→高山红→雪峰白,
                 全部锚点是 1-bit 纯组合色 (R/G/B 各自 0/255), 中间过渡
                 交给 Bayer 抖动逐通道混色, 色深仍是 1-bit

数据源 (均本地, 办公网拦外链):
  earth_clean.jpg    NASA Blue Marble, 海/冰分类 (阈值同 povstream globe 源)
  earth_topo.png     2048x1024 灰度海拔 (three-globe earth-topology, 海=0)

颜色放 baseColorTexture (glb_to_points.sample_triangles 只读它, 配
--lighting none 点云颜色 = 贴图原色); emissive 同贴图, viewer 预览不吃光照.
纯 pygltflib 手写 GLB (WSL 无 trimesh, PEP 668 拦 pip). 用法:
  python3 make_globe_glb.py [flat|relief] [--exag 0.12] [-o out.glb]
"""
import io
import os
import sys
import math
import argparse
import numpy as np
from PIL import Image
import pygltflib as gl

HERE = os.path.dirname(os.path.abspath(__file__))
R = 60.0
TEX_W, TEX_H = 1024, 512

# 分层设色锚点 (海拔归一 0..1 → RGB, 全 1-bit 纯组合色)
HYPSO = [(0.00, (0, 255, 0)),        # 低地 = 绿
         (0.30, (255, 255, 0)),      # 中山 = 黄
         (0.65, (255, 0, 0)),        # 高山 = 红
         (1.00, (255, 255, 255))]    # 雪峰 = 白


def _grid_latlon(nlat, nlon):
    lat = np.linspace(math.pi / 2, -math.pi / 2, nlat, dtype=np.float64)
    lon = np.linspace(-math.pi, math.pi, nlon, dtype=np.float64)
    return np.meshgrid(lat, lon, indexing='ij')


def _sample_equirect(img, LA, LO):
    """(lat,lon) 弧度 → 等距圆柱图像素 (最近邻)."""
    H, W = img.shape[:2]
    r = np.clip(((math.pi / 2 - LA) / math.pi * H).astype(np.int32), 0, H - 1)
    c = np.clip(((LO + math.pi) / (2 * math.pi) * W).astype(np.int32), 0, W - 1)
    return img[r, c]


def load_masks_elev():
    """earth_clean 分类掩膜 + 归一化海拔图 (全在源图分辨率)."""
    tex = np.asarray(Image.open(os.path.join(HERE, 'earth_clean.jpg'))
                     .convert('RGB'), np.int32)
    r, g, b = tex[..., 0], tex[..., 1], tex[..., 2]
    ocean = (b > g + 10) & (b > r + 10)
    ice = (r > 170) & (g > 170) & (b > 170)
    topo = np.asarray(Image.open(os.path.join(HERE, 'earth_topo.png'))
                      .convert('L'), np.float32)
    elev = topo / max(topo.max(), 1.0)
    return ocean, ice, elev


def hypso_color(e):
    """海拔归一 (...,) → RGB float (..., 3), 锚点线性插值."""
    e = np.clip(e, 0.0, 1.0)
    out = np.zeros(e.shape + (3,), np.float32)
    for (e0, c0), (e1, c1) in zip(HYPSO, HYPSO[1:]):
        m = (e >= e0) & (e <= e1)
        t = ((e[m] - e0) / (e1 - e0))[:, None] if m.any() else None
        if t is not None:
            out[m] = np.asarray(c0, np.float32) + t * (
                np.asarray(c1, np.float32) - np.asarray(c0, np.float32))
    return out


def make_texture(relief):
    """分类贴图: flat=三色, relief=分层设色 (海/冰仍来自 earth_clean)."""
    ocean, ice, elev = load_masks_elev()
    LA, LO = _grid_latlon(TEX_H, TEX_W)
    oc = _sample_equirect(ocean, LA, LO)
    ic = _sample_equirect(ice, LA, LO)
    if relief:
        # 冰盖只认高纬 (亮沙漠 RGB 全>170 会误判成冰, 撒哈拉走海拔色带→黄)
        ic &= np.abs(LA) > math.radians(50)
        e = _sample_equirect(elev, LA, LO) ** 0.7      # gamma 提升中海拔层次
        out = hypso_color(e).astype(np.uint8)
    else:
        out = np.zeros((TEX_H, TEX_W, 3), np.uint8)
        out[:] = (0, 255, 0)
    out[oc] = (0, 0, 255)
    out[ic] = (255, 255, 255)
    buf = io.BytesIO()
    Image.fromarray(out).save(buf, 'PNG', optimize=True)
    print(f'texture {TEX_W}x{TEX_H}: ocean={oc.mean():.0%} ice={ic.mean():.0%} '
          f'land={1 - oc.mean() - ic.mean():.0%} relief={relief}')
    return buf.getvalue()


def make_sphere(relief, exag, nlat, nlon):
    """UV 球 (equirect UV, y 极轴). relief 时顶点半径 = R×(1+海拔×exag),
    海面 (earth_clean 分类) 钉在 R."""
    LA, LO = _grid_latlon(nlat, nlon)
    rr = np.full(LA.shape, R, np.float64)
    if relief:
        ocean, _, elev = load_masks_elev()
        e = _sample_equirect(elev, LA, LO)
        e[_sample_equirect(ocean, LA, LO)] = 0.0
        rr = R * (1.0 + e * exag)
    pos = np.stack([rr * np.cos(LA) * np.cos(LO),
                    rr * np.sin(LA),
                    rr * np.cos(LA) * np.sin(LO)], axis=-1)
    uv = np.stack([(LO + math.pi) / (2 * math.pi),
                   (math.pi / 2 - LA) / math.pi], axis=-1)
    i = np.arange(nlat - 1)[:, None] * nlon + np.arange(nlon - 1)[None, :]
    a, b, c, d = i, i + 1, i + nlon, i + nlon + 1
    faces = np.concatenate([np.stack([a, c, b], -1).reshape(-1, 3),
                            np.stack([b, c, d], -1).reshape(-1, 3)])
    return (pos.reshape(-1, 3).astype(np.float32),
            uv.reshape(-1, 2).astype(np.float32),
            faces.astype(np.uint32).ravel())


def write_glb(out_path, png, pos, uv, idx):
    def pad4(b):
        return b + b'\0' * (-len(b) % 4)

    blobs = [pos.tobytes(), uv.tobytes(), idx.tobytes(), png]
    views, off, blob = [], 0, b''
    for b in blobs:
        views.append(gl.BufferView(buffer=0, byteOffset=off, byteLength=len(b)))
        b = pad4(b)
        blob += b
        off += len(b)

    g = gl.GLTF2(
        asset=gl.Asset(version='2.0', generator='make_globe_glb.py'),
        scene=0,
        scenes=[gl.Scene(nodes=[0])],
        nodes=[gl.Node(mesh=0, name='globe')],
        meshes=[gl.Mesh(primitives=[gl.Primitive(
            attributes=gl.Attributes(POSITION=0, TEXCOORD_0=1),
            indices=2, material=0)])],
        accessors=[
            gl.Accessor(bufferView=0, componentType=gl.FLOAT, count=len(pos),
                        type=gl.VEC3, min=pos.min(0).tolist(),
                        max=pos.max(0).tolist()),
            gl.Accessor(bufferView=1, componentType=gl.FLOAT, count=len(uv),
                        type=gl.VEC2),
            gl.Accessor(bufferView=2, componentType=gl.UNSIGNED_INT,
                        count=len(idx), type=gl.SCALAR),
        ],
        bufferViews=views,
        buffers=[gl.Buffer(byteLength=len(blob))],
        images=[gl.Image(bufferView=3, mimeType='image/png')],
        samplers=[gl.Sampler()],
        textures=[gl.Texture(source=0, sampler=0)],
        materials=[gl.Material(
            pbrMetallicRoughness=gl.PbrMetallicRoughness(
                baseColorTexture=gl.TextureInfo(index=0),
                metallicFactor=0.0, roughnessFactor=1.0),
            emissiveTexture=gl.TextureInfo(index=0),
            emissiveFactor=[1.0, 1.0, 1.0],
            doubleSided=True)],
    )
    g.set_binary_blob(blob)
    g.save(out_path)
    print(f'wrote {out_path}: {len(pos)} verts, {len(idx)//3} tris, '
          f'{os.path.getsize(out_path)/1024:.0f} KB')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('mode', nargs='?', choices=['flat', 'relief'], default='flat')
    ap.add_argument('--exag', type=float, default=0.12,
                    help='relief: 海拔夸张 (最高峰半径增幅比例)')
    ap.add_argument('--nlat', type=int, default=None)
    ap.add_argument('--nlon', type=int, default=None)
    ap.add_argument('-o', '--out', default=None)
    args = ap.parse_args()
    relief = args.mode == 'relief'
    nlat = args.nlat or (181 if relief else 121)
    nlon = args.nlon or (361 if relief else 241)
    out = args.out or os.path.join(
        HERE, 'globe_relief.glb' if relief else 'globe_pure.glb')
    png = make_texture(relief)
    pos, uv, idx = make_sphere(relief, args.exag, nlat, nlon)
    write_glb(out, png, pos, uv, idx)


if __name__ == '__main__':
    main()
