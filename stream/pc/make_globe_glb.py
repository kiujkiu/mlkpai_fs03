#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""make_globe_glb.py — 生成 POV 用地球仪 GLB (UV 球 + 纯通道色贴图).

贴图 = earth_clean.jpg 逐像素分类成纯通道色 (海纯蓝/陆纯绿/冰白, 阈值同
povstream globe 源): 1-bit Bayer 抖动下混通道色会偏青, 图形学内容一律纯
R/G/B. 颜色放 baseColorTexture (glb_to_points.sample_triangles 只读它,
配 --lighting none 点云颜色 = 贴图原色); emissive 同贴图, viewer 预览不吃光照.

纯 pygltflib 手写 GLB (WSL 无 trimesh, PEP 668 拦 pip). 用法:
  python3 make_globe_glb.py [out.glb]     默认 globe_pure.glb
"""
import io
import os
import sys
import math
import struct
import numpy as np
from PIL import Image
import pygltflib as gl

HERE = os.path.dirname(os.path.abspath(__file__))
R = 60.0
NLAT, NLON = 121, 241           # 顶点网格 (lon 末列 = 首列, 接缝闭合)
TEX_W, TEX_H = 1024, 512


def make_texture():
    """earth_clean.jpg → 纯通道色分类贴图 (阈值 = povstream globe_frames)."""
    tex = np.asarray(Image.open(os.path.join(HERE, 'earth_clean.jpg'))
                     .convert('RGB'), np.int32)
    r, g, b = tex[..., 0], tex[..., 1], tex[..., 2]
    ocean = (b > g + 10) & (b > r + 10)
    ice = (r > 170) & (g > 170) & (b > 170)
    out = np.zeros(tex.shape, np.uint8)
    out[:] = (0, 255, 0)                    # 陆 = 纯绿
    out[ocean] = (0, 0, 255)                # 海 = 纯蓝
    out[ice] = (255, 255, 255)              # 冰盖 = 白
    img = Image.fromarray(out).resize((TEX_W, TEX_H), Image.NEAREST)
    buf = io.BytesIO()
    img.save(buf, 'PNG', optimize=True)
    stat = {c: float(m.mean()) for c, m in
            [('ocean', ocean), ('ice', ice), ('land', ~(ocean | ice))]}
    print(f'texture {TEX_W}x{TEX_H}: ' +
          ' '.join(f'{k}={v:.0%}' for k, v in stat.items()))
    return buf.getvalue()


def make_sphere():
    """UV 球 (equirect UV, y 极轴, 同 zynq_pov _make_globe_glb 约定)."""
    lat = np.linspace(math.pi / 2, -math.pi / 2, NLAT, dtype=np.float64)
    lon = np.linspace(-math.pi, math.pi, NLON, dtype=np.float64)
    LA, LO = np.meshgrid(lat, lon, indexing='ij')
    pos = np.stack([np.cos(LA) * np.cos(LO) * R,
                    np.sin(LA) * R,
                    np.cos(LA) * np.sin(LO) * R], axis=-1)
    uv = np.stack([(LO + math.pi) / (2 * math.pi),
                   (math.pi / 2 - LA) / math.pi], axis=-1)
    i = np.arange(NLAT - 1)[:, None] * NLON + np.arange(NLON - 1)[None, :]
    a, b, c, d = i, i + 1, i + NLON, i + NLON + 1
    faces = np.concatenate([np.stack([a, c, b], -1).reshape(-1, 3),
                            np.stack([b, c, d], -1).reshape(-1, 3)])
    return (pos.reshape(-1, 3).astype(np.float32),
            uv.reshape(-1, 2).astype(np.float32),
            faces.astype(np.uint32).ravel())


def main(out_path):
    png = make_texture()
    pos, uv, idx = make_sphere()

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


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, 'globe_pure.glb'))
