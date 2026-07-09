#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
make_test_glb.py — 程序化生成小型动画 GLB 测试资产 (glb_anim / povstream 测试用).

  spincube.glb  纹理棋盘立方体 + 绕 Y 旋转 (节点 TRS 动画 + 纹理采样路径)
  armskin.glb   两骨骼手臂: 竖直柱体 + joint1 绕 Z 摆动 (skinning + SLERP 路径)
  morphcube.glb 立方体 + POSITION morph target (顶面拉高) + weights 动画

直接运行: python3 make_test_glb.py  → 在本目录生成三个 .glb
测试还用 make_offset_cube() 生成 glbseq 帧序列.
"""
import io
import os
import math
import numpy as np
from PIL import Image as PILImage
from pygltflib import (GLTF2, Scene, Node, Mesh, Primitive, Attributes, Buffer,
                       BufferView, Accessor, Animation, AnimationChannel,
                       AnimationChannelTarget, AnimationSampler, Skin, Material,
                       PbrMetallicRoughness, TextureInfo, Texture, Image, Sampler)

HERE = os.path.dirname(os.path.abspath(__file__))
FLOAT, UBYTE, USHORT, UINT = 5126, 5121, 5123, 5125


class GlbBuilder:
    def __init__(self):
        self.g = GLTF2(scene=0, scenes=[Scene(nodes=[])])
        self.blob = bytearray()

    def add_view(self, data):
        while len(self.blob) % 4:
            self.blob += b'\0'
        off = len(self.blob)
        self.blob += data
        self.g.bufferViews.append(BufferView(buffer=0, byteOffset=off, byteLength=len(data)))
        return len(self.g.bufferViews) - 1

    def add_accessor(self, arr, type_, ctype, minmax=False):
        bv = self.add_view(arr.tobytes())
        acc = Accessor(bufferView=bv, componentType=ctype, count=len(arr), type=type_)
        if minmax:
            a2 = arr.reshape(len(arr), -1)
            acc.max = a2.max(axis=0).tolist()
            acc.min = a2.min(axis=0).tolist()
        self.g.accessors.append(acc)
        return len(self.g.accessors) - 1

    def save(self, path):
        self.g.buffers = [Buffer(byteLength=len(self.blob))]
        self.g.set_binary_blob(bytes(self.blob))
        self.g.save_binary(path)
        print(f'[glb] wrote {path} ({os.path.getsize(path)}B)')


def rot_quat(axis, deg):
    """轴角 → glTF 四元数 [x,y,z,w]."""
    a = np.asarray(axis, np.float32)
    a = a / np.linalg.norm(a)
    h = math.radians(deg) / 2.0
    return [float(a[0] * math.sin(h)), float(a[1] * math.sin(h)),
            float(a[2] * math.sin(h)), float(math.cos(h))]


def cube_geometry(size=1.0, offset=(0.0, 0.0, 0.0)):
    """24 顶点 (每面独立, 带 UV) 12 三角立方体."""
    h = size / 2.0
    faces = [  # (normal axis, 4 corners CCW)
        [(+h, -h, -h), (+h, +h, -h), (+h, +h, +h), (+h, -h, +h)],   # +x
        [(-h, -h, +h), (-h, +h, +h), (-h, +h, -h), (-h, -h, -h)],   # -x
        [(-h, +h, -h), (-h, +h, +h), (+h, +h, +h), (+h, +h, -h)],   # +y
        [(-h, -h, +h), (-h, -h, -h), (+h, -h, -h), (+h, -h, +h)],   # -y
        [(-h, -h, +h), (+h, -h, +h), (+h, +h, +h), (-h, +h, +h)],   # +z
        [(+h, -h, -h), (-h, -h, -h), (-h, +h, -h), (+h, +h, -h)],   # -z
    ]
    pos, uv, idx = [], [], []
    for f in faces:
        b = len(pos)
        pos += f
        uv += [(0, 0), (1, 0), (1, 1), (0, 1)]
        idx += [b, b + 1, b + 2, b, b + 2, b + 3]
    pos = np.asarray(pos, np.float32) + np.asarray(offset, np.float32)
    return pos, np.asarray(uv, np.float32), np.asarray(idx, np.uint16)


def _checker_png(c0=(255, 40, 40), c1=(255, 220, 40), n=8):
    im = PILImage.new('RGB', (n, n))
    for y in range(n):
        for x in range(n):
            im.putpixel((x, y), c0 if (x + y) % 2 else c1)
    buf = io.BytesIO()
    im.save(buf, 'PNG')
    return buf.getvalue()


def make_spincube(path):
    """纹理棋盘立方体, 绕 Y 旋转一整圈 (LINEAR quats, duration 1.5s)."""
    b = GlbBuilder()
    pos, uv, idx = cube_geometry(1.0)
    a_pos = b.add_accessor(pos, 'VEC3', FLOAT, minmax=True)
    a_uv = b.add_accessor(uv, 'VEC2', FLOAT)
    a_idx = b.add_accessor(idx.reshape(-1, 1), 'SCALAR', USHORT)
    png_bv = b.add_view(_checker_png())
    b.g.images = [Image(mimeType='image/png', bufferView=png_bv)]
    b.g.samplers = [Sampler()]
    b.g.textures = [Texture(source=0, sampler=0)]
    b.g.materials = [Material(pbrMetallicRoughness=PbrMetallicRoughness(
        baseColorTexture=TextureInfo(index=0), metallicFactor=0.0))]
    b.g.meshes = [Mesh(primitives=[Primitive(
        attributes=Attributes(POSITION=a_pos, TEXCOORD_0=a_uv),
        indices=a_idx, material=0)])]
    b.g.nodes = [Node(mesh=0)]
    b.g.scenes[0].nodes = [0]
    times = np.asarray([0.0, 0.5, 1.0, 1.5], np.float32)
    quats = np.asarray([rot_quat((0, 1, 0), d) for d in (0, 120, 240, 360)], np.float32)
    a_t = b.add_accessor(times.reshape(-1, 1), 'SCALAR', FLOAT, minmax=True)
    a_q = b.add_accessor(quats, 'VEC4', FLOAT)
    b.g.animations = [Animation(
        name='spin',
        samplers=[AnimationSampler(input=a_t, output=a_q, interpolation='LINEAR')],
        channels=[AnimationChannel(sampler=0,
                                   target=AnimationChannelTarget(node=0, path='rotation'))])]
    b.save(path)


def make_armskin(path):
    """两骨骼手臂: y∈[0,2] 柱体, joint1@y=1 绕 Z 摆 0→60→90→60→0°.
    顶点色 y 渐变 (COLOR_0 路径), duration 1.6s."""
    b = GlbBuilder()
    rings = np.linspace(0.0, 2.0, 9, dtype=np.float32)
    corners = [(-0.15, -0.15), (0.15, -0.15), (0.15, 0.15), (-0.15, 0.15)]
    pos = np.asarray([(cx, y, cz) for y in rings for cx, cz in corners], np.float32)
    idx = []
    for r in range(len(rings) - 1):
        for c in range(4):
            i0 = r * 4 + c
            i1 = r * 4 + (c + 1) % 4
            j0, j1 = i0 + 4, i1 + 4
            idx += [i0, i1, j1, i0, j1, j0]
    idx = np.asarray(idx, np.uint16)
    ty = pos[:, 1] / 2.0
    vcol = np.stack([ty, 0.2 + 0 * ty, 1.0 - ty], axis=1).astype(np.float32)  # 蓝→红
    w1 = np.clip((pos[:, 1] - 0.6) / 0.8, 0.0, 1.0).astype(np.float32)
    weights = np.stack([1.0 - w1, w1, 0 * w1, 0 * w1], axis=1)
    joints = np.zeros((len(pos), 4), np.uint8)
    joints[:, 1] = 1

    a_pos = b.add_accessor(pos, 'VEC3', FLOAT, minmax=True)
    a_col = b.add_accessor(vcol, 'VEC3', FLOAT)
    a_j = b.add_accessor(joints, 'VEC4', UBYTE)
    a_w = b.add_accessor(weights, 'VEC4', FLOAT)
    a_idx = b.add_accessor(idx.reshape(-1, 1), 'SCALAR', USHORT)

    ibm = np.zeros((2, 16), np.float32)
    ibm[0] = np.eye(4, dtype=np.float32).T.flatten()          # joint0 全局 = I
    m1 = np.eye(4, dtype=np.float32)
    m1[1, 3] = -1.0                                           # inverse(T(0,1,0))
    ibm[1] = m1.T.flatten()                                   # column-major
    a_ibm = b.add_accessor(ibm, 'MAT4', FLOAT)

    b.g.meshes = [Mesh(primitives=[Primitive(
        attributes=Attributes(POSITION=a_pos, COLOR_0=a_col, JOINTS_0=a_j, WEIGHTS_0=a_w),
        indices=a_idx)])]
    b.g.nodes = [Node(name='joint0', children=[1]),
                 Node(name='joint1', translation=[0.0, 1.0, 0.0]),
                 Node(name='arm', mesh=0, skin=0)]
    b.g.skins = [Skin(joints=[0, 1], inverseBindMatrices=a_ibm, skeleton=0)]
    b.g.scenes[0].nodes = [0, 2]

    times = np.asarray([0.0, 0.4, 0.8, 1.2, 1.6], np.float32)
    quats = np.asarray([rot_quat((0, 0, 1), d) for d in (0, 60, 90, 60, 0)], np.float32)
    a_t = b.add_accessor(times.reshape(-1, 1), 'SCALAR', FLOAT, minmax=True)
    a_q = b.add_accessor(quats, 'VEC4', FLOAT)
    b.g.animations = [Animation(
        name='wave',
        samplers=[AnimationSampler(input=a_t, output=a_q, interpolation='LINEAR')],
        channels=[AnimationChannel(sampler=0,
                                   target=AnimationChannelTarget(node=1, path='rotation'))])]
    b.save(path)


def make_morphcube(path):
    """立方体 + morph target (顶面 y>0 顶点 +1) + weights 0→1→0 动画."""
    b = GlbBuilder()
    pos, uv, idx = cube_geometry(1.0)
    delta = np.zeros_like(pos)
    delta[pos[:, 1] > 0, 1] = 1.0
    a_pos = b.add_accessor(pos, 'VEC3', FLOAT, minmax=True)
    a_del = b.add_accessor(delta, 'VEC3', FLOAT, minmax=True)
    a_idx = b.add_accessor(idx.reshape(-1, 1), 'SCALAR', USHORT)
    b.g.materials = [Material(pbrMetallicRoughness=PbrMetallicRoughness(
        baseColorFactor=[0.2, 1.0, 0.4, 1.0], metallicFactor=0.0))]
    b.g.meshes = [Mesh(primitives=[Primitive(
        attributes=Attributes(POSITION=a_pos), indices=a_idx, material=0,
        targets=[Attributes(POSITION=a_del)])], weights=[0.0])]
    b.g.nodes = [Node(mesh=0)]
    b.g.scenes[0].nodes = [0]
    times = np.asarray([0.0, 0.6, 1.2], np.float32)
    wvals = np.asarray([[0.0], [1.0], [0.0]], np.float32)
    a_t = b.add_accessor(times.reshape(-1, 1), 'SCALAR', FLOAT, minmax=True)
    a_w = b.add_accessor(wvals, 'SCALAR', FLOAT)
    b.g.animations = [Animation(
        name='stretch',
        samplers=[AnimationSampler(input=a_t, output=a_w, interpolation='LINEAR')],
        channels=[AnimationChannel(sampler=0,
                                   target=AnimationChannelTarget(node=0, path='weights'))])]
    b.save(path)


def make_offset_cube(path, offset, color):
    """静态平移立方体 (glbseq 帧序列测试用). color = (r,g,b) 0..255."""
    b = GlbBuilder()
    pos, uv, idx = cube_geometry(1.0, offset)
    a_pos = b.add_accessor(pos, 'VEC3', FLOAT, minmax=True)
    a_idx = b.add_accessor(idx.reshape(-1, 1), 'SCALAR', USHORT)
    b.g.materials = [Material(pbrMetallicRoughness=PbrMetallicRoughness(
        baseColorFactor=[color[0] / 255.0, color[1] / 255.0, color[2] / 255.0, 1.0],
        metallicFactor=0.0))]
    b.g.meshes = [Mesh(primitives=[Primitive(
        attributes=Attributes(POSITION=a_pos), indices=a_idx, material=0)])]
    b.g.nodes = [Node(mesh=0)]
    b.g.scenes[0].nodes = [0]
    b.save(path)


if __name__ == '__main__':
    make_spincube(os.path.join(HERE, 'spincube.glb'))
    make_armskin(os.path.join(HERE, 'armskin.glb'))
    make_morphcube(os.path.join(HERE, 'morphcube.glb'))
