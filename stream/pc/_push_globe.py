#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""_push_globe.py — 等板子 9500 起来后立刻推 frames_globerelief.
快重试 (1.5s) 抢在别的 5s 重试推流器前面拿到单客户端槽."""
import time
from povstream import Streamer, frame_iter_from_dir

s = Streamer('10.10.20.234', fps=4.0, loop=True, reconnect=True,
             retry_interval=1.5,
             on_status=lambda ev, d: print(
                 f'[{time.strftime("%H:%M:%S")}] {ev}: {d}', flush=True),
             on_frame=lambda st: (st.frames % 36 == 1) and print(
                 f'[{time.strftime("%H:%M:%S")}] frame {st.frames} acked',
                 flush=True))
s.run(lambda: frame_iter_from_dir('frames_globerelief'))
