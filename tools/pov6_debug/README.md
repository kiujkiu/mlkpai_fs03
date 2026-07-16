# pov6 板上调试脚本 (v6 双屏 2047, /dev/mem 直控 0x40010000)

点亮 SOP: 单色静态(供电最坏工况) → pov6_colors.py 三色循环 → 数字棋盘格
(tools/gen_chess_obs.py 打包 → TCP 推板 → 双 fb 写入) → pov6_fake.py → sensor。

⚠ 铁律: ①屏在 P3 = B 引擎 = fb_B, 灌图必须双 fb ②dual_en(0x10[2]) 常开
③共阳屏全屏实心 oe_window ≤8 沿, 25% 占空会过流 brownout
④0x10 读回=slice_idx 非控制影子, 禁止读改写
