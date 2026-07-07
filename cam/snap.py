import cv2, time, sys, os
cam = int(sys.argv[1]) if len(sys.argv) > 1 else 1
n   = int(sys.argv[2]) if len(sys.argv) > 2 else 9
gap = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
out = r"D:\claude_workspace\pov3d\mlkpai_fs03\cam"
cap = cv2.VideoCapture(cam, cv2.CAP_MSMF)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
for _ in range(10): cap.read(); time.sleep(0.05)
for i in range(n):
    ok, f = cap.read()
    if ok:
        p = os.path.join(out, f"frame_{i:02d}.jpg")
        cv2.imwrite(p, f)
        b,g,r = f[...,0].astype(int), f[...,1].astype(int), f[...,2].astype(int)
        bright = (r+g+b) > 250
        cnt = int(bright.sum())
        if cnt: print(f"{i}: bright_px={cnt} meanRGB=({int(r[bright].mean())},{int(g[bright].mean())},{int(b[bright].mean())})", flush=True)
        else: print(f"{i}: dark", flush=True)
    time.sleep(gap)
print("done")
