import cv2, time, os
out = r"D:\claude_workspace\pov3d\mlkpai_fs03\cam"
cap = cv2.VideoCapture(0, cv2.CAP_MSMF)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
for _ in range(10): cap.read(); time.sleep(0.05)
best = {}
for i in range(60):                      # 2 分钟, 每 2s 一帧
    ok, f = cap.read()
    if not ok: time.sleep(2); continue
    roi = f[0:400, 500:1280]             # 屏区
    b,g,r = roi[...,0].astype(int), roi[...,1].astype(int), roi[...,2].astype(int)
    # LED 点 = 某通道显著高于其他两通道且亮
    red   = int(((r>150) & (r>g+60) & (r>b+60)).sum())
    green = int(((g>150) & (g>r+60) & (g>b+60)).sum())
    blue  = int(((b>150) & (b>r+60) & (b>g+60)).sum())
    tag = max((red,'R'),(green,'G'),(blue,'B'))
    print(f"{i}: R={red} G={green} B={blue}", flush=True)
    if red+green+blue > 30 and (tag[1] not in best or tag[0] > best[tag[1]][0]):
        best[tag[1]] = (tag[0], i)
        cv2.imwrite(os.path.join(out, f"lit_{tag[1]}.jpg"), f)
    time.sleep(2)
print("best:", best)
