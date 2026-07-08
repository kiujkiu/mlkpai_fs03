# 拍单帧: python snap_one.py <输出.jpg> [cam=0]
import cv2, sys, time
out = sys.argv[1]
cam = int(sys.argv[2]) if len(sys.argv) > 2 else 0
cap = cv2.VideoCapture(cam, cv2.CAP_MSMF)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
ok = False
for _ in range(15):
    r, f = cap.read()
    if r: ok = True
    time.sleep(0.05)
if ok:
    cv2.imwrite(out, f)
    print(f"saved {out}")
else:
    print("FAIL: no frame")
cap.release()
