import cv2
for i in range(4):
    cap = cv2.VideoCapture(i, cv2.CAP_MSMF)
    ok = cap.isOpened()
    frame_ok = False
    if ok:
        for _ in range(5):
            r, f = cap.read()
            if r: frame_ok = True; break
    print(f"cam{i}: opened={ok} frame={frame_ok}", flush=True)
    cap.release()
