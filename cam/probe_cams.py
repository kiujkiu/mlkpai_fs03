import cv2
for idx in range(4):
    for api, name in [(cv2.CAP_MSMF,'MSMF'), (cv2.CAP_DSHOW,'DSHOW')]:
        cap = cv2.VideoCapture(idx, api)
        ok, f = cap.read()
        print(idx, name, ok, f.shape if ok else '')
        cap.release()
