import threading
import time
import cv2


def get_image(delay):
    global frame
    global c
    capture = cv2.VideoCapture(0)
    while True:
        ret, frame = capture.read()
        cv2.waitKey(delay)


def show_image(delay):
    while True:
        if frame is None:
            cv2.waitKey(delay)
        else:
            cv2.imshow("img", frame)
            cv2.waitKey(delay)


frame = None
t1 = threading.Thread(target=get_image, args=(2, ))
t1.start()
# t2 = threading.Thread(target=show_image, args=(2, ))
# t2.start()

show_image(2)

