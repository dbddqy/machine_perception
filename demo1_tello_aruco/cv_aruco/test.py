from aruco_detection_tello import *

solver = Aruco_detector_tello()

capture = cv2.VideoCapture(0)

while True:
    ret, frame = capture.read()
    solver.detect(frame)
    if solver.is_detected:
        print(solver.get_pose_world())
        cv2.imshow("img", solver.image_out)
        c = cv2.waitKey()
        if c == ord("q"):
            break

capture.release()
cv2.destroyAllWindows()