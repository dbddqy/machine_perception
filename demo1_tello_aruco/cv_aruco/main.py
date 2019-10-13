from aruco_detection import *

solver = Aruco_detector()

while True:
    solver.detect()
    cv2.imshow("video", solver.image)
    if solver.is_detected:
        pose = solver.pose_world_to_tello
        print("__________")
        print(f"x={pose.x}")
        print(f"y={pose.y}")
        print(f"z={pose.z}")
        print(f"yaw={pose.yaw}")
        cv2.waitKey()
    c = cv2.waitKey(1)
    if c == ord("q"):
        break

solver.release()
