from aruco_detection import *

solver = Aruco_detector()

while True:
    solver.detect()
    cv2.imshow("video", solver.image)
    if solver.is_detected:
        print("__________")
        print(f"x={solver.pose_world_to_tello.x}")
        print(f"y={solver.pose_world_to_tello.y}")
        print(f"z={solver.pose_world_to_tello.z}")
        print(f"yaw={solver.pose_world_to_tello.yaw}")
        cv2.waitKey()
    c = cv2.waitKey(1)
    if c == ord("q"):
        break

solver.release()