from aruco_detection_py import *
import tellopy
from tello_controller import *


# solver = Aruco_detector()
def handler(event, sender, data, **args):
    drone = sender
    if event is drone.EVENT_FLIGHT_DATA:
        print(data)


while True:
    drone = tellopy.Tello()
    drone.start_video()
    controller = Tello_controller(drone=drone)
    try:
        drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)
        drone.connect()
        drone.wait_for_connection(60.0)
        drone.takeoff()
        sleep(3)
        drone.down(0)
        sleep(3)
        while True:
            pic = drone.take_picture()
            print(pic)
            c = cv2.waitKey(1)
            if c == ord("q"):
                break
    except Exception as ex:
        print(ex)
    finally:
        drone.quit()

    # solver.detect()
    # cv2.imshow("video", solver.image)
    # if solver.is_detected:
    #     pose = solver.pose_world_to_tello
    #     print("__________")
    #     print(f"x={pose.x}")
    #     print(f"y={pose.y}")
    #     print(f"z={pose.z}")
    #     print(f"yaw={pose.yaw}")
    #     cv2.waitKey()
    # c = cv2.waitKey(1)
    # if c == ord("q"):
    #     break

# solver.release()
