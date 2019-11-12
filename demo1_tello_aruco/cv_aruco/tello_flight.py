import tellopy
from time import sleep
from tello_controller import *
import av
import traceback
import numpy
import time
import sys
import threading
from aruco_detection_tello import *
from trajectory import *


def handler(event, sender, data, **args):
    drone = sender
    if event is drone.EVENT_FLIGHT_DATA:
        print(data)


def get_image(_drone):
    container = None
    global image
    # connect to video stream
    retry = 3
    while container is None and 0 < retry:
        retry -= 1
        try:
            container = av.open(_drone.get_video_stream())
        except av.AVError as ave:
            print(ave)
            print('retry...')
    _drone.start_video()
    frame_skip = 300
    try:
        while True:
            for frame in container.decode(video=0):
                if 0 < frame_skip:
                    frame_skip = frame_skip - 1
                    continue
                start_time = time.time()
                image = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)
                if frame.time_base < 1.0 / 60:
                    time_base = 1.0 / 60
                else:
                    time_base = frame.time_base
                frame_skip = int((time.time() - start_time) / time_base)
    except Exception as ex:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(ex)
    finally:
        _drone.quit()


def get_pose(solver):
    while True:
        if image is None:
            cv2.waitKey(1)
        else:
            solver.detect(image)
            if solver.is_detected:
                cv2.waitKey(1)
                cv2.imshow("img", solver.image_out)
                return solver.get_pose_world()


def main(controller, solver):
    try:
        sleep(15)
        drone.takeoff()
        sleep(3)
        drone.down(0)
        sleep(3)
        # update drone pose from cv_begin
        # drone_pose = [0, 0, 0, 0]
        # update drone pose from cv_end
        # ref_pose should be updated from file
        ref_pose = trajectory_planned
        # use the command from tello_controller
        for i in range(len(ref_pose)):
            controller.update_ref_pose(ref_pose=ref_pose[i])
            # update drone pose from cv_begin
            drone_pose = get_pose(solver)
            cv2.imshow("img", solver.image_out)
            # update drone pose from cv_end
            # pass the updated pose data to tello_controller
            controller.update_drone_pose(drone_pose=drone_pose)
            controller.update_error()
            while controller.compare_error():
                controller.update_axis_speed()
                controller.send_flight_command()
                # update drone pose from cv_begin
                drone_pose = get_pose(solver)
                # update drone pose from cv_end
                print("changed drone_pose", drone_pose)
                # pass the updated pose data to tello_controller and update error value
                controller.update_drone_pose(drone_pose=drone_pose)
                controller.update_error()
                # stop the command from tello controller
                sleep(dt)
        drone.land()
        sleep(5)
    except Exception as ex:
        print(ex)
    finally:
        drone.quit()


if __name__ == '__main__':
    drone = tellopy.Tello()
    controller = Tello_controller(drone=drone)
    # wake up tello / take off and wait
    # drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)
    drone.connect()
    drone.wait_for_connection(60.0)
    dt = 0.001
    image = None

    t_img = threading.Thread(target=get_image, args=(drone,))
    t_img.start()

    solver = Aruco_detector_tello()
    # while True:
    #     print(get_pose(solver))
    #     cv2.waitKey()
    main(controller, solver)
