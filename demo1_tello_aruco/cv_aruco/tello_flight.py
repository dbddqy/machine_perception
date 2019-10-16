import tellopy
from time import sleep
from tello_controller import *

def handler(event, sender, data, **args):
    drone = sender
    if event is drone.EVENT_FLIGHT_DATA:
        print(data)

def test():
    drone = tellopy.Tello()
    controller = Tello_controller(drone=drone)
    dt = 0.2

    try:
        # wake up tello / take off and wait
        drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)
        drone.connect()
        drone.wait_for_connection(60.0)
        drone.takeoff()
        sleep(3)
        drone.down(0)
        sleep(3)
        # update drone pose from cv_begin
        drone_pose = [0, 0, 0, 0]
        prev_drone_pose = drone_pose.copy()
        # update drone pose from cv_end
        # ref_pose should be updated from file
        ref_pose = [[0, 0, 0, 40], [0, 0, 0, -40]]
        # use the command from tello_controller
        for i in range(len(ref_pose)):
            controller.update_ref_pose(ref_pose=ref_pose[i])
            # update drone pose from cv_begin

            # update drone pose from cv_end
            # pass the updated pose data to tello_controller
            controller.update_drone_pose(drone_pose=drone_pose)
            controller.update_error()
            while controller.compare_error():
                controller.update_axis_speed()
                controller.send_flight_command()
                # update drone pose from cv_begin
                if controller.error[3] > 0:
                   #drone_pose[3] += 10
                    drone_pose[3] = prev_drone_pose[3] + controller.axis_speed["yaw"] * dt
                if controller.error[3] < 0:
                    #drone_pose[3] -= 10
                    drone_pose[3] = prev_drone_pose[3] + controller.axis_speed["yaw"] * dt
                prev_drone_pose = drone_pose.copy()
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
    test()