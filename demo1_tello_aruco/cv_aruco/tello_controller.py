from simple_pid import PID
from time import sleep

class Tello_controller:
    # initial a tello object in main script, pass it in to the function
    def __init__(self, drone):
        self.drone = drone
        self.axis_command = {
        "x": self.drone.right,
        "y": self.drone.forward,
        "z": self.drone.up,
        "yaw": self.drone.clockwise
        }
        self.axis_speed = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
        self.prev_axis_speed = self.axis_speed.copy()
        self.pid_x = PID(0.5, 0, 0, setpoint=0, output_limits=(-20, 20))  # for x
        self.pid_y = PID(0.5, 0, 0, setpoint=0, output_limits=(-20, 20))  # for y
        self.pid_z = PID(0.5, 0, 0, setpoint=0, output_limits=(-20, 20))  # for z
        self.pid_yaw = PID(0.5, 0, 0, setpoint=0, output_limits=(-50, 50))  # for yaw
        self.ref_pose = None
        self.drone_pose = [0.0, 0.0, 0.0, 0.0]
        self.error = [0.0, 0.0, 0.0, 0.0]
        self.tolerance = [1.0, 1.0, 1.0, 1.0]

    # set axis speed and send flight command to the drone
    def send_flight_command(self):
        for axis, command in self.axis_command.items():
            if self.axis_speed[axis] is not None and self.axis_speed[axis] != self.prev_axis_speed[axis]:
                command(self.axis_speed[axis])
                self.prev_axis_speed[axis] = self.axis_speed[axis]
                print("sent command to flight", command, self.axis_speed)
            else:
                self.axis_speed[axis] = self.prev_axis_speed[axis]

    # calculate error between reference pose and current pose
    def update_error(self):
        pose_list_length = 4
        error = [0, 0, 0, 0]
        if len(self.ref_pose) != pose_list_length or len(self.drone_pose) != pose_list_length:
            print("Not able to process error calculation. Please check list length.")
        else:
            for i in range(pose_list_length):
                error[i] = self.ref_pose[i] - self.drone_pose[i]
            self.error = error
            print("current error", self.error)

    # compare current error with the tolerance
    def compare_error(self):
        error_abs = [abs(self.error[0]), abs(self.error[1]), abs(self.error[2]), abs(self.error[3])]
        state = 0
        for i in range(len(self.error)):
            if error_abs[i] < self.tolerance[i]:
                state += 1
            else:
                state += 0
        if state == len(self.error):
            return False
        else:
            return True

    # update ref_pose
    def update_ref_pose(self, ref_pose: list):
        self.ref_pose = ref_pose
        print("current ref_pose", self.ref_pose)

    # update the current pose data from the cv
    def update_drone_pose(self, drone_pose: list):
        self.drone_pose = drone_pose
        print("current drone_pose", self.drone_pose)


    # update pid control data
    def update_axis_speed(self):
        self.prev_axis_speed = self.axis_speed.copy()
        self.axis_speed["x"] = int(-self.pid_x(self.error[0]))
        self.axis_speed["y"] = int(-self.pid_y(self.error[1]))
        self.axis_speed["z"] = int(-self.pid_z(self.error[2]))
        self.axis_speed["yaw"] = int(-self.pid_yaw(self.error[3]))
        print("current axis speed", self.axis_speed)
















