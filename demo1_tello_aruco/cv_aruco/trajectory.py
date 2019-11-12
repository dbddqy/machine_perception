from math import pi, sin, cos


center_x = 0.
center_y = -120.
center_z = 0


def get_circular_trajectory(r):
    tra = []
    tra.append([center_x, center_y, center_z, 0.])
    for index in range(100):
        theta = index * 0.01 * 2 * pi
        tra.append([center_x+cos(theta)*r, center_y, center_z+sin(theta)*r, 0.])
    tra.append([center_x, center_y, center_z, 0.])
    print(tra)
    return tra


trajectory_planned = get_circular_trajectory(10)
