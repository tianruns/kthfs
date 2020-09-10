import json
import math
import matplotlib.pyplot as plt
import numpy as np

# parameters of kinematic model
wheelbase = 1
velocity = 1
dt = 0.1
lad = 0.05

show_animation = True
# import all goal points
with open('path.json', 'r') as f:
    path = json.load(f)


def update(state, steering_angle):
    # define state update
    u = np.array(
        [velocity * dt * np.cos(state[2]),
         velocity * dt * np.sin(state[2]),
         velocity * dt * np.tan(steering_angle) / wheelbase]
    )

    state = state + u

    return state


def target(state, path_x, path_y):
    # define finding target path point
    # search index of nearest point
    dx = []
    dy = []
    dis = []
    for inx in path_x:
        dx.append(state[0] - inx)
    for iny in path_y:
        dy.append(state[1] - iny)
    for (inx, iny) in zip(dx, dy):
        dis.append((inx ** 2 + iny ** 2) ** 0.5)
    ind = dis.index(min(dis))
    # search the index of next goal point
    ind_1 = ind
    d = 0
    while d<lad and (ind + 1) < len(path_x):
        lx = path_x[ind+1] - path_x[ind_1]
        ly = path_y[ind+1] - path_y[ind_1]
        d = (lx ** 2 + ly ** 2) ** 0.5
        if d <= lad:
            ind = ind + 1
        else:
            break

    return ind


def pure_pursuit(state, path_x, path_y):
    # define pure pursuit algorithm
    ind = target(state, path_x, path_y)
    if ind < len(path_x):
        x_0 = path_x[ind]
        y_0 = path_y[ind]
    else:
        x_0 = path_x[-1]
        y_0 = path_y[-1]
    # alpha = math.atan2(y_0 - state[1], x_0 - state[0]) - state[0]
    # change coordinate
    x_1 = x_0 - state[0]
    y_1 = y_0 - state[1]
    theta = state[2] % (2 * math.pi) - math.pi
    x_2 = x_1 * np.cos(theta) + y_1 * np.sin(theta)
    y_2 = y_1 * np.cos(theta) - x_1 * np.sin(theta)
    # calculate curvature
    delta = math.atan2((2 * x_2) / (x_2 ** 2 + y_2 ** 2) * wheelbase, 1)
    # delta = math.atan2(2.0 * 1 * math.sin(alpha) / lad, 1.0)  # 核心计算公式

    return delta, ind


def main():
    #  set of target points
    path_x = []
    path_y = []
    for [i, j] in path:
        path_x.append(i)
        path_y.append(j)
    x = []
    y = []
    yaw =[]
    t =[]
    # max simulation time
    T = 1000.0

    # initial state
    state = np.array([0, 0, 0])  # (x, y, yaw)
    time = 0.0
    t = [0.0]
    target_ind = target(state, path_x, path_y)
    while T >= time and target_ind < len(path_x):
        delta, target_ind = pure_pursuit(state, path_x, path_y)
        state = update(state, delta)
        time = time + dt
        # visualization
        x.append(state[0])
        y.append(state[1])
        yaw.append(state[2])
        plt.cla()
        plt.plot(path_x, path_y, ".r", label="path")
        plt.plot(x, y, "-b", label="trajectory")
        plt.plot(path_x[target_ind], path_y[target_ind], "go", label="target")
        plt.axis("equal")
        plt.grid(True)
        plt.title("Speed[m/s]:" + str(velocity)[:4])
        plt.pause(0.001)
    plt.show()


if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()























