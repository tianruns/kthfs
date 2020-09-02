# import important modules
from itertools import count
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation


# define parent object


class Point(object):

    def __init__(self, t):
        self.t = float(t)

    def fun(self):
        lamb = 5*math.sin(2*math.pi*self.t)
        h = math.pi*3*math.exp(-lamb)
        return h


index = count()


# lists to store x and y axis points
t_val, y_val = [], []


# animation function
def animate(i):
    t = 0.01 * next(index)
    x = Point(t)
    y = x.fun()
    t_val.append(t)
    y_val.append(y)

    plt.cla()
    plt.plot(t_val,y_val)


ani = animation.FuncAnimation(plt.gcf(), animate,
                              interval=1)
plt.tight_layout()
plt.show()
