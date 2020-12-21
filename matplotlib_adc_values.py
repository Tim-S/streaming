import numpy as np
import matplotlib
from matplotlib import pyplot, animation
import socket
matplotlib.use("TkAgg")

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('pi0.local', 2000))

fifo = s.makefile('r')

# fifo_name = "/tmp/adc.fifo"
npoints = 256
interval = 100
xlim = (0, 1)
ylim = (0, 5)

# fifo = open(fifo_name, "r")
fig = pyplot.figure()
ax = pyplot.axes(xlim=xlim, ylim=ylim)
line, = ax.plot([], [], lw=1)


def init():
    line.set_data([], [])
    return line,


def animate(i):
    y = np.fromstring(fifo.readline(), sep=',')
    x = np.linspace(0, 1, len(y))
    # y = np.fromstring(fifo.readline(), sep=',')
    # line.set_data(x, y)
    line.set_data(x, y)
    return line,


anim = animation.FuncAnimation(fig, animate, init_func=init, interval=interval, blit=True)
pyplot.show()


