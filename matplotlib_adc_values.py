import numpy as np
import matplotlib
from matplotlib import pyplot, animation
import socket
from scipy import fftpack
matplotlib.use("TkAgg")

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('pi0.local', 4950))

fifo = s.makefile('r')

# fifo_name = "/tmp/adc.fifo"
npoints = 1024
interval = 20
xlim = (0, npoints)
ylim = (0, 3.5)

# fifo = open(fifo_name, "r")


fig, axs = pyplot.subplots(2, 1)
axs[0].set_xlim(xlim)
axs[0].set_xlabel("Time (Sample#)")
axs[0].set_ylim(ylim)
axs[0].set_ylabel("Amplitude (V)")
line, = axs[0].plot([], [], lw=1)

axs[1].set_xlim((0, npoints//2))
axs[1].set_xlabel("Doppler Bin")
axs[1].set_ylim(ylim)
axs[1].set_ylabel("Magnitude")

fig.tight_layout()

line2, = axs[1].plot([], [], lw=1)


def init():
    line.set_data([], [])
    return line,


def animate(i):
    y = np.fromstring(fifo.readline(), sep=',')[1:]
    npoints = len(y)# Number of sample-points
    x = np.linspace(0, npoints, npoints)
    line.set_data(x, y)

    xf = np.linspace(0, npoints//2, npoints//2) # display only half the fft size, since the second half is a mirror of the first half, if x is real but not complex
    yf = fftpack.fft(y)
    line2.set_data(xf, np.abs(yf)[:npoints//2])

    return line, line2,


anim = animation.FuncAnimation(fig, animate, init_func=init, interval=interval, blit=True)
pyplot.show()


