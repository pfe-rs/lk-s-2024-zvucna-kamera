from matplotlib import pyplot
from matplotlib.animation import FuncAnimation
from pyargus.directionEstimation import *
import serial
import traceback
import numpy as np
import math

def delay_and_sum(signal, angle_grid, sample_rate, num_samples, d):
    u = 343 # [m/s]
    dist_per_sample = u/sample_rate
    count = int((10*d) / dist_per_sample)
    _max = -100000000
    tdoa = 0
    for i in range(count):
        temp1 = np.dot(signal[0], np.concatenate((np.roll(signal[1], i)[i:], np.zeros(i,dtype = int))))
        if i == 0:
            temp2 = -1000000
        else:
            temp2 = np.dot(signal[0], np.concatenate((np.zeros(i,dtype = int), np.roll(signal[1], -i)[:-i])))
        
        if temp1 > _max:
            _max = temp1
            tdoa = i
        if temp2 > _max:
            _max = temp1
            tdoa = -i
    tdoa = tdoa/sample_rate
    power = 180*math.acos((u*tdoa)/d)/math.pi-90
    return power

x_data, y_data = [], []
M = 2
speed_of_sound = 343  # Speed of sound in m/s
sample_rate = 24000 # Hz


with serial.Serial('/dev/ttyACM0', 10000000, timeout=0.01) as ser:

    x_data, y_data = [], []

    fig, (ax1, ax2, ax3) = pyplot.subplots(3, 1)
    line1, = ax1.plot(x_data, y_data, '-')
    line2, = ax1.plot(x_data, y_data, '-.')

    line3, = ax2.plot(x_data, y_data, '-')

    def update(frame):
        try:
            spacing = 0.058
            num_samples = 8192

            ser.timeout = 0.01
            c = 0
            while not c:
                c = ser.read(1)

            ser.timeout = 0.2
            data = ser.read(16384 - 1)

            data = data + c

            if len(data) < 16384:
                return
            data = [int(x) for x in data]
            data1=data[0:(16384//2)]
            data2=data[16384//2:16384]

            time_values = np.linspace(0, num_samples / sample_rate, num_samples)

            data1 = 2 * (np.array(data1) / 255) - 1
            data2 = 2 * (np.array(data2) / 255) - 1

            line1.set_data(time_values, data1)
            line2.set_data(time_values, data2)
            ax1.relim()
            ax1.autoscale_view()

            signal = np.vstack((data1, data2))
            angle_grid = np.linspace(-90, 90, 181)
            power = delay_and_sum(signal, angle_grid, sample_rate, num_samples, spacing)
            print(power)
            #line3.set_data(angle_grid, power)
            #ax2.relim()
            #ax2.autoscale_view()

        except KeyboardInterrupt:
            exit()
        except Exception as e:
            print(e)
            traceback.print_exc()   
            return
    animation = FuncAnimation(fig, update, interval=0.2)

    pyplot.show()
