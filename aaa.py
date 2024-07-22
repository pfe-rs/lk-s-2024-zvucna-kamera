from matplotlib import pyplot
from matplotlib.animation import FuncAnimation

import serial
import numpy as np

x_data, y_data = [], []

with serial.Serial('COM9', 230400, timeout=4) as ser:

    x_data, y_data = [], []

    figure = pyplot.figure()
    line1, = pyplot.plot(x_data, y_data, '-')
    line2, = pyplot.plot(x_data, y_data, '-.')    

    def update(frame):

        l = ser.readline()  

        try:
            data = l.decode("utf-8").split(',')
            data1 = []
            data2 = []
            for dat in data[:-1]:
                c1, c2 = dat.split(' ')
                data1.append(int(c1))
                data2.append(int(c2))
                
            num_samples = len(data1)
            sample_rate = 48000  
            time_values = np.linspace(0, num_samples / sample_rate, num_samples)


            line1.set_data(time_values, data1)
            line2.set_data(time_values, data2)
            figure.gca().relim()
            figure.gca().autoscale_view()
            return line1, line2,
        except Exception as e:
            print(e)
            return line1, line2,

    animation = FuncAnimation(figure, update, interval=4)

    pyplot.show()


        

