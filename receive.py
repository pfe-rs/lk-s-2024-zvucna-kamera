import serial
import numpy as np

x_data, y_data = [], []

with serial.Serial('/dev/ttyACM0', 10000000, timeout=0.25) as ser:
    while True:
        data = ser.read(16384)
        if len(data) < 16384:
            continue
        data = [int(x) for x in data]
        #print(data)
        print(len(data))
        data1=data[0:(16384//2)-1]
        data2=data[16384//2:16384]

        # num_samples = len(data2)
        # print(num_samples)
