import numpy as np
from matplotlib import pyplot
import math

def delay_and_sum(signal, sample_rate, num_samples, d):
    u = 343 # [m/s]
    dist_per_sample = u/sample_rate
    count = int((3*d) / dist_per_sample)
    _max = -100000000
    tdoa = 0
    t1_list = []
    t2_list = []
    for i in range(-count, count):
        temp1 = np.dot(signal[0], np.roll(signal[1], i))#np.concatenate((np.roll(signal[1], i)[i:], np.zeros(i,dtype = int))))
        if temp1 > _max:# and -1 <= (u*(i/sample_rate))/d <=1:
            _max = temp1
            tdoa = i
        t1_list.append(temp1)

    tdoa = tdoa/sample_rate
    power = 180*np.acos((u*tdoa)/d)/math.pi
    return power

x_data, y_data = [], []



# Parameters
signal_freq = 440 # Hz
sampling_rate = 48000  # Hz
N = 2**12  # Number of samples
M = 2  # Number of sensors (microphones)
spacing = 0.058  # Spacing between microphones in meters
speed_of_sound = 343  # Speed of sound in m/s
wavelength = speed_of_sound / signal_freq  # Wavelength of the signal
r = 2 # m
th = 70 # deg
PI = math.pi
A = 1
c = speed_of_sound          
th_rad = (th / 180) * PI
d1 = np.sqrt(r*r - r*np.cos(th_rad) + 0.25)
d2 = np.sqrt(r*r + r*np.cos(th_rad) + 0.25)
t = 5

#t = np.arange(N) / sampling_rate
#signal = np.sin(2 * np.pi * signal_freq * t + 1)
fig, (ax1, ax2, ax3) = pyplot.subplots(3, 1)
line1, = ax1.plot(x_data, y_data, '-')
line2, = ax1.plot(x_data, y_data, '-.')
line3, = ax2.plot(x_data, y_data, '-')
line4, = ax2.plot(x_data, y_data, '-.')
line5, = ax3.plot(x_data, y_data, '-')
line6, = ax3.plot(x_data, y_data, '-.')

angles = []
for j in range(30, 120):
    th = j
    th_rad = (th / 180) * PI
    d1 = r
    d2 = np.sqrt((r*np.sin(th_rad))**2 + (r*np.cos(th_rad) - spacing)**2)         
    # Simuliraj signale
    received_signals = np.zeros((M, N), dtype=float)

    time_inc = 1/sampling_rate
    for i in range(N):
        received_signals[0][i] = np.sin(2 * PI * signal_freq * (t - d1/c))
        received_signals[1][i] = np.sin(2 * PI * signal_freq * (t - d2/c))
        t += time_inc

    

    #noise = np.random.normal(0,np.sqrt(10**-1),(M,N))
    #received_signals = received_signals# + noise
    
    angle = delay_and_sum(received_signals, sampling_rate, N, spacing)
    print(th, "  ", angle)
    angles.append(angle)

time_values = np.linspace(0, N / sampling_rate, N)
samp = [x for x in range(N)]
line1.set_data(samp, received_signals[0])
line2.set_data(samp, received_signals[1])
ax1.relim()
ax1.autoscale_view()

cnt = [x for x in range(len(angles))]
line3.set_data(cnt, angles)
ax2.relim()
ax2.autoscale_view()
"""
cnt = [x for x in range(max(len(angle_in), len(angle_out)))]
line5.set_data(cnt, angle_in)
line6.set_data(cnt, angle_out)
ax3.relim()
ax3.autoscale_view()"""
pyplot.show()

