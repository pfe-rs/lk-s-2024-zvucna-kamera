from matplotlib import pyplot
from matplotlib.animation import FuncAnimation

import serial
import numpy as np

# Perform MVDR beamforming
def mvdr_beamforming(R, steering_vectors):
    num_angles = steering_vectors.shape[1]
    output_power = np.zeros(num_angles)

    for i in range(num_angles):
        a_theta = steering_vectors[:, i]
        a_theta_H = np.conjugate(a_theta).T
        numerator = np.dot(a_theta_H, a_theta)
        denominator = np.dot(np.dot(a_theta_H, np.linalg.inv(R)), a_theta)
        output_power[i] = np.abs(numerator / denominator)

    return output_power
# Generate steering vectors manually
def generate_steering_vectors(M, d, angles, wavelength):
    steering_vectors = np.zeros((M, len(angles)), dtype=complex)
    for idx, angle in enumerate(angles):
        theta = np.deg2rad(angle)
        for m in range(M):
            steering_vectors[m, idx] = np.exp(-1j * 2 * np.pi * d * m * np.sin(theta) / wavelength)
    return steering_vectors

x_data, y_data = [], []
M = 2
spacing = 0.018
speed_of_sound = 343  # Speed of sound in m/s


with serial.Serial('/dev/ttyUSB0', 230400, timeout=4) as ser:

    x_data, y_data = [], []

    fig, (ax1, ax2, ax3) = pyplot.subplots(3, 1)
    line1, = ax1.plot(x_data, y_data, '-')
    line2, = ax1.plot(x_data, y_data, '-.')

    line3, = ax2.plot(x_data, y_data, '-')
    line4, = ax2.plot(x_data, y_data, '-.')

    line5, = ax3.plot(x_data, y_data, '-')

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
            sample_rate = 48000 # Hz
            time_values = np.linspace(0, num_samples / sample_rate, num_samples)

            line1.set_data(time_values, data1)
            line2.set_data(time_values, data2)
            ax1.relim()
            ax1.autoscale_view()

            data1 = np.array(data1)
            data2 = np.array(data2)
            data1 = 2 * (data1 / data1.max()) - 1
            data2 = 2 * (data2 / data2.max()) - 1
            print(data1, data2)
            # pretvaranje signala u sinusoidu najizrazenije frekvencije
            d1_fft = np.fft.fft(data1)
            d2_fft = np.fft.fft(data2)

            magnitudes = np.abs(d1_fft)
            sig_index = np.argmax(magnitudes[1:])

            signal_freq = (sample_rate/num_samples)*sig_index  # Hz
            wavelength = speed_of_sound / signal_freq  # Wavelength of the signal

            t = np.arange(num_samples) / sample_rate
            signal = np.sin(2 * np.pi * signal_freq * t)

            phase_angle = np.array([np.angle(d1_fft[sig_index]), np.angle(d2_fft[sig_index])])
            received_signals = np.zeros((M, num_samples), dtype=complex)
            print(signal_freqf)
            for m in range(M):
                phase_shift = np.exp(-1j * phase_angle[m])
                received_signals[m, :] = signal * phase_shift

            data1_fft = abs(np.fft.fft(data1))
            data2_fft = abs(np.fft.fft(data2))


            line3.set_data(np.fft.fftfreq(len(data1)) * sample_rate, data1_fft)
            line4.set_data(np.fft.fftfreq(len(data2)) * sample_rate, data2_fft)
            ax2.relim()
            ax2.autoscale_view()

            R = np.cov(received_signals)



            angle_grid = np.linspace(-90, 90, 181)
            steering_vectors = generate_steering_vectors(M, spacing, angle_grid, wavelength)


            mvdr_output = mvdr_beamforming(R, steering_vectors)
            line5.set_data(angle_grid, mvdr_output)
            ax3.relim()
            ax3.autoscale_view()

            return line1, line2,
        except Exception as e:
            print(e)
            return line1, line2,

    animation = FuncAnimation(fig, update, interval=5)

    pyplot.show()
