from matplotlib import pyplot
from matplotlib.animation import FuncAnimation
# from pyargus.directionEstimation import *
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
speed_of_sound = 343  # Speed of sound in m/s
sample_rate = 24000 # Hz


num_channels = 9
num_samples = 4096
num_bytes = num_channels * num_samples

with serial.Serial('COM10', 10000000, timeout=0.01) as ser:

    x_data, y_data = [], []

    fig, (ax1, ax2, ax3) = pyplot.subplots(3, 1)
    lines = []
    for i in range(num_channels):
        lines.append(ax1.plot(x_data, y_data, '-')[0])
    

    def update(frame):
        try:
            
            ser.timeout = 0.01
            c = 0
            while not c:
                c = ser.read(1)

            ser.timeout = 0.15
            data = ser.read(num_bytes - 1)


            data = c + data


            if len(data) < num_bytes:
                return
            data = [int(x) for x in data]
            
            datas=[]
            for i in range(num_channels):

                datas.append(data[i*num_samples:(i+1)*num_samples])

            time_values = np.linspace(0, num_samples / sample_rate, num_samples)

            for i in range(num_channels):
                datas[i] = np.array(datas[i])
                datas[i] = 2 * (datas[i] / 255) - 1
            # data1 = np.array(data1)
            # data2 = np.array(data2)
            # data1 = 2 * (data1 / 255) - 1
            # data2 = 2 * (data2 / 255) - 1

            for i in range(num_channels):
                lines[i].set_data(time_values, datas[i])
            ax1.relim()
            ax1.autoscale_view()


            # pretvaranje signala u sinusoidu najizrazenije frekvencije
            # d1_fft_peak = (abs(np.fft.fft(data[0]))/num_samples)**2
            # d2_fft_peak = (abs(np.fft.fft(data[1]))/num_samples)**2

            # d1_fft = np.fft.fft(data[0])
            # d2_fft = np.fft.fft(data[1])

            # magnitudes = np.abs(d1_fft_peak)
            # sig_index = np.argmax(magnitudes[1:])


            return

            signal_freq = (sample_rate/num_samples)*sig_index  # Hz
            wavelength = speed_of_sound / signal_freq  # Wavelength of the signal
            spacing = spacing/wavelength

            t = np.arange(num_samples) / sample_rate
            signal = np.sin(2 * np.pi * signal_freq * t)

            phase_angle = np.array([np.angle(d1_fft[sig_index]), np.angle(d2_fft[sig_index])])
            received_signals = np.zeros((M, num_samples), dtype=complex)

            for m in range(M):
                phase_shift = np.exp(-1j * phase_angle[m])
                received_signals[m, :] = signal * phase_shift


            line3.set_data(np.fft.fftfreq(len(data1)) * sample_rate, d1_fft_peak)
            line4.set_data(np.fft.fftfreq(len(data2)) * sample_rate, d2_fft_peak)
            ax2.relim()
            ax2.autoscale_view()
            ax2.set_xlim((0, 5000))

            R = np.cov(received_signals)
            #R = corr_matrix_estimate(received_signals.T, imp="mem_eff")


            angle_grid = np.linspace(-90, 90, 181)
            steering_vectors = generate_steering_vectors(M, spacing, angle_grid, wavelength)
            #ula_scanning_vectors = gen_ula_scanning_vectors([0, spacing],angle_grid)
            #Bartlett = DOA_Bartlett(R,steering_vectors)
            mvdr_output = mvdr_beamforming(R, steering_vectors)
            #DOA_plot(Bartlett, angle_grid, log_scale_min = -50)
            line5.set_data(angle_grid, mvdr_output)
            ax3.relim()
            ax3.autoscale_view()

        except KeyboardInterrupt:
            exit()
        # except Exception as e:
        #     print(e)
        #     return
    animation = FuncAnimation(fig, update, interval=0.25)

    pyplot.show()
