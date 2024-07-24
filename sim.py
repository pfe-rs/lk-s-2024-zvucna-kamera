import numpy as np
import matplotlib.pyplot as plt

# Parameters
signal_freq = 1  # Hz
sampling_rate = 10000  # Hz
N = 1024  # Number of samples
M = 2  # Number of sensors (microphones)
spacing = 0.02  # Spacing between microphones in meters
speed_of_sound = 343  # Speed of sound in m/s
wavelength = speed_of_sound / signal_freq  # Wavelength of the signal

# Generate a signal
t = np.arange(N) / sampling_rate
signal = np.sin(2 * np.pi * signal_freq * t + 1)

# Simulate received signals at different sensors
received_signals = np.zeros((M, N), dtype=complex)
theta_deg = 30  # Angle of arrival in degrees
theta_rad = np.deg2rad(theta_deg)

# Calculate phase shifts for each sensor
for m in range(M):
    phase_shift = np.exp(-1j * 2 * np.pi * spacing * m * np.sin(theta_rad) / wavelength)
    received_signals[m, :] = signal * phase_shift

angle_grid = np.linspace(-90, 90, 181)
steering_vectors = generate_steering_vectors(M, spacing, angle_grid, wavelength)


mvdr_output = mvdr_beamforming(R, steering_vectors)

# Plot the MVDR beamforming output
plt.figure(figsize=(10, 6))
plt.plot(angle_grid, mvdr_output)
plt.title('Beamforming Output (MVDR)')
plt.xlabel('Angle (degrees)')
plt.ylabel('Output power')
plt.grid()
plt.show()
