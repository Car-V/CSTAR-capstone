import RPi.GPIO as GPIO
import numpy as np
import matplotlib.pyplot as plt
import time

GPIO.setmode(GPIO.BCM)

GPIO.setup(18,GPIO.OUT)
GPIO.setup(19,GPIO.OUT)
GPIO.setup(4,GPIO.OUT)
GPIO.setup(20,GPIO.IN)

pwm = GPIO.PWM(18,1000)
pwm2 = GPIO.PWM(19,1000)
pwm3 = GPIO.PWM(4,1000)
pwm.start(50)
pwm2.start(50)
pwm3.start(50)

input_pin = 20


# Number of samples to collect
num_samples = 40000  # Adjust based on the resolution you need
sampling_rate = 20000  # Samples per second (1 kHz)

# Create an empty list to store the digital values (0 or 1)
samples = []

# Collect the data
print(f"Collecting {num_samples} samples...")
for _ in range(num_samples):
    samples.append(GPIO.input(input_pin))
    time.sleep(1.0 / sampling_rate)

# Convert the list to a numpy array for FFT processing
samples = np.array(samples)

# Perform FFT on the collected data
fft_result = np.fft.fft(samples)
fft_freq = np.fft.fftfreq(num_samples, 1.0 / sampling_rate)

# Plot the FFT result (only the positive frequencies)
positive_freqs = fft_freq[:num_samples // 2]
positive_fft = np.abs(fft_result[:num_samples // 2])

# Plot the frequency spectrum
plt.figure(figsize=(10, 6))
plt.plot(positive_freqs, positive_fft)
plt.title("FFT of GPIO Input Signal")
plt.xlabel("Frequency (Hz)")
plt.ylabel("Magnitude")
plt.grid(True)
plt.show()

# Clean up GPIO setup

pwm.stop()
pwm2.stop()
pwm3.stop()
GPIO.cleanup()