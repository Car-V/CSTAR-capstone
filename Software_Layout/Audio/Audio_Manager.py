import RPi.GPIO as GPIO
import pyaudio
import numpy as np
import matplotlib.pyplot as plt
import wave


class AudioManager:
    def __init__(self, omni_mic, uni_mic, filter):
        self.lrck = 19      #lrck pins for both mics connected to GPIO 19
        self.bick = 18      #bick pins for both mics connected to GPIO 18
        self.sdto = 29      #sdto with tdm signal from both mics connected to GPIO 29
        
        GPIO.setmode(GPIO.BCM) 

        GPIO.setup(self.lrck, GPIO.OUT)     #lrck output clk signal
        GPIO.setup(self.bick, GPIO.OUT)     #bick output clk signal
        GPIO.setup(self.sdto, GPIO.IN)      #sdto with tdm signal input
        

    def collect_samples(time_recording):     #collects audio signals from given time
        print("\nRecording...")

        FORMAT = pyaudio.paUInt8 #format of TDM signal is 24 bits
        CHANNELS = 2    #two ADC channels
        RATE = 20000     #rate must be at least twice as large as highest frequency signal
        CHUNK = 1024    #frames per buffer
        filename = "recordedTest.wav"

        p = pyaudio.PyAudio()

        #open stream to start collecting audio samples
        stream = p.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        frames_per_buffer=CHUNK)
        
        frames_raw = []     #raw signals without demultiplexing
        samples = []

        for i in range(0, int(RATE/CHUNK * time_recording)):
            data = stream.read(CHUNK)
            frames_raw.append(data)     #append raw data
            samples = np.frombuffer(data, dtype=np.int32)     #reads 32 bit slots


        stream.stop_stream()
        stream.close()
        p.terminate()

        waveFile = wave.open(filename, 'wb')
        waveFile.setnchannels(CHANNELS)
        waveFile.setsampwidth(p.get_sample_size(FORMAT))
        waveFile.setframerate(RATE)
        waveFile.writeframes(b''.join(frames_raw))
        waveFile.close()


        return samples


def __main__():
    print("\nStarting...")
    samples = AudioManager.collect_samples(10)

    # Convert the list to a numpy array for FFT processing
    samples = np.array(samples)

    # Number of samples to collect
    num_samples = len(samples)# Adjust based on the resolution you need
    sampling_rate = 20000  # Samples per second (1 kHz)

    # Perform FFT on the collected data
    fft_result = np.fft.fft(samples)
    fft_freq = np.fft.fftfreq(num_samples, 1.0 / sampling_rate)

    # Plot the FFT result (only the positive frequencies)
    positive_freqs = fft_freq[:num_samples // 2]
    positive_fft = np.abs(fft_result[:num_samples // 2])

    print("\nPlotting...")

    # Plot the frequency spectrum
    plt.figure(figsize=(10, 6))
    plt.plot(positive_freqs, positive_fft)
    plt.title("FFT of USB-Mic Input Signal")
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Magnitude")
    plt.grid(True)
    plt.show()

    # Clean up GPIO setup

    GPIO.cleanup()

    
__main__()        
        

        

    

    




