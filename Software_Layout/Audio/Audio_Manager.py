import RPi.GPIO as GPIO
import pyaudio
import numpy as np


class AudioManager:
    def __init__(self, omni_mic, uni_mic, filter):
        self.lrck = 19      #lrck pins for both mics connected to GPIO 19
        self.bick = 18      #bick pins for both mics connected to GPIO 18
        self.sdto = 29      #sdto with tdm signal from both mics connected to GPIO 29
        
        GPIO.setmode(GPIO.BCM) 

        GPIO.setup(self.lrck, GPIO.OUT)     #lrck output clk signal
        GPIO.setup(self.bick, GPIO.OUT)     #bick output clk signal
        GPIO.setup(self.sdto, GPIO.IN)      #sdto with tdm signal input
        

    def collect_sample(time_recording):     #collects audio signals from given time
        FORMAT = pyaudio.paInt32 #format of TDM signal is 24 bits
        CHANNELS = 4    #two ADC channels
        RATE =10000     #rate must be at least twice as large as highest frequency signal
        CHUNK = 1024    #frames per buffer

        p = pyaudio.PyAudio()

        #open stream to start collecting audio samples
        stream = p.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        frames_per_buffer=CHUNK)
        
        frames_raw = []     #raw signals without demultiplexing
        uni_samples = []    #demultiplexed uni samples 
        omni_samples = []   #demultiplexed omni samples

        for i in range(0, int(RATE/CHUNK * time_recording)):
            data = stream.read(CHUNK)
            frames_raw.append(data)     #append raw data
            samples = np.frombuffer(data, dtype=np.int32)     #reads 32 bit slots
            uni_samples = samples[::4].astype(np.float32)     #uni signal capture
            omni_samples = samples[1::4].astype(np.float32)   #omni signal capture

        stream.stop_stream()
        stream.close()
        p.terminate()

        samples = {uni_samples, omni_samples}

        return samples
    


        
        

        

    

    




