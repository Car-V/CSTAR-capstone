import pyaudio
import numpy as np
import wave

class Audio_Collector:
    def __init__(self, rate = 48000, file = "recording.wav"):
        self.rate = rate
        self.filename = file

    def collect_samples(self, time_recording):     #collects audio signals from given time
        print("\nRecording...")

        FORMAT = pyaudio.paInt32 #32-bit input
        CHANNELS = 2    #two ADC channels
        RATE = Audio_Collector.rate    #rate must be at least twice as large as highest frequency signal
        CHUNK = 1024    #frames per buffer --> more ideal to use 512, but have to test pi capabilities 
        filename = Audio_Collector.filename

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


        return [samples, self.rate]
    
    def __main__():
        user_input = input().lower()
        collector = Audio_Collector.collect_samples(user_input)

    __main__()