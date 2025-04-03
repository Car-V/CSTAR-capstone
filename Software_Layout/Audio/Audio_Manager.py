import numpy as np
import matplotlib.pyplot as plt
#from scipy.io import wavfile
import soundfile as sf
from numpy.fft import fft, fftfreq
from scipy.signal import stft
from Digitize_Filter import DigitizeFilter


class AudioManager:
    def __init__(self, packetNo, start_pos, end_pos, delam_range_start = 1000, delam_range_end = 5000, rate = 48000):
        self.rate = rate        #sets sampling rate of collected audio
        self.packetNo = packetNo          #sets packet number for tracking
        self.start_pos = start_pos      #start position for audio recording
        self.end_pos = end_pos      #end position for audio recording
        self.delam_range_start = delam_range_start      #sets start of range being evaluated for delamination
        self.delam_range_end = delam_range_end          #sets end of range being evaluated for delamination
        self.digi_filter = DigitizeFilter()

    def convert_from_wav(self, file):

        filepath = 'C:\\Users\\kayla\\Documents\\MATLAB\\' #need to make this relative, just for now
        filepath = filepath + file

        try:
            samples, sample_rate = sf.read(filepath)
            self.rate = sample_rate
        except FileNotFoundError:
            print(f"Error: The file '{filepath}' was not found.")
        except Exception as e:
            print(f"Error occurred: {e}")


        return samples
    
    def apply_filters(self, samples):
        samples_filtered = self.digi_filter.bandpass_butterworth_filter(samples, self.delam_range_start, self.delam_range_end, self.rate, 5)
        return samples_filtered
    


    def convert_to_fft(self, samples):
        n = len(samples)        #number of samples
        fft_result = np.fft.fft(samples, axis = 0)       #calculate fft
        freq = fftfreq(n, 1/self.rate)          #generate frequencies for FFT
        normalized_fft = abs(fft_result/n)       #normalize magnitude
        single_normalized_fft = 2* (normalized_fft[0:int((n/2)+1)])     #make data single-sided and double amplitude
        
        #plotting for visual test
        plt.plot(freq[0:int((n/2)+1)], single_normalized_fft, 'b')
        plt.title("Recorded FFT")
        plt.xlabel("Frequency (Hz)")
        plt.ylabel("Magnitude")
        plt.xlim([0, self.rate/2])    
        plt.show()
        
        return [single_normalized_fft, freq]
        

    def stft_plot(self, samples):

        if len(samples.shape)>1:
            samples = np.mean(samples, axis=1)

        print(f"Samples shape: {samples.shape}")
        print(f"Samples min/max: {np.min(samples)}, {np.max(samples)}")


        f, t, Zxx = stft(samples, self.rate, nperseg=512)

        plt.figure(figsize=(10, 6))
        plt.pcolormesh(t, f, np.abs(Zxx), shading='gouraud')
        plt.title('Short-Time Fourier Transform (STFT)')
        plt.ylabel('Frequency [Hz]')
        plt.xlabel('Time [sec]')
        plt.colorbar(label='Magnitude')
        plt.show()

    def return_mag(self, fft, freq):
        sum_mag = 0
        low_index = self.delam_range_start
        high_index = 0

        # Assuming f is a numpy array of frequencies
        for i in range(1, len(freq)):  # Start from index 1 to avoid f[i-1] out of bounds
            if ((freq[i] >= self.delam_range_start) and (freq[i-1] <= self.delam_range_start)):
                low_index = i
            if (freq[i] > self.delam_range_start and freq[i] <= self.delam_range_end and freq[i+1] >=self.delam_range_end):
                high_index = i
                
        # Loop through the specified FFT range
        for i in range(low_index, high_index + 1):  # Include high_index
            if (type(fft[i]) == list):
                sum_mag += (fft[i][0] + fft[i][1])/2  # Sum the FFT magnitudes average
            else:
                sum_mag += fft[i]
        

             
        return sum_mag
    
    
        
    

    

    
        

        


        



def __main__():
    audioGroup = AudioManager(1, 0, 10)

    samples = audioGroup.convert_from_wav("LAM_floor_4_single_1_alewife.wav")  #convert wav to sample array
    samples = np.array(samples)
    #samples = audioGroup.apply_filters(samples)

    [samples_fft, freq] = audioGroup.convert_to_fft(samples)

    audioGroup.stft_plot(samples)

    print(audioGroup.return_mag(samples_fft, freq))

    # Convert the list to a numpy array for FFT processing
    #samples = np.array(samples)

    # Number of samples to collect
    num_samples = len(samples)# Adjust based on the resolution you need
    sampling_rate = 48000  # Samples per second (1 kHz)

    



    
    
    
__main__()        
        

        

    

    




