import numpy as np
import matplotlib.pyplot as plt
#from scipy.io import wavfile
import soundfile as sf
from numpy.fft import fft, fftfreq
from Digitize_Filter import DigitizeFilter


class AudioManager:
    def __init__(self, packetNo, start_pos, mid_pos, end_pos, threshhold, delam_range_start = 1200, delam_range_end = 2000, rate = 48000):
        self.rate = rate        #sets sampling rate of collected audio
        self.packetNo = packetNo          #sets packet number for tracking
        self.start_pos = start_pos      #start position for audio recording
        self.mid_pos = mid_pos      #middle position for audio recording
        self.end_pos = end_pos      #end position for audio recording
        self.threshhold = threshhold    #sets threshhold magnitude for delam detection
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
    
    """this part doesn't make sense, leaving in for reference purposes"""
    """
    def concat_with_position(self, samples):

        # calculates increments of positional data for each audio frequency
        start_to_mid = self.mid_pos - self.start_pos 
        first_half_increments = start_to_mid / (len(samples)/2.0)
        mid_to_end = self.end_pos - self.mid_pos
        second_half_increments = mid_to_end / (len(samples)/2.0)

        half_samples = int(len(samples)/2)# Adjust based on the resolution you need
        cur_pos = self.start_pos

        #increment through samples and set the second column to positional data 
        for i in samples[0:half_samples]: 
            i[1] = cur_pos
            cur_pos += first_half_increments

        for i in samples[half_samples-1:]:
            i[1] = cur_pos
            cur_pos += second_half_increments """

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
        
    def check_against_threshhold(self, fft):
        sum_mag = 0  
        freq = fftfreq(len(fft), 1/self.rate)          #generate frequencies for FFT

        low_index = self.delam_range_start
        high_index = 0

        # Assuming f is a numpy array of frequencies
        for i in range(1, len(freq)):  # Start from index 1 to avoid f[i-1] out of bounds
            if ((freq[i] >= self.delam_range_start) and (freq[i-1] <= self.delam_range_start)):
                low_index = i
            if (freq[i] > self.delam_range_start and freq[i] <= self.delam_range_end and freq[i+1] >=self.delam_range_end):
                high_index = i
                
        print(low_index)
        print(high_index)
        # Loop through the specified FFT range
        for i in range(low_index, high_index + 1):  # Include high_index
            sum_mag += fft[i]  # Sum the FFT magnitudes

        # Compute the average magnitude
        avg_mag = sum_mag / (high_index - low_index)

        # Check if it exceeds the threshold
        delam = avg_mag > 0.001

        # Display results
        print(f"Delam: {delam}")
        print(f"Magnitude: {avg_mag}")

                  
        return [delam, avg_mag]
    
    
        
    

    

    
        

        


        



def __main__():
    audioGroup = AudioManager(1, 0, 5, 10, 0.0025)

    samples = audioGroup.convert_from_wav("DELAM_floor_3_single_6_alewife.wav")  #convert wav to sample array
    samples = np.array(samples)
    samples = audioGroup.apply_filters(samples)
   # audioGroup.concat_with_position(samples)

    """ calculate positional data and increments needed to match sample size
    start_to_mid = audioGroup.mid_pos - audioGroup.start_pos 
    first_half_increments = start_to_mid / (len(samples)/2.0)
    mid_to_end = audioGroup.end_pos - audioGroup.mid_pos
    second_half_increments = mid_to_end / (len(samples)/2.0)

    half_samples = int(len(samples)/2)# Adjust based on the resolution you need
    cur_pos = audioGroup.start_pos """

    """ increment through samples and set the second column to positional data
    for i in samples[0:half_samples]: 
        i[1] = cur_pos
        cur_pos += first_half_increments

    for i in samples[half_samples-1:]:
        i[1] = cur_pos
        cur_pos += second_half_increments
"""
    audioGroup.convert_to_fft(samples)
    audioGroup.check_against_threshhold(samples)

    # Convert the list to a numpy array for FFT processing
    #samples = np.array(samples)

    # Number of samples to collect
    num_samples = len(samples)# Adjust based on the resolution you need
    sampling_rate = 48000  # Samples per second (1 kHz)

    



    
    
    
__main__()        
        

        

    

    




