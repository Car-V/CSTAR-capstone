import numpy as np
import matplotlib.pyplot as plt
#from scipy.io import wavfile
import soundfile as sf


class AudioManager:
    def __init__(self, packetNo, start_pos, mid_pos, end_pos, threshhold, samples = [], delam_range_start = 2000, delam_range_end = 5000, rate = 48000):
        self.rate = rate        #sets sampling rate of collected audio
        self.samples = samples      #sets samples array
        self.packetNo = packetNo          #sets packet number for tracking
        self.start_pos = start_pos      #start position for audio recording
        self.mid_pos = mid_pos      #middle position for audio recording
        self.end_pos = end_pos      #end position for audio recording
        self.threshhold = threshhold    #sets threshhold magnitude for delam detection
        self.delam_range_start = delam_range_start      #sets start of range being evaluated for delamination
        self.delam_range_end = delam_range_end          #sets end of range being evaluated for delamination

    def convert_from_wav(file):

        filepath = 'C:\\Users\\kayla\\Documents\\MATLAB\\' #need to make this relative, just for now
        filepath = filepath + file

        try:
            wav_array, sample_rate = sf.read(filepath)
            AudioManager.rate = sample_rate
        except FileNotFoundError:
            print(f"Error: The file '{filepath}' was not found.")
        except Exception as e:
            print(f"Error occurred: {e}")

        return wav_array

        



def __main__():
    audioGroup = AudioManager(1, 0, 5, 10, 1000)

    samples = AudioManager.convert_from_wav("recordedTest.wav")  #convert wav to sample array

    """ calculate positional data and increments needed to match sample size"""
    start_to_mid = audioGroup.mid_pos - audioGroup.start_pos 
    first_half_increments = start_to_mid / (len(samples)/2.0)
    mid_to_end = audioGroup.end_pos - audioGroup.mid_pos
    second_half_increments = mid_to_end / (len(samples)/2.0)

    half_samples = int(len(samples)/2)# Adjust based on the resolution you need
    cur_pos = audioGroup.start_pos

    """ increment through samples and set the second column to positional data """
    for i in samples[0:half_samples]: 
        i[1] = cur_pos
        cur_pos += first_half_increments

    for i in samples[half_samples:]:
        i[1] = cur_pos
        cur_pos += second_half_increments

    end = samples[int(len(samples))-1]
    mid_1 = samples[half_samples - 2]
    mid_2 = samples[half_samples-1]
    mid_3 = samples[half_samples]

    # Convert the list to a numpy array for FFT processing
    #samples = np.array(samples)

    # Number of samples to collect
    num_samples = len(samples)# Adjust based on the resolution you need
    sampling_rate = 48000  # Samples per second (1 kHz)

    



    
    
    
__main__()        
        

        

    

    




