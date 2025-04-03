import numpy as np
import matplotlib.pyplot as plt
import soundfile as sf
from numpy.fft import fft, fftfreq
from scipy.signal import stft
from scipy.signal import butter, lfilter

def audio_manager():
    filepath = filepath

    try:
        samples, sample_rate = sf.read(filepath)
    except FileNotFoundError:
        print(f"Error: The file '{filepath}' was not found.")
    except Exception as e:
        print(f"Error occurred: {e}")
        
    fs = 48000
    lowcut = 500        #delam start freq range
    highcut = 5000      #delam end freq range
    
    order = 5
        
    nyq = 0.5 * fs      # nyquist frequency
    band_low = lowcut / nyq
    band_high = highcut / nyq

    b, a = butter(order, [band_low, band_high], btype='band')      #butter imported from scipy
    samples = lfilter(b, a, samples)     #apply filter to signal
    
    n = len(samples)        #number of samples
    fft_result = np.fft.fft(samples, axis = 0)       #calculate fft
    freq = fftfreq(n, 1/fs)          #generate frequencies for FFT
    normalized_fft = abs(fft_result/n)       #normalize magnitude
    single_normalized_fft = 2* (normalized_fft[0:int((n/2)+1)])     #make data single-sided and double amplitude
    
    single_normalized_fft = np.array(single_normalized_fft)
    freq = np.array(freq)

    sum_mag = 0
    low_index = lowcut
    high_index = 0

    # Assuming f is a numpy array of frequencies
    for i in range(1, len(freq)):  # Start from index 1 to avoid f[i-1] out of bounds
        if ((freq[i] >= lowcut) and (freq[i-1] <= lowcut)):
            low_index = i
        if (freq[i] > lowcut and freq[i] <= highcut and freq[i+1] >=highcut):
            high_index = i
                
    # Loop through the specified FFT range
    for i in range(low_index, high_index + 1):  # Include high_index
        if (type(single_normalized_fft[i]) == list):
            sum_mag += (single_normalized_fft[i][0] + single_normalized_fft[i][1])/2  # Sum the FFT magnitudes average
        else:
            sum_mag += single_normalized_fft[i]
        

             
    return sum_mag
    