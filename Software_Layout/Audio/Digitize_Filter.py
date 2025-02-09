import numpy as np
import matplotlib.pyplot as plt
from scipy import butter, lfilter
import math

class DigitizeFilter:
    def __init__(self):
        pass
    
    # creates bandpass filter 
    # arguments:
    #   -low cut-off frequency
    #   -high cut-off frequency
    #   -sampling rate
    #   -order (default 5)
    # returns filter coefficients b,a
    def bandpass_butterworth(lowcut, highcut, fs, order=5):
        nyq = 0.5 * fs      # nyquist frequency
        band_low = lowcut / nyq
        band_high = highcut / nyq

        b, a = butter(order, [band_low, band_high], btype='band')      #butter imported from scipy

        return b, a
    

    # applies bandpass filter 
    # arguments:
    #   -audio data
    #   -low cut-off frequency
    #   -high cut-off frequency
    #   -sampling rate
    #   -order (default 5)
    # returns filtered signal
    def bandpass_butterworth_filter(data, lowcut, highcut, fs, order=5):
        b,a = DigitizeFilter.bandpass_butterworth(lowcut, highcut, fs, order=order)     #create filter from specs
        y = lfilter(b, a, data)     #apply filter to signal
        return y
    
    
