import numpy as np
import csv
import matplotlib.pyplot as plt
#from scipy.io import wavfile
import soundfile as sf
from numpy.fft import fft, fftfreq
from scipy.signal import stft
from scipy.interpolate import griddata
from Digitize_Filter import DigitizeFilter
from Audio_Manager import AudioManager

class Audio_Delam_Testing():

    def __init__(self):
        self.collected_averages = []
        self.freq = []
        self.x_mid_positions = []
        self.y_mid_positions = []
    

    def new_sample(self, fft, freq):

        self.collected_averages.append(fft)
        self.freq.append(freq)
        #self.x_mid_positions.append(pos_x)
        #self.y_mid_positions.append(pos_y)


    def create_rough_map(self):
        
        X = np.array(self.x_mid_positions)
        Y = np.array(self.y_mid_positions)
        
        Z = self.collected_averages

        # Determine the maximum length of inner lists
        min_len = min(len(i) for i in Z)

        # Pad shorter lists with NaN (or any other placeholder)
        padded_data = [i + [np.nan] * (min_len - len(i)) for i in Z]

        # Now create the numpy array
        Z = np.array(padded_data)


        X, Y = np.meshgrid(X, Y)

        Z = self.collected_averages

        plt.figure(figsize=(10, 6))
        plt.pcolormesh(X, Y, np.abs(Z), shading='auto')
        plt.title('Map of Delaminations')
        plt.ylabel('y')
        plt.xlabel('x')
        plt.colorbar(label='Magnitude')
        plt.show()

        def send_to_csv(self):
            # Open a file for writing
            with open('fft_outputs.csv', mode='w', newline='') as file:
                writer = csv.writer(file)
                
                # Write the rows to the CSV file
                writer.writerows(self.collected_averages)


    