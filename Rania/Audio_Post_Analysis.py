import numpy as np
import matplotlib.pyplot as plt
#from scipy.io import wavfile
import soundfile as sf
from numpy.fft import fft, fftfreq
from scipy.signal import stft
from Digitize_Filter import DigitizeFilter
from Audio_Manager import AudioManager

class AudioPostAnalysis():

    def __init__(self):
        self.collected_averages = []
        self.x_positions = []
        self.y_positions = []
    

    def new_sample(self, average, pos_x, pos_y):
        self.collected_averages.append(average)
        self.x_positions.append(pos_x)
        self.y_positions.append(pos_y)

    def create_rough_map(self):
        plt.figure(figsize=(10, 6))
        plt.pcolormesh(self.x_positions, self.y_positions, np.abs(self.collected_averages), shading='gouraud')
        plt.title('Map of Delaminations')
        plt.ylabel('y')
        plt.xlabel('x')
        plt.colorbar(label='Magnitude')
        plt.show()

    

def __main__():
    delamTest = AudioPostAnalysis()
    audio1 = AudioManager(1, 0, 1)
    audio1sam = audio1.convert_from_wav("DELAM_floor_3_single_6_alewife.wav")
    [audio1sam, audio1freq] = audio1.convert_to_fft(audio1sam)
    audio1out = audio1.return_average_mag(audio1sam, audio1freq)
    delamTest.new_sample(audio1out, 0, 0) 
    audio2 = AudioManager(2, 1, 2)
    audio2sam = audio2.convert_from_wav("DELAM_floor_4_single_5_alewife.wav") 
    [audio2sam, audio2freq] = audio2.convert_to_fft(audio2sam)
    audio2out = audio2.return_average_mag(audio2sam, audio2freq)
    delamTest.new_sample(audio2out, 1, 0)
    audio3 = AudioManager(3, 2, 3)
    audio3sam = audio3.convert_from_wav("DELAM floor 4 single 1 alewife.wav")
    [audio3sam, audio3freq] = audio3.convert_to_fft(audio3sam)
    audio3out = audio3.return_average_mag(audio3sam, audio3freq)
    delamTest.new_sample(audio3out, 2, 0)
    audio4 = AudioManager(4, 3, 4)
    audio4sam = audio4.convert_from_wav("LAM ramp 2 single 1 alewife.wav")
    [audio4sam, audio4freq] = audio4.convert_to_fft(audio4sam)
    audio4out = audio4.return_average_mag(audio4sam, audio4freq)
    delamTest.new_sample(audio4out, 3, 0)
    audio5 = AudioManager(5, 4, 5)
    audio5sam = audio5.convert_from_wav("LAM_floor_3_single_7_alewife.wav")
    [audio5sam, audio5freq] = audio5.convert_to_fft(audio5sam)
    audio5out = audio5.return_average_mag(audio5sam, audio5freq)
    delamTest.new_sample(audio5out, 4, 0)
    audio6 = AudioManager(6, 5, 6)
    audio6sam = audio6.convert_from_wav("LAM_floor_4_single_1_alewife.wav")
    [audio6sam, audio6freq] = audio6.convert_to_fft(audio6sam)
    audio6out = audio6.return_average_mag(audio6sam, audio6freq)
    delamTest.new_sample(audio6out, 5, 0)
    delamTest.create_rough_map()



__main__()    

    
        


    

    

    