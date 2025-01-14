import Filter
import Microphone
import DataCache
class Audio_Manager(Filter, DataCache, Microphone):
    def __init__(self):
        self.directional_microphone = Microphone()
        self.omnidirectional_microphone = Microphone()
        self.filter = Filter()
        self.data_cache = DataCache()

    def configure(self):
        self.is_configured = False
        try:
            # Initialize mircophones
            #set up audio processing parameters or whatever is needed -- these are examples idk all thats necessary yet
            self.directional_microphone.pin(1)
            self.omnidirectional_microphone.pin(2)
            self.filter.configure()
            self.data_cache.configure()
            self.is_configured = True
            return True
        
        except Exception as e:
            return False
        
    def active(self):
        if self.is_configured:
            self.directional_microphone.start()
            self.omnidirectional_microphone.start()
            self.audio_samples = []
            # add audio samples to array
            # when array size equals certain amount send to filter?
            # after filter, send to cache?