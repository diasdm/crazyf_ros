import numpy as np
import pyaudio
import wave
import time
import math
from signal import signal, SIGPIPE, SIG_DFL
import scipy.io.wavfile
import constants

FS = 44100 #Hz
CHUNKSZ = 1024 #samples
RECORD_SECONDS = 20
FORMAT = pyaudio.paInt16

class MicrophoneRecorder():
    def __init__(self):
        self.p = pyaudio.PyAudio()
        self.frames = np.zeros(int(math.floor(FS * RECORD_SECONDS / CHUNKSZ)) * CHUNKSZ, dtype=np.int16)
        self.times = []
        
    def read(self): 
        self.stream = self.p.open(format=pyaudio.paInt16, channels=1, rate=FS, input=True, frames_per_buffer=CHUNKSZ)
        print("PC microphone started recording.")
        for i in range(0, int(FS * RECORD_SECONDS / CHUNKSZ)):
            data = self.stream.read(CHUNKSZ)
            self.times.append(time.time())
            self.frames[i*CHUNKSZ:(i+1)*CHUNKSZ] = np.fromstring(data, dtype=np.int16)
            
        self.close()
            
    def close(self):
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()
        np.savetxt(constants.FILE_PATH + "PCMicTimeStamps.csv", np.asarray(self.times), delimiter=",")
        scipy.io.wavfile.write(constants.FILE_PATH + "PCMicRecording.wav", FS, self.frames)
        print("PC microphone finished recording.")
    
