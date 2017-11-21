import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
from multiprocessing import Process
import time
import signal
from signal import signal, SIGPIPE, SIG_DFL, SIGINT
import constants

class ContinousStream:
    # Displays continous stream by buffering <samplesBuffered> samples
    def __init__(self, q, secToDisp, SAMPLING_FREQ, graphUpdateFreq):
        # Prevents SIGPIPE exception to be displayed
        signal(SIGPIPE,SIG_DFL)
        signal(SIGINT,SIG_DFL)
        # Process in which the ploting runs
        self.process = Process(target=self.run)
        # Data queue
        self.q = q
        # Audio sampling frequency
        self.SAMPLING_FREQ = SAMPLING_FREQ
        # Buffer length
        self.storedSamp = secToDisp * SAMPLING_FREQ
        self.ybuffer = np.zeros(self.storedSamp, dtype=np.uint16)
        # Update frequency of the plot
        self.graphUpdateFreq = graphUpdateFreq
        # Number of seconds of data displayed
        self.xdata = np.arange(-secToDisp, 0, 1. / SAMPLING_FREQ)
        self.ydata = np.full(self.xdata.size, constants.AUDIO_MEAN, dtype=np.uint16)
        self.ptr = 0
        # Number of samples used for computing the Spectrogram
        self.sampToSpec = 1024
        self.timer = pg.QtCore.QTimer()
        self.app = QtGui.QApplication([])

    def start(self):
        self.process.start()

    def run(self):
        self.win = pg.GraphicsWindow()
        self.win.setWindowTitle('Microphone data')
        # Sound wave plot
        plot = self.win.addPlot()
        plot.setLabel('bottom', 'Time', 's')
        plot.setLabel('left', 'Amplitude', '')
        plot.showGrid(x=True, y=True)
        self.curve = plot.plot(x=self.xdata, y=self.ydata, pen=(255,0,0))
        
        # Next row
        self.win.nextRow()
        
        # Spectrogram plot
        self.specImg = pg.ImageItem()
        specPlot = self.win.addPlot()
        specPlot.addItem(self.specImg)
        self.imgArray = np.zeros((1000, self.sampToSpec/2+1))
        
        # Bipolar colormap
        pos = np.array([0., 1., 0.5, 0.25, 0.75])
        color = np.array([[0,255,255,255], [255,255,0,255], [0,0,0,255], (0, 0, 255, 255), (255, 0, 0, 255)], dtype=np.ubyte)
        cmap = pg.ColorMap(pos, color)
        lut = cmap.getLookupTable(0.0, 1.0, 256)

        # Set colormap
        self.specImg.setLookupTable(lut)
        self.specImg.setLevels([-50,40])
        
        # Setup the correct scaling for y-axis
        freq = np.arange((self.sampToSpec/2)+1)/(float(self.sampToSpec)/self.SAMPLING_FREQ)
        yscale = 1.0/(self.imgArray.shape[1]/freq[-1])
        self.specImg.scale((1./self.SAMPLING_FREQ)*self.sampToSpec, yscale)
        
        specPlot.setLabel('left', 'Frequency', units='Hz')
        self.wind = np.hanning(self.sampToSpec)
        self.timer.timeout.connect(self.update)
        
        # Timer init
        self.timer.start(1./self.graphUpdateFreq * 1000)
        self.app.exec_()

    def update(self):
        ptrOld = self.ptr
        # Gets samples from queue
        while not self.q.empty():
            samp = self.q.get()
            self.ybuffer[self.ptr - ptrOld] = samp
            self.ptr += 1
        # Rolls vector
        self.ydata = np.roll(self.ydata, -(self.ptr - ptrOld))
        # Copies samples to the ploted vector
        self.ydata[self.storedSamp - (self.ptr - ptrOld):] = self.ybuffer[:(self.ptr - ptrOld)]
        # Plots data
        self.curve.setData(x=self.xdata, y=self.ydata)
    
        # Chunk used in Spectrogram
        chunk = self.ydata[(self.storedSamp-self.sampToSpec):]
        spec = np.fft.rfft(chunk*self.wind) / self.sampToSpec
        # Get magnitude 
        psd = abs(spec)
        # Convert to dB scale
        psd = 20 * np.log10(psd)
        # Roll down image array
        self.imgArray = np.roll(self.imgArray, -1, 0)
        self.imgArray[-1:] = psd
        # Sets image
        self.specImg.setImage(self.imgArray, autoLevels=False)
        self.app.processEvents()
