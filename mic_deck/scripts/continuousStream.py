"""
Displays continous stream
"""
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
from multiprocessing import Process, Queue
import time

class ContinousStream:
    # Displays continous stream by buffering <samplesBuffered> samples
    def __init__(self, secToDisp, SAMPLING_FREQ, graphUpdateFreq):
        self.process = Process(target=self.run)
        self.q = Queue()
        # Buffer length
        self.storedSamp = secToDisp * SAMPLING_FREQ
        self.xbuffer = np.zeros(self.storedSamp, dtype=np.float)
        self.ybuffer = np.zeros(self.storedSamp, dtype=np.uint16)
        # Update frequency of the plot
        self.graphUpdateFreq = graphUpdateFreq
        # Number of seconds of data displayed
        self.xdata = np.zeros(self.storedSamp, dtype=np.float)
        self.ydata = np.zeros(self.storedSamp, dtype=np.uint16)
        self.ptr = 0
        self.timer = pg.QtCore.QTimer()

    def start(self):
        self.process.start()
        return self.q

    def run(self):
        self.app = QtGui.QApplication([])
        self.win = pg.GraphicsWindow()
        self.win.setWindowTitle('pyqtgraph example: Scrolling Plots')

        plot = self.win.addPlot()
        plot.setLabel('bottom', 'Time', 's')

        self.curve = plot.plot(x=self.xdata, y=self.ydata)

        self.timer.timeout.connect(self.update)
        self.timer.start(1/self.graphUpdateFreq * 1000)
        self.app.exec_()

    def update(self):
        ptrOld = self.ptr

        while not self.q.empty():
            pair = self.q.get()
            self.ybuffer[self.ptr - ptrOld] = pair[1]
            self.xbuffer[self.ptr - ptrOld] = pair[0]
            self.ptr += 1

        self.ydata = np.roll(self.ydata, -(self.ptr - ptrOld))
        self.xdata = np.roll(self.xdata, -(self.ptr - ptrOld))

        self.ydata[self.storedSamp - (self.ptr - ptrOld):] = self.ybuffer[:(self.ptr - ptrOld)]
        self.xdata[self.storedSamp - (self.ptr - ptrOld):] = self.xbuffer[:(self.ptr - ptrOld)]

        if self.ptr < self.storedSamp:
            self.curve.setData(x=self.xdata[(self.storedSamp - self.ptr):], y=self.ydata[(self.storedSamp - self.ptr):])
        else:
            self.curve.setData(x=self.xdata, y=self.ydata)