#!/usr/bin/env python2.7

import rospy
from rospy_tutorials.msg import Floats
import numpy as np
import constants
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from geometry_msgs.msg import PoseStamped
import tf
import std_msgs.msg

THRESHOLD = 9.5
FREQ_AMP_DIF = 6
CTRL_DELAY = 15
FREQ_INTERVAL = 4

class SoundController:
    def __init__(self):
        self.app = QtGui.QApplication([])
        self.win = pg.GraphicsWindow()
        self.win.setWindowTitle('Sound controller')
        # Sound wave plot
        self.plot = self.win.addPlot()
        self.plot.setLabel('bottom', 'Frequency', 'Hz')
        self.plot.setLabel('left', 'Amplitude', '')
        self.plot.showGrid(x=True, y=True)
        self.xdata = np.linspace(0, 3500, 513)
        self.upData = np.zeros(513, dtype=np.float)
        self.downData = np.zeros(513, dtype=np.float)
        self.upCurve = self.plot.plot(x=self.xdata, y=self.upData, pen=(255,0,0))
        self.downCurve = self.plot.plot(x=self.xdata, y=self.downData, pen=(0,255,0))
        # Last pose switch
        self.initTime = rospy.Time.now()
        # Up Frequency
        self.upFreq = 1550
        self.upIndex = int(self.upFreq/(3500.0/513))
        print("Up index %d", self.upIndex)
        # Down Frequency 
        self.downFreq = 1200
        self.downIndex = int(self.downFreq/(3500.0/513))
        print("Down index %d", self.downIndex)
        # Height increment
        self.hInc = 0.02
        # Pose topic name
        self.worldFrame = rospy.get_param("~worldFrame", "/world")
        self.name = rospy.get_param("~name")
        self.x = rospy.get_param("~x")
        self.y = rospy.get_param("~y")
        self.z = rospy.get_param("~z")
        self.filesDir = rospy.get_param("~files_dir")
        # Pose object
        self.msg = PoseStamped()
        self.msg.header.seq = 0
        self.msg.header.stamp = self.initTime
        self.msg.header.frame_id = self.worldFrame
        self.msg.pose.position.x = self.x
        self.msg.pose.position.y = self.y
        self.msg.pose.position.z = self.z
        self.quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        self.msg.pose.orientation.x = self.quaternion[0]
        self.msg.pose.orientation.y = self.quaternion[1]
        self.msg.pose.orientation.z = self.quaternion[2]
        self.msg.pose.orientation.w = self.quaternion[3]
        # FFT compare value
        self.silenceFFT = np.genfromtxt(self.filesDir + 'src/mic_deck/scripts/meanSilenceFFT.csv', delimiter=',')
        self.clappingFFT = np.genfromtxt(self.filesDir + 'src/mic_deck/scripts/meanClappingFFT.csv', delimiter=',')
        # Pose publiser
        self.posePub = rospy.Publisher(self.name, PoseStamped, queue_size=1)
        # FFT subscriber
        self.fftSub = rospy.Subscriber("fftValues", Floats, self.callback)
        
    def callback(self, values):
        # Up values
        self.upData = np.roll(self.upData, -1)
        self.upData[-1] = np.mean(values.data[(self.upIndex-FREQ_INTERVAL):(self.upIndex+FREQ_INTERVAL)])
        self.upCurve.setData(x=self.xdata, y=self.upData)
        # Down values
        self.downData = np.roll(self.downData, -1)
        self.downData[-1] = np.mean(values.data[(self.downIndex-FREQ_INTERVAL):(self.downIndex+FREQ_INTERVAL)])
        self.downCurve.setData(x=self.xdata, y=self.downData)
        
        maxIndex = np.argmax([self.downData[-1], self.upData[-1]])
        max = np.maximum(self.downData[-1], self.upData[-1])
        dif = np.absolute(self.downData[-1] - self.upData[-1])
        
        if max > THRESHOLD and dif > FREQ_AMP_DIF and rospy.Time.now().secs > self.initTime.secs + CTRL_DELAY:
            if maxIndex == 0:
                if self.z > 1.0:
                    self.z = self.z - self.hInc
            else:
                if self.z < 1.5:
                    self.z = self.z + self.hInc
        
        self.msg.header.seq += 1
        self.msg.pose.position.z = self.z
        self.msg.header.stamp = rospy.Time.now()
        self.posePub.publish(self.msg)

if __name__ == '__main__':
    rospy.init_node('publish_pose', anonymous=True)
    
    # Sound controller
    sCtrl = SoundController()
    sCtrl.app.exec_()
    sCtrl.fftSub.unregister()
