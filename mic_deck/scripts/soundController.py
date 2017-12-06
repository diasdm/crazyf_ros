#!/usr/bin/env python2.7

import rospy
from rospy_tutorials.msg import Floats
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from geometry_msgs.msg import PoseStamped
import tf
import std_msgs.msg

THRESHOLD = 9.5 # Threshold above which a sound is considered
FREQ_AMP_DIF = 6 # Aplitude above which is consired that a frequency was targeted (to avoid errors when other sounds may occur)
CTRL_DELAY = 15 # Time in seconds after which the controller starts to work
FREQ_INTERVAL = 4 # Frequency band width

class SoundController:
    def __init__(self):
        # GUI app
        self.app = QtGui.QApplication([])
        self.win = pg.GraphicsWindow()
        self.win.setWindowTitle('Sound controller')
        # FFT plot
        self.plot = self.win.addPlot()
        self.plot.setLabel('bottom', 'Frequency', 'Hz')
        self.plot.setLabel('left', 'Amplitude', 'dB')
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
        # Up frequency FFT index
        self.upIndex = int(self.upFreq/(3500.0/513))
        # Down Frequency 
        self.downFreq = 1200
        # Down frequency FFT index
        self.downIndex = int(self.downFreq/(3500.0/513))
        # Height increment
        self.hInc = 0.02
        # Parameters
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
        # Pose publiser
        self.posePub = rospy.Publisher(self.name, PoseStamped, queue_size=1)
        # FFT subscriber
        self.fftSub = rospy.Subscriber("fftValues", Floats, self.callback)
        
    def callback(self, values):
        # fftValues subscriber callback
        # We publish goal positions at the same frequency that the callback is called ~20Hz
        
        # Up values
        self.upData = np.roll(self.upData, -1)
        self.upData[-1] = np.mean(values.data[(self.upIndex-FREQ_INTERVAL):(self.upIndex+FREQ_INTERVAL)])
        self.upCurve.setData(x=self.xdata, y=self.upData)
        # Down values
        self.downData = np.roll(self.downData, -1)
        self.downData[-1] = np.mean(values.data[(self.downIndex-FREQ_INTERVAL):(self.downIndex+FREQ_INTERVAL)])
        self.downCurve.setData(x=self.xdata, y=self.downData)
        # Maxmum, maximum index and absolute difference
        max = np.maximum(self.downData[-1], self.upData[-1])
        maxIndex = np.argmax([self.downData[-1], self.upData[-1]])
        dif = np.absolute(self.downData[-1] - self.upData[-1])
        # Goal stamp
        now = rospy.Time.now()
        
        # Once the delay has passed it checks if the maximum is above the threshold 
        # and the difference is above the difference threshold
        if max > THRESHOLD and dif > FREQ_AMP_DIF and now.secs > self.initTime.secs + CTRL_DELAY:
            if maxIndex == 0:
                # Goes down
                if self.z > 1.0:
                    self.z = self.z - self.hInc
            else:
                # Goes up
                if self.z < 1.5:
                    self.z = self.z + self.hInc
        
        # Publishes goal
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
