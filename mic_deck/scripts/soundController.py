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

THRESHOLD = 30

def mse(a, b):
    return ((a - b) ** 2).mean(dtype=np.float32)

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
        self.ydata = np.zeros(513, dtype=np.float)
        self.curve = self.plot.plot(x=self.xdata, y=self.ydata, pen=(255,0,0))
        # Last pose switch
        self.poseTime = rospy.Time.now()
        # Maximum pose switch period
        self.posePeriod = 8
        # Robot goal
        self.goal = 0
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
        self.msg.header.stamp = self.poseTime
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
        self.errPub = rospy.Publisher("mse", std_msgs.msg.Float32, queue_size=1)
        # FFT subscriber
        self.fftSub = rospy.Subscriber("fftValues", Floats, self.callback)
        
    def callback(self, values):
        self.ydata = np.roll(self.ydata, -1)
        self.ydata[-1] = np.sum(values.data[(330-2):(330+2)]) / 5 
        self.curve.setData(x=self.xdata, y=self.ydata)
        self.app.processEvents()
        err = mse(self.clappingFFT, values.data)
        self.errPub.publish(err)
        if err > THRESHOLD and rospy.Time.now().secs > self.poseTime.secs + self.posePeriod:
           self.poseTime = rospy.Time.now()
           if self.goal == 0: 
               self.goal = 1 
           else:
               self.goal = 0

        self.msg.header.seq += 1
        self.msg.pose.position.z = self.z + self.goal*0.5
        self.msg.header.stamp = rospy.Time.now()
        self.posePub.publish(self.msg)
    
    def posePublisher(self, x, y, z):
        msg = PoseStamped()
        msg.header.seq = 0
        msg.header.stamp = self.poseTime
        msg.header.frame_id = self.worldFrame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        msg.pose.orientation.x = quaternion[0]
        msg.pose.orientation.y = quaternion[1]
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = quaternion[3]
        self.posePub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('publish_pose', anonymous=True)
    
    # Sound controller
    sCtrl = SoundController()
    sCtrl.app.exec_()
    sCtrl.fftSub.unregister()
