#!/usr/bin/env python2.7

import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy as np
import constants
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
from geometry_msgs.msg import PoseStamped
import tf

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
        #self.posePeriod = 
        # World frame
        self.worldFrame = rospy.get_param("~worldFrame", "/world")
        # Robot goal
        self.goal = 2
        # Pose topic name
        self.name = rospy.get_param("~name")
        # Pose publiser
        self.posePub = rospy.Publisher(self.name, PoseStamped, queue_size=1)
        # FFT subscriber
        self.fftSub = rospy.Subscriber("fftValues", Floats, self.callback)
        
    def callback(self, values):
        self.ydata = np.roll(self.ydata, -1)
        self.ydata[-1] = values.data[147]
        self.curve.setData(x=self.xdata, y=self.ydata)
        self.app.processEvents()
        if self.ydata[-1] > 2 and rospy.Time.now().secs > self.poseTime.secs + 10:
            self.poseTime = rospy.Time.now()
            self.posePublisher(0, 0, 0.5*self.goal)
            if self.goal == 1: 
                self.goal = 2 
            else:
                self.goal = 1
    
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
    rospy.init_node('soundController', anonymous=True)
    # Sound controller
    sCtrl = SoundController()
    sCtrl.app.exec_()
    sCtrl.fftSub.unregister()
