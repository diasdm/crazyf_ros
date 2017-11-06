#!/usr/bin/env python2.7

import rospy
from std_msgs.msg import String
from crazyflie_driver.msg import crtpPacket
from continuousStream import ContinousStream
from streamPort import StreamPort
from multiprocessing import Queue
import numpy as np
import datetime
from signal import signal, SIGPIPE, SIG_DFL

signal(SIGPIPE,SIG_DFL)
CF_ON_TIME = 60 # How many seconds the cf is connected
SAMPLING_FREQ = 7000 # Microphone sampling frequency
timestamp = datetime.datetime.now() # Gets timestamp
global sub

def callback(packet):
    if packet.header == 16:
        sp.incoming(packet)
       
def listener():
    rospy.init_node('micDeckClient', anonymous=True)
    global sub
    sub = rospy.Subscriber("crazyflie/packets", crtpPacket, callback)
    # Waits for shutdownCall
    rospy.spin()

if __name__ == '__main__':
    # Queue where data to be displayed is added
    q = Queue()
    # Displays graph
    cs = ContinousStream(q, 4, SAMPLING_FREQ, 24)
    cs.start()
    # Unpacks and queues audio signal
    sp = StreamPort(q, 29, CF_ON_TIME, SAMPLING_FREQ)
    # Subscribes to topic and spins
    listener()
    # Unsubscribe
    sub.unregister()
    # Stops QT timer
    cs.timer.stop()
    # Closes app
    cs.app.closeAllWindows()
    # Joins process
    cs.process.join()
    # Saves audio array to a csv file
    np.savetxt(timestamp.strftime("%d-%m_%H:%M:%S") + ".csv", sp.audioVector[:sp.ptr], delimiter=",")
    # Closes queue
    q.close()
