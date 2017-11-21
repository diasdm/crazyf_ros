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
import scipy.io.wavfile
import constants
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

signal(SIGPIPE,SIG_DFL)
timestamp = datetime.datetime.now() # Gets timestamp
global sub # Subscriber object

# Function called on packet arrival
def callback(packet):
    # Checks header
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
    cs = ContinousStream(q, 4, constants.CF_FS, 24)
    cs.start()
    # FFT values publisher
    specPub = rospy.Publisher('fftValues', numpy_msg(Floats), queue_size=10)
    # Unpacks and queues audio signal
    sp = StreamPort(q, 29, constants.CF_FS, constants.AUDIO_MEAN, specPub)
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
    scipy.io.wavfile.write(constants.FILE_PATH + timestamp.strftime("%d-%m_%H:%M:%S") + ".wav", constants.CF_FS, np.asarray(sp.audioVector, dtype=np.int16))
    # Closes queue
    q.close()
