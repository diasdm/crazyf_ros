#!/usr/bin/env python2.7

import rospy
from std_msgs.msg import String
from crazyflie_driver.msg import crtpPacket
from continuousStream import ContinousStream
from streamPort import StreamPort
from multiprocessing import Queue
import numpy as np
import datetime
import scipy.io.wavfile
import constants
from rospy_tutorials.msg import Floats

class MicDeckNode:
    def __init__(self):
        rospy.init_node('micDeckClient', anonymous=True)
        rospy.on_shutdown(self.cleanUp)
        # Queue where data to be displayed is added
        self.q = Queue()
        # Displays graph
        self.cs = ContinousStream(self.q, 4, constants.CF_FS, 24)
        self.cs.start()
        # FFT values publisher
        self.specPub = rospy.Publisher('fftValues', Floats, queue_size=10)
        # Unpacks and queues audio signal
        self.sp = StreamPort(self.q, 29, constants.CF_FS, constants.AUDIO_MEAN, self.specPub)
        # Subscribes to topic and spins
        self.sub = rospy.Subscriber("packets", crtpPacket, self.callback)
        self.timestamp = datetime.datetime.now() # Gets timestamp
        
    def cleanUp(self):
        # Waits for any computation to end
        rospy.sleep(1)
        print('Will close window')
        self.cs.app.quit()
        self.flushQueue()
        # Closes queue
        self.q.close()
        print('Queue closed')
        
    def flushQueue(self):
        while not self.q.empty():
            garbage = self.q.get()
            
    # Function called on packet arrival
    def callback(self, packet):
        # Checks header
        if packet.header == 16:
            self.sp.incoming(packet)

    def listener(self):
        # Waits for shutdownCall
        rospy.spin()
        print('Spining stoped')
        
if __name__ == '__main__':
    node = MicDeckNode()
    node.listener()
    node.cs.process.join()
    # Saves audio array to a wav file
    print('Saving WAV')
    scipy.io.wavfile.write(constants.FILE_PATH + node.timestamp.strftime("%d-%m_%H:%M:%S") + ".wav", constants.CF_FS, 
    np.asarray(node.sp.audioVector, dtype=np.int16))
    print('Saving CSV')
    # Saves audio array to a wav file, given that this values aren't processed this is better for a direct comparation
    np.savetxt(node.timestamp.strftime(constants.FILE_PATH + "%d-%m_%H:%M:%S") + ".csv", node.sp.audioVector, delimiter=",")
