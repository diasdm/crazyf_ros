import rospy
from std_msgs.msg import String
from crazyflie_driver.msg import crtpPacket
from continuousStream import ContinousStream
from streamPort import StreamPort
import numpy as np

CF_ON_TIME = 60 # How many seconds the cf is connected
SAMPLING_FREQ = 7000 # Microphone sampling frequency

def callback(packet):
    sp.incoming(packet)
    
def shutdownCall():
    # Saves audio array to a csv file
    np.savetxt("new.csv", sp.audioVector[:sp.ptr], delimiter=",")
    cs.process.terminate()
    
def listener():
    rospy.init_node('micDeckClient', anonymous=True)

    rospy.Subscriber("packets", crtpPacket, callback)
    # Function called on sutdown
    rospy.on_shutdown(shutdownCall)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    # Displays graph
    cs = ContinousStream(4, SAMPLING_FREQ, 24)
    # Queue where data to be displayed is added
    q = cs.start()
    sp = StreamPort(q, 29, CF_ON_TIME, SAMPLING_FREQ)
    listener()
