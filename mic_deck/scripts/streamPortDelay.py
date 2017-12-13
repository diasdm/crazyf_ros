#!/usr/bin/env python2.7

import rospy
import numpy as np
from signal import signal, SIGPIPE, SIG_DFL
import time
from crazyflie_driver.msg import crtpPacket
from multiprocessing import Process
from checkDelay import MicrophoneRecorder
import scipy.io.wavfile
import constants

signal(SIGPIPE,SIG_DFL)

class StreamPort:
    # Unpacks and queues audio samples
    def __init__(self, bytesOfData, SAMPLING_FREQ, AUDIO_MEAN):
        signal(SIGPIPE,SIG_DFL)
        self.bytesOfData = bytesOfData
        self.AUDIO_MEAN = AUDIO_MEAN
        # Last received data buffer
        self.newData = np.zeros(self.bytesOfData, dtype=np.uint8)
        # Previous received data buffer
        # We buffer one packet to know if any packets where lost between the two last recieved
        self.dispData = np.full(self.bytesOfData, self.AUDIO_MEAN, dtype=np.uint8)
        # Vector to which the data in unpacked
        self.unpackedData = np.zeros(19, dtype=np.uint16)
        # Flags if any packet was received
        self.packetRecieved = 0
        # Packet lost count
        self.packetLostCount = 0
        # Audio vector
        self.audioVector = []
        self.ptr = 0;
        self.times = []
        self.previousTime = time.time()
        
    def unpack_stream(self):
        aux = np.uint16
        self.unpackedData = np.zeros(19, dtype=np.uint16)
        ptr = 0
        halfPtr = False
        # Unpacking loop
        for i in range(0, 19):
            aux = 0
            if not halfPtr:
                aux = self.dispData[ptr]
                aux = aux << 4
                ptr += 1
                aux = aux | (self.dispData[ptr] >> 4)
                self.unpackedData[i] = aux
                halfPtr = True
            else:
                aux = self.dispData[ptr] & 0x0F
                aux = aux << 8
                ptr += 1
                aux = aux | self.dispData[ptr]
                ptr += 1
                self.unpackedData[i] = aux
                halfPtr = False

    def incoming(self, packet):
        # Callback for data received from Crazyflie
        # If it is the first packet received buffers it
        now = time.time()
        if not self.packetRecieved:
            self.previousTime = now
            # Increment value
            self.newDataPacketCount = ord(packet.data[0])
            # Audio samples
            self.newData = np.fromstring(packet.data[1:], dtype=np.uint8)
            self.packetRecieved += 1
        else:
            # For some reason some times the same packet is recieved twice so this if was added
            if self.newDataPacketCount != ord(packet.data[0]):
                # Buffers last recieved packet
                self.dispDataPacketCount = self.newDataPacketCount
                self.dispData = self.newData
                self.newDataPacketCount = ord(packet.data[0])
                self.newData = np.fromstring(packet.data[1:], dtype=np.uint8)
                # Unpacks the packet to display
                self.unpack_stream()
                self.packetRecieved += 1
                # Computes how many packets were lost
                if self.newDataPacketCount > self.dispDataPacketCount:
                    jump = self.newDataPacketCount - self.dispDataPacketCount - 1
                else:
                    jump = 255 - self.dispDataPacketCount + self.newDataPacketCount

                # Displays percentage of packets lost when a packet is lost
                if jump:
                    self.packetLostCount += jump
                    print('The % of lost packet is', 1.*self.packetLostCount/(self.packetRecieved+self.packetLostCount))
                for i in range(0, jump):
                    self.times.append(self.previousTime)
                # Queues the average value when a packet is lost
                for i in range(0, 19*jump):
                    self.audioVector.append(self.AUDIO_MEAN)
                    self.ptr += 1
                self.times.append(self.previousTime)
                # Queues recieved data
                for i in range(0, 19):
                    self.audioVector.append(self.unpackedData[i])
                    self.ptr += 1
                self.previousTime = now
                    
    def callback(self, packet):
        # Checks header
        if packet.header == 16:
            self.incoming(packet)
            
if __name__ == '__main__':
    sp = StreamPort(29, constants.CF_FS, constants.AUDIO_MEAN)
    rospy.init_node('micDeckClient', anonymous=True)
    rospy.Subscriber("crazyflie/packets", crtpPacket, sp.callback, queue_size=1, tcp_nodelay=True)
    mic = MicrophoneRecorder()
    p = Process(target=mic.read())
    p.start()
    rospy.spin()
    np.savetxt(constants.FILE_PATH + "CrazyMicTimeStamps.csv", sp.times, delimiter=",")
    scipy.io.wavfile.write(constants.FILE_PATH + "CrazyMicRecording.wav", constants.CF_FS, np.asarray(sp.audioVector, dtype=np.int16))
    p.join()
