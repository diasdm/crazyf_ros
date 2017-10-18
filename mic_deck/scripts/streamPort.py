import numpy as np

class StreamPort:

    def __init__(self, q, bytesOfData, CF_ON_TIME, SAMPLING_FREQ):
        # Initializes different values
        self.bytesOfData = bytesOfData
        # Last received data buffer
        self.newData = np.zeros(self.bytesOfData, dtype=np.uint8)
        # Previous to last received data buffer
        self.dispData = np.zeros(self.bytesOfData, dtype=np.uint8)
        # Vector to which the data in unpacked
        self.unpackedData = np.zeros(19, dtype=np.uint16)
        # Flags if any packet was received
        self.packetRecieved = 0
        # Imported queue from continousStream
        self.q = q
        # Packet count
        self.packetCountLoop = 0
        # Packet lost count
        self.packetLostCount = 0
        # Audio vector
        self.audioVector = np.zeros(CF_ON_TIME*SAMPLING_FREQ, dtype=np.int32)
        self.ptr = 0;

    def unpack_stream(self):
        aux = np.uint16
        self.unpackedData = np.zeros(19, dtype=np.uint16)
        ptr = 0
        halfPtr = False

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
        # Callback for data received from the copter.
        # If it is the first packet received buffers it
        if not self.packetRecieved:
            self.newDataPacketCount = packet.data[0]
            self.newData = np.fromstring(packet.data[1:], dtype=np.uint8)
            self.packetRecieved += 1
        else:
            # For some reason some times the same message is recieved
            if self.newDataPacketCount != packet.data[0]:
                self.dispDataPacketCount = self.newDataPacketCount
                self.dispData = self.newData
                self.newDataPacketCount = packet.data[0]
                self.newData = np.fromstring(packet.data[1:], dtype=np.uint8)

                self.unpack_stream()
                self.packetRecieved += 1
                # Computes how many packets were lost
                # Computes times for received and not lost data
                if self.newDataPacketCount > self.dispDataPacketCount:
                    jump = self.newDataPacketCount - self.dispDataPacketCount - 1
                    times = np.arange(self.dispDataPacketCount + self.packetCountLoop * 256,
                                    self.newDataPacketCount + self.packetCountLoop * 256, 1 / 19)
                else:
                    jump = 255 - self.dispDataPacketCount + self.newDataPacketCount
                    times = np.arange(self.dispDataPacketCount + self.packetCountLoop * 256,
                                    self.newDataPacketCount + (self.packetCountLoop + 1) * 256, 1 / 19)
                    self.packetCountLoop += 1
                # Displays percentage of packets lost
                if jump:
                    self.packetLostCount += jump
                    print('The % of lost packet is', self.packetLostCount/(self.packetRecieved+self.packetLostCount))
                # Sends fake data to queue when packet is lost (average value)
                for i in range(0, 19*jump):
                    self.q.put([times[i], 1743])
                    self.audioVector[self.ptr] = 1743
                    self.ptr += 1
                # Sends data to queue
                for i in range(0, 19):
                    self.q.put([times[i+19*jump], self.unpackedData[i]])
                    self.audioVector[self.ptr] = self.unpackedData[i]
                    self.ptr += 1
