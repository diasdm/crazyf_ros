import numpy as np
from signal import signal, SIGPIPE, SIG_DFL

class StreamPort:
    # Unpacks and queues audio samples
    def __init__(self, q, bytesOfData, SAMPLING_FREQ, AUDIO_MEAN, specPub):
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
        # Flags if any packet was received and ignores the first 500 packets
        # For some reason in the beginig the packet lost rate is too high
        self.packetRecieved = -500 
        # Queue used to send the audio to continuousStream process
        self.q = q
        # Packet lost count
        self.packetLostCount = 0
        # Audio vector
        self.audioVector = []
        # Spectrogram
        self.specPub = specPub
        self.sampToSpec = 1024
        self.partAudVect = np.full(self.sampToSpec, self.AUDIO_MEAN, dtype=np.uint16)
        self.wind = np.hanning(self.sampToSpec)
    
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
        if self.packetRecieved < 0:
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
                    self.partAudVect = np.roll(self.partAudVect, -19*jump)
                    self.partAudVect[-19*jump:] = np.full(19*jump, self.AUDIO_MEAN, dtype=np.uint16)
                # Queues the average value when a packet is lost
                for i in range(0, 19*jump):
                    self.q.put(self.AUDIO_MEAN)
                    self.audioVector.append(self.AUDIO_MEAN)
                # Queues recieved data
                for i in range(0, 19):
                    self.q.put(self.unpackedData[i])
                    self.audioVector.append(self.unpackedData[i])
                    
                self.partAudVect = np.roll(self.partAudVect, -19)
                self.partAudVect[-19:] = self.unpackedData
                
                if not self.packetRecieved % 25:
                    # FFT computation
                    spec = np.fft.rfft(self.partAudVect*self.wind) / self.sampToSpec
                    # Get magnitude 
                    psd = abs(spec)
                    # Convert to dB scale
                    psd = 20 * np.log10(psd)
                    # Publishes fft values
                    self.specPub.publish(psd)
