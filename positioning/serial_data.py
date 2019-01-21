import serial
import struct
import math

class Serial:
    def __init__(self,port,timeout=3,debug=False,swap_xy=False):
        self.s = serial.Serial(\
            port = port,\
            baudrate = 115200,\
            parity=serial.PARITY_NONE,\
            stopbits=serial.STOPBITS_ONE,\
            bytesize=serial.EIGHTBITS,\
            timeout=timeout
        )
        if self.s.is_open==False:
            raise ValueException("Could not open serial port")
        self.debug = debug
        self.swap_xy = swap_xy

    def string_to_bytes(self,b):
        return list(map(ord,b))

    def read_data(self):
        sample = []
        self.s.flushInput()
        self.s.write(b'#')
        line = self.s.read_until(b'\r\n',size=20)
        if self.debug==True: print self.string_to_bytes(line)
        if len(line)==19:
            if line[0]==b'>' and line[17:19]==b'\r\n':
                for r in [1,5,9,13]:
                    x = struct.unpack( ">h", line[r:r+2])[0]
                    y = struct.unpack( ">h", line[r+2:r+4])[0]
                    x,y = (x/16000.0)*(math.pi/2),(y/16000.0)*(math.pi/2)
                    if self.swap_xy==True: x,y = y,x
                    sample.append([x,y])
        return sample


'''
To test this module run:
python serial_data.py </dev/ttyUSB0>
'''
if __name__=="__main__":
    import sys
    import time
    port = '/dev/ttyUSB0'
    if len(sys.argv)>1:
        port = sys.argv[1]
        print 'Using port',port
    ser = Serial(port,debug=False)
    while(True):
        sample = ser.read_data()
        if len(sample)>0:
            print sample
        else:
            print '.'
        time.sleep(0.02)
