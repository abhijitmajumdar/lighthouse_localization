import socket
import struct
import time
import tf

class SocketInterface:
    def __init__(self, IP="127.0.0.1", PORT=5005, debug=False):
        self.ip = IP
        self.port = PORT
        self.debug = debug
        self.sock = socket.socket(socket.AF_INET, # Internet
                             socket.SOCK_DGRAM) # UDP
        if self.debug==True:
            print "Socket connection over IP:",self.ip," PORT:",self.port

    def send_target(self,th,roll,pitch,yaw,arm):
        sendstr = bytearray(struct.pack("fffff", th,roll,pitch,yaw,arm))
        n = self.sock.sendto(sendstr, (self.ip, self.port))
        if self.debug==True:
            print "sending data:",th,roll,pitch,yaw,arm

    def publish_transform(self, T, src_frame, dest_frame, t=None):
        # TODO: populate this to also communicate transforms over ipc
        pass

    def publish_pose(self, T, frame, t=None):
        timestamp = time.time() if t is None else t
        Q = tf.transformations.quaternion_from_matrix(T)
        sendstr = bytearray(
                            struct.pack(
                                        "ffffffff",
                                        timestamp,
                                        T[0][3],
                                        T[1][3],
                                        T[2][3],
                                        Q[0],
                                        Q[1],
                                        Q[2],
                                        Q[3]
                                        )
                            )
        n = self.sock.sendto(sendstr, (self.ip, self.port))
        if self.debug==True:
            print "sending data:",timestamp,T[0][3],T[1][3],T[2][3],Q[0],Q[1],Q[2],Q[3]

    def is_alive(self):
        # TODO: Add logic here
        return True
