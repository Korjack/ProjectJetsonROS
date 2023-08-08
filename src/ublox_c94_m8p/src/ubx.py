#!/usr/bin/python3
import rospy, serial, threading
import struct
from rtcm_msgs.msg import Message
from ublox_c94_m8p.msg import GPSmsg

class UBX:
    def __init__(self, port="/dev/ttyACM0", baudrate="19200"):
        self.port = port
        self.baudrate = baudrate

        self.dev = serial.Serial(self.port, baudrate=self.baudrate, dsrdtr=False, rtscts=False, xonxoff=False, timeout=1)

        # self.rtcm_sub = rospy.Subscriber("/rtcm", Message, self.rtcmcallback, queue_size=10)
        self.gps_pub = rospy.Publisher("/gps", GPSmsg, queue_size=1)

        self.gps_msg = GPSmsg()
        self.gps_msg.header.frame_id = "gps"


    def close(self):
        self.dev.close()
        self.dev = None

    def write(self, buf):
        return self.dev.write(buf)

    def read(self, n):
        return self.dev.read(n)

    def receive_message(self):
        rate = rospy.Rate(20)
        gpsmode = None
        N, E, Accuracy = 0, 0, 0
        
        while not rospy.is_shutdown():
            data = self.dev.readline()
            
            if data[:2] == b"\xb5\x62":
                HPPOSECEF = data[:36]
                GNGGA = data[36:].decode().split(",")
                if len(HPPOSECEF) == 36:
                    Accuracy = struct.unpack("<L", HPPOSECEF[30:34])
                    Accuracy = float(Accuracy[0]) / 10000
                if GNGGA[0] == "$GNGGA":
                    N = self.convert(GNGGA[2], GNGGA[3])
                    E = self.convert(GNGGA[4], GNGGA[5])
                    gpsmode = int(GNGGA[6])

                if N and E:
                    self.gps_msg.header.stamp = rospy.get_rostime()
                    self.gps_msg.lati = N
                    self.gps_msg.longti = E
                    self.gps_msg.Accuracy = Accuracy
                    self.gps_msg.gpsmode = gpsmode

                    self.gps_pub.publish(self.gps_msg)
            rate.sleep()

    def rtcmcallback(self, msg):
        self.dev.write(msg.message)

    
    def convert(self, value, dir):
        degree = 0
        minute = 0
        if dir == "N":
            degree = float(value[:2])
            minute = float(value[2:]) / 60
        elif dir == "E":
            degree = float(value[:3])
            minute = float(value[3:]) / 60
        
        return degree + minute



if __name__ == "__main__":
    rospy.init_node("ubx", anonymous=True)
    u = UBX()
    u.receive_message()
