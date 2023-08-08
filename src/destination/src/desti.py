#!/usr/bin/python3

import rospy
from math import sqrt, pow

from std_msgs.msg import Bool
from gps_c94_m8p.msg import gps_data


class DEST:
    def __init__(self):
        rospy.init_node("destination", anonymous=True)

        rospy.Subscriber("/gps_fix", gps_data, self.gpscallback)

        self.dest_pub = rospy.Publisher("/is_dest", Bool, queue_size=1)

        self.utm_x = None
        self.utm_y = None
        self.is_arrived = False
        self.dest_stop = False

        self.destination_coord = [290968.338571748, 3979992.730329507]

        while not rospy.is_shutdown():
            if self.utm_x and self.utm_y and (not self.is_arrived):
                # 현재 위치랑 목적지와의 거리 계산
                dist = sqrt(pow(self.destination_coord[0] - self.utm_x, 2) + pow(self.destination_coord[1] - self.utm_y, 2))
                if dist < 1: # 거리가 1M보다 가까운지 확인
                    self.is_arrived = True
                    self.dest_stop = True
                    otime = rospy.get_rostime().secs
                    while rospy.get_rostime().secs - otime < 30: # ROS Time을 기준으로 일정 시간동안 정지
                        print("waitting...", rospy.get_rostime().secs - otime)
                        self.dest_pub.publish(self.dest_stop)
                    self.dest_stop = False
                elif dist > 5:
                    self.is_arrived = False
            self.dest_pub.publish(self.dest_stop)



    def gpscallback(self, msg):
        self.utm_x = msg.utm_x
        self.utm_y = msg.utm_y



if __name__ == "__main__":
    d = DEST()