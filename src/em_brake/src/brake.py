#!/usr/bin/python3

import rospy

from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from em_brake.msg import em_data
from jetson_msg.msg import driving_msg

class EM_BRAKE:
    def __init__(self):
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/ctrl_cmd", driving_msg, self.ctrlcallback)

        self.em_stop_pub = rospy.Publisher("/em_brake", em_data, queue_size=1)
        self.em_msg = em_data()

        self.em_flag = False

        self.min_dist = float("inf")
        self.steer = 0


    def scan_callback(self, msg):
        self.degrees = msg.intensities
        self.ranges = msg.ranges

        for i, r in enumerate(self.ranges):
            # 차량의 전방을 0도를 기준으로 왼쪽으로 30도, 오른쪽으로 30도를 주시
            if (i >= 0 and i <= 30) or (i >= 330 and i <=360):
                if self.min_dist > r:
                    self.min_dist = r

        # 1.8M 보다 거리가 가깝다라는 것이 판정이 되었을 때 정지 플래그 활성화
        if self.min_dist < 1.8:
            self.em_flag = True
        else:
            self.em_flag = False
        
        self.em_msg.stop = self.em_flag
        self.em_msg.min_dist = self.min_dist
        self.em_stop_pub.publish(self.em_msg)
        self.min_dist = float("inf")
    
    def ctrlcallback(self, msg):
        self.steer = msg.steer



if __name__ == "__main__":
    rospy.init_node("em_brake", anonymous=True)

    emb = EM_BRAKE()
    rospy.spin()