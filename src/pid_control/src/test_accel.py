#!/usr/bin/python3

import rospy
import time

from jetson_msg.msg import driving_msg

class testAccel:
    def __init__(self):
        rospy.init_node("test_accel", anonymous=True)

        self.ac_pub = rospy.Publisher("/ctrl_cmd", driving_msg, queue_size=1)
        self.ac_msg = driving_msg()


        while not rospy.is_shutdown():
            self.ac_msg.left = 5.0
            self.ac_msg.right = 5.0

            self.ac_pub.publish(self.ac_msg)


if __name__ == "__main__":
    ta = testAccel()
            
