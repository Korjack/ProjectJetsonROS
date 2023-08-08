#!/usr/bin/python3

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class Global_Path:
    def __init__(self):
        rospy.init_node("path_pub", anonymous=True)
        
        self.global_path_pub = rospy.Publisher("/global_path", Path, queue_size=1)

        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = "/map"

        self.f = open("./path4.txt", "r").readlines()

        for line in self.f:
            line = line.replace(" ", "")
            x, y = line.split(",")
            
            read_pose = PoseStamped()
            read_pose.pose.position.x = float(x)
            read_pose.pose.position.y = float(y)
            read_pose.pose.orientation.w = 1

            self.global_path_msg.poses.append(read_pose)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.global_path_pub.publish(self.global_path_msg)

            rate.sleep()



if __name__ == "__main__":

    try:
        global_p = Global_Path()
    except rospy.ROSInterruptException:
        pass