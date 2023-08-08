#!/usr/bin/python3

import rospy
import rospkg
from math import sqrt
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from gps_c94_m8p.msg import gps_data

class Global_Path:
    def __init__(self):
        rospy.init_node("path_pub", anonymous=True)
        rospy.Subscriber("/gps_fix", gps_data, self.odom_callback)
        
        self.global_path_pub = rospy.Publisher("/global_path", Path, queue_size=1)
        self.local_path_pub = rospy.Publisher("/local_path", Path, queue_size=1)

        self.global_path_msg = Path()
        self.global_path_msg.header.frame_id = "/map"

        self.is_odom = False
        self.local_path_size = 50

        rospack = rospkg.RosPack()
        path = rospack.get_path("path_planning") + "/path/path14.txt"
        self.f = open(path, "r")
        # self.f = open("./path.txt", "r").readlines()

        for line in self.f:
            line = line.replace(" ", "")
            x, y = line.split(",")
            
            read_pose = PoseStamped()
            read_pose.pose.position.x = float(x)
            read_pose.pose.position.y = float(y)
            read_pose.pose.orientation.w = 1

            self.global_path_msg.poses.append(read_pose)



    def odom_callback(self, msg):
        x = msg.utm_x
        y = msg.utm_y

        if self.is_odom:
            local_path_msg = Path()
            local_path_msg.header.frame_id = "/map"

            min_dis = float("inf")
            current_waypoint = -1

            for i, waypoint in enumerate(self.global_path_msg.poses):
                distance = sqrt(pow(x-waypoint.pose.position.x, 2) + pow(y-waypoint.pose.position.y, 2))
                
                if distance < min_dis:
                    min_dis = distance
                    current_waypoint = i

            
            if current_waypoint != -1:
                if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
                    for num in range(current_waypoint, current_waypoint + self.local_path_size):
                        tmp_pose = PoseStamped()
                        tmp_pose.pose.position.x = self.global_path_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y = self.global_path_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w = 1
                        local_path_msg.poses.append(tmp_pose)

                else:
                    for num in range(current_waypoint, len(self.global_path_msg.poses)):
                        tmp_pose = PoseStamped()

                        tmp_pose.pose.position.x = self.global_path_msg.poses[num].pose.position.x
                        tmp_pose.pose.position.y = self.global_path_msg.poses[num].pose.position.y
                        tmp_pose.pose.orientation.w = 1
                        local_path_msg.poses.append(tmp_pose)

            self.global_path_pub.publish(self.global_path_msg)
            self.local_path_pub.publish(local_path_msg)

        else:
            self.is_odom = True
            self.prev_x = x
            self.prev_y = y



if __name__ == "__main__":

    try:
        global_p = Global_Path()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass