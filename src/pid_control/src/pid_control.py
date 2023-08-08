#!/usr/bin/python3

from os import path
from threading import local
import rospy
import rospkg

from math import cos, sin, pi, sqrt, pow, atan2
import numpy as np

from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from jetson_msg.msg import driving_msg
from gps_c94_m8p.msg import gps_data


class Pure_Pursuit:
    def __init__(self):
        rospy.init_node("pure_pursuit", anonymous=True)

        rospy.Subscriber("/local_path", Path, self.path_callback)
        rospy.Subscriber("/gps_fix", gps_data, self.odom_callback)

        self.ctrl_cmd_pub = rospy.Publisher("/ctrl_cmd", driving_msg, queue_size=1)
        self.ctrl_cmd_msg = driving_msg()

        self.is_path = False
        self.is_odom = False

        self.forward_point = Point()
        self.current_position = Point()
        self.is_look_forward_point = False
        self.vehicle_length = 1
        self.lfd = 3

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            if self.is_path == True and self.is_odom == True:
                vehcle_position = self.current_position
                self.is_look_forward_point = False

                translation = [vehcle_position.x, vehcle_position.y]

                t = np.array([
                    [cos(self.vehicle_yaw), -sin(self.vehicle_yaw), translation[0]],
                    [sin(self.vehicle_yaw), cos(self.vehicle_yaw), translation[1]],
                    [0, 0, 1]
                ])

                det_t = np.array([
                    [t[0][0], t[1][0], -(t[0][0]*translation[0] + t[1][0]*translation[1])],
                    [t[0][1], t[1][1], -(t[0][1]*translation[0] + t[1][1]*translation[1])],
                    [0, 0, 1]
                ])

                for num, i in enumerate(self.path.poses):
                    path_point = i.pose.position

                    global_path_point = [path_point.x, path_point.y, 1]
                    local_path_point = det_t.dot(global_path_point)

                    if local_path_point[0] > 0:
                        dis = sqrt(pow(local_path_point[0], 2) + pow(local_path_point[1], 2))
                        
                        if dis >= self.lfd:
                            self.forward_point = path_point
                            self.is_look_forward_point = True
                            break

                theta = atan2(local_path_point[1], local_path_point[0])

                if self.is_look_forward_point: # 경로가 존재한다면
                    steering = atan2((2*self.vehicle_length*sin(theta)), self.lfd) # 회전을 계산하여 라디안값이 나옴
                    self.ctrl_cmd_msg.steer = steering
                    if steering > 0: # 좌측으로 움직여야한다면
                        self.ctrl_cmd_msg.left = 50.0 - (steering * 30)
                        self.ctrl_cmd_msg.right = 50.0
                    else:   # 우측으로 움직여야한다면
                        self.ctrl_cmd_msg.left = 50.0
                        self.ctrl_cmd_msg.right = 50.0 + (steering * 30)
                    # print(steering)
                else: # 경로가 존재하지 않아 주행할 수 없는 상태
                    self.ctrl_cmd_msg.left = 0.0
                    self.ctrl_cmd_msg.right = 0.0

                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg) # 차량을 제어해줄 데이터 전송

            rate.sleep()



    def path_callback(self, msg):
        self.is_path = True
        self.path = msg

    def odom_callback(self, msg):
        self.is_odom = True
        self.current_position.x = msg.utm_x
        self.current_position.y = msg.utm_y
        self.vehicle_yaw = msg.heading_rad


if __name__ == "__main__":
    try:
        track = Pure_Pursuit()
    except rospy.ROSInterruptException:
        pass