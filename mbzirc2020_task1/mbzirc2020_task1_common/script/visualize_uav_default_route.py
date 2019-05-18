#!/usr/bin/env python
import rospy
import sys
import time
import math
import copy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class uavRouteVis:
    def init(self):
        rospy.init_node('hawk_route_visualization', anonymous=True)
        rospy.loginfo("Hawk route wait for 5 seconds to be visualized");
        rospy.sleep(5.0)
        rospy.loginfo("Hawk program is visualized.")

        self.__radius = rospy.get_param('~radius', 16.0)
        self.__fixed_height = rospy.get_param('~object_height', 2.5)
        self.__route_cross_ang = rospy.get_param('~cross_angle', math.pi / 4.0) ## cross angle of two straight line: self.__route_cross_ang * 2.0

        self.__left_down_corner = [-self.__radius / math.tan(self.__route_cross_ang) * math.cos(self.__route_cross_ang),
                                   -self.__radius / math.tan(self.__route_cross_ang) * math.sin(self.__route_cross_ang),
                                   self.__fixed_height]
        self.__right_down_corner = [self.__radius / math.tan(self.__route_cross_ang) * math.cos(self.__route_cross_ang),
                                    -self.__radius / math.tan(self.__route_cross_ang) * math.sin(self.__route_cross_ang),
                                    self.__fixed_height]
        self.__left_circle_center = [-self.__radius / math.sin(self.__route_cross_ang), 0.0, self.__fixed_height]
        self.__right_circle_center = [self.__radius / math.sin(self.__route_cross_ang), 0.0, self.__fixed_height]

        self.__uav_route_path_pub = rospy.Publisher('/guard_default_path', Path, queue_size=1)
        rospy.sleep(1.0)
        self.visualize()

    def visualize(self):
        route_msg = Path()
        route_msg.header.frame_id = "/world"
        route_msg.header.stamp = rospy.Time.now()
        route_msg.header.seq = 0
        route_pt = PoseStamped()
        ## left_down
        route_pt.pose.position.x = self.__left_down_corner[0]
        route_pt.pose.position.y = self.__left_down_corner[1]
        route_pt.pose.position.z = self.__left_down_corner[2]
        route_msg.poses.append(copy.deepcopy(route_pt))
        ## right_up
        route_pt.pose.position.x = -self.__left_down_corner[0]
        route_pt.pose.position.y = -self.__left_down_corner[1]
        route_pt.pose.position.z = self.__left_down_corner[2]
        route_msg.poses.append(copy.deepcopy(route_pt))

        ## right circle
        start_ang = math.pi - self.__route_cross_ang
        ang_range = 2.0 * (math.pi - self.__route_cross_ang)
        circle_resolution = 100
        for i in range(1, circle_resolution + 1):
            cur_ang = start_ang - i * ang_range / circle_resolution
            circle_pt = [self.__right_circle_center[0] + self.__radius * math.cos(cur_ang),
                         self.__right_circle_center[1] + self.__radius * math.sin(cur_ang),
                         self.__fixed_height]
            route_pt.pose.position.x = circle_pt[0]
            route_pt.pose.position.y = circle_pt[1]
            route_pt.pose.position.z = circle_pt[2]
            route_msg.poses.append(copy.deepcopy(route_pt))

        ## right_down
        route_pt.pose.position.x = self.__right_down_corner[0]
        route_pt.pose.position.y = self.__right_down_corner[1]
        route_pt.pose.position.z = self.__right_down_corner[2]
        route_msg.poses.append(copy.deepcopy(route_pt))

        ## left_up
        route_pt.pose.position.x = -self.__right_down_corner[0]
        route_pt.pose.position.y = -self.__right_down_corner[1]
        route_pt.pose.position.z = self.__right_down_corner[2]
        route_msg.poses.append(copy.deepcopy(route_pt))

        ## left circle
        start_ang = self.__route_cross_ang
        ang_range = 2.0 * (math.pi - self.__route_cross_ang)
        circle_resolution = 100
        for i in range(1, circle_resolution + 1):
            cur_ang = start_ang + i * ang_range / circle_resolution
            circle_pt = [self.__left_circle_center[0] + self.__radius * math.cos(cur_ang),
                         self.__left_circle_center[1] + self.__radius * math.sin(cur_ang),
                         self.__fixed_height]
            route_pt.pose.position.x = circle_pt[0]
            route_pt.pose.position.y = circle_pt[1]
            route_pt.pose.position.z = circle_pt[2]
            route_msg.poses.append(copy.deepcopy(route_pt))

        ## left_down
        route_pt.pose.position.x = self.__left_down_corner[0]
        route_pt.pose.position.y = self.__left_down_corner[1]
        route_pt.pose.position.z = self.__left_down_corner[2]
        route_msg.poses.append(copy.deepcopy(route_pt))

        self.__uav_route_path_pub.publish(route_msg)

if __name__ == '__main__':
    try:
        ## test
        hawk_route_vis = uavRouteVis()
        hawk_route_vis.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
