#!/usr/bin/env python
import rospy
import sys
import time
import math

from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from hector_uav_msgs.srv import EnableMotors
from std_msgs.msg import Empty
from hector_uav_msgs.srv import EnableMotors

CLOCK_WISE = 1
COUNTER_CLOCK = -1

class hawkCircleMotionCommand:
    def init(self):
        rospy.init_node('hawk_circle_motion_command', anonymous=True)
        rospy.loginfo("Hawk program wait for 5 seconds to be activated");
        rospy.sleep(5.0)
        rospy.loginfo("Hawk program is activated.")

        self.__hawk_pose_command_pub = rospy.Publisher('/hawk/command/pose', PoseStamped, queue_size=1)
        self.__hawk_vel_command_pub = rospy.Publisher('/hawk/command/twist', TwistStamped, queue_size=1)
        self.__time_cnt = 0.0
        self.__timer_gap = 0.01
        self.__radius = rospy.get_param('~radius', 16.0)
        self.__fixed_height = rospy.get_param('~object_height', 2.5)
        self.__linear_vel = rospy.get_param('~linear_velocity', 5.0)
        self.__route_cross_ang = rospy.get_param('~cross_angle', math.pi / 4.0) ## cross angle of two straight line: self.__route_cross_ang * 2.0
        self.__ang_vel = self.__linear_vel / self.__radius

        self.__left_down_corner = [-self.__radius / math.tan(self.__route_cross_ang) * math.cos(self.__route_cross_ang),
                                   -self.__radius / math.tan(self.__route_cross_ang) * math.sin(self.__route_cross_ang),
                                   self.__fixed_height]
        self.__right_down_corner = [self.__radius / math.tan(self.__route_cross_ang) * math.cos(self.__route_cross_ang),
                                    -self.__radius / math.tan(self.__route_cross_ang) * math.sin(self.__route_cross_ang),
                                    self.__fixed_height]
        self.__left_circle_center = [-self.__radius / math.sin(self.__route_cross_ang), 0.0, self.__fixed_height]
        self.__right_circle_center = [self.__radius / math.sin(self.__route_cross_ang), 0.0, self.__fixed_height]
        self.__time_checkpoint = [0.0,
         2 / math.tan(self.__route_cross_ang) * self.__radius / self.__linear_vel,
         (2 / math.tan(self.__route_cross_ang) + math.pi + 2.0 * self.__route_cross_ang) * self.__radius / self.__linear_vel,
         (4 / math.tan(self.__route_cross_ang) + math.pi + 2.0 * self.__route_cross_ang) * self.__radius / self.__linear_vel,
         (4 / math.tan(self.__route_cross_ang) + 2.0 * math.pi + 4.0 * self.__route_cross_ang) * self.__radius / self.__linear_vel]
        self.__route_id = 0

        self.__init_pose = self.__left_down_corner  ## start from left-down corner
        ## hawk enable motor
        rospy.wait_for_service('/hawk/enable_motors')
        try:
            enable_hawk_motor_srv = rospy.ServiceProxy('/hawk/enable_motors', EnableMotors)
            resp1 = enable_hawk_motor_srv(True)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed")
        rospy.loginfo("Hawk motors are enabled.")
        rospy.sleep(1.0)

        self.__hawk_init_pose()
        rospy.loginfo("Hawk fly to the inital position, wait for 30 seconds.")
        rospy.sleep(30.0)
        rospy.loginfo("Hawk starts routine flight.")

        rospy.Timer(rospy.Duration(self.__timer_gap), self.__timerCallback)

    def __hawk_init_pose(self):
        hawk_pose_cmd_msg = PoseStamped()
        hawk_pose_cmd_msg.header.stamp = rospy.Time.now()
        hawk_pose_cmd_msg.header.frame_id = "world"
        hawk_pose_cmd_msg.pose.position.x = self.__init_pose[0]
        hawk_pose_cmd_msg.pose.position.y = self.__init_pose[1]
        hawk_pose_cmd_msg.pose.position.z = self.__init_pose[2]
        self.__hawk_pose_command_pub.publish(hawk_pose_cmd_msg)

    def __timerCallback(self, event):
        if self.__time_cnt >= self.__time_checkpoint[self.__route_id + 1]:
            self.__route_id += 1
            if self.__route_id >= len(self.__time_checkpoint) - 1:
                self.__time_cnt -= self.__time_checkpoint[self.__route_id]
                self.__route_id = 0
        if self.__route_id == 0:
            cur_pose, cur_vel = self.__straightMotion(self.__left_down_corner, self.__route_cross_ang)
        elif self.__route_id == 1:
            cur_pose, cur_vel = self.__circleMotion(self.__right_circle_center, COUNTER_CLOCK, math.pi / 2.0 + self.__route_cross_ang)
        elif self.__route_id == 2:
            cur_pose, cur_vel = self.__straightMotion(self.__right_down_corner, math.pi - self.__route_cross_ang)
        elif self.__route_id == 3:
            cur_pose, cur_vel = self.__circleMotion(self.__left_circle_center, CLOCK_WISE, math.pi / 2.0 - self.__route_cross_ang)
        self.__time_cnt += self.__timer_gap

        hawk_pose_cmd_msg = PoseStamped()
        hawk_pose_cmd_msg.header.stamp = rospy.Time.now()
        hawk_pose_cmd_msg.header.frame_id = "world"
        hawk_pose_cmd_msg.pose.position.x = cur_pose[0]
        hawk_pose_cmd_msg.pose.position.y = cur_pose[1]
        hawk_pose_cmd_msg.pose.position.z = cur_pose[2]
        self.__hawk_pose_command_pub.publish(hawk_pose_cmd_msg)

        # hawk_vel_cmd_msg = TwistStamped()
        # hawk_vel_cmd_msg.header.stamp = rospy.Time.now()
        # hawk_vel_cmd_msg.header.frame_id = "/world"
        # hawk_vel_cmd_msg.twist.linear.x = cur_vel[0]
        # hawk_vel_cmd_msg.twist.linear.y = cur_vel[1]
        # hawk_vel_cmd_msg.twist.linear.z = cur_vel[2]
        # self.__hawk_vel_command_pub.publish(hawk_vel_cmd_msg)

    def __circleMotion(self, circle_center, rotation, start_ang):
        time = self.__time_cnt - self.__time_checkpoint[self.__route_id]
        ang_vel = rotation * self.__ang_vel
        ang = start_ang + ang_vel * time
        cur_pose = [circle_center[0], circle_center[1], circle_center[2]]
        cur_pose[0] += self.__radius * math.cos(ang)
        cur_pose[1] += self.__radius * math.sin(ang)

        cur_vel = [0.0, 0.0, 0.0]
        cur_vel[0] = -self.__radius * math.sin(ang) * ang_vel
        cur_vel[1] = self.__radius * math.cos(ang) * ang_vel
        return cur_pose, cur_vel

    def __straightMotion(self, start_pt, direction):
        time = self.__time_cnt - self.__time_checkpoint[self.__route_id]
        cur_pose = [start_pt[0], start_pt[1], start_pt[2]]
        cur_vel = [0.0, 0.0, 0.0]
        cur_vel[0] = self.__linear_vel * math.cos(direction)
        cur_vel[1] = self.__linear_vel * math.sin(direction)
        cur_pose[0] += cur_vel[0] * time
        cur_pose[1] += cur_vel[1] * time
        return cur_pose, cur_vel

    def __diff(self, vec_a, vec_b):
        result = []
        for i in range(0, len(vec_a)):
            result.append(vec_a[i] - vec_b[i])
        return result

    def __add(self, vec_a, vec_b):
        result = []
        for i in range(0, len(vec_a)):
            result.append(vec_a[i] + vec_b[i])
        return result

if __name__ == '__main__':
    try:
        ## test
        hawk_cmd_interface = hawkCircleMotionCommand()
        hawk_cmd_interface.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
