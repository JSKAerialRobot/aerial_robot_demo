#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)

# Copyright (c) 2018, JSK Robotics Laboratory, The University of Tokyo
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import print_function
import sys
import rospy
import math
import tf
from std_msgs.msg import Empty
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from aerial_robot_msgs.msg import FlightNav
from aerial_robot_msgs.msg import MpcWaypointList
from mbzirc2020_task1_tasks.msg import PrimitiveParams
from sensor_msgs.msg import JointState

__author__ = 'shifan@jsk.imi.i.u-tokyo.ac.jp (Fan Shi)'
POS_VEL = 0
POS = 1
POS_VEL_PSI = 2
MPC = 3
class hydrusTrackingController:
    def init(self):
        rospy.init_node('hydrus_tracking_controller', anonymous=True)

        self.__hydrus_controller_freq = rospy.get_param('~hydrus_controller_freq', 100.0)
        self.__control_mode = rospy.get_param('~control_model', POS_VEL)
        self.__mpc_flag = rospy.get_param('~mpc', False)
        self.__mpc_horizon = rospy.get_param('~mpc_horizon', 1.0)

        if self.__mpc_flag:
            rospy.loginfo("[hydrus_tracking_controller] MPC mode")
            self.__control_mode = MPC
        self.__hydrus_odom = Odometry()
        self.__object_odom = Odometry()
        self.__primitive_params = PrimitiveParams()
        self.__primitive_recv_flag = False
        self.__hydrus_init_process = True

        self.__object_odom_sub = rospy.Subscriber("/vision/object_odom", Odometry, self.__objectOdomCallback)
        self.__hydrus_odom_sub = rospy.Subscriber("/uav/cog/odom", Odometry, self.__hydrusOdomCallback)
        self.__primitive_params_sub = rospy.Subscriber("/track/primitive_params", PrimitiveParams, self.__primitiveParamsCallback)

        self.__hydrus_motion_init_flag_pub = rospy.Publisher('/track/hydrus_motion_init_flag', Empty, queue_size=1)
        self.__hydrusx_start_pub = rospy.Publisher('/teleop_command/start', Empty, queue_size=1)
        self.__hydrusx_takeoff_pub = rospy.Publisher('/teleop_command/takeoff', Empty, queue_size=1)
        self.__hydrusx_nav_cmd_pub = rospy.Publisher("/uav/nav", FlightNav, queue_size=1)
        self.__hydrusx_joints_ctrl_pub = rospy.Publisher("/hydrusx/joints_ctrl", JointState, queue_size=1)
        self.__mpc_stop_flag_pub = rospy.Publisher('/mpc/stop_cmd', Bool, queue_size=1)
        self.__mpc_target_waypoints_pub = rospy.Publisher('/mpc/target_waypoints', MpcWaypointList, queue_size=1)
        rospy.sleep(1.0)
        self.__hydrus_nav_cmd = FlightNav()
        self.__hydrus_nav_cmd.header.stamp = rospy.Time.now()
        self.__hydrus_nav_cmd.header.frame_id = "world"
        self.__hydrus_nav_cmd.control_frame = self.__hydrus_nav_cmd.WORLD_FRAME
        self.__hydrus_nav_cmd.target = self.__hydrus_nav_cmd.COG
        self.__hydrus_nav_cmd.pos_xy_nav_mode = self.__hydrus_nav_cmd.POS_MODE
        self.__hydrus_nav_cmd.target_pos_x = 0.0
        self.__hydrus_nav_cmd.target_pos_y = 0.0
        self.__hydrus_nav_cmd.pos_z_nav_mode = self.__hydrus_nav_cmd.POS_MODE
        self.__hydrus_nav_cmd.target_pos_z = 2.0
        self.__hydrus_nav_cmd.psi_nav_mode = self.__hydrus_nav_cmd.POS_MODE
        self.__hydrus_nav_cmd.target_psi = -math.pi / 2.0 # 0.0

        self.__hydrus_joints_ctrl_msg = JointState()
        self.__hydrus_joints_ctrl_msg.name.append("joint1")
        self.__hydrus_joints_ctrl_msg.name.append("joint2")
        self.__hydrus_joints_ctrl_msg.name.append("joint3")
        self.__hydrus_joints_ctrl_msg.position.append(math.pi / 2.0)
        self.__hydrus_joints_ctrl_msg.position.append(math.pi / 2.0)
        self.__hydrus_joints_ctrl_msg.position.append(math.pi / 2.0)

        ## hydrusx start and takeoff
        if self.__hydrus_init_process:
            rospy.sleep(1.0)
            self.__hydrusx_start_pub.publish(Empty())
            rospy.sleep(1.0)
            self.__hydrusx_takeoff_pub.publish(Empty())
            rospy.loginfo("Hydrus arming and takeoff command is sent.")
            rospy.sleep(21.0)
            rospy.loginfo("Hydrus takeoff finsihed.")
            self.__hydrusx_nav_cmd_pub.publish(self.__hydrus_nav_cmd)
            rospy.sleep(9.0)
            rospy.loginfo("Hydrus moved to initial position.")
            if self.__control_mode == MPC:
                mpc_stop_msg = Bool()
                mpc_stop_msg.data = True
                self.__mpc_stop_flag_pub.publish(mpc_stop_msg)
                rospy.sleep(0.2)
                self.__sendMpcTargetOdom([0.0, 0.0, 0.0], self.__mpc_horizon)
                rospy.sleep(0.2)
                mpc_stop_msg.data = False
                self.__mpc_stop_flag_pub.publish(mpc_stop_msg)
                rospy.loginfo("Start MPC control.")
                rospy.sleep(0.2)
                self.__mpc_start_odom = self.__hydrus_odom

        self.__hydrus_motion_init_flag_pub.publish(Empty())
        rospy.Timer(rospy.Duration(1.0 / self.__hydrus_controller_freq), self.__hydrusControllerCallback)

    def __hydrusControllerCallback(self, event):
        if not self.__primitive_recv_flag:
            return
        ## send command
        cur_time = rospy.get_time()
        target_pos = self.__getPositionFromPrimitive(cur_time)
        target_vel = self.__getVelocityFromPrimitive(cur_time)

        ## velocity control
        if self.__control_mode == POS_VEL or self.__control_mode == POS_VEL_PSI:
            p_para = 0.5
            self.__hydrus_nav_cmd.header.stamp = rospy.Time.now()
            self.__hydrus_nav_cmd.pos_xy_nav_mode = self.__hydrus_nav_cmd.POS_VEL_MODE
            # self.__hydrus_nav_cmd.target_vel_x = target_vel[0] + p_para * (target_pos[0] - self.__hydrus_odom.pose.pose.position.x)
            self.__hydrus_nav_cmd.target_vel_x = target_vel[0]
            self.__hydrus_nav_cmd.target_pos_x = target_pos[0]

            # self.__hydrus_nav_cmd.target_vel_y = target_vel[1] + p_para * (target_pos[1] - self.__hydrus_odom.pose.pose.position.y)
            self.__hydrus_nav_cmd.target_vel_y = target_vel[1]
            self.__hydrus_nav_cmd.target_pos_y = target_pos[1]

            self.__hydrus_nav_cmd.pos_z_nav_mode = self.__hydrus_nav_cmd.POS_VEL_MODE
            # self.__hydrus_nav_cmd.target_vel_z = target_vel[2] + p_para * (target_pos[2] - self.__hydrus_odom.pose.pose.position.z)
            self.__hydrus_nav_cmd.target_vel_z = target_vel[2]
            self.__hydrus_nav_cmd.target_pos_z = target_pos[2]

            if self.__control_mode == POS_VEL_PSI:
                target_psi = self.__getPsiFromPrimitive(cur_time)
                self.__hydrus_nav_cmd.target_psi = target_psi
            self.__hydrusx_nav_cmd_pub.publish(self.__hydrus_nav_cmd)

        elif self.__control_mode == POS:
            self.__hydrus_nav_cmd.header.stamp = rospy.Time.now()
            self.__hydrus_nav_cmd.target_pos_x = target_pos[0]
            self.__hydrus_nav_cmd.target_pos_y = target_pos[1]
            self.__hydrus_nav_cmd.target_pos_z = target_pos[2]
            self.__hydrusx_nav_cmd_pub.publish(self.__hydrus_nav_cmd)

        elif self.__control_mode == MPC:
            # future 1 seconds waypoints
            self.__sendMpcWaypointsFromPrimitive()


        if self.__primitive_params.grap_flag:
            ## todo: better joints trajectory planning
            ## case 1: open
            if cur_time - self.__primitive_params.header.stamp.to_sec() < self.__primitive_params.period / 2.0:
                self.__hydrus_joints_ctrl_msg.position[0] = 1.1
                self.__hydrus_joints_ctrl_msg.position[2] = 1.1
                self.__hydrusx_joints_ctrl_pub.publish(self.__hydrus_joints_ctrl_msg)
            ## case 2: close
            else:
                self.__hydrus_joints_ctrl_msg.position[0] = math.pi / 2.0
                self.__hydrus_joints_ctrl_msg.position[2] = math.pi / 2.0
                self.__hydrusx_joints_ctrl_pub.publish(self.__hydrus_joints_ctrl_msg)

    def __objectOdomCallback(self, msg):
        self.__object_odom = msg

    def __hydrusOdomCallback(self, msg):
        self.__hydrus_odom = msg

    def __primitiveParamsCallback(self, msg):
        self.__primitive_params = msg
        self.__primitive_recv_flag = True

    def __getPositionFromPrimitive(self, time):
        t = time - self.__primitive_params.header.stamp.to_sec()
        if t > self.__primitive_params.period:
            # rospy.logwarn("[hydrus_tracking_controller] Time is out of the bound")
            t = self.__primitive_params.period
        pos = [0.0, 0.0, 0.0]
        for i in range(0, self.__primitive_params.order):
            pos[0] += math.pow(t, i) * self.__primitive_params.x_params[i]
            pos[1] += math.pow(t, i) * self.__primitive_params.y_params[i]
            pos[2] += math.pow(t, i) * self.__primitive_params.z_params[i]
        return pos

    def __getVelocityFromPrimitive(self, time):
        ## rospy.get_time()
        t = time - self.__primitive_params.header.stamp.to_sec()
        if t > self.__primitive_params.period:
            # rospy.logwarn("[hydrus_tracking_controller] Time is out of the bound")
            t = self.__primitive_params.period
        elif t < 0.0:
            # rospy.logwarn("[hydrus_tracking_controller] Time is lower than 0")
            t = 0.0
        vel = [0.0, 0.0, 0.0]
        for i in range(1, self.__primitive_params.order):
            vel[0] += math.pow(t, i - 1) * i * self.__primitive_params.x_params[i]
            vel[1] += math.pow(t, i - 1) * i * self.__primitive_params.y_params[i]
            vel[2] += math.pow(t, i - 1) * i * self.__primitive_params.z_params[i]
        return vel

    def __getPsiFromPrimitive(self, time):
        t = time - self.__primitive_params.header.stamp.to_sec()
        if t > self.__primitive_params.period:
            # rospy.logwarn("[hydrus_tracking_controller] Time is out of the bound")
            t = self.__primitive_params.period
        psi = 0.0
        for i in range(0, self.__primitive_params.order):
            psi += math.pow(t, i) * self.__primitive_params.psi_params[i]
        return psi

    def __sendMpcTargetOdom(self, pos_offset, period):
        mpc_waypoints = MpcWaypointList()
        mpc_waypoints.mode = mpc_waypoints.FULL
        mpc_waypoints.header.stamp = rospy.Time.now()

        mpc_waypoints.list.append(Odometry())
        mpc_waypoints.list[0].header.stamp = rospy.Time.now() + rospy.Duration(period)
        mpc_waypoints.list[0].pose.pose.position.x = self.__hydrus_odom.pose.pose.position.x + pos_offset[0]
        mpc_waypoints.list[0].pose.pose.position.y = self.__hydrus_odom.pose.pose.position.y + pos_offset[1]
        mpc_waypoints.list[0].pose.pose.position.z = self.__hydrus_odom.pose.pose.position.z + pos_offset[2]
        current_quaternion = (
            self.__hydrus_odom.pose.pose.orientation.x,
            self.__hydrus_odom.pose.pose.orientation.y,
            self.__hydrus_odom.pose.pose.orientation.z,
            self.__hydrus_odom.pose.pose.orientation.w)
        current_euler = tf.transformations.euler_from_quaternion(current_quaternion)
        roll = current_euler[0]
        pitch = current_euler[1]
        yaw = current_euler[2]

        target_quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        mpc_waypoints.list[0].pose.pose.orientation.x = target_quaternion[0]
        mpc_waypoints.list[0].pose.pose.orientation.y = target_quaternion[1]
        mpc_waypoints.list[0].pose.pose.orientation.z = target_quaternion[2]
        mpc_waypoints.list[0].pose.pose.orientation.w = target_quaternion[3]

        mpc_waypoints.list[0].twist.twist.linear.x = 0.0
        mpc_waypoints.list[0].twist.twist.linear.y = 0.0
        mpc_waypoints.list[0].twist.twist.linear.z = 0.0
        mpc_waypoints.list[0].twist.twist.angular.x = 0.0
        mpc_waypoints.list[0].twist.twist.angular.y = 0.0
        mpc_waypoints.list[0].twist.twist.angular.z = 0.0

        self.__mpc_target_waypoints_pub.publish(mpc_waypoints)

    def __sendMpcWaypointsFromPrimitive(self):
        candidate = 21
        time_gap = self.__mpc_horizon / (candidate - 1.0)
        mpc_waypoints = MpcWaypointList()
        mpc_waypoints.mode = mpc_waypoints.FULL
        mpc_waypoints.header.stamp = rospy.Time.now()

        for i in range(0, candidate):
            mpc_waypoints.list.append(Odometry())
            mpc_waypoints.list[i].header.stamp = mpc_waypoints.header.stamp + rospy.Duration(time_gap * i)
            current_time = mpc_waypoints.list[i].header.stamp.to_sec()
            pos = self.__getPositionFromPrimitive(current_time)
            mpc_waypoints.list[i].pose.pose.position.x = pos[0]
            mpc_waypoints.list[i].pose.pose.position.y = pos[1]
            mpc_waypoints.list[i].pose.pose.position.z = pos[2]
            current_quaternion = (
                self.__mpc_start_odom.pose.pose.orientation.x,
                self.__mpc_start_odom.pose.pose.orientation.y,
                self.__mpc_start_odom.pose.pose.orientation.z,
                self.__mpc_start_odom.pose.pose.orientation.w)
            current_euler = tf.transformations.euler_from_quaternion(current_quaternion)
            roll = current_euler[0]
            pitch = current_euler[1]
            yaw = current_euler[2]
            ## target yaw from primitive
            ## yaw = self.__getPsiFromPrimitive(current_time)

            target_quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
            mpc_waypoints.list[i].pose.pose.orientation.x = target_quaternion[0]
            mpc_waypoints.list[i].pose.pose.orientation.y = target_quaternion[1]
            mpc_waypoints.list[i].pose.pose.orientation.z = target_quaternion[2]
            mpc_waypoints.list[i].pose.pose.orientation.w = target_quaternion[3]

            vel = self.__getVelocityFromPrimitive(current_time)
            mpc_waypoints.list[i].twist.twist.linear.x = vel[0]
            mpc_waypoints.list[i].twist.twist.linear.y = vel[1]
            mpc_waypoints.list[i].twist.twist.linear.z = vel[2]
            mpc_waypoints.list[i].twist.twist.angular.x = 0.0
            mpc_waypoints.list[i].twist.twist.angular.y = 0.0
            mpc_waypoints.list[i].twist.twist.angular.z = 0.0

        self.__mpc_target_waypoints_pub.publish(mpc_waypoints)

if __name__ == '__main__':
    try:
        ## test
        hydrus_tracking_controller = hydrusTrackingController()
        hydrus_tracking_controller.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
