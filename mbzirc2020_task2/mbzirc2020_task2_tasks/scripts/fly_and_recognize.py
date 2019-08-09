#!/usr/bin/env python

# from __future__ import print_function
import sys
import rospy
import time
from std_msgs.msg import Empty
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3Stamped
from aerial_robot_msgs.msg import FlightNav
from sensor_msgs.msg import JointState
import tf2_geometry_msgs
import tf
import tf2_ros
import numpy as np
from spinal.msg import FourAxisCommand
from copy import deepcopy
import math

# from jsk_rviz_plugins.msg import OverlayText
from jsk_recognition_msgs.msg import PolygonArray
from jsk_recognition_msgs.msg import ModelCoefficientsArray

__author__ = "kuromiya@jsk.imi.i.u-tokyo.ac.jp (Naoki Kuromiya)"


class flying_controller():
    def __init__(self):

        self.uav_odom_sub_topic_name_ = rospy.get_param("~uav_odom_sub_topic_name", "/uav/baselink/odom")


        #  publishers
        self.publish_start = rospy.Publisher("/teleop_command/start", Empty, queue_size=1)
        self.publihsh_takeoff = rospy.Publisher("/teleop_command/takeoff", Empty, queue_size=1)
        self.publish_flight = rospy.Publisher("/uav/nav", FlightNav, queue_size=1)
        self.publish_joints_ctrl = rospy.Publisher("/hydrusx/joints_ctrl", JointState, queue_size=1)
        self.publish_extra_joints_ctrl = rospy.Publisher("/hydrusx/extra_servos_ctrl", JointState, queue_size=1)
        self.publish_target_point = rospy.Publisher("/world_target_point", Point, queue_size=1)  #  target destination in world coordinate
        # self.publish_image = rospy.Publisher("/image_center", Image, queue_size=1)
        self.task_start = False
        self.task_number = 0

        #  subscribers
        self.uav_odom_sub_ = rospy.Subscriber(self.uav_odom_sub_topic_name_, Odometry, self.uavOdomCallback)
        # self.image_sub_ = rospy.Subscriber("/zed/rgb/image_rect_color", Image, self.imageCallback, queue_size=1, buff_size=2)
        # self.detection_sub_ = rospy.Subscriber("/multi_plane_estimate/output_polygon", PolygonArray, self.detectionCallback)
        # self.point_sub = rospy.Subscriber("/xy_point", Point, self.pointCallback, queue_size=1, buff_size=2)
        # self.coef_sub = rospy.Subscriber("/multi_plane_estimate/output_coefficients", ModelCoefficientsArray, self.coefCallback, queue_size=1, buff_size=2)
        self.target_sub = rospy.Subscriber("/target_object/pos", Vector3Stamped, self.targetCallback, queue_size=1, buff_size=2)
        self.target_angle_sub = rospy.Subscriber("/target_object/angle", Float64, self.targetAngleCallback, queue_size=1, buff_size=2)
        #  message instances
        self.pose = PoseStamped()
        self.empty = Empty()
        self.flight = FlightNav()
        self.joints = JointState()

        self.WAIT = 0.5

        #additional
        self.capture_image = False
        self.count = 0

        self.GLOBAL_FRAME_ = 0
        self.LOCAL_FRAME_ = 1
        #  target
        self.local_target_xy_pos_ = np.zeros(2)
        self.local_target_z_pos_ = 0.0
        self.target_xy_pos_ = np.zeros(2)
        self.target_z_pos_ = 0.0
        self.target_yaw_ = 0.0
        self.target_frame_ = self.GLOBAL_FRAME_
        self.detect_object_ = False

        self.uav_xyz_pos = np.zeros(3)
        self.move_to_target = False
        self.odom_update_flag_ = False
        self.uav_xy_global_pos_ = np.zeros(2)
        self.uav_z_pos_ = 0.0
        self.uav_yaw_ = 0.0
        self.uav_yaw_old_ = 0.0
        self.uav_yaw_overflow_ = 0
        self.uav_accumulated_yaw_ = 0.0
        self.camera_pose = [0.0]
        self.coef = []
        self.detect_count = 0

        self.working_fhase = 1
        self.task_start_time_ = rospy.Time.now()
        self.task_elapsed_time_ = rospy.Time(0)

        ##define finisher
        self.approached = False

        ##listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.listener = tf.TransformListener()

        #timer
        self.control_rate_ = rospy.get_param("~control_rate", 20)
        self.control_timer_ = rospy.Timer(rospy.Duration(1.0 / self.control_rate_), self.controlCallback)
        self.yaw_adjust = True

    def TaskStartCallback(self, msg):
        rospy.loginfo("Task Start")
        self.task_start_time_ = rospy.Time.now()
        self.task_start_ = True


    # def send_pose(self):

    # def send_flight(self):

    def takeoff(self):
        # arm and takeoff
        time.sleep(self.WAIT)
        self.publish_start.publish(self.empty)
        time.sleep(self.WAIT)
        self.publihsh_takeoff.publish(self.empty)

    def goPos(self, x, y):
        rospy.loginfo("Go to Destination")
        msg = FlightNav()
        msg.pos_xy_nav_mode = FlightNav.POS_MODE
        msg.psi_nav_mode = FlightNav.NO_NAVIGATION
        msg.pos_z_nav_mode = FlightNav.POS_MODE

        msg.header.stamp = rospy.Time.now()
        msg.control_frame = FlightNav.WORLD_FRAME
        msg.target = FlightNav.COG
        msg.target_pos_x = x
        msg.target_pos_y = y
        msg.target_pos_z = 1.8
        time.sleep(self.WAIT)
        self.publish_flight.publish(msg)
        time.sleep(self.WAIT)

    def open_joints(self):
        """open links"""
        msg = JointState()
        msg.position = [1.40, 1.40, 1.40]
        self.publish_joints_ctrl.publish(msg)
        time.sleep(3)
        msg.position = [1.25, 1.25, 1.25]
        self.publish_joints_ctrl.publish(msg)
        time.sleep(3)
        msg.position = [1.10, 1.10, 1.10]
        self.publish_joints_ctrl.publish(msg)
        time.sleep(3)


    def close_joints(self):
        """close links"""
        msg = JointState()
        msg.position = [0.80, 0.80, 0.80]
        self.publish_joints_ctrl.publish(msg)
        time.sleep(3)
        msg.position = [1.56, 1.56, 1.56]
        self.publish_joints_ctrl.publish(msg)
        time.sleep(3)

    def uavOdomCallback(self, msg):
        self.uav_xyz_pos = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
        q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        rpy = tf.transformations.euler_from_quaternion(q)
        self.uav_yaw_ = rpy[2]

        # if abs(msg.pose.pose.position.x - 0.0) < 1.0 and abs(msg.pose.pose.position.y - 5.0) < 0.5 and self.count == 0:
        #     self.approached = True
        #     camera_angle = JointState()
        #     camera_angle.position = [0.30]
        #     self.publish_extra_joints_ctrl.publish(camera_angle)
        #     time.sleep(10)


    def detectionCallback(self, msg):
        # rospy.loginfo("hello")
        if self.approached == True and self.detect_object_ != True:
            rospy.loginfo("detection mode !")
            rospy.loginfo(self.coef[0].values[2])

            x = []
            y = []
            z = []

            z_highest = 0.0
            x_best = 0.0
            y_best = 0.0
            z_best = 0.0

            for i in range(len(msg.polygons)):
                pos = msg.polygons[i].polygon.points
                for j in range(len(pos)):
                    x.append(pos[j].x)
                    y.append(pos[j].y)
                    z.append(pos[j].z)

                if z_highest < np.array(z).mean() and abs(self.coef[i].values[2]) > 0.90:
                    rospy.loginfo("update : %s", self.coef[i].values[2])
                    x_best = np.array(x).mean()
                    y_best = np.array(y).mean()
                    z_best = np.array(z).mean()
                    z_highest = np.array(z).mean()

            if len(msg.polygons) > 0 and x_best != 0.0 and y_best != 0.0 and z_best != 0.0:
                self.local_target_xy_pos_[0] = x_best
                self.local_target_xy_pos_[1] = y_best
                self.local_target_z_pos_ = z_best

                self.detect_object_ = True

            # print("local_x : {}".format(x.mean()))
            # print("local_y : {}".format(y.mean()))
            # print("local_z : {}".format(z.mean()))

    def coefCallback(self, msg):
        if self.approached == True: # and self.detect_object_ != True:
            # for i in range(len(msg.coefficients)):
            self.coef = msg.coefficients

    def controlCallback(self, event):
        nav_msg = FlightNav()
        nav_msg.pos_xy_nav_mode = FlightNav.NO_NAVIGATION
        nav_msg.psi_nav_mode = FlightNav.NO_NAVIGATION
        nav_msg.pos_z_nav_mode = FlightNav.POS_MODE

        nav_msg.header.stamp = rospy.Time.now()
        nav_msg.control_frame = FlightNav.WORLD_FRAME
        nav_msg.target = FlightNav.COG
        try:
            #get the postion of the destination in world coordinates
            if self.detect_object_ == True and self.count < 1:
                w2cam_tf = self.tfBuffer.lookup_transform('world', "zed_left_camera_optical_frame", rospy.Time(0))
                translation = (w2cam_tf.transform.translation.x, w2cam_tf.transform.translation.y, w2cam_tf.transform.translation.z)
                rotation = (w2cam_tf.transform.rotation.x, w2cam_tf.transform.rotation.y, w2cam_tf.transform.rotation.z, w2cam_tf.transform.rotation.w)
            #get 4*4 matrix representing translation and rotation
                tf_matrix = tf.TransformerROS.fromTranslationRotation(tf.TransformerROS(), translation=translation, rotation=rotation)
                center_point = np.transpose(np.array([self.local_target_xy_pos_[0], self.local_target_xy_pos_[1], self.local_target_z_pos_, 1]))
                target_point = np.matmul(tf_matrix, center_point)
                self.target_xy_pos_[0] = target_point[0]
                self.target_xy_pos_[1] = target_point[1]
                self.target_z_pos_ = target_point[2]

                print("x : {}".format(self.target_xy_pos_[0]))
                print("y : {}".format(self.target_xy_pos_[1]))
                print("z : {}".format(self.target_z_pos_))


            if self.detect_object_ == True and self.count < 1:
                rospy.loginfo("grasp mode1 !")

                #move to target

                time.sleep(5)
                nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
                nav_msg.pos_z_nav_mode = FlightNav.NO_NAVIGATION
                nav_msg.target_pos_x = self.target_xy_pos_[0] - 1.0
                nav_msg.target_pos_y = self.target_xy_pos_[1] - 1.0
                self.detect_object_ == False
                self.count += 1
                self.publish_flight.publish(nav_msg)
                time.sleep(5)
                nav_msg.pos_xy_nav_mode = FlightNav.NO_NAVIGATION
                nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
                nav_msg.target_pos_z = self.target_z_pos_ + 1.0
                self.publish_flight.publish(nav_msg)
                time.sleep(5)
                nav_msg.target_pos_z = self.target_z_pos_ + 0.50
                time.sleep(5)
                self.publish_flight.publish(nav_msg)
                nav_msg.target_pos_z = self.target_z_pos_
                time.sleep(5)
                self.publish_flight.publish(nav_msg)
                nav_msg.target_pos_z = self.target_z_pos_ - 0.15
                time.sleep(5)
                self.publish_flight.publish(nav_msg)
                self.yaw_adjust = False #ajust yaw

            elif self.detect_object_ == True and self.yaw_adjust == True and self.count == 2:
                rospy.loginfo("grasp mode2 !")
                #open mode
                self.open_joints()
                nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
                nav_msg.pos_z_nav_mode = FlightNav.NO_NAVIGATION
                nav_msg.target_pos_x = 1.05 * self.target_xy_pos_[0]
                nav_msg.target_pos_y = 1.05 * self.target_xy_pos_[1]

                self.publish_flight.publish(nav_msg)
                time.sleep(5)

                #close mode
                self.close_joints()
                time.sleep(5)

                #go to destination with grasping a object
                nav_msg.pos_xy_nav_mode = FlightNav.NO_NAVIGATION
                nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
                nav_msg.target_pos_z = self.target_z_pos_ + 2.50
                time.sleep(5)
                self.publish_flight.publish(nav_msg)



        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo_throttle(10,"no tf ")

    def pointCallback(self, msg):
        target_x = msg.x
        target_y = msg.y
        # rospy.loginfo("x : %s", target_x)
        # rospy.loginfo("y : %s", target_y)

        camera_angle = JointState()
        new_msg = FlightNav()
        new_msg.psi_nav_mode = FlightNav.NO_NAVIGATION
        new_msg.header.stamp = rospy.Time.now()
        new_msg.control_frame = FlightNav.WORLD_FRAME
        new_msg.target = FlightNav.COG
        new_msg.pos_xy_nav_mode = FlightNav.NO_NAVIGATION
        new_msg.pos_z_nav_mode = FlightNav.NO_NAVIGATION
        new_msg.target_pos_x = self.uav_xyz_pos[0]
        new_msg.target_pos_y = self.uav_xyz_pos[1]
        new_msg.target_pos_z = self.uav_xyz_pos[2]
        camera_angle.position = self.camera_pose
        self.camera_pose = camera_angle.position


        if self.approached != True:
            # if target_y > 360 + 200:
            #     new_msg.pos_z_nav_mode = FlightNav.POS_MODE
            #     camera_angle.position = [float(camera_angle.position[0]) - 0.1]  #  to see more below objects

            # elif target_y < 360 - 200 :
            #     camera_angle.position = [float(camera_angle.position[0]) + 0.1]  #  to see more upper objects
            if target_x < 640 - 80:  #  below 480
                new_msg.psi_nav_mode = FlightNav.POS_MODE
                new_msg.target_psi = self.uav_yaw_ + 0.15

            elif target_x > 640 + 80:  #  over 800
                new_msg.psi_nav_mode = FlightNav.POS_MODE
                new_msg.target_psi = self.uav_yaw_ - 0.15

            else:
                new_msg.pos_xy_nav_mode = FlightNav.POS_MODE
                cam_xy_angle = 0.0
                try:
                    #get the postion of the destination in world coordinates
                    w2cam_tf = self.tfBuffer.lookup_transform('world', "zed_left_camera_frame", rospy.Time(0))
                    translation = (w2cam_tf.transform.translation.x, w2cam_tf.transform.translation.y, w2cam_tf.transform.translation.z)
                    rotation = np.array([w2cam_tf.transform.rotation.x, w2cam_tf.transform.rotation.y, w2cam_tf.transform.rotation.z, w2cam_tf.transform.rotation.w])
                    rpy = tf.transformations.euler_from_quaternion(rotation)
                    cam_xy_angle = rpy[2]
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.loginfo_throttle(10,"no tf ")


                move_distance = 0.5
                trans_x = math.cos(cam_xy_angle) * move_distance
                trans_y = math.sin(cam_xy_angle) * move_distance
                # rospy.loginfo("x : %s", trans_x)
                # rospy.loginfo("y : %s", trans_y)

                new_msg.pos_xy_nav_mode = FlightNav.POS_MODE
                new_msg.pos_z_nav_mode = FlightNav.NO_NAVIGATION
                new_msg.target_pos_x += trans_x
                new_msg.target_pos_y += trans_y
                self.camera_pose = camera_angle.position

                if target_x < 640 - 80:  #  below 560
                    new_msg.psi_nav_mode = FlightNav.POS_MODE
                    new_msg.target_psi = self.uav_yaw_ + 0.1

                elif target_x > 640 + 80:  #  over 720
                    new_msg.psi_nav_mode = FlightNav.POS_MODE
                    new_msg.target_psi = self.uav_yaw_ - 0.1

            self.publish_flight.publish(new_msg)

        elif self.yaw_adjust == False:
            rospy.loginfo("yaw adjust mode !")
            rospy.loginfo("count : %s", self.count)
            if self.count == 1:
                rospy.loginfo("camera moves !")
                camera_angle = JointState()
                camera_angle.position = [1.20]
                self.publish_extra_joints_ctrl.publish(camera_angle)
                time.sleep(10)
                self.count += 1

            if target_x < 250 - 40:  #  below 600
                new_msg.psi_nav_mode = FlightNav.POS_MODE
                new_msg.target_psi = self.uav_yaw_ + 0.1
                self.publish_flight.publish(new_msg)

            elif target_x > 250 + 40:  #  over 680
                new_msg.psi_nav_mode = FlightNav.POS_MODE
                new_msg.target_psi = self.uav_yaw_ - 0.1
                self.publish_flight.publish(new_msg)

            else:
                rospy.loginfo("yaw adjust finish !")
                self.yaw_adjust = True

    def targetCallback(self, msg):
        new_msg = FlightNav()
        new_msg.pos_xy_nav_mode = FlightNav.POS_MODE
        new_msg.psi_nav_mode = FlightNav.NO_NAVIGATION
        new_msg.pos_z_nav_mode = FlightNav.POS_MODE

        new_msg.header.stamp = rospy.Time.now()
        new_msg.control_frame = FlightNav.WORLD_FRAME
        new_msg.target = FlightNav.COG
        new_msg.target_pos_x = 0.0
        new_msg.target_pos_y = 0.0
        new_msg.target_pos_z = 1.8
        new_msg.target_psi = 0.0

        if self.working_fhase == 0:
            target_x = msg.vector.x
            target_y = msg.vector.y
            target_z = 4.0  #  to watch from the sky

            if target_y > 3.0:
                new_msg.target_pos_x = target_x
                new_msg.target_pos_y = 14.5
                new_msg.target_pos_z = target_z
                self.publish_flight.publish(new_msg)
                time.sleep(70)
                self.working_fhase += 1
                camera_angle = JointState()
                camera_angle.position = [1.57]
                self.publish_extra_joints_ctrl.publish(camera_angle)
                time.sleep(10)

            else:
                self.working_fhase = 0
        elif self.working_fhase == 1:
            target_angle = self.uav_yaw_ - self.angle - 1.57
            initial_angle = 0.0  #initial pose
            new_msg.pos_xy_nav_mode = FlightNav.NO_NAVIGATION
            new_msg.psi_nav_mode = FlightNav.POS_MODE
            new_msg.pos_z_nav_mode = FlightNav.NO_NAVIGATION
            new_msg.target_psi = initial_angle
            self.publish_flight.publish(new_msg)
            print("initial!")
            time.sleep(10)
            target_x = msg.vector.x
            target_y = msg.vector.y
            target_z = msg.vector.z + 1.0  #  to watch from the sky

            new_msg.pos_xy_nav_mode = FlightNav.NO_NAVIGATION
            new_msg.psi_nav_mode = FlightNav.POS_MODE
            new_msg.pos_z_nav_mode = FlightNav.NO_NAVIGATION
            new_msg.target_psi = target_angle
            if abs(self.angle) > 0.1:
                self.publish_flight.publish(new_msg)
                print("rotate")
                time.sleep(15)
            new_msg.pos_xy_nav_mode = FlightNav.POS_MODE
            new_msg.psi_nav_mode = FlightNav.NO_NAVIGATION
            new_msg.pos_z_nav_mode = FlightNav.POS_MODE
            new_msg.target_pos_x = target_x
            new_msg.target_pos_y = target_y
            new_msg.target_pos_z = 1.5
            print(target_x)
            print(target_y)
            print(target_z)
            if msg.vector.z > 0.30:
                self.publish_flight.publish(new_msg)
                print("move!")
                self.working_fhase += 1
                time.sleep(10)
            # new_msg.target_pos_x = target_x
            # new_msg.target_pos_y = target_y
            # new_msg.target_pos_z = target_z - 0.5
            # self.publish_flight.publish(new_msg)
            # time.sleep(10)

            # new_msg.target_pos_x = target_x
            # new_msg.target_pos_y = target_y
            # new_msg.target_pos_z = target_z - 0.5
            # self.publish_flight.publish(new_msg)
            # time.sleep(10)

    def targetAngleCallback(self, msg):
        self.angle = msg.data





if __name__ == "__main__":
    rospy.init_node("task_2_fly_and_recognize", anonymous=True)
    controller = flying_controller()
    rospy.spin()
    # time.sleep(5)
    # controller.goPos(2.0, 2.0)
    # time.sleep(5)
    # controller.open_joints()
    # controller.close_joints()


    # controller.goPos(2.0, 1.0)
    # controller.goPos(2.0, 2.0)
    # controller.goPos(3.0, 3.0)

    # print("finished")
