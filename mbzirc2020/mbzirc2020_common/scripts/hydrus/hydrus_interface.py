import rospy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import FlightNav
import numpy as np
import ros_numpy as ros_np
from tf.transformations import *
from std_msgs.msg import Empty
import math

class HydrusInterface:
    def __init__(self):
        self.joint_state_sub_ = rospy.Subscriber('hydrusx/joint_states', JointState, self.jointStateCallback)
        self.joint_ctrl_pub_ = rospy.Publisher('hydrusx/joints_ctrl', JointState, queue_size = 1)
        self.cog_odom_sub_ = rospy.Subscriber('uav/cog/odom', Odometry, self.cogOdomCallback)
        self.baselink_odom_sub_ = rospy.Subscriber('uav/baselink/odom', Odometry, self.baselinkOdomCallback)
        self.nav_pub_ = rospy.Publisher('uav/nav', FlightNav, queue_size = 1)
        self.start_pub_ = rospy.Publisher('teleop_command/start', Empty, queue_size = 1)
        self.takeoff_pub_ = rospy.Publisher('teleop_command/takeoff', Empty, queue_size = 1)
        self.land_pub_ = rospy.Publisher('teleop_command/land', Empty, queue_size = 1)
        self.force_landing_pub_ = rospy.Publisher('teleop_command/force_landing', Empty, queue_size = 1)

        self.joint_update_freq_ = rospy.get_param("~joint_update_freq", 20)

        self.joint_state_ = None
        self.cog_odom_ = None
        self.baselink_odom_ = None

    def jointStateCallback(self, msg):
        self.joint_state_ = msg

    def cogOdomCallback(self, msg):
        self.cog_odom_ = msg

    def baselinkOdomCallback(self, msg):
        self.baselink_odom_ = msg

    #time [ms]
    def setJointAngle(self, target_joint_state, time = 1000):
        joint_seq_len = int(time * self.joint_update_freq_ / 1000.0)
        joint_seq = []

        if joint_seq_len > 1:
            for position, name in zip(target_joint_state.position, target_joint_state.name):
                current_position = self.joint_state_.position[self.joint_state_.name.index(name)]
                joint_seq.append(np.linspace(current_position, position, num = joint_seq_len, endpoint = True))
            joint_seq = np.stack(joint_seq).transpose()
        else:
            joint_seq = [target_joint_state.position]

        joint_msg = JointState()
        joint_msg.name = target_joint_state.name

        for joint_pos in joint_seq:
            joint_msg.header.stamp = rospy.Time.now()
            joint_msg.position = joint_pos
            self.joint_ctrl_pub_.publish(joint_msg)
            rospy.sleep(1.0 / self.joint_update_freq_)

    def getJointState(self):
        return self.joint_state_

    def start(self):
        self.start_pub_.publish()

    def takeoff(self):
        self.takeoff_pub_.publish()

    def startAndTakeoff(self):
        self.start()
        rospy.sleep(0.5)
        self.takeoff()

    def land(self):
        self.land_pub_.publish()

    def forceLanding(self):
        self.force_landing_pub_.publish()

    def getBaselinkOdom(self):
        return self.baselink_odom_

    def getCogOdom(self):
        return self.cog_odom_

    def getBaselinkPos(self):
        return ros_np.numpify(self.baselink_odom_.pose.pose.position)

    def getBaselinkRot(self):
        return quaternion_matrix(ros_np.numpify(self.baselink_odom_.pose.pose.orientation))

    def getBaselinkRPY(self):
        return euler_from_quaternion(ros_np.numpify(self.baselink_odom_.pose.pose.orientation))

    def getBaselinkLinearVel(self):
        return ros_np.numpify(self.baselink_odom_.twist.twist.linear)

    def getCogPos(self):
        return ros_np.numpify(self.cog_odom_.pose.pose.position)

    def getCogRot(self):
        return quaternion_matrix(ros_np.numpify(self.cog_odom_.pose.pose.orientation))

    def getCogRPY(self):
        return euler_from_quaternion(ros_np.numpify(self.cog_odom_.pose.pose.orientation))

    def getCogLinearVel(self):
        return ros_np.numpify(self.cog_odom_.twist.twist.linear)

    #navigation
    def noNavigation(self):
        nav_msg = FlightNav()
        nav_msg.header.stamp = rospy.Time.now()
        nav_msg.pos_xy_nav_mode = FlightNav.NO_NAVIGATION
        nav_msg.psi_nav_mode = FlightNav.NO_NAVIGATION
        nav_msg.pos_z_nav_mode = FlightNav.NO_NAVIGATION
        self.navigation(nav_msg)

    def goPos(self, frame, target_xy, target_z, target_yaw):
        nav_msg = FlightNav()
        if frame == 'global':
            nav_msg.control_frame = nav_msg.WORLD_FRAME
        elif frame == 'local':
            nav_msg.control_frame = nav_msg.LOCAL_FRAME
        else:
            rospy.logerr('invalid frame %s' % (frame))
            return

        nav_msg.header.stamp = rospy.Time.now()
        nav_msg.target = FlightNav.BASELINK
        nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
        nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
        nav_msg.psi_nav_mode = FlightNav.POS_MODE
        nav_msg.target_pos_x = target_xy[0]
        nav_msg.target_pos_y = target_xy[1]
        nav_msg.target_psi = target_yaw
        nav_msg.target_pos_z = target_z

        self.nav_pub_.publish(nav_msg)

    def isConvergent(self, frame, target_xy, target_z, target_yaw, pos_conv_thresh, yaw_conv_thresh, vel_conv_thresh):
        current_xy = self.getBaselinkPos()[0:2]
        current_z = self.getBaselinkPos()[2]
        current_yaw = self.getBaselinkRPY()[2]
        current_vel = self.getBaselinkLinearVel()

        if frame == 'global':
            delta_pos = np.array([target_xy[0] - current_xy[0], target_xy[1] - current_xy[1], target_z - current_z])
        elif frame == 'local':
            delta_pos = np.array([target_xy[0], target_xy[1], target_z - current_z])
        else:
            rospy.logerr('invalid frame %s' % (frame))
            return

        delta_yaw = target_yaw - current_yaw
        if delta_yaw > np.pi:
            delta_yaw -= np.pi * 2
        elif delta_yaw < -np.pi:
            delta_yaw += np.pi * 2

        if np.linalg.norm(delta_pos) < pos_conv_thresh and abs(delta_yaw) < yaw_conv_thresh and np.linalg.norm(current_vel) < vel_conv_thresh:
            return True
        else:
            return False

    def goPosWaitConvergence(self, frame, target_xy, target_z, target_yaw, pos_conv_thresh = 0.1, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1, timeout = 30):
        self.goPos(frame, target_xy, target_z, target_yaw)
        start_time = rospy.get_time()

        while not self.isConvergent(frame, target_xy, target_z, target_yaw, pos_conv_thresh, yaw_conv_thresh, vel_conv_thresh):
            elapsed_time = rospy.get_time() - start_time
            if elapsed_time > timeout:
                return False
            rospy.sleep(0.1)

        return True
