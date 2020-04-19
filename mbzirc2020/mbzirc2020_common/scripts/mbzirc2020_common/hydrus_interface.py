import rospy
from sensor_msgs.msg import JointState, NavSatFix
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import FlightNav
import numpy as np
import ros_numpy as ros_np
from tf.transformations import *
from std_msgs.msg import Empty
import math
from aerial_robot_model.srv import AddExtraModule, AddExtraModuleRequest
from std_msgs.msg import UInt8
from jsk_rviz_plugins.msg import OverlayText
from spinal.msg import Gps
from std_srvs.srv import SetBool, SetBoolRequest
from gps_utils import *

class HydrusInterface:
    def __init__(self, debug_view = False):
        self.debug_view_ = debug_view
        self.joint_state_ = None
        self.cog_odom_ = None
        self.baselink_odom_ = None
        self.flight_state_ = None
        self.target_xy_ = None
        self.target_z_ = None
        self.target_yaw_ = None

        self.xy_pos_offset_ = [0, 0]
        self.gps_lat_lon_ = None

        #flight state
        self.ARM_OFF_STATE = 0
        self.START_STATE = 1
        self.ARM_ON_STATE = 2
        self.TAKEOFF_STATE = 3
        self.LAND_STATE = 4
        self.HOVER_STATE = 5
        self.STOP_STATE = 6

        self.joint_state_sub_ = rospy.Subscriber('hydrusx/joint_states', JointState, self.jointStateCallback)
        self.joint_ctrl_pub_ = rospy.Publisher('hydrusx/joints_ctrl', JointState, queue_size = 1)
        self.extra_joint_ctrl_pub_ = rospy.Publisher('hydrusx/extra_servos_ctrl', JointState, queue_size = 1)
        self.cog_odom_sub_ = rospy.Subscriber('uav/cog/odom', Odometry, self.cogOdomCallback)
        self.baselink_odom_sub_ = rospy.Subscriber('uav/baselink/odom', Odometry, self.baselinkOdomCallback)
        self.nav_pub_ = rospy.Publisher('uav/nav', FlightNav, queue_size = 1)
        self.start_pub_ = rospy.Publisher('teleop_command/start', Empty, queue_size = 1)
        self.takeoff_pub_ = rospy.Publisher('teleop_command/takeoff', Empty, queue_size = 1)
        self.land_pub_ = rospy.Publisher('teleop_command/land', Empty, queue_size = 1)
        self.force_landing_pub_ = rospy.Publisher('teleop_command/force_landing', Empty, queue_size = 1)
        self.halt_pub_ = rospy.Publisher('teleop_command/halt', Empty, queue_size = 1)
        self.add_extra_module_client_ = rospy.ServiceProxy('hydrusx/add_extra_module', AddExtraModule)
        self.flight_state_sub_ = rospy.Subscriber('flight_state', UInt8, self.flightStateCallback)
        self.ros_gps_sub_ = rospy.Subscriber('fix', NavSatFix, self.rosGpsCallback)
        self.gps_sub_ = rospy.Subscriber('gps', Gps, self.gpsCallback)
        self.set_joint_torque_client_ = rospy.ServiceProxy('/hydrusx/joints/torque_enable', SetBool)

        if self.debug_view_:
            self.nav_debug_pub_ = rospy.Publisher('~nav_debug', OverlayText, queue_size = 1)

        self.joint_update_freq_ = rospy.get_param("~joint_update_freq", 20)

    def rosGpsCallback(self, msg):
        self.gps_lat_lon_ = [msg.latitude, msg.longitude]

    def gpsCallback(self, msg):
        self.gps_lat_lon_ = msg.location

    def jointStateCallback(self, msg):
        self.joint_state_ = msg

    def cogOdomCallback(self, msg):
        msg.pose.pose.position.x -= self.xy_pos_offset_[0]
        msg.pose.pose.position.y -= self.xy_pos_offset_[1]
        self.cog_odom_ = msg

    def baselinkOdomCallback(self, msg):
        msg.pose.pose.position.x -= self.xy_pos_offset_[0]
        msg.pose.pose.position.y -= self.xy_pos_offset_[1]
        self.baselink_odom_ = msg

        if self.target_xy_ is None:
            self.target_xy_ = self.getBaselinkPos()[0:2]
        if self.target_z_ is None:
            self.target_z_ = self.getBaselinkPos()[2]
        if self.target_yaw_ is None:
            self.target_yaw_ = self.getBaselinkRPY()[2]

    def flightStateCallback(self, msg):
        self.flight_state_ = msg.data

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

    def setExtraJointAngle(self, target_joint_state, time = 1000):
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
            self.extra_joint_ctrl_pub_.publish(joint_msg)
            rospy.sleep(1.0 / self.joint_update_freq_)

    def setJointTorque(self, state):
        req = SetBoolRequest()
        req.data = state
        try:
            self.set_joint_torque_client_(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def setXYPosOffset(self, xy_pos_offset_):
        self.xy_pos_offset_ = xy_pos_offset_

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

    def halt(self):
        self.halt_pub_.publish()

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

    def getFlightState(self):
        return self.flight_state_

    def getTargetXY(self):
        return self.target_xy_

    def getTargetZ(self):
        return self.target_z_

    def getTargetYaw(self):
        return self.target_yaw_

    #navigation
    def noNavigation(self):
        nav_msg = FlightNav()
        nav_msg.header.stamp = rospy.Time.now()
        nav_msg.pos_xy_nav_mode = FlightNav.NO_NAVIGATION
        nav_msg.psi_nav_mode = FlightNav.NO_NAVIGATION
        nav_msg.pos_z_nav_mode = FlightNav.NO_NAVIGATION
        self.navigation(nav_msg)

    def goPos(self, frame, target_xy, target_z, target_yaw, gps_mode=False):
        nav_msg = FlightNav()
        if frame == 'global':
            nav_msg.control_frame = nav_msg.WORLD_FRAME
        elif frame == 'local':
            nav_msg.control_frame = nav_msg.LOCAL_FRAME
        else:
            rospy.logerr('invalid frame %s' % (frame))
            return

        target_yaw =  (target_yaw + np.pi) % (2 * np.pi) - np.pi

        nav_msg.header.stamp = rospy.Time.now()
        nav_msg.target = FlightNav.BASELINK
        if gps_mode:
            nav_msg.pos_xy_nav_mode = FlightNav.GPS_WAYPOINT_MODE
        else:
            nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
        nav_msg.pos_z_nav_mode = FlightNav.POS_MODE
        nav_msg.psi_nav_mode = FlightNav.POS_MODE
        if gps_mode:
            nav_msg.target_pos_x = target_xy[0]
            nav_msg.target_pos_y = target_xy[1]
        else:
            nav_msg.target_pos_x = target_xy[0] + self.xy_pos_offset_[0]
            nav_msg.target_pos_y = target_xy[1] + self.xy_pos_offset_[1]
        nav_msg.target_psi = target_yaw
        nav_msg.target_pos_z = target_z

        self.nav_pub_.publish(nav_msg)

        self.target_xy_ = target_xy
        self.target_z_ = target_z
        self.target_yaw_ = target_yaw

    def isConvergent(self, frame, target_xy, target_z, target_yaw, gps_mode, pos_conv_thresh, yaw_conv_thresh, vel_conv_thresh, att_conv_thresh=0.06):
        current_xy = self.getBaselinkPos()[0:2]
        current_z = self.getBaselinkPos()[2]
        current_yaw = self.getBaselinkRPY()[2]
        current_vel = self.getBaselinkLinearVel()
        current_rp = self.getBaselinkRPY()[0:2]

        if gps_mode:
            delta_pos = gps2xy(self.gps_lat_lon_, target_xy)
        else:
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

        if self.debug_view_:
            text = OverlayText()
            text.width = 400
            text.height = 200
            text.left = 10
            text.top = 10
            text.text_size = 12
            text.line_width = 2
            text.font = "DejaVu Sans Mono"
            text.text = 'Diff\n  pos: {:.4g}, yaw: {:.4g}, vel: {:.4g}\n  roll: {:.4g}, pitch: {:.4g}\nSetPoint\n  x: {:.11g} y: {:.11g} z: {:.4g} yaw: {:.4g}'.format(np.linalg.norm(delta_pos), abs(delta_yaw), np.linalg.norm(current_vel), abs(current_rp[0]), abs(current_rp[1]), target_xy[0], target_xy[1], target_z, target_yaw)

            self.nav_debug_pub_.publish(text)

        if np.linalg.norm(delta_pos) < pos_conv_thresh and abs(delta_yaw) < yaw_conv_thresh and np.linalg.norm(current_vel) < vel_conv_thresh and abs(current_rp[0]) < att_conv_thresh and abs(current_rp[0]) < att_conv_thresh:
            return True
        else:
            return False

    def goPosWaitConvergence(self, frame, target_xy, target_z, target_yaw, gps_mode = False, pos_conv_thresh = 0.1, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1, att_conv_thresh = 0.06, timeout = 30):
        self.goPos(frame, target_xy, target_z, target_yaw, gps_mode)
        start_time = rospy.get_time()

        while not self.isConvergent(frame, target_xy, target_z, target_yaw, gps_mode, pos_conv_thresh, yaw_conv_thresh, vel_conv_thresh, att_conv_thresh):
            elapsed_time = rospy.get_time() - start_time
            if elapsed_time > timeout:
                return False
            rospy.sleep(0.1)

        return True

    def addExtraModule(self, action, module_name, parent_link_name, transform, inertia):
        try:
            req = AddExtraModuleRequest()
            if action == 'add':
                req.action = AddExtraModuleRequest.ADD
            elif action == 'remove':
                req.action = AddExtraModuleRequest.REMOVE
            req.module_name = module_name
            req.parent_link_name = parent_link_name
            req.transform = transform
            req.inertia = inertia

            self.add_extra_module_client_(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
