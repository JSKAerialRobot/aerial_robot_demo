#!/usr/bin/env python

import math
import numpy as np

import rospy

import tf
from tf.transformations import quaternion_matrix

from std_msgs.msg import Empty
from std_msgs.msg import Int8
from std_msgs.msg import UInt8
from std_msgs.msg import Float64
from geometry_msgs.msg import WrenchStamped
from aerial_robot_msgs.msg import FlightNav
from aerial_robot_msgs.msg import Pid
from dynamic_reconfigure.msg import Config,BoolParameter,DoubleParameter
from nav_msgs.msg import Odometry

from std_srvs.srv import Empty as EmptyService
from dynamic_reconfigure.srv import Reconfigure

class FTSensorFeedbackInterface:
    def __init__(self,robot_ns,sensor_takeoff,sensor_landing):
        self.robot_ns = robot_ns
        self.sensor_takeoff = sensor_takeoff
        self.sensor_landing = sensor_landing

        self.m = 4.5
        self.m_o = 1.8
        self.X_acc = np.array([0,0,0])

        self.flight_state = 0
        self.ft_sensor_feedback_flag = False

        self.commanded_takeoff = False
        self.commanded_landing = False
        self.takeoff_thre=0.05
        self.landing_thre=-0.05

        self.short_span = 25
        self.long_span = 100
        self.smooth_const_short = 2.0/(self.short_span + 1)
        self.smooth_const_long = 2.0/(self.long_span + 1)
        self.ft_ema_short_left = WrenchStamped()
        self.ft_ema_long_left = WrenchStamped()
        self.ft_ema_short_cog_left = WrenchStamped()
        self.ft_ema_long_cog_left = WrenchStamped()
        self.ft_ema_short_right = WrenchStamped()
        self.ft_ema_long_right = WrenchStamped()
        self.ft_ema_short_cog_right = WrenchStamped()
        self.ft_ema_long_cog_right = WrenchStamped()

        self.not_calib_yet = True
        self.x_unit_vector_from_cog_left = np.array([0,0,0])
        self.x_unit_vector_from_cog_right = np.array([0,0,0])
        self.debug_val1 = 0
        self.debug_val2 = 0
        self.debug_val3 = 0
        self.debug_val4 = 0
        self.debug_val5 = 0
        self.debug_val6 = 0

        self.position = np.array([0,0,0])
        self.orientation_euler_angle = [0,0,0]

        self.X_nav_W_1 = 0.8
        self.X_nav_W_2 = 1 - self.X_nav_W_1
        self.X_nav_L = 0.1
        self.X_nav_norm_maximum = 0.5
        self.X_nav_p_gain = 0.01
        self.X_nav_i_gain = 0.015
        self.X_nav_d_gain = 0.003
        self.X_nav_pid_maximum = 0.05
        self.X_nav_pid_minimum = -0.05
        self.X_nav_i_max = 0.01
        self.X_nav_i_min = -0.01
        self.X_nav_PID_controller = PIDController(K_P = self.X_nav_p_gain, K_I = self.X_nav_i_gain, K_D = self.X_nav_d_gain, maximum = self.X_nav_pid_maximum, minimum = self.X_nav_pid_minimum,i_max=self.X_nav_i_max,i_min=self.X_nav_i_min,topic_name=robot_ns + '/debug/pid/X_nav')

        self.yaw_nav_p_gain = 0.6
        self.yaw_nav_i_gain = 0.4
        self.yaw_nav_d_gain = 0.125
        self.yaw_nav_maximum = 0.78
        self.yaw_nav_minimum = -0.78
        self.yaw_nav_i_max = 0.78
        self.yaw_nav_i_min = -0.78
        self.yaw_nav_PID_controller = PIDController(K_P = self.yaw_nav_p_gain, K_I = self.yaw_nav_i_gain, K_D = self.yaw_nav_d_gain, maximum = self.yaw_nav_maximum, minimum = self.yaw_nav_minimum,i_max=self.yaw_nav_i_max,i_min=self.yaw_nav_i_min,topic_name=robot_ns + '/debug/pid/yaw_nav')

        self.start_pub = rospy.Publisher(robot_ns + '/teleop_command/start', Empty, queue_size=10)
        self.takeoff_pub = rospy.Publisher(robot_ns + '/teleop_command/takeoff', Empty, queue_size=10)
        self.landing_pub = rospy.Publisher(robot_ns + '/teleop_command/land', Empty, queue_size=10)
        self.uav_nav_pub = rospy.Publisher(robot_ns + '/uav/nav', FlightNav, queue_size=10)
        self.debug_left_ft_cog_pub = rospy.Publisher(robot_ns + '/debug/left_ft_cog',WrenchStamped, queue_size=10)
        self.debug_right_ft_cog_pub = rospy.Publisher(robot_ns + '/debug/right_ft_cog',WrenchStamped, queue_size=10)
        self.debug_ft_cog_pub = rospy.Publisher(robot_ns + '/debug/ft_cog',WrenchStamped, queue_size=10)
        self.debug_x_axis_force_pub = rospy.Publisher(robot_ns + '/debug/x_axis_force',WrenchStamped, queue_size=10)
        self.debug_ft_ema_short_left_pub = rospy.Publisher(robot_ns + '/debug/ema_short_left',WrenchStamped, queue_size=10)
        self.debug_ft_ema_short_right_pub = rospy.Publisher(robot_ns + '/debug/ema_short_right',WrenchStamped, queue_size=10)
        self.debug_ft_ema_long_left_pub = rospy.Publisher(robot_ns + '/debug/ema_long_left',WrenchStamped, queue_size=10)
        self.debug_ft_ema_long_right_pub = rospy.Publisher(robot_ns + '/debug/ema_long_right',WrenchStamped, queue_size=10)
        self.debug_ft_ema_short_cog_left_pub = rospy.Publisher(robot_ns + '/debug/ema_short_cog_left',WrenchStamped, queue_size=10)
        self.debug_ft_ema_short_cog_right_pub = rospy.Publisher(robot_ns + '/debug/ema_short_cog_right',WrenchStamped, queue_size=10)
        self.debug_X_axis_acc_inner_product_pub = rospy.Publisher(robot_ns + '/debug/X_axis_acc_inner_product', Float64, queue_size=10)
        self.debug_ft_ema_long_cog_left_pub = rospy.Publisher(robot_ns + '/debug/ema_long_cog_left',WrenchStamped, queue_size=10)
        self.debug_ft_ema_long_cog_right_pub = rospy.Publisher(robot_ns + '/debug/ema_long_cog_right',WrenchStamped, queue_size=10)
        self.debug_vector_pub = rospy.Publisher(robot_ns + '/debug/vector',WrenchStamped, queue_size=10)
        self.debug_vector2_pub = rospy.Publisher(robot_ns + '/debug/vector2',WrenchStamped, queue_size=10)
        self.debug_X_axis_movement_pub = rospy.Publisher(robot_ns + '/debug/X_axis_movement',WrenchStamped, queue_size=10)
        self.debug_yaw_pub = rospy.Publisher(robot_ns + '/debug/yaw',Float64, queue_size=10)

        self.flight_state_sub = rospy.Subscriber(robot_ns + '/flight_state', UInt8, self.FlightStateCallback)
        self.cfs_left_sub = rospy.Subscriber('cfs/data/left', WrenchStamped, self.SensorCallbackLeft)
        self.cfs_right_sub = rospy.Subscriber('cfs/data/right', WrenchStamped, self.SensorCallbackRight)
        self.odom_sub = rospy.Subscriber(robot_ns +'/uav/cog/odom', Odometry, self.OdometryCallback)
        self.ft_sensor_feedback_switch_sub = rospy.Subscriber(robot_ns + '/ft_sensor_feedback_switch', Int8, self.FTSensorFeedbackSwitchCallback)

        rospy.wait_for_service('/cfs/sensor_calib/left')
        self.calib_left = rospy.ServiceProxy('/cfs/sensor_calib/left',EmptyService)
        rospy.wait_for_service('/cfs/sensor_calib/right')
        self.calib_right = rospy.ServiceProxy('/cfs/sensor_calib/right',EmptyService)
        rospy.wait_for_service(robot_ns + '/controller/lqi/set_parameters')
        self.set_yaw_free_service = rospy.ServiceProxy(robot_ns + '/controller/lqi/set_parameters',Reconfigure)

        self.listener = tf.TransformListener()

        rospy.loginfo('init_ft_sensor_feedback_interface')

    def takeoff_landing_check(self):

        left_val = self.ft_ema_short_left.wrench.torque.x - self.ft_ema_long_left.wrench.torque.x
        right_val = self.ft_ema_short_right.wrench.torque.x - self.ft_ema_long_right.wrench.torque.x

        #debug
        self.debug_val1 = left_val
        self.debug_val2 = right_val

        # 0 == ARM_OFF_STATE
        if self.flight_state == 0 and self.commanded_takeoff == False and left_val >= self.takeoff_thre and right_val >= self.takeoff_thre:
            if self.sensor_takeoff:
                self.start_pub.publish(Empty())
                rospy.sleep(0.1)
                self.takeoff_pub.publish(Empty())
            rospy.loginfo('start takeoff !!!!!!!!!!!!!!!!!')
            rospy.loginfo('start takeoff !!!!!!!!!!!!!!!!!')
            rospy.loginfo('start takeoff !!!!!!!!!!!!!!!!!')
            rospy.loginfo('start takeoff !!!!!!!!!!!!!!!!!')
            rospy.loginfo('start takeoff !!!!!!!!!!!!!!!!!')
            rospy.loginfo('start takeoff !!!!!!!!!!!!!!!!!')
            self.commanded_takeoff = True
            self.commanded_landing = False


        # 5 == HOVER_STATE
        if self.flight_state == 5 and self.commanded_landing == False and left_val <= self.landing_thre and right_val <=  self.landing_thre:
            rospy.loginfo('start landing !!!!!!!!!!!!!!!!!')
            rospy.loginfo('start landing !!!!!!!!!!!!!!!!!')
            rospy.loginfo('start landing !!!!!!!!!!!!!!!!!')
            rospy.loginfo('start landing !!!!!!!!!!!!!!!!!')
            rospy.loginfo('start landing !!!!!!!!!!!!!!!!!')
            rospy.loginfo('start landing !!!!!!!!!!!!!!!!!')
            if self.sensor_landing:
                self.landing_pub.publish(Empty())
            self.commanded_takeoff = False
            self.commanded_landing = True

        ### publish debug
        wrench_msg = WrenchStamped()
        wrench_msg.header.frame_id = self.robot_ns + '/cog'
        wrench_msg.wrench.force.x = self.debug_val1
        wrench_msg.wrench.force.y = self.debug_val2
        wrench_msg.wrench.force.z = self.debug_val3
        wrench_msg.wrench.torque.x = self.debug_val4
        wrench_msg.wrench.torque.y = self.debug_val5
        wrench_msg.wrench.torque.z = self.debug_val6
        #self.debug_vector_pub.publish(wrench_msg)

    def ft_sensor_feedback_navigation(self):

        ave_force_cog_left = np.array([
            self.ft_ema_short_cog_left.wrench.force.x-self.ft_ema_long_cog_left.wrench.force.x,
            self.ft_ema_short_cog_left.wrench.force.y-self.ft_ema_long_cog_left.wrench.force.y,
            self.ft_ema_short_cog_left.wrench.force.z-self.ft_ema_long_cog_left.wrench.force.z])
        ave_force_cog_right = np.array([
            self.ft_ema_short_cog_right.wrench.force.x-self.ft_ema_long_cog_right.wrench.force.x,
            self.ft_ema_short_cog_right.wrench.force.y-self.ft_ema_long_cog_right.wrench.force.y,
            self.ft_ema_short_cog_right.wrench.force.z-self.ft_ema_long_cog_right.wrench.force.z])
        ave_torque_cog_left = np.array([
            self.ft_ema_short_cog_left.wrench.torque.x-self.ft_ema_long_cog_left.wrench.torque.x,
            self.ft_ema_short_cog_left.wrench.torque.y-self.ft_ema_long_cog_left.wrench.torque.y,
            self.ft_ema_short_cog_left.wrench.torque.z-self.ft_ema_long_cog_left.wrench.torque.z])
        ave_torque_cog_right = np.array([
            self.ft_ema_short_cog_right.wrench.torque.x-self.ft_ema_long_cog_right.wrench.torque.x,
            self.ft_ema_short_cog_right.wrench.torque.y-self.ft_ema_long_cog_right.wrench.torque.y,
            self.ft_ema_short_cog_right.wrench.torque.z-self.ft_ema_long_cog_right.wrench.torque.z])

        ave_force_cog = (ave_force_cog_left+ave_force_cog_right)/2
        ave_torque_cog = (ave_torque_cog_left+ave_torque_cog_right)/2

        ######################## X axis nav ###########################

        X_acc_before = self.X_acc

        X_unit_vector_from_cog = - self.x_unit_vector_from_cog_left + self.x_unit_vector_from_cog_right
        norm = math.sqrt(X_unit_vector_from_cog[0]**2 + X_unit_vector_from_cog[1]**2)
        if norm == 0:
            return

        X_unit_vector_from_cog = X_unit_vector_from_cog/norm

        # for debug
        (trans, rot) = self.listener.lookupTransform('world',self.robot_ns+'/cog', rospy.Time(0))
        X_unit_vector = np.dot(quaternion_matrix(rot)[0:3,0:3],X_unit_vector_from_cog)

        position_inner_product = (X_unit_vector[0]*self.position[0]) + (X_unit_vector[1]*self.position[1])

        movement_msg = WrenchStamped()
        movement_msg.header.frame_id = 'world'

        movement_msg.wrench.force.x = X_unit_vector[0]*position_inner_product
        movement_msg.wrench.force.y = X_unit_vector[1]*position_inner_product
        movement_msg.wrench.force.z = 0
        movement_msg.wrench.torque.x = 0
        movement_msg.wrench.torque.y = 0
        movement_msg.wrench.torque.z = position_inner_product
        self.debug_X_axis_movement_pub.publish(movement_msg)


        # force
        #x_component_of_force_left = self.x_unit_vector_from_cog_left * (self.ft_ema_short_left.wrench.force.x - self.ft_ema_long_left.wrench.force.x)
        #x_component_of_force_right = self.x_unit_vector_from_cog_right * (self.ft_ema_short_right.wrench.force.x - self.ft_ema_long_right.wrench.force.x)
        x_component_of_force_left = X_unit_vector_from_cog * (-(self.ft_ema_short_left.wrench.force.x - self.ft_ema_long_left.wrench.force.x))
        x_component_of_force_right = X_unit_vector_from_cog * (self.ft_ema_short_right.wrench.force.x - self.ft_ema_long_right.wrench.force.x)
        # torque
        x_component_of_torque_left = X_unit_vector_from_cog * (-(self.ft_ema_short_left.wrench.torque.y-self.ft_ema_long_left.wrench.torque.y))
        x_component_of_torque_right = X_unit_vector_from_cog * ((self.ft_ema_short_right.wrench.torque.y-self.ft_ema_long_right.wrench.torque.y))

        delta_f_o = self.X_nav_W_1*(x_component_of_force_left+x_component_of_force_right) + \
                    self.X_nav_W_2*(x_component_of_torque_left+x_component_of_torque_right)/self.X_nav_L

        #a = np.linalg.norm(x_component_of_force_left, ord=2)
        #b = np.linalg.norm(x_component_of_force_right, ord=2)
        #c = np.linalg.norm(x_component_of_torque_left/self.X_nav_L, ord=2)
        #d = np.linalg.norm(x_component_of_torque_right/self.X_nav_L, ord=2)
        #print(a,b,c,d)

        e_X = (1.0/2.0)*(3.0 + self.m_o/self.m)*delta_f_o

        # use self.x_unit_vector_from_cog_right as a representative vector of X_axis
        e_X_raw_inner_product = (e_X[0] * X_unit_vector_from_cog[0]) + (e_X[1] * X_unit_vector_from_cog[1])

        if e_X_raw_inner_product == 0:
            return

        e_X_pid_inner_product = self.X_nav_PID_controller.update(e_X_raw_inner_product)

        # in theory 
        self.X_acc = X_acc_before + (1.0/(2*self.m))*(3 + self.m_o/self.m)*delta_f_o + (1.0/self.m)*(e_X_pid_inner_product/e_X_raw_inner_product)*e_X
        # in use
        #self.X_acc = (1.0/(2*self.m))*(3 + self.m_o/self.m)*delta_f_o + (1.0/self.m)*(e_X_pid_inner_product/e_X_raw_inner_product)*e_X

        total_raw_norm = np.linalg.norm(self.X_acc, ord=2)
        total_clamped_norm = min(total_raw_norm, self.X_nav_norm_maximum)

        inner_product_msg = Float64()
        inner_product_msg.data = X_unit_vector_from_cog[0]*self.X_acc[0] + X_unit_vector_from_cog[1]*self.X_acc[1]
        self.debug_X_axis_acc_inner_product_pub.publish(inner_product_msg)

        if total_raw_norm == 0:
            return

        X_acc = self.X_acc*(total_clamped_norm/total_raw_norm)
        self.X_acc = X_acc

        ######################### YAW nav ###########################

        delta_yaw = (self.ft_ema_short_left.wrench.torque.y - self.ft_ema_long_left.wrench.torque.y) + (self.ft_ema_short_right.wrench.torque.y - self.ft_ema_long_right.wrench.torque.y)

        delta_yaw = self.yaw_nav_PID_controller.update(delta_yaw)

        target_yaw = self.orientation_euler_angle[2] + delta_yaw

        #################### publish nav msg ########################
        nav_msg = FlightNav()
        nav_msg.header.frame_id='global'
        # LOCAL
        nav_msg.control_frame=1
        # COG
        nav_msg.target=1
        nav_msg.pos_xy_nav_mode = FlightNav.ACC_MODE
        nav_msg.target_acc_x = X_acc[0]
        nav_msg.target_acc_y = X_acc[1]
        nav_msg.yaw_nav_mode = FlightNav.POS_MODE
        nav_msg.target_yaw = target_yaw
        self.uav_nav_pub.publish(nav_msg)

        ######################### debug ############################

        ### publish cog force sum for debug
        wrench_msg = WrenchStamped()
        wrench_msg.header.frame_id = self.robot_ns + '/cog'

        wrench_msg.wrench.force.x = ave_force_cog[0]
        wrench_msg.wrench.force.y = ave_force_cog[1]
        wrench_msg.wrench.force.z = ave_force_cog[2]
        wrench_msg.wrench.torque.x = ave_torque_cog[0]
        wrench_msg.wrench.torque.y = ave_torque_cog[1]
        wrench_msg.wrench.torque.z = ave_torque_cog[2]
        self.debug_ft_cog_pub.publish(wrench_msg)

        ### publish x component force sum for debug
        wrench_msg = WrenchStamped()
        wrench_msg.header.frame_id = self.robot_ns + '/cog'
        wrench_msg.wrench.force.x = self.X_acc[0]
        wrench_msg.wrench.force.y = self.X_acc[1]
        wrench_msg.wrench.force.z = 0
        wrench_msg.wrench.torque.x = 0
        wrench_msg.wrench.torque.y = 0
        wrench_msg.wrench.torque.z = delta_yaw
        self.debug_x_axis_force_pub.publish(wrench_msg)

        ### publish debug
        wrench_msg = WrenchStamped()
        wrench_msg.header.frame_id = self.robot_ns + '/cog'
        wrench_msg.wrench.force.x = ((1.0/(2*self.m))*(3 + self.m_o/self.m)*delta_f_o)[0]
        wrench_msg.wrench.force.y = ((1.0/(2*self.m))*(3 + self.m_o/self.m)*delta_f_o)[1]
        #wrench_msg.wrench.force.x = x_component_of_force_left[0]
        #wrench_msg.wrench.force.y = x_component_of_force_left[1]
        wrench_msg.wrench.force.z = 0
        #wrench_msg.wrench.torque.x = x_component_of_torque_left[0]
        #wrench_msg.wrench.torque.y = x_component_of_torque_left[1]
        wrench_msg.wrench.torque.z = 0
        self.debug_vector_pub.publish(wrench_msg)

        ### publish debug
        wrench_msg = WrenchStamped()
        wrench_msg.header.frame_id = self.robot_ns + '/cog'
        wrench_msg.wrench.force.x = ((1.0/self.m)*(e_X_pid_inner_product/e_X_raw_inner_product)*e_X)[0]
        wrench_msg.wrench.force.y = ((1.0/self.m)*(e_X_pid_inner_product/e_X_raw_inner_product)*e_X)[1]
        #wrench_msg.wrench.force.x = x_component_of_force_right[0]
        #wrench_msg.wrench.force.y = x_component_of_force_right[1]
        wrench_msg.wrench.force.z = 0
        #wrench_msg.wrench.torque.x = x_component_of_torque_right[0]
        #wrench_msg.wrench.torque.y = x_component_of_torque_right[1]
        wrench_msg.wrench.torque.z = 0
        self.debug_vector2_pub.publish(wrench_msg)

    def calc_ema_ft(self,y,x,alpha):

        y.header = x.header

        y.wrench.force.x = x.wrench.force.x*alpha + y.wrench.force.x*(1-alpha)
        y.wrench.force.y = x.wrench.force.y*alpha + y.wrench.force.y*(1-alpha)
        y.wrench.force.z = x.wrench.force.z*alpha + y.wrench.force.z*(1-alpha)
        y.wrench.torque.x = x.wrench.torque.x*alpha + y.wrench.torque.x*(1-alpha)
        y.wrench.torque.y = x.wrench.torque.y*alpha + y.wrench.torque.y*(1-alpha)
        y.wrench.torque.z = x.wrench.torque.z*alpha + y.wrench.torque.z*(1-alpha)

        return y

    def calc_orthogonal_projection(self,force,side):
        if side=='left':
            a = np.matrix([self.x_unit_vector_from_cog_left[0],self.x_unit_vector_from_cog_left[1]])
            P = (a.T * a)/(a * a.T)
            x_component_of_force = P * np.matrix([force[0],force[1]]).T
        elif side=='right':
            a = np.matrix([self.x_unit_vector_from_cog_right[0],self.x_unit_vector_from_cog_right[1]])
            P = (a.T * a)/(a * a.T)
            x_component_of_force = P * np.matrix([force[0],force[1]]).T

        return np.array([float(x_component_of_force[0][0]),float(x_component_of_force[1][0]),0])

    def set_yaw_free(self,flag=True):
        set_yaw_free_srv = Config()
        bool_param = BoolParameter()
        bool_param.name = 'lqi_flag'
        bool_param.value = True
        set_yaw_free_srv.bools = [bool_param]
        if flag == True:
            doubles1 = DoubleParameter()
            doubles1.name = 'yaw_p'
            doubles1.value = 0.0
            doubles2 = DoubleParameter()
            doubles2.name = 'yaw_i'
            doubles2.value = 0.0
            doubles3 = DoubleParameter()
            doubles3.name = 'yaw_d'
            doubles3.value = 0.0
            doubles = [doubles1,doubles2,doubles3]
        elif flag == False:
            doubles1 = DoubleParameter()
            doubles1.name = 'yaw_p'
            doubles1.value = 100.0
            doubles2 = DoubleParameter()
            doubles2.name = 'yaw_i'
            doubles2.value = 0.05
            doubles3 = DoubleParameter()
            doubles3.name = 'yaw_d'
            doubles3.value = 50.0
            doubles = [doubles1,doubles2,doubles3]
        
        for d in doubles:
            set_yaw_free_srv.doubles = [d]
            self.set_yaw_free_service(set_yaw_free_srv)

    def hovering_on_the_spot(self):
        nav_msg = FlightNav()
        nav_msg.header.frame_id='global'
        # LOCAL
        nav_msg.control_frame=1
        # COG
        nav_msg.target=1
        nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
        nav_msg.target_vel_x = 0.0
        nav_msg.target_vel_y = 0.0

        self.uav_nav_pub.publish(nav_msg)

    #####################################
    ############# callback ##############
    #####################################

    def FlightStateCallback(self,msg):
        self.flight_state = msg.data

        # hovering state
        if self.not_calib_yet and msg.data == 5:
            rospy.loginfo('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            rospy.loginfo('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            rospy.loginfo('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            rospy.loginfo('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            rospy.loginfo('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            rospy.loginfo('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            rospy.loginfo('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!HOVERING!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            rospy.loginfo('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            rospy.loginfo('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            rospy.loginfo('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            rospy.loginfo('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            rospy.loginfo('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            rospy.loginfo('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            rospy.loginfo('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
            #self.calib_left()
            #self.calib_right()
            #self.ft_sensor_feedback_flag = True
            self.not_calib_yet = False


    def SensorCallbackLeft(self,msg):

        self.ft_ema_short_left = self.calc_ema_ft(self.ft_ema_short_left,msg,self.smooth_const_short)
        self.ft_ema_long_left = self.calc_ema_ft(self.ft_ema_long_left,msg,self.smooth_const_long)

        self.debug_ft_ema_short_left_pub.publish(self.ft_ema_short_left)
        self.debug_ft_ema_long_left_pub.publish(self.ft_ema_long_left)


        if self.ft_sensor_feedback_flag:
            try:
                (trans, rot) = self.listener.lookupTransform(self.robot_ns+'/cog',self.robot_ns+'/ball_joint11_left', rospy.Time(0))
                force = [msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z]
                force_cog = np.dot(quaternion_matrix(rot)[0:3,0:3],np.array(force))
                torque = [msg.wrench.torque.x,msg.wrench.torque.y,msg.wrench.torque.z]
                torque_cog = np.dot(quaternion_matrix(rot)[0:3,0:3],np.array(torque))

                wrench_msg = WrenchStamped()
                wrench_msg.header = msg.header #######################
                wrench_msg.header.frame_id = self.robot_ns + '/cog'
                wrench_msg.wrench.force.x = force[0]
                wrench_msg.wrench.force.y = force[1]
                wrench_msg.wrench.force.z = force[2]
                wrench_msg.wrench.torque.x = torque_cog[0]
                wrench_msg.wrench.torque.y = torque_cog[1]
                wrench_msg.wrench.torque.z = torque_cog[2]

                self.debug_left_ft_cog_pub.publish(wrench_msg)

                self.ft_ema_short_cog_left = self.calc_ema_ft(self.ft_ema_short_cog_left,wrench_msg,self.smooth_const_short)
                self.ft_ema_long_cog_left = self.calc_ema_ft(self.ft_ema_long_cog_left,wrench_msg,self.smooth_const_long)

                self.debug_ft_ema_short_cog_left_pub.publish(self.ft_ema_short_cog_left)
                self.debug_ft_ema_long_cog_left_pub.publish(self.ft_ema_long_cog_left)

                x_unit_vector = [1,0,0]
                x_unit_vector_cog = np.dot(quaternion_matrix(rot)[0:3,0:3],np.array(x_unit_vector))
                self.x_unit_vector_from_cog_left = np.array([x_unit_vector_cog[0],x_unit_vector_cog[1],x_unit_vector_cog[2]])

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass



    def SensorCallbackRight(self,msg):

        self.ft_ema_short_right = self.calc_ema_ft(self.ft_ema_short_right,msg,self.smooth_const_short)
        self.ft_ema_long_right = self.calc_ema_ft(self.ft_ema_long_right,msg,self.smooth_const_long)

        self.debug_ft_ema_short_right_pub.publish(self.ft_ema_short_right)
        self.debug_ft_ema_long_right_pub.publish(self.ft_ema_long_right)

        if self.ft_sensor_feedback_flag:
            try:
                (trans, rot) = self.listener.lookupTransform(self.robot_ns+'/cog',self.robot_ns+'/ball_joint11_right', rospy.Time(0))
                force = [msg.wrench.force.x,msg.wrench.force.y,msg.wrench.force.z]
                force_cog = np.dot(quaternion_matrix(rot)[0:3,0:3],np.array(force))
                torque = [msg.wrench.torque.x,msg.wrench.torque.y,msg.wrench.torque.z]
                torque_cog = np.dot(quaternion_matrix(rot)[0:3,0:3],np.array(torque))

                wrench_msg = WrenchStamped()
                wrench_msg.header = msg.header #######################
                wrench_msg.header.frame_id = self.robot_ns + '/cog'
                wrench_msg.wrench.force.x = force_cog[0]
                wrench_msg.wrench.force.y = force_cog[1]
                wrench_msg.wrench.force.z = force_cog[2]
                wrench_msg.wrench.torque.x = torque_cog[0]
                wrench_msg.wrench.torque.y = torque_cog[1]
                wrench_msg.wrench.torque.z = torque_cog[2]

                self.debug_right_ft_cog_pub.publish(wrench_msg)

                self.ft_ema_short_cog_right = self.calc_ema_ft(self.ft_ema_short_cog_right,wrench_msg,self.smooth_const_short)
                self.ft_ema_long_cog_right = self.calc_ema_ft(self.ft_ema_long_cog_right,wrench_msg,self.smooth_const_long)

                self.debug_ft_ema_short_cog_right_pub.publish(self.ft_ema_short_cog_right)
                self.debug_ft_ema_long_cog_right_pub.publish(self.ft_ema_long_cog_right)

                x_unit_vector = [1,0,0]
                x_unit_vector_cog = np.dot(quaternion_matrix(rot)[0:3,0:3],np.array(x_unit_vector))
                self.x_unit_vector_from_cog_right = np.array([x_unit_vector_cog[0],x_unit_vector_cog[1],x_unit_vector_cog[2]])

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass

    def OdometryCallback(self, msg):

        self.position = np.array([msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z])

        euler_angle = tf.transformations.euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))

        self.orientation_euler_angle = list(euler_angle)

        yaw_msg = Float64()
        if self.orientation_euler_angle[2]:
            yaw_msg.data = self.orientation_euler_angle[2]
        self.debug_yaw_pub.publish(yaw_msg)

    def FTSensorFeedbackSwitchCallback(self,msg):
        if msg.data == 0:
            self.ft_sensor_feedback_flag = False
        elif msg.data == 1:
            self.ft_sensor_feedback_flag = True

class PIDController:
    def __init__(self,K_P,K_I,K_D,maximum,minimum,i_max,i_min,topic_name):

        self.K_P = K_P
        self.K_I = K_I
        self.K_D = K_D
        self.maximum = maximum
        self.minimum = minimum
        self.i_max = i_max
        self.i_min = i_min
        self.init_update = False
        self.last = rospy.Time().now()

        self._p_err = 0
        self._i_err = 0
        self._d_err = 0
        self._p_err_last = 0

        self.pid_debug_pub = rospy.Publisher(topic_name,Pid,queue_size=10)

    def update(self,err_p):

        if not self.init_update:
            self.last = rospy.Time().now()
            self.init_update = True
            return 0

        self._p_err = err_p

        now = rospy.Time.now()
        dt = (now - self.last).secs - (now - self.last).nsecs*10**-9
        self.last = now

        self._i_err += self._p_err * dt
        self._i_err = max(min(self._i_err, self.i_max), self.i_min)

        self._d_err = (self._p_err - self._p_err_last)/dt

        self._p_err_last = self._p_err

        p_term = self.K_P * self._p_err
        i_term = self.K_I * self._i_err
        d_term = self.K_D * self._d_err

        pid_sum = p_term + i_term + d_term

        pid_sum_clamped = max(min(pid_sum, self.maximum), self.minimum)

        pid = Pid()
        pid.total = [pid_sum_clamped]
        pid.p_term = [p_term]
        pid.i_term = [i_term]
        pid.d_term = [d_term]
        pid.err_p = self._p_err
        self.pid_debug_pub.publish(pid)

        return pid_sum_clamped

