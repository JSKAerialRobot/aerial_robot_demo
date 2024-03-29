import rospy
import smach
import smach_ros
from mbzirc2020_common.hydrus_interface import *
from task2_hydrus_interface import Task2HydrusInterface
from sensor_msgs.msg import JointState
import numpy as np
from geometry_msgs.msg import Transform, Inertia, PoseArray
import tf.transformations as tft
import tf2_ros
import ros_numpy
from std_msgs.msg import UInt8, Empty
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest
import copy
import std_srvs.srv
import subprocess
from mbzirc2020_common.gps_utils import *

class Task2State(smach.State):
    def __init__(self, robot, outcomes=[], input_keys=[], output_keys=[], io_keys=[]):
        smach.State.__init__(self, outcomes, input_keys, output_keys, io_keys)
        self.task_start_sub = rospy.Subscriber('/task_start', Empty, self.taskStartCallback)
        self.robot = robot
        self.task_start = False

    def taskStartCallback(self, msg):
        self.task_start = True

    # write this at the top of execute
    def waitUntilTaskStart(self):
        while not self.task_start:
            pass


class Start(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, robot,
                            outcomes=['succeeded'])

        self.object_lookdown_height = rospy.get_param('~object_lookdown_height')
        self.skip_takeoff = rospy.get_param('~skip_takeoff', False)

    def execute(self, userdata):
        self.waitUntilTaskStart()

        self.robot.setXYPosOffset(self.robot.getBaselinkPos()[0:2])

        if self.skip_takeoff:
            return 'succeeded'

        self.robot.setJointTorque(True)
        rospy.logwarn(self.__class__.__name__ + ': Takeoff')
        rospy.sleep(0.5)
        self.robot.startAndTakeoff()
        while not (self.robot.getFlightState() == self.robot.HOVER_STATE):
            pass
        self.robot.goPosWaitConvergence('global', self.robot.getBaselinkPos()[0:2], self.object_lookdown_height, self.robot.getBaselinkRPY()[2], timeout=10, pos_conv_thresh = 0.4, yaw_conv_thresh = 0.2, vel_conv_thresh = 0.2)

        self.robot.setCameraJointAngle(np.pi / 2)
        return 'succeeded'

class ApproachPickPosition(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, robot,
                            outcomes=['succeeded'],
                            input_keys=['object_count'],
                            output_keys=['object_count'])

        self.skip_approach_pick_position = rospy.get_param('~skip_approach_pick_position', False)
        self.object_lookdown_height = rospy.get_param('~object_lookdown_height')
        self.global_lookdown_pos_gps = rospy.get_param('~global_lookdown_pos_gps')
        self.grasping_yaw = rospy.get_param('~grasping_yaw')
        self.global_object_yaw = rospy.get_param('~global_object_yaw')
        self.disable_alt_sensor = rospy.get_param('~disable_alt_sensor', True)
        self.alt_sensor_service_name = rospy.get_param('~alt_sensor_service_name')

        if self.disable_alt_sensor:
            self.alt_sensor_service_client = rospy.ServiceProxy(self.alt_sensor_service_name, std_srvs.srv.SetBool)

    def execute(self, userdata):
        self.waitUntilTaskStart()

        if self.skip_approach_pick_position:
            return 'succeeded'

        target_uav_yaw = self.global_object_yaw - self.grasping_yaw

        #enable alt sensor
        if self.disable_alt_sensor:
            try:
                req = std_srvs.srv.SetBoolRequest()
                req.data = True
                res = self.alt_sensor_service_client(req)

                if res is not None:
                    rospy.logwarn("Enable alt sensor")
                else:
                    rospy.logerr("Failed to disable alt sensor")

            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s", e)

        self.robot.goPosWaitConvergence('global', self.global_lookdown_pos_gps, self.robot.getTargetZ(), target_uav_yaw, gps_mode = True, pos_conv_thresh = 0.4, yaw_conv_thresh = 0.2, vel_conv_thresh = 0.2)
        self.robot.goPosWaitConvergence('global', self.global_lookdown_pos_gps, self.object_lookdown_height, target_uav_yaw, gps_mode = True, pos_conv_thresh = 0.4, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.2)

        return 'succeeded'

class LookDown(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, robot,
                            outcomes=['succeeded', 'failed'],
                            input_keys=['orig_global_trans', 'search_count', 'search_failed'],
                            output_keys=['orig_global_trans', 'search_count', 'search_failed'])

        self.skip_look_down = rospy.get_param('~skip_look_down', False)
        self.no_lookdown_mode = rospy.get_param('~no_lookdown_mode')
        self.do_object_recognition = rospy.get_param('~do_object_recognition')
        self.object_approach_height = rospy.get_param('~object_approach_height')
        self.object_yaw_thresh = rospy.get_param('~object_yaw_thresh')
        self.grasping_yaw = rospy.get_param('~grasping_yaw')
        self.recognition_wait = rospy.get_param('~recognition_wait')
        self.global_lookdown_pos_gps = rospy.get_param('~global_lookdown_pos_gps')

        self.object_pose_sub = rospy.Subscriber('rectangle_detection_color/target_object_color', PoseArray, self.objectPoseCallback)
        self.object_pose = PoseArray()

    def objectPoseCallback(self, msg):
        if len(msg.poses) != 0:
            self.object_pose = msg

    def execute(self, userdata):
        self.waitUntilTaskStart()

        if self.skip_look_down:
            return 'succeeded'

        if not self.do_object_recognition:
            rospy.logwarn(self.__class__.__name__ + ': no recognition, skip')
            return 'succeeded'

        if self.no_lookdown_mode:
            rospy.logwarn(self.__class__.__name__ + ': no lookdown mode')
            self.robot.goPosWaitConvergence('global', self.global_lookdown_pos_gps, self.object_approach_height, self.robot.getTargetYaw(), gps_mode=True, pos_conv_thresh = 0.2, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1, timeout = 10)
            return 'succeeded'

        object_found = True

        self.object_pose = PoseArray() #reset
        rospy.sleep(self.recognition_wait)

        if len(self.object_pose.poses) != 0:
            try:
                cam_trans = self.robot.getTF(self.object_pose.header.frame_id)
                cam_trans = ros_numpy.numpify(cam_trans.transform)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr(self.__class__.__name__ + ": cannot find camera tf")
                return 'failed'
        else:
            object_found = False

        if object_found:
            #determine valid target object
            #detect most foreground object
            object_poses = self.object_pose.poses
            object_poses = sorted(object_poses, key=lambda x: x.position.y)
            target_object_pose = object_poses[-1]

            #check the validity in terms of yaw angle (yaw angle should be around -pi/2 in camera frame)
            object_yaw = tft.euler_from_quaternion(ros_numpy.numpify(target_object_pose.orientation))[2]
            if abs(object_yaw - (-np.pi / 2)) > self.object_yaw_thresh:
                rospy.logerr(self.__class__.__name__ + ": object invalide yaw")
                object_found = False

        if object_found:
            #succeeded to find object, then move

            object_global_coords = tft.concatenate_matrices(cam_trans, ros_numpy.numpify(target_object_pose))
            object_global_x_axis = object_global_coords[0:3, 0]
            object_global_yaw = np.arctan2(object_global_x_axis[1], object_global_x_axis[0])
            object_global_pos = tft.translation_from_matrix(object_global_coords)
            uav_target_yaw = object_global_yaw - self.grasping_yaw

            rospy.logwarn("%s: succeed to find valid object x: %f, y: %f, yaw: %f", self.__class__.__name__, object_global_pos[0], object_global_pos[1], object_global_yaw)
            self.robot.goPosWaitConvergence('global', [object_global_pos[0], object_global_pos[1]], self.robot.getTargetZ(), uav_target_yaw, pos_conv_thresh = 0.2, yaw_conv_thresh = 0.2, vel_conv_thresh = 0.1)
            self.robot.goPosWaitConvergence('global', [object_global_pos[0], object_global_pos[1]], self.object_approach_height, uav_target_yaw, pos_conv_thresh = 0.15, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1, timeout = 10)

            userdata.search_count = 0
            userdata.search_failed = False
            return 'succeeded'

        else:
            rospy.logerr(self.__class__.__name__ + ": no object found")
            if userdata.search_count == 0:
                userdata.orig_global_trans = ros_numpy.numpify(self.robot.getBaselinkOdom().pose.pose)

            if userdata.search_failed:
                #init search state
                userdata.search_count = 0
                userdata.search_failed = False

                #go to next state
                self.robot.goPosWaitConvergence('global', self.robot.getTargetXY(), self.object_approach_height, self.robot.getTargetYaw(), pos_conv_thresh = 0.2, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1)

                return 'succeeded' 
            else:
                return 'failed'

class AdjustGraspPosition(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, robot,
                            outcomes=['succeeded', 'failed', 'adjust_again'],
                            input_keys=['orig_global_trans', 'search_count', 'search_failed'],
                            output_keys=['orig_global_trans', 'search_count', 'search_failed'])

        self.skip_adjust_grasp_position = rospy.get_param('~skip_adjust_grasp_position', False)
        self.do_object_recognition = rospy.get_param('~do_object_recognition')
        self.object_yaw_thresh = rospy.get_param('~object_yaw_thresh')
        self.global_object_yaw = rospy.get_param('~global_object_yaw')
        self.grasping_yaw = rospy.get_param('~grasping_yaw')
        self.recognition_wait = rospy.get_param('~recognition_wait')
        self.adjust_grasp_image_type = rospy.get_param('~adjust_grasp_image_type', 'depth')
        self.object_pos_thresh = rospy.get_param('~object_pos_thresh')

        grasping_point = rospy.get_param('~grasping_point')
        grasping_yaw = rospy.get_param('~grasping_yaw')
        self.grasping_point_coords = tft.compose_matrix(translate=[grasping_point[0], grasping_point[1], grasping_point[2]], angles=[0, 0, grasping_yaw])

        self.object_pose_sub = rospy.Subscriber('rectangle_detection_' + self.adjust_grasp_image_type + '/target_object_' + self.adjust_grasp_image_type, PoseArray, self.objectPoseCallback)
        self.object_pose = PoseArray()

    def objectPoseCallback(self, msg):
        if len(msg.poses) != 0:
            self.object_pose = msg

    def execute(self, userdata):
        self.waitUntilTaskStart()

        if self.skip_adjust_grasp_position:
            return 'succeeded'

        if not self.do_object_recognition:
            rospy.logwarn(self.__class__.__name__ + ": no recognition, skip")
            return 'succeeded'

        object_found = True

        self.object_pose = PoseArray() #reset
        rospy.sleep(self.recognition_wait)

        if len(self.object_pose.poses) != 0:
            try:
                cam_trans = self.robot.getTF(self.object_pose.header.frame_id)
                cam_trans = ros_numpy.numpify(cam_trans.transform)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr(self.__class__.__name__ + ": cannot find camera tf")
                return 'failed'
        else:
            object_found = False


        if object_found:
            #determine valid target object
            #detect most center object
            object_poses = self.object_pose.poses
            object_poses = sorted(object_poses, key=lambda x: x.position.x ** 2 + x.position.y ** 2)
            target_object_pose = object_poses[0]

            #check the validity in terms of yaw angle (yaw angle should be around -pi/2 in camera frame)
            object_yaw = tft.euler_from_quaternion(ros_numpy.numpify(target_object_pose.orientation))[2]
            if abs(object_yaw - (-np.pi / 2)) > self.object_yaw_thresh:
                rospy.logerr(self.__class__.__name__ + ": object invalide yaw")
                object_found = False

        if object_found:
            object_global_coords = tft.concatenate_matrices(cam_trans, ros_numpy.numpify(target_object_pose))
            object_global_pos = tft.translation_from_matrix(object_global_coords)
            object_pos_local = object_global_pos - self.robot.getBaselinkPos()
            object_global_x_axis = object_global_coords[0:3, 0]
            object_global_yaw = np.arctan2(object_global_x_axis[1], object_global_x_axis[0])
            rospy.logwarn("%s: succeed to find valid object x: %f, y: %f, yaw: %f", self.__class__.__name__, object_global_pos[0], object_global_pos[1], object_global_yaw)

            object_global_coords_modified = tft.compose_matrix(translate=tft.translation_from_matrix(object_global_coords), angles=[0, 0, object_global_yaw])

            uav_target_coords = tft.concatenate_matrices(object_global_coords_modified, tft.inverse_matrix(self.grasping_point_coords))
            uav_target_pos = tft.translation_from_matrix(uav_target_coords)
            uav_target_yaw = tft.euler_from_matrix(uav_target_coords)[2]

            self.robot.goPosWaitConvergence('global', [object_global_pos[0], object_global_pos[1]], self.robot.getTargetZ(), uav_target_yaw, pos_conv_thresh = 0.15, yaw_conv_thresh = 0.05, vel_conv_thresh = 0.1)
            #reset search state
            userdata.search_count = 0
            userdata.search_failed = False

            # check object is directly below
            if np.linalg.norm(object_pos_local[0:2]) > self.object_pos_thresh:
                rospy.logwarn("%s: succeed to find object, but not directly below. diff: %f", self.__class__.__name__, np.linalg.norm(object_pos_local[0:2]))
                return 'adjust_again'

            self.robot.preshape()
            self.robot.goPosWaitConvergence('global', [uav_target_pos[0], uav_target_pos[1]], self.robot.getTargetZ(), uav_target_yaw, pos_conv_thresh = 0.15, yaw_conv_thresh = 0.05, vel_conv_thresh = 0.1)
            return 'succeeded'

        else:
            rospy.logerr(self.__class__.__name__ + ": no object found")
            if userdata.search_count == 0:
                userdata.orig_global_trans = ros_numpy.numpify(self.robot.getBaselinkOdom().pose.pose)

            if userdata.search_failed:
                #reset search state
                userdata.search_count = 0
                userdata.search_failed = False
                self.robot.preshape()

                return 'succeeded' #go to next state
            else:
                return 'failed'

class Grasp(Task2State):
    def __init__(self, robot, add_object_model_func):
        Task2State.__init__(self, robot,
                            outcomes=['succeeded', 'failed'],
                            input_keys=['object_count'],
                            output_keys=['object_count'])

        self.add_object_model_func = add_object_model_func
        self.do_object_recognition = rospy.get_param('~do_object_recognition')
        self.joint_torque_thresh = rospy.get_param('~joint_torque_thresh')
        self.skip_grasp = rospy.get_param('~skip_grasp', False)
        self.object_grasping_height = rospy.get_param('~object_grasping_height')
        self.object_interval = rospy.get_param('~object_interval')
        self.lane_interval = rospy.get_param('~lane_interval')
        self.global_first_object_pos = rospy.get_param('~global_first_object_pos')
        self.global_object_yaw = rospy.get_param('~global_object_yaw')
        self.grasp_land_mode = rospy.get_param('~grasp_land_mode')
        self.reset_realsense_odom = rospy.get_param('~reset_realsense_odom')
        self.stop_if_grasp_failed = rospy.get_param('~stop_if_grasp_failed')
        self.reset_realsense_client = rospy.ServiceProxy('/realsense1/odom/reset', std_srvs.srv.Empty)
        grasping_point = rospy.get_param('~grasping_point')
        grasping_yaw = rospy.get_param('~grasping_yaw')
        self.grasping_point_coords = tft.compose_matrix(translate=[grasping_point[0], grasping_point[1], grasping_point[2]], angles=[0, 0, grasping_yaw])
        self.grasp_force_landing_mode = rospy.get_param('~grasp_force_landing_mode')
        self.force_landing_height = rospy.get_param('~force_landing_height')

    def execute(self, userdata):
        self.waitUntilTaskStart()

        if self.skip_grasp:
            return 'succeeded'

        if not self.do_object_recognition:

            rospy.logwarn(self.__class__.__name__ + ": no recognition, skip")

            vec_from_first_object = [0.0, 0.0, 0.0]
            if (userdata.object_count == 2) or (userdata.object_count == 3):
                vec_from_first_object[0] = self.object_interval
            if (userdata.object_count == 1) or (userdata.object_count == 3):
                vec_from_first_object[1] = self.lane_interval

            trans_from_first_object = tft.translation_matrix(vec_from_first_object)
            global_first_object_coords = tft.compose_matrix(translate=[self.global_first_object_pos[0], self.global_first_object_pos[1], 0.0], angles=[0, 0, self.global_object_yaw])
            global_target_object_coords = tft.concatenate_matrices(global_first_object_coords, trans_from_first_object)
            uav_target_coords = tft.concatenate_matrices(global_target_object_coords, tft.inverse_matrix(self.grasping_point_coords))
            uav_target_pos = tft.translation_from_matrix(uav_target_coords)
            uav_target_yaw = tft.euler_from_matrix(uav_target_coords)[2]

            rospy.logwarn(self.__class__.__name__ + ": Directly go to x: %f, y: %f, yaw: %f", uav_target_pos[0], uav_target_pos[1], uav_target_yaw)

            self.robot.preshape()
            self.robot.goPosWaitConvergence('global', [uav_target_pos[0], uav_target_pos[1]], self.robot.getTargetZ(), uav_target_yaw, pos_conv_thresh = 0.1, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1)

        if self.grasp_land_mode and (not self.grasp_force_landing_mode):
            rospy.logwarn(self.__class__.__name__ + ": land mode, landing")
            self.robot.land()
            start_time = rospy.get_time()
            while not (self.robot.getFlightState() == self.robot.ARM_OFF_STATE):
                elapsed_time = rospy.get_time() - start_time
                if elapsed_time > 10.0:
                    self.robot.halt()
                    rospy.logwarn(self.__class__.__name__ + ": force halt")
                    break
        else:
            if self.grasp_force_landing_mode:
                self.robot.goPosWaitConvergence('global', self.robot.getTargetXY(), self.force_landing_height, self.robot.getTargetYaw(), pos_conv_thresh = 0.15, yaw_conv_thresh = 0.05, vel_conv_thresh = 0.1, timeout=15)
                rospy.logwarn(self.__class__.__name__ + ": force land mode, force landing")
                self.robot.forceLanding()
                start_time = rospy.get_time()
                while not (self.robot.getFlightState() == self.robot.ARM_OFF_STATE):
                    elapsed_time = rospy.get_time() - start_time
                    if elapsed_time > 5.0:
                        self.robot.halt()
                        rospy.logwarn(self.__class__.__name__ + ": force halt")
                        break

            else:
                #descend
                self.robot.goPosWaitConvergence('global', self.robot.getTargetXY(), self.object_grasping_height, self.robot.getTargetYaw(), pos_conv_thresh = 0.2, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1)
        self.robot.openJoint()
        self.robot.grasp()
        rospy.sleep(1);
        joint_state = self.robot.getJointState()
        joint_torque = []
        joint_torque.append(joint_state.effort[joint_state.name.index('joint1')])
        joint_torque.append(joint_state.effort[joint_state.name.index('joint3')])

        result = 'succeeded'

        if all(np.array(joint_torque) > self.joint_torque_thresh):
            self.add_object_model_func(self.robot)
            result = 'succeeded'
        else:
            rospy.logerr("%s: grasp failed! joint1: %f, joint3: %f, thresh: %f", self.__class__.__name__, joint_torque[0], joint_torque[1], self.joint_torque_thresh)
            result = 'failed'
            if self.stop_if_grasp_failed:
                rospy.signal_shutdown("finish state machine")
                rospy.sleep(100)

        if self.reset_realsense_odom:
            rospy.logwarn(self.__class__.__name__ + ": reset realsense odom")
            cmd = "rosrun mbzirc2020_task2_common reset_vo.sh"
            subprocess.Popen(cmd.split())

            rospy.sleep(10)

        if self.grasp_land_mode:
            rospy.logwarn(self.__class__.__name__ + ": land mode, takeoff")
            self.robot.startAndTakeoff()
            while not (self.robot.getFlightState() == self.robot.HOVER_STATE):
                pass #wait until hover

        return result

class ApproachPlacePosition(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, robot, outcomes=['succeeded'],
                            input_keys=['object_count', 'orig_channel_xy_yaw'],
                            output_keys=['object_count', 'orig_channel_xy_yaw'])

        self.simulation = rospy.get_param('/simulation')
        self.skip_approach_place_position = rospy.get_param('~skip_approach_place_position', False)
        self.disable_alt_sensor = rospy.get_param('~disable_alt_sensor', True)
        self.alt_sensor_service_name = rospy.get_param('~alt_sensor_service_name')
        self.vo_service_name = rospy.get_param('~vo_service_name', '/estimator/sensor_plugin/vo1/estimate_flag')
        self.global_place_channel_z = rospy.get_param('~global_place_channel_z')
        self.grasping_yaw = rospy.get_param('~grasping_yaw')
        self.place_z_offset = rospy.get_param('~place_z_offset')
        self.global_place_channel_center_pos_gps = rospy.get_param('~global_place_channel_center_pos_gps')
        self.place_channel_length = rospy.get_param('~place_channel_length')
        self.global_place_channel_yaw = rospy.get_param('~global_place_channel_yaw')

        self.waypoint_list = []
        self.waypoint_list.append(self.global_place_channel_center_pos_gps)
        self.waypoint_list.append(getGpsLocation(self.global_place_channel_center_pos_gps, np.cos(self.global_place_channel_yaw), np.sin(self.global_place_channel_yaw)))
        self.waypoint_list.append(getGpsLocation(self.global_place_channel_center_pos_gps, -np.cos(self.global_place_channel_yaw), -np.sin(self.global_place_channel_yaw)))

        if self.disable_alt_sensor:
            self.alt_sensor_service_client = rospy.ServiceProxy(self.alt_sensor_service_name, std_srvs.srv.SetBool)
            self.vo_service_client = rospy.ServiceProxy(self.vo_service_name, std_srvs.srv.SetBool)

    def execute(self, userdata):
        self.waitUntilTaskStart()

        if self.skip_approach_place_position:
            return 'succeeded'

        #calc place coords in gps
        place_pos_gps = self.waypoint_list[userdata.object_count]
        uav_target_yaw = self.global_place_channel_yaw - self.grasping_yaw

        if self.simulation:
            client = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
            req = ApplyBodyWrenchRequest()
            req.body_name = 'hydrus::root'
            req.wrench.force.z = 10
            req.duration.nsecs = 300000000
            try:
                res = client(req)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

        self.robot.goPosWaitConvergence('global', self.robot.getTargetXY(), self.global_place_channel_z + self.place_z_offset, self.robot.getBaselinkRPY()[2], pos_conv_thresh = 0.4, yaw_conv_thresh = 0.2, vel_conv_thresh = 0.2)

        #disable alt sensor
        if self.disable_alt_sensor:
            try:
                req = std_srvs.srv.SetBoolRequest()
                req.data = False
                res = self.alt_sensor_service_client(req)

                if res is not None:
                    rospy.logwarn("Disable alt sensor")
                else:
                    rospy.logerr("Failed to disable alt sensor")

            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s", e)

            try:
                rospy.logwarn("Enable VO pos")
                req = std_srvs.srv.SetBoolRequest()
                req.data = True
                res = self.vo_service_client(req)

            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s", e)

        self.robot.goPosWaitConvergence('global', place_pos_gps, self.global_place_channel_z + self.place_z_offset, uav_target_yaw, gps_mode = True, timeout=60, pos_conv_thresh = 0.3, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1)
        userdata.orig_channel_xy_yaw = (self.robot.getBaselinkPos()[0:2], self.global_place_channel_yaw)

        return 'succeeded'

class AdjustPlacePosition(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, robot,
                            outcomes=['succeeded', 'failed', 'adjust_again'],
                            input_keys=['orig_global_trans', 'search_count', 'search_failed'],
                            output_keys=['orig_global_trans', 'search_count', 'search_failed'])

        self.skip_adjust_place_position = rospy.get_param('~skip_adjust_place_position', False)
        self.do_channel_recognition = rospy.get_param('~do_channel_recognition')
        self.channel_tf_frame_id = rospy.get_param('~channel_tf_frame_id')
        self.global_place_channel_z = rospy.get_param('~global_place_channel_z')
        self.recognition_wait = rospy.get_param('~recognition_wait')
        self.channel_pos_thresh = rospy.get_param('~channel_pos_thresh')

    def execute(self, userdata):
        self.waitUntilTaskStart()

        if self.skip_adjust_place_position:
            return 'succeeded'

        if not self.do_channel_recognition:
            rospy.logwarn(self.__class__.__name__ + ': no recognition, skip')
            return 'succeeded'

        try:
            channel_trans = self.robot.getTF(self.channel_tf_frame_id, wait=self.recognition_wait)
            channel_trans = ros_numpy.numpify(channel_trans.transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr(self.__class__.__name__ + ": channel position detect failed")

            if userdata.search_count == 0:
                userdata.orig_global_trans = ros_numpy.numpify(self.robot.getBaselinkOdom().pose.pose)

            if userdata.search_failed:
                #init search state
                userdata.search_count = 0
                userdata.search_failed = False

                return 'succeeded' #go to next state
            else:
                return 'failed'

        channel_pos = tft.translation_from_matrix(channel_trans)
        rospy.logwarn("%s: succeed to find channel x: %f, y: %f", self.__class__.__name__, channel_pos[0], channel_pos[1])
        channel_pos_local = channel_pos - self.robot.getBaselinkPos()

        self.robot.goPosWaitConvergence('global', channel_pos[0:2], self.robot.getTargetZ(), self.robot.getTargetYaw(), pos_conv_thresh = 0.25, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1, timeout = 15)

        #reset search state
        userdata.search_count = 0
        userdata.search_failed = False

        # check channel is directly below
        if np.linalg.norm(channel_pos_local[0:2]) > self.channel_pos_thresh:
            rospy.logwarn("%s: succeed to find channel, but not directly below. diff: %f", self.__class__.__name__, np.linalg.norm(channel_pos_local[0:2]))
            return 'adjust_again'

        return 'succeeded'

class Ungrasp(Task2State):
    def __init__(self, robot, remove_object_model_func):
        Task2State.__init__(self, robot, outcomes=['finish', 'continue'],
                             input_keys=['object_count', 'orig_channel_xy_yaw'],
                             output_keys=['object_count', 'orig_channel_xy_yaw'])

        self.remove_object_model_func = remove_object_model_func
        self.global_place_channel_z = rospy.get_param('~global_place_channel_z')
        self.global_place_channel_yaw = rospy.get_param('~global_place_channel_yaw')
        self.place_z_margin = rospy.get_param('~place_z_margin')
        self.place_z_offset = rospy.get_param('~place_z_offset')
        self.object_num = rospy.get_param('~object_num')
        self.skip_ungrasp = rospy.get_param('~skip_ungrasp', False)
        self.do_channel_recognition = rospy.get_param('~do_channel_recognition')
        self.single_object_mode = rospy.get_param('~single_object_mode')

        grasping_point = rospy.get_param('~grasping_point')
        self.grasping_yaw = rospy.get_param('~grasping_yaw')
        self.grasping_point_coords = tft.compose_matrix(translate=[grasping_point[0], grasping_point[1], grasping_point[2]], angles=[0, 0, self.grasping_yaw])

    def execute(self, userdata):
        self.waitUntilTaskStart()

        if self.skip_ungrasp:
            return 'succeeded'

        if self.do_channel_recognition:
            channel_center_coords = tft.compose_matrix(translate=[self.robot.getTargetXY()[0], self.robot.getTargetXY()[1], 0], angles=[0, 0, self.global_place_channel_yaw])
        else:
            channel_center_coords = tft.compose_matrix(translate=[userdata.orig_channel_xy_yaw[0][0], userdata.orig_channel_xy_yaw[0][1], 0], angles=[0, 0, userdata.orig_channel_xy_yaw[1]])

        uav_target_coords = tft.concatenate_matrices(channel_center_coords, tft.inverse_matrix(self.grasping_point_coords))
        uav_target_pos = tft.translation_from_matrix(uav_target_coords)

        rospy.logwarn(self.__class__.__name__ + ": Go to x: %f, y: %f, yaw: %f", uav_target_pos[0], uav_target_pos[1], self.robot.getTargetYaw())
        self.robot.goPosWaitConvergence('global', uav_target_pos[0:2], self.robot.getTargetZ(), self.robot.getTargetYaw(), pos_conv_thresh = 0.25, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1)

        #descend
        self.robot.goPosWaitConvergence('global', self.robot.getTargetXY(), self.global_place_channel_z + self.place_z_margin, self.robot.getTargetYaw(), pos_conv_thresh = 0.15, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1, timeout = 10)

        self.robot.ungrasp()
        self.remove_object_model_func(self.robot)
        userdata.object_count += 1
        self.robot.goPosWaitConvergence('global', self.robot.getTargetXY(), self.global_place_channel_z + self.place_z_offset, self.robot.getTargetYaw(), pos_conv_thresh = 0.3, yaw_conv_thresh = 0.2, vel_conv_thresh = 0.3)
        self.robot.resetPose()

        if (userdata.object_count == self.object_num) or self.single_object_mode:
            rospy.logwarn(self.__class__.__name__ + ": finish task")
            return 'finish'
        else:
            return 'continue'

class SearchMotion(Task2State):
    def __init__(self, robot, search_state):
        Task2State.__init__(self, robot, outcomes=['succeeded'],
                            input_keys=['orig_global_trans', 'search_count', 'search_failed'],
                            output_keys=['orig_global_trans', 'search_count', 'search_failed'])

        self.grid = rospy.get_param('~' + search_state + '/grid')
        self.grid_scale_x = rospy.get_param('~' + search_state + '/grid_scale_x')
        self.grid_scale_y = rospy.get_param('~' + search_state + '/grid_scale_y')
        self.grid_search_timeout = rospy.get_param('~' + search_state + '/grid_search_timeout')
        self.grasping_yaw = rospy.get_param('~grasping_yaw')

    def execute(self, userdata):
        target_pos_from_orig_pos_local_frame = tft.translation_matrix((self.grid[userdata.search_count][0] * self.grid_scale_x, self.grid[userdata.search_count][1] * self.grid_scale_y, 0.0))
        grasp_yaw_rotation_mat = tft.euler_matrix(0, 0, self.grasping_yaw)
        uav_target_coords = tft.concatenate_matrices(userdata.orig_global_trans, grasp_yaw_rotation_mat, target_pos_from_orig_pos_local_frame)
        uav_target_pos = tft.translation_from_matrix(uav_target_coords)

        self.robot.goPosWaitConvergence('global', [uav_target_pos[0], uav_target_pos[1]], self.robot.getTargetZ(), self.robot.getTargetYaw(), pos_conv_thresh = 0.2, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1, timeout=self.grid_search_timeout)

        userdata.search_count += 1
        if userdata.search_count == len(self.grid):
            userdata.search_count = 0
            userdata.search_failed = True

            orig_pos = tft.translation_from_matrix(userdata.orig_global_trans)
            rospy.logwarn(self.__class__.__name__ + ": return to original pos")
            self.robot.goPosWaitConvergence('global', [orig_pos[0], orig_pos[1]], self.robot.getTargetZ(), self.robot.getTargetYaw(), pos_conv_thresh = 0.1, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1, timeout=7)

        return 'succeeded'

class Finish(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, robot, outcomes=['succeeded'])

    def execute(self, userdata):
        self.robot.goPosWaitConvergence('global', [0, 0], self.robot.getBaselinkPos()[2], self.robot.getBaselinkRPY()[2])
        rospy.loginfo('Landing')
        self.robot.land()
        return 'succeeded'
