import rospy
import smach
import smach_ros
from mbzirc2020_common.hydrus_interface import *
from task2_hydrus_interface import Task2HydrusInterface
from sensor_msgs.msg import JointState
import numpy as np
from geometry_msgs.msg import Transform, Inertia, PoseArray, Quaternion, PoseStamped, Pose
import tf.transformations as tft
import tf2_ros
import ros_numpy
from std_msgs.msg import UInt8, Empty
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest
import copy
import std_srvs.srv
import subprocess
from mbzirc2020_common.gps_utils import *
from jsk_recognition_msgs.msg import BoundingBoxArray

class Task2State(smach.State):
    def __init__(self, robot, outcomes=[], input_keys=[], output_keys=[], io_keys=[]):
        smach.State.__init__(self, outcomes, input_keys, output_keys, io_keys)
        self.task_start_sub = rospy.Subscriber('/task_start', Empty, self.taskStartCallback)

        self.robot = robot
        self.task_start = False

        #ros params
        self.simulation = rospy.get_param('/simulation')
        self.skip_takeoff = rospy.get_param('~skip_takeoff', False)
        self.skip_approach_pick_area = rospy.get_param('~skip_approach_pick_area', False)
        self.skip_scan_pick_area = rospy.get_param('~skip_scan_pick_area', False)
        self.skip_pick_visual_servoing = rospy.get_param('~skip_pick_visual_servoing', False)
        self.skip_grasp = rospy.get_param('~skip_grasp', False)
        self.skip_approach_place_area = rospy.get_param('~skip_approach_place_area', False)
        self.skip_place_visual_servoing = rospy.get_param('~skip_place_visual_servoing', False)
        self.skip_ungrasp = rospy.get_param('~skip_ungrasp', False)

        self.object_lookdown_height = rospy.get_param('~object_lookdown_height')
        self.global_lookdown_pos_gps = rospy.get_param('~global_lookdown_pos_gps')
        self.grasping_yaw_in_fc = rospy.get_param('~grasping_yaw_in_fc')
        self.global_object_yaw = rospy.get_param('~global_object_yaw')
        self.alt_sensor_service_name = rospy.get_param('~alt_sensor_service_name')
        self.plane_detection_service_name = rospy.get_param('~plane_detection_service_name')
        self.realsense_reset_service_name = rospy.get_param('~realsense_reset_service_name')
        self.scan_area_velocity = rospy.get_param('~scan_area_velocity')
        self.scan_area_length = rospy.get_param('~scan_area_length')
        self.scan_area_pos_thre = rospy.get_param('~scan_area_pos_thre')
        self.scan_rp_thre = rospy.get_param('~scan_rp_thre')
        self.pick_visual_servoing_z_diff = rospy.get_param('~pick_visual_servoing_z_diff')
        self.object_approach_height = rospy.get_param('~object_approach_height')
        self.grasp_vs_end_height = rospy.get_param('~grasp_vs_end_height')
        self.joint_torque_thresh = rospy.get_param('~joint_torque_thresh')
        self.stop_if_grasp_failed = rospy.get_param('~stop_if_grasp_failed')
        self.object_length = rospy.get_param('~object_length')
        self.grasp_horizontal_interpolation_time = rospy.get_param('~grasp_horizontal_interpolation_time')
        self.grasp_vertical_interpolation_time = rospy.get_param('~grasp_vertical_interpolation_time')
        self.global_place_channel_center_pos_gps = rospy.get_param('~global_place_channel_center_pos_gps')
        self.global_place_channel_yaw = rospy.get_param('~global_place_channel_yaw')
        self.place_lookdown_height = rospy.get_param('~place_lookdown_height')
        self.place_height = rospy.get_param('~place_height')
        self.channel_tf_frame_id = rospy.get_param('~channel_tf_frame_id')
        self.recognition_wait = rospy.get_param('~recognition_wait')
        self.channel_pos_thresh = rospy.get_param('~channel_pos_thresh')


        self.alt_sensor_service_client = rospy.ServiceProxy(self.alt_sensor_service_name, std_srvs.srv.SetBool)
        self.plane_detection_service_client = rospy.ServiceProxy(self.plane_detection_service_name, std_srvs.srv.SetBool)
        self.realsense_reset_service_client = rospy.ServiceProxy(self.realsense_reset_service_name, std_srvs.srv.Empty)

        grasping_point = rospy.get_param('~grasping_point')
        grasping_yaw = rospy.get_param('~grasping_yaw')
        self.grasping_point_coords = tft.compose_matrix(translate=[grasping_point[0], grasping_point[1], grasping_point[2]], angles=[0, 0, grasping_yaw])

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

    def execute(self, userdata):
        self.waitUntilTaskStart()

        if self.skip_takeoff:
            return 'succeeded'

        self.robot.setJointTorque(True)
        rospy.logwarn(self.__class__.__name__ + ': Takeoff')
        rospy.sleep(0.5)
        self.robot.startAndTakeoff()
        while not (self.robot.getFlightState() == self.robot.HOVER_STATE):
            pass
        self.robot.goPosWaitConvergence('global', self.robot.getBaselinkPos()[0:2], self.object_lookdown_height, None, timeout=10, pos_conv_thresh = 0.5, yaw_conv_thresh = 0.2, vel_conv_thresh = 0.3)

        self.robot.setCameraJointAngle(np.pi / 2)
        return 'succeeded'

class ApproachPickArea(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, robot,
                            outcomes=['succeeded'])

    def execute(self, userdata):
        self.waitUntilTaskStart()

        if self.skip_approach_pick_area:
            return 'succeeded'

        #enable alt sensor, disable plane detection
        if not self.simulation:
            try:
                req = std_srvs.srv.SetBoolRequest()
                req.data = True
                res = self.alt_sensor_service_client(req)

                if res is not None:
                    rospy.logwarn("Enable alt sensor")
                else:
                    rospy.logerr("Failed to enable alt sensor")

            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s", e)

            try:
                req = std_srvs.srv.SetBoolRequest()
                req.data = False
                res = self.plane_detection_service_client(req)

                if res is not None:
                    rospy.logwarn("Disable plane detection")
                else:
                    rospy.logerr("Failed to disable place detection")

            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s", e)

        target_uav_yaw = self.global_object_yaw - self.grasping_yaw_in_fc
        self.robot.goPosWaitConvergence('global', self.global_lookdown_pos_gps, None, target_uav_yaw, gps_mode = True, pos_conv_thresh = 0.4, yaw_conv_thresh = 0.2, vel_conv_thresh = 0.2)

        return 'succeeded'

class ScanPickArea(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, robot,
                            outcomes=['succeeded'],
                            input_keys=['target_object_pos'],
                            output_keys=['target_object_pos'])


        self.object_pose_sub = rospy.Subscriber('rectangle_detection_color/target_object_color', PoseArray, self.objectPoseCallback)
        self.object_pose = PoseArray()

    def objectPoseCallback(self, msg):
        if len(msg.poses) != 0:
            self.object_pose = msg

    def execute(self, userdata):
        self.waitUntilTaskStart()

        if self.skip_scan_pick_area:
            return 'succeeded'

        target_x_pos = self.robot.getBaselinkPos()[0] + self.scan_area_length * np.cos(self.global_object_yaw)
        target_y_pos = self.robot.getBaselinkPos()[1] + self.scan_area_length * np.sin(self.global_object_yaw)
        self.object_pose = PoseArray() #reset

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.robot.getBaselinkRPY()[0] > self.scan_rp_thre or self.robot.getBaselinkRPY()[1] > self.scan_rp_thre:
                continue
            #set target velocity or position
            diff_pos = np.array([target_x_pos - self.robot.getBaselinkPos()[0], target_y_pos - self.robot.getBaselinkPos()[1]])
            if np.linalg.norm(diff_pos) > self.scan_area_pos_thre:
                target_vel = (diff_pos / np.linalg.norm(diff_pos)) * self.scan_area_velocity
                self.robot.goVel('global', target_vel, None, None)
            else:
                self.robot.goPos('global', [target_x_pos, target_y_pos], None, None)

            if len(self.object_pose.poses) != 0:
                #detect most foreground object
                object_poses = self.object_pose.poses
                object_poses = sorted(object_poses, key=lambda x: x.position.y)
                target_object_pose = object_poses[-1]

                if target_object_pose.position.y < 0:
                    #if valid object found, do visual servoing
                    try:
                        cam_trans = self.robot.getTF(self.object_pose.header.frame_id)
                        cam_trans = ros_numpy.numpify(cam_trans.transform)
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        rospy.logerr(self.__class__.__name__ + ": cannot find camera tf")
                        return 'failed'

                    object_global_coords = tft.concatenate_matrices(cam_trans, ros_numpy.numpify(target_object_pose))
                    object_global_x_axis = object_global_coords[0:3, 0]
                    object_global_yaw = np.arctan2(object_global_x_axis[1], object_global_x_axis[0])
                    object_global_pos = tft.translation_from_matrix(object_global_coords)
                    #uav_target_yaw = object_global_yaw - self.grasping_yaw

                    userdata.target_object_pos = [0, 0, 0]
                    userdata.target_object_pos[0] = object_global_pos[0]
                    userdata.target_object_pos[1] = object_global_pos[1]
                    userdata.target_object_pos[2] = tft.translation_from_matrix(cam_trans)[2] - object_global_pos[2] # distance from object to camera in world frame

                    rospy.logwarn("%s: succeed to find valid object x: %f, y: %f, yaw: %f", self.__class__.__name__, object_global_pos[0], object_global_pos[1], object_global_yaw)
                    self.robot.goVel('global', [0, 0], None, None)
                    rospy.sleep(0.1)
                    self.robot.goPosWaitConvergence('global', object_global_pos[0:2], None, None, timeout=10, pos_conv_thresh = 0.3, yaw_conv_thresh = 0.2, vel_conv_thresh = 0.2)
                    return 'succeeded'

                self.object_pose = PoseArray() #reset
            r.sleep()

class PickVisualServoing(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, robot,
                            outcomes=['succeeded'],
                            input_keys=['target_object_pos'],
                            output_keys=['target_object_pos'])

        self.object_pose_sub = rospy.Subscriber('rectangle_detection_color/target_object_color', PoseArray, self.objectPoseCallback)
        self.object_pose = PoseArray()

    def objectPoseCallback(self, msg):
        if len(msg.poses) != 0:
            self.object_pose = msg

    def execute(self, userdata):
        self.waitUntilTaskStart()

        if self.skip_pick_visual_servoing:
            return 'succeeded'

        self.object_pose = PoseArray() #reset
        prev_object_x = userdata.target_object_pos[0]
        prev_object_y = userdata.target_object_pos[1]
        object2cam_distance = userdata.target_object_pos[2]
        #rospy.logwarn("%f", object2cam_distance)

        #visual servoing
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()
            if len(self.object_pose.poses) != 0:

                #find most nearest object
                min_distance = 1e8
                target_object_coords = None

                try:
                    cam_trans = self.robot.getTF(self.object_pose.header.frame_id)
                    cam_trans = ros_numpy.numpify(cam_trans.transform)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr(self.__class__.__name__ + ": cannot find camera tf")
                    return 'failed'

                for obj in self.object_pose.poses:
                    object_global_coords = tft.concatenate_matrices(cam_trans, ros_numpy.numpify(obj))
                    object_global_pos = tft.translation_from_matrix(object_global_coords)
                    distance = (prev_object_x - object_global_pos[0]) ** 2 + (prev_object_y - object_global_pos[1]) ** 2
                    #rospy.logwarn("%f", distance)
                    if distance < min_distance:
                        min_distance = distance
                        target_object_coords = object_global_coords

                # calc uav target coords
                try:
                    cam2baselink = self.robot.getTF('hydrus/fc', parent_frame_id=self.object_pose.header.frame_id)
                    cam2baselink = ros_numpy.numpify(cam2baselink.transform)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logerr(self.__class__.__name__ + ": cannot find camera tf")
                    return 'failed'

                # set roll and pitch of target_object_coords to 0
                object_global_pos = tft.translation_from_matrix(target_object_coords)
                prev_object_x = object_global_pos[0]
                prev_object_y = object_global_pos[1]
                rospy.logerr("%s: succeed to find valid object x: %f, y: %f", self.__class__.__name__, object_global_pos[0], object_global_pos[1])
                #rospy.logerr("prev_x: %f, prev_y: %f", prev_object_x, prev_object_y)

                object_global_x_axis = target_object_coords[0:3, 0]
                object_global_yaw = np.arctan2(object_global_x_axis[1], object_global_x_axis[0])
                new_object_global_rot = tft.euler_matrix(0, 0, object_global_yaw)
                target_object_coords = tft.concatenate_matrices(tft.translation_matrix(object_global_pos), new_object_global_rot)

                object2camera = tft.concatenate_matrices(tft.translation_matrix([0, 0, object2cam_distance]), tft.euler_matrix(0, np.pi, np.pi/2))
                object2cam_distance -= self.pick_visual_servoing_z_diff

                uav_target_coords = tft.concatenate_matrices(target_object_coords, object2camera, cam2baselink)
                uav_target_pos = tft.translation_from_matrix(uav_target_coords)
                uav_target_yaw = tft.euler_from_matrix(uav_target_coords)[2]

                self.robot.goPos('global', uav_target_pos[0:2], uav_target_pos[2], uav_target_yaw)
                rospy.logwarn("%s: uav target x: %f, y: %f, z: %f, yaw: %f", self.__class__.__name__, uav_target_pos[0], uav_target_pos[1], uav_target_pos[2], uav_target_yaw)

                self.object_pose = PoseArray() #reset

            if object2cam_distance <= self.object_approach_height:
                userdata.target_object_pos[0] = prev_object_x
                userdata.target_object_pos[1] = prev_object_y

                return 'succeeded'

class Grasp(Task2State):
    def __init__(self, robot, add_object_model_func):
        Task2State.__init__(self, robot,
                            outcomes=['succeeded', 'failed'],
                            input_keys=['target_object_pos'],
                            output_keys=['target_object_pos'])

        self.add_object_model_func = add_object_model_func
        self.object_bbox_sub = rospy.Subscriber('cluster_decomposer/boxes', BoundingBoxArray, self.objectBBoxCallback)
        self.object_bbox = None
        self.object_center_pub = rospy.Publisher('~target_pose', PoseStamped, queue_size = 1)

        #moving average filter
        self.filter_buffer = []
        self.filter_buffer_length = 5

    def objectBBoxCallback(self, msg):
        q = Quaternion()
        if len(msg.boxes) == 0 or (len(msg.boxes) == 1 and msg.boxes[0].pose.orientation == q):
            self.object_bbox = None
            return
        self.object_bbox = msg

    def execute(self, userdata):
        self.waitUntilTaskStart()

        if self.skip_grasp:
            return 'succeeded'

        if userdata.target_object_pos:
            prev_object_x = userdata.target_object_pos[0]
            prev_object_y = userdata.target_object_pos[1]
        else:
            try:
                cam_trans = self.robot.getTF(self.object_bbox.header.frame_id, time=self.object_bbox.header.stamp)
                cam_trans = ros_numpy.numpify(cam_trans.transform)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr(self.__class__.__name__ + ": cannot find camera tf")
                return 'failed'

            object_global_coords = tft.concatenate_matrices(cam_trans, ros_numpy.numpify(self.object_bbox.boxes[0].pose))
            object_global_pos = tft.translation_from_matrix(object_global_coords)
            prev_object_x = object_global_pos[0]
            prev_object_y = object_global_pos[1]

        self.object_bbox = None #reset

        try:
            baselink2cam_trans = self.robot.getTF('hydrus/rs_d435_color_optical_frame', parent_frame_id='hydrus/fc')
            baselink2cam_trans = ros_numpy.numpify(baselink2cam_trans.transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr(self.__class__.__name__ + ": cannot find camera tf")
            return 'failed'

        #set object to link3 orig waypoints
        try:
            cam2link3_trans = self.robot.getTF('hydrus/link3', parent_frame_id='hydrus/rs_d435_color_optical_frame')
            cam2link3_trans = ros_numpy.numpify(cam2link3_trans.transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr(self.__class__.__name__ + ": cannot find camera tf")
            return 'failed'

        cam2link3_pos = tft.translation_from_matrix(cam2link3_trans)
        waypoint1 = np.array([-cam2link3_pos[1], 0, self.object_approach_height])

        grasping_point2link3 = tft.inverse_matrix(self.grasping_point_coords)
        grasping_point2link3_pos = tft.translation_from_matrix(grasping_point2link3)
        waypoint2 = np.array([grasping_point2link3_pos[0], grasping_point2link3_pos[1], self.object_approach_height])
        waypoint3 = np.array([grasping_point2link3_pos[0], grasping_point2link3_pos[1], self.grasp_vs_end_height])
        object2link3_rot = tft.quaternion_matrix(tft.quaternion_from_matrix(grasping_point2link3))

        object2baselink_waypoints = []

        try:
            link32baselink_trans = self.robot.getTF('hydrus/fc', parent_frame_id='hydrus/link3')
            link32baselink_trans = ros_numpy.numpify(link32baselink_trans.transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr(self.__class__.__name__ + ": cannot find camera tf")
            return 'failed'


        for i in range(self.grasp_horizontal_interpolation_time):
            p = 1.0 * (i + 1) / self.grasp_horizontal_interpolation_time
            object2link3_pos = waypoint1 * (1.0 - p) + waypoint2 * p
            object2link3_trans = tft.concatenate_matrices(tft.translation_matrix(object2link3_pos), object2link3_rot)

            object2baselink_waypoints.append(tft.concatenate_matrices(object2link3_trans, link32baselink_trans))

        for i in range(self.grasp_vertical_interpolation_time):
            p = 1.0 * (i + 1) / self.grasp_vertical_interpolation_time
            object2link3_pos = waypoint2 * (1.0 - p) + waypoint3 * p
            object2link3_trans = tft.concatenate_matrices(tft.translation_matrix(object2link3_pos), object2link3_rot)

            object2baselink_waypoints.append(tft.concatenate_matrices(object2link3_trans, link32baselink_trans))

        self.robot.preshape()

        #visual servoing
        r = rospy.Rate(10)
        # while not rospy.is_shutdown():
        #     r.sleep()
        #     if self.object_bbox is not None:

        #         #find most nearest object
        #         min_distance = 1e8
        #         target_object_bbox = None

        #         try:
        #             cam_trans = self.robot.getTF(self.object_bbox.header.frame_id, time=self.object_bbox.header.stamp)
        #             cam_trans = ros_numpy.numpify(cam_trans.transform)
        #         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #             rospy.logerr(self.__class__.__name__ + ": cannot find camera tf")
        #             return 'failed'

        #         rospy.logerr("prev_x: %f, prev_y: %f", prev_object_x, prev_object_y)

        #         for obj in self.object_bbox.boxes:
        #             object_global_coords = tft.concatenate_matrices(cam_trans, ros_numpy.numpify(obj.pose))
        #             object_global_pos = tft.translation_from_matrix(object_global_coords)
        #             distance = (prev_object_x - object_global_pos[0]) ** 2 + (prev_object_y - object_global_pos[1]) ** 2
        #             rospy.logwarn("%f", distance)
        #             if distance < min_distance:
        #                 min_distance = distance
        #                 target_object_bbox = obj

        #         #calc center of object
        #         target_object_bbox.pose.orientation.x = 0.0
        #         target_object_bbox.pose.orientation.y = 0.0
        #         target_object_bbox.pose.orientation.z = 0.0
        #         target_object_bbox.pose.orientation.w = 1.0
        #         bbox_center_coords = ros_numpy.numpify(target_object_bbox.pose)
        #         bbox_center_world_coords = tft.concatenate_matrices(cam_trans, bbox_center_coords)
        #         bbox_center_world_pos = tft.translation_from_matrix(bbox_center_world_coords)
        #         prev_object_x, prev_object_y = bbox_center_world_pos[0], bbox_center_world_pos[1]

        #         object_center_coords = tft.concatenate_matrices(bbox_center_coords, tft.translation_matrix([-target_object_bbox.dimensions.x / 2 + self.object_length / 2, 0, target_object_bbox.dimensions.z / 2]))
        #         object_center_worldcoords = tft.concatenate_matrices(cam_trans, object_center_coords)
        #         object_center_msg = PoseStamped()
        #         object_center_msg.header.stamp = self.object_bbox.header.stamp
        #         object_center_msg.header.frame_id = '/world'
        #         object_center_msg.pose = ros_numpy.msgify(Pose, object_center_worldcoords)
        #         self.object_center_pub.publish(object_center_msg) #for debug

        #         #set camera angle
        #         baselink_worldcoords = ros_numpy.numpify(self.robot.getBaselinkOdom().pose.pose)
        #         cam_worldcoords = tft.concatenate_matrices(baselink_worldcoords, baselink2cam_trans)
        #         object_coords_in_cam_frame = tft.concatenate_matrices(tft.inverse_matrix(cam_worldcoords), object_center_worldcoords)
        #         object_pos_in_cam_frame = tft.translation_from_matrix(object_coords_in_cam_frame)
        #         cam_angle = np.arctan2(object_pos_in_cam_frame[2], -object_pos_in_cam_frame[1])

        #         #moving average filter
        #         self.filter_buffer.append(cam_angle)
        #         filtered_cam_angle = sum(self.filter_buffer) / len(self.filter_buffer)
        #         if len(self.filter_buffer) >= self.filter_buffer_length:
        #             self.filter_buffer.pop(0)

        #         self.robot.setCameraJointAngle(filtered_cam_angle, time=0)
        #         #rospy.logwarn("%f", cam_angle)

        #         #calc uav target
        #         if object2baselink_waypoints:
        #             object2baselink_trans = object2baselink_waypoints.pop(0)
        #             uav_target_coords = tft.concatenate_matrices(object_center_worldcoords, object2baselink_trans)
        #             uav_target_pos = tft.translation_from_matrix(uav_target_coords)
        #             uav_target_yaw = tft.euler_from_matrix(uav_target_coords)[2]
        #             rospy.logwarn("%s: uav target x: %f, y: %f, z: %f, yaw: %f", self.__class__.__name__, uav_target_pos[0], uav_target_pos[1], uav_target_pos[2], uav_target_yaw)
        #             self.robot.goPos('global', uav_target_pos[0:2], uav_target_pos[2], None)
        #         else:
        #             break

        #         self.object_bbox = None #reset


        rospy.logwarn(self.__class__.__name__ + ": landing")
        self.robot.land()
        start_time = rospy.get_time()
        while not (self.robot.getFlightState() == self.robot.ARM_OFF_STATE):
            elapsed_time = rospy.get_time() - start_time
            if elapsed_time > 10.0:
                self.robot.halt()
                rospy.logwarn(self.__class__.__name__ + ": force halt")
                break

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

        #reset realsense odom
        if not self.simulation:
            rospy.logwarn(self.__class__.__name__ + ": reset realsense odom")

            #this process is no longer necessary after realsense v2.29.0
            #cmd = "rosrun mbzirc2020_task2_common reset_vo.sh"
            #subprocess.Popen(cmd.split())

            #just call reset
            try:
                req = std_srvs.srv.EmptyRequest()
                res = self.realsense_reset_service_client(req)

                if res is not None:
                    rospy.logwarn("Reset realsense")
                else:
                    rospy.logerr("Failed to reset realsense")

            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s", e)

            rospy.logerr("WARNING!! THE ROBOT WILL TAKE OFF AFTER 10 SEC")
            rospy.sleep(10)

        rospy.logwarn(self.__class__.__name__ + ": takeoff")
        self.robot.setCameraJointAngle(np.pi / 2)
        self.robot.startAndTakeoff()
        while not (self.robot.getFlightState() == self.robot.HOVER_STATE):
            pass #wait until hover

        return result

class ApproachPlaceArea(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, robot, outcomes=['succeeded'])

    def execute(self, userdata):
        self.waitUntilTaskStart()

        if self.skip_approach_place_area:
            return 'succeeded'

        if self.simulation:
            client = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
            req = ApplyBodyWrenchRequest()
            req.body_name = 'hydrus::root'
            req.wrench.force.z = 3
            req.duration.nsecs = 300000000
            try:
                res = client(req)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

        self.robot.goPosWaitConvergence('global', None, self.place_lookdown_height, None, pos_conv_thresh = 0.4, yaw_conv_thresh = 0.2, vel_conv_thresh = 0.2)

        #enable plane detection, disable alt sensor
        if not self.simulation:

            try:
                req = std_srvs.srv.SetBoolRequest()
                req.data = True
                res = self.plane_detection_service_client(req)

                if res is not None:
                    rospy.logwarn("Enable plane detection")
                else:
                    rospy.logerr("Failed to enable place detection")

            except rospy.ServiceException, e:
                rospy.logerr("Service call failed: %s", e)

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


        uav_target_yaw = self.global_place_channel_yaw - self.grasping_yaw_in_fc
        self.robot.goPosWaitConvergence('global', self.global_place_channel_center_pos_gps, None, uav_target_yaw, gps_mode = True, timeout=60, pos_conv_thresh = 0.3, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1)

        return 'succeeded'

class PlaceVisualServoing(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, robot,
                            outcomes=['succeeded', 'failed', 'adjust_again'],
                            input_keys=['orig_global_trans', 'search_count', 'search_failed'],
                            output_keys=['orig_global_trans', 'search_count', 'search_failed'])

    def execute(self, userdata):
        self.waitUntilTaskStart()

        if self.skip_place_visual_servoing:
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

        self.robot.goPosWaitConvergence('global', channel_pos[0:2], None, None, pos_conv_thresh = 0.25, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1, timeout = 15)

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
        Task2State.__init__(self, robot, outcomes=['finish'])

        self.remove_object_model_func = remove_object_model_func

    def execute(self, userdata):
        self.waitUntilTaskStart()

        if self.skip_ungrasp:
            return 'succeeded'


        channel_center_coords = tft.compose_matrix(translate=[self.robot.getTargetXY()[0], self.robot.getTargetXY()[1], 0], angles=[0, 0, self.global_place_channel_yaw])

        uav_target_coords = tft.concatenate_matrices(channel_center_coords, tft.inverse_matrix(self.grasping_point_coords))
        uav_target_pos = tft.translation_from_matrix(uav_target_coords)

        rospy.logwarn(self.__class__.__name__ + ": Go to x: %f, y: %f, yaw: %f", uav_target_pos[0], uav_target_pos[1], self.robot.getTargetYaw())
        self.robot.goPosWaitConvergence('global', uav_target_pos[0:2], None, None, pos_conv_thresh = 0.25, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1)

        #descend
        self.robot.goPosWaitConvergence('global', None, self.place_height, None, pos_conv_thresh = 0.15, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1, timeout = 10)

        self.robot.ungrasp()
        self.remove_object_model_func(self.robot)

        self.robot.goPosWaitConvergence('global', None, self.place_lookdown_height, None, pos_conv_thresh = 0.3, yaw_conv_thresh = 0.2, vel_conv_thresh = 0.3)
        self.robot.resetPose()

        return 'finish'

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

        self.robot.goPosWaitConvergence('global', [uav_target_pos[0], uav_target_pos[1]], None, None, pos_conv_thresh = 0.2, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1, timeout=self.grid_search_timeout)

        userdata.search_count += 1
        if userdata.search_count == len(self.grid):
            userdata.search_count = 0
            userdata.search_failed = True

            orig_pos = tft.translation_from_matrix(userdata.orig_global_trans)
            rospy.logwarn(self.__class__.__name__ + ": return to original pos")
            self.robot.goPosWaitConvergence('global', [orig_pos[0], orig_pos[1]], None, None, pos_conv_thresh = 0.1, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1, timeout=7)

        return 'succeeded'

class Finish(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, robot, outcomes=['succeeded'])

    def execute(self, userdata):
        self.robot.goPosWaitConvergence('global', [0, 0], None, None)
        rospy.loginfo('Landing')
        self.robot.land()
        return 'succeeded'
