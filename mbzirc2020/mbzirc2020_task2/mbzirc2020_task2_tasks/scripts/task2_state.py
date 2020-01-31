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
        Task2State.__init__(self, robot, outcomes=['succeeded'])
        self.object_lookdown_height = rospy.get_param('~object_lookdown_height')
        self.skip_takeoff = rospy.get_param('~skip_takeoff', False)

    def execute(self, userdata):
        self.waitUntilTaskStart()

        self.robot.setXYPosOffset(self.robot.getBaselinkPos()[0:2])

        if self.skip_takeoff:
            return 'succeeded'

        rospy.loginfo('Takeoff')
        rospy.sleep(0.5)
        self.robot.startAndTakeoff()
        while not (self.robot.getFlightState() == self.robot.HOVER_STATE):
            pass
        self.robot.goPosWaitConvergence('global', self.robot.getTargetXY(), self.object_lookdown_height, self.robot.getTargetYaw())

        self.robot.setCameraJointAngle(np.pi / 2)
        return 'succeeded'

class MoveToGraspPosition(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, robot, outcomes=['succeeded'], input_keys=['object_count'], output_keys=['object_count'])
        self.global_object_pos = rospy.get_param('~global_object_pos')
        self.object_lookdown_height = rospy.get_param('~object_lookdown_height')
        self.grasping_yaw = rospy.get_param('~grasping_yaw')
        self.skip_move_to_grasp_position = rospy.get_param('~skip_move_to_grasp_position', False)

    def execute(self, userdata):
        self.waitUntilTaskStart()

        if self.skip_move_to_grasp_position:
            return 'succeeded'

        #calc uav target coords
        target_object_pos = copy.copy(self.global_object_pos[userdata.object_count])
        target_object_pos[2] -= self.grasping_yaw

        self.robot.goPosWaitConvergence('global', [target_object_pos[0], target_object_pos[1]], self.object_lookdown_height, target_object_pos[2], pos_conv_thresh = 0.2, yaw_conv_thresh = 0.2, vel_conv_thresh = 0.2)
        self.robot.goPosWaitConvergence('global', [target_object_pos[0], target_object_pos[1]], self.object_lookdown_height, target_object_pos[2], pos_conv_thresh = 0.1, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1)

        return 'succeeded'

class LookDown(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, robot, outcomes=['succeeded', 'failed'])
        self.object_approach_height = rospy.get_param('~object_approach_height')
        self.object_yaw_thresh = rospy.get_param('~object_yaw_thresh')
        self.grasping_yaw = rospy.get_param('~grasping_yaw')
        self.skip_look_down = rospy.get_param('~skip_look_down', False)
        self.do_object_recognition = rospy.get_param('~do_object_recognition')

        self.object_pose_sub = rospy.Subscriber('rectangle_detection_color/target_object_color', PoseArray, self.objectPoseCallback)
        self.object_pose = PoseArray()

    def objectPoseCallback(self, msg):
        self.object_pose = msg

    def execute(self, userdata):
        self.waitUntilTaskStart()

        if self.skip_look_down:
            return 'succeeded'

        if not self.do_object_recognition:
            rospy.logwarn('no recognition: skip LookDown')
            return 'succeeded'

        if (len(self.object_pose.poses) != 0) and (rospy.Time.now() - self.object_pose.header.stamp).to_sec() < 0.5 and len(self.object_pose.poses) != 0:
            try:
                cam_trans = self.robot.getTF(self.object_pose.header.frame_id)
                cam_trans = ros_numpy.numpify(cam_trans.transform)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                return 'failed'
        else:
            return 'failed'

        #determine valid target object
        #detect most foreground object
        object_poses = self.object_pose.poses
        object_poses = sorted(object_poses, key=lambda x: x.position.y)
        target_object_pose = object_poses[-1]

        #check the validity in terms of yaw angle (yaw angle should be around -pi/2 in camera frame)
        object_yaw = tft.euler_from_quaternion(ros_numpy.numpify(target_object_pose.orientation))[2]
        if abs(object_yaw - (-np.pi / 2)) > self.object_yaw_thresh:
            rospy.logerror("invalide yaw")
            return 'failed'

        #succeeded to find object, then move

        object_global_coords = tft.concatenate_matrices(cam_trans, ros_numpy.numpify(target_object_pose))
        object_global_x_axis = object_global_coords[0:3, 0]
        object_global_yaw = np.arctan2(object_global_x_axis[1], object_global_x_axis[0])
        object_global_pos = tft.translation_from_matrix(object_global_coords)
        uav_target_yaw = object_global_yaw - self.grasping_yaw

        rospy.logwarn("succeed to find valid object: %f, %f", object_global_pos[0], object_global_pos[1])
        self.robot.goPosWaitConvergence('global', [object_global_pos[0], object_global_pos[1]], self.robot.getTargetZ(), uav_target_yaw, pos_conv_thresh = 0.2, yaw_conv_thresh = 0.2, vel_conv_thresh = 0.2)
        self.robot.goPosWaitConvergence('global', [object_global_pos[0], object_global_pos[1]], self.object_approach_height, uav_target_yaw, pos_conv_thresh = 0.1, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1)

        return 'succeeded'

class AdjustGraspPosition(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, robot, outcomes=['succeeded', 'failed'], input_keys=['object_count'], output_keys=['object_count'])
        self.object_grasping_height = rospy.get_param('~object_grasping_height')
        self.object_yaw_thresh = rospy.get_param('~object_yaw_thresh')
        self.skip_adjust_grasp_position = rospy.get_param('~skip_adjust_grasp_position', False)
        self.do_object_recognition = rospy.get_param('~do_object_recognition')
        self.global_object_pos = rospy.get_param('~global_object_pos')

        grasping_point = rospy.get_param('~grasping_point')
        grasping_yaw = rospy.get_param('~grasping_yaw')
        self.grasping_point_coords = tft.compose_matrix(translate=[grasping_point[0], grasping_point[1], grasping_point[2]], angles=[0, 0, grasping_yaw])

        self.object_pose_sub = rospy.Subscriber('rectangle_detection_depth/target_object_depth', PoseArray, self.objectPoseCallback)
        self.object_pose = PoseArray()

    def objectPoseCallback(self, msg):
        self.object_pose = msg

    def execute(self, userdata):
        self.waitUntilTaskStart()

        if self.skip_adjust_grasp_position:
            return 'succeeded'

        if self.do_object_recognition:
            if (len(self.object_pose.poses) != 0) and (rospy.Time.now() - self.object_pose.header.stamp).to_sec() < 0.5:
                try:
                    cam_trans = self.robot.getTF(self.object_pose.header.frame_id)
                    cam_trans = ros_numpy.numpify(cam_trans.transform)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    return 'failed'
            else:
                return 'failed'

            #determine valid target object
            #detect most center object
            object_poses = self.object_pose.poses
            object_poses = sorted(object_poses, key=lambda x: x.position.x ** 2 + x.position.y ** 2)
            target_object_pose = object_poses[0]

            #check the validity in terms of yaw angle (yaw angle should be around -pi/2 in camera frame)
            object_yaw = tft.euler_from_quaternion(ros_numpy.numpify(target_object_pose.orientation))[2]
            if abs(object_yaw - (-np.pi / 2)) > self.object_yaw_thresh:
                rospy.logerr("invalide yaw")
                return 'failed'

            object_global_coords = tft.concatenate_matrices(cam_trans, ros_numpy.numpify(target_object_pose))
            object_global_pos = tft.translation_from_matrix(object_global_coords)
            object_global_x_axis = object_global_coords[0:3, 0]
            object_global_yaw = np.arctan2(object_global_x_axis[1], object_global_x_axis[0])
            rospy.logwarn("succeed to find valid object x: %f, y: %f, yaw: %f", object_global_pos[0], object_global_pos[1], object_global_yaw)

            object_global_coords_modified = tft.compose_matrix(translate=tft.translation_from_matrix(object_global_coords), angles=[0, 0, object_global_yaw])

            uav_target_coords = tft.concatenate_matrices(object_global_coords_modified, tft.inverse_matrix(self.grasping_point_coords))
            uav_target_pos = tft.translation_from_matrix(uav_target_coords)
            uav_target_yaw = tft.euler_from_matrix(uav_target_coords)[2]

        else: #no recognition
            rospy.logwarn('no recognition: skip AdjustGraspPosition')
            target_object_pos = copy.copy(self.global_object_pos[userdata.object_count])
            object_global_coords = tft.compose_matrix(translate=[target_object_pos[0], target_object_pos[1], 0], angles=[0, 0, target_object_pos[2]])
            uav_target_coords = tft.concatenate_matrices(object_global_coords, tft.inverse_matrix(self.grasping_point_coords))
            uav_target_pos = tft.translation_from_matrix(uav_target_coords)
            uav_target_yaw = tft.euler_from_matrix(uav_target_coords)[2]

            rospy.logwarn("go to x: %f, y: %f, yaw: %f", uav_target_pos[0], uav_target_pos[1], uav_target_yaw)

        self.robot.preshape()
        self.robot.goPosWaitConvergence('global', [uav_target_pos[0], uav_target_pos[1]], self.robot.getTargetZ(), uav_target_yaw, pos_conv_thresh = 0.1, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1)
        self.robot.goPosWaitConvergence('global', [uav_target_pos[0], uav_target_pos[1]], self.object_grasping_height, uav_target_yaw, pos_conv_thresh = 0.1, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1)

        return 'succeeded'

class Grasp(Task2State):
    def __init__(self, robot, add_object_model_func):
        Task2State.__init__(self, robot, outcomes=['succeeded', 'failed'])
        self.add_object_model_func = add_object_model_func
        self.joint_torque_thresh = rospy.get_param('~joint_torque_thresh')
        self.skip_grasp = rospy.get_param('~skip_grasp', False)

    def execute(self, userdata):
        self.waitUntilTaskStart()

        if self.skip_grasp:
            return 'succeeded'

        self.robot.grasp()
        rospy.sleep(1);
        joint_state = self.robot.getJointState()
        joint_torque = []
        joint_torque.append(joint_state.effort[joint_state.name.index('joint1')])
        joint_torque.append(joint_state.effort[joint_state.name.index('joint3')])

        if all(np.array(joint_torque) > self.joint_torque_thresh):
            self.add_object_model_func(self.robot)
            return 'succeeded'
        else:
            return 'failed'

class MoveToPlacePosition(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, robot, outcomes=['succeeded'],
                            input_keys=['object_count'],
                            output_keys=['object_count'])

        self.simulation = rospy.get_param('/simulation')
        self.global_place_channel_pos = rospy.get_param('~global_place_channel_pos')
        self.global_place_channel_z = rospy.get_param('~global_place_channel_z')
        self.place_z_offset = rospy.get_param('~place_z_offset')
        self.skip_move_to_place_position = rospy.get_param('~skip_move_to_place_position', False)

        grasping_point = rospy.get_param('~grasping_point')
        grasping_yaw = rospy.get_param('~grasping_yaw')
        self.grasping_point_coords = tft.compose_matrix(translate=[grasping_point[0], grasping_point[1], grasping_point[2]], angles=[0, 0, grasping_yaw])

    def execute(self, userdata):
        self.waitUntilTaskStart()

        if self.skip_move_to_place_position:
            return 'succeeded'

        #calc place coords
        target_channel_pos = self.global_place_channel_pos[0]
        object_count_in_channel = (userdata.object_count % 4) + 1
        place_pos_x = target_channel_pos[0] + (target_channel_pos[2] - target_channel_pos[0]) * object_count_in_channel / 5.0
        place_pos_y = target_channel_pos[1] + (target_channel_pos[3] - target_channel_pos[1]) * object_count_in_channel / 5.0
        place_pos_yaw = np.arctan2(target_channel_pos[1] - target_channel_pos[3], target_channel_pos[0] - target_channel_pos[2])
        place_pos_coords = tft.compose_matrix(translate=[place_pos_x, place_pos_y, self.global_place_channel_z], angles=[0, 0, place_pos_yaw])

        uav_target_coords = tft.concatenate_matrices(place_pos_coords, tft.inverse_matrix(self.grasping_point_coords))
        uav_target_pos = tft.translation_from_matrix(uav_target_coords)
        uav_target_yaw = tft.euler_from_matrix(uav_target_coords)[2]

        if self.simulation:
            client = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
            req = ApplyBodyWrenchRequest()
            req.body_name = 'hydrusx::root'
            req.wrench.force.z = 10
            req.duration.nsecs = 300000000
            try:
                res = client(req)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

        self.robot.goPosWaitConvergence('global', self.robot.getTargetXY(), self.global_place_channel_z + self.place_z_offset, self.robot.getTargetYaw(), pos_conv_thresh = 0.2, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.2)
        self.robot.goPosWaitConvergence('global', uav_target_pos[0:2], self.global_place_channel_z + self.place_z_offset, uav_target_yaw, timeout=60)
        return 'succeeded'

class AdjustPlacePosition(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, robot, outcomes=['succeeded', 'failed'])
        self.skip_adjust_place_position = rospy.get_param('~skip_adjust_place_position', False)
        self.do_channel_recognition = rospy.get_param('~do_channel_recognition')
        self.channel_tf_frame_id = rospy.get_param('~channel_tf_frame_id')
        self.global_place_channel_z = rospy.get_param('~global_place_channel_z')

        grasping_point = rospy.get_param('~grasping_point')
        self.grasping_yaw = rospy.get_param('~grasping_yaw')
        self.grasping_point_coords = tft.compose_matrix(translate=[grasping_point[0], grasping_point[1], grasping_point[2]], angles=[0, 0, self.grasping_yaw])

    def execute(self, userdata):
        self.waitUntilTaskStart()

        if self.skip_adjust_place_position:
            return 'succeeded'

        if self.do_channel_recognition:
            try:
                channel_trans = self.robot.getTF(self.channel_tf_frame_id, wait=2.0)
                channel_trans = ros_numpy.numpify(channel_trans.transform)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("channel position detect failed")
                return 'failed'

            my_object_global_yaw = self.grasping_yaw + self.robot.getBaselinkRPY()[2]
            channel_center_coords = tft.compose_matrix(translate=tft.translation_from_matrix(channel_trans), angles=[0, 0, my_object_global_yaw])
            uav_target_coords = tft.concatenate_matrices(channel_center_coords, tft.inverse_matrix(self.grasping_point_coords))
            uav_target_pos = tft.translation_from_matrix(uav_target_coords)


            self.robot.goPosWaitConvergence('global', uav_target_pos[0:2], self.robot.getTargetZ(), self.robot.getTargetYaw(), pos_conv_thresh = 0.1, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1)

        return 'succeeded'


class Ungrasp(Task2State):
    def __init__(self, robot, remove_object_model_func):
        Task2State.__init__(self, robot, outcomes=['finish', 'continue'],
                             input_keys=['object_count'],
                             output_keys=['object_count'])

        self.remove_object_model_func = remove_object_model_func
        self.global_place_channel_z = rospy.get_param('~global_place_channel_z')
        self.place_z_margin = rospy.get_param('~place_z_margin')
        self.place_z_offset = rospy.get_param('~place_z_offset')
        self.object_num = rospy.get_param('~object_num')
        self.skip_ungrasp = rospy.get_param('~skip_ungrasp', False)

        self.object_count_pub = rospy.Publisher('/object_count', UInt8, queue_size = 1) #for dummy object pos publisher

    def execute(self, userdata):
        self.waitUntilTaskStart()

        if self.skip_ungrasp:
            return 'succeeded'

        #descend
        self.robot.goPosWaitConvergence('global', self.robot.getTargetXY(), self.global_place_channel_z + self.place_z_margin, self.robot.getTargetYaw(), pos_conv_thresh = 0.1, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1)

        self.robot.ungrasp()
        self.remove_object_model_func(self.robot)
        userdata.object_count += 1
        self.object_count_pub.publish(userdata.object_count)
        self.robot.goPosWaitConvergence('global', self.robot.getTargetXY(), self.global_place_channel_z + self.place_z_offset, self.robot.getTargetYaw(), pos_conv_thresh = 0.2, yaw_conv_thresh = 0.2, vel_conv_thresh = 0.3)
        self.robot.resetPose()

        if userdata.object_count == self.object_num:
            return 'finish'
        else:
            return 'continue'

class Finish(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.robot = robot

    def execute(self, userdata):
        self.robot.goPosWaitConvergence('global', [0, 0], self.robot.getBaselinkPos()[2], self.robot.getBaselinkRPY()[2])
        rospy.loginfo('Landing')
        self.robot.land()
        return 'succeeded'
