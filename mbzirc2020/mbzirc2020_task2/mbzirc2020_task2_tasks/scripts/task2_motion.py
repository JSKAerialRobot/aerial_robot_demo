#!/usr/bin/env python

import rospy
import smach
import smach_ros
from mbzirc2020_common.hydrus_interface import *
from sensor_msgs.msg import JointState
import numpy as np
from geometry_msgs.msg import Transform, Inertia
import tf.transformations as tft
import tf2_ros
import ros_numpy
from std_msgs.msg import UInt8, Empty
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest

def addObjectToModel(robot, action, translation, yaw, mass):
    transform = Transform()
    transform.translation.x = translation[0]
    transform.translation.y = translation[1]
    transform.translation.z = translation[2]
    quaternion = tft.quaternion_from_euler(0, 0, yaw)
    transform.rotation.x = quaternion[0]
    transform.rotation.y = quaternion[1]
    transform.rotation.z = quaternion[2]
    transform.rotation.w = quaternion[3]
    inertia = Inertia()
    inertia.m = mass
    inertia.com.x = 0.0
    inertia.com.y = 0.0
    inertia.com.z = 0.0
    inertia.ixx = 1/6.0 * mass * (0.2**2 + 0.2**2)
    inertia.iyy = 1/6.0 * mass * (0.2**2 + 0.3**2)
    inertia.izz = 1/6.0 * mass * (0.2**2 + 0.3**2)

    robot.addExtraModule(action, 'object', 'link2', transform, inertia)

class Start(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.robot = robot
        self.object_approach_height = rospy.get_param('~object_approach_height')
        self.start_sub = rospy.Subscriber('/task_start', Empty, self.taskStartCallback)
        self.task_start = False

    def taskStartCallback(self, msg):
        self.task_start = True

    def execute(self, userdata):
        while not self.task_start:
            pass

        rospy.loginfo('Takeoff')
        rospy.sleep(0.5)
        self.robot.startAndTakeoff()
        while not (self.robot.getFlightState() == self.robot.HOVER_STATE):
            pass
        self.robot.goPosWaitConvergence('global', self.robot.getBaselinkPos()[0:2], self.object_approach_height, self.robot.getBaselinkRPY()[2])

        joint_state = JointState()
        joint_state.name = ['rs_d435_servo_joint']
        joint_state.position = [np.pi / 2] #look down
        self.robot.setExtraJointAngle(joint_state, time = 1000)
        return 'succeeded'

class Finish(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.robot = robot

    def execute(self, userdata):
        self.robot.goPosWaitConvergence('global', [0, 0], self.robot.getBaselinkPos()[2], 0.0)
        rospy.loginfo('Landing')
        self.robot.land()
        return 'succeeded'

class MoveToGraspPosition(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded'], input_keys=['object_count'], output_keys=['object_count'])
        self.robot = robot
        self.global_object_pos = rospy.get_param('~global_object_pos')
        self.object_approach_height = rospy.get_param('~object_approach_height')
        self.preshape_joint_angle = rospy.get_param('~preshape_joint_angle')

        grasping_point = rospy.get_param('~grasping_point')
        grasping_yaw = rospy.get_param('~grasping_yaw')
        self.grasping_point_coords = tft.compose_matrix(translate=[grasping_point[0], grasping_point[1], grasping_point[2]], angles=[0, 0, grasping_yaw])

    def execute(self, userdata):
        #preshape
        joint_state = JointState()
        joint_state.name = ['joint1', 'joint3']
        joint_state.position = self.preshape_joint_angle
        self.robot.setJointAngle(joint_state, time = 3000)

        #calc uav target coords
        target_object_pos = self.global_object_pos[userdata.object_count]
        object_global_coords = tft.compose_matrix(translate=[target_object_pos[0], target_object_pos[1], self.object_approach_height], angles=[0, 0, target_object_pos[2]])
        uav_target_coords = tft.concatenate_matrices(object_global_coords, tft.inverse_matrix(self.grasping_point_coords))
        uav_target_pos = tft.translation_from_matrix(uav_target_coords)
        uav_target_yaw = tft.euler_from_matrix(uav_target_coords)[2]

        self.robot.goPosWaitConvergence('global', [uav_target_pos[0], uav_target_pos[1]], self.robot.getBaselinkPos()[2], uav_target_yaw, pos_conv_thresh = 0.1, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1)
        self.robot.goPosWaitConvergence('global', [uav_target_pos[0], uav_target_pos[1]], uav_target_pos[2], uav_target_yaw, pos_conv_thresh = 0.1, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1)

        while True:
            pass
        return 'succeeded'

class AdjustGraspPosition(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.robot = robot
        self.object_approach_height = rospy.get_param('~object_approach_height')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        grasping_point = rospy.get_param('~grasping_point')
        grasping_yaw = rospy.get_param('~grasping_yaw')
        self.grasping_point_coords = tft.compose_matrix(translate=[grasping_point[0], grasping_point[1], grasping_point[2]], angles=[0, 0, grasping_yaw])

    def execute(self, userdata):
        try:
            trans = self.tf_buffer.lookup_transform('world', 'target_object_color', rospy.Time(), rospy.Duration(1.0))
            rospy.logwarn("found object! %f %f %f", trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return 'failed'

        object_global_coords = ros_numpy.numpify(trans.transform)
        object_global_x_axis = object_global_coords[0:3, 0]
        object_global_yaw = np.arctan2(object_global_x_axis[1], object_global_x_axis[0])
        object_global_coords_modified = tft.compose_matrix(translate=tft.translation_from_matrix(object_global_coords), angles=[0, 0, object_global_yaw])

        uav_target_coords = tft.concatenate_matrices(object_global_coords_modified, tft.inverse_matrix(self.grasping_point_coords))
        uav_target_pos = tft.translation_from_matrix(uav_target_coords)
        uav_target_yaw = tft.euler_from_matrix(uav_target_coords)[2]

        self.robot.goPosWaitConvergence('global', [uav_target_pos[0], uav_target_pos[1]], self.object_approach_height, uav_target_yaw, pos_conv_thresh = 0.1, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1)
        self.robot.goPosWaitConvergence('global', [uav_target_pos[0], uav_target_pos[1]], uav_target_pos[2], uav_target_yaw, pos_conv_thresh = 0.1, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.1)

        return 'succeeded'

class Grasp(smach.State):
    def __init__(self, robot, add_object_model_func):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.robot = robot
        self.add_object_model_func = add_object_model_func
        self.grasp_joint_angle = rospy.get_param('~grasp_joint_angle')

    def execute(self, userdata):
        joint_state = JointState()
        joint_state.name = ['joint1', 'joint3']
        joint_state.position = self.grasp_joint_angle
        self.robot.setJointAngle(joint_state, time = 3000)
        rospy.sleep(1);
        self.add_object_model_func(self.robot)
        return 'succeeded'

class MoveToPlacePosition(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['object_count'],
                             output_keys=['object_count'])

        self.robot = robot
        self.simulation = rospy.get_param('/simulation')
        self.global_place_channel_pos = rospy.get_param('~global_place_channel_pos')
        self.global_place_channel_z = rospy.get_param('~global_place_channel_z')
        self.place_z_margin = rospy.get_param('~place_z_margin')
        self.place_z_offset = rospy.get_param('~place_z_offset')
        grasping_point = rospy.get_param('~grasping_point')
        grasping_yaw = rospy.get_param('~grasping_yaw')
        self.grasping_point_coords = tft.compose_matrix(translate=[grasping_point[0], grasping_point[1], grasping_point[2]], angles=[0, 0, grasping_yaw])

    def execute(self, userdata):
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

        self.robot.goPosWaitConvergence('global', self.robot.getBaselinkPos()[0:2], self.global_place_channel_z + self.place_z_offset, self.robot.getBaselinkRPY()[2], pos_conv_thresh = 0.2, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.2)
        self.robot.goPosWaitConvergence('global', uav_target_pos[0:2], self.global_place_channel_z + self.place_z_offset, uav_target_yaw, timeout=60)
        self.robot.goPosWaitConvergence('global', uav_target_pos[0:2], self.global_place_channel_z + self.place_z_margin, uav_target_yaw)
        return 'succeeded'

class Ungrasp(smach.State):
    def __init__(self, robot, remove_object_model_func):
        smach.State.__init__(self, outcomes=['finish', 'continue'],
                             input_keys=['object_count'],
                             output_keys=['object_count'])
        self.robot = robot
        self.remove_object_model_func = remove_object_model_func
        self.ungrasp_joint_angle = rospy.get_param('~ungrasp_joint_angle')
        self.global_place_channel_z = rospy.get_param('~global_place_channel_z')
        self.place_z_offset = rospy.get_param('~place_z_offset')
        self.object_num = rospy.get_param('~object_num')

        self.object_count_pub = rospy.Publisher('/object_count', UInt8, queue_size = 1) #for dummy object pos publisher

    def execute(self, userdata):
        joint_state = JointState()
        joint_state.name = ['joint1', 'joint3']
        joint_state.position = self.ungrasp_joint_angle
        self.robot.setJointAngle(joint_state, time = 3000)
        self.remove_object_model_func(self.robot)
        userdata.object_count += 1
        self.object_count_pub.publish(userdata.object_count)
        self.robot.goPosWaitConvergence('global', self.robot.getBaselinkPos()[0:2], self.global_place_channel_z + self.place_z_offset, self.robot.getBaselinkRPY()[2], pos_conv_thresh = 0.2, yaw_conv_thresh = 0.1, vel_conv_thresh = 0.2)

        if userdata.object_count == self.object_num:
            return 'finish'
        else:
            return 'continue'

def main():
    rospy.init_node('mbzirc2020_task2_motion')
    sm_top = smach.StateMachine(outcomes=['succeeded'])
    sm_top.userdata.object_count = 0

    robot = HydrusInterface()

    object_translation = rospy.get_param('~object_translation')
    object_yaw = rospy.get_param('~object_yaw')
    object_mass = rospy.get_param('~object_mass')
    add_object_model_func = lambda robot: addObjectToModel(robot, 'add', object_translation, object_yaw, object_mass)
    remove_object_model_func = lambda robot: addObjectToModel(robot, 'remove', object_translation, object_yaw, object_mass)

    with sm_top:
        smach.StateMachine.add('Start', Start(robot),
                               transitions={'succeeded':'Pick'})

        sm_pick = smach.StateMachine(outcomes=['pick_succeeded'], input_keys=['object_count'], output_keys=['object_count'])

        with sm_pick:
            smach.StateMachine.add('MoveToGraspPosition', MoveToGraspPosition(robot),
                                   transitions={'succeeded':'AdjustGraspPosition'})

            smach.StateMachine.add('AdjustGraspPosition', AdjustGraspPosition(robot),
                                   transitions={'succeeded':'Grasp',
                                                'failed':'Grasp'})

            smach.StateMachine.add('Grasp', Grasp(robot, add_object_model_func),
                                   transitions={'succeeded':'pick_succeeded'})


        smach.StateMachine.add('Pick', sm_pick,
                               transitions={'pick_succeeded':'Place'})

        sm_place = smach.StateMachine(outcomes=['finish', 'continue'], input_keys=['object_count'], output_keys=['object_count'])

        with sm_place:
            smach.StateMachine.add('MoveToPlacePosition', MoveToPlacePosition(robot),
                                   transitions={'succeeded':'Ungrasp'},
                                   remapping={'object_count':'object_count'})

            smach.StateMachine.add('Ungrasp', Ungrasp(robot, remove_object_model_func),
                                   transitions={'finish':'finish',
                                                'continue':'continue'},
                                   remapping={'object_count':'object_count'})

        smach.StateMachine.add('Place', sm_place,
                               transitions={'finish':'Finish',
                                            'continue':'Pick'},
                               remapping={'object_count':'object_count'})

        smach.StateMachine.add('Finish', Finish(robot),
                               transitions={'succeeded':'succeeded'})

        sis = smach_ros.IntrospectionServer('task2_smach_server', sm_top, '/SM_ROOT')
        sis.start()

    outcome = sm_top.execute()

    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
