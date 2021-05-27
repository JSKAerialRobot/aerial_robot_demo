#!/usr/bin/env python

import numpy as np

import rospy
import ros_numpy
import tf.transformations as tft
import smach
import smach_ros
import tf2_ros

from std_msgs.msg import Empty, String

from task2_hydrus_interface import Task2HydrusInterface

class Task2State(smach.State):
    def __init__(self, state_name, robot, outcomes=[], input_keys=[], output_keys=[], io_keys=[]):
        smach.State.__init__(self, outcomes, input_keys, output_keys, io_keys)
        self.state_pub = rospy.Publisher('~smach_state', String, queue_size=100)
        self.state = state_name

        self.manager_state = ""

        self.simulation = rospy.get_param('~simulation')
        self.outdoor = rospy.get_param('~outdoor')

        self.robot = robot

        self.manager_state_sub = rospy.Subscriber('/two_hydrus_task/smach_state', String, self.ManagerStateCallback)

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def ManagerStateCallback(self, msg):
        self.manager_state = msg.data


class Start(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, state_name=self.__class__.__name__, robot=robot, outcomes=['succeeded'])

    def execute(self, userdata):

        while not self.manager_state == self.state:
            rospy.sleep(0.1)

        self.robot.saveInitialPosition()

        self.publish_state()

        return 'succeeded'

class Grasp(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, state_name=self.__class__.__name__, robot=robot, outcomes=['succeeded'])


    def execute(self, userdata):

        while not self.manager_state == self.state:
            rospy.sleep(0.1)

        #self.robot.grasp()

        if not self.simulation:
            #self.robot.add_long_object_to_model(action="add")
            pass

        self.publish_state()

        return 'succeeded'

class Takeoff(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, state_name=self.__class__.__name__, robot=robot, outcomes=['succeeded'])

    def execute(self, userdata):

        while not self.manager_state == self.state:
            rospy.sleep(0.1)

        self.robot.start()

        rospy.sleep(0.5)

        self.robot.takeoff()

        self.publish_state()

        return 'succeeded'

class WaitForHovering(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, state_name=self.__class__.__name__, robot=robot, outcomes=['succeeded'])

    def execute(self, userdata):

        while not self.manager_state == self.state:
            rospy.sleep(0.1)

        self.robot.wait_for_hovering()

        self.publish_state()

        return 'succeeded'

class ChangeHeight(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, state_name=self.__class__.__name__, robot=robot, outcomes=['succeeded'])

    def execute(self, userdata):

        while not self.manager_state == self.state:
            rospy.sleep(0.1)

        if self.outdoor:
            self.robot.goPosHeightInterpolation('global', None, 3.5, None, gps_mode=False, time = 40000)
        elif not self.outdoor:
            self.robot.goPosHeightInterpolation('global', None, 1.2, None, gps_mode=False, time = 5000)

        self.publish_state()

        return 'succeeded'

class EnablePlaneDetection(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, state_name=self.__class__.__name__, robot=robot, outcomes=['succeeded'])

    def execute(self, userdata):

        while not self.manager_state == self.state:
            rospy.sleep(0.1)

        #self.robot.enable_plane_detection(flag = True)

        #self.robot.enable_alt_sensor(flag = False)

        self.publish_state()

        return 'succeeded'

class SetYawFree(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, state_name=self.__class__.__name__, robot=robot, outcomes=['succeeded'])

    def execute(self, userdata):

        while not self.manager_state == self.state:
            rospy.sleep(0.1)

        self.robot.set_yaw_free(flag=True)

        self.publish_state()

        return 'succeeded'

class LeaderApproachPlacePosition(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, state_name=self.__class__.__name__, robot=robot, outcomes=['succeeded'])
        
        if self.outdoor:
            self.global_place_channel_center_pos_gps = rospy.get_param('~global_place_channel_center_pos_gps')
            self.global_place_channel_yaw = rospy.get_param('~global_place_channel_yaw')
            self.place_channel_length = rospy.get_param('~place_channel_length')
        elif not self.outdoor:
            self.channel_center_pos = rospy.get_param('~channel_pos')

    def execute(self, userdata):

        while not self.manager_state == self.state:
            rospy.sleep(0.1)

        if self.outdoor:
            self.robot.goPosWaitConvergence(
                'global',
                self.global_place_channel_center_pos_gps, 
                None, 
                None, 
                gps_mode = True, 
                timeout=20, 
                pos_conv_thresh = 0.3, 
                yaw_conv_thresh = 1.0, 
                vel_conv_thresh = 0.1)

            baselink_pos = self.robot.getBaselinkPos()[0:2]
            rospy.loginfo(baselink_pos)
            rospy.loginfo(self.global_place_channel_yaw)
            target_pos = [baselink_pos[0]+(self.place_channel_length/2)*np.cos(self.global_place_channel_yaw),
                          baselink_pos[1]+(self.place_channel_length/2)*np.sin(self.global_place_channel_yaw)]
            rospy.loginfo(target_pos)
            self.robot.goPosWaitConvergence(
                'global', 
                target_pos, 
                None, 
                None, 
                timeout=10, 
                pos_conv_thresh = 0.3, 
                yaw_conv_thresh = 0.1, 
                vel_conv_thresh = 0.1)

        elif not self.outdoor:
            self.robot.goPosWaitConvergence(
                'global', 
                [self.channel_center_pos[0],self.channel_center_pos[1]-1.0], 
                None, 
                None, 
                timeout=10, 
                pos_conv_thresh = 0.3, 
                yaw_conv_thresh = 0.1, 
                vel_conv_thresh = 0.1)

        self.publish_state()

        return 'succeeded'

class LeaderAdjustPlacePosition(Task2State):
    def __init__(self, robot,robot_ns):
        Task2State.__init__(self, state_name=self.__class__.__name__, robot=robot, outcomes=['succeeded'])
        self.robot_ns = robot_ns
        self.recognition_wait = rospy.get_param('~recognition_wait')
        self.channel_pos_thresh = rospy.get_param('~channel_pos_thresh')

    def execute(self, userdata):

        while not self.manager_state == self.state:
            rospy.sleep(0.1)

        try:
            channel_trans = self.robot.getTF(self.robot_ns + '/channel_center', wait=self.recognition_wait)
            channel_trans = ros_numpy.numpify(channel_trans.transform)
            channel_pos = tft.translation_from_matrix(channel_trans)
            rospy.loginfo("succeed to find channel x: %f, y: %f", channel_pos[0], channel_pos[1])
            channel_pos_local = channel_pos - self.robot.getBaselinkPos()

            self.robot.goPosWaitConvergence(
                'global', 
                channel_pos[0:2], 
                None, 
                None, 
                pos_conv_thresh = 0.25, 
                yaw_conv_thresh = 0.1, 
                vel_conv_thresh = 0.1, 
                timeout = 10)

            # check channel is directly below
            if np.linalg.norm(channel_pos_local[0:2]) > self.channel_pos_thresh:
                rospy.logwarn("succeed to find channel, but not directly below. diff: %f", np.linalg.norm(channel_pos_local[0:2]))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("channel position detect failed")

        self.publish_state()

        return 'succeeded'

class LeaderNavigated(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, state_name=self.__class__.__name__, robot=robot, outcomes=['succeeded'])

    def execute(self, userdata):

        while not self.manager_state == self.state:
            rospy.sleep(0.1)

        self.robot.change_ctrl_mode(mode='vel')

        self.publish_state()

        return 'succeeded'

class AdjustHeight(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, state_name=self.__class__.__name__, robot=robot, outcomes=['succeeded'])

    def execute(self, userdata):

        while not self.manager_state == self.state:
            rospy.sleep(0.1)

        self.robot.change_ctrl_mode(mode='pos')

        rospy.sleep(3)

        if self.outdoor:
            self.robot.goPosHeightInterpolation('global', None, 2.3, None, gps_mode=False, time = 10000)
        elif not self.outdoor:
            self.robot.goPosHeightInterpolation('global', None, 0.9, None, gps_mode=False, time = 2000)

        self.publish_state()

        return 'succeeded'

class Ungrasp(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, state_name=self.__class__.__name__, robot=robot, outcomes=['succeeded'])

    def execute(self, userdata):

        while not self.manager_state == self.state:
            rospy.sleep(0.1)

        self.robot.ungrasp()

        self.publish_state()

        return 'succeeded'

class SetYawNotFree(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, state_name=self.__class__.__name__, robot=robot, outcomes=['succeeded'])

    def execute(self, userdata):

        while not self.manager_state == self.state:
            rospy.sleep(0.1)

        self.robot.set_yaw_free(flag=False)

        self.publish_state()

        return 'succeeded'

class LeaderBacktoStartPosition(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, state_name=self.__class__.__name__, robot=robot, outcomes=['succeeded'])

    def execute(self, userdata):

        while not self.manager_state == self.state:
            rospy.sleep(0.1)

        if self.outdoor:
            self.robot.goPosWaitConvergence(
                'global', 
                None, 
                3.0, 
                None, 
                timeout=10, 
                pos_conv_thresh = 0.3, 
                yaw_conv_thresh = 0.1, 
                vel_conv_thresh = 0.1)

        elif not self.outdoor:
            self.robot.goPosWaitConvergence(
                'global', 
                None, 
                1.0, 
                None, 
                timeout=10, 
                pos_conv_thresh = 0.3, 
                yaw_conv_thresh = 0.1, 
                vel_conv_thresh = 0.1)

        self.robot.goPosWaitConvergence(
            'global', 
            [self.robot.init_x,self.robot.init_y], 
            None, 
            None, 
            timeout=10, 
            pos_conv_thresh = 0.3, 
            yaw_conv_thresh = 0.1, 
            vel_conv_thresh = 0.1)

        self.publish_state()

        return 'succeeded'

class Landing(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, state_name=self.__class__.__name__, robot=robot, outcomes=['succeeded'])

    def execute(self, userdata):

        while not self.manager_state == self.state:
            rospy.sleep(0.1)

        self.robot.land()

        self.publish_state()

        return 'succeeded'

class Finish(Task2State):
    def __init__(self, robot):
        Task2State.__init__(self, state_name=self.__class__.__name__, robot=robot, outcomes=['succeeded'])

    def execute(self, userdata):

        while not self.manager_state == self.state:
            rospy.sleep(0.1)

        self.publish_state()

        return 'succeeded'

def main():

    robot_ns = rospy.get_param('~robot_ns', "hydrus")

    hydrus = Task2HydrusInterface(robot_ns=robot_ns)

    sm_top = smach.StateMachine(outcomes=['succeeded'])

    with sm_top:
        smach.StateMachine.add('Start', Start(hydrus),
                               transitions={'succeeded':'Grasp'})

        smach.StateMachine.add('Grasp', Grasp(hydrus),
                               transitions={'succeeded':'Takeoff'})

        smach.StateMachine.add('Takeoff', Takeoff(hydrus),
                               transitions={'succeeded':'WaitForHovering'})

        smach.StateMachine.add('WaitForHovering', WaitForHovering(hydrus),
                               transitions={'succeeded':'ChangeHeight'})

        smach.StateMachine.add('ChangeHeight', ChangeHeight(hydrus),
                               transitions={'succeeded':'EnablePlaneDetection'})

        smach.StateMachine.add('EnablePlaneDetection', EnablePlaneDetection(hydrus),
                               transitions={'succeeded':'SetYawFree'})

        smach.StateMachine.add('SetYawFree', SetYawFree(hydrus),
                               transitions={'succeeded':'LeaderApproachPlacePosition'})

        smach.StateMachine.add('LeaderApproachPlacePosition', LeaderApproachPlacePosition(hydrus),
                               transitions={'succeeded':'LeaderAdjustPlacePosition'})

        smach.StateMachine.add('LeaderAdjustPlacePosition', LeaderAdjustPlacePosition(hydrus,robot_ns = robot_ns),
                               transitions={'succeeded':'LeaderNavigated'})

        smach.StateMachine.add('LeaderNavigated', LeaderNavigated(hydrus),
                               transitions={'succeeded':'AdjustHeight'})

        smach.StateMachine.add('AdjustHeight', AdjustHeight(hydrus),
                               transitions={'succeeded':'Ungrasp'})

        smach.StateMachine.add('Ungrasp', Ungrasp(hydrus),
                               transitions={'succeeded':'SetYawNotFree'})

        smach.StateMachine.add('SetYawNotFree', SetYawNotFree(hydrus),
                               transitions={'succeeded':'LeaderBacktoStartPosition'})

        smach.StateMachine.add('LeaderBacktoStartPosition', LeaderBacktoStartPosition(hydrus),
                               transitions={'succeeded':'Landing'})

        smach.StateMachine.add('Landing', Landing(hydrus),
                               transitions={'succeeded':'Finish'})

        smach.StateMachine.add('Finish', Finish(hydrus),
                               transitions={'succeeded':'succeeded'})

        sis = smach_ros.IntrospectionServer('task2_smach_server', sm_top, '/TASK_MANAGER/LEADER')
        sis.start()

    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()

if __name__=="__main__":

    rospy.init_node('two_hydrus_task_leader')

    main()
