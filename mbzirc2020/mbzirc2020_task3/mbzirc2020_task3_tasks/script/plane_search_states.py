#!/usr/bin/env python

import rospy
import smach
import smach_ros

import tf
import tf2_ros
import numpy as np

from geometry_msgs.msg import Vector3Stamped

from hydrus_commander import HydrusCommander

class PlaneFireFightStateMachineCreator():
    def create(self, search_method):
        ''' search_method: {'rectangular_grid'}
            params: dictionary'''
        if search_method=='rectangular_grid':
            sm = smach.StateMachine(outcomes={'success', 'failure'})
            with sm:
                smach.StateMachine.add('searching', RectangularGridSearchState(),
                                        transitions={'found':'approach_on_top',
                                                     'not_found':'failure',
                                                     'still_searching':'searching'},
                                        remapping={'target_pos':'target_pos'})
                smach.StateMachine.add('approach_on_top', AproachOnTargetState(),
                                       transitions={'success':'covering',
                                                    'target_lost':'searching',
                                                    'ongoing':'approach_on_top'},
                                        remapping={'target_pos':'target_pos'})
                smach.StateMachine.add('covering', CoveringState(),
                                       transitions={'done':'success'})
            return sm


class RectangularGridSearchState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['found', 'not_found', 'still_searching'],
                                   output_keys=['target_pos'])
        self.commander = HydrusCommander()

        # retrieve parameters
        self.target_topic_name = rospy.get_param('~target_topic_name', '/target_object/pos')
        self.control_rate = rospy.get_param('~control_rate', 5.0)
        self.area_corners = rospy.get_param('~area_corners', [[-5,5],[5,5]])
        self.search_grid_size = rospy.get_param('~search_grid_size', 1.0)
        self.reach_margin = rospy.get_param('~reach_margin', 0.2)
        self.search_height = rospy.get_param('~search_height', 3.0)
        self.search_mode = rospy.get_param('~search_mode', 1)

        self.target_pos = None # set when target_pos topic recieved

        # tf Buffer
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # define subscriber
        self.target_pos_sub_ = rospy.Subscriber(self.target_topic_name, Vector3Stamped, self.targetPositionCallback)
        self.target_pos_flag = False

        if self.search_mode == 0:
            self.searched_grid = np.zeros((int((self.area_corners[1][0]-self.area_corners[0][0])/self.search_grid_size), int((self.area_corners[1][1]-self.area_corners[0][1])/self.search_grid_size)), dtype=bool)
            self.start_idx = [0, 0]

        elif self.search_mode == 1:
            self.searched_grid = np.zeros((int(2*int(((self.area_corners[1][0]-self.area_corners[0][0])/(2*self.search_grid_size)))+1), int(2*int(((self.area_corners[1][1]-self.area_corners[0][1])/(2*self.search_grid_size)))+1)), dtype=bool)
            self.start_idx = [int(((self.area_corners[1][0]-self.area_corners[0][0])/(2*self.search_grid_size))), int(((self.area_corners[1][1]-self.area_corners[0][1])/(2*self.search_grid_size)))]

        self.current_grid_idx = [0,0]
        self.x_move_dir = 0
        self.y_move_dir = 0
        self.thresh_num = 1
        self.count = 0

    def grid_pos(self, grid_idx):
        """return position from grid index"""
        # TODO check valid range
        if self.search_mode == 0:
            start_pos = self.area_corners[0]
        elif self.search_mode == 1:
            start_pos = [(self.area_corners[1][0]-self.area_corners[0][0])/2, (self.area_corners[1][1]-self.area_corners[0][1])/2]

        return (start_pos[0]+self.search_grid_size*grid_idx[0],
                start_pos[1]+self.search_grid_size*grid_idx[1])

    def is_grid_full(self):
        return np.all(self.searched_grid)

    def next_grid_idx_naive(self):
        result_idx = [0,0]
        if self.current_grid_idx[0] < self.searched_grid.shape[0]:
            result_idx[0] = self.current_grid_idx[0]+1
            result_idx[1] = self.current_grid_idx[1]
        elif self.current_grid_idx[1] < self.searched_grid.shape[1]:
            result_idx[0] = 0
            result_idx[1] = self.current_grid_idx[1] + 1
        return result_idx

    def move_to_next_idx(self):
        self.current_grid_idx[0] = self.current_grid_idx[0] + self.x_move_dir
        self.current_grid_idx[1] = self.current_grid_idx[1] + self.y_move_dir


        # conventional serch mode
        if self.search_mode == 0:
            # TODO make it more efficient
            if self.current_grid_idx[0] == self.searched_grid.shape[0]-1:
                if self.y_move_dir == 0 and self.x_move_dir == 1:
                    self.x_move_dir = 0
                    self.y_move_dir = 1
                else:
                    self.x_move_dir = -1
                    self.y_move_dir = 0
            elif self.current_grid_idx[0] == 0:
                if self.y_move_dir == 0 and self.x_move_dir == -1:
                    self.x_move_dir = 0
                    self.y_move_dir = 1
                else:
                    self.x_move_dir = 1
                    self.y_move_dir = 0


        # vortex serch mode
        if self.search_mode == 1:
            if self.count < self.thresh_num:
                self.x_move_dir = (-1)**(self.thresh_num+1)
                self.y_move_dir = 0

            elif self.count >= self.thresh_num and self.count < 2*self.thresh_num:
                self.x_move_dir = 0
                self.y_move_dir = (-1)**(self.thresh_num+1)

            self.count+=1

            if self.count == 2*self.thresh_num:
                self.count = 0
                self.thresh_num += 1

        return

    def targetPositionCallback(self, msg):
        """set target position if receiving topic"""
        self.target_pos_flag = True
        self.target_pos = msg

    def execute(self, userdata):
        # retrieve cog position from tf
        if self.target_pos_flag:
            userdata.target_pos = self.target_pos
            return 'found'
        try:
            self.cog_pos = self.tfBuffer.lookup_transform('world', 'cog', rospy.Time(), rospy.Duration(0.5))
        except (tf2_ros.LookupException, tf2_ros.ConvertRegistration, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            rospy.logwarn("tf lookup exception catched: could not find tf from world to cog")

        search_pos = self.grid_pos(self.current_grid_idx)
        self.commander.change_height(self.search_height)
        self.commander.move_to(search_pos[0], search_pos[1])
        if abs(self.cog_pos.transform.translation.x - search_pos[0]) < self.reach_margin and abs(self.cog_pos.transform.translation.y - search_pos[1]) < self.reach_margin:
            if self.is_grid_full():
                return 'not_found'
            else:
                self.searched_grid[self.start_idx[0]+self.current_grid_idx[0],self.start_idx[1]+self.current_grid_idx[1]] = True
                self.move_to_next_idx()

        rospy.sleep(1/self.control_rate)
        return 'still_searching'


class AproachOnTargetState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'target_lost', 'ongoing'],
                                   input_keys = ['target_pos'] )
        self.commander = HydrusCommander()

        # retrieve parameters
        self.target_topic_name = rospy.get_param('~target_topic_name', '/target_object/pos')
        self.control_rate = rospy.get_param('~control_rate', 5.0)
        self.target_timeout = rospy.get_param('~target_timeout', 1.0)
        self.approach_height = rospy.get_param('~approach_height', 3.0)
        self.descending_height = rospy.get_param('~descending_height', 2.0)
        self.approach_margin = rospy.get_param('~approach_margin', 0.05)
        self.height_margin = rospy.get_param('~height_margin', 0.05)

        # tf Buffer
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # define subscriber
        self.target_pos_sub_ = rospy.Subscriber(self.target_topic_name, Vector3Stamped, self.targetPositionCallback)

        self.target_pos = None
        self.is_userdata_input_retrieved = False

    def targetPositionCallback(self, msg):
        """set target position if receiving topic"""
        self.target_pos = msg

    def execute(self, userdata):
        if self.is_userdata_input_retrieved is False:
            self.is_userdata_input_retrieved = True
            self.target_pos = userdata.target_pos

        if self.target_pos is None or rospy.Time.now() - self.target_pos.header.stamp > rospy.Duration(self.target_timeout) :
            return 'target_lost'

        try:
            self.cog_pos = self.tfBuffer.lookup_transform('world', 'cog', rospy.Time(), rospy.Duration(0.5))
        except (tf2_ros.LookupException, tf2_ros.ConvertRegistration, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            rospy.logwarn("tf lookup exception catched: could not find tf from world to cog")
        self.commander.change_height(self.approach_height)
        self.commander.move_to(self.target_pos.vector.x, self.target_pos.vector.y)
        if abs(self.target_pos.vector.x - self.cog_pos.transform.translation.x) < self.approach_margin and abs(self.target_pos.vector.y - self.cog_pos.transform.translation.y) < self.approach_margin:
            return 'success'

        rospy.sleep(1/self.control_rate)
        return 'ongoing'


class CoveringState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
        self.commander = HydrusCommander()

        # retrieve parameters
        self.covering_pre_height = rospy.get_param('~covering_pre_height', 1.0)
        self.covering_post_height = rospy.get_param('~covering_post_height', 0.2)
        self.covering_move_dist = rospy.get_param('~covering_move_dist', 1.0)

        # tf Buffer
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def execute(self, userdata):
        try:
            self.cog_pos = self.tfBuffer.lookup_transform('world', 'cog', rospy.Time(), rospy.Duration(0.5))
        except (tf2_ros.LookupException, tf2_ros.ConvertRegistration, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
            rospy.logwarn("tf lookup exception catched: could not find tf from world to cog")
        rot = self.cog_pos.transform.rotation
        q = (rot.x, rot.y, rot.z, rot.w)
        cog_euler = tf.transformations.euler_from_quaternion(q)
        self.commander.covering_motion(self.cog_pos.transform.translation.x, self.cog_pos.transform.translation.y, cog_euler[2], self.covering_pre_height, self.covering_post_height, self.covering_move_dist)
        return 'done'


if __name__ == "__main__":
    rospy.init_node('plane_fire_fighting', anonymous=True)

    sm_creator = PlaneFireFightStateMachineCreator()
    sm = sm_creator.create('rectangular_grid')

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()
