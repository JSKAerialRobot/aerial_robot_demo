#!/usr/bin/env python

import rospy
import smach
import smach_ros
import copy

import tf
import tf2_ros
import numpy as np

from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry

from hydrus_commander import HydrusCommander

class PlaneFireFightStateMachineCreator():
    def create(self, search_method, params):
        ''' search_method: {'rectangular_grid', '4_corner'}
            params: dictionary'''
        if search_method=='rectangular_grid':
            sm = smach.StateMachine(outcomes={'success', 'failure'})
            with sm:
                smach.StateMachine.add('searching', RectangularGridSearchState(params),
                                        transitions={'found':'approach_on_top',
                                                     'not_found':'failure',
                                                     'still_searching':'searching'},
                                        remapping={'target_pos':'target_pos'})
                smach.StateMachine.add('approach_on_top', AproachOnTargetState(params),
                                       transitions={'success':'covering',
                                                    'target_lost':'searching',
                                                    'ongoing':'approach_on_top'},
                                        remapping={'target_pos':'target_pos'})
                smach.StateMachine.add('covering', CoveringState(params),
                                       transitions={'done':'success'})
        elif search_method in {"4_corner", "4_corner_relative", "vortex"}:
            sm = smach.StateMachine(outcomes={'success', 'failure'})
            with sm:
                if search_method == "4_corner":
                    smach.StateMachine.add('searching', FourCornerSearchState(params),
                                            transitions={'found':'approach_on_top',
                                                         'not_found':'failure',
                                                         'still_searching':'searching'},
                                            remapping={'target_pos':'target_pos'})
                elif search_method == '4_corner_relative':
                    smach.StateMachine.add('searching', FourCornerSearchState(params, mode='relative'),
                                            transitions={'found':'approach_on_top',
                                                         'not_found':'failure',
                                                         'still_searching':'searching'},
                                            remapping={'target_pos':'target_pos'})


                elif search_method == 'vortex':

                    smach.StateMachine.add('searching', VortexSearchState(params, mode='relative'),
                                            transitions={'found':'approach_on_top',
                                                         'not_found':'failure',
                                                         'still_searching':'searching'},
                                            remapping={'target_pos':'target_pos'})


                smach.StateMachine.add('approach_on_top', AproachOnTargetState(params),
                                       transitions={'success':'covering',
                                                    'target_lost':'searching',
                                                    'ongoing':'approach_on_top'},
                                        remapping={'target_pos':'target_pos'})
                smach.StateMachine.add('covering', CoveringState(params),
                                       transitions={'done':'success'})

            return sm

class VortexSearchState(smach.State):
    def __init__(self, params, mode='absolute'):
        smach.State.__init__(self, outcomes=['found', 'not_found', 'still_searching'],
                                   output_keys=['target_pos'])

        self.relative_xy_bias = [0,0]
        self.mode = mode
        if mode == 'absolute':
            self.commander = HydrusCommander(nav_mode=params['nav_mode'])
        elif mode == 'relative':
            self.commander = HydrusCommander(nav_mode=2)
            self.is_first_execution = True

        rospy.logerr("vortex search start !!!!")
        # retrieve parameters
        self.target_topic_name = params['target_topic_name']
        self.control_rate = params['control_rate']
        self.search_area_length = params['search_area_length']
        self.trip_number = params['trip_number']
        self.reach_margin = params['reach_margin']
        self.search_height = params['search_height']

        self.target_pos = None # set when target_pos topic recieved

        # define subscriber
        self.target_pos_sub_ = rospy.Subscriber(self.target_topic_name, Vector3Stamped, self.targetPositionCallback)
        self.target_pos_flag = False
        rospy.logerr(self.search_area_length)
        self.waypoints = self.createWayPointList()
        self.waypoint_idx = 0




    def createWayPointList(self):
        waypoints = []

        waypoint_init = []
        waypoint_init.append(self.search_area_length/2*(1/float(self.trip_number)))
        waypoint_init.append(0.0)
        waypoint_init[0] += self.relative_xy_bias[0]
        waypoint_init[1] += self.relative_xy_bias[1]
        waypoints.append(waypoint_init)


        for i in range(1, self.trip_number+1):

            waypoint_tmp1 = []
            waypoint_tmp1.append(self.search_area_length/2*(i/float(self.trip_number)))
            waypoint_tmp1.append(self.search_area_length/2*(i/float(self.trip_number)))
            waypoint_tmp1[0] += self.relative_xy_bias[0]
            waypoint_tmp1[1] += self.relative_xy_bias[1]
            waypoints.append(waypoint_tmp1)

            waypoint_tmp2 = []
            waypoint_tmp2.append(-self.search_area_length/2*(i/float(self.trip_number)))
            waypoint_tmp2.append(self.search_area_length/2*(i/float(self.trip_number)))
            waypoint_tmp2[0] += self.relative_xy_bias[0]
            waypoint_tmp2[1] += self.relative_xy_bias[1]
            waypoints.append(waypoint_tmp2)


            waypoint_tmp3 = []
            waypoint_tmp3.append(-self.search_area_length/2*(i/float(self.trip_number)))
            waypoint_tmp3.append(-self.search_area_length/2*(i/float(self.trip_number)))
            waypoint_tmp3[0] += self.relative_xy_bias[0]
            waypoint_tmp3[1] += self.relative_xy_bias[1]
            waypoints.append(waypoint_tmp3)


            waypoint_tmp4 = []
            waypoint_tmp4.append(self.search_area_length/2*((i+1)/float(self.trip_number)))
            waypoint_tmp4.append(-self.search_area_length/2*(i/float(self.trip_number)))
            waypoint_tmp4[0] += self.relative_xy_bias[0]
            waypoint_tmp4[1] += self.relative_xy_bias[1]
            waypoints.append(waypoint_tmp4)

        return waypoints

    def targetPositionCallback(self, msg):
        """set target position if receiving topic"""
        self.target_pos_flag = True
        self.target_pos = msg

    def execute(self, userdata):
        # retrieve cog position from tf
        if self.target_pos_flag:
            userdata.target_pos = self.target_pos
            self.target_pos_flag= False
            return 'found'

        if self.mode == 'relative' and self.is_first_execution==True:
            msg = rospy.wait_for_message('/uav/cog/odom', Odometry)
            self.relative_xy_bias = [msg.pose.pose.position.x, msg.pose.pose.position.y]
            print('relative bias: ', str(self.relative_xy_bias))
            self.is_first_execution = False


        rospy.logerr(self.waypoints)
        search_pos = copy.copy(self.waypoints[self.waypoint_idx])
        search_pos[0] = self.waypoints[self.waypoint_idx][0]
        search_pos[1] = self.waypoints[self.waypoint_idx][1]
        search_pos[0] += self.relative_xy_bias[0]
        search_pos[1] += self.relative_xy_bias[1]
        self.commander.move_to(search_pos[0], search_pos[1])
        self.commander.change_height(self.search_height)

        rospy.loginfo("Searching waypoint "+
                str(self.waypoint_idx+1)+"/"+str(len(self.waypoints))
                + ", navigating to "+str([search_pos[0],search_pos[1],self.search_height]))

        rospy.sleep(1/self.control_rate)

        target_pos_err = self.commander.target_pos_error()
        if abs(target_pos_err[0]) < self.reach_margin and abs(target_pos_err[1]) < self.reach_margin and abs(target_pos_err[2]) < self.reach_margin:
            self.waypoint_idx = self.waypoint_idx + 1
            if len(self.waypoints) < self.waypoint_idx+1:
                return 'not_found'
        return 'still_searching'


class FourCornerSearchState(smach.State):
    def __init__(self, params, mode='absolute'):
        smach.State.__init__(self, outcomes=['found', 'not_found', 'still_searching'],
                                   output_keys=['target_pos'])

        self.relative_xy_bias = [0,0]
        self.mode = mode
        if mode == 'absolute':
            self.commander = HydrusCommander(nav_mode=params['nav_mode'])
        elif mode == 'relative':
            self.commander = HydrusCommander(nav_mode=2)
            self.is_first_execution = True

        # retrieve parameters
        self.target_topic_name = params['target_topic_name']
        self.control_rate = params['control_rate']
        self.area_corners = params['area_corners'][0:4]
        self.trip_number = params['area_corners'][4]
        self.reach_margin = params['reach_margin']
        self.search_height = params['search_height']

        self.target_pos = None # set when target_pos topic recieved

        # define subscriber
        self.target_pos_sub_ = rospy.Subscriber(self.target_topic_name, Vector3Stamped, self.targetPositionCallback)
        self.target_pos_flag = False

        self.waypoints = self.createWayPointList()
        self.waypoint_idx = 0

    def createWayPointList(self):
        ''''''
        if len(self.area_corners) !=4:
            rospy.signal_shutdown("area_corners should have 4 elements")

        waypoints = []

        for i in range(self.trip_number):
            waypoint_tmp1 = []
            waypoint_tmp1.append((self.area_corners[1][0]-self.area_corners[0][0])/(self.trip_number-1)*i + self.area_corners[0][0])
            waypoint_tmp1.append((self.area_corners[1][1]-self.area_corners[0][1])/(self.trip_number-1)*i + self.area_corners[0][1])
            waypoint_tmp1[0] += self.relative_xy_bias[0]
            waypoint_tmp1[1] += self.relative_xy_bias[1]
            waypoints.append(waypoint_tmp1)

            waypoint_tmp2 = []
            waypoint_tmp2.append((self.area_corners[2][0]-self.area_corners[3][0])/(self.trip_number-1)*i + self.area_corners[3][0])
            waypoint_tmp2.append((self.area_corners[2][1]-self.area_corners[3][1])/(self.trip_number-1)*i + self.area_corners[3][1])
            waypoint_tmp2[0] += self.relative_xy_bias[0]
            waypoint_tmp2[1] += self.relative_xy_bias[1]
            waypoints.append(waypoint_tmp2)

        return waypoints

    def targetPositionCallback(self, msg):
        """set target position if receiving topic"""
        self.target_pos_flag = True
        self.target_pos = msg

    def execute(self, userdata):
        # retrieve cog position from tf
        if self.target_pos_flag:
            userdata.target_pos = self.target_pos
            self.target_pos_flag= False
            return 'found'

        if self.mode == 'relative' and self.is_first_execution==True:
            msg = rospy.wait_for_message('/uav/cog/odom', Odometry)
            self.relative_xy_bias = [msg.pose.pose.position.x, msg.pose.pose.position.y]
            print('relative bias: ', str(self.relative_xy_bias))
            self.is_first_execution = False

        search_pos = copy.copy(self.waypoints[self.waypoint_idx])
        search_pos[0] = self.waypoints[self.waypoint_idx][0]
        search_pos[1] = self.waypoints[self.waypoint_idx][1]
        search_pos[0] += self.relative_xy_bias[0]
        search_pos[1] += self.relative_xy_bias[1]
        self.commander.move_to(search_pos[0], search_pos[1])
        self.commander.change_height(self.search_height)

        rospy.loginfo("Searching waypoint "+
                str(self.waypoint_idx+1)+"/"+str(len(self.waypoints))
                + ", navigating to "+str([search_pos[0],search_pos[1],self.search_height]))

        rospy.sleep(1/self.control_rate)

        target_pos_err = self.commander.target_pos_error()
        if abs(target_pos_err[0]) < self.reach_margin and abs(target_pos_err[1]) < self.reach_margin and abs(target_pos_err[2]) < self.reach_margin:
            self.waypoint_idx = self.waypoint_idx + 1
            if len(self.waypoints) < self.waypoint_idx+1:
                return 'not_found'
        return 'still_searching'

class RectangularGridSearchState(smach.State):
    def __init__(self, params):
        smach.State.__init__(self, outcomes=['found', 'not_found', 'still_searching'],
                                   output_keys=['target_pos'])
        self.commander = HydrusCommander(nav_mode=params['nav_mode'])

        # retrieve parameters
        self.target_topic_name = params['target_topic_name']
        self.control_rate = params['control_rate']
        self.area_corners = params['area_corners']
        self.area_orientation = self.area_corners[2]
        self.search_grid_size = [params['search_grid_size'], params['search_grid_size']]
        self.reach_margin = params['reach_margin']
        self.search_height = params['search_height']

        self.target_pos = None # set when target_pos topic recieved

        # tf Buffer
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # define subscriber
        self.target_pos_sub_ = rospy.Subscriber(self.target_topic_name, Vector3Stamped, self.targetPositionCallback)
        self.target_pos_flag = False

        # rotated grid edge length
        x = self.area_corners[1][0]-self.area_corners[0][0]
        y = self.area_corners[1][1]-self.area_corners[0][1]
        self.search_grid_size[0] = self.search_grid_size[0] * np.sign(x)
        self.search_grid_size[1] = self.search_grid_size[1] * np.sign(y)
        theta = np.deg2rad(self.area_orientation)
        self.x_dash = x*np.cos(theta) + y*np.sin(theta)
        self.y_dash = -x*np.sin(theta)+ y*np.cos(theta)
        self.searched_grid = np.zeros((abs(int(self.x_dash/self.search_grid_size[0])), abs(int(self.y_dash/self.search_grid_size[1]))), dtype=bool)
        self.current_grid_idx = [0,0]
        self.x_move_dir = 1
        self.y_move_dir = 0

    def grid_pos(self, grid_idx):
        """return position from grid index"""
        # TODO check valid range
        start_pos = self.area_corners[0]
        a = self.search_grid_size[0]*grid_idx[0]
        b = self.search_grid_size[1]*grid_idx[1]
        theta = np.deg2rad(self.area_orientation)
        sin_t = np.sin(theta)
        cos_t = np.cos(theta)
        x = a*cos_t - b*sin_t
        y = a*sin_t + b*cos_t
        return (start_pos[0]+x, start_pos[1]+y)

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
        return

    def targetPositionCallback(self, msg):
        """set target position if receiving topic"""
        self.target_pos_flag = True
        self.target_pos = msg

    def execute(self, userdata):
        # retrieve cog position from tf
        if self.target_pos_flag:
            userdata.target_pos = self.target_pos
            self.target_pos_flag= False
            return 'found'
        # try:
        #     self.cog_pos = self.tfBuffer.lookup_transform('world', 'cog', rospy.Time(), rospy.Duration(0.5))
        # except (tf2_ros.LookupException, tf2_ros.ConvertRegistration, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
        #     rospy.logwarn("tf lookup exception catched: could not find tf from world to cog")

        search_pos = self.grid_pos(self.current_grid_idx)
        self.commander.change_height(self.search_height)
        self.commander.move_to(search_pos[0], search_pos[1])

        rospy.loginfo("Searching grid "+
                str(np.count_nonzero(self.searched_grid)+1)+"/"+str(self.searched_grid.size)
                + ", navigating to "+str([search_pos[0],search_pos[1],self.search_height]))

        #if abs(self.cog_pos.transform.translation.x - search_pos[0]) < self.reach_margin and abs(self.cog_pos.transform.translation.y - search_pos[1]) < self.reach_margin:
        target_pos_err = self.commander.target_pos_error()
        if abs(target_pos_err[0]) < self.reach_margin and abs(target_pos_err[1]) < self.reach_margin and abs(target_pos_err[2]) < self.reach_margin:
            if self.is_grid_full():
                return 'not_found'
            else:
                self.searched_grid[self.current_grid_idx[0],self.current_grid_idx[1]] = True
                self.move_to_next_idx()

        rospy.sleep(1/self.control_rate)
        return 'still_searching'


class AproachOnTargetState(smach.State):
    def __init__(self, params):
        smach.State.__init__(self, outcomes=['success', 'target_lost', 'ongoing'],
                                   input_keys = ['target_pos'] )
        self.commander = HydrusCommander(nav_mode=params['nav_mode'])

        # retrieve parameters
        self.target_topic_name = params['target_topic_name']
        self.control_rate = params['control_rate']
        self.target_timeout = params['target_timeout']
        self.approach_height = params['approach_height']
        self.descending_height = params['descending_height']
        self.approach_margin = params['approach_margin']
        self.height_margin = params['height_margin']

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
            self.is_userdata_input_retrieved = False
            return 'target_lost'

        # try:
        #     self.cog_pos = self.tfBuffer.lookup_transform('world', 'cog', rospy.Time(), rospy.Duration(0.5))
        # except (tf2_ros.LookupException, tf2_ros.ConvertRegistration, tf2_ros.ExtrapolationException, tf2_ros.ConnectivityException):
        #     rospy.logwarn("tf lookup exception catched: could not find tf from world to cog")
        self.commander.change_height(self.approach_height)
        self.commander.move_to(self.target_pos.vector.x, self.target_pos.vector.y, 2)
        rospy.loginfo("Navigating to target position "
                +str([self.target_pos.vector.x, self.target_pos.vector.y, 0]))
        # if abs(self.target_pos.vector.x - self.cog_pos.transform.translation.x) < self.approach_margin and abs(self.target_pos.vector.y - self.cog_pos.transform.translation.y) < self.approach_margin:
        target_pos_err = self.commander.target_pos_error()
        if abs(target_pos_err[0]) < self.approach_margin and abs(target_pos_err[1]) < self.approach_margin and abs(target_pos_err[2]) < self.approach_margin:
            return 'success'

        rospy.sleep(1/self.control_rate)
        return 'ongoing'


class CoveringState(smach.State):
    def __init__(self, params):
        smach.State.__init__(self, outcomes=['done'])
        self.commander = HydrusCommander(nav_mode=params['nav_mode'])

        # retrieve parameters
        self.covering_pre_height = params['covering_pre_height']
        self.covering_post_height = params['covering_post_height']
        self.covering_move_dist = params['covering_move_dist']

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
