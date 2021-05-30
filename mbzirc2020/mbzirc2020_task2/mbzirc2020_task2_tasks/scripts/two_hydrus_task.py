#!/usr/bin/env python

import rospy
import smach
import smach_ros

from std_msgs.msg import Empty, String

class Task2State(smach.State):
    def __init__(self, state_name,  outcomes=[], input_keys=[], output_keys=[], io_keys=[]):
        smach.State.__init__(self, outcomes, input_keys, output_keys, io_keys)
        self.state_pub = rospy.Publisher('~smach_state', String, queue_size=100)
        self.state = state_name

        self.leader_state = ""
        self.follower_state = ""

        robot_ns_leader = rospy.get_param('~robot_ns_leader')
        robot_ns_follower = rospy.get_param('~robot_ns_follower')

        self.skip_pick = rospy.get_param('/skip_pick')

        self.leader_state_sub = rospy.Subscriber(robot_ns_leader + '/two_hydrus_task_leader/smach_state', String, self.LeaderStateCallback)
        self.follower_state_sub = rospy.Subscriber(robot_ns_follower + '/two_hydrus_task_follower/smach_state', String, self.FollowerStateCallback)

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)

    def LeaderStateCallback(self, msg):
        self.leader_state = msg.data

    def FollowerStateCallback(self, msg):
        self.follower_state = msg.data

class Start(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['skip_pick', 'entire_task'])

        self.task_start_sub = rospy.Subscriber('/task_start', Empty, self.TaskStartCallback)

        self.task_start = False

    def TaskStartCallback(self,msg):
        self.task_start = True

    def execute(self, userdata):

        while not self.task_start:
            rospy.sleep(0.1)

        self.publish_state()

        while not(self.leader_state == self.state and self.follower_state == self.state):
            rospy.sleep(0.1)

        if self.skip_pick:
            return 'skip_pick'

        return 'entire_task'

class LeaderTakeoff(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['succeeded'])

    def execute(self, userdata):
        self.publish_state()

        rospy.sleep(1)

        while not(self.leader_state == self.state):
            rospy.sleep(0.1)

        return 'succeeded'

class LeaderApproachPickArea(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['succeeded'])

    def execute(self, userdata):
        self.publish_state()

        rospy.sleep(1)

        while not(self.leader_state == self.state):
            rospy.sleep(0.1)

        return 'succeeded'

class LeaderAdjustPickPosition(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['succeeded'])

    def execute(self, userdata):
        self.publish_state()

        rospy.sleep(1)

        while not(self.leader_state == self.state):
            rospy.sleep(0.1)

        return 'succeeded'

class LeaderLanding(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['succeeded'])

    def execute(self, userdata):
        self.publish_state()

        rospy.sleep(1)

        while not(self.leader_state == self.state):
            rospy.sleep(0.1)

        return 'succeeded'

class FollowerTakeoff(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['succeeded'])

    def execute(self, userdata):
        self.publish_state()

        rospy.sleep(1)

        while not(self.follower_state == self.state):
            rospy.sleep(0.1)

        return 'succeeded'

class FollowerApproachPickArea(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['succeeded'])

    def execute(self, userdata):
        self.publish_state()

        rospy.sleep(1)

        while not(self.follower_state == self.state):
            rospy.sleep(0.1)

        return 'succeeded'

class FollowerAdjustPickPosition(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['succeeded'])

    def execute(self, userdata):
        self.publish_state()

        rospy.sleep(1)

        while not(self.follower_state == self.state):
            rospy.sleep(0.1)

        return 'succeeded'

class FollowerLanding(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['succeeded'])

    def execute(self, userdata):
        self.publish_state()

        rospy.sleep(1)

        while not(self.follower_state == self.state):
            rospy.sleep(0.1)

        return 'succeeded'

class Grasp(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['succeeded'])

    def execute(self, userdata):
        self.publish_state()

        rospy.sleep(1)

        while not(self.leader_state == self.state and self.follower_state == self.state):
            rospy.sleep(0.1)

        return 'succeeded'

class Takeoff(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['succeeded'])

    def execute(self, userdata):
        self.publish_state()

        rospy.sleep(1)

        while not(self.leader_state == self.state and self.follower_state == self.state):
            rospy.sleep(0.1)

        return 'succeeded'

class WaitForHovering(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['succeeded'])

    def execute(self, userdata):
        self.publish_state()

        rospy.sleep(1)

        while not (self.leader_state == self.state and self.follower_state == self.state):
            rospy.sleep(0.1)

        return 'succeeded'

class ChangeHeight(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['succeeded'])

    def execute(self, userdata):
        self.publish_state()

        rospy.sleep(1)

        while not (self.leader_state == self.state and self.follower_state == self.state):
            rospy.sleep(0.1)

        return 'succeeded'

class EnablePlaneDetection(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['succeeded'])

    def execute(self, userdata):
        self.publish_state()

        rospy.sleep(1)

        while not (self.leader_state == self.state and self.follower_state == self.state):
            rospy.sleep(0.1)

        return 'succeeded'

class FollowerNavigated(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['succeeded'])

    def execute(self, userdata):
        self.publish_state()

        rospy.sleep(1)

        while not (self.follower_state == self.state):
            rospy.sleep(0.1)

        return 'succeeded'

class SetYawFree(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['succeeded'])

    def execute(self, userdata):
        self.publish_state()

        rospy.sleep(1)

        while not (self.leader_state == self.state and self.follower_state == self.state):
            rospy.sleep(0.1)

        return 'succeeded'

class LeaderApproachPlacePosition(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['succeeded'])

    def execute(self, userdata):
        self.publish_state()

        rospy.sleep(1)

        while not(self.leader_state == self.state):
            rospy.sleep(0.1)

        return 'succeeded'

class LeaderAdjustPlacePosition(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['succeeded'])

    def execute(self, userdata):
        self.publish_state()

        rospy.sleep(1)

        while not(self.leader_state == self.state):
            rospy.sleep(0.1)

        return 'succeeded'

class LeaderNavigated(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['succeeded'])

    def execute(self, userdata):
        self.publish_state()

        rospy.sleep(1)

        while not(self.leader_state == self.state):
            rospy.sleep(0.1)

        return 'succeeded'

class FollowerApproachPlacePosition(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['succeeded'])

    def execute(self, userdata):
        self.publish_state()

        rospy.sleep(1)

        while not(self.follower_state == self.state):
            rospy.sleep(0.1)

        return 'succeeded'

class FollowerAdjustPlacePosition(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['succeeded'])

    def execute(self, userdata):
        self.publish_state()

        rospy.sleep(1)

        while not(self.follower_state == self.state):
            rospy.sleep(0.1)

        return 'succeeded'

class AdjustHeight(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['succeeded'])

    def execute(self, userdata):
        self.publish_state()

        rospy.sleep(1)

        while not(self.leader_state == self.state and self.follower_state == self.state):
            rospy.sleep(0.1)

        return 'succeeded'

class Ungrasp(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['succeeded'])

    def execute(self, userdata):
        self.publish_state()

        rospy.sleep(1)

        while not(self.leader_state == self.state and self.follower_state == self.state):
            rospy.sleep(0.1)

        return 'succeeded'

class SetYawNotFree(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['succeeded'])

    def execute(self, userdata):
        self.publish_state()

        rospy.sleep(1)

        while not (self.leader_state == self.state and self.follower_state == self.state):
            rospy.sleep(0.1)

        return 'succeeded'

class FollowerBacktoStartPosition(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['succeeded'])

    def execute(self, userdata):
        self.publish_state()

        rospy.sleep(1)

        while not(self.follower_state == self.state):
            rospy.sleep(0.1)

        return 'succeeded'

class LeaderBacktoStartPosition(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['succeeded'])

    def execute(self, userdata):
        self.publish_state()

        rospy.sleep(1)

        while not(self.leader_state == self.state):
            rospy.sleep(0.1)

        return 'succeeded'

class Landing(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['succeeded'])

    def execute(self, userdata):
        self.publish_state()

        rospy.sleep(1)

        while not(self.leader_state == self.state and self.follower_state == self.state):
            rospy.sleep(0.1)

        return 'succeeded'

class Finish(Task2State):
    def __init__(self):
        Task2State.__init__(self, state_name=self.__class__.__name__,
                                  outcomes=['succeeded'])

    def execute(self, userdata):
        self.publish_state()

        rospy.sleep(1)

        while not(self.leader_state == self.state and self.follower_state == self.state):
            rospy.sleep(0.1)

        return 'succeeded'

def main():

    sm_top = smach.StateMachine(outcomes=['succeeded'])

    with sm_top:
        smach.StateMachine.add('Start', Start(),
                               transitions={'entire_task':'LeaderTakeoff',
                                            'skip_pick':'Takeoff'})

        smach.StateMachine.add('LeaderTakeoff', LeaderTakeoff(),
                               transitions={'succeeded':'LeaderApproachPickArea'})

        smach.StateMachine.add('LeaderApproachPickArea', LeaderApproachPickArea(),
                               transitions={'succeeded':'LeaderAdjustPickPosition'})

        smach.StateMachine.add('LeaderAdjustPickPosition', LeaderAdjustPickPosition(),
                               transitions={'succeeded':'LeaderLanding'})

        smach.StateMachine.add('LeaderLanding', LeaderLanding(),
                               transitions={'succeeded':'FollowerTakeoff'})

        smach.StateMachine.add('FollowerTakeoff', FollowerTakeoff(),
                               transitions={'succeeded':'FollowerApproachPickArea'})

        smach.StateMachine.add('FollowerApproachPickArea', FollowerApproachPickArea(),
                               transitions={'succeeded':'FollowerAdjustPickPosition'})

        smach.StateMachine.add('FollowerAdjustPickPosition', FollowerAdjustPickPosition(),
                               transitions={'succeeded':'FollowerLanding'})

        smach.StateMachine.add('FollowerLanding', FollowerLanding(),
                               transitions={'succeeded':'Grasp'})

        smach.StateMachine.add('Grasp', Grasp(),
                               transitions={'succeeded':'Takeoff'})

        smach.StateMachine.add('Takeoff', Takeoff(),
                               transitions={'succeeded':'WaitForHovering'})

        smach.StateMachine.add('WaitForHovering', WaitForHovering(),
                               transitions={'succeeded':'ChangeHeight'})

        smach.StateMachine.add('ChangeHeight', ChangeHeight(),
                               transitions={'succeeded':'EnablePlaneDetection'})

        smach.StateMachine.add('EnablePlaneDetection', EnablePlaneDetection(),
                               transitions={'succeeded':'FollowerNavigated'})

        smach.StateMachine.add('FollowerNavigated', FollowerNavigated(),
                               transitions={'succeeded':'SetYawFree'})

        smach.StateMachine.add('SetYawFree', SetYawFree(),
                               transitions={'succeeded':'LeaderApproachPlacePosition'})

        smach.StateMachine.add('LeaderApproachPlacePosition', LeaderApproachPlacePosition(),
                               transitions={'succeeded':'LeaderAdjustPlacePosition'})

        smach.StateMachine.add('LeaderAdjustPlacePosition', LeaderAdjustPlacePosition(),
                               transitions={'succeeded':'LeaderNavigated'})

        smach.StateMachine.add('LeaderNavigated', LeaderNavigated(),
                               transitions={'succeeded':'FollowerApproachPlacePosition'})

        smach.StateMachine.add('FollowerApproachPlacePosition', FollowerApproachPlacePosition(),
                               transitions={'succeeded':'FollowerAdjustPlacePosition'})

        smach.StateMachine.add('FollowerAdjustPlacePosition', FollowerAdjustPlacePosition(),
                               transitions={'succeeded':'AdjustHeight'})

        smach.StateMachine.add('AdjustHeight', AdjustHeight(),
                               transitions={'succeeded':'Ungrasp'})

        smach.StateMachine.add('Ungrasp', Ungrasp(),
                               transitions={'succeeded':'SetYawNotFree'})

        smach.StateMachine.add('SetYawNotFree', SetYawNotFree(),
                               transitions={'succeeded':'FollowerBacktoStartPosition'})

        smach.StateMachine.add('FollowerBacktoStartPosition', FollowerBacktoStartPosition(),
                               transitions={'succeeded':'LeaderBacktoStartPosition'})

        smach.StateMachine.add('LeaderBacktoStartPosition', LeaderBacktoStartPosition(),
                               transitions={'succeeded':'Landing'})

        smach.StateMachine.add('Landing', Landing(),
                               transitions={'succeeded':'Finish'})

        smach.StateMachine.add('Finish', Finish(),
                               transitions={'succeeded':'succeeded'})

        sis = smach_ros.IntrospectionServer('task2_smach_server', sm_top, '/TASK_MANAGER')
        sis.start()

    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()

if __name__=="__main__":

    rospy.init_node('two_hydrus_task')

    main()


