#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
import tf2_ros

class HydrusPositionReset:
    def __init__(self):
        rospy.init_node('hydrus_position_reset')
        self.respawn_pos_x = rospy.get_param('~respawn_pos_x')
        self.respawn_pos_y = rospy.get_param('~respawn_pos_y')
        self.tf_prefix = rospy.get_param('~tf_prefix')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.sleep(1)

        client = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        req = SetModelStateRequest()

        trans = self.tf_buffer.lookup_transform(self.tf_prefix + '/root', self.tf_prefix + '/fc', rospy.Time(), rospy.Duration(10))
        req.model_state.pose.position.x = self.respawn_pos_x - trans.transform.translation.x
        req.model_state.pose.position.y = self.respawn_pos_y - trans.transform.translation.y
        req.model_state.model_name = self.tf_prefix

        result = client(req)
        print(result)

        rospy.loginfo("position reset ok")
        rospy.signal_shutdown('hydrus position reset finished')


if __name__ == '__main__':
    node = HydrusPositionReset()
    rospy.spin()
