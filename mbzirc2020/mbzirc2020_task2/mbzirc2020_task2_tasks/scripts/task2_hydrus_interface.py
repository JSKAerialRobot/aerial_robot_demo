from mbzirc2020_common.hydrus_interface import HydrusInterface
import rospy
from sensor_msgs.msg import JointState
import tf2_ros

class Task2HydrusInterface(HydrusInterface):
    def __init__(self):
        HydrusInterface.__init__(self, debug_view=True)
        self.grasp_joint_angle = rospy.get_param('~grasp_joint_angle')
        self.ungrasp_joint_angle = rospy.get_param('~ungrasp_joint_angle')
        self.preshape_joint_angle = rospy.get_param('~preshape_joint_angle')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def grasp(self, time = 3000):
        joint_state = JointState()
        joint_state.name = ['joint1', 'joint3']
        joint_state.position = self.grasp_joint_angle
        self.setJointAngle(joint_state, time = time)

    def ungrasp(self, time = 3000):
        joint_state = JointState()
        joint_state.name = ['joint1', 'joint3']
        joint_state.position = self.ungrasp_joint_angle
        self.setJointAngle(joint_state, time = time)

    def preshape(self, time = 3000):
        joint_state = JointState()
        joint_state.name = ['joint1', 'joint3']
        joint_state.position = self.preshape_joint_angle
        self.setJointAngle(joint_state, time = time)

    def setCameraJointAngle(self, angle, time = 1000):
        joint_state = JointState()
        joint_state.name = ['rs_d435_servo_joint']
        joint_state.position = [angle] #look down
        self.setExtraJointAngle(joint_state, time = 1000)

    def getCameraTransform(self, frame_id):
        cam_trans = self.tf_buffer.lookup_transform('world', frame_id, rospy.Time(), rospy.Duration(0.5))
        cam_trans.transform.translation.x -= self.xy_pos_offset_[0]
        cam_trans.transform.translation.y -= self.xy_pos_offset_[1]
        return cam_trans
