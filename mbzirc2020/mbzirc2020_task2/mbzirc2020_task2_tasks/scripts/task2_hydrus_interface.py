from mbzirc2020_common.hydrus_interface import HydrusInterface
import rospy
from sensor_msgs.msg import JointState, Joy
import tf2_ros

class Task2HydrusInterface(HydrusInterface):
    def __init__(self):
        HydrusInterface.__init__(self, debug_view=True)
        self.grasp_joint_angle = rospy.get_param('~grasp_joint_angle', [1.5, 1.5])
        self.ungrasp_joint_angle = rospy.get_param('~ungrasp_joint_angle', [0.9, 0.9])
        self.preshape_joint_angle = rospy.get_param('~preshape_joint_angle', [0.75, 0.75])
        self.reset_joint_angle = rospy.get_param('~reset_joint_angle', [1.5, 1.5])
        self.open_joint_angle = rospy.get_param('~open_joint_angle', [0.6, 0.6])

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joyCallback)
        self.prev_joy_state = Joy()

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

    def resetPose(self, time = 3000):
        joint_state = JointState()
        joint_state.name = ['joint1', 'joint3']
        joint_state.position = self.reset_joint_angle
        self.setJointAngle(joint_state, time = time)

    def openJoint(self, time = 3000):
        joint_state = JointState()
        joint_state.name = ['joint1', 'joint3']
        joint_state.position = self.open_joint_angle
        self.setJointAngle(joint_state, time = time)

    def setCameraJointAngle(self, angle, time = 1000):
        joint_state = JointState()
        joint_state.name = ['rs_d435_servo_joint']
        joint_state.position = [angle] #look down
        self.setExtraJointAngle(joint_state, time = 1000)

    def getTF(self, frame_id, wait=0.5, parent_frame_id='world'):
        trans = self.tf_buffer.lookup_transform(parent_frame_id, frame_id, rospy.Time.now(), rospy.Duration(wait))
        return trans

    def joyCallback(self, msg):
        if (msg.buttons[4] == 1) and (self.prev_joy_state.buttons[4] == 0): #L1 servo on
            self.setJointTorque(True)
            rospy.loginfo('Servo on')

        if (msg.buttons[5] == 1) and (self.prev_joy_state.buttons[5] == 0): #R1 servo off
            self.setJointTorque(False)
            rospy.loginfo('Servo off')

        self.prev_joy_state = msg
