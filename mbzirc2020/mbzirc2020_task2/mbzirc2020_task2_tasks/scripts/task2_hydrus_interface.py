from mbzirc2020_common.hydrus_interface import HydrusInterface
import rospy
from std_msgs.msg import Int8
from std_msgs.msg import UInt8
from sensor_msgs.msg import JointState, Joy
from dynamic_reconfigure.msg import Config,BoolParameter,DoubleParameter
import tf2_ros

from dynamic_reconfigure.srv import Reconfigure
import std_srvs.srv

class Task2HydrusInterface(HydrusInterface):
    def __init__(self,robot_ns="hydrus"):
        HydrusInterface.__init__(self, robot_ns=robot_ns,debug_view=True)
        self.grasp_joint_angle = rospy.get_param('~grasp_joint_angle', [1.5, 1.5])
        self.ungrasp_joint_angle = rospy.get_param('~ungrasp_joint_angle', [0.9, 0.9])
        self.preshape_joint_angle = rospy.get_param('~preshape_joint_angle', [0.75, 0.75])
        self.reset_joint_angle = rospy.get_param('~reset_joint_angle', [1.5, 1.5])
        self.open_joint_angle = rospy.get_param('~open_joint_angle', [0.6, 0.6])
        self.flight_state = -1

        self.ctrl_mode_pub = rospy.Publisher(robot_ns + '/teleop_command/ctrl_mode', Int8, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.flight_state_sub = rospy.Subscriber(robot_ns + '/flight_state', UInt8, self.FlightStateCallback)
        self.joy_sub = rospy.Subscriber(robot_ns+'/joy', Joy, self.joyCallback)
        self.prev_joy_state = Joy()

        self.set_yaw_free_service = rospy.ServiceProxy(robot_ns + '/controller/lqi/set_parameters',Reconfigure)
        self.alt_sensor_service_client = rospy.ServiceProxy(robot_ns + '/sensor_plugin/alt1/estimate_flag', std_srvs.srv.SetBool)
        self.plane_detection_service_client = rospy.ServiceProxy(robot_ns + '/sensor_plugin/plane_detection1/estimate_flag', std_srvs.srv.SetBool)

    def enable_alt_sensor(self,flag):
        try:
            req = std_srvs.srv.SetBoolRequest()
            req.data = flag
            res = self.alt_sensor_service_client(req)
            if res is not None:
                if flag:
                    rospy.logwarn("%s: Enable alt sensor", self.robot_ns)
                else:
                    rospy.logwarn("%s: Disable alt sensor", self.robot_ns)
            else:
                if flag:
                    rospy.logerr("%s: Failed to enable alt sensor", self.robot_ns)
                else:
                    rospy.logerr("%s: Failed to disable alt sensor", self.robot_ns)

        except rospy.ServiceException, e:
            rospy.logerr("%s: Service call failed: %s", self.robot_ns, e)

    def enable_plane_detection(self,flag):
        try:
            req = std_srvs.srv.SetBoolRequest()
            req.data = flag
            res = self.plane_detection_service_client(req)
            if res is not None:
                if flag:
                    rospy.logwarn("%s: Enable plane detection",self.robot_ns)
                else:
                    rospy.logwarn("%s: Disable plane detection",self.robot_ns)
            else:
                if flag:
                    rospy.logerr("%s: Failed to enable place detection",self.robot_ns)
                else:
                    rospy.logerr("%s: Failed to disable place detection",self.robot_ns)
        except rospy.ServiceException, e:
            rospy.logerr("%s: Service call failed: %s", self.robot_ns, e)

    def change_ctrl_mode(self,mode):
        msg = Int8()
        if mode == 'pos':
            msg.data = 0
        elif mode == 'vel':
            msg.data = 1
        elif mode == 'acc':
            msg.data = 2

        self.ctrl_mode_pub.publish(msg)

    def wait_for_hovering(self):
        while True:
            if self.flight_state == 5:
                break
            rospy.sleep(0.5)

    def set_yaw_free(self,flag=True):
        set_yaw_free_srv = Config()
        bool_param = BoolParameter()
        bool_param.name = 'lqi_flag'
        bool_param.value = True
        set_yaw_free_srv.bools = [bool_param]
        if flag == True:
            doubles1 = DoubleParameter()
            doubles1.name = 'yaw_p'
            doubles1.value = 0.0
            doubles2 = DoubleParameter()
            doubles2.name = 'yaw_i'
            doubles2.value = 0.0
            doubles3 = DoubleParameter()
            doubles3.name = 'yaw_d'
            doubles3.value = 0.0
            doubles = [doubles1,doubles2,doubles3]
        elif flag == False:
            doubles1 = DoubleParameter()
            doubles1.name = 'yaw_p'
            doubles1.value = 100.0
            doubles2 = DoubleParameter()
            doubles2.name = 'yaw_i'
            doubles2.value = 0.05
            doubles3 = DoubleParameter()
            doubles3.name = 'yaw_d'
            doubles3.value = 50.0
            doubles = [doubles1,doubles2,doubles3]
        
        for d in doubles:
            set_yaw_free_srv.doubles = [d]
            self.set_yaw_free_service(set_yaw_free_srv)

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
        self.setExtraJointAngle(joint_state, time = time)

    def getTF(self, frame_id, wait=0.5, parent_frame_id='world', time = None):
        if time is None:
            time = rospy.Time.now()
        trans = self.tf_buffer.lookup_transform(parent_frame_id, frame_id, time, rospy.Duration(wait))
        return trans

    def joyCallback(self, msg):
        if (msg.buttons[4] == 1) and (self.prev_joy_state.buttons[4] == 0): #L1 servo on
            self.setJointTorque(True)
            rospy.loginfo('Servo on')

        if (msg.buttons[5] == 1) and (self.prev_joy_state.buttons[5] == 0): #R1 servo off
            self.setJointTorque(False)
            rospy.loginfo('Servo off')

        self.prev_joy_state = msg

    def FlightStateCallback(self,msg):
        self.flight_state = msg.data
