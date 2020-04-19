#!/usr/bin/env python
import rospy
import tf
from aerial_robot_msgs.msg import FlightNav
from nav_msgs.msg import Odometry

class QuaternionConverter:
    def init(self):
        rospy.init_node('cog_quaternion_to_euler', anonymous=True)
        rospy.sleep(0.2)

        self.__hydrus_odom_sub = rospy.Subscriber("/uav/cog/odom", Odometry, self.__hydrusOdomCallback)
        rospy.sleep(0.2)

    def __hydrusOdomCallback(self, msg):
        self.__hydrus_odom = msg
        """Convert Quaternion to Euler Angles

        quarternion: geometry_msgs/Quaternion
        euler: geometry_msgs/Vector3
        """
        e = tf.transformations.euler_from_quaternion((self.__hydrus_odom.pose.pose.orientation.x,
                                                      self.__hydrus_odom.pose.pose.orientation.y,
                                                      self.__hydrus_odom.pose.pose.orientation.z,
                                                      self.__hydrus_odom.pose.pose.orientation.w))
        print (e)
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        ## test
        quaternion_to_euler = QuaternionConverter()
        quaternion_to_euler.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
