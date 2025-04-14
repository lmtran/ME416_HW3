""" Example of how to set attributes in ROS messages """
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

def twist_fill():
    '''
    Fill a twist message with non-zero values
    '''
    msg = Twist()
    msg.linear = Vector3()
    msg.linear.x = 1.0
    msg.linear.y = 0.5
    msg.linear.z = 0.5
    msg.angular = Vector3()
    msg.angular.x = 0.25
    msg.angular.y = 0.25
    msg.angular.z = 1.0
    return msg
