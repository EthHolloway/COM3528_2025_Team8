#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped

def joy_callback(data):
    # Create a stamped Twist message
    msg = TwistStamped()
    msg.header.stamp = rospy.Time.now()

    # RT is axes[5], LT is axes[2]
    rt_val = (1.0 - data.axes[5]) / 2.0  # Normalized: 0.0 to 1.0
    lt_val = (1.0 - data.axes[2]) / 2.0  # Normalized: 0.0 to 1.0

    # Forward is RT, backward is LT
    msg.twist.linear.x = rt_val - lt_val  # Range: -1.0 to +1.0

    pub.publish(msg)

rospy.init_node('miro_joy_control')
pub = rospy.Publisher('/miro/control/cmd_vel', TwistStamped, queue_size=10)
rospy.Subscriber('/joy', Joy, joy_callback)
rospy.spin()

