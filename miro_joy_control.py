#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped

# Store the latest trigger values
latest_rt = 0.0
latest_lt = 0.0

def joy_callback(data):
    global latest_rt, latest_lt
    latest_rt = (1.0 - data.axes[5]) / 2.0  # RT normalized
    latest_lt = (1.0 - data.axes[2]) / 2.0  # LT normalized

def publish_cmd_vel():
    rate = rospy.Rate(20)  # 20 Hz
    while not rospy.is_shutdown():
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.twist.linear.x = latest_rt - latest_lt
        pub.publish(msg)
        rate.sleep()

rospy.init_node('miro_joy_control')
pub = rospy.Publisher('/miro/control/cmd_vel', TwistStamped, queue_size=10)
rospy.Subscriber('/joy', Joy, joy_callback)

publish_cmd_vel()
