#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped

# store latest inputs
latest_rt   = 0.0
latest_lt   = 0.0
latest_turn = 0.0

def joy_callback(data):
    global latest_rt, latest_lt, latest_turn
    # Normalise RT and LT
    latest_rt   = (1.0 - data.axes[5]) / 2.0
    latest_lt   = (1.0 - data.axes[2]) / 2.0
    # Left stick horizontal: -1.0 (left) to +1.0 (right)
    latest_turn = data.axes[0]

def publish_cmd_vel():
    rate = rospy.Rate(20)  # Rate of 20 Hz
    while not rospy.is_shutdown():
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        # Forward and backward movement
        msg.twist.linear.x  = latest_rt - latest_lt
        # Turning movement
        msg.twist.angular.z = latest_turn
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('miro_joy_control')
    pub = rospy.Publisher('/miro/control/cmd_vel', TwistStamped, queue_size=10)
    rospy.Subscriber('/joy', Joy, joy_callback)
    publish_cmd_vel()
