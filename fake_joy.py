#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
import time

rospy.init_node('fake_joy_node')
pub = rospy.Publisher('/joy', Joy, queue_size=10)

rate = rospy.Rate(10)

# Trigger axis values: 1.0 = unpressed, -1.0 = fully pressed

def make_joy(rt_val=1.0, lt_val=1.0):
    joy = Joy()
    joy.header.stamp = rospy.Time.now()
    joy.axes = [0.0] * 8
    joy.buttons = [0] * 11

    joy.axes[5] = rt_val  # RT
    joy.axes[2] = lt_val  # LT

    return joy

rospy.loginfo("Alternating fake RT and LT clicks")

while not rospy.is_shutdown():
    # Simulate RT pressed
    pub.publish(make_joy(rt_val=-1.0, lt_val=1.0))
    rospy.loginfo("RT pressed")
    time.sleep(2)

    # Simulate LT pressed
    pub.publish(make_joy(rt_val=1.0, lt_val=-1.0))
    rospy.loginfo("LT pressed")
    time.sleep(2)

    # Neutral
    pub.publish(make_joy(rt_val=1.0, lt_val=1.0))
    rospy.loginfo("No trigger pressed")
    time.sleep(1)
