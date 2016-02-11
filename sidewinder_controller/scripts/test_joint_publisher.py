#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')

    rate = rospy.Rate(100)  # 100 hz
    joint_state_msg = JointState()
    joint_state_msg.header = Header()
    joint_state_msg.name = ['joint1', 'joint2', 'joint3']
    joint_state_msg.velocity = []
    joint_state_msg.effort = []

    position = [0, 0, 0]
    while not rospy.is_shutdown():
        joint_state_msg.header.stamp = rospy.Time.now()
        position[0] += 0.01
        position[1] += 0.01
        position[2] += 0.01
        joint_state_msg.position = position
        pub.publish(joint_state_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
