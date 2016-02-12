#!/usr/bin/env python

import serial
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

ser = serial.Serial(port="/dev/ttyO4", baudrate=115200)

def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('joint_state_publisher')

    rate = rospy.Rate(100)  # 100 hz
    joint_state_msg = JointState()
    joint_state_msg.header = Header()
    joint_state_msg.name = ['joint1', 'joint2', 'joint3']
    joint_state_msg.velocity = []
    joint_state_msg.effort = []

    while not rospy.is_shutdown():
        ser_line = None
        while ser_line is None or ser_line[0] != '2':
            ser_line = ser.readline()
        joint_states = ser_line.split(' ')[1:]
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.position = [float(i)/10.0 * 3.14159 / 180 for i in joint_states]
        pub.publish(joint_state_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
