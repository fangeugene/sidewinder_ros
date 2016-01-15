#!/usr/bin/env python

import serial
import rospy
import math

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


ser = serial.Serial(port="/dev/ttyO4", baudrate=115200)


def callback(twist):
    x = twist.linear.x * 250
    y = twist.linear.y * 250
    rot_vel = twist.angular.z * 200
    
    t_mag_setp = math.sqrt(math.pow(x, 2) + math.pow(y, 2))
    t_head_setp = math.atan2(y, x)
    if x <= 0:
        t_head_setp += math.pi
    if t_head_setp > math.pi:
        t_head_setp -= 2 * math.pi
    t_head_setp = t_head_setp * 180 / math.pi

    msg = '1 %03d %03d %03d\n' % (t_mag_setp, t_head_setp + 500, rot_vel + 500)
    #rospy.logerr(msg)
    ser.write(msg)
    ser.write('9\n')  # watchdog
    ser.write('0 1\n')  # enable


def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('sidewinder_uart_communicator')
    rospy.Subscriber('cmd_vel', Twist, callback)

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
        #joint_states = [joint_states[1], joint_states[3], joint_states[5]]
        #rospy.logerr(joint_states)
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.position = [float(i) * math.pi / 1800.0 for i in joint_states]
        pub.publish(joint_state_msg)

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
