#!/usr/bin/env python

import serial
from math import sin, cos

import rospy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import quaternion_from_euler

JOINT_NAMES = ['front_left_wheel_joint', 'front_right_wheel_joint', 'rear_left_wheel_joint', 'rear_right_wheel_joint']

ODOM_FRAME = 'odom'
ODOM_CHILD_FRAME = 'base_link'

LX = 0.3
LY = 0.3
WR = 0.076
SP = 0.00327249234749 
PP = 0.000245436926062
CP = 101.859163579

BUFFER_SIZE = 512

class MecanumController(object):

    def __init__(self):

        rospy.init_node('mecanum_controller') 

        self.front_controller_port = rospy.get_param('~front_controller_port', '/dev/ttyACM0')
        self.front_controller_baud = rospy.get_param('~front_controller_baud', 115200)
        self.front_controller = serial.Serial(self.front_controller_port, self.front_controller_baud, timeout=0.005)   
        self.front_controller.flushInput()
        self.front_controller.flushOutput()
        rospy.sleep(1.0)

        self.rear_controller_port = rospy.get_param('~rear_controller_port', '/dev/ttyACM1')
        self.rear_controller_baud = rospy.get_param('~rear_controller_baud', 115200)
        self.rear_controller = serial.Serial(self.rear_controller_port, self.rear_controller_baud, timeout=0.005)  
        self.rear_controller.flushInput()
        self.rear_controller.flushOutput() 
        rospy.sleep(1.0)

        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = JOINT_NAMES 

        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = ODOM_FRAME
        self.odom_msg.child_frame_id = ODOM_CHILD_FRAME
        self.w1_p = 0.0
        self.w2_p = 0.0
        self.w3_p = 0.0
        self.w4_p = 0.0
        self.w1_v = 0.0
        self.w2_v = 0.0
        self.w3_v = 0.0
        self.w4_v = 0.0
        self.last_w1_p = 0.0
        self.last_w2_p = 0.0
        self.last_w3_p = 0.0
        self.last_w4_p = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        rospy.Subscriber('/cmd_vel', Twist, self.vel_cb)
        self.cmd = None
        while not rospy.is_shutdown():
            self.control_loop()

    def vel_cb(self, msg):
        self.cmd = msg

    def control_loop(self):        
        ec1 = self.get_value(self.front_controller, 'C', 1)  
        if ec1 != -1:
            self.w1_p = ec1*PP
        ec2 = self.get_value(self.front_controller, 'C', 2)  
        if ec2 != -1:
            self.w2_p = ec2*PP
        ec3 = self.get_value(self.rear_controller, 'C', 1)  
        if ec3 != -1:
            self.w3_p = ec3*PP
        ec4 = self.get_value(self.rear_controller, 'C', 2)
        if ec4 != -1:
            self.w4_p = ec4*PP
        es1 = self.get_value(self.front_controller, 'S', 1)  
        if es1 != -1:
            self.w1_v = es1*SP
        es2 = self.get_value(self.front_controller, 'S', 2)  
        if es2 != -1:
            self.w2_v = es2*SP
        es3 = self.get_value(self.rear_controller, 'S', 1)  
        if es3 != -1:
            self.w3_v = es3*SP
        es4 = self.get_value(self.rear_controller, 'S', 2)  
        if es4 != -1:
            self.w4_v = es4*SP

        self.joint_state_msg.header.stamp = rospy.Time.now()
        self.joint_state_msg.position = [self.w1_p, self.w2_p, self.w3_p, self.w4_p]
        self.joint_state_msg.velocity = [self.w1_v, self.w2_v, self.w3_v, self.w4_v]
        self.joint_state_pub.publish(self.joint_state_msg)

        self.odom_msg.header.stamp = rospy.Time.now()
        delta_w1_p = (self.w1_p - self.last_w1_p)
        delta_w2_p = (self.w2_p - self.last_w2_p)
        delta_w3_p = (self.w3_p - self.last_w3_p)
        delta_w4_p = (self.w4_p - self.last_w4_p)
        self.last_w1_p = self.w1_p
        self.last_w2_p = self.w2_p
        self.last_w3_p = self.w3_p
        self.last_w4_p = self.w4_p
        delta_x = (-delta_w1_p+delta_w2_p-delta_w3_p+delta_w4_p)*(WR/4.0)
        delta_y = (+delta_w1_p+delta_w2_p-delta_w3_p-delta_w4_p)*(WR/4.0)
        self.theta += (+delta_w1_p+delta_w2_p+delta_w3_p+delta_w4_p)*(WR/4.0)/(LX+LY)
        self.x += delta_x*cos(self.theta)-delta_y*sin(self.theta)
        self.y += delta_x*sin(self.theta)+delta_y*cos(self.theta)
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        quaternion = quaternion_from_euler(0, 0, self.theta)
        self.odom_msg.pose.pose.orientation.x = quaternion[0]
        self.odom_msg.pose.pose.orientation.y = quaternion[1]
        self.odom_msg.pose.pose.orientation.z = quaternion[2]
        self.odom_msg.pose.pose.orientation.w = quaternion[3]
        self.odom_msg.twist.twist.linear.x = (-self.w1_v+self.w2_v-self.w3_v+self.w4_v)*(WR/4.0)
        self.odom_msg.twist.twist.linear.y = (+self.w1_v+self.w2_v-self.w3_v-self.w4_v)*(WR/4.0)
        self.odom_msg.twist.twist.angular.z = (+self.w1_v+self.w2_v+self.w3_v+self.w4_v)*(WR/4.0)/(LX+LY)
        self.odom_pub.publish(self.odom_msg)
        
        if self.cmd is None:
            self.set_command(self.front_controller, '!G 1 0\r') 
            self.set_command(self.front_controller, '!G 2 0\r') 
            self.set_command(self.rear_controller, '!G 1 0\r') 
            self.set_command(self.rear_controller, '!G 2 0\r') 
        else:
            v1 = (-self.cmd.linear.x+self.cmd.linear.y+(LX+LY)*self.cmd.angular.z)/WR
            v2 = (+self.cmd.linear.x+self.cmd.linear.y+(LX+LY)*self.cmd.angular.z)/WR
            v3 = (-self.cmd.linear.x-self.cmd.linear.y+(LX+LY)*self.cmd.angular.z)/WR
            v4 = (+self.cmd.linear.x-self.cmd.linear.y+(LX+LY)*self.cmd.angular.z)/WR  
            self.set_command(self.front_controller, '!G 1 '+str(int(v1*CP))+'\r') 
            self.set_command(self.front_controller, '!G 2 '+str(int(v2*CP))+'\r') 
            self.set_command(self.rear_controller, '!G 1 '+str(int(v3*CP))+'\r') 
            self.set_command(self.rear_controller, '!G 2 '+str(int(v4*CP))+'\r') 
            self.cmd = None     

    def get_value(self, controller, command, channel):
        controller.write('?'+command+' '+str(channel)+'\r')
        rcv_data = ''
        tmp_data = controller.read(BUFFER_SIZE)
        while len(tmp_data) > 0:
            rcv_data += tmp_data
            if len(tmp_data) < BUFFER_SIZE:
                break
            tmp_data = controller.read(BUFFER_SIZE)
        if len(tmp_data) == 0:
            return -1
        pos = rcv_data.rfind(command+'=')
        if pos == -1:
            return -1
        pos += (len(command)+1)
        carr = rcv_data.find('\r', pos) 
        if carr == -1:
            return -1
        return int(rcv_data[pos:carr])

    def set_command(self, controller, command): 
        controller.write(command)

if __name__ == '__main__':
    MecanumController()

