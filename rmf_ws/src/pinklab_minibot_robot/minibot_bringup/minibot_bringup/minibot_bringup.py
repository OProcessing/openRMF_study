#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import SetBool

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion

import serial
import time
import numpy as np
import math


class motors_command(object):
    command = [0xfa, 0xfe, 0x01, 0, 0x1, 0x3, 0, 0xfa, 0xfd]
    
class send_cmd_to_controller(object):
    command = [0xfa, 0xfe, 0x2, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x9, 0x0, 0xfa, 0xfd]

class get_state(object):
    command = [0xfa, 0xfe, 0x3, 0x1, 0x4, 0xfa, 0xfd]

class OdomPose(object):
    x = 0.0
    y = 0.0
    theta = 0.0
    timestamp = 0
    pre_timestamp = 0

class OdomVel(object):
    x = 0.0
    y = 0.0
    w = 0.0

class Joint(object):
    joint_name = ['l_wheel_joint', 'r_wheel_joint']
    joint_pos = [0.0, 0.0]
    joint_vel = [0.0, 0.0]

class Minibotserial(Node):
    def __init__(self):
        super().__init__('serial')
        arduino_port = '/dev/ttyArduino'
        baud_rate = 1000000
        self.ser = serial.Serial(arduino_port, baud_rate)
        time.sleep(3)
        self.get_logger().debug('아두이노 연결!!')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.pub_joint_states = self.create_publisher( JointState, 'joint_states', 10)
        self.odom_reset = self.create_service(SetBool,'reset_odom', self.reset_odom_handle)

        self.odom_broadcaster = TransformBroadcaster(self)
        
        self.l_lamp = 0
        self.r_lamp = 0

        self.wheel_ang_left = 0
        self.wheel_ang_right = 0 
        
        self.odom_pose = OdomPose()
        self.odom_vel = OdomVel()
        self.joint = Joint()

        self.odom_pose.pre_timestamp = self.get_clock().now().to_msg()
        self.l_last_enc, self.r_last_enc = 0.0, 0.0

        self.hw_commands = [0, 0]
        self.hw_positions = [0, 0]
        
        self.recv_buf = None #np.zeros(20, dtype=np.uint8)
        self.last_distance = 0

        self.cnt = 0

        self.create_timer(0.01, self.update_robot)
        self.enable_motors()
        time.sleep(1)
        self.get_logger().info('minibot ready!!')

    def update_robot(self):
        self.get()
        self.update_odometry(self.l_pos_enc, self.r_pos_enc, self.l_last_enc, self.r_last_enc)
        self.updateJointStates()

        if self.cnt == 50: #cnt가 50이 될때 로봇에 cmd_vel명령이 없으면 로봇 멈추기
            self.stop_wheel()

        self.cnt += 1

    def read(self, size=1, timeout=None):
        self.ser.timeout = timeout
        readed = self.ser.read(size)
        return readed

    def cmd_callback(self, msg):
         self.get()
         self.hw_commands = [msg.linear.x / 5 + -msg.angular.z / 10, msg.linear.x / 5 + msg.angular.z / 10] #기본 cmd_vel은 속도가 0.5이다 0.1로 맞추기 
         self.wheel()
         self.cnt = 0

    def wheel(self):
        l_cmd = (self.hw_commands[0] * 44.0 / (2.0 * np.pi) * 56.0) * -1.0 # hw_commands의 값을 펄스 값으로
        r_cmd = (self.hw_commands[1] * 44.0 / (2.0 * np.pi) * 56.0)
        self.send_comand(int(l_cmd), int(r_cmd), self.l_lamp, self.r_lamp)

    def stop_wheel(self):
        l_cmd = 0
        r_cmd = 0
        self.send_comand(int(l_cmd), int(r_cmd), self.l_lamp, self.r_lamp)

    def enable_motors(self):
        command = motors_command.command
        command[3] = 1
        command[6] = np.uint8(sum(command[2:6]))
        self.ser.write(bytes(command))
        readed = self.read(size=20, timeout=1)
        self.get_logger().debug('모터 on!!')

    def send_comand(self, l_vel, r_vel, l_lamp, r_lamp):
        command = send_cmd_to_controller.command
        command[3] = self.enabled
        command[4] = (l_vel >> 8) & 0xFF
        command[5] = l_vel & 0xFF
        command[6] = (r_vel >> 8) & 0xFF
        command[7] = r_vel & 0xFF
        command[8] = l_lamp
        command[9] = r_lamp
        command[12] =  np.uint8(sum(command[2:12]))
        self.ser.write(bytes(command))
        self.get_logger().debug('send!!')

    def get(self):
        command = get_state.command
        self.ser.write(bytes(command))

        try:
            self.recv_buf = self.read(size=20, timeout=1)
            if self.recv_buf[2] != 0x93:
                self.get_logger().error( "Failed to enable motors... check the boards...")
                return
        except:
            self.get_logger().error("Exceptions: \033[91m%s\033[0m")
            return
        
        self.enabled = self.recv_buf[3]
        self.l_pos_enc = (self.recv_buf[4] << 24) + (self.recv_buf[5] << 16) + (self.recv_buf[6] << 8) + self.recv_buf[7]
        self.r_pos_enc = (self.recv_buf[8] << 24) + (self.recv_buf[9] << 16) + (self.recv_buf[10] << 8) + self.recv_buf[11]
        self.l_lamp_val = self.recv_buf[12]
        self.r_lamp_val = self.recv_buf[13]
        self.range_sensor_val = (self.recv_buf[14] << 8) + self.recv_buf[15]

        self.l_lamp = self.l_lamp_val
        self.r_lamp = self.r_lamp_val
        
    def update_odometry(self, l_pos_enc, r_pos_enc, l_last_enc, r_last_enc):
        if l_pos_enc - l_last_enc > 1000 or l_pos_enc - l_last_enc < -1000: #2의 보수 건너뛰기
            pass
        else:
            self.hw_positions[0] = (l_pos_enc - l_last_enc) / 44.0 / 56.0 * (2.0 * np.pi) * -1.0 # 펄스 차이를 바퀴 회전각도로(라디안)

        if r_pos_enc - r_last_enc > 1000 or r_pos_enc - r_last_enc < - 1000:
            pass
        else: 
            self.hw_positions[1] = (r_pos_enc - r_last_enc) / 44.0 / 56.0 * (2.0 * np.pi)
        self.l_last_enc = l_pos_enc
        self.r_last_enc = r_pos_enc


        delta_distance = (self.hw_positions[0] + self.hw_positions[1]) / 2 * 3.5 * 0.01 #3.5는 원의 반지름 0.01은 센티미터로
        delta_theta = (self.hw_positions[1] - self.hw_positions[0]) / 5.6  #바퀴 5.6회전 = 로봇 1바퀴회전 
        
        trans_vel = delta_distance
        orient_vel = delta_theta

        self.odom_pose.timestamp = self.get_clock().now().to_msg()
        self.odom_pose.theta += orient_vel

        d_x = trans_vel * math.cos(self.odom_pose.theta) 
        d_y = trans_vel * math.sin(self.odom_pose.theta)

        self.odom_pose.x += d_x 
        self.odom_pose.y += d_y
        odom_orientation_quat = quaternion_from_euler(0, 0, self.odom_pose.theta)
        
        self.odom_vel.x = trans_vel
        self.odom_vel.y = 0.
        self.odom_vel.w = orient_vel

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.transform.translation.x = self.odom_pose.x
        t.transform.translation.y = self.odom_pose.y
        t.transform.translation.z = 0.0
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.rotation.x = odom_orientation_quat[0]
        t.transform.rotation.y = odom_orientation_quat[1]
        t.transform.rotation.z = odom_orientation_quat[2]
        t.transform.rotation.w = odom_orientation_quat[3]
        self.odom_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.pose.pose.position.x = self.odom_pose.x
        odom.pose.pose.position.y = self.odom_pose.y
        odom.pose.pose.position.z = 0.
        odom.pose.pose.orientation.x = odom_orientation_quat[0]
        odom.pose.pose.orientation.y = odom_orientation_quat[1]
        odom.pose.pose.orientation.z = odom_orientation_quat[2]
        odom.pose.pose.orientation.w = odom_orientation_quat[3]
        odom.twist.twist.linear.x = self.odom_vel.x
        odom.twist.twist.linear.y = self.odom_vel.y
        odom.twist.twist.linear.z = 0.
        odom.twist.twist.linear.z = 0.
        odom.twist.twist.linear.z = self.odom_vel.w
        self.odom_pub.publish(odom)

    def updateJointStates(self):
        wheel_ang_vel_left = self.hw_positions[0] 
        wheel_ang_vel_right = self.hw_positions[1] 

        self.wheel_ang_left += self.hw_positions[0] 
        self.wheel_ang_right += self.hw_positions[1]

        self.joint.joint_pos = [self.wheel_ang_left, self.wheel_ang_right]
        self.joint.joint_vel = [wheel_ang_vel_left, wheel_ang_vel_right]

        joint_states = JointState()
        joint_states.header.frame_id = ""
        joint_states.header.stamp = self.get_clock().now().to_msg()
        joint_states.name = self.joint.joint_name
        joint_states.position = self.joint.joint_pos
        joint_states.velocity = self.joint.joint_vel
        joint_states.effort = []

        self.pub_joint_states.publish(joint_states)

    def reset_odom_handle(self, request, response):
        if request.data:
            self.odom_pose.x = 0
            self.odom_pose.y = 0
            self.odom_pose.theta = 0
        
        response.success = True
        response.message = "reset odom"

        return response

def main(args=None):
    rclpy.init(args=args)

    node = Minibotserial()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()