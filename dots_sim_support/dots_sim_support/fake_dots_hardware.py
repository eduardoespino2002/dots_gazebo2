#!/usr/bin/env python3

'''
Perform some of the roles of dots_hardware for the simulation environment,
conditioning the simulator senses and actuators to more closely approximate 
the real robot.
'''

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data as qos_sensor
from rclpy.qos import qos_profile_system_default as qos_default
import math
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from std_msgs.msg import ColorRGBA
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import String
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import BatteryState

from geometry_msgs.msg import Twist
import sys
import numpy as np
import random

import struct


class Controller(Node):

    def __init__(self):
        super().__init__('dots_node')
        self.rate       = 100.0
        self.dt         = 1.0 / self.rate
        self.timer      = self.create_timer(self.dt, self.run_callback)
        self.loops      = 0
        self.led_state  = [0]*16

        self.lifter_sub     = self.create_subscription(Bool, 'lifter', self.lifter_callback, qos_default)
        self.reflect_sub    = self.create_subscription(Float64, 'reflect_in', self.reflect_callback, qos_default)
        self.power_en_sub   = self.create_subscription(Bool, 'power_en', self.power_en_callback, qos_default)
        self.motor_en_sub   = self.create_subscription(Bool, 'power_en', self.motor_en_callback, qos_default)
        self.led_sub        = self.create_subscription(Int32MultiArray, 'led', self.led_callback, qos_default)

        self.ground_truth_sub = self.create_subscription(Odometry, 'ground_truth', self.gt_callback, qos_default)


        self.lifter_pos_pub = self.create_publisher(Float32, 'lifter_pos', qos_default)
        self.reflect_pub    = self.create_publisher(Float64, 'reflect_out', qos_default)
        self.bstate_pub     = self.create_publisher(BatteryState, 'battery_state', qos_sensor)

        self.led_pub        = [self.create_publisher(ColorRGBA, 'led%d' % i, qos_default) for i in range(16)]
        self.compass_pub    = self.create_publisher(Float32, 'sensor/compass', qos_sensor)

        self.lifter = False
        self.lifter_min = 0.0
        self.lifter_max = 0.05
        self.lifter_vel = 0.2
        self.lifter_pos = self.lifter_min

    def run_callback(self):
        self.lifter_pos += (1 if self.lifter else -1) * self.lifter_vel * self.dt
        if self.lifter_pos < self.lifter_min: self.lifter_pos = self.lifter_min
        if self.lifter_pos > self.lifter_max: self.lifter_pos = self.lifter_max
        
        msg = Float32()
        msg.data = self.lifter_pos
        self.lifter_pos_pub.publish(msg)

        if (self.loops % 100) == 0:
            #print('spinning..')
            msg                 = BatteryState()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.voltage         = 4.8
            msg.current         = -2.0
            msg.capacity        = 20.0
            msg.design_capacity = 20.0
            msg.percentage      = 80.0
            msg.charge          = msg.capacity * msg.percentage / 100.0
            msg.cell_voltage    = [2.35, 2.45]
            msg.location        = 'robot'
            msg.serial_number   = '0'

            self.bstate_pub.publish(msg)

        self.loops += 1

    def gt_callback(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 *(q.y * q.y + q.z * q.z))

        m = Float32()
        # Add some Gaussian noise to the reading
        m.data = yaw + np.random.normal(scale=0.1)
        self.compass_pub.publish(m)


    def lifter_callback(self, msg):
        self.lifter = msg.data

    def reflect_callback(self, msg):
        self.reflect_pub(msg)

    def led_callback(self, msg):
        if len(msg.data) != 16:
            return
        
        # Only change leds if necessary
        for i in range(16):
            if msg.data[i] != self.led_state[i]:
                val = msg.data[i]
                self.led_state[i] = val
                omsg = ColorRGBA()
                omsg.r = float(val&0xff) / 255.0
                omsg.g = float((val>>8)&0xff) / 255.0
                omsg.b = float((val>>16)&0xff) / 255.0
                omsg.a = 1.0
                self.led_pub[i].publish(omsg)

    def power_en_callback(self, msg):
        pass

    def motor_en_callback(self, msg):
        pass


def main():
    
    rclpy.init()
    controller = Controller()

    rclpy.spin(controller)


if __name__ == '__main__':
    main()
    
    


