# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField


import board

import adafruit_lsm6ds.lsm6dsox
import adafruit_lis3mdl


class RosLsm6dsoxLis3mdlPublisher(Node):

    def __init__(self):
        super().__init__('ros_lsm6dsox_lis3mdl_publisher')
        self.i2c = board.I2C() 

        self.lsm6dsox = adafruit_lsm6ds.lsm6dsox.LSM6DSOX(self.i2c)
        self.lis3mdl = adafruit_lis3mdl.LIS3MDL(self.i2c)

        self.lsm6dsox_publisher = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.lis3mdl_publisher = self.create_publisher(MagneticField, '/imu/mag', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f'Initalization finished.')



    def timer_callback(self):
        acc_x, acc_y, acc_z = self.lsm6dsox.acceleration
        rot_x, rot_y, rot_z = self.lsm6dsox.gyro
        mag_x, mag_y, mag_z = self.lis3mdl.magnetic

        lsm6dsox_msg = Imu()
        lsm6dsox_msg.header.stamp = self.get_clock().now().to_msg()
        lsm6dsox_msg.header.frame_id = "ros_lsm6dsox"
        lsm6dsox_msg.linear_acceleration.x = acc_x
        lsm6dsox_msg.linear_acceleration.y = acc_y 
        lsm6dsox_msg.linear_acceleration.z = acc_z
        lsm6dsox_msg.angular_velocity.x = rot_x
        lsm6dsox_msg.angular_velocity.y = rot_y
        lsm6dsox_msg.angular_velocity.z = rot_z

        lis3mdl_msg = MagneticField()
        lis3mdl_msg.header = lsm6dsox_msg.header
        lis3mdl_msg.magnetic_field.x = mag_x
        lis3mdl_msg.magnetic_field.y = mag_y
        lis3mdl_msg.magnetic_field.z = mag_z
        
        self.lsm6dsox_publisher.publish(lsm6dsox_msg)
        self.lis3mdl_publisher.publish(lis3mdl_msg)


def main(args=None):
    rclpy.init(args=args)

    ros_lsm6dsox_lis3mdl_publisher = RosLsm6dsoxLis3mdlPublisher()

    rclpy.spin(ros_lsm6dsox_lis3mdl_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ros_lsm6dsox_lis3mdl_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
