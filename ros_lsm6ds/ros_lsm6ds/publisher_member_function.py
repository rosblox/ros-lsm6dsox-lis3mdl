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
from sensor_msgs.msg import Imu


import board
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX

class RosLsm6dsPublisher(Node):

    def __init__(self):
        super().__init__('ros_lsm6ds_publisher')
        self.i2c = board.I2C()
        self.lsm6ds = LSM6DSOX(self.i2c)

        self.publisher_ = self.create_publisher(Imu, '/imu/data_raw', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        acc_x, acc_y, acc_z = self.lsm6ds.acceleration
        rot_x, rot_y, rot_z = self.lsm6ds.gyro

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ros_lsm6ds"
        msg.linear_acceleration.x = acc_x
        msg.linear_acceleration.y = acc_y
        msg.linear_acceleration.z = acc_z
        msg.angular_velocity.x = rot_x
        msg.angular_velocity.y = rot_y
        msg.angular_velocity.z = rot_z

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    ros_lsm6ds_publisher = RosLsm6dsPublisher()

    rclpy.spin(ros_lsm6ds_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ros_lsm6ds_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
