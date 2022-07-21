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
from geometry_msgs.msg import Vector3Stamped


import board
import adafruit_lis3dh


class BloxLis3dhPublisher(Node):

    def __init__(self):
        super().__init__('blox_lis3dh_publisher')
        self.i2c = board.I2C()
        self.lis3dh = adafruit_lis3dh.LIS3DH_I2C(self.i2c)
        self.publisher_ = self.create_publisher(Vector3Stamped, 'blox_lis3dh/data', 10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        x, y, z = self.lis3dh.acceleration
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "blox_lis3dh"
        msg.vector.x = x
        msg.vector.y = y
        msg.vector.z = z
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    blox_lis3dh_publisher = BloxLis3dhPublisher()

    rclpy.spin(blox_lis3dh_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    blox_lis3dh_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
