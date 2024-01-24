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

import yaml

import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import Imu, MagneticField

from adafruit_extended_bus import ExtendedI2C as I2C

import adafruit_lsm6ds.lsm6dsox
import adafruit_lis3mdl


class RosLsm6dsoxLis3mdlPublisher(Node):

    def __init__(self):
        super().__init__('ros_lsm6dsox_lis3mdl_publisher')

        self.file_path = '/tmp/bias.yaml'

        self.bias_x, self.bias_y, self.bias_z = self.load_bias_from_file()
        self.bias_x, self.bias_y, self.bias_z = tuple(self.declare_parameter('bias', [self.bias_x, self.bias_y, self.bias_z]).value)

        self.add_on_set_parameters_callback(self.on_set_parameters_callback)

        self.i2c = I2C(3)

        self.lsm6dsox = adafruit_lsm6ds.lsm6dsox.LSM6DSOX(self.i2c)
        self.lis3mdl = adafruit_lis3mdl.LIS3MDL(self.i2c)

        self.lsm6dsox_publisher = self.create_publisher(Imu, '/imu/data_raw', rclpy.qos.qos_profile_sensor_data)
        self.lis3mdl_publisher = self.create_publisher(MagneticField, '/imu/mag', rclpy.qos.qos_profile_sensor_data)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(f'Initalization finished with bias x: {self.bias_x}, y: {self.bias_y}, z: {self.bias_z} ')


    def load_bias_from_file(self):
        try:
            with open(self.file_path, 'r') as file:
                data = yaml.safe_load(file)
                if "bias" in data:
                    return data["bias"]
        except (FileNotFoundError, yaml.YAMLError, TypeError):
            pass
        return 0.0, 0.0, 0.0  # Default value if file doesn't exist or is invalid


    def write_bias_to_file(self):
        data = {
            'bias': [self.bias_x, self.bias_y, self.bias_z],
        }
        with open(self.file_path, 'w') as file:
            yaml.dump(data, file)


    def on_set_parameters_callback(self, parameter_list):
        result = SetParametersResult()

        for param in parameter_list:
            if param.name == 'bias':
                delta_bias_x, delta_bias_y, delta_bias_z = tuple(param.value)

                self.bias_x += delta_bias_x
                self.bias_y += delta_bias_y
                self.bias_z += delta_bias_z

                self.get_logger().info(f'Updated (total) bias: [{self.bias_x}, {self.bias_y}, {self.bias_z}]!')   
                result.successful = True             
        
        self.write_bias_to_file()

        return result


    def timer_callback(self):
        acc_x, acc_y, acc_z = self.lsm6dsox.acceleration
        rot_x, rot_y, rot_z = self.lsm6dsox.gyro
        mag_x, mag_y, mag_z = self.lis3mdl.magnetic

        lsm6dsox_msg = Imu()
        lsm6dsox_msg.header.stamp = self.get_clock().now().to_msg()
        lsm6dsox_msg.header.frame_id = "imu_link"
        lsm6dsox_msg.linear_acceleration.x = acc_x
        lsm6dsox_msg.linear_acceleration.y = acc_y 
        lsm6dsox_msg.linear_acceleration.z = acc_z
        lsm6dsox_msg.angular_velocity.x = rot_x
        lsm6dsox_msg.angular_velocity.y = rot_y
        lsm6dsox_msg.angular_velocity.z = rot_z

        lis3mdl_msg = MagneticField()
        lis3mdl_msg.header = lsm6dsox_msg.header
        lis3mdl_msg.magnetic_field.x = (mag_x - self.bias_x) #* 1e-6
        lis3mdl_msg.magnetic_field.y = (mag_y - self.bias_y) #* 1e-6
        lis3mdl_msg.magnetic_field.z = (mag_z - self.bias_z) #* 1e-6
        
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
