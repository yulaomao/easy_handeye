#!/usr/bin/env python3

import rclpy
from easy_handeye.handeye_server_robot import HandeyeServerRobot


def main(args=None):
    rclpy.init(args=args)
    
    # Get calibration_namespace parameter
    # In ROS2, we'll handle this in the HandeyeServerRobot class
    handeye_server_robot = HandeyeServerRobot()
    
    try:
        rclpy.spin(handeye_server_robot)
    except KeyboardInterrupt:
        pass
    finally:
        handeye_server_robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
