#!/usr/bin/env python3

import rclpy
from easy_handeye.handeye_server import HandeyeServer


def main(args=None):
    rclpy.init(args=args)
    
    handeye_server = HandeyeServer()
    
    try:
        rclpy.spin(handeye_server)
    except KeyboardInterrupt:
        pass
    finally:
        handeye_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
