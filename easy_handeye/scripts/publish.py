#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
import geometry_msgs.msg
from easy_handeye.handeye_calibration import HandeyeCalibration


class HandeyeCalibrationPublisher(Node):
    def __init__(self):
        super().__init__('handeye_calibration_publisher')
        
        # Get parameters
        self.declare_parameter('inverse', False)
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('robot_effector_frame', '')
        self.declare_parameter('robot_base_frame', '')
        self.declare_parameter('tracking_base_frame', '')
        
        inverse = self.get_parameter('inverse').get_parameter_value().bool_value
        filename = self.get_parameter('calibration_file').get_parameter_value().string_value
        
        if filename == '':
            self.get_logger().debug('No path specified for the calibration file, loading from the standard location')
            filename = HandeyeCalibration.filename_for_namespace(self.get_namespace())
        
        self.get_logger().info(f"Loading the calibration from file: {filename}")
        calib = HandeyeCalibration.from_filename(filename)
        
        if calib.parameters.eye_on_hand:
            overriding_robot_effector_frame = self.get_parameter('robot_effector_frame').get_parameter_value().string_value
            if overriding_robot_effector_frame != "":
                calib.transformation.header.frame_id = overriding_robot_effector_frame
        else:
            overriding_robot_base_frame = self.get_parameter('robot_base_frame').get_parameter_value().string_value
            if overriding_robot_base_frame != "":
                calib.transformation.header.frame_id = overriding_robot_base_frame
                
        overriding_tracking_base_frame = self.get_parameter('tracking_base_frame').get_parameter_value().string_value
        if overriding_tracking_base_frame != "":
            calib.transformation.child_frame_id = overriding_tracking_base_frame
        
        self.get_logger().info(f'loading calibration parameters into namespace {self.get_namespace()}')
        HandeyeCalibration.store_to_parameter_server(calib, self)
        
        orig = calib.transformation.header.frame_id  # tool or base link
        dest = calib.transformation.child_frame_id  # tracking_base_frame
        
        broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        static_transformStamped = geometry_msgs.msg.TransformStamped()
        
        static_transformStamped.header.stamp = self.get_clock().now().to_msg()
        static_transformStamped.header.frame_id = orig
        static_transformStamped.child_frame_id = dest
        
        static_transformStamped.transform = calib.transformation.transform
        
        broadcaster.sendTransform(static_transformStamped)
        self.get_logger().info(f'Publishing static transform from {orig} to {dest}')


def main(args=None):
    rclpy.init(args=args)
    publisher = HandeyeCalibrationPublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
