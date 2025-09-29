import os
import yaml

from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped

# Try to import rclpy for ROS2 compatibility
try:
    import rclpy
    import rclpy.parameter
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


# TODO: make this a data class in python3
class HandeyeCalibrationParameters(object):
    def __init__(self, namespace, move_group_namespace='/', move_group='manipulator', eye_on_hand=None,
                 robot_base_frame=None, robot_effector_frame=None,
                 tracking_base_frame=None,
                 tracking_marker_frame=None,
                 freehand_robot_movement=None):
        """
        Creates a HandeyeCalibrationParameters object.

        :param namespace: the namespace of the calibration (will determine the filename)
        :type namespace: string
        :param move_group: the MoveIt group name (e.g. "manipulator")
        :type move_group: string
        :param eye_on_hand: if false, it is a eye-on-base calibration
        :type eye_on_hand: bool
        :param robot_base_frame: needed only for eye-on-base calibrations: robot base link tf name
        :type robot_base_frame: string
        :param robot_effector_frame: needed only for eye-on-hand calibrations: robot tool tf name
        :type robot_effector_frame: string
        :param tracking_base_frame: tracking system tf name
        :type tracking_base_frame: string
        """
        self.namespace = namespace
        self.move_group_namespace = move_group_namespace
        self.move_group = move_group
        self.eye_on_hand = eye_on_hand
        self.robot_base_frame = robot_base_frame
        self.robot_effector_frame = robot_effector_frame
        self.tracking_base_frame = tracking_base_frame
        self.tracking_marker_frame = tracking_marker_frame
        self.freehand_robot_movement = freehand_robot_movement

    @staticmethod
    def init_from_parameter_server(namespace, node=None):
        """
        Initialize parameters from parameter server.
        
        :param namespace: the namespace for the parameters
        :param node: ROS2 node (None for ROS1 compatibility)
        """
        if node is not None:  # ROS2
            node.get_logger().info(f"Loading parameters for calibration {namespace} from the parameters server")
            
            if not namespace.endswith('/'):
                namespace = namespace + '/'
            
            # Declare parameters if they don't exist
            param_names = ['move_group_namespace', 'move_group', 'eye_on_hand', 
                          'robot_effector_frame', 'robot_base_frame', 'tracking_base_frame',
                          'tracking_marker_frame', 'freehand_robot_movement']
            
            for param_name in param_names:
                full_param_name = namespace.rstrip('/') + '.' + param_name
                if not node.has_parameter(full_param_name):
                    # Declare with default values
                    if param_name == 'move_group_namespace':
                        node.declare_parameter(full_param_name, '/')
                    elif param_name == 'move_group':
                        node.declare_parameter(full_param_name, 'manipulator')
                    elif param_name == 'eye_on_hand':
                        node.declare_parameter(full_param_name, False)
                    elif param_name == 'freehand_robot_movement':
                        node.declare_parameter(full_param_name, False)
                    else:
                        node.declare_parameter(full_param_name, '')
            
            ret = HandeyeCalibrationParameters(
                namespace=namespace,
                move_group_namespace=node.get_parameter(namespace.rstrip('/') + '.move_group_namespace').get_parameter_value().string_value,
                move_group=node.get_parameter(namespace.rstrip('/') + '.move_group').get_parameter_value().string_value,
                eye_on_hand=node.get_parameter(namespace.rstrip('/') + '.eye_on_hand').get_parameter_value().bool_value,
                robot_effector_frame=node.get_parameter(namespace.rstrip('/') + '.robot_effector_frame').get_parameter_value().string_value,
                robot_base_frame=node.get_parameter(namespace.rstrip('/') + '.robot_base_frame').get_parameter_value().string_value,
                tracking_base_frame=node.get_parameter(namespace.rstrip('/') + '.tracking_base_frame').get_parameter_value().string_value,
                tracking_marker_frame=node.get_parameter(namespace.rstrip('/') + '.tracking_marker_frame').get_parameter_value().string_value,
                freehand_robot_movement=node.get_parameter(namespace.rstrip('/') + '.freehand_robot_movement').get_parameter_value().bool_value
            )
        else:  # ROS1
            import rospy
            rospy.loginfo("Loading parameters for calibration {} from the parameters server".format(namespace))

            if not namespace.endswith('/'):
                namespace = namespace + '/'

            ret = HandeyeCalibrationParameters(namespace=namespace,
                                               move_group_namespace=rospy.get_param(namespace + 'move_group_namespace'),
                                               move_group=rospy.get_param(namespace + 'move_group'),
                                               eye_on_hand=rospy.get_param(namespace + 'eye_on_hand'),
                                               robot_effector_frame=rospy.get_param(namespace + 'robot_effector_frame'),
                                               robot_base_frame=rospy.get_param(namespace + 'robot_base_frame'),
                                               tracking_base_frame=rospy.get_param(namespace + 'tracking_base_frame'),
                                               tracking_marker_frame=rospy.get_param(namespace + 'tracking_marker_frame'),
                                               freehand_robot_movement=rospy.get_param(namespace + 'freehand_robot_movement'))
        return ret

    @staticmethod
    def store_to_parameter_server(parameters, node=None):
        """
        Store parameters to parameter server.
        
        :param parameters: the parameters to store
        :param node: ROS2 node (None for ROS1 compatibility)
        """
        namespace = parameters.namespace
        
        if node is not None:  # ROS2
            node.get_logger().info(f"Storing parameters for calibration {namespace} into the parameters server")
            
            param_prefix = namespace.rstrip('/') + '.'
            
            # Set parameters
            node.set_parameters([
                rclpy.parameter.Parameter(param_prefix + 'move_group_namespace', rclpy.Parameter.Type.STRING, parameters.move_group_namespace),
                rclpy.parameter.Parameter(param_prefix + 'move_group', rclpy.Parameter.Type.STRING, parameters.move_group),
                rclpy.parameter.Parameter(param_prefix + 'eye_on_hand', rclpy.Parameter.Type.BOOL, parameters.eye_on_hand),
                rclpy.parameter.Parameter(param_prefix + 'robot_effector_frame', rclpy.Parameter.Type.STRING, parameters.robot_effector_frame),
                rclpy.parameter.Parameter(param_prefix + 'robot_base_frame', rclpy.Parameter.Type.STRING, parameters.robot_base_frame),
                rclpy.parameter.Parameter(param_prefix + 'tracking_base_frame', rclpy.Parameter.Type.STRING, parameters.tracking_base_frame),
                rclpy.parameter.Parameter(param_prefix + 'tracking_marker_frame', rclpy.Parameter.Type.STRING, parameters.tracking_marker_frame),
            ])
        else:  # ROS1
            import rospy
            rospy.loginfo("Storing parameters for calibration {} into the parameters server".format(namespace))

            rospy.set_param(namespace + 'move_group_namespace', parameters.move_group_namespace)
            rospy.set_param(namespace + 'move_group', parameters.move_group)
            rospy.set_param(namespace + 'eye_on_hand', parameters.eye_on_hand)
            rospy.set_param(namespace + 'robot_effector_frame', parameters.robot_effector_frame)
            rospy.set_param(namespace + 'robot_base_frame', parameters.robot_base_frame)
            rospy.set_param(namespace + 'tracking_base_frame', parameters.tracking_base_frame)
            rospy.set_param(namespace + 'tracking_marker_frame', parameters.tracking_marker_frame)

    @staticmethod
    def from_dict(in_dict):
        return HandeyeCalibrationParameters(**in_dict)

    @staticmethod
    def to_dict(parameters):
        return vars(parameters)


class HandeyeCalibration(object):
    """
    Stores parameters and transformation of a hand-eye calibration for publishing.
    """
    DIRECTORY = os.path.expanduser('~/.ros/easy_handeye')
    """Default directory for calibration yaml files."""

    # TODO: use the HandeyeCalibration message instead, this should be HandeyeCalibrationConversions
    def __init__(self,
                 calibration_parameters=None,
                 transformation=None):
        """
        Creates a HandeyeCalibration object.

        :param transformation: transformation between optical origin and base/tool robot frame as tf tuple
        :type transformation: ((float, float, float), (float, float, float, float))
        :return: a HandeyeCalibration object

        :rtype: easy_handeye.handeye_calibration.HandeyeCalibration
        """
        self.parameters = calibration_parameters

        if transformation is None:
            transformation = ((0, 0, 0), (0, 0, 0, 1))

        self.transformation = TransformStamped(transform=Transform(
            Vector3(*transformation[0]), Quaternion(*transformation[1])))
        """
        transformation between optical origin and base/tool robot frame

        :type: geometry_msgs.msg.TransformedStamped
        """

        # tf names
        if self.parameters.eye_on_hand:
            self.transformation.header.frame_id = calibration_parameters.robot_effector_frame
        else:
            self.transformation.header.frame_id = calibration_parameters.robot_base_frame
        self.transformation.child_frame_id = calibration_parameters.tracking_base_frame

    @staticmethod
    def to_dict(calibration):
        """
        Returns a dictionary representing this calibration.

        :return: a dictionary representing this calibration.

        :rtype: dict[string, string|dict[string,float]]
        """
        ret = {
            'parameters': HandeyeCalibrationParameters.to_dict(calibration.parameters),
            'transformation': {
                'x': calibration.transformation.transform.translation.x,
                'y': calibration.transformation.transform.translation.y,
                'z': calibration.transformation.transform.translation.z,
                'qx': calibration.transformation.transform.rotation.x,
                'qy': calibration.transformation.transform.rotation.y,
                'qz': calibration.transformation.transform.rotation.z,
                'qw': calibration.transformation.transform.rotation.w
            }
        }

        return ret

    @staticmethod
    def from_dict(in_dict):
        """
        Sets values parsed from a given dictionary.

        :param in_dict: input dictionary.
        :type in_dict: dict[string, string|dict[string,float]]

        :rtype: None
        """
        tr = in_dict['transformation']
        ret = HandeyeCalibration(calibration_parameters=HandeyeCalibrationParameters.from_dict(in_dict['parameters']),
                                 transformation=((tr['x'], tr['y'], tr['z']), (tr['qx'], tr['qy'], tr['qz'], tr['qw'])))
        return ret

    @staticmethod
    def to_yaml(calibration):
        """
        Returns a yaml string representing this calibration.

        :return: a yaml string

        :rtype: string
        """
        return yaml.dump(HandeyeCalibration.to_dict(calibration), default_flow_style=False)

    @staticmethod
    def from_yaml(in_yaml):
        """
        Parses a yaml string and sets the contained values in this calibration.

        :param in_yaml: a yaml string
        :rtype: None
        """
        return HandeyeCalibration.from_dict(yaml.safe_load(in_yaml))

    @staticmethod
    def init_from_parameter_server(namespace):
        import rospy
        rospy.loginfo("Loading calibration {} from the parameters server".format(namespace))

        params = HandeyeCalibrationParameters.init_from_parameter_server(namespace)

        ret = HandeyeCalibration(calibration_parameters=params,
                                 transformation=((
                                                     rospy.get_param(namespace + 'transformation/x'),
                                                     rospy.get_param(namespace + 'transformation/y'),
                                                     rospy.get_param(namespace + 'transformation/z'),
                                                 ), (

                                                     rospy.get_param(namespace + 'transformation/qx'),
                                                     rospy.get_param(namespace + 'transformation/qy'),
                                                     rospy.get_param(namespace + 'transformation/qz'),
                                                     rospy.get_param(namespace + 'transformation/qw'),
                                                 )))
        return ret

    @staticmethod
    def store_to_parameter_server(calibration, node=None):
        """
        Store calibration to parameter server.
        
        :param calibration: the calibration to store
        :param node: ROS2 node (None for ROS1 compatibility)
        """
        namespace = calibration.parameters.namespace
        t = calibration.transformation.transform
        
        if node is not None:  # ROS2
            node.get_logger().info(f"Storing calibration {namespace} into the parameters server")

            HandeyeCalibrationParameters.store_to_parameter_server(calibration.parameters, node)
            
            param_prefix = namespace.rstrip('/') + '.'
            
            # Set transformation parameters
            node.set_parameters([
                rclpy.parameter.Parameter(param_prefix + 'transformation.x', rclpy.Parameter.Type.DOUBLE, float(t.translation.x)),
                rclpy.parameter.Parameter(param_prefix + 'transformation.y', rclpy.Parameter.Type.DOUBLE, float(t.translation.y)),
                rclpy.parameter.Parameter(param_prefix + 'transformation.z', rclpy.Parameter.Type.DOUBLE, float(t.translation.z)),
                rclpy.parameter.Parameter(param_prefix + 'transformation.qx', rclpy.Parameter.Type.DOUBLE, float(t.rotation.x)),
                rclpy.parameter.Parameter(param_prefix + 'transformation.qy', rclpy.Parameter.Type.DOUBLE, float(t.rotation.y)),
                rclpy.parameter.Parameter(param_prefix + 'transformation.qz', rclpy.Parameter.Type.DOUBLE, float(t.rotation.z)),
                rclpy.parameter.Parameter(param_prefix + 'transformation.qw', rclpy.Parameter.Type.DOUBLE, float(t.rotation.w)),
            ])
        else:  # ROS1
            import rospy
            rospy.loginfo("Storing calibration {} into the parameters server".format(namespace))

            HandeyeCalibrationParameters.store_to_parameter_server(calibration.parameters)

            rospy.set_param(namespace + 'transformation/x', t.translation.x)
            rospy.set_param(namespace + 'transformation/y', t.translation.y)
            rospy.set_param(namespace + 'transformation/z', t.translation.z)

            rospy.set_param(namespace + 'transformation/qx', t.rotation.x)
            rospy.set_param(namespace + 'transformation/qy', t.rotation.y)
            rospy.set_param(namespace + 'transformation/qz', t.rotation.z)
            rospy.set_param(namespace + 'transformation/qw', t.rotation.w)

    def filename(self):
        return HandeyeCalibration.filename_for_namespace(self.parameters.namespace)

    @staticmethod
    def filename_for_namespace(namespace):
        return HandeyeCalibration.DIRECTORY + '/' + namespace.rstrip('/').split('/')[-1] + '.yaml'

    @staticmethod
    def to_file(calibration):
        """
        Saves this calibration in a yaml file in the default path.

        The default path consists of the default directory and the namespace the node is running in.

        :rtype: None
        """
        if not os.path.exists(HandeyeCalibration.DIRECTORY):
            os.makedirs(HandeyeCalibration.DIRECTORY)

        with open(calibration.filename(), 'w') as calib_file:
            calib_file.write(HandeyeCalibration.to_yaml(calibration))

    @staticmethod
    def from_file(namespace):
        """
        Parses a yaml file in the default path and sets the contained values in this calibration.

        The default path consists of the default directory and the namespace the node is running in.

        :rtype: None
        """

        with open(HandeyeCalibration.filename_for_namespace(namespace)) as calib_file:
            return HandeyeCalibration.from_yaml(calib_file.read())

    @staticmethod
    def from_filename(filename):
        """
        Parses a yaml file at the specified location.

        :rtype: None
        """

        with open(filename) as calib_file:
            return HandeyeCalibration.from_yaml(calib_file.read())
