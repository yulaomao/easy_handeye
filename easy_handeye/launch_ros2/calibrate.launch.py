from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
import os


def generate_launch_description():
    # Declare launch arguments
    eye_on_hand_arg = DeclareLaunchArgument(
        'eye_on_hand', 
        description='if true, eye-on-hand instead of eye-on-base'
    )
    
    namespace_prefix_arg = DeclareLaunchArgument(
        'namespace_prefix',
        default_value='easy_handeye',
        description='the prefix of the namespace the node will run in, and of the folder in which the result will be saved'
    )
    
    freehand_robot_movement_arg = DeclareLaunchArgument(
        'freehand_robot_movement',
        default_value='false',
        description='if false, the rqt plugin for the automatic robot motion with MoveIt! will be started'
    )
    
    move_group_namespace_arg = DeclareLaunchArgument(
        'move_group_namespace',
        default_value='/',
        description='the namespace of move_group for the automatic robot motion with MoveIt!'
    )
    
    move_group_arg = DeclareLaunchArgument(
        'move_group',
        default_value='manipulator',
        description='the name of move_group for the automatic robot motion with MoveIt!'
    )
    
    translation_delta_meters_arg = DeclareLaunchArgument(
        'translation_delta_meters',
        default_value='0.1',
        description='the maximum movement that the robot should perform in the translation phase'
    )
    
    rotation_delta_degrees_arg = DeclareLaunchArgument(
        'rotation_delta_degrees',
        default_value='25',
        description='the maximum rotation that the robot should perform'
    )
    
    robot_velocity_scaling_arg = DeclareLaunchArgument(
        'robot_velocity_scaling',
        default_value='0.3',
        description='the maximum speed the robot should reach, as a factor of the speed declared in the joint_limits.yaml'
    )
    
    robot_acceleration_scaling_arg = DeclareLaunchArgument(
        'robot_acceleration_scaling',
        default_value='0.2',
        description='the maximum acceleration the robot should reach, as a factor of the acceleration declared in the joint_limits.yaml'
    )
    
    robot_base_frame_arg = DeclareLaunchArgument(
        'robot_base_frame',
        default_value='base_link'
    )
    
    robot_effector_frame_arg = DeclareLaunchArgument(
        'robot_effector_frame',
        default_value='tool0'
    )
    
    tracking_base_frame_arg = DeclareLaunchArgument(
        'tracking_base_frame',
        default_value='tracking_origin'
    )
    
    tracking_marker_frame_arg = DeclareLaunchArgument(
        'tracking_marker_frame',
        default_value='tracking_target'
    )
    
    publish_dummy_arg = DeclareLaunchArgument(
        'publish_dummy',
        default_value='true',
        description='if true, a dummy calibration will be published to keep all frames in a single tf tree, hence visualized in RViz'
    )
    
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='if true, rviz will be started with the provided config file'
    )
    
    rviz_config_file_arg = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=PathJoinSubstitution([FindPackageShare('easy_handeye'), 'config', 'rviz_easy_handeye.config']),
        description='the path to the rviz config file to be opened'
    )
    
    start_sampling_gui_arg = DeclareLaunchArgument(
        'start_sampling_gui',
        default_value='true',
        description='if true, rqt will be started with the provided perspective'
    )
    
    rqt_perspective_file_arg = DeclareLaunchArgument(
        'rqt_perspective_file',
        default_value=PathJoinSubstitution([FindPackageShare('easy_handeye'), 'config', 'rqt_easy_handeye.perspective']),
        description='the path to the rqt perspective file to be opened'
    )

    def launch_setup(context, *args, **kwargs):
        # Get parameter values
        eye_on_hand = LaunchConfiguration('eye_on_hand').perform(context).lower() == 'true'
        namespace_prefix = LaunchConfiguration('namespace_prefix').perform(context)
        freehand_robot_movement = LaunchConfiguration('freehand_robot_movement').perform(context).lower() == 'true'
        
        # Determine namespace based on eye_on_hand
        if eye_on_hand:
            namespace = f"{namespace_prefix}_eye_on_hand"
        else:
            namespace = f"{namespace_prefix}_eye_on_base"

        nodes_to_start = []

        # Dummy static transform publishers for visualization
        if LaunchConfiguration('publish_dummy').perform(context).lower() == 'true':
            if not eye_on_hand:
                dummy_handeye_node = Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    arguments=['1', '1', '1', '0', '1.5', '0', LaunchConfiguration('robot_base_frame'), LaunchConfiguration('tracking_base_frame')]
                )
            else:
                dummy_handeye_node = Node(
                    package='tf2_ros', 
                    executable='static_transform_publisher',
                    arguments=['0', '0', '0.05', '0', '0', '0', LaunchConfiguration('robot_effector_frame'), LaunchConfiguration('tracking_base_frame')]
                )
            nodes_to_start.append(dummy_handeye_node)

        # Robot backend node (outside namespace)
        if not freehand_robot_movement:
            robot_server_node = Node(
                package='easy_handeye',
                executable='robot',
                name='easy_handeye_calibration_server_robot',
                output='screen',
                parameters=[
                    {'calibration_namespace': namespace},
                    {'translation_delta_meters': LaunchConfiguration('translation_delta_meters')},
                    {'rotation_delta_degrees': LaunchConfiguration('rotation_delta_degrees')},
                    {'max_velocity_scaling': LaunchConfiguration('robot_velocity_scaling')},
                    {'max_acceleration_scaling': LaunchConfiguration('robot_acceleration_scaling')},
                ]
            )
            nodes_to_start.append(robot_server_node)

        # Main calibration server in namespace
        calibration_server_node = Node(
            package='easy_handeye',
            executable='calibrate',
            name='easy_handeye_calibration_server',
            namespace=namespace,
            output='screen',
            parameters=[
                {'eye_on_hand': eye_on_hand},
                {'move_group_namespace': LaunchConfiguration('move_group_namespace')},
                {'move_group': LaunchConfiguration('move_group')},
                {'robot_base_frame': LaunchConfiguration('robot_base_frame')},
                {'robot_effector_frame': LaunchConfiguration('robot_effector_frame')},
                {'tracking_base_frame': LaunchConfiguration('tracking_base_frame')},
                {'tracking_marker_frame': LaunchConfiguration('tracking_marker_frame')},
                {'freehand_robot_movement': freehand_robot_movement},
            ]
        )
        nodes_to_start.append(calibration_server_node)

        # RQT GUI for taking samples
        if LaunchConfiguration('start_sampling_gui').perform(context).lower() == 'true':
            sampling_gui_node = Node(
                package='rqt_easy_handeye',
                executable='rqt_easy_handeye',
                namespace=namespace,
                output='screen'
            )
            nodes_to_start.append(sampling_gui_node)

        # GUI for moving the robot around the starting pose
        if not freehand_robot_movement:
            movement_gui_node = Node(
                package='rqt_easy_handeye',
                executable='rqt_calibrationmovements',
                name='calibration_mover',
                namespace=namespace,
                parameters=[
                    {'move_group': LaunchConfiguration('move_group')},
                    {'translation_delta_meters': LaunchConfiguration('translation_delta_meters')},
                    {'rotation_delta_degrees': LaunchConfiguration('rotation_delta_degrees')},
                    {'max_velocity_scaling': LaunchConfiguration('robot_velocity_scaling')},
                    {'max_acceleration_scaling': LaunchConfiguration('robot_acceleration_scaling')},
                ]
            )
            nodes_to_start.append(movement_gui_node)

        # RViz
        if LaunchConfiguration('start_rviz').perform(context).lower() == 'true':
            move_group_namespace = LaunchConfiguration('move_group_namespace').perform(context)
            rviz_node = Node(
                package='rviz2',
                executable='rviz2',
                name='rviz',
                namespace=move_group_namespace if move_group_namespace != '/' else '',
                arguments=['-d', LaunchConfiguration('rviz_config_file')],
                output='screen'
            )
            nodes_to_start.append(rviz_node)

        return nodes_to_start

    return LaunchDescription([
        eye_on_hand_arg,
        namespace_prefix_arg,
        freehand_robot_movement_arg,
        move_group_namespace_arg,
        move_group_arg,
        translation_delta_meters_arg,
        rotation_delta_degrees_arg,
        robot_velocity_scaling_arg,
        robot_acceleration_scaling_arg,
        robot_base_frame_arg,
        robot_effector_frame_arg,
        tracking_base_frame_arg,
        tracking_marker_frame_arg,
        publish_dummy_arg,
        start_rviz_arg,
        rviz_config_file_arg,
        start_sampling_gui_arg,
        rqt_perspective_file_arg,
        OpaqueFunction(function=launch_setup)
    ])