from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    eye_on_hand_arg = DeclareLaunchArgument(
        'eye_on_hand',
        description='eye-on-hand instead of eye-on-base'
    )
    
    namespace_prefix_arg = DeclareLaunchArgument(
        'namespace_prefix',
        default_value='easy_handeye'
    )
    
    robot_effector_frame_arg = DeclareLaunchArgument(
        'robot_effector_frame',
        default_value='',
        description='it is possible to override the link names saved in the yaml file in case of name clashes, for example'
    )
    
    robot_base_frame_arg = DeclareLaunchArgument(
        'robot_base_frame',
        default_value='',
        description='it is possible to override the link names saved in the yaml file in case of name clashes, for example'
    )
    
    tracking_base_frame_arg = DeclareLaunchArgument(
        'tracking_base_frame',
        default_value=''
    )
    
    inverse_arg = DeclareLaunchArgument(
        'inverse',
        default_value='false'
    )
    
    calibration_file_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value=''
    )

    def launch_setup(context, *args, **kwargs):
        # Get parameter values
        eye_on_hand = LaunchConfiguration('eye_on_hand').perform(context).lower() == 'true'
        namespace_prefix = LaunchConfiguration('namespace_prefix').perform(context)
        
        # Determine namespace based on eye_on_hand
        if eye_on_hand:
            namespace = f"{namespace_prefix}_eye_on_hand"
        else:
            namespace = f"{namespace_prefix}_eye_on_base"

        # Publish hand-eye calibration
        handeye_publisher_node = Node(
            package='easy_handeye',
            executable='publish',
            name='handeye_publisher',
            namespace=namespace,
            output='screen',
            parameters=[
                {'eye_on_hand': eye_on_hand},
                {'robot_base_frame': LaunchConfiguration('robot_base_frame')},
                {'robot_effector_frame': LaunchConfiguration('robot_effector_frame')},
                {'tracking_base_frame': LaunchConfiguration('tracking_base_frame')},
                {'inverse': LaunchConfiguration('inverse')},
                {'calibration_file': LaunchConfiguration('calibration_file')},
            ]
        )

        return [handeye_publisher_node]

    return LaunchDescription([
        eye_on_hand_arg,
        namespace_prefix_arg,
        robot_effector_frame_arg,
        robot_base_frame_arg,
        tracking_base_frame_arg,
        inverse_arg,
        calibration_file_arg,
        OpaqueFunction(function=launch_setup)
    ])