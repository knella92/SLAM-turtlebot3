from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(  # Creates a launch argument regarding use of rviz upon launch
            name='use_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Controls whether rviz is launched'
        ),

        DeclareLaunchArgument(  # Creates a launch argument regarding use of
                                # joint_state_publisher upon launch
            name='use_jsp',
            default_value='true',
            choices=['true', 'false'],
            description="""Controls whether joint_state_publisher is used to
                         publish default joint states"""
        ),

        DeclareLaunchArgument(  # Creates a launch argument for color
            name='color',
            default_value='purple',
            choices=['red', 'green', 'blue', 'purple', ''],
            description='Sets color'
        ),

        SetLaunchConfiguration(
            name='rviz_path',
            value=['config/basic_', LaunchConfiguration('color'), '.rviz']
        ),

        Node(
            namespace=LaunchConfiguration('color'),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description':
                    Command([ExecutableInPackage('xacro', 'xacro'), ' ',
                            PathJoinSubstitution(
                            [FindPackageShare('nuturtle_description'),
                                'urdf/turtlebot3_burger.urdf.xacro']),
                        # passes color argument into xacro file
                        ' ', 'color:=', LaunchConfiguration('color')]),
                    'frame_prefix': [LaunchConfiguration('color'),
                                     TextSubstitution(text='/')]}
                        ]
            ),

        Node(
            namespace=LaunchConfiguration('color'),
            condition=IfCondition(LaunchConfiguration('use_jsp')),
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
            ),

        Node(
            namespace=LaunchConfiguration('color'),
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([FindPackageShare('nuturtle_description'),
                                                   LaunchConfiguration('rviz_path')]),
                       '-f', [LaunchConfiguration('color'), '/base_footprint']],
            on_exit=Shutdown()  # launchfile action to terminate launchfile upon exiting rviz2 node
            )
    ])
