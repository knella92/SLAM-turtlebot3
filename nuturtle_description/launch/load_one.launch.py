import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage

def generate_launch_description():
        
    return LaunchDescription([
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
            ),
    
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description':
                Command([ExecutableInPackage('xacro','xacro'), ' ',
                        PathJoinSubstitution(
                        [FindPackageShare('nuturtle_description'), 'turtlebot3_burger.urdf.xacro'])
                        ])}
                        ]
            ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
            )
    ])