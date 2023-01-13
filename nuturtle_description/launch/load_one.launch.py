from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage

def generate_launch_description():
        
    return LaunchDescription([
        
        DeclareLaunchArgument(
            name='use_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Controls whether rviz is launched'
        )
        ,
    
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description':
                Command([ExecutableInPackage('xacro','xacro'), ' ',
                        PathJoinSubstitution(
                        [FindPackageShare('nuturtle_description'), 'urdf/turtlebot3_burger.urdf.xacro'])
                        ])}
                        ]
            ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
            ),

        Node(
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
            )
    ])