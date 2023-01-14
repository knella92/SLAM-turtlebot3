# Nuturtle Description

URDF files for NUturtle franklin
* 'ros2 launch nuturtle_description load_one.launch.py' to see the robot in rviz.
![] (images/rviz.png)
* The rqt_graph when all four robots are visualized (Nodes Only, Hide Debug) is:
![] (images/rqt_graph.svg)

# Launch File Details
* 'ros2 launch nuturtle_description load_one.launch.py -s'
  'Arguments (pass arguments as '<name>:=<value>'):

    'use_rviz':
        Controls whether rviz is launched. Valid choices are: ['true', 'false']
        (default: 'true')

    'use_jsp':
        Controls whether joint_state_publisher is used to publish default joint states. Valid choices are: ['true', 'false']
        (default: 'true')

    'color':
        Sets color. Valid choices are: ['red', 'green', 'blue', 'purple', '']
        (default: 'purple')'


* 'ros2 launch nuturtle_description load_all.launch.xml -s'
'Arguments (pass arguments as '<name>:=<value>'):

    'use_rviz':
        Controls whether rviz is launched. Valid choices are: ['true', 'false']
        (default: 'true')

    'use_jsp':
        Controls whether joint_state_publisher is used to publish default joint states. Valid choices are: ['true', 'false']
        (default: 'true')

    'color':
        Sets color. Valid choices are: ['red', 'green', 'blue', 'purple', '']
        (default: 'purple')'


Worked With nobody ;_(