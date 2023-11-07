import os

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='rviz_performance_ros2',
            executable='rviz_performance_ros2',
            name=['rviz_performance_ros2'],
            output='screen',
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('rviz_performance_ros2'), 'rviz_performance_ros2.rviz')]
        ),

    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
