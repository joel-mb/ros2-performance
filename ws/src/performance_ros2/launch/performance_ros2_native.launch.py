import launch
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='performance_ros2',
            executable='performance_ros2',
            name=['performance_ros2'],
            output='screen',
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
