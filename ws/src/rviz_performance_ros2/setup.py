from setuptools import setup

package_name = 'rviz_performance_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/rviz_performance_ros2_bridge.launch.py']),
        ('share/' + package_name, ['launch/rviz_performance_ros2_native.launch.py']),
        ('share/' + package_name, ['rviz/rviz_performance_ros2.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jmoriana',
    maintainer_email='joel.moriana@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rviz_performance_ros2 = rviz_performance_ros2.rviz_performance_ros2:main'
        ],
    },
)
