from setuptools import setup

package_name = 'performance_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/performance_ros2_bridge.launch.py']),
        ('share/' + package_name, ['launch/performance_ros2_native.launch.py']),
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
            'performance_ros2 = performance_ros2.performance_ros2:main'
        ],
    },
)
