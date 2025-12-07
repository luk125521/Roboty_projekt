from setuptools import setup

package_name = 'robot_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/interface_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='kapi125125@gmail.com',
    description='Robot interface with ArUco detection and TurtleBot control',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_publisher = robot_interface.nodes.camera_publisher:main',
            'aruco_detector = robot_interface.nodes.aruco_detector:main',
            'turtle_controller = robot_interface.nodes.turtle_controller:main',
            'image_viewer = robot_interface.nodes.image_viewer:main',
        ],
    },
)
