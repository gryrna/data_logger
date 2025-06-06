from setuptools import find_packages, setup

package_name = 'data_logger'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'gazebo_msgs', 'rclpy', 'tf_transformations'],
    zip_safe=True,
    maintainer='gryrna',
    maintainer_email='gumeshrana1@gmail.com',
    description='Logs camera and lidar data',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_logger_node = data_logger.data_logger_node:main',
            'ground_truth_listener = data_logger.ground_truth_listener:main',
        ],
    },
)
