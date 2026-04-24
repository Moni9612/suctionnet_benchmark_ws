from setuptools import setup

package_name = 'suction_grasp'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/suction_grasp_launch.py']),
    ],
    install_requires=['setuptools', 'launch', 'launch_ros'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Package for publishing grasp data using RealSense and GraspNet',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'suction_grasp = suction_grasp.suction_grasp:main',
        ],
    },
)
