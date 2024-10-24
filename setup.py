from setuptools import find_packages, setup

package_name = 'joy_control_trajectory'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/joystick_mapping.yaml', 'config/joy_velocity.yaml']),
        ('share/' + package_name + '/launch', ['launch/launch_joystick_mapping.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='almog',
    maintainer_email='almog6790@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "joy_velocity_mapper=joy_control_trajectory.joy_velocity_mapper:main",
            "joy_to_multiarray = joy_control_trajectory.joy_to_multiarray:main"
        ],
    },
)
