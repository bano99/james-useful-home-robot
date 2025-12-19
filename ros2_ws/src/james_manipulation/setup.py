from setuptools import setup
import os
from glob import glob

package_name = 'james_manipulation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        (os.path.join('share', package_name, 'config/moveit'), glob('config/moveit/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='James Developer',
    maintainer_email='james@example.com',
    description='Manipulation package for James Robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'platform_serial_bridge = james_manipulation.platform_serial_bridge:main',
            'arm_cartesian_controller = james_manipulation.arm_cartesian_controller:main',
            'teensy_serial_bridge = james_manipulation.teensy_serial_bridge:main',
            'calibrate_arm = james_manipulation.calibrate_arm:main',
        ],
    },
)
