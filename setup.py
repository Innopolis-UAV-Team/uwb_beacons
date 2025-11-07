from setuptools import setup
from glob import glob

package_name = 'uwb_beacons'
import setuptools
import subprocess
import os

# Automatically install pip requirements when building via colcon
if os.environ.get("COLCON_BUILD") or os.environ.get("AMENT_PREFIX_PATH"):
    subprocess.check_call(['pip', 'install', '-r', 'requirements.txt'])

setup(
    name=package_name,
    version='0.1.0',
    packages=['uwb_beacons'],  # keep this name since that’s your folder
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'localization'],
    zip_safe=True,
    maintainer='Anastasiia Stepanova',
    maintainer_email='asiiapine@gmail.com',
    description='UWB localization node (ROS2 wrapper for UWB positioning)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'uwb_localizer_node = uwb_beacons.uwb_localizer_node:main',
        ],
    },
)
