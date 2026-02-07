from setuptools import setup
from glob import glob
import os

package_name = 'robot_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'static'),
            glob('robot_arm/static/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='annie',
    description='Robot arm web server',
    license='TODO',
    entry_points={
        'console_scripts': [
            'ws_server.py = robot_arm.ws_server:main',
        ],
    },
)
