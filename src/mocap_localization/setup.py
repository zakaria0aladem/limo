from setuptools import setup
import os
from glob import glob

package_name = 'mocap_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zakaria',
    maintainer_email='zakaria@example.com',
    description='OptiTrack (VRPN) map->odom localizer for Nav2 on the LIMO Pro.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mocap_map_odom = mocap_localization.mocap_map_odom:main',
        ],
    },
)
