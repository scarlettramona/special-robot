#!/usr/bin/env python3
from setuptools import setup

package_name = 'color_sorting'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # so that ROS 2 tools can find your package.xml and HSV file
        ('share/' + package_name, [
            'package.xml',
            package_name + '/HSV_config.txt',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Color sorting node for Dofbot',
    license='MIT',
    entry_points={
        'console_scripts': [
            # this makes “ros2 run color_sorting color_sort” invoke your main()
            'color_sort = color_sorting.color_sort:main',
        ],
    },
)

