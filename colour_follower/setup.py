import os
from glob import glob
from setuptools import setup

package_name = 'colour_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        # (os.path.join('share', package_name), glob('config/*.[yaml]*'))
        ('share/' + package_name + '/config', [
            "config/twist_mux_topics.yaml",
            "config/twist_mux_locks.yaml",
            "config/assignment_map.yaml",
            "config/waffle.yaml",
            "config/assignment_map.pgm"
        ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kasm-user',
    maintainer_email='kasm-user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "wanderer = colour_follower.wanderer:main",
            "drive_forward = colour_follower.drive_forward:main",
            "recovery = colour_follower.recovery:main",
            "colour_chaser = colour_follower.colour_chaser:main",
            "colour_nav = colour_follower.colour_nav:main"
        ],
    },
)
