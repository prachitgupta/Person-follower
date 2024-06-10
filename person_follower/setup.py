from setuptools import setup

package_name = 'person_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["point_follower = person_follower.red_point:main",
                            "range_finder = person_follower.rangefinder:main",
                            "depth = person_follower.onlydepth:main",
                            "point_follower_lidar = person_follower.lidar:main",
                            "point_follower_pid = person_follower.pid:main",
                            "speed_test = person_follower.max_speed:main",
                            "camera_feed = person_follower.video_feed:main",
                            "blazepose = person_follower.blazepose_pose:main",
                            "point_follower_skeleton = person_follower.blazepose_follower:main"
        ],
    },
)
