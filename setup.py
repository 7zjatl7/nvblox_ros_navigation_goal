from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'nvblox_ros_navigation_goal'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        ("share/" + package_name + "/maps", glob("maps/*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "RandomSetNavigationGoal = nvblox_ros_navigation_goal.random_set_goal:main",
            "SetNavigationGoal = nvblox_ros_navigation_goal.set_goal:main"
        ],
    },
)
