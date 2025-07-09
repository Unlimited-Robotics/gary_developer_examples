from setuptools import setup
from glob import glob

package_name = 'gary_examples_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, package_name + '.arms',
              package_name + '.grippers',
              package_name + '.communication'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='noam',
    maintainer_email='noam@unlimited-robotics.com',
    description='An example Python ROS 2 package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'right_arm = gary_examples_package.arms.right_arm:main',
            'left_arm = gary_examples_package.arms.left_arm:main',
            'both_arms = gary_examples_package.arms.both_arms:main',
            'right_gripper = gary_examples_package.grippers.right_gripper:main',
            'left_gripper = gary_examples_package.grippers.left_gripper:main',
            'right_communication = gary_examples_package.communication.right_arm:main',
            'left_communication = gary_examples_package.communication.left_arm:main',
        ],
    },
)
