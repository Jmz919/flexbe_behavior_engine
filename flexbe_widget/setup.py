import os
from glob import glob
from setuptools import setup

package_name = 'flexbe_widget'

setup(
    name=package_name,
    version='1.3.1',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phil',
    maintainer_email='philsplus@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'behavior_launcher = flexbe_widget.behavior_launcher',
            'behavior_action_server = flexbe_widget.behavior_action_server',
            'be_action_server = flexbe_widget.be_action_server:main',
            'be_compress = flexbe_widget.be_compress:main',
            'be_launcher = flexbe_widget.be_launcher:main',
            'breakpoint = flexbe_widget.breakpoint:main',
            'create_repo = flexbe_widget.create_repo',
            'evaluate_logs = flexbe_widget.evaluate_logs:main',
        ],
    },
)
