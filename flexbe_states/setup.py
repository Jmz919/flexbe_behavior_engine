from setuptools import setup
from setuptools import find_packages

package_name = 'flexbe_states'

setup(
    name=package_name,
    version='1.3.1',
    packages=find_packages(),
    data_files=[
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
            'calculation_state = flexbe_states.calculation_state',
            'check_condition_state = flexbe_states.check_condition_state',
            'decision_state = flexbe_states.decision_state',
            'flexible_calculation_state = flexbe_states.flexible_calculation_state',
            'flexible_check_calculation_state = flexbe_states.flexible_check_calculation_state',
            'input_state = flexbe_states.input_state',
            'log_key_state = flexbe_states.log_key_state',
            'log_state = flexbe_states.log_state',
            'operator_decision_state = flexbe_states.operator_decision_state',
            'publisher_bool_state = flexbe_states.publisher_bool_state',
            'publisher_empty_state = flexbe_states.publisher_empty_state',
            'publisher_string_state = flexbe_states.publisher_string_state',
            'subscriber_state = flexbe_states.subscriber_state',
            'wait_state = flexbe_states.wait_state',
        ],
    },
)
