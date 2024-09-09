from setuptools import find_packages, setup

package_name = 'rover_commands'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xplore',
    maintainer_email='xplore@todo.todo',
    description='Practice : Rover Commands',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = rover_commands.publisher:main',
            'subscriber = rover_commands.subscriber:main',
            'check_position_server = rover_commands.check_position_server:main',
        ],
    },
)
