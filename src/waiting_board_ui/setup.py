from setuptools import setup

package_name = 'waiting_board_ui'

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
    maintainer='flynn',
    maintainer_email='flynn@todo.todo',
    description='Waiting board UI + ROS2 publisher',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run waiting_board_ui waiting_board_ui
            'waiting_board_ui = waiting_board_ui.waiting_board_ui:main',
        ],
    },
)
