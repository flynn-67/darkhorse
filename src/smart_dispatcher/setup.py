from setuptools import setup
import os
from glob import glob  # <--- [1. 맨 위에 이 줄 추가하세요!]

package_name = 'smart_dispatcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ---------------------------------------------------------
        # [2. 아래 줄을 복사해서 여기에 끼워 넣으세요!]
        # launch 폴더 안에 있는 모든 .launch.py 파일을 설치하라는 뜻입니다.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # ---------------------------------------------------------
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='flynn',
    maintainer_email='flynn@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dispatcher = smart_dispatcher.dispatcher_node:main',
        ],
    },
)