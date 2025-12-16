from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'smart_dispatcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Launch 파일
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # ✅ [추가 1] Resource 폴더 통째로 복사 (wav 파일 인식용)
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='flynn',
    maintainer_email='flynn@todo.todo',
    description='Smart Dispatcher',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dispatcher = smart_dispatcher.smart_dispatcher_node:main',
            # ✅ [추가 2] 사이렌 노드 실행 명령어 등록
            'siren_node = smart_dispatcher.siren_node:main',
            'ui_node = smart_dispatcher.ui_node:main',
        ],
    },
)