# from setuptools import setup
# import os
# from glob import glob  # <--- 1. 추가!

# package_name = 'my_bringup'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=[package_name],
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         # ---------------------------------------------------
#         # 2. 아래 줄을 복사해서 넣으세요!
#         (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),        # ---------------------------------------------------
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='wego',
#     maintainer_email='wego@todo.todo',
#     description='System Bringup Package',
#     license='TODO: License declaration',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [],
#     },
# )

from setuptools import setup
import os
from glob import glob

package_name = 'my_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 파일 설치
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wego',
    maintainer_email='wego@todo.todo',
    description='System Bringup Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 여긴 비워둬도 됨 (launch는 console_scripts 없어도 됨)
        ],
    },
)
