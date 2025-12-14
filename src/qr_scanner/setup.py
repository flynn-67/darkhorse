from setuptools import setup

# 패키지 이름은 폴더 이름과 똑같이 언더스코어(_)를 사용해야 합니다.
package_name = 'qr_scanner' 

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
    maintainer='wego',
    maintainer_email='wego@todo.todo',
    description='QR Scanner Node for Patient Registration',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 여기가 가장 중요합니다. 
            # '실행명령어 = 패키지명.파일명:함수명' 형식이어야 합니다.
            'qr_registration = qr_scanner.qr_registration:main',
        ],
    },
)