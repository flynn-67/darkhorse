from setuptools import setup

package_name = 'form_start_trigger'

setup(
    name=package_name,              # ⭐ 여기!
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jinshim',
    maintainer_email='jinshim@example.com',
    description='Google Form webhook trigger',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'webhook_node = form_start_trigger.webhook_node:main',
            'start_limo_node = form_start_trigger.start_limo_node:main',
            'system_start_receiver_node = form_start_trigger.system_start_receiver_node:main',
        ],
    },
)
