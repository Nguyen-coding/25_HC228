from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'hot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', glob('msg/*.msg')),  # ← 추가

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cv25',
    maintainer_email='kim5537217@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'step_detector = hot.step_detection.step_detector_node:main',
            'node1 = hot.step_detection.node1:main',
            'node2 = hot.step_detection.node2:main',
            'node3 = hot.step_detection.node3:main',
            'step = hot.step_detection.step:main',
            'line = hot.step_detection.line:main',
            'node5 = hot.step_detection.node5:main',
            'node6 = hot.step_detection.node6:main',
            'test = hot.step_detection.test:main',
            'server = hot.server:main',
            'shock = hot.shock:main',
            'lidar = hot.lidar:main',
            'data = hot.data:main',
            'lidar_data = hot.lidar_data:main',
            'lidar2 = hot.lidar2:main',
            'stm = hot.stm:main',
            'auto = hot.auto:main',
            'avoid = hot.avoid:main'





        ],
    },
)
