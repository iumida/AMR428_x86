from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'sensorlaunch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS 2 package 索引
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # 安裝 package.xml
        ('share/' + package_name, ['package.xml']),

        # ✅ 安裝 launch 檔案到 install/share/sensorlaunch/launch/
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ace428',
    maintainer_email='zxcvbnm28196544@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)

