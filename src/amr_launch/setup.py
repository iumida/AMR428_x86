from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'amr_launch'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install the ament resource marker
        ('share/ament_index/resource_index/packages',
         [os.path.join('resource', package_name)]),

        # Install package.xml
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),

        # Install URDF files
        (os.path.join('share', package_name, 'urdf'),
         glob('urdf/*.urdf')),

        # Install config files (舊有 config，如果不需要可刪除以下這一段)
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),

        # Install params files
        (os.path.join('share', package_name, 'params'),
         glob('params/*.yaml')),

        # Install meshes
        (os.path.join('share', package_name, 'meshes'),
         glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ace428',
    maintainer_email='iumida0918@gmail.com',
    description='Launch files and configurations for AMR navigation system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)

